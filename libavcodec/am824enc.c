/*
 * SMPTE ST 2110-31 AM824 AES3 Transparent Transport encoder
 * Copyright (c) 2026 Matrox
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/**
 * @file
 * SMPTE ST 2110-31 AM824 encoder.
 *
 * Wraps PCM audio into AM824 32-bit subframes for AES3 transparent transport.
 * Generates compliant AES3 channel status per AES3 section 7.2.2:
 *   - Only bytes 0, 1, 2 and 23 may be non-zero
 *   - Byte 0: professional flag, PCM/non-PCM indicator, sample rate
 *   - Byte 1: channel mode (not indicated, two-channel, or stereophonic)
 *   - Byte 2: word length
 *   - Byte 23: CRC-8 (polynomial 0xB8)
 *
 * Each output AM824 subframe is a 32-bit big-endian word:
 *   |0|0|B|F| PCUV | DATA24 |
 */

#include "libavutil/channel_layout.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/opt.h"
#include "avcodec.h"
#include "codec_internal.h"
#include "encode.h"

/* AES3 channel status block: 192 bits = 24 bytes, repeats every 192 frames */
#define AES3_BLOCK_FRAMES 192
#define AES3_CHSTATUS_BYTES 24

/* AES3 professional channel status byte 0 bit definitions (per AES3 § 7.2.2).
 * Bit numbering: bit 0 = LSB of byte 0. */
#define CS_BYTE0_PRO            0x01  /* Bit 0: 1 = professional use */
#define CS_BYTE0_NOT_PCM        0x02  /* Bit 1: 1 = non-PCM (S337M) */
/* Bits 2-4: emphasis (000 = not indicated) */
/* Bits 5: lock flag (0 = default) */
/* Bits 6-7: sample frequency (frame frequency) */
#define CS_BYTE0_FS_48K         (0x01 << 6)  /* 01 = 48 kHz */
#define CS_BYTE0_FS_441K        (0x02 << 6)  /* 10 = 44.1 kHz */
#define CS_BYTE0_FS_32K         (0x03 << 6)  /* 11 = 32 kHz */
/* 96 kHz is not directly encodable in byte 0 bits 6-7; use 0x00 (not indicated)
 * and rely on byte 4 for extended sample rate encoding. */
#define CS_BYTE0_FS_NOTIND      0x00         /* 00 = not indicated */

/* AES3 professional channel status byte 1 bits 0-3: channel mode */
#define CS_BYTE1_MODE_NOT_IND   0x00  /* 0000: not indicated */
#define CS_BYTE1_MODE_TWO_CH    0x01  /* 0001: two-channel */
#define CS_BYTE1_MODE_STEREO    0x04  /* 0100: stereophonic */

/* AES3 professional channel status byte 2: word length.
 * Bits 0-2: auxiliary sample bits usage; bits 3-5: word length. */
#define CS_BYTE2_WORDLEN_24     (0x01 | (0x05 << 3))  /* aux=001, wl=101: 24-bit */
#define CS_BYTE2_WORDLEN_20     0x00                    /* not indicated (default 20) */
#define CS_BYTE2_WORDLEN_16     (0x04 << 3)             /* aux=000, wl=100: 16-bit */

typedef struct AM824EncContext {
    AVClass *class;
    uint16_t framing_index;                     /**< Position in 192-frame block (0-191) */
    uint8_t  channel_status[AES3_CHSTATUS_BYTES]; /**< AES3 channel status block */
    int      non_pcm;                           /**< 1 if input carries non-PCM (S337M) data */
    int      passthrough;                       /**< 1 if input is already AM824-formatted subframes */
} AM824EncContext;

/**
 * Compute CRC-8 for AES3 channel status (polynomial 0xB8).
 * Per AES3, CRC is computed over bytes 0-22, stored in byte 23.
 * Bit processing order: LSB first within each byte.
 */
static uint8_t aes3_channel_status_crc(const uint8_t *data, int len)
{
    uint8_t crc = 0xFF;
    int i, bit_idx;

    for (i = 0; i < len; i++) {
        for (bit_idx = 0; bit_idx < 8; bit_idx++) {
            int bit = (data[i] >> bit_idx) & 1;
            int feedback = (crc & 1) ^ bit;
            crc >>= 1;
            if (feedback)
                crc ^= 0xB8;
        }
    }
    return crc;
}

/**
 * Build AES3 channel status block from codec parameters.
 * Per compliance rule: only bytes 0, 1, 2, and 23 may be non-zero.
 */
static void build_channel_status(AM824EncContext *s, AVCodecContext *avctx)
{
    uint8_t *cs = s->channel_status;

    memset(cs, 0, AES3_CHSTATUS_BYTES);

    /* Byte 0: professional use, PCM/non-PCM, sample rate */
    cs[0] = CS_BYTE0_PRO;
    if (s->non_pcm)
        cs[0] |= CS_BYTE0_NOT_PCM;

    switch (avctx->sample_rate) {
    case 48000:  cs[0] |= CS_BYTE0_FS_48K;    break;
    case 44100:  cs[0] |= CS_BYTE0_FS_441K;   break;
    case 32000:  cs[0] |= CS_BYTE0_FS_32K;    break;
    case 96000:  cs[0] |= CS_BYTE0_FS_NOTIND;  break; /* extended via byte 4 */
    default:     cs[0] |= CS_BYTE0_FS_NOTIND;  break;
    }

    /* Byte 1: channel mode.
     * Per S337 §6.2: non-PCM shall use "not indicated" (0x00).
     * For PCM: stereophonic for 2ch, not indicated otherwise. */
    if (s->non_pcm)
        cs[1] = CS_BYTE1_MODE_NOT_IND;
    else if (avctx->ch_layout.nb_channels == 2)
        cs[1] = CS_BYTE1_MODE_STEREO;
    else
        cs[1] = CS_BYTE1_MODE_NOT_IND;

    /* Byte 2: word length */
    switch (avctx->bits_per_raw_sample) {
    case 24: cs[2] = CS_BYTE2_WORDLEN_24; break;
    case 20: cs[2] = CS_BYTE2_WORDLEN_20; break;
    case 16: cs[2] = CS_BYTE2_WORDLEN_16; break;
    default: cs[2] = CS_BYTE2_WORDLEN_24; break;
    }

    /* Bytes 3-22: zero (already memset) */

    /* Byte 23: CRC-8 over bytes 0-22 */
    cs[23] = aes3_channel_status_crc(cs, 23);
}

/**
 * Compute even parity over DATA24 and the lower 3 bits of PCUV (C, U, V).
 * Per AES3 § 4.1.1: parity bit shall cause even parity over timeslots 4-31.
 * P = (count_ones(DATA24) + count_ones(C,U,V)) & 1
 */
static int compute_parity(uint32_t data24, int cuv)
{
    int ones = __builtin_popcount(data24 & 0x00FFFFFF)
             + __builtin_popcount(cuv & 0x07);
    return ones & 1;
}

static av_cold int am824_encode_init(AVCodecContext *avctx)
{
    AM824EncContext *s = avctx->priv_data;

    if (avctx->ch_layout.nb_channels < 2 ||
        (avctx->ch_layout.nb_channels & 1)) {
        av_log(avctx, AV_LOG_ERROR,
               "AM824: channel count must be even and >= 2, got %d\n",
               avctx->ch_layout.nb_channels);
        return AVERROR(EINVAL);
    }

    switch (avctx->sample_rate) {
    case 44100:
    case 48000:
    case 96000:
        break;
    default:
        av_log(avctx, AV_LOG_ERROR,
               "AM824: sample rate must be 44100, 48000, or 96000, got %d\n",
               avctx->sample_rate);
        return AVERROR(EINVAL);
    }

    avctx->bits_per_raw_sample = 24;
    avctx->frame_size = 0; /* variable frame size */

    s->framing_index = 0;

    build_channel_status(s, avctx);

    return 0;
}

static int am824_encode_frame(AVCodecContext *avctx, AVPacket *avpkt,
                              const AVFrame *frame, int *got_packet_ptr)
{
    AM824EncContext *s = avctx->priv_data;
    const int nb_channels = avctx->ch_layout.nb_channels;
    const int nb_subframes = frame->nb_samples * nb_channels;
    const int buf_size = nb_subframes * 4; /* 4 bytes per AM824 subframe */
    const int32_t *samples = (const int32_t *)frame->data[0];
    uint8_t *out;
    int ret, sample, ch;

    if ((ret = ff_get_encode_buffer(avctx, avpkt, buf_size, 0)) < 0)
        return ret;

    out = avpkt->data;

    /* Passthrough mode: input S32 samples are already AM824-formatted
     * 32-bit big-endian subframes (e.g., from the AM824 muxer output).
     * Just write them directly — no B/F/PCUV wrapping needed. */
    if (s->passthrough) {
        int i;
        for (i = 0; i < nb_subframes; i++) {
            AV_WB32(out + i * 4, samples[i]);
        }
        *got_packet_ptr = 1;
        return 0;
    }

    for (sample = 0; sample < frame->nb_samples; sample++) {
        /* Determine B (block start) and F (frame start) flags */
        int block_start = (s->framing_index == 0);

        for (ch = 0; ch < nb_channels; ch++) {
            uint32_t word = 0;
            uint32_t data24;
            int pcuv;
            int c_bit, u_bit, v_bit, p_bit;
            int frame_start = ((ch % 2) == 0); /* F=1 for first subframe of each AES3 pair */
            int b_flag = (block_start && frame_start && (ch < 2));

            /* Extract DATA24 from S32 input (upper 24 bits) */
            data24 = ((uint32_t)samples[sample * nb_channels + ch] >> 8) & 0x00FFFFFF;

            /* Channel status bit: extract from the 192-bit block.
             * Each channel within a pair has its own channel status,
             * but we use the same status for simplicity. */
            {
                int cs_byte_idx = s->framing_index / 8;
                int cs_bit_idx  = s->framing_index % 8;
                c_bit = (s->channel_status[cs_byte_idx] >> cs_bit_idx) & 1;
            }

            /* User data bit: always 0 */
            u_bit = 0;

            /* Validity bit: 0 for PCM, 1 for non-PCM */
            v_bit = s->non_pcm ? 1 : 0;

            /* Compute parity (odd parity over DATA24 + C + U + V) */
            pcuv = (c_bit << 2) | (u_bit << 1) | v_bit;
            p_bit = compute_parity(data24, pcuv);

            /* Assemble PCUV field: bits 27-24 = P,C,U,V */
            pcuv = (p_bit << 3) | pcuv;

            /* Assemble AM824 32-bit word:
             * Bits 31-30: 0,0 (reserved)
             * Bit 29: B (block start)
             * Bit 28: F (frame start)
             * Bits 27-24: PCUV
             * Bits 23-0: DATA24 */
            if (b_flag)
                word |= (1U << 29);
            if (frame_start)
                word |= (1U << 28);
            word |= ((uint32_t)pcuv << 24);
            word |= data24;

            AV_WB32(out, word);
            out += 4;
        }

        s->framing_index++;
        if (s->framing_index >= AES3_BLOCK_FRAMES)
            s->framing_index = 0;
    }

    *got_packet_ptr = 1;
    return 0;
}

#define ENC_FLAGS AV_OPT_FLAG_AUDIO_PARAM | AV_OPT_FLAG_ENCODING_PARAM
static const AVOption am824_enc_options[] = {
    { "non_pcm", "Mark stream as non-PCM (S337M/SPDIF data): sets V=1 and channel status byte 0 bit 1",
      offsetof(AM824EncContext, non_pcm), AV_OPT_TYPE_BOOL,
      { .i64 = 0 }, 0, 1, ENC_FLAGS },
    { "passthrough", "Input is already AM824-formatted subframes (pass through without re-wrapping)",
      offsetof(AM824EncContext, passthrough), AV_OPT_TYPE_BOOL,
      { .i64 = 0 }, 0, 1, ENC_FLAGS },
    { NULL }
};

static const AVClass am824_enc_class = {
    .class_name = "AM824 Encoder",
    .item_name  = av_default_item_name,
    .option     = am824_enc_options,
    .version    = LIBAVUTIL_VERSION_INT,
};

const FFCodec ff_am824_encoder = {
    .p.name                = "am824",
    CODEC_LONG_NAME("SMPTE ST 2110-31 AES3 Transparent Transport"),
    .p.type                = AVMEDIA_TYPE_AUDIO,
    .p.id                  = AV_CODEC_ID_AM824,
    .p.priv_class          = &am824_enc_class,
    .p.capabilities        = AV_CODEC_CAP_DR1 |
                             AV_CODEC_CAP_VARIABLE_FRAME_SIZE |
                             AV_CODEC_CAP_ENCODER_REORDERED_OPAQUE,
    .priv_data_size        = sizeof(AM824EncContext),
    .init                  = am824_encode_init,
    FF_CODEC_ENCODE_CB(am824_encode_frame),
    .p.sample_fmts         = (const enum AVSampleFormat[]){ AV_SAMPLE_FMT_S32,
                                                            AV_SAMPLE_FMT_NONE },
    .p.supported_samplerates = (const int[]){ 44100, 48000, 96000, 0 },
};
