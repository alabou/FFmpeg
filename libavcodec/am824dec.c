/*
 * SMPTE ST 2110-31 AM824 AES3 Transparent Transport decoder
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
 * SMPTE ST 2110-31 AM824 decoder.
 *
 * Decodes AM824 RTP payload containing AES3 subframes into PCM audio.
 * Each AM824 subframe is a 32-bit big-endian word per Section 5.4:
 *   Bits 31-30: Reserved (0,0)
 *   Bit 29:     Block Start (B)
 *   Bit 28:     Frame Start (F)
 *   Bits 27-24: PCUV — Parity, Channel status, User data, Validity
 *   Bits 23-0:  DATA24 — 24-bit audio sample data
 *
 * The PCUV metadata for each subframe is stored as frame side data
 * (array of uint8_t, one byte per subframe, containing the 4-bit PCUV
 * value in the lower nibble and B/F flags in bits 5-4).
 */

#include "libavutil/channel_layout.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/opt.h"
#include "libavutil/log.h"
#include "avcodec.h"
#include "codec_internal.h"
#include "decode.h"

/* AM824 subframe bit field masks */
#define AM824_BLOCK_START_BIT   (1U << 29)
#define AM824_FRAME_START_BIT   (1U << 28)
#define AM824_PCUV_SHIFT        24
#define AM824_PCUV_MASK         0x0F
#define AM824_DATA24_MASK       0x00FFFFFF

/* PCUV bit positions within the 4-bit field */
#define AM824_P_BIT  (1 << 3)  /* Parity */
#define AM824_C_BIT  (1 << 2)  /* Channel status */
#define AM824_U_BIT  (1 << 1)  /* User data */
#define AM824_V_BIT  (1 << 0)  /* Validity: 0=PCM, 1=non-PCM */

/* Side data layout: one byte per subframe
 * Bits 0-3: PCUV (P,C,U,V)
 * Bit 4:    F (frame start)
 * Bit 5:    B (block start) */
#define AM824_SD_F_BIT  (1 << 4)
#define AM824_SD_B_BIT  (1 << 5)

typedef struct AM824DecContext {
    AVClass *class;
    int non_pcm_mode; /**< 0=copy (pass DATA24 through), 1=drop (silence non-PCM) */
} AM824DecContext;

static av_cold int am824_decode_init(AVCodecContext *avctx)
{
    avctx->sample_fmt = AV_SAMPLE_FMT_S32;
    avctx->bits_per_raw_sample = 24;

    if (avctx->ch_layout.nb_channels <= 0) {
        av_channel_layout_uninit(&avctx->ch_layout);
        avctx->ch_layout = (AVChannelLayout)AV_CHANNEL_LAYOUT_STEREO;
    }

    return 0;
}

static int am824_decode_frame(AVCodecContext *avctx, AVFrame *frame,
                              int *got_frame_ptr, AVPacket *avpkt)
{
    AM824DecContext *s = avctx->priv_data;
    const uint8_t *buf = avpkt->data;
    int buf_size = avpkt->size;
    int channels, nb_subframes, nb_samples;
    int ret, i;
    int32_t *out;
    uint8_t *pcuv_sd;
    AVBufferRef *pcuv_buf;

    if (buf_size < 4 || (buf_size % 4) != 0) {
        av_log(avctx, AV_LOG_ERROR,
               "AM824: invalid packet size %d (must be multiple of 4)\n", buf_size);
        return AVERROR_INVALIDDATA;
    }

    nb_subframes = buf_size / 4;

    channels = avctx->ch_layout.nb_channels;
    if (channels <= 0 || (channels & 1))
        channels = 2;

    if (nb_subframes % channels != 0) {
        /* Fallback: try stereo */
        channels = 2;
        if (nb_subframes % channels != 0) {
            av_log(avctx, AV_LOG_ERROR,
                   "AM824: %d subframes not divisible by %d channels\n",
                   nb_subframes, channels);
            return AVERROR_INVALIDDATA;
        }
        av_channel_layout_uninit(&avctx->ch_layout);
        avctx->ch_layout = (AVChannelLayout)AV_CHANNEL_LAYOUT_STEREO;
    }

    nb_samples = nb_subframes / channels;

    avctx->sample_fmt = AV_SAMPLE_FMT_S32;
    avctx->bits_per_raw_sample = 24;

    frame->nb_samples = nb_samples;
    if ((ret = ff_get_buffer(avctx, frame, 0)) < 0)
        return ret;

    out = (int32_t *)frame->data[0];

    /* Allocate side data for PCUV metadata: one byte per subframe */
    pcuv_buf = av_buffer_alloc(nb_subframes);
    if (!pcuv_buf)
        return AVERROR(ENOMEM);
    pcuv_sd = pcuv_buf->data;

    /* Decode AM824 subframes into packed (interleaved) S32 output.
     * AV_SAMPLE_FMT_S32 is packed — all channels in frame->data[0]:
     * [ch0, ch1, ch2, ch3, ch0, ch1, ch2, ch3, ...] */
    for (i = 0; i < nb_subframes; i++) {
        uint32_t word = AV_RB32(buf + i * 4);
        uint32_t data24 = word & AM824_DATA24_MASK;
        int pcuv = (word >> AM824_PCUV_SHIFT) & AM824_PCUV_MASK;
        int validity = pcuv & AM824_V_BIT;
        uint8_t sd_byte = pcuv & 0x0F;

        if (word & AM824_FRAME_START_BIT)
            sd_byte |= AM824_SD_F_BIT;
        if (word & AM824_BLOCK_START_BIT)
            sd_byte |= AM824_SD_B_BIT;

        pcuv_sd[i] = sd_byte;

        if (validity && (s->non_pcm_mode & 1)) {
            out[i] = 0;
        } else {
            /* Output DATA24 as S32 left-justified.
             * For PCM: this is the audio sample.
             * For non-PCM (V=1): this preserves S337M sync words
             * in DATA24 so downstream SPDIF demuxer can find them. */
            out[i] = ((int32_t)data24) << 8;
        }
    }

    /* Attach PCUV side data to the frame.
     * Use AV_FRAME_DATA_A53_CC as a placeholder type —
     * a proper AM824-specific side data type would be defined upstream. */
    ret = av_frame_new_side_data_from_buf(frame, AV_FRAME_DATA_A53_CC, pcuv_buf);
    if (!ret) {
        /* Side data takes ownership of the buffer on success (ret != NULL).
         * If it fails (ret == NULL), we must free the buffer. */
        av_buffer_unref(&pcuv_buf);
    }

    *got_frame_ptr = 1;
    return avpkt->size;
}

#define FLAGS AV_OPT_FLAG_AUDIO_PARAM | AV_OPT_FLAG_DECODING_PARAM
static const AVOption am824_dec_options[] = {
    { "non_pcm_mode", "Handling of non-PCM subframes (V=1)",
      offsetof(AM824DecContext, non_pcm_mode), AV_OPT_TYPE_INT,
      { .i64 = 0 }, 0, 1, FLAGS, "non_pcm_mode" },
    { "copy", "Pass non-PCM DATA24 through (preserves S337M sync words)",
      0, AV_OPT_TYPE_CONST, { .i64 = 0 }, 0, 1, FLAGS, "non_pcm_mode" },
    { "drop", "Replace non-PCM subframes with silence",
      0, AV_OPT_TYPE_CONST, { .i64 = 1 }, 0, 1, FLAGS, "non_pcm_mode" },
    { NULL }
};

static const AVClass am824_dec_class = {
    .class_name = "AM824 Decoder",
    .item_name  = av_default_item_name,
    .option     = am824_dec_options,
    .version    = LIBAVUTIL_VERSION_INT,
};

const FFCodec ff_am824_decoder = {
    .p.name         = "am824",
    CODEC_LONG_NAME("SMPTE ST 2110-31 AES3 Transparent Transport"),
    .p.type         = AVMEDIA_TYPE_AUDIO,
    .p.id           = AV_CODEC_ID_AM824,
    .p.priv_class   = &am824_dec_class,
    .priv_data_size = sizeof(AM824DecContext),
    .init           = am824_decode_init,
    FF_CODEC_DECODE_CB(am824_decode_frame),
    .p.capabilities = AV_CODEC_CAP_CHANNEL_CONF |
                      AV_CODEC_CAP_DR1,
    .p.sample_fmts  = (const enum AVSampleFormat[]){ AV_SAMPLE_FMT_S32,
                                                      AV_SAMPLE_FMT_NONE },
};
