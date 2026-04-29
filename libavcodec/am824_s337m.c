/*
 * S337M/IEC 61937 burst framing for AM824 non-PCM transport
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
 * S337M / IEC 61937 burst framing encoder for AM824 non-PCM transport.
 *
 * This encoder wraps compressed audio (AAC, AC-3, DTS, etc.) in
 * IEC 61937 burst framing and outputs the result as S32 PCM samples
 * suitable for feeding directly into the AM824 encoder.
 *
 * Each IEC 61937 burst consists of:
 *   Pa (sync word 1): 0xF872
 *   Pb (sync word 2): 0x4E1F
 *   Pc (burst info):  data_type + stream_number + error_flag
 *   Pd (length code):  payload length in bits
 *   Payload:           compressed audio data (16-bit BE words)
 *   Padding:           zeros to fill burst period
 *
 * Output format: S32 stereo (interleaved L,R pairs).
 * Each 16-bit SPDIF word maps to one S32 sample: word << 16.
 * The S337M preamble words Pa/Pb/Pc/Pd and payload words
 * are interleaved as stereo pairs (L=word[0], R=word[1]).
 *
 * The AM824 encoder then extracts DATA24 = (S32 >> 8) & 0xFFFFFF,
 * placing each 16-bit SPDIF word in DATA24 bits 23-8.
 *
 * This matches the Python reference SpdifAes3SignalSource in ipmx_am824.py.
 */

#include "libavutil/intreadwrite.h"
#include "libavutil/common.h"
#include "avcodec.h"
#include "am824_s337m.h"

/* IEC 61937 sync words */
#define IEC61937_PA  0xF872
#define IEC61937_PB  0x4E1F

/* IEC 61937 data types (from spdif.h) */
#define IEC61937_AC3            0x01
#define IEC61937_MPEG2_AAC      0x07
#define IEC61937_EAC3           0x15

/**
 * Takes a compressed audio packet and produces S32 stereo samples
 * containing the IEC 61937 burst framing.
 *
 * @param codec_id   Input codec (AV_CODEC_ID_AAC, AC3, etc.)
 * @param pkt_data   Compressed audio payload
 * @param pkt_size   Size of compressed payload
 * @param out_buf    Output buffer for S32 stereo samples (caller allocates)
 * @param out_size   Size of output buffer in bytes
 * @param sample_rate Sample rate for the stream
 * @return Number of S32 samples written (total, both channels), or negative error
 */
int ff_am824_s337m_frame(int codec_id, const uint8_t *pkt_data, int pkt_size,
                         int32_t *out_buf, int out_capacity_samples,
                         int sample_rate)
{
    uint16_t data_type;
    int pkt_offset;       /* burst period in bytes */
    int total_16bit_words; /* burst period in 16-bit words */
    int total_stereo_samples; /* burst period in stereo sample pairs */
    int payload_words;
    int length_code;
    int i, w;

    /* Determine burst parameters */
    switch (codec_id) {
    case AV_CODEC_ID_AC3:
        data_type = IEC61937_AC3;
        pkt_offset = 6144;
        break;
    case AV_CODEC_ID_AAC:
        data_type = IEC61937_MPEG2_AAC;
        pkt_offset = 1024 * 4; /* 1024 samples * 4 bytes */
        break;
    case AV_CODEC_ID_EAC3:
        data_type = IEC61937_EAC3;
        pkt_offset = 24576;
        break;
    default:
        return AVERROR(EINVAL);
    }

    /* Burst period in 16-bit words and stereo samples */
    total_16bit_words = pkt_offset / 2;
    total_stereo_samples = total_16bit_words / 2; /* 2 words per stereo pair */

    if (total_stereo_samples * 2 > out_capacity_samples)
        return AVERROR(ENOMEM);

    /* Clear output buffer (padding is zeros) */
    memset(out_buf, 0, total_stereo_samples * 2 * sizeof(int32_t));

    /* Length code: payload size in bits */
    length_code = FFALIGN(pkt_size, 2) * 8;

    /* Write preamble: Pa, Pb, Pc, Pd as 16-bit BE values in S32 samples.
     * Each 16-bit word goes into the upper 16 bits of S32: word << 16.
     * This way, the AM824 encoder's DATA24 extraction (S32 >> 8) places
     * the 16-bit word in DATA24 bits 23-8, matching S337M mode 0. */
    w = 0;
    out_buf[w++] = (int32_t)((uint32_t)IEC61937_PA << 16); /* Pa → L */
    out_buf[w++] = (int32_t)((uint32_t)IEC61937_PB << 16); /* Pb → R */
    out_buf[w++] = (int32_t)((uint32_t)data_type << 16);   /* Pc → L */
    out_buf[w++] = (int32_t)((uint32_t)length_code << 16); /* Pd → R */

    /* Write payload: each pair of bytes becomes one 16-bit BE word → S32 */
    payload_words = pkt_size / 2;
    for (i = 0; i < payload_words && w < total_stereo_samples * 2; i++) {
        uint16_t word = AV_RB16(pkt_data + i * 2);
        out_buf[w++] = (int32_t)((uint32_t)word << 16);
    }

    /* Handle odd trailing byte (MSB-aligned per IEC 61937) */
    if (pkt_size & 1 && w < total_stereo_samples * 2) {
        uint16_t word = (uint16_t)pkt_data[pkt_size - 1] << 8;
        out_buf[w++] = (int32_t)((uint32_t)word << 16);
    }

    /* Remaining samples are zero (already memset) — this is the burst padding */

    return total_stereo_samples * 2; /* total S32 samples written */
}
