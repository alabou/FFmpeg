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

#ifndef AVFORMAT_AM824_S337M_H
#define AVFORMAT_AM824_S337M_H

#include <stdint.h>

/**
 * Wrap a compressed audio packet in IEC 61937 / S337M burst framing
 * and output as S32 stereo PCM samples.
 *
 * Each 16-bit SPDIF word is placed in the upper 16 bits of an S32 sample
 * (word << 16), so the AM824 encoder extracts it as DATA24 bits 23-8.
 *
 * @param codec_id           Input codec (AV_CODEC_ID_AAC, AC3, EAC3, etc.)
 * @param pkt_data           Compressed audio payload
 * @param pkt_size           Size of compressed payload in bytes
 * @param out_buf            Output buffer for S32 samples (caller allocates)
 * @param out_capacity_samples  Capacity of out_buf in S32 samples
 * @param sample_rate        Sample rate (used for burst period calculation)
 * @return Number of S32 samples written (total, both channels), or negative error
 */
int ff_am824_s337m_frame(int codec_id, const uint8_t *pkt_data, int pkt_size,
                         int32_t *out_buf, int out_capacity_samples,
                         int sample_rate);

#endif /* AVFORMAT_AM824_S337M_H */
