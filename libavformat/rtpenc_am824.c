/*
 * RTP packetizer for AM824 payload format (SMPTE ST 2110-31)
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
 * RTP packetizer for SMPTE ST 2110-31 AM824.
 *
 * Per ST 2110-31 Section 5.4: "Each packet within a stream shall contain
 * AES3 Subframes from the same number of periods of the sampling clock."
 *
 * This packetizer buffers incoming AM824 data and sends exactly
 * ptime-aligned packets. Partial data at the end of stream is flushed.
 */

#include "libavutil/intreadwrite.h"
#include "avformat.h"
#include "rtpenc.h"

static int am824_periods_per_packet(int sample_rate)
{
    switch (sample_rate) {
    case 96000: return 96;
    case 44100: return 48;
    case 48000:
    default:    return 48;
    }
}

void ff_rtp_send_am824(AVFormatContext *s1, const uint8_t *buf, int size)
{
    RTPMuxContext *s = s1->priv_data;
    int nchan = s1->streams[0]->codecpar->ch_layout.nb_channels;
    int sample_rate = s1->streams[0]->codecpar->sample_rate;
    int periods_per_pkt, bytes_per_pkt;

    if (nchan < 2)
        nchan = 2;

    periods_per_pkt = am824_periods_per_packet(sample_rate);
    bytes_per_pkt = periods_per_pkt * nchan * 4;

    if (bytes_per_pkt > s->max_payload_size) {
        int sample_size = nchan * 4;
        bytes_per_pkt = (s->max_payload_size / sample_size) * sample_size;
        periods_per_pkt = bytes_per_pkt / (nchan * 4);
    }

    if (bytes_per_pkt <= 0)
        return;

    /* We manage s->timestamp ourselves to ensure strictly constant deltas
     * of exactly periods_per_pkt. On the very first call (packet_count==0
     * and buffer empty), seed from cur_timestamp. After that, timestamp
     * only advances by periods_per_pkt when we emit an RTP packet. */
    if (s->packet_count == 0 && s->buf_ptr == s->buf)
        s->timestamp = s->cur_timestamp;

    /* Append incoming data to the RTP buffer.
     * Send a packet each time we accumulate exactly bytes_per_pkt.
     * We manage s->timestamp ourselves to ensure constant delta of
     * exactly periods_per_pkt between consecutive RTP packets. */
    while (size > 0) {
        int buffered = s->buf_ptr - s->buf;
        int space = bytes_per_pkt - buffered;
        int to_copy = FFMIN(space, size);

        memcpy(s->buf_ptr, buf, to_copy);
        s->buf_ptr += to_copy;
        buf += to_copy;
        size -= to_copy;

        buffered = s->buf_ptr - s->buf;
        if (buffered >= bytes_per_pkt) {
            s1->payload_header_size = 0;
            s1->full_ctr = 1;
            ff_rtp_send_data(s1, s->buf, bytes_per_pkt, 0);

            /* Advance timestamp by exactly periods_per_pkt */
            s->timestamp += periods_per_pkt;

            s->buf_ptr = s->buf;
        }
    }
}
