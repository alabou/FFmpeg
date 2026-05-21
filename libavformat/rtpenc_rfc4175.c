/*
 * RTP Packetization of RAW video (RFC4175)
 * Copyright (c) 2021 Limin Wang
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
#include "avformat.h"
#include "rtpenc.h"
#include "libavutil/pixdesc.h"

/* Per-pixel-format wire-format parameters for RFC 4175 / SMPTE ST 2110-20.
 *
 * The packetizer is a thin scanline-segment fragmenter — it does not pack
 * pixels. The contract is that pkt->data already holds RFC 4175 wire-format
 * bytes (pgroup-after-pgroup, scanline-after-scanline). That holds for
 * pix_fmts whose memory layout matches the wire layout (UYVY422 / RGB24
 * under -c:v rawvideo) and for pix_fmts packed by the bitpacked encoder
 * (libavcodec/bitpacked_enc.c).
 *
 * Adding a new sampling/depth combination means: (1) appending an entry
 * here, (2) implementing the matching packer in bitpacked_enc.c if the
 * AV_PIX_FMT memory layout doesn't already equal the wire layout,
 * (3) adding the symmetric depacketizer entry in rtpdec_rfc4175.c, and
 * (4) the SDP sampling/depth in sdp.c (and ipmxenc.c if used through IPMX).
 *
 * yinc is the number of source scan lines covered by one pgroup-row. For
 * 4:2:0 the chroma is shared between two consecutive luma rows so yinc=2;
 * everything else is yinc=1 progressive (or yinc=2 when interlaced
 * doubles the row stride).
 */
/* Memory-packed pix_fmts (UYVY422, RGB24) are valid under either codec_id:
 * rawvideo's identity copy and bitpacked's no-op are equivalent because
 * the in-memory layout already matches the RFC 4175 wire layout. The IPMX
 * muxer forces codec_id to BITPACKED on the child rtp_ctx for all raw
 * video (ipmxenc.c:2846-2847), so the BITPACKED rows must be present even
 * though no actual bitpacked-encoder packing happens for them.
 *
 * BGR24 is deliberately *not* listed: ST 2110-20 / RFC 4175 §6.2.4 only
 * define the `RGB` sampling token (R then G then B on the wire). Emitting
 * `sampling=BGR` with the B-then-G-then-R memory order on the wire is
 * non-conformant and causes any compliant receiver — including hardware
 * analyzers like Telestream VB440 — to fall back to RGB decoding,
 * swapping red and blue. Users with a BGR24 source should lavfi-convert
 * via `,format=bgr24,format=rgb24,` before this packetizer. */
const FFRFC4175Format ff_rfc4175_formats[] = {
    /* pix_fmt              codec_id              pgroup xinc yinc pix dep sampling */
    { AV_PIX_FMT_UYVY422,   AV_CODEC_ID_RAWVIDEO,   4,   2,   1,  2,  8, "YCbCr-4:2:2" },
    { AV_PIX_FMT_UYVY422,   AV_CODEC_ID_BITPACKED,  4,   2,   1,  2,  8, "YCbCr-4:2:2" },
    { AV_PIX_FMT_RGB24,     AV_CODEC_ID_RAWVIDEO,   3,   1,   1,  1,  8, "RGB"         },
    { AV_PIX_FMT_RGB24,     AV_CODEC_ID_BITPACKED,  3,   1,   1,  1,  8, "RGB"         },
    { AV_PIX_FMT_YUV422P10, AV_CODEC_ID_BITPACKED,  5,   2,   1,  2, 10, "YCbCr-4:2:2" },
    { AV_PIX_FMT_YUV444P,   AV_CODEC_ID_BITPACKED,  3,   1,   1,  1,  8, "YCbCr-4:4:4" },
    { AV_PIX_FMT_YUV422P,   AV_CODEC_ID_BITPACKED,  4,   2,   1,  2,  8, "YCbCr-4:2:2" },
    /* YUV420P: ST 2110-20 §6.2.5 — pgroup covers 2H × 2V pixels (yinc=2),
     * progressive-only (interlaced 4:2:0 is forbidden by spec; the validation
     * gate in rtpenc.c rejects it). */
    { AV_PIX_FMT_YUV420P,   AV_CODEC_ID_BITPACKED,  6,   2,   2,  4,  8, "YCbCr-4:2:0" },
    /* YUV420P10: ST 2110-20 §6.2.5 Table 3 + Figure 5 — 10-bit 4:2:0 pgroup
     * covers 8 pixels (4 H × 2 V) in 15 octets because 6×10 bits = 60 = 7.5
     * octets does not octet-align, so two adjacent 2×2 blocks are
     * concatenated. xinc=4, yinc=2, pixels=8. */
    { AV_PIX_FMT_YUV420P10, AV_CODEC_ID_BITPACKED, 15,   4,   2,  8, 10, "YCbCr-4:2:0" },
    { AV_PIX_FMT_NONE,      AV_CODEC_ID_NONE,       0,   0,   0,  0,  0, NULL          },
};

const FFRFC4175Format *ff_rfc4175_get_format(enum AVCodecID codec_id,
                                             enum AVPixelFormat pix_fmt)
{
    const FFRFC4175Format *f;
    for (f = ff_rfc4175_formats; f->pix_fmt != AV_PIX_FMT_NONE; f++) {
        if (f->codec_id == codec_id && f->pix_fmt == pix_fmt)
            return f;
    }
    return NULL;
}

void ff_rtp_send_raw_rfc4175(AVFormatContext *s1, const uint8_t *buf, int size, int interlaced, int field)
{
    RTPMuxContext *s = s1->priv_data;
    int width = s1->streams[0]->codecpar->width;
    int height = s1->streams[0]->codecpar->height;
    const FFRFC4175Format *fmt;
    int xinc, yinc, pgroup;
    int i = 0;
    int offset = 0;

    s->timestamp = s->cur_timestamp;

    fmt = ff_rfc4175_get_format(s1->streams[0]->codecpar->codec_id,
                                s1->streams[0]->codecpar->format);
    if (!fmt) {
        av_log(s1, AV_LOG_ERROR,
               "RFC 4175: pixel format %s not supported by the rfc4175 "
               "packetizer (this should have been caught at write_header)\n",
               av_get_pix_fmt_name(s1->streams[0]->codecpar->format));
        return;
    }
    xinc   = fmt->xinc;
    yinc   = fmt->yinc << interlaced;
    pgroup = fmt->pgroup;

    // For RFC 4175 (video raw) the paylaod header is 2 byte in addition to 6 byte
    // per scanline included. All the payload header bytes are at the beginning of
    // payload.
    s1->full_ctr = 1;

    while (i < height) {
        int left = s->max_payload_size;
        uint8_t *dest = s->buf;
        uint8_t *headers;
        const int head_size = 6;
        int next_line;
        int length, cont, pixels;
        int payload_header_size = 0;

        /* Extended Sequence Number (ST 2110-20 §6.1.4) — high 16 bits of
         * the 32-bit extended sequence. The low 16 bits are the RTP
         * `seq` field written by ff_rtp_send_data. The high word is
         * stored on the muxer ctx (s->rfc4175_ext_seq) and bumped after
         * ff_rtp_send_data below whenever s->seq wraps 0xFFFF→0x0000. */
        *dest++ = (s->rfc4175_ext_seq >> 8) & 0xff;
        *dest++ = s->rfc4175_ext_seq & 0xff;
        left   -= 2;
        payload_header_size += 2;

        headers = dest;
        do {
            int l_line;

            pixels = width - offset;
            length = (pixels * pgroup) / xinc;

            left -= head_size;
            if (left >= length) {
                next_line = 1;
            } else {
                pixels = (left / pgroup) * xinc;
                length = (pixels * pgroup) / xinc;
                next_line = 0;
            }
            left -= length;

            /* Length */
            *dest++ = (length >> 8) & 0xff;
            *dest++ = length & 0xff;

            /* Line No */
            l_line = i >> interlaced;
            *dest++ = ((l_line >> 8) & 0x7f) | ((field << 7) & 0x80);
            *dest++ = l_line & 0xff;
            if (next_line) i += yinc;

            cont = (left > (head_size + pgroup) && i < height) ? 0x80 : 0x00;
            /* Offset and Continuation marker */
            *dest++ = ((offset >> 8) & 0x7f) | cont;
            *dest++ = offset & 0xff;

            if (next_line)
                offset  = 0;
            else
                offset += pixels;

            payload_header_size += 6;

        } while (cont);

        do {
            int l_field;
            int l_line;
            int l_off;
            int copy_offset;

            length    = (headers[0] << 8) | headers[1];
            l_field   = (headers[2] & 0x80) >> 7;
            l_line    = ((headers[2] & 0x7f) << 8) | headers[3];
            l_off     = ((headers[4] & 0x7f) << 8) | headers[5];
            cont      = headers[4] & 0x80;
            headers  += head_size;

            if (interlaced)
                l_line = 2 * l_line + l_field;
            /* Map source-line index to wire-row index. For 4:2:0 (fmt->yinc=2)
             * one wire-row covers two source scan lines, so the byte offset
             * within pkt->data steps by W*pgroup/xinc per wire-row, not per
             * source line. For non-4:2:0 (fmt->yinc=1) the divisor is 1 and
             * the math collapses to the original (l_line * width) form. */
            copy_offset = ((l_line / fmt->yinc) * width + l_off) * pgroup / xinc;
            if (copy_offset + length > size)
                break;
            memcpy (dest, buf + copy_offset, length);
            dest += length;
        } while (cont);

        s1->payload_header_size = payload_header_size;
        ff_rtp_send_data (s1, s->buf, s->max_payload_size - left, i >= height);
        /* ff_rtp_send_data sent the RTP packet with s->seq, then bumped
         * `s->seq = (s->seq + 1) & 0xffff`. If that wrapped to 0, we just
         * crossed the 16→17-bit boundary in the extended-seq counter —
         * bump the high word so the NEXT packet's header carries the
         * correct extended seq. */
        if (s->seq == 0)
            s->rfc4175_ext_seq++;

        if (next_line) {
            s1->full_ctr = 1;
        } else {
            s1->full_ctr = 0;
        }
   }
}
