/*
 * bitpacked encoder
 *
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

#include "avcodec.h"
#include "codec_internal.h"
#include "encode.h"
#include "internal.h"
#include "put_bits.h"
#include "libavutil/pixdesc.h"

struct BitpackedContext {
    int (*encode)(AVCodecContext *avctx, AVPacket *pkt, const AVFrame *frame);
};

static int encode_yuv422p10(AVCodecContext *avctx, AVPacket *pkt, const AVFrame *frame)
{
    const int buf_size = avctx->height * avctx->width * avctx->bits_per_coded_sample / 8;
    int ret;
    uint8_t *dst;
    const uint16_t *y;
    const uint16_t *u;
    const uint16_t *v;
    PutBitContext pb;

    ret = ff_get_encode_buffer(avctx, pkt,  buf_size, 0);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "Error getting output packet.\n");
        return ret;
    }
    dst = pkt->data;

    init_put_bits(&pb, dst, buf_size);

    for (int i = 0; i < avctx->height; i++) {
        y = (uint16_t*)(frame->data[0] + i * frame->linesize[0]);
        u = (uint16_t*)(frame->data[1] + i * frame->linesize[1]);
        v = (uint16_t*)(frame->data[2] + i * frame->linesize[2]);

        for (int j = 0; j < avctx->width; j += 2) {
            /* u, y0, v, y1 */
            put_bits(&pb, 10, av_clip_uintp2(*u++, 10));
            put_bits(&pb, 10, av_clip_uintp2(*y++, 10));
            put_bits(&pb, 10, av_clip_uintp2(*v++, 10));
            put_bits(&pb, 10, av_clip_uintp2(*y++, 10));
        }
    }
    flush_put_bits(&pb);

    return 0;
}

/* RFC 4175 §4.3 / ST 2110-20 §6.2.3 — YCbCr 4:4:4 8-bit, pgroup=3, 1 px: Cb Y Cr. */
static int encode_yuv444p(AVCodecContext *avctx, AVPacket *pkt, const AVFrame *frame)
{
    const int buf_size = avctx->height * avctx->width * 3;
    int ret;
    uint8_t *dst;

    ret = ff_get_encode_buffer(avctx, pkt, buf_size, 0);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "Error getting output packet.\n");
        return ret;
    }
    dst = pkt->data;

    for (int i = 0; i < avctx->height; i++) {
        const uint8_t *y = frame->data[0] + i * frame->linesize[0];
        const uint8_t *u = frame->data[1] + i * frame->linesize[1];
        const uint8_t *v = frame->data[2] + i * frame->linesize[2];
        for (int j = 0; j < avctx->width; j++) {
            *dst++ = u[j];
            *dst++ = y[j];
            *dst++ = v[j];
        }
    }
    return 0;
}

/* RFC 4175 §6.2.3 / ST 2110-20 §6.2.4 — YCbCr 4:2:2 8-bit, pgroup=4, 2 px: Cb Y0 Cr Y1.
 * Identical wire layout to AV_PIX_FMT_UYVY422; this path lets planar producers
 * (libavfilter, lavfi sources) feed the muxer without an explicit format=uyvy422 shim. */
static int encode_yuv422p(AVCodecContext *avctx, AVPacket *pkt, const AVFrame *frame)
{
    const int buf_size = avctx->height * avctx->width * 2;
    int ret;
    uint8_t *dst;

    ret = ff_get_encode_buffer(avctx, pkt, buf_size, 0);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "Error getting output packet.\n");
        return ret;
    }
    dst = pkt->data;

    for (int i = 0; i < avctx->height; i++) {
        const uint8_t *y = frame->data[0] + i * frame->linesize[0];
        const uint8_t *u = frame->data[1] + i * frame->linesize[1];
        const uint8_t *v = frame->data[2] + i * frame->linesize[2];
        for (int j = 0; j < avctx->width; j += 2) {
            *dst++ = u[j / 2];
            *dst++ = y[j];
            *dst++ = v[j / 2];
            *dst++ = y[j + 1];
        }
    }
    return 0;
}

/* ST 2110-20:2022 §6.2.5 Table 3 + Figure 5 — YCbCr 4:2:0 10-bit. The 10-bit
 * pgroup is 15 octets covering 8 pixels (4 H × 2 V) — RFC 4175's pgroup
 * doubles relative to the 8-bit case because 6 samples × 10 bits = 60 bits
 * = 7.5 octets does not octet-align, so two adjacent 2×2 blocks are
 * concatenated for 12 samples × 10 bits = 120 bits = 15 octets. The sample
 * order per ST 2110-20 §6.2.5 Figure 5 is:
 *   Y'00 Y'01 Y'10 Y'11 Cb'00 Cr'00  Y'02 Y'03 Y'12 Y'13 Cb'01 Cr'01
 * (block-major: each half of the pgroup carries one 8-bit-style 2×2 block
 * with its own Cb/Cr pair). */
static int encode_yuv420p10(AVCodecContext *avctx, AVPacket *pkt, const AVFrame *frame)
{
    const int buf_size = (avctx->width / 4) * (avctx->height / 2) * 15;
    PutBitContext pb;
    int ret;

    ret = ff_get_encode_buffer(avctx, pkt, buf_size, 0);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "Error getting output packet.\n");
        return ret;
    }
    init_put_bits(&pb, pkt->data, buf_size);

    for (int i = 0; i < avctx->height; i += 2) {
        const uint16_t *y0 = (const uint16_t*)(frame->data[0] +  i      * frame->linesize[0]);
        const uint16_t *y1 = (const uint16_t*)(frame->data[0] + (i + 1) * frame->linesize[0]);
        const uint16_t *u  = (const uint16_t*)(frame->data[1] + (i / 2) * frame->linesize[1]);
        const uint16_t *v  = (const uint16_t*)(frame->data[2] + (i / 2) * frame->linesize[2]);
        for (int j = 0; j < avctx->width; j += 4) {
            /* Block 1: source columns j, j+1 across the line pair */
            put_bits(&pb, 10, av_clip_uintp2(y0[j    ], 10));
            put_bits(&pb, 10, av_clip_uintp2(y0[j + 1], 10));
            put_bits(&pb, 10, av_clip_uintp2(y1[j    ], 10));
            put_bits(&pb, 10, av_clip_uintp2(y1[j + 1], 10));
            put_bits(&pb, 10, av_clip_uintp2(u[ j / 2     ], 10));
            put_bits(&pb, 10, av_clip_uintp2(v[ j / 2     ], 10));
            /* Block 2: source columns j+2, j+3 across the same line pair */
            put_bits(&pb, 10, av_clip_uintp2(y0[j + 2], 10));
            put_bits(&pb, 10, av_clip_uintp2(y0[j + 3], 10));
            put_bits(&pb, 10, av_clip_uintp2(y1[j + 2], 10));
            put_bits(&pb, 10, av_clip_uintp2(y1[j + 3], 10));
            put_bits(&pb, 10, av_clip_uintp2(u[ j / 2 + 1], 10));
            put_bits(&pb, 10, av_clip_uintp2(v[ j / 2 + 1], 10));
        }
    }
    flush_put_bits(&pb);
    return 0;
}

/* RFC 4175 §6.2.4 / ST 2110-20 §6.2.5 — YCbCr 4:2:0 8-bit, pgroup=6 covering a
 * 2×2 source block: Y00 Y01 Y10 Y11 Cb00 Cr00. The packetizer pairs two source
 * scan lines per pgroup-row (yinc=2), so two consecutive Y rows are interleaved
 * here in the same byte order. */
static int encode_yuv420p(AVCodecContext *avctx, AVPacket *pkt, const AVFrame *frame)
{
    const int buf_size = avctx->height * avctx->width * 3 / 2;
    int ret;
    uint8_t *dst;

    ret = ff_get_encode_buffer(avctx, pkt, buf_size, 0);
    if (ret < 0) {
        av_log(avctx, AV_LOG_ERROR, "Error getting output packet.\n");
        return ret;
    }
    dst = pkt->data;

    for (int i = 0; i < avctx->height; i += 2) {
        const uint8_t *y0 = frame->data[0] +  i      * frame->linesize[0];
        const uint8_t *y1 = frame->data[0] + (i + 1) * frame->linesize[0];
        const uint8_t *u  = frame->data[1] + (i / 2) * frame->linesize[1];
        const uint8_t *v  = frame->data[2] + (i / 2) * frame->linesize[2];
        for (int j = 0; j < avctx->width; j += 2) {
            *dst++ = y0[j];
            *dst++ = y0[j + 1];
            *dst++ = y1[j];
            *dst++ = y1[j + 1];
            *dst++ = u[j / 2];
            *dst++ = v[j / 2];
        }
    }
    return 0;
}

static av_cold int encode_init(AVCodecContext *avctx)
{
    struct BitpackedContext *s = avctx->priv_data;
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(avctx->pix_fmt);

    if (avctx->width & 1) {
        av_log(avctx, AV_LOG_ERROR, "bitpacked needs even width\n");
        return AVERROR(EINVAL);
    }

    avctx->bits_per_coded_sample = av_get_bits_per_pixel(desc);
    avctx->bit_rate = ff_guess_coded_bitrate(avctx);

    switch (avctx->pix_fmt) {
    case AV_PIX_FMT_YUV422P10:
        s->encode = encode_yuv422p10;
        break;
    case AV_PIX_FMT_YUV444P:
        s->encode = encode_yuv444p;
        break;
    case AV_PIX_FMT_YUV422P:
        s->encode = encode_yuv422p;
        break;
    case AV_PIX_FMT_YUV420P:
        if (avctx->height & 1) {
            av_log(avctx, AV_LOG_ERROR,
                   "bitpacked YUV420P needs even height (got %d)\n",
                   avctx->height);
            return AVERROR(EINVAL);
        }
        s->encode = encode_yuv420p;
        break;
    case AV_PIX_FMT_YUV420P10:
        /* ST 2110-20 §6.2.5 — 10-bit 4:2:0 pgroup is 8 pixels (4 H × 2 V),
         * so width must be a multiple of 4 and height even. */
        if (avctx->width & 3) {
            av_log(avctx, AV_LOG_ERROR,
                   "bitpacked YUV420P10 needs width divisible by 4 (got %d)\n",
                   avctx->width);
            return AVERROR(EINVAL);
        }
        if (avctx->height & 1) {
            av_log(avctx, AV_LOG_ERROR,
                   "bitpacked YUV420P10 needs even height (got %d)\n",
                   avctx->height);
            return AVERROR(EINVAL);
        }
        s->encode = encode_yuv420p10;
        break;
    default:
        return AVERROR(EINVAL);
    }

    return 0;
}

static int encode_frame(AVCodecContext *avctx, AVPacket *pkt,
                        const AVFrame *frame, int *got_packet)
{
    struct BitpackedContext *s = avctx->priv_data;
    int ret;

    ret = s->encode(avctx, pkt, frame);
    if (ret)
        return ret;

    *got_packet = 1;
    return 0;
}

const FFCodec ff_bitpacked_encoder = {
    .p.name         = "bitpacked",
    CODEC_LONG_NAME("Bitpacked"),
    .p.type         = AVMEDIA_TYPE_VIDEO,
    .p.id           = AV_CODEC_ID_BITPACKED,
    .priv_data_size = sizeof(struct BitpackedContext),
    .p.capabilities = AV_CODEC_CAP_DR1 | AV_CODEC_CAP_FRAME_THREADS |
                      AV_CODEC_CAP_ENCODER_REORDERED_OPAQUE,
    .init           = encode_init,
    FF_CODEC_ENCODE_CB(encode_frame),
    CODEC_PIXFMTS(AV_PIX_FMT_YUV422P10,
                  AV_PIX_FMT_YUV444P,
                  AV_PIX_FMT_YUV422P,
                  AV_PIX_FMT_YUV420P,
                  AV_PIX_FMT_YUV420P10),
};
