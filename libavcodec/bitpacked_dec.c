/*
 * Unpack bit-packed streams to formats supported by FFmpeg
 * Copyright (c) 2017 Savoir-faire Linux, Inc
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

/* Development sponsored by CBC/Radio-Canada */

/**
 * @file
 * Bitpacked
 */

#include "avcodec.h"
#include "codec_internal.h"
#include "get_bits.h"
#include "libavutil/imgutils.h"
#include "thread.h"

struct BitpackedContext {
    int (*decode)(AVCodecContext *avctx, AVFrame *frame,
                  const AVPacket *pkt);
};

/* For this format, it's a simple passthrough */
static int bitpacked_decode_uyvy422(AVCodecContext *avctx, AVFrame *frame,
                                    const AVPacket *avpkt)
{
    int ret;

    /* there is no need to copy as the data already match
     * a known pixel format */
    frame->buf[0] = av_buffer_ref(avpkt->buf);
    if (!frame->buf[0]) {
        return AVERROR(ENOMEM);
    }

    ret = av_image_fill_arrays(frame->data, frame->linesize, avpkt->data,
                               avctx->pix_fmt, avctx->width, avctx->height, 1);
    if (ret < 0) {
        av_buffer_unref(&frame->buf[0]);
        return ret;
    }

    return 0;
}

static int bitpacked_decode_yuv422p10(AVCodecContext *avctx, AVFrame *frame,
                                      const AVPacket *avpkt)
{
    uint64_t frame_size = (uint64_t)avctx->width * (uint64_t)avctx->height * 20;
    uint64_t packet_size = (uint64_t)avpkt->size * 8;
    uint8_t *src;
    uint16_t *y, *u, *v;
    int ret, i, j;

    ret = ff_thread_get_buffer(avctx, frame, 0);
    if (ret < 0)
        return ret;

    if (frame_size > packet_size)
        return AVERROR_INVALIDDATA;

    if (avctx->width % 2)
        return AVERROR_PATCHWELCOME;

    src = avpkt->data;
    for (i = 0; i < avctx->height; i++) {
        y = (uint16_t*)(frame->data[0] + i * frame->linesize[0]);
        u = (uint16_t*)(frame->data[1] + i * frame->linesize[1]);
        v = (uint16_t*)(frame->data[2] + i * frame->linesize[2]);

        for (j = 0; j < avctx->width; j += 2) {
            *u++ = (src[0] << 2) | (src[1] >> 6);
            *v++ = ((src[2] << 6) | (src[3] >> 2)) & 0x3ff;
            *y++ = ((src[1] << 4) | (src[2] >> 4)) & 0x3ff;
            *y++ = ((src[3] << 8) | (src[4]))      & 0x3ff;
            src += 5;
        }
    }

    return 0;
}

/* RFC 4175 §6.2.3 / ST 2110-20 §6.2.4 — YCbCr-4:4:4 8-bit, 3-byte pgroup
 * Cb-Y-Cr per pixel, unpacked into AV_PIX_FMT_YUV444P. */
static int bitpacked_decode_yuv444p(AVCodecContext *avctx, AVFrame *frame,
                                    const AVPacket *avpkt)
{
    const int expected = avctx->width * avctx->height * 3;
    const uint8_t *src;
    int ret, i, j;

    if (avpkt->size < expected)
        return AVERROR_INVALIDDATA;

    ret = ff_thread_get_buffer(avctx, frame, 0);
    if (ret < 0)
        return ret;

    src = avpkt->data;
    for (i = 0; i < avctx->height; i++) {
        uint8_t *y = frame->data[0] + i * frame->linesize[0];
        uint8_t *u = frame->data[1] + i * frame->linesize[1];
        uint8_t *v = frame->data[2] + i * frame->linesize[2];
        for (j = 0; j < avctx->width; j++) {
            *u++ = *src++;
            *y++ = *src++;
            *v++ = *src++;
        }
    }
    return 0;
}

/* ST 2110-20:2022 §6.2.5 Table 3 + Figure 5 — YCbCr-4:2:0 10-bit, 15-octet
 * pgroup covering 8 pixels (4 H × 2 V) as two adjacent 8-bit-style 2×2
 * blocks: Y00 Y01 Y10 Y11 Cb00 Cr00 Y02 Y03 Y12 Y13 Cb01 Cr01 (each 10 bits,
 * MSB-first bitstream). */
static int bitpacked_decode_yuv420p10(AVCodecContext *avctx, AVFrame *frame,
                                      const AVPacket *avpkt)
{
    const int expected = (avctx->width / 4) * (avctx->height / 2) * 15;
    GetBitContext gb;
    int ret, i, j;

    if ((avctx->width  & 3) || (avctx->height & 1))
        return AVERROR_PATCHWELCOME;
    if (avpkt->size < expected)
        return AVERROR_INVALIDDATA;

    ret = ff_thread_get_buffer(avctx, frame, 0);
    if (ret < 0)
        return ret;

    if ((ret = init_get_bits8(&gb, avpkt->data, avpkt->size)) < 0)
        return ret;

    for (i = 0; i < avctx->height; i += 2) {
        uint16_t *y0 = (uint16_t*)(frame->data[0] +  i      * frame->linesize[0]);
        uint16_t *y1 = (uint16_t*)(frame->data[0] + (i + 1) * frame->linesize[0]);
        uint16_t *u  = (uint16_t*)(frame->data[1] + (i / 2) * frame->linesize[1]);
        uint16_t *v  = (uint16_t*)(frame->data[2] + (i / 2) * frame->linesize[2]);
        for (j = 0; j < avctx->width; j += 4) {
            /* Block 1 */
            *y0++ = get_bits(&gb, 10);
            *y0++ = get_bits(&gb, 10);
            *y1++ = get_bits(&gb, 10);
            *y1++ = get_bits(&gb, 10);
            *u++  = get_bits(&gb, 10);
            *v++  = get_bits(&gb, 10);
            /* Block 2 */
            *y0++ = get_bits(&gb, 10);
            *y0++ = get_bits(&gb, 10);
            *y1++ = get_bits(&gb, 10);
            *y1++ = get_bits(&gb, 10);
            *u++  = get_bits(&gb, 10);
            *v++  = get_bits(&gb, 10);
        }
    }
    return 0;
}

/* RFC 4175 §6.2.4 / ST 2110-20 §6.2.5 — YCbCr-4:2:0 8-bit, 6-byte pgroup
 * covering a 2×2 source block: Y00 Y01 Y10 Y11 Cb00 Cr00. ST 2110-20 §6.2.5
 * forbids interlaced 4:2:0; the depacketizer's even-height assumption mirrors
 * that and is already enforced upstream by the encoder validation gate. */
static int bitpacked_decode_yuv420p(AVCodecContext *avctx, AVFrame *frame,
                                    const AVPacket *avpkt)
{
    const int expected = avctx->width * avctx->height * 3 / 2;
    const uint8_t *src;
    int ret, i, j;

    if (avctx->width  % 2 || avctx->height % 2)
        return AVERROR_PATCHWELCOME;
    if (avpkt->size < expected)
        return AVERROR_INVALIDDATA;

    ret = ff_thread_get_buffer(avctx, frame, 0);
    if (ret < 0)
        return ret;

    src = avpkt->data;
    for (i = 0; i < avctx->height; i += 2) {
        uint8_t *y0 = frame->data[0] +  i      * frame->linesize[0];
        uint8_t *y1 = frame->data[0] + (i + 1) * frame->linesize[0];
        uint8_t *u  = frame->data[1] + (i / 2) * frame->linesize[1];
        uint8_t *v  = frame->data[2] + (i / 2) * frame->linesize[2];
        for (j = 0; j < avctx->width; j += 2) {
            *y0++ = *src++;
            *y0++ = *src++;
            *y1++ = *src++;
            *y1++ = *src++;
            *u++  = *src++;
            *v++  = *src++;
        }
    }
    return 0;
}

static av_cold int bitpacked_init_decoder(AVCodecContext *avctx)
{
    struct BitpackedContext *bc = avctx->priv_data;

    if (!avctx->codec_tag || !avctx->width || !avctx->height)
        return AVERROR_INVALIDDATA;

    if (avctx->codec_tag == MKTAG('U', 'Y', 'V', 'Y')) {
        if (avctx->bits_per_coded_sample == 16 &&
            avctx->pix_fmt == AV_PIX_FMT_UYVY422)
            bc->decode = bitpacked_decode_uyvy422;
        else if (avctx->bits_per_coded_sample == 20 &&
                 avctx->pix_fmt == AV_PIX_FMT_YUV422P10)
            bc->decode = bitpacked_decode_yuv422p10;
        else
            return AVERROR_INVALIDDATA;
    } else if (avctx->codec_tag == MKTAG('I', '4', '4', '4') &&
               avctx->pix_fmt == AV_PIX_FMT_YUV444P) {
        bc->decode = bitpacked_decode_yuv444p;
    } else if (avctx->codec_tag == MKTAG('I', '4', '2', '0') &&
               avctx->pix_fmt == AV_PIX_FMT_YUV420P) {
        bc->decode = bitpacked_decode_yuv420p;
    } else if (avctx->codec_tag == MKTAG('I', '4', '2', '0') &&
               avctx->pix_fmt == AV_PIX_FMT_YUV420P10) {
        bc->decode = bitpacked_decode_yuv420p10;
    } else {
        return AVERROR_INVALIDDATA;
    }

    return 0;
}

static int bitpacked_decode(AVCodecContext *avctx, AVFrame *frame,
                            int *got_frame, AVPacket *avpkt)
{
    struct BitpackedContext *bc = avctx->priv_data;
    int buf_size = avpkt->size;
    int res;

    res = bc->decode(avctx, frame, avpkt);
    if (res)
        return res;

    *got_frame = 1;
    return buf_size;

}

const FFCodec ff_bitpacked_decoder = {
    .p.name          = "bitpacked",
    CODEC_LONG_NAME("Bitpacked"),
    .p.type          = AVMEDIA_TYPE_VIDEO,
    .p.id            = AV_CODEC_ID_BITPACKED,
    .p.capabilities  = AV_CODEC_CAP_FRAME_THREADS,
    .priv_data_size        = sizeof(struct BitpackedContext),
    .init = bitpacked_init_decoder,
    FF_CODEC_DECODE_CB(bitpacked_decode),
    .codec_tags     = (const uint32_t []){
        MKTAG('U', 'Y', 'V', 'Y'),
        MKTAG('I', '4', '4', '4'),
        MKTAG('I', '4', '2', '0'),
        FF_CODEC_TAGS_END,
    },
};
