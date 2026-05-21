/*
 * AM824 S337M wrapping pseudo-decoder
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
 * AM824 S337M wrapping pseudo-decoder.
 *
 * "Decodes" a compressed audio packet (AAC, AC-3, EAC3) into S32 stereo
 * samples that carry the corresponding IEC 61937 burst (Pa/Pb/Pc/Pd
 * preamble + payload + zero padding). Output is directly suitable as
 * input to the AM824 encoder with `-non_pcm 1`, which then writes the
 * burst content into AES3 subframes with V=1.
 *
 * Use:
 *   ffmpeg -c:a am824_s337m_wrap -i in.aac \
 *          -c:a am824 -non_pcm 1 \
 *          -f ipmx udp://...
 *
 * Why this exists: bringing AAC content to the wire as IEC 61937 bursts
 * (per IEC 61937-6 / TR-10-12) used to require an intermediate `.spdif`
 * file written by the spdif muxer and then re-read as raw S16BE PCM —
 * an awkward two-step pipeline that bypasses ffmpeg's normal
 * encoder→muxer flow. This decoder collapses that to one ffmpeg
 * invocation: the AAC source is read as a normal codec stream, the
 * decoder wraps each frame in an IEC 61937 burst inline, and the AM824
 * encoder + downstream muxer (rtp / ipmx) take it from there.
 *
 * Internally calls ff_am824_s337m_frame() — the same wrap helper that
 * libavformat/am824.c uses inside its muxer-side s337m_wrap_compressed.
 */

#include "libavutil/channel_layout.h"
#include "libavutil/intreadwrite.h"
#include "avcodec.h"
#include "codec_internal.h"
#include "decode.h"
#include "am824_s337m.h"

typedef struct AM824S337mWrapContext {
    AVClass *av_class;
} AM824S337mWrapContext;

/* Burst period (in IEC 61937 bytes) per codec. Mirrors the pkt_offset
 * switch in libavcodec/am824_s337m.c. Determines how many S32 stereo
 * samples we produce per input compressed packet. */
static int burst_period_bytes(enum AVCodecID codec_id)
{
    switch (codec_id) {
    case AV_CODEC_ID_AC3:  return 6144;
    case AV_CODEC_ID_AAC:  return 4096;   /* 1024 audio samples × 4 bytes */
    case AV_CODEC_ID_EAC3: return 24576;
    default:               return -1;
    }
}

/* Source-codec samples per compressed frame. Identical to the burst-period
 * sample counts since the IEC 61937 burst preserves the source frame size
 * (Pa/Pb/Pc/Pd preamble + payload + zero padding all fit in one burst
 * period of source-codec samples). avctx->frame_size populates
 * av_get_audio_frame_duration() which the AAC/AC3/EAC3 demuxers need so
 * compute_pkt_fields() can derive a per-packet DTS — without it, the
 * AAC ADTS demuxer cannot build seek-index entries and -stream_loop fails
 * at the EOF wrap-around with EPERM. */
static int source_frame_size(enum AVCodecID codec_id)
{
    switch (codec_id) {
    case AV_CODEC_ID_AAC:  return 1024;
    case AV_CODEC_ID_AC3:  return 1536;
    case AV_CODEC_ID_EAC3: return 1536;
    default:               return 0;
    }
}

static av_cold int am824_s337m_wrap_init(AVCodecContext *avctx)
{
    if (burst_period_bytes(avctx->codec_id) < 0) {
        av_log(avctx, AV_LOG_ERROR,
               "am824_s337m_wrap: codec_id=%d not supported (AAC/AC3/EAC3 only)\n",
               avctx->codec_id);
        return AVERROR(EINVAL);
    }

    av_channel_layout_uninit(&avctx->ch_layout);
    av_channel_layout_default(&avctx->ch_layout, 2);
    avctx->sample_fmt = AV_SAMPLE_FMT_S32;
    if (avctx->sample_rate <= 0)
        avctx->sample_rate = 48000;
    avctx->bits_per_raw_sample = 24;
    avctx->frame_size          = source_frame_size(avctx->codec_id);
    return 0;
}

static int am824_s337m_wrap_decode(AVCodecContext *avctx, AVFrame *frame,
                                   int *got_frame, AVPacket *avpkt)
{
    int pkt_offset = burst_period_bytes(avctx->codec_id);
    int total_stereo_samples;
    int n_samples;
    int ret;

    if (pkt_offset < 0)
        return AVERROR(EINVAL);

    if (avpkt->size <= 0) {
        *got_frame = 0;
        return 0;
    }

    /* Burst period in stereo sample pairs (one S32 sample per IEC 61937
     * 16-bit word; two words per stereo sample period). */
    total_stereo_samples = pkt_offset / 4;

    frame->nb_samples = total_stereo_samples;
    av_channel_layout_uninit(&frame->ch_layout);
    av_channel_layout_default(&frame->ch_layout, 2);
    frame->format      = AV_SAMPLE_FMT_S32;
    frame->sample_rate = avctx->sample_rate;

    ret = ff_get_buffer(avctx, frame, 0);
    if (ret < 0)
        return ret;

    n_samples = ff_am824_s337m_frame(avctx->codec_id,
                                     avpkt->data, avpkt->size,
                                     (int32_t *)frame->data[0],
                                     total_stereo_samples * 2,
                                     avctx->sample_rate);
    if (n_samples < 0) {
        *got_frame = 0;
        return n_samples;
    }

    /* Hint downstream (the am824 encoder, in particular) that this frame's
     * samples are an IEC 61937 / S337m burst, not linear PCM. The encoder
     * uses this to emit V=1 / NOT_PCM channel-status without the user
     * needing to pass `-non_pcm 1` explicitly. */
    {
        AVFrameSideData *sd = av_frame_new_side_data(frame,
                                                     AV_FRAME_DATA_AM824_NON_PCM,
                                                     1);
        if (sd)
            sd->data[0] = 1;
    }

    *got_frame = 1;
    return avpkt->size;
}

const FFCodec ff_am824_s337m_wrap_decoder = {
    .p.name         = "am824_s337m_wrap",
    CODEC_LONG_NAME("AM824 S337M wrap (compressed → S32 IEC 61937 burst frames)"),
    .p.type         = AVMEDIA_TYPE_AUDIO,
    .p.id           = AV_CODEC_ID_AAC,
    /* CHANNEL_CONF tells avcodec.c:open2 that this decoder can be opened
     * before sample_rate / ch_layout are filled in by upstream parsing —
     * we set them ourselves in init() and decode_frame() based on the
     * input codec_id and packets. Without this, opening the decoder during
     * avformat_find_stream_info fails with "Invalid sample rate: 0" before
     * our init even runs. */
    .p.capabilities = AV_CODEC_CAP_DR1 | AV_CODEC_CAP_CHANNEL_CONF,
    .priv_data_size = sizeof(AM824S337mWrapContext),
    .init           = am824_s337m_wrap_init,
    FF_CODEC_DECODE_CB(am824_s337m_wrap_decode),
};
