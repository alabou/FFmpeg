/*
 * RTP depacketizer for AM824 payload format (SMPTE ST 2110-31)
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
 * RTP depacketizer for SMPTE ST 2110-31 AM824.
 *
 * Creates one AVStream per AES3 signal pair. PCM signals (channel-order "ST")
 * output S32BE audio. Non-PCM signals ("AES3") are parsed for S337M/IEC 61937
 * bursts and output compressed codec packets (AAC, AC-3, etc.) that FFmpeg
 * decodes with the standard pipeline.
 *
 * Follows the MPEG2TS multi-stream RTP pattern: parse_packet deinterleaves
 * the RTP payload across all signal pairs and queues per-signal packets.
 */

#include "libavutil/avstring.h"
#include "libavutil/channel_layout.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/mem.h"
#include "libavcodec/avcodec.h"
#include "avformat.h"
#include "internal.h"
#include "rtpdec_formats.h"

#define AM824_MAX_SIGNALS    40
#define AM824_SUBFRAME_SIZE  4
#define AM824_DATA24_MASK    0x00FFFFFF
#define AM824_PCUV_SHIFT     24
#define AM824_PCUV_MASK      0x0F
#define AM824_V_BIT          0x01

/* S337M 16-bit mode sync words */
#define S337M_PA  0xF872
#define S337M_PB  0x4E1F

/* IEC 61937 data types → FFmpeg codec IDs */
static enum AVCodecID s337m_type_to_codec(int data_type)
{
    switch (data_type & 0x1F) {
    case 0x01: return AV_CODEC_ID_AC3;
    case 0x07: return AV_CODEC_ID_AAC;
    case 0x15: return AV_CODEC_ID_EAC3;
    default:   return AV_CODEC_ID_NONE;
    }
}

/* Per-signal S337M burst accumulator */
typedef struct S337MAccum {
    uint16_t *words;
    int capacity;
    int count;
    enum AVCodecID codec_id;
} S337MAccum;

struct PayloadContext {
    int nchan;
    int nsignals;
    int sample_rate;
    char *channel_order;
    int signal_is_nonpcm[AM824_MAX_SIGNALS];
    int signal_stream_idx[AM824_MAX_SIGNALS]; /**< AVStream index per signal */

    /* S337M accumulators for non-PCM signals */
    S337MAccum s337m[AM824_MAX_SIGNALS];

    /* Packet queue: parse_packet fills, successive buf==NULL calls drain */
    AVPacket *queue[AM824_MAX_SIGNALS];
    int queue_count;
    int queue_rd;

    int64_t pts;
};

/**
 * Parse channel-order groups from SMPTE2110.(group1,group2,...).
 */
static void parse_channel_order_groups(PayloadContext *data)
{
    const char *p;
    int sig = 0;

    memset(data->signal_is_nonpcm, 0, sizeof(data->signal_is_nonpcm));
    data->nsignals = 0;

    if (!data->channel_order)
        return;

    p = strchr(data->channel_order, '(');
    if (!p)
        return;
    p++;

    while (*p && *p != ')' && sig < AM824_MAX_SIGNALS) {
        const char *end = p;
        while (*end && *end != ',' && *end != ')')
            end++;

        if (end - p == 4 && !av_strncasecmp(p, "AES3", 4)) {
            data->signal_is_nonpcm[sig] = 1;
        } else if (end - p == 2 && !av_strncasecmp(p, "51", 2)) {
            sig += 2;
        } else if (end - p == 2 && !av_strncasecmp(p, "71", 2)) {
            sig += 3;
        }

        sig++;
        p = end;
        if (*p == ',')
            p++;
    }

    data->nsignals = sig;
}

/**
 * Try to extract a complete S337M burst from a non-PCM signal's accumulator.
 * On success, allocates and returns an AVPacket with the compressed payload.
 */
static AVPacket *try_extract_burst(S337MAccum *acc)
{
    int w = 0;
    AVPacket *pkt;
    int pc, pd, data_type, length_bits, payload_bytes, payload_words;
    int consumed, pi;

    while (w + 3 < acc->count) {
        if (acc->words[w] == S337M_PA && acc->words[w+1] == S337M_PB) {
            pc = acc->words[w+2];
            pd = acc->words[w+3];
            data_type = pc & 0x1F;
            length_bits = pd;
            payload_bytes = (length_bits + 7) / 8;
            payload_words = (payload_bytes + 1) / 2;

            if (w + 4 + payload_words > acc->count) {
                /* Discard non-sync prefix, wait for more data */
                if (w > 0) {
                    memmove(acc->words, acc->words + w,
                            (acc->count - w) * sizeof(uint16_t));
                    acc->count -= w;
                }
                return NULL;
            }

            if (s337m_type_to_codec(data_type) == AV_CODEC_ID_NONE) {
                w += 4 + payload_words;
                continue;
            }

            /* Update detected codec */
            acc->codec_id = s337m_type_to_codec(data_type);

            /* Build compressed payload packet */
            pkt = av_packet_alloc();
            if (!pkt) return NULL;
            if (av_new_packet(pkt, payload_bytes) < 0) {
                av_packet_free(&pkt);
                return NULL;
            }
            for (pi = 0; pi < payload_words; pi++) {
                uint16_t pw = acc->words[w + 4 + pi];
                if (pi * 2 + 1 < payload_bytes) {
                    pkt->data[pi * 2]     = (pw >> 8) & 0xFF;
                    pkt->data[pi * 2 + 1] = pw & 0xFF;
                } else if (pi * 2 < payload_bytes) {
                    pkt->data[pi * 2] = (pw >> 8) & 0xFF;
                }
            }

            /* Consume burst + padding */
            consumed = w + 4 + payload_words;
            while (consumed < acc->count && acc->words[consumed] == 0)
                consumed++;
            if (consumed < acc->count)
                memmove(acc->words, acc->words + consumed,
                        (acc->count - consumed) * sizeof(uint16_t));
            acc->count -= consumed;
            return pkt;
        }
        w++;
    }

    /* Discard scanned non-sync bytes */
    if (w > 0 && w < acc->count) {
        memmove(acc->words, acc->words + w,
                (acc->count - w) * sizeof(uint16_t));
        acc->count -= w;
    }
    return NULL;
}

static av_cold int am824_init(AVFormatContext *s, int st_index,
                               PayloadContext *data)
{
    AVStream *st;
    int i;

    if (st_index < 0)
        return 0;

    st = s->streams[st_index];
    data->nchan = st->codecpar->ch_layout.nb_channels;
    data->sample_rate = st->codecpar->sample_rate;
    data->pts = 0;
    data->queue_count = 0;
    data->queue_rd = 0;

    if (data->nchan < 2 || (data->nchan & 1))
        data->nchan = 2;

    if (!data->nsignals)
        data->nsignals = data->nchan / 2;

    for (i = 0; i < AM824_MAX_SIGNALS; i++) {
        memset(&data->s337m[i], 0, sizeof(data->s337m[i]));
        data->queue[i] = NULL;
    }

    return 0;
}

/**
 * Parse SDP a= lines. On the fmtp line with channel-order, create
 * per-signal AVStreams with the correct codec parameters.
 */
static int am824_parse_sdp_line(AVFormatContext *s, int st_index,
                                PayloadContext *data, const char *line)
{
    const char *p;
    int i;

    if (av_strstart(line, "fmtp:", &p)) {
        const char *co;

        /* Skip payload type number */
        while (*p && *p != ' ')
            p++;
        if (*p)
            p++;

        co = av_stristr(p, "channel-order=");
        if (co) {
            const char *end;
            co += strlen("channel-order=");
            end = co;
            while (*end && *end != ';' && *end != ' ' && *end != '\r' && *end != '\n')
                end++;
            av_free(data->channel_order);
            data->channel_order = av_strndup(co, end - co);
            parse_channel_order_groups(data);
        }

        /* Now create per-signal AVStreams.
         * Stream 0 already exists (created by SDP parser for the m= line).
         * We modify it for signal 0, and create new streams for signals 1..N-1.
         */
        if (st_index >= 0 && data->nsignals > 0) {
            AVStream *st0 = s->streams[st_index];
            int sr = st0->codecpar->sample_rate;

            for (i = 0; i < data->nsignals; i++) {
                AVStream *st;

                if (i == 0) {
                    /* Reuse existing stream for signal 0 */
                    st = st0;
                } else {
                    st = avformat_new_stream(s, NULL);
                    if (!st)
                        return AVERROR(ENOMEM);
                    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
                    st->codecpar->sample_rate = sr;
                    avpriv_set_pts_info(st, 32, 1, sr);
                }

                data->signal_stream_idx[i] = st->index;
                st->codecpar->ch_layout = (AVChannelLayout)AV_CHANNEL_LAYOUT_STEREO;

                if (data->signal_is_nonpcm[i]) {
                    /* Non-PCM: codec_id stays NONE until S337M detection.
                     * Set to NONE explicitly so avformat_find_stream_info probes. */
                    st->codecpar->codec_id = AV_CODEC_ID_NONE;
                } else {
                    /* PCM: output as S32BE (DATA24 left-justified to 32 bits) */
                    st->codecpar->codec_id = AV_CODEC_ID_PCM_S32BE;
                    st->codecpar->bits_per_coded_sample = 32;
                    st->codecpar->bits_per_raw_sample   = 24;
                }
            }

            av_log(s, AV_LOG_INFO,
                   "AM824 RTP: %d signal pairs from %s\n",
                   data->nsignals, data->channel_order ? data->channel_order : "(default)");
            for (i = 0; i < data->nsignals; i++) {
                av_log(s, AV_LOG_INFO, "  signal %d → stream %d (%s)\n",
                       i, data->signal_stream_idx[i],
                       data->signal_is_nonpcm[i] ? "non-PCM/S337M" : "PCM");
            }
        }
    }

    return 0;
}

/**
 * Handle an AM824 RTP packet. Deinterleaves ALL signal pairs and queues
 * per-signal packets. Returns first packet; subsequent buf==NULL calls
 * drain the queue (return 1 = more, 0 = done, <0 = error/empty).
 */
static int am824_handle_packet(AVFormatContext *ctx, PayloadContext *data,
                               AVStream *st, AVPacket *pkt,
                               uint32_t *timestamp,
                               const uint8_t *buf, int len,
                               uint16_t seq, int flags)
{
    int sig, p, ch, ret;
    int nchan, nsignals, periods;

    /* --- Drain queued packets (buf==NULL) --- */
    if (!buf) {
        if (data->queue_rd < data->queue_count) {
            AVPacket *qp = data->queue[data->queue_rd];
            data->queue[data->queue_rd] = NULL;
            data->queue_rd++;
            av_packet_move_ref(pkt, qp);
            av_packet_free(&qp);
            return (data->queue_rd < data->queue_count) ? 1 : 0;
        }
        return AVERROR(EAGAIN);
    }

    /* --- Process new RTP payload --- */
    if (len < 4 || (len % 4) != 0) {
        av_log(ctx, AV_LOG_WARNING,
               "AM824: invalid payload size %d\n", len);
        return AVERROR_INVALIDDATA;
    }

    nchan = data->nchan;
    nsignals = data->nsignals;
    if (nsignals <= 0) {
        /* Fallback: no channel-order parsed, single signal */
        nsignals = 1;
        data->signal_stream_idx[0] = st ? st->index : 0;
    }

    periods = (len / AM824_SUBFRAME_SIZE) / nchan;
    if (periods <= 0)
        return AVERROR_INVALIDDATA;

    /* Free any leftover queued packets */
    {
        int qi;
        for (qi = data->queue_rd; qi < data->queue_count; qi++)
            av_packet_free(&data->queue[qi]);
    }
    data->queue_count = 0;
    data->queue_rd = 0;

    /* Deinterleave ALL signals from this RTP payload */
    for (sig = 0; sig < nsignals; sig++) {
        int st_idx = data->signal_stream_idx[sig];

        if (!data->signal_is_nonpcm[sig]) {
            /* PCM signal: extract DATA24 as S32BE stereo */
            AVPacket *pp = av_packet_alloc();
            if (!pp) return AVERROR(ENOMEM);
            ret = av_new_packet(pp, periods * 2 * 4);
            if (ret < 0) { av_packet_free(&pp); return ret; }

            for (p = 0; p < periods; p++) {
                int src_off = (p * nchan + sig * 2) * AM824_SUBFRAME_SIZE;
                int dst_off = p * 2 * 4;
                for (ch = 0; ch < 2; ch++) {
                    uint32_t word = AV_RB32(buf + src_off + ch * AM824_SUBFRAME_SIZE);
                    uint32_t data24 = word & AM824_DATA24_MASK;
                    int32_t sample = ((int32_t)data24) << 8;
                    AV_WB32(pp->data + dst_off + ch * 4, sample);
                }
            }

            pp->stream_index = st_idx;
            pp->pts = data->pts;
            pp->dts = data->pts;
            data->queue[data->queue_count++] = pp;

        } else {
            /* Non-PCM signal: accumulate 16-bit SPDIF words */
            S337MAccum *acc = &data->s337m[sig];
            int needed = acc->count + periods * 2;
            AVPacket *burst_pkt;

            if (needed > acc->capacity) {
                int new_cap = FFMAX(needed, FFMAX(acc->capacity * 2, 8192));
                uint16_t *nw = av_realloc(acc->words, new_cap * sizeof(uint16_t));
                if (!nw) return AVERROR(ENOMEM);
                acc->words = nw;
                acc->capacity = new_cap;
            }

            for (p = 0; p < periods; p++) {
                int src_off = (p * nchan + sig * 2) * AM824_SUBFRAME_SIZE;
                for (ch = 0; ch < 2; ch++) {
                    uint32_t word = AV_RB32(buf + src_off + ch * AM824_SUBFRAME_SIZE);
                    uint32_t data24 = word & AM824_DATA24_MASK;
                    acc->words[acc->count++] = (uint16_t)(data24 >> 8);
                }
            }

            /* Try to extract S337M burst(s) */
            while ((burst_pkt = try_extract_burst(acc)) != NULL) {
                /* On first burst: update the stream's codec_id */
                if (st_idx >= 0 && st_idx < ctx->nb_streams &&
                    ctx->streams[st_idx]->codecpar->codec_id == AV_CODEC_ID_NONE) {
                    ctx->streams[st_idx]->codecpar->codec_id = acc->codec_id;
                    ctx->streams[st_idx]->codecpar->sample_rate = data->sample_rate;
                    ctx->streams[st_idx]->codecpar->ch_layout =
                        (AVChannelLayout)AV_CHANNEL_LAYOUT_STEREO;
                    av_log(ctx, AV_LOG_INFO,
                           "AM824 RTP: signal %d detected %s (S337M)\n",
                           sig, avcodec_get_name(acc->codec_id));
                }

                burst_pkt->stream_index = st_idx;
                burst_pkt->pts = data->pts;
                burst_pkt->dts = data->pts;
                data->queue[data->queue_count++] = burst_pkt;
            }
        }
    }

    data->pts += periods;

    /* Return first queued packet */
    if (data->queue_count > 0) {
        AVPacket *qp = data->queue[0];
        data->queue[0] = NULL;
        data->queue_rd = 1;
        av_packet_move_ref(pkt, qp);
        av_packet_free(&qp);

        /* Tell rtpdec we set timestamps ourselves */
        *timestamp = RTP_NOTS_VALUE;
        return (data->queue_rd < data->queue_count) ? 1 : 0;
    }

    return AVERROR(EAGAIN);
}

static void am824_close_context(PayloadContext *data)
{
    int i;
    av_freep(&data->channel_order);
    for (i = 0; i < AM824_MAX_SIGNALS; i++) {
        av_freep(&data->s337m[i].words);
        av_packet_free(&data->queue[i]);
    }
}

const RTPDynamicProtocolHandler ff_am824_dynamic_handler = {
    .enc_name         = "AM824",
    .codec_type       = AVMEDIA_TYPE_AUDIO,
    .codec_id         = AV_CODEC_ID_AM824,
    .need_parsing     = AVSTREAM_PARSE_NONE,
    .priv_data_size   = sizeof(PayloadContext),
    .init             = am824_init,
    .parse_sdp_a_line = am824_parse_sdp_line,
    .parse_packet     = am824_handle_packet,
    .close            = am824_close_context,
};
