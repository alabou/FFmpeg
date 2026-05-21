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
 * Creates one AVStream per channel-order group (per token in
 * SMPTE2110.(token1,token2,…)). Examples:
 *   channel-order=SMPTE2110.(51)        → 1 stream, 5.1 PCM
 *   channel-order=SMPTE2110.(ST,AES3)   → 2 streams: stereo PCM + AES3 pair
 *   channel-order=SMPTE2110.(AES3,AES3) → 2 streams: 2 independent AES3 pairs
 *
 * PCM groups are output as PCM_S32BE (DATA24 left-justified to 32 bits),
 * with the canonical multichannel layout from the token (mono / stereo /
 * 5.1 / 7.1 / 22.2 / …). AES3 groups are parsed for S337M / IEC 61937
 * bursts and emit compressed packets (AAC, AC-3, EAC-3) once the data-type
 * field is decoded from the first burst.
 */

#include "libavutil/avstring.h"
#include "libavutil/channel_layout.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/mem.h"
#include "libavcodec/avcodec.h"
#include "avformat.h"
#include "internal.h"
#include "ipmx_chgroup.h"
#include "rtpdec.h"
#include "rtpdec_formats.h"

#define AM824_SUBFRAME_SIZE  4
#define AM824_DATA24_MASK    0x00FFFFFF

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

/* Per-AES3-group S337M burst accumulator */
typedef struct S337MAccum {
    uint16_t *words;
    int capacity;
    int count;
    enum AVCodecID codec_id;
} S337MAccum;

struct PayloadContext {
    int             nchan;            /* total wire channels (sum across groups) */
    int             n_groups;
    int             sample_rate;
    char           *channel_order;    /* original SDP body, kept for diagnostics */
    IpmxChGroup     groups[IPMX_MAX_CHGROUPS];
    int             streams_initialized;

    /* S337M accumulators — one per group, only AES3 groups use theirs */
    S337MAccum      s337m[IPMX_MAX_CHGROUPS];

    /* Per-RTP-packet output queue: parse_packet fills it, successive
     * buf==NULL calls drain it (one queue slot per emitted AVPacket). */
    AVPacket       *queue[IPMX_MAX_CHGROUPS * 2];
    int             queue_count;
    int             queue_rd;

    int64_t         pts;
};

/**
 * Try to extract a complete S337M burst from an AES3 group's accumulator.
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
    data->nchan       = st->codecpar->ch_layout.nb_channels;
    data->sample_rate = st->codecpar->sample_rate;
    data->pts         = 0;
    data->queue_count = 0;
    data->queue_rd    = 0;

    if (data->nchan < 2 || (data->nchan & 1))
        data->nchan = 2;

    for (i = 0; i < IPMX_MAX_CHGROUPS; i++) {
        memset(&data->s337m[i], 0, sizeof(data->s337m[i]));
    }
    for (i = 0; i < (int)(sizeof(data->queue) / sizeof(data->queue[0])); i++)
        data->queue[i] = NULL;

    return 0;
}

/**
 * Parse SDP a= lines. On the fmtp line with channel-order, parse the
 * tokens into per-group descriptors and allocate per-group AVStreams.
 */
static int am824_parse_sdp_line(AVFormatContext *s, int st_index,
                                PayloadContext *data, const char *line)
{
    const char *p;
    int i, n_groups, total_ch, ret;

    if (!av_strstart(line, "fmtp:", &p))
        return 0;

    /* Skip payload type number */
    while (*p && *p != ' ')
        p++;
    if (*p)
        p++;

    {
        const char *co = av_stristr(p, "channel-order=");
        if (co) {
            const char *body_paren, *end;
            co += strlen("channel-order=");
            end = co;
            while (*end && *end != ';' && *end != ' ' && *end != '\r' && *end != '\n')
                end++;
            av_free(data->channel_order);
            data->channel_order = av_strndup(co, end - co);

            /* Move past `SMPTE2110.(` to the first token. */
            body_paren = strchr(data->channel_order, '(');
            if (body_paren) {
                body_paren++;
                n_groups = ff_ipmx_parse_chorder(s, body_paren,
                                                 data->groups,
                                                 IPMX_MAX_CHGROUPS,
                                                 &total_ch);
                if (n_groups < 0)
                    return n_groups;
                data->n_groups = n_groups;
                if (total_ch > 0)
                    data->nchan = total_ch;
            }
        }
    }

    /* Allocate per-group AVStreams (idempotent: skip if already done — see
     * the streams_initialized comment below). */
    if (st_index >= 0 && data->n_groups > 0 && !data->streams_initialized) {
        AVStream *st0 = s->streams[st_index];

        ret = ff_ipmx_alloc_group_streams(s, st0,
                                          data->groups, data->n_groups,
                                          AV_CODEC_ID_PCM_S32BE,
                                          /* bits_per_coded_sample = */ 32,
                                          /* bits_per_raw_sample   = */ 24);
        if (ret < 0)
            return ret;

        data->streams_initialized = 1;

        av_log(s, AV_LOG_INFO,
               "AM824 RTP: %d signal group(s) from %s\n",
               data->n_groups,
               data->channel_order ? data->channel_order : "(default)");
        for (i = 0; i < data->n_groups; i++) {
            const IpmxChGroup *g = &data->groups[i];
            av_log(s, AV_LOG_INFO,
                   "  group %d → stream %d (%s, %d ch, offset %d)\n",
                   i, g->stream_idx,
                   g->kind == IPMX_CHGROUP_AES3 ? "AES3 / S337M" : "PCM",
                   g->nchannels, g->channel_offset);
        }
    }

    return 0;
}

/**
 * Handle an AM824 RTP packet. Deinterleaves all groups and queues per-group
 * packets. Returns first packet; subsequent buf==NULL calls drain the queue.
 */
static int am824_handle_packet(AVFormatContext *ctx, PayloadContext *data,
                               AVStream *st, AVPacket *pkt,
                               uint32_t *timestamp,
                               const uint8_t *buf, int len,
                               uint16_t seq, int flags)
{
    int g_idx, p, ch, ret;
    int nchan, n_groups, periods;

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

    if (len < 4 || (len % 4) != 0) {
        av_log(ctx, AV_LOG_WARNING,
               "AM824: invalid payload size %d\n", len);
        return AVERROR_INVALIDDATA;
    }

    /* IPMX PEP / HDCP decryption (TR-10-13). The generic rtpdec.c decrypt
     * path runs only when no dynamic handler is registered; handlers like
     * us are expected to decrypt their own payloads in place. The whole
     * AM824 RTP payload is encrypted (no per-payload header), so we
     * decrypt `buf, len` directly. This mirrors rtpdec_rfc4175 / rtpdec_h264.
     *
     * `s` (RTPDemuxContext) is reachable via ctx->parent.d, set by the
     * caller (rtp_recv_data) before invoking parse_packet. */
    {
        RTPDemuxContext *rtps = ctx->parent.d;
        int cipher_flags = 0;
        if (flags & RTP_FLAG_EXTHDRFULL)
            cipher_flags |= AVCIPHERCB_VALIDPARAM | AVCIPHERCB_VALIDINPUTCTR;

        #pragma GCC diagnostic ignored "-Wdiscarded-qualifiers"
        if (rtps && rtps->privacy_cipher) {
            if (rtps->privacy_cipher->callback(rtps->privacy_cipher->opaque,
                    rtps->encryption_stream_ctr, rtps->encryption_input_ctr,
                    buf, len, cipher_flags) < 0)
                return AVERROR_INVALIDDATA;
            len -= rtps->privacy_mac_size;  /* strip CMAC tag if present */
        }
        if (rtps && rtps->hdcp_cipher) {
            if (rtps->hdcp_cipher->callback(rtps->hdcp_cipher->opaque,
                    rtps->encryption_stream_ctr, rtps->encryption_input_ctr,
                    buf, len, cipher_flags) < 0)
                return AVERROR_INVALIDDATA;
        }
        if (rtps) {
            rtps->encryption_current_ctr = rtps->encryption_input_ctr +
                ((len & 15) ? (len >> 4) + 1 : (len >> 4));
        }
    }

    /* Re-validate length after MAC stripping. */
    if (len < 4 || (len % 4) != 0) {
        av_log(ctx, AV_LOG_WARNING,
               "AM824: invalid payload size %d after MAC strip\n", len);
        return AVERROR_INVALIDDATA;
    }

    nchan    = data->nchan;
    n_groups = data->n_groups;
    if (n_groups <= 0) {
        /* No channel-order parsed: fall back to one stereo PCM group so
         * we at least emit something usable. */
        data->groups[0].kind            = IPMX_CHGROUP_PCM;
        data->groups[0].nchannels       = 2;
        data->groups[0].channel_offset  = 0;
        data->groups[0].layout          = (AVChannelLayout)AV_CHANNEL_LAYOUT_STEREO;
        data->groups[0].stream_idx      = st ? st->index : 0;
        data->n_groups = n_groups = 1;
    }

    periods = (len / AM824_SUBFRAME_SIZE) / nchan;
    if (periods <= 0)
        return AVERROR_INVALIDDATA;

    /* Free any leftover queued packets from the previous RTP packet. */
    {
        int qi;
        for (qi = data->queue_rd; qi < data->queue_count; qi++)
            av_packet_free(&data->queue[qi]);
    }
    data->queue_count = 0;
    data->queue_rd    = 0;

    /* --- Deinterleave wire bytes into per-group output ---
     *
     * Wire layout per period: nchan AES3 subframes (4 B each), in declared
     * channel order. A group spans `nchannels` consecutive subframes
     * starting at `channel_offset`. */
    for (g_idx = 0; g_idx < n_groups; g_idx++) {
        IpmxChGroup *g = &data->groups[g_idx];
        int g_ch       = g->nchannels;
        int g_off      = g->channel_offset;
        int st_idx     = g->stream_idx;

        if (g->kind == IPMX_CHGROUP_PCM) {
            /* Output PCM_S32BE: DATA24 left-justified, repeated for every
             * channel in the group, every period. */
            AVPacket *pp = av_packet_alloc();
            int dst_per_period;
            if (!pp) return AVERROR(ENOMEM);
            dst_per_period = g_ch * 4;
            ret = av_new_packet(pp, periods * dst_per_period);
            if (ret < 0) { av_packet_free(&pp); return ret; }

            for (p = 0; p < periods; p++) {
                int src_period_off = p * nchan * AM824_SUBFRAME_SIZE;
                int dst_off        = p * dst_per_period;
                for (ch = 0; ch < g_ch; ch++) {
                    int src_off = src_period_off +
                                  (g_off + ch) * AM824_SUBFRAME_SIZE;
                    uint32_t word   = AV_RB32(buf + src_off);
                    uint32_t data24 = word & AM824_DATA24_MASK;
                    int32_t  sample = ((int32_t)data24) << 8;
                    AV_WB32(pp->data + dst_off + ch * 4, sample);
                }
            }

            pp->stream_index = st_idx;
            pp->pts          = data->pts;
            pp->dts          = data->pts;
            data->queue[data->queue_count++] = pp;

        } else {
            /* AES3 group (always 2 channels): accumulate 16-bit S/PDIF
             * words and try to extract S337M bursts. */
            S337MAccum *acc = &data->s337m[g_idx];
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
                int src_period_off = p * nchan * AM824_SUBFRAME_SIZE;
                for (ch = 0; ch < 2; ch++) {
                    int src_off = src_period_off +
                                  (g_off + ch) * AM824_SUBFRAME_SIZE;
                    uint32_t word   = AV_RB32(buf + src_off);
                    uint32_t data24 = word & AM824_DATA24_MASK;
                    acc->words[acc->count++] = (uint16_t)(data24 >> 8);
                }
            }

            while ((burst_pkt = try_extract_burst(acc)) != NULL) {
                /* On first burst: upgrade the AVStream's codec_id from NONE
                 * (set at SDP-allocation time) to the burst-derived codec. */
                if (st_idx >= 0 && st_idx < ctx->nb_streams &&
                    ctx->streams[st_idx]->codecpar->codec_id == AV_CODEC_ID_NONE) {
                    ctx->streams[st_idx]->codecpar->codec_id    = acc->codec_id;
                    ctx->streams[st_idx]->codecpar->sample_rate = data->sample_rate;
                    ctx->streams[st_idx]->codecpar->ch_layout   =
                        (AVChannelLayout)AV_CHANNEL_LAYOUT_STEREO;
                    av_log(ctx, AV_LOG_INFO,
                           "AM824 RTP: group %d (stream %d) detected %s (S337M)\n",
                           g_idx, st_idx, avcodec_get_name(acc->codec_id));
                }

                burst_pkt->stream_index = st_idx;
                burst_pkt->pts          = data->pts;
                burst_pkt->dts          = data->pts;
                if (data->queue_count <
                    (int)(sizeof(data->queue) / sizeof(data->queue[0]))) {
                    data->queue[data->queue_count++] = burst_pkt;
                } else {
                    av_packet_free(&burst_pkt);
                }
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
    for (i = 0; i < IPMX_MAX_CHGROUPS; i++)
        av_freep(&data->s337m[i].words);
    for (i = 0; i < (int)(sizeof(data->queue) / sizeof(data->queue[0])); i++)
        av_packet_free(&data->queue[i]);
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
