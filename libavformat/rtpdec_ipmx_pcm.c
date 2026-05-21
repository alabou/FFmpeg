/*
 * RTP depacketizer for L24 / L16 PCM with IPMX channel-order awareness
 * (SMPTE ST 2110-30 §6.2 multi-program signal groups).
 *
 * Activates only when:
 *   - the SDP fmtp carries `channel-order=SMPTE2110.(…)` AND
 *   - AVFormatContext::ipmx == 1 (set by CLI or auto-detected from SDP).
 *
 * In that case it parses the channel-order body into per-token groups and
 * allocates one AVStream per group, deinterleaving the wire payload into
 * per-group PCM AVPackets. Outside those conditions the depacketizer is a
 * no-op and the standard rtpdec.c raw-payload path emits one multichannel
 * AVStream per RFC 3551 — preserving legacy behaviour bit-for-bit.
 *
 * Copyright (c) 2026 Matrox
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License as published by the
 * Free Software Foundation; either version 2.1 of the License, or (at your
 * option) any later version.
 */

#include "libavutil/avstring.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/mem.h"
#include "libavcodec/avcodec.h"
#include "avformat.h"
#include "internal.h"
#include "ipmx_chgroup.h"
#include "rtpdec.h"
#include "rtpdec_formats.h"

typedef struct PCMVariant {
    enum AVCodecID codec_id;
    int            bits_per_sample;
    int            bytes_per_sample;
} PCMVariant;

struct PayloadContext {
    int            n_groups;
    int            total_channels;        /* sum across groups */
    char          *channel_order;
    IpmxChGroup    groups[IPMX_MAX_CHGROUPS];
    int            streams_initialized;
    int            ipmx_active;           /* 1 once we've decided to take over */
    PCMVariant     pcm;                   /* L24 by default; resolved at init */

    AVPacket      *queue[IPMX_MAX_CHGROUPS];
    int            queue_count;
    int            queue_rd;

    int64_t        pts;
};

static av_cold int ipmx_pcm_init(AVFormatContext *s, int st_index,
                                 PayloadContext *data)
{
    AVStream *st;
    int i;

    if (st_index < 0)
        return 0;

    st = s->streams[st_index];

    /* L24 by default; if the registered handler is for L16, codec_id will
     * already be PCM_S16BE on the AVStream. */
    if (st->codecpar->codec_id == AV_CODEC_ID_PCM_S16BE) {
        data->pcm.codec_id         = AV_CODEC_ID_PCM_S16BE;
        data->pcm.bits_per_sample  = 16;
        data->pcm.bytes_per_sample = 2;
    } else {
        data->pcm.codec_id         = AV_CODEC_ID_PCM_S24BE;
        data->pcm.bits_per_sample  = 24;
        data->pcm.bytes_per_sample = 3;
    }

    data->total_channels = st->codecpar->ch_layout.nb_channels;
    data->pts            = 0;
    data->queue_count    = 0;
    data->queue_rd       = 0;
    data->ipmx_active    = 0;
    for (i = 0; i < IPMX_MAX_CHGROUPS; i++)
        data->queue[i] = NULL;
    return 0;
}

static int ipmx_pcm_parse_sdp_line(AVFormatContext *s, int st_index,
                                   PayloadContext *data, const char *line)
{
    const char *p, *co;

    if (!av_strstart(line, "fmtp:", &p))
        return 0;

    /* Skip payload type number */
    while (*p && *p != ' ')
        p++;
    if (*p)
        p++;

    co = av_stristr(p, "channel-order=");
    if (!co)
        return 0;  /* no channel-order → leave the standard path alone */

    /* IPMX gate: only activate when the session is declared / detected as
     * IPMX. Otherwise behave like the legacy raw-PCM path: do nothing here
     * and let the standard rtpdec.c emit one multichannel AVStream. */
    if (s->ipmx != 1)
        return 0;

    {
        const char *body_paren, *end;
        int n_groups, total_ch, ret;

        co += strlen("channel-order=");
        end = co;
        while (*end && *end != ';' && *end != ' ' && *end != '\r' && *end != '\n')
            end++;
        av_free(data->channel_order);
        data->channel_order = av_strndup(co, end - co);

        body_paren = strchr(data->channel_order, '(');
        if (!body_paren)
            return 0;
        body_paren++;

        n_groups = ff_ipmx_parse_chorder(s, body_paren,
                                         data->groups, IPMX_MAX_CHGROUPS,
                                         &total_ch);
        if (n_groups < 0)
            return n_groups;
        data->n_groups       = n_groups;
        data->total_channels = total_ch > 0 ? total_ch : data->total_channels;

        /* Reject AES3 groups on a plain L24/L16 stream — AES3 means
         * S337M-wrapped non-PCM, which the AM824 depacketizer handles.
         * If a sender claims `(AES3)` on an L24 m= block that's almost
         * certainly an SDP authoring error; we fall back to non-IPMX
         * behaviour rather than producing garbage. */
        {
            int has_aes3 = 0, i;
            for (i = 0; i < n_groups; i++) {
                if (data->groups[i].kind == IPMX_CHGROUP_AES3) {
                    has_aes3 = 1;
                    break;
                }
            }
            if (has_aes3) {
                av_log(s, AV_LOG_WARNING,
                       "L24/L16 channel-order contains AES3 token, which is "
                       "only valid on AM824. Ignoring channel-order on this "
                       "stream and falling back to single multichannel.\n");
                data->n_groups = 0;
                return 0;
            }
        }

        if (st_index >= 0 && data->n_groups > 0 && !data->streams_initialized) {
            AVStream *st0 = s->streams[st_index];
            int i;
            ret = ff_ipmx_alloc_group_streams(s, st0, data->groups,
                                              data->n_groups,
                                              data->pcm.codec_id,
                                              data->pcm.bits_per_sample,
                                              data->pcm.bits_per_sample);
            if (ret < 0)
                return ret;
            data->streams_initialized = 1;
            data->ipmx_active         = 1;

            av_log(s, AV_LOG_INFO,
                   "IPMX L%d RTP: %d signal group(s) from %s\n",
                   data->pcm.bits_per_sample,
                   data->n_groups,
                   data->channel_order ? data->channel_order : "(default)");
            for (i = 0; i < data->n_groups; i++) {
                const IpmxChGroup *g = &data->groups[i];
                av_log(s, AV_LOG_INFO,
                       "  group %d → stream %d (%d ch, offset %d)\n",
                       i, g->stream_idx, g->nchannels, g->channel_offset);
            }
        }
    }
    return 0;
}

static int ipmx_pcm_handle_packet(AVFormatContext *ctx, PayloadContext *data,
                                  AVStream *st, AVPacket *pkt,
                                  uint32_t *timestamp,
                                  const uint8_t *buf, int len,
                                  uint16_t seq, int flags)
{
    int g_idx, p, ch, ret;
    int periods, bps;
    int qlim = (int)(sizeof(data->queue) / sizeof(data->queue[0]));

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

    /* IPMX PEP / HDCP decryption (TR-10-13). The standard rtpdec.c decrypt
     * path only runs when no dynamic handler is registered; we're a
     * registered handler so we must decrypt our own payload here. The
     * whole RTP payload is encrypted (no per-payload header on plain
     * L24/L16). Mirrors rtpdec_rfc4175 / rtpdec_h264. */
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
            len -= rtps->privacy_mac_size;
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

    /* Legacy / non-IPMX path — mimic the standard rtpdec.c raw-payload
     * behaviour (one packet of the raw bytes, attached to the m= line's
     * AVStream). This is what would have happened before this depacketizer
     * was registered, so RFC 3551 receivers see no behavioural change. */
    if (!data->ipmx_active || data->n_groups <= 0) {
        if ((ret = av_new_packet(pkt, len)) < 0)
            return ret;
        memcpy(pkt->data, buf, len);
        if (st)
            pkt->stream_index = st->index;
        return 0;  /* timestamp left to caller; standard wrap-around handling */
    }

    bps = data->pcm.bytes_per_sample;
    if (data->total_channels <= 0 || bps <= 0 ||
        len % (data->total_channels * bps) != 0)
        return AVERROR_INVALIDDATA;

    periods = len / (data->total_channels * bps);
    if (periods <= 0)
        return AVERROR_INVALIDDATA;

    /* Free any leftover queued packets from the previous RTP packet */
    {
        int qi;
        for (qi = data->queue_rd; qi < data->queue_count; qi++)
            av_packet_free(&data->queue[qi]);
    }
    data->queue_count = 0;
    data->queue_rd    = 0;

    /* Deinterleave wire bytes into per-group output. */
    for (g_idx = 0; g_idx < data->n_groups; g_idx++) {
        IpmxChGroup *g = &data->groups[g_idx];
        int          g_ch  = g->nchannels;
        int          g_off = g->channel_offset;
        int          dst_per_period = g_ch * bps;
        AVPacket    *pp;

        pp = av_packet_alloc();
        if (!pp) return AVERROR(ENOMEM);
        ret = av_new_packet(pp, periods * dst_per_period);
        if (ret < 0) { av_packet_free(&pp); return ret; }

        for (p = 0; p < periods; p++) {
            int src_period_off = p * data->total_channels * bps;
            int dst_off        = p * dst_per_period;
            for (ch = 0; ch < g_ch; ch++) {
                const uint8_t *src = buf + src_period_off + (g_off + ch) * bps;
                uint8_t       *dst = pp->data + dst_off + ch * bps;
                memcpy(dst, src, bps);
            }
        }

        pp->stream_index = g->stream_idx;
        pp->pts          = data->pts;
        pp->dts          = data->pts;
        if (data->queue_count < qlim)
            data->queue[data->queue_count++] = pp;
        else
            av_packet_free(&pp);
    }

    data->pts += periods;

    if (data->queue_count > 0) {
        AVPacket *qp = data->queue[0];
        data->queue[0] = NULL;
        data->queue_rd = 1;
        av_packet_move_ref(pkt, qp);
        av_packet_free(&qp);
        *timestamp = RTP_NOTS_VALUE;
        return (data->queue_rd < data->queue_count) ? 1 : 0;
    }

    return AVERROR(EAGAIN);
}

static void ipmx_pcm_close(PayloadContext *data)
{
    int i;
    av_freep(&data->channel_order);
    for (i = 0; i < (int)(sizeof(data->queue) / sizeof(data->queue[0])); i++)
        av_packet_free(&data->queue[i]);
}

const RTPDynamicProtocolHandler ff_ipmx_l24_dynamic_handler = {
    .enc_name         = "L24",
    .codec_type       = AVMEDIA_TYPE_AUDIO,
    .codec_id         = AV_CODEC_ID_PCM_S24BE,
    .need_parsing     = AVSTREAM_PARSE_NONE,
    .priv_data_size   = sizeof(PayloadContext),
    .init             = ipmx_pcm_init,
    .parse_sdp_a_line = ipmx_pcm_parse_sdp_line,
    .parse_packet     = ipmx_pcm_handle_packet,
    .close            = ipmx_pcm_close,
};

const RTPDynamicProtocolHandler ff_ipmx_l16_dynamic_handler = {
    .enc_name         = "L16",
    .codec_type       = AVMEDIA_TYPE_AUDIO,
    .codec_id         = AV_CODEC_ID_PCM_S16BE,
    .need_parsing     = AVSTREAM_PARSE_NONE,
    .priv_data_size   = sizeof(PayloadContext),
    .init             = ipmx_pcm_init,
    .parse_sdp_a_line = ipmx_pcm_parse_sdp_line,
    .parse_packet     = ipmx_pcm_handle_packet,
    .close            = ipmx_pcm_close,
};
