/*
 * SMPTE ST 2110-31 AM824 AES3 multi-signal muxer/demuxer
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
 * SMPTE ST 2110-31 AM824 multi-signal demuxer and muxer.
 *
 * DEMUXER: Splits an interleaved AM824 stream into N output streams
 * (one per AES3 signal pair). Inspects the V (validity) bit:
 *   - V=0: PCM — outputs S32BE audio
 *   - V=1: non-PCM — outputs raw DATA24 as S32BE for S337M/SPDIF pipeline
 *
 * MUXER: Accepts N stereo input streams and interleaves them into a single
 * AM824 stream per ST 2110-31 Section 5.4:
 *   For each sample period: [sig0_L, sig0_R, sig1_L, sig1_R, ...]
 */

#include "libavutil/avstring.h"
#include "libavutil/channel_layout.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavcodec/avcodec.h"
#include "am824_s337m.h"
#include "avformat.h"
#include "demux.h"
#include "internal.h"
#include "mux.h"
#include "url.h"
#include "rtpenc_chain.h"

#define AM824_SUBFRAME_SIZE  4
#define AM824_DATA24_MASK    0x00FFFFFF
#define AM824_PCUV_SHIFT     24
#define AM824_PCUV_MASK      0x0F
#define AM824_V_BIT          0x01

#define AM824_MAX_SIGNALS    40

/* =====================================================================
 *  DEMUXER
 * ===================================================================== */

enum AM824SignalType {
    AM824_SIG_UNKNOWN = 0,
    AM824_SIG_PCM     = 1,
    AM824_SIG_NONPCM  = 2,
};

/* S337M 16-bit mode sync words in DATA24 (word << 8) */
#define S337M_PA_DATA24  0xF87200
#define S337M_PB_DATA24  0x4E1F00

/* IEC 61937 data types → FFmpeg codec IDs */
static enum AVCodecID s337m_data_type_to_codec(int data_type)
{
    switch (data_type & 0x1F) {
    case 0x01: return AV_CODEC_ID_AC3;
    case 0x07: return AV_CODEC_ID_AAC;
    case 0x15: return AV_CODEC_ID_EAC3;
    default:   return AV_CODEC_ID_NONE;
    }
}

/* Per-signal S337M burst accumulator for non-PCM streams */
typedef struct S337MAccum {
    uint16_t *words;        /**< Accumulated 16-bit SPDIF words (from DATA24 >> 8) */
    int capacity;           /**< Allocated capacity in words */
    int count;              /**< Current word count */
    int burst_detected;     /**< 1 if Pa/Pb found and we're accumulating payload */
    int data_type;          /**< Detected IEC 61937 data type from Pc */
    int length_bits;        /**< Declared payload length in bits from Pd */
    int payload_start;      /**< Word index where payload starts (after Pd) */
    enum AVCodecID codec_id; /**< Detected codec */
} S337MAccum;

/* RTP header size (no CSRC, no extension) */
#define RTP_HEADER_SIZE 12

typedef struct AM824DemuxContext {
    AVClass *class;
    int nchan;
    int nsignals;
    int sample_rate;
    char *am824_chorder;        /**< channel-order string e.g. "SMPTE2110.(ST,AES3)" */
    enum AM824SignalType signal_types[AM824_MAX_SIGNALS];
    S337MAccum s337m_accum[AM824_MAX_SIGNALS]; /**< Per-signal S337M accumulator */
    int cur_signal;             /**< Round-robin: next signal to emit */
    int64_t pts;
    uint8_t *read_buf;          /**< Reusable read buffer */
    int read_buf_size;
    int probed;                 /**< 1 after V-bit probing is done */
    /* Per-signal PCM buffers for deinterleaving */
    AVPacket *pcm_pending[AM824_MAX_SIGNALS]; /**< Buffered PCM packets */
    int chunk_loaded;           /**< 1 if current chunk has been deinterleaved */
    /* RTP reception state (when input is SDP) */
    URLContext *rtp_handle;     /**< RTP/UDP URL handle, or NULL for raw file input */
    uint8_t *rtp_buf;           /**< Buffer for receiving RTP datagrams */
    int rtp_buf_size;
} AM824DemuxContext;

/**
 * Auto-detect signal type from the V bit of the first subframe in a pair.
 */
static enum AM824SignalType detect_v_bit(const uint8_t *subframe)
{
    uint32_t word = AV_RB32(subframe);
    int pcuv = (word >> AM824_PCUV_SHIFT) & AM824_PCUV_MASK;
    return (pcuv & AM824_V_BIT) ? AM824_SIG_NONPCM : AM824_SIG_PCM;
}

/**
 * Parse channel-order string (e.g. "SMPTE2110.(ST,AES3)") to determine
 * which signals are non-PCM. Sets signal_types[] for known signals.
 */
static void parse_am824_chorder(AM824DemuxContext *ctx, const char *co)
{
    const char *p;
    int sig = 0;

    if (!co)
        return;

    /* Find the opening parenthesis */
    p = strchr(co, '(');
    if (!p)
        return;
    p++; /* skip '(' */

    while (*p && *p != ')' && sig < ctx->nsignals) {
        if (!strncmp(p, "AES3", 4)) {
            ctx->signal_types[sig] = AM824_SIG_NONPCM;
            p += 4;
        } else if (!strncmp(p, "ST", 2)) {
            ctx->signal_types[sig] = AM824_SIG_PCM;
            p += 2;
        } else {
            p++;
            continue;
        }
        sig++;
        if (*p == ',') p++;
    }
}

/**
 * Probe the first few chunks of AM824 data to detect V bits and,
 * for non-PCM signals, scan for S337M sync words to identify the codec.
 * The probed data is stored in the S337M accumulators for non-PCM signals
 * so it is not lost.
 */
static int am824_probe_signals(AVFormatContext *s)
{
    AM824DemuxContext *ctx = s->priv_data;
    int nchan = ctx->nchan;
    int periods = (ctx->sample_rate == 96000) ? 96 : 48;
    int total_bytes = periods * nchan * AM824_SUBFRAME_SIZE;
    /* Read enough data to find at least one S337M burst.
     * AAC burst period = 4096 bytes / 4 = 1024 stereo samples = ~21ms at 48kHz.
     * Read 50ms worth of data (enough for 2+ bursts). */
    int probe_chunks = 50;
    int chunk, sig, p, ch;
    uint8_t *buf;

    buf = av_malloc(total_bytes);
    if (!buf)
        return AVERROR(ENOMEM);

    for (chunk = 0; chunk < probe_chunks; chunk++) {
        int ret = avio_read(s->pb, buf, total_bytes);
        int read_subframes, read_periods;
        if (ret <= 0)
            break;

        read_subframes = ret / AM824_SUBFRAME_SIZE;
        read_periods = read_subframes / nchan;
        if (read_periods <= 0)
            break;

        for (sig = 0; sig < ctx->nsignals; sig++) {
            /* Detect V bit on first chunk if not already known */
            if (ctx->signal_types[sig] == AM824_SIG_UNKNOWN) {
                int offset = sig * 2 * AM824_SUBFRAME_SIZE;
                ctx->signal_types[sig] = detect_v_bit(buf + offset);
            }

            /* For non-PCM signals, accumulate 16-bit SPDIF words */
            if (ctx->signal_types[sig] == AM824_SIG_NONPCM) {
                S337MAccum *acc = &ctx->s337m_accum[sig];
                int needed = acc->count + read_periods * 2;
                if (needed > acc->capacity) {
                    int new_cap = FFMAX(needed, 8192);
                    acc->words = av_realloc(acc->words, new_cap * sizeof(uint16_t));
                    if (!acc->words) { acc->capacity = 0; av_free(buf); return AVERROR(ENOMEM); }
                    acc->capacity = new_cap;
                }
                for (p = 0; p < read_periods; p++) {
                    int src_base = (p * nchan + sig * 2) * AM824_SUBFRAME_SIZE;
                    for (ch = 0; ch < 2; ch++) {
                        uint32_t word = AV_RB32(buf + src_base + ch * AM824_SUBFRAME_SIZE);
                        uint32_t data24 = word & AM824_DATA24_MASK;
                        acc->words[acc->count++] = (uint16_t)(data24 >> 8);
                    }
                }
            }
        }

        /* Check if all non-PCM signals have detected their codec */
        {
            int all_detected = 1;
            for (sig = 0; sig < ctx->nsignals; sig++) {
                if (ctx->signal_types[sig] == AM824_SIG_NONPCM &&
                    ctx->s337m_accum[sig].codec_id == AV_CODEC_ID_NONE) {
                    /* Scan for Pa/Pb */
                    S337MAccum *acc = &ctx->s337m_accum[sig];
                    int w;
                    for (w = 0; w + 3 < acc->count; w++) {
                        if (acc->words[w] == 0xF872 && acc->words[w+1] == 0x4E1F) {
                            int pc = acc->words[w+2];
                            int dt = pc & 0x1F;
                            acc->codec_id = s337m_data_type_to_codec(dt);
                            acc->data_type = dt;
                            if (acc->codec_id != AV_CODEC_ID_NONE) {
                                av_log(s, AV_LOG_INFO,
                                       "AM824 probe: signal %d → codec %s (S337M type 0x%02x)\n",
                                       sig, avcodec_get_name(acc->codec_id), dt);
                            }
                            break;
                        }
                    }
                    if (acc->codec_id == AV_CODEC_ID_NONE)
                        all_detected = 0;
                }
            }
            if (all_detected)
                break;
        }
    }

    av_free(buf);

    /* Seek back to the beginning so read_packet starts from the top.
     * For non-seekable streams, the probed data remains in accumulators. */
    if (s->pb->seekable & AVIO_SEEKABLE_NORMAL) {
        avio_seek(s->pb, 0, SEEK_SET);
        /* Clear all accumulators — data will be re-read from beginning */
        for (sig = 0; sig < ctx->nsignals; sig++)
            ctx->s337m_accum[sig].count = 0;
    }

    ctx->probed = 1;
    return 0;
}

static int am824_read_header(AVFormatContext *s)
{
    AM824DemuxContext *ctx = s->priv_data;
    int i, ret;

    if (ctx->nchan < 2 || (ctx->nchan & 1)) {
        av_log(s, AV_LOG_ERROR,
               "AM824: nchan must be even and >= 2, got %d\n", ctx->nchan);
        return AVERROR(EINVAL);
    }

    ctx->nsignals = ctx->nchan / 2;
    if (ctx->nsignals > AM824_MAX_SIGNALS) {
        av_log(s, AV_LOG_ERROR, "AM824: too many signals (%d > %d)\n",
               ctx->nsignals, AM824_MAX_SIGNALS);
        return AVERROR(EINVAL);
    }

    ctx->cur_signal = 0;
    ctx->pts = 0;
    ctx->probed = 0;

    for (i = 0; i < ctx->nsignals; i++) {
        ctx->signal_types[i] = AM824_SIG_UNKNOWN;
        memset(&ctx->s337m_accum[i], 0, sizeof(ctx->s337m_accum[i]));
    }

    /* If am824_chorder is provided, parse it to pre-set signal types */
    if (ctx->am824_chorder)
        parse_am824_chorder(ctx, ctx->am824_chorder);

    /* Pre-allocate read buffer */
    ctx->read_buf_size = 0;
    ctx->read_buf = NULL;

    /* Probe V bits and S337M codec types from the data */
    ret = am824_probe_signals(s);
    if (ret < 0)
        return ret;

    /* Create one output stream per AES3 signal pair with the correct codec */
    for (i = 0; i < ctx->nsignals; i++) {
        AVStream *st = avformat_new_stream(s, NULL);
        if (!st)
            return AVERROR(ENOMEM);

        st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
        st->codecpar->sample_rate = ctx->sample_rate;
        st->codecpar->ch_layout   = (AVChannelLayout)AV_CHANNEL_LAYOUT_STEREO;

        if (ctx->signal_types[i] == AM824_SIG_NONPCM &&
            ctx->s337m_accum[i].codec_id != AV_CODEC_ID_NONE) {
            /* Non-PCM: use the detected compressed codec */
            st->codecpar->codec_id = ctx->s337m_accum[i].codec_id;
            av_log(s, AV_LOG_INFO,
                   "AM824: stream %d → %s (non-PCM/S337M)\n",
                   i, avcodec_get_name(st->codecpar->codec_id));
        } else {
            /* PCM: output as S32BE */
            st->codecpar->codec_id = AV_CODEC_ID_PCM_S32BE;
            st->codecpar->bits_per_coded_sample = 32;
            st->codecpar->bits_per_raw_sample   = 24;
        }

        avpriv_set_pts_info(st, 64, 1, ctx->sample_rate);
    }

    return 0;
}

/**
 * Try to extract a complete S337M burst from the accumulator for the given signal.
 * Returns 1 if a burst packet was produced, 0 if not enough data yet.
 */
static int try_extract_s337m_burst(AVFormatContext *s, int sig, AVPacket *pkt)
{
    AM824DemuxContext *ctx = s->priv_data;
    S337MAccum *acc = &ctx->s337m_accum[sig];
    int w = 0;
    int ret;

    while (w + 3 < acc->count) {
        if (acc->words[w] == 0xF872 && acc->words[w+1] == 0x4E1F) {
            int pc = acc->words[w+2];
            int pd = acc->words[w+3];
            int data_type = pc & 0x1F;
            int length_bits = pd;
            int payload_bytes = (length_bits + 7) / 8;
            int payload_words = (payload_bytes + 1) / 2;
            int consumed, pi;

            if (w + 4 + payload_words > acc->count) {
                /* Discard any non-sync bytes before the burst start */
                if (w > 0) {
                    memmove(acc->words, acc->words + w,
                            (acc->count - w) * sizeof(uint16_t));
                    acc->count -= w;
                }
                return 0; /* Incomplete burst */
            }

            if (s337m_data_type_to_codec(data_type) == AV_CODEC_ID_NONE) {
                w += 4 + payload_words;
                continue;
            }

            /* Extract compressed payload */
            ret = av_new_packet(pkt, payload_bytes);
            if (ret < 0) return ret;
            for (pi = 0; pi < payload_words; pi++) {
                uint16_t pw = acc->words[w + 4 + pi];
                if (pi * 2 + 1 < payload_bytes) {
                    pkt->data[pi * 2]     = (pw >> 8) & 0xFF;
                    pkt->data[pi * 2 + 1] = pw & 0xFF;
                } else if (pi * 2 < payload_bytes) {
                    pkt->data[pi * 2] = (pw >> 8) & 0xFF;
                }
            }
            pkt->stream_index = sig;
            pkt->pts = ctx->pts;
            pkt->dts = ctx->pts;

            /* Consume burst + trailing zeros (padding) */
            consumed = w + 4 + payload_words;
            while (consumed < acc->count && acc->words[consumed] == 0)
                consumed++;
            if (consumed < acc->count)
                memmove(acc->words, acc->words + consumed,
                        (acc->count - consumed) * sizeof(uint16_t));
            acc->count -= consumed;
            return 1;
        }
        w++;
    }

    /* Discard any non-sync data at the start */
    if (w > 0 && w < acc->count) {
        memmove(acc->words, acc->words + w,
                (acc->count - w) * sizeof(uint16_t));
        acc->count -= w;
    }
    return 0;
}

/**
 * Read one chunk of interleaved AM824 data and deinterleave ALL signals.
 * PCM signals get buffered packets; non-PCM signals get accumulated words.
 */
static int am824_load_chunk(AVFormatContext *s)
{
    AM824DemuxContext *ctx = s->priv_data;
    int nchan = ctx->nchan;
    int periods, total_subframes, total_bytes;
    int ret, p, sig, ch;

    if (ctx->sample_rate == 96000)
        periods = 96;
    else
        periods = 48;

    total_subframes = periods * nchan;
    total_bytes = total_subframes * AM824_SUBFRAME_SIZE;

    /* Grow read buffer if needed */
    if (total_bytes > ctx->read_buf_size) {
        av_free(ctx->read_buf);
        ctx->read_buf = av_malloc(total_bytes);
        if (!ctx->read_buf) {
            ctx->read_buf_size = 0;
            return AVERROR(ENOMEM);
        }
        ctx->read_buf_size = total_bytes;
    }

    ret = avio_read(s->pb, ctx->read_buf, total_bytes);
    if (ret <= 0)
        return ret < 0 ? ret : AVERROR_EOF;

    total_subframes = ret / AM824_SUBFRAME_SIZE;
    if (total_subframes < nchan)
        return AVERROR_EOF;
    periods = total_subframes / nchan;

    /* Deinterleave ALL signals from this chunk */
    for (sig = 0; sig < ctx->nsignals; sig++) {
        if (ctx->signal_types[sig] == AM824_SIG_PCM) {
            /* PCM: create a buffered packet with S32BE stereo */
            AVPacket *pp;
            if (ctx->pcm_pending[sig])
                av_packet_free(&ctx->pcm_pending[sig]);
            pp = av_packet_alloc();
            if (!pp) return AVERROR(ENOMEM);
            ret = av_new_packet(pp, periods * 2 * 4);
            if (ret < 0) { av_packet_free(&pp); return ret; }
            pp->stream_index = sig;
            pp->pts = ctx->pts;
            pp->dts = ctx->pts;

            for (p = 0; p < periods; p++) {
                int src_base = (p * nchan + sig * 2) * AM824_SUBFRAME_SIZE;
                int dst_base = p * 2 * 4;
                for (ch = 0; ch < 2; ch++) {
                    uint32_t word = AV_RB32(ctx->read_buf + src_base + ch * AM824_SUBFRAME_SIZE);
                    uint32_t data24 = word & AM824_DATA24_MASK;
                    int32_t sample = ((int32_t)data24) << 8;
                    AV_WB32(pp->data + dst_base + ch * 4, sample);
                }
            }
            ctx->pcm_pending[sig] = pp;
        } else {
            /* Non-PCM: accumulate 16-bit SPDIF words from DATA24 */
            S337MAccum *acc = &ctx->s337m_accum[sig];
            int needed = acc->count + periods * 2;
            if (needed > acc->capacity) {
                int new_cap = FFMAX(needed, acc->capacity * 2);
                if (new_cap < 8192) new_cap = 8192;
                acc->words = av_realloc(acc->words, new_cap * sizeof(uint16_t));
                if (!acc->words) { acc->capacity = 0; return AVERROR(ENOMEM); }
                acc->capacity = new_cap;
            }
            for (p = 0; p < periods; p++) {
                int src_base = (p * nchan + sig * 2) * AM824_SUBFRAME_SIZE;
                for (ch = 0; ch < 2; ch++) {
                    uint32_t word = AV_RB32(ctx->read_buf + src_base + ch * AM824_SUBFRAME_SIZE);
                    uint32_t data24 = word & AM824_DATA24_MASK;
                    acc->words[acc->count++] = (uint16_t)(data24 >> 8);
                }
            }
        }
    }

    ctx->pts += periods;
    ctx->chunk_loaded = 1;
    return 0;
}

/**
 * Read and emit packets, draining PCM buffers eagerly before loading
 * new chunks. Non-PCM signals accumulate words across multiple chunks
 * until a complete S337M burst is found.
 *
 * Emit order: PCM packets first (one per signal per chunk), then
 * S337M burst packets when complete. This avoids losing PCM data
 * while accumulating for non-PCM bursts.
 */
static int am824_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    AM824DemuxContext *ctx = s->priv_data;
    int sig, ret;

    for (;;) {
        /* Phase 1: Drain any pending PCM packets (from prior chunk load) */
        for (sig = 0; sig < ctx->nsignals; sig++) {
            if (ctx->pcm_pending[sig]) {
                av_packet_move_ref(pkt, ctx->pcm_pending[sig]);
                av_packet_free(&ctx->pcm_pending[sig]);
                return 0;
            }
        }

        /* Phase 2: Try to extract a complete S337M burst from any non-PCM signal */
        for (sig = 0; sig < ctx->nsignals; sig++) {
            if (ctx->signal_types[sig] == AM824_SIG_NONPCM) {
                ret = try_extract_s337m_burst(s, sig, pkt);
                if (ret > 0)
                    return 0;
                if (ret < 0)
                    return ret;
            }
        }

        /* Phase 3: Load another chunk — this creates PCM pending packets
         * and accumulates non-PCM words for all signals simultaneously. */
        ret = am824_load_chunk(s);
        if (ret < 0)
            return ret;
        /* Loop back to phase 1 to drain newly-loaded PCM packets */
    }
}

static int am824_read_close(AVFormatContext *s)
{
    AM824DemuxContext *ctx = s->priv_data;
    int i;
    for (i = 0; i < ctx->nsignals; i++) {
        av_freep(&ctx->s337m_accum[i].words);
        av_packet_free(&ctx->pcm_pending[i]);
    }
    av_freep(&ctx->read_buf);
    ctx->read_buf_size = 0;
    return 0;
}

#define DOFFSET(x) offsetof(AM824DemuxContext, x)
#define DEC AV_OPT_FLAG_DECODING_PARAM

static const AVOption am824_demux_options[] = {
    { "nchan", "Number of AES3 subframe sequences (always even)",
      DOFFSET(nchan), AV_OPT_TYPE_INT, { .i64 = 2 }, 2, 80, DEC },
    { "sample_rate", "Sampling frequency (Hz)",
      DOFFSET(sample_rate), AV_OPT_TYPE_INT, { .i64 = 48000 }, 44100, 96000, DEC },
    { "am824_chorder", "Channel order string e.g. SMPTE2110.(ST,AES3)",
      DOFFSET(am824_chorder), AV_OPT_TYPE_STRING, { .str = NULL }, 0, 0, DEC },
    { NULL }
};

static const AVClass am824_demux_class = {
    .class_name = "AM824 demuxer",
    .item_name  = av_default_item_name,
    .option     = am824_demux_options,
    .version    = LIBAVUTIL_VERSION_INT,
};

const FFInputFormat ff_am824_demuxer = {
    .p.name         = "am824",
    .p.long_name    = NULL_IF_CONFIG_SMALL("SMPTE ST 2110-31 AM824 AES3 Transparent Transport"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.priv_class   = &am824_demux_class,
    .priv_data_size = sizeof(AM824DemuxContext),
    .read_header    = am824_read_header,
    .read_packet    = am824_read_packet,
    .read_close     = am824_read_close,
};

/* =====================================================================
 *  MUXER
 * ===================================================================== */

typedef struct AM824MuxContext {
    AVClass *class;
    int nsignals;               /**< Number of input streams = signal pairs */
    int sample_rate;
    int *is_nonpcm;             /**< Per-stream: 1 if non-PCM/SPDIF */
    int am824_passthrough;      /**< 1 if input is already AM824 (just write raw bytes) */
    /* Buffering for true interleaving: one pending frame per stream */
    AVPacket **pending;         /**< One buffered packet per stream */
    int pending_ready;          /**< Bitmask: which streams have pending data */
    /* RTP output state (when output URL is rtp://) */
    AVFormatContext *rtp_ctx;   /**< Chained RTP muxer context, or NULL for file output */
    int64_t rtp_ts;             /**< Running RTP timestamp */
} AM824MuxContext;

static int am824_write_header(AVFormatContext *s)
{
    AM824MuxContext *ctx = s->priv_data;
    int i;

    ctx->nsignals = s->nb_streams;
    if (ctx->nsignals < 1 || ctx->nsignals > AM824_MAX_SIGNALS) {
        av_log(s, AV_LOG_ERROR,
               "AM824 muxer: need 1-%d input streams, got %d\n",
               AM824_MAX_SIGNALS, ctx->nsignals);
        return AVERROR(EINVAL);
    }

    ctx->is_nonpcm = av_calloc(ctx->nsignals, sizeof(*ctx->is_nonpcm));
    ctx->pending = av_calloc(ctx->nsignals, sizeof(*ctx->pending));
    if (!ctx->is_nonpcm || !ctx->pending)
        return AVERROR(ENOMEM);

    ctx->pending_ready = 0;

    for (i = 0; i < ctx->nsignals; i++) {
        AVStream *st = s->streams[i];
        if (st->codecpar->codec_type != AVMEDIA_TYPE_AUDIO) {
            av_log(s, AV_LOG_ERROR,
                   "AM824 muxer: stream %d is not audio\n", i);
            return AVERROR(EINVAL);
        }
        /* Detect input codec type */
        if (st->codecpar->codec_id == AV_CODEC_ID_AM824) {
            /* Input is already AM824-encoded: passthrough mode.
             * Just write the raw AM824 bytes to output. */
            ctx->am824_passthrough = 1;
        }
        /* Detect non-PCM: compressed codecs need S337M framing,
         * S302M/SPDIF-tagged streams are already framed. */
        switch (st->codecpar->codec_id) {
        case AV_CODEC_ID_AC3:
        case AV_CODEC_ID_AAC:
        case AV_CODEC_ID_EAC3:
        case AV_CODEC_ID_DTS:
        case AV_CODEC_ID_S302M:
            ctx->is_nonpcm[i] = 1;
            break;
        default:
            if (st->codecpar->codec_tag == MKTAG('s','p','d','f'))
                ctx->is_nonpcm[i] = 1;
            break;
        }

        if (ctx->sample_rate == 0)
            ctx->sample_rate = st->codecpar->sample_rate;

        ctx->pending[i] = NULL;
    }

    if (ctx->sample_rate == 0)
        ctx->sample_rate = 48000;

    ctx->rtp_ctx = NULL;
    ctx->rtp_ts = 0;

    /* If output URL is rtp://, chain through the RTP muxer.
     * Create a virtual AM824 output stream for the RTP chain. */
    if (s->url && av_stristart(s->url, "rtp://", NULL)) {
        AVFormatContext *rtpctx = NULL;
        URLContext *handle = NULL;
        AVStream *rtp_st;
        int nchan = ctx->nsignals * 2;
        int ret2;

        /* Open the RTP URL */
        ret2 = ffurl_open_whitelist(&handle, s->url, AVIO_FLAG_WRITE,
                                    &s->interrupt_callback, NULL,
                                    s->protocol_whitelist,
                                    s->protocol_blacklist, NULL);
        if (ret2 < 0) {
            av_log(s, AV_LOG_ERROR, "AM824: cannot open RTP URL %s\n", s->url);
            return ret2;
        }

        /* Create a temporary AVStream with AM824 codec parameters for the
         * RTP chain. ff_rtp_chain_mux_open copies the codecpar, so we can
         * use a temporary stream without adding it to the parent context.
         * We use s->streams[0] as a base and override the codec params. */
        {
            /* Create a temporary format context just to hold the virtual stream */
            AVFormatContext *tmpctx = avformat_alloc_context();
            if (!tmpctx) {
                ffurl_close(handle);
                return AVERROR(ENOMEM);
            }
            rtp_st = avformat_new_stream(tmpctx, NULL);
            if (!rtp_st) {
                avformat_free_context(tmpctx);
                ffurl_close(handle);
                return AVERROR(ENOMEM);
            }
            rtp_st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
            rtp_st->codecpar->codec_id = AV_CODEC_ID_AM824;
            rtp_st->codecpar->sample_rate = ctx->sample_rate;
            av_channel_layout_default(&rtp_st->codecpar->ch_layout, nchan);
            rtp_st->codecpar->bits_per_coded_sample = 32;
            rtp_st->codecpar->bits_per_raw_sample = 24;
            rtp_st->time_base = (AVRational){1, ctx->sample_rate};

            ret2 = ff_rtp_chain_mux_open(&rtpctx, s, rtp_st, handle, 1500, 0);
            avformat_free_context(tmpctx);
        }
        if (ret2 < 0) {
            av_log(s, AV_LOG_ERROR, "AM824: cannot open RTP chain\n");
            return ret2;
        }

        ctx->rtp_ctx = rtpctx;
        av_log(s, AV_LOG_INFO,
               "AM824: RTP output to %s (nchan=%d, rate=%d)\n",
               s->url, nchan, ctx->sample_rate);
    }

    return 0;
}

/**
 * Flush all pending packets into one interleaved AM824 output.
 * True sample-by-sample interleaving per ST 2110-31 Section 5.4:
 * For each sample period, output all signal pairs' L/R subframes.
 *
 * Input packets are S32 stereo (interleaved L,R,L,R,...).
 * Output: AM824 32-bit words [s0L,s0R, s1L,s1R, ..., sN-1L,sN-1R, s0L,s0R, ...]
 */
static int flush_interleaved(AVFormatContext *s)
{
    AM824MuxContext *ctx = s->priv_data;
    int nsig = ctx->nsignals;
    int min_samples = INT_MAX;
    int nchan = nsig * 2;
    int total_subframes, out_size;
    uint8_t *out;
    int sig, p, ch;

    /* Find minimum sample count across all pending packets */
    for (sig = 0; sig < nsig; sig++) {
        AVPacket *pend = ctx->pending[sig];
        int nsamples;
        if (!pend)
            return 0; /* not all streams ready yet */
        nsamples = pend->size / (2 * 4); /* stereo S32 = 8 bytes per sample period */
        if (nsamples < min_samples)
            min_samples = nsamples;
    }

    if (min_samples <= 0)
        return 0;

    total_subframes = min_samples * nchan;
    out_size = total_subframes * AM824_SUBFRAME_SIZE;

    out = av_malloc(out_size);
    if (!out)
        return AVERROR(ENOMEM);

    /* Interleave: for each sample period, write all signals' L/R */
    for (p = 0; p < min_samples; p++) {
        for (sig = 0; sig < nsig; sig++) {
            const uint8_t *src = ctx->pending[sig]->data + p * 2 * 4;
            int out_base = (p * nchan + sig * 2) * AM824_SUBFRAME_SIZE;

            for (ch = 0; ch < 2; ch++) {
                int32_t sample = (int32_t)AV_RB32(src + ch * 4);
                uint32_t data24 = ((uint32_t)(sample >> 8)) & AM824_DATA24_MASK;
                uint32_t word = data24;

                /* V bit for non-PCM signals */
                if (ctx->is_nonpcm[sig])
                    word |= ((uint32_t)AM824_V_BIT << AM824_PCUV_SHIFT);

                /* F (frame start) for left channel */
                if (ch == 0)
                    word |= (1U << 28);

                AV_WB32(out + out_base + ch * AM824_SUBFRAME_SIZE, word);
            }
        }
    }

    if (ctx->rtp_ctx) {
        /* Send interleaved AM824 data through the chained RTP muxer.
         * The RTP muxer dispatches to ff_rtp_send_am824() which handles
         * ptime-aligned packetization and RTP header generation. */
        AVPacket *rtp_pkt = av_packet_alloc();
        if (!rtp_pkt) {
            av_free(out);
            return AVERROR(ENOMEM);
        }
        rtp_pkt->data = out;
        rtp_pkt->size = out_size;
        rtp_pkt->stream_index = 0;
        rtp_pkt->pts = ctx->rtp_ts;
        rtp_pkt->dts = ctx->rtp_ts;
        ctx->rtp_ts += min_samples;

        av_interleaved_write_frame(ctx->rtp_ctx, rtp_pkt);
        /* Note: av_interleaved_write_frame does NOT take ownership of data
         * for non-refcounted packets, so we still free out below. */
        av_packet_free(&rtp_pkt);
    } else {
        /* Write raw AM824 data to file/pipe */
        avio_write(s->pb, out, out_size);
    }
    av_free(out);

    /* Consume min_samples from each pending packet.
     * If a packet had more samples, keep the remainder for the next flush. */
    for (sig = 0; sig < nsig; sig++) {
        AVPacket *pend = ctx->pending[sig];
        int pend_samples = pend->size / (2 * 4);
        int consumed_bytes = min_samples * 2 * 4;

        if (pend_samples > min_samples) {
            /* Shift remaining data to front of packet */
            int remaining = pend->size - consumed_bytes;
            memmove(pend->data, pend->data + consumed_bytes, remaining);
            pend->size = remaining;
        } else {
            av_packet_free(&ctx->pending[sig]);
            ctx->pending_ready &= ~(1 << sig);
        }
    }

    return 0;
}

/**
 * Apply S337M/IEC 61937 burst framing to a compressed audio packet,
 * producing an S32 stereo packet that can be treated as PCM by the
 * AM824 interleaver.
 *
 * Returns a new AVPacket with S32BE stereo data, or NULL on error.
 * Caller must free the returned packet.
 */
static AVPacket *s337m_wrap_compressed(AVFormatContext *s, AVPacket *pkt,
                                       int codec_id, int sample_rate)
{
    int32_t *buf;
    int capacity, n_samples;
    AVPacket *out;

    /* Maximum burst size: 24576 bytes for EAC3 → 6144 stereo S32 samples */
    capacity = 24576 / 2; /* max stereo samples */
    buf = av_calloc(capacity, sizeof(int32_t));
    if (!buf)
        return NULL;

    n_samples = ff_am824_s337m_frame(codec_id, pkt->data, pkt->size,
                                     buf, capacity, sample_rate);
    if (n_samples <= 0) {
        av_free(buf);
        av_log(s, AV_LOG_ERROR,
               "AM824: S337M framing failed for codec %d, size %d\n",
               codec_id, pkt->size);
        return NULL;
    }

    out = av_packet_alloc();
    if (!out) {
        av_free(buf);
        return NULL;
    }

    /* Convert S32 native-endian to S32BE for the AM824 interleaver */
    if (av_new_packet(out, n_samples * 4) < 0) {
        av_packet_free(&out);
        av_free(buf);
        return NULL;
    }

    {
        int i;
        for (i = 0; i < n_samples; i++)
            AV_WB32(out->data + i * 4, buf[i]);
    }

    out->stream_index = pkt->stream_index;
    out->pts = pkt->pts;
    out->dts = pkt->dts;

    av_free(buf);
    return out;
}

/**
 * Accept a packet from one input stream. Buffer it until all streams
 * have contributed data for this time slice, then flush interleaved.
 *
 * For compressed audio streams (AAC, AC-3, etc.), applies S337M/IEC 61937
 * burst framing before buffering, so the interleaver sees S32 stereo data
 * with S337M sync words embedded in the sample values.
 */
static int am824_write_packet(AVFormatContext *s, AVPacket *pkt)
{
    AM824MuxContext *ctx = s->priv_data;
    int idx = pkt->stream_index;
    int ret;

    if (idx < 0 || idx >= ctx->nsignals)
        return AVERROR(EINVAL);

    /* AM824 codec passthrough: write raw AM824 bytes directly */
    if (ctx->am824_passthrough) {
        avio_write(s->pb, pkt->data, pkt->size);
        return 0;
    }

    /* For compressed audio: apply S337M framing first */
    if (ctx->is_nonpcm[idx]) {
        AVStream *st = s->streams[idx];
        enum AVCodecID cid = st->codecpar->codec_id;

        /* Only apply S337M framing for known compressed codecs.
         * S302M and SPDIF-tagged streams are already framed. */
        if (cid == AV_CODEC_ID_AAC || cid == AV_CODEC_ID_AC3 ||
            cid == AV_CODEC_ID_EAC3 || cid == AV_CODEC_ID_DTS) {
            AVPacket *framed;
            /* AAC over S337M requires ADTS framing (IEC 61937-6, data type 0x07).
             * Reject raw AAC (from MP4/MKV) that lacks ADTS headers. */
            if (cid == AV_CODEC_ID_AAC && pkt->size >= 2 &&
                (AV_RB16(pkt->data) & 0xFFF0) != 0xFFF0) {
                av_log(s, AV_LOG_ERROR,
                       "AM824: AAC stream %d is not ADTS-framed (got 0x%04x, need 0xFFF_).\n"
                       "  Use ADTS input (-f adts) or re-encode (-c:a aac) instead of -c:a copy from MP4.\n",
                       idx, AV_RB16(pkt->data));
                return AVERROR(EINVAL);
            }
            framed = s337m_wrap_compressed(
                s, pkt, cid, ctx->sample_rate ? ctx->sample_rate : 48000);
            if (!framed)
                return AVERROR(ENOMEM);

            if (ctx->pending[idx] && ctx->pending[idx]->size > 0) {
                /* Append S337M-framed data to existing leftover */
                AVPacket *merged = av_packet_alloc();
                int old_size = ctx->pending[idx]->size;
                int new_size = framed->size;
                if (!merged) { av_packet_free(&framed); return AVERROR(ENOMEM); }
                if (av_new_packet(merged, old_size + new_size) < 0) {
                    av_packet_free(&merged); av_packet_free(&framed);
                    return AVERROR(ENOMEM);
                }
                memcpy(merged->data, ctx->pending[idx]->data, old_size);
                memcpy(merged->data + old_size, framed->data, new_size);
                merged->stream_index = framed->stream_index;
                merged->pts = ctx->pending[idx]->pts;
                merged->dts = ctx->pending[idx]->dts;
                av_packet_free(&ctx->pending[idx]);
                av_packet_free(&framed);
                ctx->pending[idx] = merged;
            } else {
                if (ctx->pending[idx])
                    av_packet_free(&ctx->pending[idx]);
                ctx->pending[idx] = framed;
            }
            ctx->pending_ready |= (1 << idx);
            goto check_flush;
        }
    }

    /* Buffer this stream's packet (PCM or pre-framed non-PCM).
     * If there's leftover data from a previous partial flush, append. */
    if (ctx->pending[idx] && ctx->pending[idx]->size > 0) {
        /* Append new data to existing leftover */
        AVPacket *merged = av_packet_alloc();
        int old_size = ctx->pending[idx]->size;
        int new_size = pkt->size;
        if (!merged)
            return AVERROR(ENOMEM);
        if (av_new_packet(merged, old_size + new_size) < 0) {
            av_packet_free(&merged);
            return AVERROR(ENOMEM);
        }
        memcpy(merged->data, ctx->pending[idx]->data, old_size);
        memcpy(merged->data + old_size, pkt->data, new_size);
        merged->stream_index = pkt->stream_index;
        merged->pts = ctx->pending[idx]->pts;
        merged->dts = ctx->pending[idx]->dts;
        av_packet_free(&ctx->pending[idx]);
        ctx->pending[idx] = merged;
    } else {
        if (ctx->pending[idx])
            av_packet_free(&ctx->pending[idx]);
        ctx->pending[idx] = av_packet_clone(pkt);
        if (!ctx->pending[idx])
            return AVERROR(ENOMEM);
    }

    ctx->pending_ready |= (1 << idx);

check_flush:
    /* If single-stream, flush immediately (no interleaving needed) */
    if (ctx->nsignals == 1) {
        ret = flush_interleaved(s);
        return ret;
    }

    /* For multi-stream: flush when all streams have pending data */
    if (ctx->pending_ready == (1 << ctx->nsignals) - 1) {
        ret = flush_interleaved(s);
        return ret;
    }

    return 0;
}

static int am824_write_trailer(AVFormatContext *s)
{
    AM824MuxContext *ctx = s->priv_data;
    int i;

    /* Flush any remaining buffered data */
    flush_interleaved(s);

    /* Close the chained RTP muxer if active */
    if (ctx->rtp_ctx) {
        av_write_trailer(ctx->rtp_ctx);
        avio_closep(&ctx->rtp_ctx->pb);
        avformat_free_context(ctx->rtp_ctx);
        ctx->rtp_ctx = NULL;
    }

    for (i = 0; i < ctx->nsignals; i++)
        av_packet_free(&ctx->pending[i]);
    av_freep(&ctx->pending);
    av_freep(&ctx->is_nonpcm);
    return 0;
}

#define MOFFSET(x) offsetof(AM824MuxContext, x)
#define ENC AV_OPT_FLAG_ENCODING_PARAM

static const AVOption am824_mux_options[] = {
    { "sample_rate", "Sampling frequency (Hz)",
      MOFFSET(sample_rate), AV_OPT_TYPE_INT, { .i64 = 0 }, 0, 96000, ENC },
    { NULL }
};

static const AVClass am824_mux_class = {
    .class_name = "AM824 muxer",
    .item_name  = av_default_item_name,
    .option     = am824_mux_options,
    .version    = LIBAVUTIL_VERSION_INT,
};

const FFOutputFormat ff_am824_muxer = {
    .p.name         = "am824",
    .p.long_name    = NULL_IF_CONFIG_SMALL("SMPTE ST 2110-31 AM824 AES3 Transparent Transport"),
    .p.audio_codec  = AV_CODEC_ID_AM824,
    .p.flags        = AVFMT_NOTIMESTAMPS,
    .p.priv_class   = &am824_mux_class,
    .priv_data_size = sizeof(AM824MuxContext),
    .write_header   = am824_write_header,
    .write_packet   = am824_write_packet,
    .write_trailer  = am824_write_trailer,
};
