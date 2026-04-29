/*
 * MPEG-2 transport stream (aka DVB) muxer
 * Copyright (c) 2003 Fabrice Bellard
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

#include "libavutil/avassert.h"
#include "libavutil/bswap.h"
#include "libavutil/crc.h"
#include "libavutil/dict.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/mathematics.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/time.h"

/* Architecture-specific spin hint for busy-wait pacing loops.
 * On x86: PAUSE instruction reduces power and avoids memory-order violations.
 * On ARM: YIELD hints the core to switch to another hardware thread.
 * Elsewhere: no-op. */
#if defined(__x86_64__) || defined(__i386__)
#include <immintrin.h>
#define spin_hint() _mm_pause()
#elif defined(__aarch64__)
#define spin_hint() __asm__ volatile("yield" ::: "memory")
#else
#define spin_hint() ((void)0)
#endif

/* CBR pacing: busy-spin the last N microseconds for sub-ms precision.
 * Coarse sleep (av_usleep) handles the bulk of the wait, then the
 * spin loop provides precise wakeup at the target time. */
#define CBR_PACE_SPIN_US  500   /* spin last 0.5 ms */
#define CBR_PACE_MAX_EARLY_US 500000  /* ignore target if > 0.5s in future */

#include <stdatomic.h>
#include <stdio.h>
#include <pthread.h>

#include "libavcodec/ac3_parser_internal.h"
#include "libavcodec/bytestream.h"
#include "libavcodec/defs.h"
#include "libavcodec/h264.h"
#include "libavcodec/hevc/hevc.h"
#include "libavcodec/vvc.h"
#include "libavcodec/startcode.h"

#include "avc.h"
#include "avformat.h"
#include "avio_internal.h"
#include "internal.h"
#include "mpegts.h"
#include "mux.h"
#include "nal.h"
#include "rtp_pep.h"

/* ------------------------------------------------------------------
 * Threaded output: a single dedicated thread drains a lockless SPSC
 * ring buffer at strict mux_rate (one packet per packet-duration).
 * The mux thread enqueues packets into the ring without pacing, so
 * that scheduler stalls in the mux thread are absorbed by the ring
 * rather than creating on-wire bursts when the mux thread catches up.
 *
 * The ring is a classical single-producer / single-consumer lockless
 * FIFO: mux thread owns tail, output thread owns head.  No mutex, no
 * condvar — both threads make forward progress without ever entering
 * the kernel (except for av_usleep in the pacer, which the output
 * thread avoids by busy-spinning).  This removes the ~µs-per-emit
 * scheduler disturbance that pthread_mutex/cond_wait introduced.
 *
 * The output thread is the sole writer to:
 *   - s->pb (avio)
 *   - ts->interleave_output_bytes (used for PCR rewrite)
 *   - ts->out_last_emit_us (the pacing clock)
 *
 * Capacity must be a power of two (index masking).  MPEGTS_OUT_RING_CAP
 * = 4096 absorbs ~260 ms of mux-thread stall at 23.8 Mbps.  Memory
 * cost: ~770 kB.
 * ------------------------------------------------------------------ */
#define MPEGTS_OUT_RING_CAP  4096
#define MPEGTS_OUT_RING_MASK (MPEGTS_OUT_RING_CAP - 1)

typedef struct MpegTSOutRing {
    uint8_t packets[MPEGTS_OUT_RING_CAP][TS_PACKET_SIZE];
    _Atomic int head;        /* next index to dequeue (owned: consumer) */
    _Atomic int tail;        /* next index to enqueue (owned: producer) */
    _Atomic int shutdown;    /* 1 = mux thread has signalled end-of-stream */
    /* Diagnostics (written by either thread; only read at end). */
    _Atomic int64_t total_emits;
    _Atomic int64_t late_emits;
    _Atomic int64_t producer_full_spins;
    _Atomic int64_t consumer_empty_spins;
    _Atomic int max_depth;
} MpegTSOutRing;

#define PCR_TIME_BASE 27000000

/* write DVB SI sections */

#define DVB_PRIVATE_NETWORK_START 0xff01

/*********************************************/
/* mpegts section writer */

typedef struct MpegTSSection {
    int pid;
    int cc;
    int discontinuity;
    void (*write_packet)(struct MpegTSSection *s, const uint8_t *packet);
    void *opaque;
    int remaining;
} MpegTSSection;

typedef struct MpegTSService {
    MpegTSSection pmt; /* MPEG-2 PMT table context */
    int sid;           /* service ID */
    uint8_t name[256];
    uint8_t provider_name[256];
    int pcr_pid;
    AVProgram *program;
} MpegTSService;

// service_type values as defined in ETSI 300 468
enum {
    MPEGTS_SERVICE_TYPE_DIGITAL_TV                   = 0x01,
    MPEGTS_SERVICE_TYPE_DIGITAL_RADIO                = 0x02,
    MPEGTS_SERVICE_TYPE_TELETEXT                     = 0x03,
    MPEGTS_SERVICE_TYPE_ADVANCED_CODEC_DIGITAL_RADIO = 0x0A,
    MPEGTS_SERVICE_TYPE_MPEG2_DIGITAL_HDTV           = 0x11,
    MPEGTS_SERVICE_TYPE_ADVANCED_CODEC_DIGITAL_SDTV  = 0x16,
    MPEGTS_SERVICE_TYPE_ADVANCED_CODEC_DIGITAL_HDTV  = 0x19,
    MPEGTS_SERVICE_TYPE_HEVC_DIGITAL_HDTV            = 0x1F,
};
typedef struct MpegTSWrite {
    const AVClass *av_class;
    MpegTSSection pat; /* MPEG-2 PAT table */
    MpegTSSection sdt; /* MPEG-2 SDT table context */
    MpegTSSection nit; /* MPEG-2 NIT table context */
    MpegTSService **services;
    AVPacket *pkt;
    int64_t sdt_period; /* SDT period in PCR time base */
    int64_t pat_period; /* PAT/PMT period in PCR time base */
    int64_t nit_period; /* NIT period in PCR time base */
    int nb_services;
    int64_t first_pcr;
    int first_dts_checked;
    int64_t next_pcr;
    int mux_rate; ///< set to 1 when VBR
    int pes_payload_size;
    int64_t total_size;

    int transport_stream_id;
    int original_network_id;
    int service_id;
    int service_type;

    int pmt_start_pid;
    int start_pid;
    int m2ts_mode;
    int m2ts_video_pid;
    int m2ts_audio_pid;
    int m2ts_pgssub_pid;
    int m2ts_textsub_pid;

    int pcr_period_ms;
#define MPEGTS_FLAG_REEMIT_PAT_PMT  0x01
#define MPEGTS_FLAG_AAC_LATM        0x02
#define MPEGTS_FLAG_PAT_PMT_AT_FRAMES           0x04
#define MPEGTS_FLAG_SYSTEM_B        0x08
#define MPEGTS_FLAG_DISCONT         0x10
#define MPEGTS_FLAG_NIT             0x20
#define MPEGTS_FLAG_OMIT_RAI        0x40
    int flags;
    int copyts;
    int tables_version;
    int64_t pat_period_us;
    int64_t sdt_period_us;
    int64_t nit_period_us;
    int64_t pat_min_gap_us;
    int64_t pat_min_gap;   /* in PCR time base, derived from pat_min_gap_us */
    int64_t last_pat_ts;
    int64_t last_sdt_ts;
    int64_t last_nit_ts;

    uint8_t provider_name[256];

    int omit_video_pes_length;

    /* CBR output pacing: when enabled, write_packet() sleeps to maintain
     * constant bitrate output. This prevents burst delivery on UDP.
     * The pacing target is derived from total_size (the same counter that
     * drives PCR), so pacing and PCR are locked — zero drift by construction. */
    int cbr_pacing;             /* 0 = off (default), 1 = enabled */
    int cbr_pacing_threaded;    /* 1 = pace in a dedicated output thread */
    int cbr_pacing_spin;        /* threaded pacer: pure busy-spin instead of sleep+spin */
    int64_t pacing_start_us;    /* wall-clock time when total_size was 0 */

    /* Threaded-output state (used only when cbr_pacing_threaded).
     * The output thread is the sole writer of s->pb,
     * interleave_output_bytes, and out_last_emit_us. */
    MpegTSOutRing *out_ring;
    pthread_t out_thread;
    int out_thread_started;
    int64_t out_last_emit_us;

    /* TS packet interleaving: completes FFmpeg's AVPacket-level interleaving
     * (ff_interleave_packet_per_dts in mux.c) with TS-packet-level
     * distribution.  Without this, mpegts_write_pes() dumps all TS packets
     * for a single PES contiguously (~1000 video packets in a row), violating
     * the H.222.0 T-STD TBn 512-byte transport buffer model. */
    int ts_interleave;          /* 0 = off (chunked), 1 = null-interleave, 2 = full-interleave */
    int ts_interleave_chunk;    /* video TS packets per round (default 7) */
    int interleave_buffering;   /* 1 = inside mpegts_write_pes, buffer packets */
    int64_t interleave_output_bytes; /* bytes written to avio during drain */
    int64_t last_psi_output_bytes;   /* iob at last PSI emission (for TBsys spacing) */

    /* Per-PID interleave buffers — indexed by stream index.
     * SI/null packets use interleave_si_buf. */
    uint8_t *interleave_si_buf;
    int interleave_si_capacity; /* in packets */
    int interleave_si_count;

    /* PEP encryption (TR-10-13) for MPEG2-TS over UDP */
    char *pep_psk;
    char *pep_mode;
    char *pep_iv;
    char *pep_key_generator;
    char *pep_key_version;
    void *pep_context;             /**< RtpPepContext* (NULL = disabled) */
    uint64_t pep_input_ctr;
    uint32_t pep_key_ver;
} MpegTSWrite;

/* a PES packet header is generated every DEFAULT_PES_HEADER_FREQ packets */
#define DEFAULT_PES_HEADER_FREQ  16
#define DEFAULT_PES_PAYLOAD_SIZE ((DEFAULT_PES_HEADER_FREQ - 1) * 184 + 170)

/* The section length is 12 bits. The first 2 are set to 0, the remaining
 * 10 bits should not exceed 1021. */
#define SECTION_LENGTH 1020

/* NOTE: 4 bytes must be left at the end for the crc32 */
static void mpegts_write_section(MpegTSSection *s, uint8_t *buf, int len)
{
    unsigned int crc;
    unsigned char packet[TS_PACKET_SIZE];
    const unsigned char *buf_ptr;
    unsigned char *q;
    int first, b, len1, left;

    crc = av_bswap32(av_crc(av_crc_get_table(AV_CRC_32_IEEE),
                            -1, buf, len - 4));

    buf[len - 4] = (crc >> 24) & 0xff;
    buf[len - 3] = (crc >> 16) & 0xff;
    buf[len - 2] = (crc >>  8) & 0xff;
    buf[len - 1] =  crc        & 0xff;

    /* send each packet */
    buf_ptr = buf;
    while (len > 0) {
        first = buf == buf_ptr;
        q     = packet;
        *q++  = SYNC_BYTE;
        b     = s->pid >> 8;
        if (first)
            b |= 0x40;
        *q++  = b;
        *q++  = s->pid;
        s->cc = s->cc + 1 & 0xf;
        *q++  = 0x10 | s->cc;
        if (s->discontinuity) {
            q[-1] |= 0x20;
            *q++ = 1;
            *q++ = 0x80;
            s->discontinuity = 0;
        }
        if (first)
            *q++ = 0; /* 0 offset */
        len1 = TS_PACKET_SIZE - (q - packet);
        if (len1 > len)
            len1 = len;
        memcpy(q, buf_ptr, len1);
        q += len1;
        /* add known padding data */
        left = TS_PACKET_SIZE - (q - packet);
        if (left > 0)
            memset(q, STUFFING_BYTE, left);

        s->write_packet(s, packet);

        buf_ptr += len1;
        len     -= len1;
    }
}

static inline void put16(uint8_t **q_ptr, int val)
{
    uint8_t *q;
    q      = *q_ptr;
    *q++   = val >> 8;
    *q++   = val;
    *q_ptr = q;
}

static int mpegts_write_section1(MpegTSSection *s, int tid, int id,
                                 int version, int sec_num, int last_sec_num,
                                 uint8_t *buf, int len)
{
    uint8_t section[1024], *q;
    unsigned int tot_len;
    /* reserved_future_use field must be set to 1 for SDT and NIT */
    unsigned int flags = (tid == SDT_TID || tid == NIT_TID) ? 0xf000 : 0xb000;

    tot_len = 3 + 5 + len + 4;
    /* check if not too big */
    if (tot_len > 1024)
        return AVERROR_INVALIDDATA;

    q    = section;
    *q++ = tid;
    put16(&q, flags | (len + 5 + 4)); /* 5 byte header + 4 byte CRC */
    put16(&q, id);
    *q++ = 0xc1 | (version << 1); /* current_next_indicator = 1 */
    *q++ = sec_num;
    *q++ = last_sec_num;
    memcpy(q, buf, len);

    mpegts_write_section(s, section, tot_len);
    return 0;
}

/*********************************************/
/* mpegts writer */

#define DEFAULT_PROVIDER_NAME   "FFmpeg"
#define DEFAULT_SERVICE_NAME    "Service"

/* we retransmit the SI info at this rate */
#define SDT_RETRANS_TIME 500
#define PAT_RETRANS_TIME 100
#define PCR_RETRANS_TIME 20
#define NIT_RETRANS_TIME 500

typedef struct MpegTSWriteStream {
    int pid; /* stream associated pid */
    int cc;
    int discontinuity;
    int payload_size;
    int first_timestamp_checked; ///< first pts/dts check needed
    int prev_payload_key;
    int64_t payload_pts;
    int64_t payload_dts;
    int payload_flags;
    uint8_t *payload;
    AVFormatContext *amux;
    int data_st_warning;

    int64_t pcr_period; /* PCR period in PCR time base */
    int64_t last_pcr;

    /* For Opus */
    int opus_queued_samples;
    int opus_pending_trim_start;

    DVBAC3Descriptor *dvb_ac3_desc;

    /* Per-stream interleave output buffer (ts_interleave mode) */
    uint8_t *interleave_buf;
    int interleave_buf_capacity; /* in packets */
    int interleave_buf_count;
} MpegTSWriteStream;

static void mpegts_write_pat(AVFormatContext *s)
{
    MpegTSWrite *ts = s->priv_data;
    MpegTSService *service;
    uint8_t data[SECTION_LENGTH], *q;
    int i;

    q = data;
    if (ts->flags & MPEGTS_FLAG_NIT) {
        put16(&q, 0x0000);
        put16(&q, NIT_PID);
    }
    for (i = 0; i < ts->nb_services; i++) {
        service = ts->services[i];
        put16(&q, service->sid);
        put16(&q, 0xe000 | service->pmt.pid);
    }
    mpegts_write_section1(&ts->pat, PAT_TID, ts->transport_stream_id, ts->tables_version, 0, 0,
                          data, q - data);
}

static void putbuf(uint8_t **q_ptr, const uint8_t *buf, size_t len)
{
    memcpy(*q_ptr, buf, len);
    *q_ptr += len;
}

static int put_arib_caption_descriptor(AVFormatContext *s, uint8_t **q_ptr,
                                       AVCodecParameters *codecpar)
{
    uint8_t stream_identifier;
    uint16_t data_component_id;
    uint8_t *q = *q_ptr;

    switch (codecpar->profile) {
    case AV_PROFILE_ARIB_PROFILE_A:
        stream_identifier = 0x30;
        data_component_id = 0x0008;
        break;
    case AV_PROFILE_ARIB_PROFILE_C:
        stream_identifier = 0x87;
        data_component_id = 0x0012;
        break;
    default:
        av_log(s, AV_LOG_ERROR,
               "Unset/unknown ARIB caption profile %d utilized!\n",
               codecpar->profile);
        return AVERROR_INVALIDDATA;
    }

    // stream_identifier_descriptor
    *q++ = STREAM_IDENTIFIER_DESCRIPTOR;  // descriptor_tag
    *q++ = 1;     // descriptor_length
    *q++ = stream_identifier;  // component_tag: stream_identifier

    // data_component_descriptor, defined in ARIB STD-B10, part 2, 6.2.20
    *q++ = DATA_COMPONENT_DESCRIPTOR;  // descriptor_tag: ARIB data coding type descriptor
    *q++ = 3;     // descriptor_length
    put16(&q, data_component_id);  // data_component_id
    // additional_arib_caption_info: defined in ARIB STD-B24, fascicle 1, Part 3, 9.6.1
    // Here we utilize a pre-defined set of values defined in ARIB TR-B14,
    // Fascicle 2, 4.2.8.5 for PMT usage, with the reserved bits in the middle
    // set to 1 (as that is what every broadcaster seems to be doing in
    // production).
    *q++ = 0x3D; // DMF('0011'), Reserved('11'), Timing('01')

    *q_ptr = q;

    return 0;
}

/* AAC profile_and_level_indication for ETSI EN 300 468 §6.2.3.2
 * AAC_descriptor (tag 0x7C). Derived from codec profile + channel count +
 * sample rate. Tells DVB-compliant probes the correct T-STD Rxn class so
 * multichannel AAC streams aren't validated against the 2 Mbps stereo
 * baseline. Table mapping per ETSI EN 300 468 Table 110:
 *
 *   AAC Main:          0x18..0x1B (L1..L4..L5)
 *   AAC LC:            0x28..0x2C (L1, L2, L4, L5, L6)
 *   HE-AAC:            0x40..0x44 (L2..L6)
 *   HE-AAC v2 (PS):    0x58..0x59 (L2..L3)
 */

/* Decode AAC profile from the AudioSpecificConfig (ISO/IEC 14496-3 § 1.6.2):
 * the top 5 bits of byte 0 carry audioObjectType (AOT). Maps to the FFmpeg
 * AV_PROFILE_AAC_* constants. Returns AV_PROFILE_UNKNOWN if extradata is too
 * short, the AOT is escape-coded (>= 31, requires 11-bit decode), or the AOT
 * is one we don't represent.
 *
 * Only the common AOTs that have a corresponding AV_PROFILE_AAC_* constant
 * are mapped; anything else falls through to UNKNOWN so the caller can use
 * its own default.
 */
static int aac_profile_from_extradata(const AVCodecParameters *par)
{
    int aot;

    if (!par->extradata || par->extradata_size < 1)
        return AV_PROFILE_UNKNOWN;

    aot = (par->extradata[0] >> 3) & 0x1F;
    switch (aot) {
    case 1:  return AV_PROFILE_AAC_MAIN;
    case 2:  return AV_PROFILE_AAC_LOW;
    case 5:  return AV_PROFILE_AAC_HE;
    case 23: return AV_PROFILE_AAC_LD;
    case 29: return AV_PROFILE_AAC_HE_V2;
    case 39: return AV_PROFILE_AAC_ELD;
    default: return AV_PROFILE_UNKNOWN;
    }
}

static int aac_profile_and_level(const AVCodecParameters *par)
{
    int channels = par->ch_layout.nb_channels;
    int sample_rate = par->sample_rate;
    int profile = par->profile;
    int base, level;

    /* When codecpar->profile is unset (live encode through an encoder that
     * doesn't write avctx->profile back, e.g. libfdk_aac), fall back to the
     * AudioSpecificConfig in extradata. Requires the encoder to publish ASC
     * — set AV_CODEC_FLAG_GLOBAL_HEADER or AV_CODEC_FLAG2_EXTRADATA on the
     * encoder context to get that. If neither path yields a profile, the
     * existing AAC LC default applies. */
    if (profile == AV_PROFILE_UNKNOWN)
        profile = aac_profile_from_extradata(par);

    if (profile == AV_PROFILE_AAC_HE_V2)
        base = 0x58;        /* HE-AAC v2 */
    else if (profile == AV_PROFILE_AAC_HE)
        base = 0x40;        /* HE-AAC */
    else if (profile == AV_PROFILE_AAC_MAIN)
        base = 0x18;        /* AAC Main */
    else
        base = 0x28;        /* AAC LC (default) */

    /* level offset within the profile block:
     *   0 = L1 (≤24 kHz mono)
     *   1 = L2 (≤48 kHz stereo)
     *   2 = L4 (5.1 ≤ 48 kHz)
     *   3 = L5 (5.1 ≤ 96 kHz / higher rate)
     *   4 = L6 (7.1)
     */
    if (channels >= 7)
        level = 4;
    else if (channels >= 3 && sample_rate > 48000)
        level = 3;
    else if (channels >= 3)
        level = 2;
    else if (sample_rate > 24000 || channels > 1)
        level = 1;
    else
        level = 0;

    return base + level;
}

static void put_aac_descriptor(uint8_t **q_ptr, const AVCodecParameters *par)
{
    uint8_t *q = *q_ptr;
    *q++ = AAC_DESCRIPTOR;                  /* tag = 0x7C */
    *q++ = 2;                               /* descriptor_length */
    *q++ = aac_profile_and_level(par);      /* profile_and_level_indication */
    *q++ = 0x00;                            /* AAC_type_flag=0, SAOC_DE_flag=0, reserved */
    *q_ptr = q;
}

/*
 * H.222.0 § 2.6.64 (Table 2-92): AVC_video_descriptor.
 *
 * Carries profile_idc / constraint flags / level_idc so receivers can apply the
 * § 2.14.3.1 T-STD MBn formulas without parsing the elementary stream. We do
 * not emit AVC_timing_and_HRD_descriptor (tag 0x2A); per § 2.6.66 its absence
 * signals the leak method, which is what we use.
 *
 * codecpar->profile carries profile_idc in the low 8 bits and constraint
 * flags in the upper bits (AV_PROFILE_H264_CONSTRAINED -> constraint_set1,
 * AV_PROFILE_H264_INTRA -> constraint_set3).
 */
/*
 * Locate AVC SPS bytes in extradata and fill profile_idc / constraint_flags /
 * level_idc. Two layouts handled:
 *   - AVCDecoderConfigurationRecord (mp4 format): byte 0=0x01, byte 1=profile,
 *     byte 2=constraint_flags, byte 3=level (ISO/IEC 14496-15 § 5.3.3.1.2)
 *   - Annex B (raw NAL units): scan for the SPS NAL via the standard helpers
 *     (avpriv_find_start_code + ff_nal_find_startcode), then parse with
 *     ff_avc_decode_sps. Same pattern as mxfenc.c.
 *
 * Returns 1 on success, 0 if SPS not found / not parseable.
 */
static int extract_avc_sps_fields(const AVCodecParameters *par,
                                  int *out_profile_idc,
                                  int *out_constraint_flags,
                                  int *out_level_idc)
{
    const uint8_t *ed = par->extradata;
    int ed_size = par->extradata_size;

    if (!ed || ed_size < 4)
        return 0;

    /* mp4 AVCDecoderConfigurationRecord: profile/cs/level at fixed offsets. */
    if (ed[0] == 0x01) {
        *out_profile_idc      = ed[1];
        *out_constraint_flags = ed[2];
        *out_level_idc        = ed[3];
        return 1;
    }

    /* Annex B: scan for the SPS NAL using ffmpeg's standard helpers. */
    {
        const uint8_t *buf = ed, *buf_end = ed + ed_size, *nal_end;
        uint32_t state = -1;
        H264SPS sps;
        int i;

        while (buf < buf_end) {
            buf = avpriv_find_start_code(buf, buf_end, &state);
            if (buf >= buf_end)
                break;
            if ((state & 0x1F) != H264_NAL_SPS)
                continue;
            nal_end = ff_nal_find_startcode(buf, buf_end);
            if (ff_avc_decode_sps(&sps, buf, nal_end - buf) < 0)
                return 0;
            *out_profile_idc = sps.profile_idc;
            *out_level_idc   = sps.level_idc;
            /* H264SPS.constraint_set_flags is LSB-first (cs0=bit0..cs5=bit5);
             * the caller and our descriptor wire format expect MSB-first
             * (cs0=bit7..cs5=bit2, low 2 bits = AVC_compatible_flags = 0). */
            *out_constraint_flags = 0;
            for (i = 0; i < 6; i++)
                if (sps.constraint_set_flags & (1 << i))
                    *out_constraint_flags |= 1 << (7 - i);
            return 1;
        }
    }
    return 0;
}

static void put_avc_video_descriptor(uint8_t **q_ptr, const AVCodecParameters *par)
{
    uint8_t *q = *q_ptr;
    int profile = par->profile;
    int level   = par->level;
    int profile_idc, cs1, cs3, constraint_flags = 0;

    /* First try codecpar (set by demuxer for -c copy flows). If that's
     * unset (live encode flows where the encoder hasn't written back to
     * AVCodecContext::profile/level), fall back to scanning extradata —
     * works when the encoder ran with AV_CODEC_FLAG_GLOBAL_HEADER, e.g.
     * ffmpeg CLI:  -flags:v +global_header -bsf:v dump_extra=freq=keyframe */
    if (profile != AV_PROFILE_UNKNOWN && level != AV_LEVEL_UNKNOWN) {
        profile_idc = profile & 0xFF;
        cs1 = (profile & AV_PROFILE_H264_CONSTRAINED) ? 1 : 0;
        cs3 = (profile & AV_PROFILE_H264_INTRA)       ? 1 : 0;
    } else {
        int sps_profile, sps_level;
        if (!extract_avc_sps_fields(par, &sps_profile, &constraint_flags, &sps_level))
            return;
        profile_idc = sps_profile;
        level       = sps_level;
        cs1 = (constraint_flags >> 6) & 1;   /* constraint_set1_flag */
        cs3 = (constraint_flags >> 4) & 1;   /* constraint_set3_flag */
    }

    *q++ = AVC_VIDEO_DESCRIPTOR;              /* tag 0x28 */
    *q++ = 4;                                 /* descriptor_length */
    *q++ = profile_idc;
    *q++ = (cs1 << 6) | (cs3 << 4);           /* cs0..cs5 + AVC_compatible_flags=00 */
    *q++ = level & 0xFF;                      /* level_idc */
    *q++ = (1 << 5) | 0x1F;                   /* AVC_still=0, 24h=0,
                                               * Frame_Packing_SEI_not_present=1, reserved=11111 */
    *q_ptr = q;
}

/*
 * Extract the 12-byte profile_tier_level() block (general_profile_space ..
 * general_level_idc) from HEVC extradata. Both layouts are supported:
 *   - HEVCDecoderConfigurationRecord (ISO/IEC 14496-15 § 8.3.3): bytes 1..12
 *   - Annex B (raw NAL units): scan for an SPS NAL (type 33), then skip the
 *     2-byte NAL header and the first SPS byte (sps_vps_id[4] |
 *     sps_max_sub_layers_minus1[3] | sps_temporal_id_nesting_flag[1]); the
 *     next 12 bytes are profile_tier_level() with profilePresentFlag=1 and
 *     maxNumSubLayersMinus1=0.
 *
 * Returns 0 on success (out_ptl filled with 12 bytes), <0 if neither layout
 * yields the data — caller skips emission rather than guess.
 */
static int hevc_extract_profile_tier_level(const AVCodecParameters *par,
                                           uint8_t out_ptl[12])
{
    const uint8_t *ed = par->extradata;
    int ed_size = par->extradata_size;
    int i;

    if (!ed || ed_size < 13)
        return -1;

    /* HEVCDecoderConfigurationRecord: configurationVersion = 1 in byte 0. */
    if (ed[0] == 0x01 && ed_size >= 23) {
        memcpy(out_ptl, ed + 1, 12);
        return 0;
    }

    /* Annex B: scan for an SPS NAL (type 33) start code. */
    for (i = 0; i + 4 <= ed_size; i++) {
        const uint8_t *p;
        int hdr_len, nal_type;

        if (ed[i] != 0x00)
            continue;
        if (i + 3 < ed_size && ed[i + 1] == 0x00 && ed[i + 2] == 0x00 && ed[i + 3] == 0x01) {
            p = ed + i + 4;
            hdr_len = 4;
        } else if (ed[i + 1] == 0x00 && ed[i + 2] == 0x01) {
            p = ed + i + 3;
            hdr_len = 3;
        } else {
            continue;
        }
        if (p + 2 + 1 + 12 > ed + ed_size)
            return -1;
        nal_type = (p[0] >> 1) & 0x3F;
        if (nal_type == HEVC_NAL_SPS) {
            memcpy(out_ptl, p + 2 + 1, 12);
            return 0;
        }
        i += hdr_len - 1;
    }
    return -1;
}

/*
 * H.222.0 § 2.6.95 (Table 2-113): HEVC_video_descriptor.
 *
 * The descriptor's first 12 bytes (profile_space..level_idc) are byte-for-byte
 * identical to bytes 1..12 of HEVCDecoderConfigurationRecord and to the
 * profile_tier_level() block inside the SPS, so we extract them as-is.
 *
 * Per § 2.14.3 the MBn -> EBn leak rate for HEVC depends on tier_flag, which
 * cannot be derived from codecpar alone. If extradata does not give us
 * profile_tier_level we skip emission rather than emit incorrect tier info.
 */
static void put_hevc_video_descriptor(uint8_t **q_ptr, const AVCodecParameters *par)
{
    uint8_t *q = *q_ptr;
    uint8_t ptl[12];

    if (hevc_extract_profile_tier_level(par, ptl) < 0)
        return;

    *q++ = HEVC_VIDEO_DESCRIPTOR;             /* tag 0x38 */
    *q++ = 13;                                /* descriptor_length */
    memcpy(q, ptl, 12);                       /* profile_space..level_idc (96 bits) */
    q += 12;
    *q++ = (1 << 4) | (0x3 << 2) | 0x3;       /* temporal_layer_subset=0, still=0, 24h=0,
                                               * sub_pic_hrd_params_not_present=1,
                                               * reserved=11, HDR_WCG_idc=11 (no indication) */
    *q_ptr = q;
}

static void put_registration_descriptor(uint8_t **q_ptr, uint32_t tag)
{
    uint8_t *q = *q_ptr;
    *q++ = REGISTRATION_DESCRIPTOR;
    *q++ = 4;
    *q++ = tag;
    *q++ = tag >> 8;
    *q++ = tag >> 16;
    *q++ = tag >> 24;
    *q_ptr = q;
}

static int get_dvb_stream_type(AVFormatContext *s, AVStream *st)
{
    MpegTSWrite *ts = s->priv_data;
    MpegTSWriteStream *ts_st = st->priv_data;
    int stream_type;

    switch (st->codecpar->codec_id) {
    case AV_CODEC_ID_MPEG1VIDEO:
    case AV_CODEC_ID_MPEG2VIDEO:
        stream_type = STREAM_TYPE_VIDEO_MPEG2;
        break;
    case AV_CODEC_ID_MPEG4:
        stream_type = STREAM_TYPE_VIDEO_MPEG4;
        break;
    case AV_CODEC_ID_H264:
        stream_type = STREAM_TYPE_VIDEO_H264;
        break;
    case AV_CODEC_ID_HEVC:
        stream_type = STREAM_TYPE_VIDEO_HEVC;
        break;
    case AV_CODEC_ID_VVC:
        stream_type = STREAM_TYPE_VIDEO_VVC;
        break;
    case AV_CODEC_ID_CAVS:
        stream_type = STREAM_TYPE_VIDEO_CAVS;
        break;
    case AV_CODEC_ID_AVS2:
        stream_type = STREAM_TYPE_VIDEO_AVS2;
        break;
    case AV_CODEC_ID_AVS3:
        stream_type = STREAM_TYPE_VIDEO_AVS3;
        break;
    case AV_CODEC_ID_DIRAC:
        stream_type = STREAM_TYPE_VIDEO_DIRAC;
        break;
    case AV_CODEC_ID_VC1:
        stream_type = STREAM_TYPE_VIDEO_VC1;
        break;
    case AV_CODEC_ID_MP2:
    case AV_CODEC_ID_MP3:
        if (   st->codecpar->sample_rate > 0
            && st->codecpar->sample_rate < 32000) {
            stream_type = STREAM_TYPE_AUDIO_MPEG2;
        } else {
            stream_type = STREAM_TYPE_AUDIO_MPEG1;
        }
        break;
    case AV_CODEC_ID_AAC:
        stream_type = (ts->flags & MPEGTS_FLAG_AAC_LATM)
                      ? STREAM_TYPE_AUDIO_AAC_LATM
                      : STREAM_TYPE_AUDIO_AAC;
        break;
    case AV_CODEC_ID_AAC_LATM:
        stream_type = STREAM_TYPE_AUDIO_AAC_LATM;
        break;
    case AV_CODEC_ID_AC3:
        stream_type = (ts->flags & MPEGTS_FLAG_SYSTEM_B)
                      ? STREAM_TYPE_PRIVATE_DATA
                      : STREAM_TYPE_ATSC_AUDIO_AC3;
        break;
    case AV_CODEC_ID_EAC3:
        stream_type = (ts->flags & MPEGTS_FLAG_SYSTEM_B)
                      ? STREAM_TYPE_PRIVATE_DATA
                      : STREAM_TYPE_ATSC_AUDIO_EAC3;
        break;
    case AV_CODEC_ID_DTS:
        stream_type = STREAM_TYPE_BLURAY_AUDIO_DTS; // should be STREAM_TYPE_PRIVATE_DATA (ETSI TS 101 154), needs a DTS_descriptor() (ETSI EN 300 468)
        break;
    case AV_CODEC_ID_TRUEHD:
        stream_type = STREAM_TYPE_BLURAY_AUDIO_TRUEHD; // should be STREAM_TYPE_PRIVATE_DATA (ETSI TS 101 154), needs a DTS-HD_descriptor() (ETSI EN 300 468)
        break;
    case AV_CODEC_ID_OPUS:
        stream_type = STREAM_TYPE_PRIVATE_DATA;
        break;
    case AV_CODEC_ID_TIMED_ID3:
        stream_type = STREAM_TYPE_METADATA;
        break;
    case AV_CODEC_ID_SMPTE_2038:
        stream_type = STREAM_TYPE_PRIVATE_DATA;
        break;
    case AV_CODEC_ID_DVB_SUBTITLE:
    case AV_CODEC_ID_DVB_TELETEXT:
    case AV_CODEC_ID_ARIB_CAPTION:
        stream_type = STREAM_TYPE_PRIVATE_DATA;
        break;
    case AV_CODEC_ID_SMPTE_KLV:
        if (st->codecpar->profile == AV_PROFILE_KLVA_SYNC) {
            stream_type = STREAM_TYPE_METADATA;
        } else {
            stream_type = STREAM_TYPE_PRIVATE_DATA;
        }
        break;
    default:
        av_log_once(s, AV_LOG_WARNING, AV_LOG_DEBUG, &ts_st->data_st_warning,
                    "Stream %d, codec %s, is muxed as a private data stream "
                    "and may not be recognized upon reading.\n", st->index,
                    avcodec_get_name(st->codecpar->codec_id));
        stream_type = STREAM_TYPE_PRIVATE_DATA;
        break;
    }

    return stream_type;
}

static int get_m2ts_stream_type(AVFormatContext *s, AVStream *st)
{
    int stream_type;
    MpegTSWriteStream *ts_st = st->priv_data;

    switch (st->codecpar->codec_id) {
    case AV_CODEC_ID_MPEG2VIDEO:
        stream_type = STREAM_TYPE_VIDEO_MPEG2;
        break;
    case AV_CODEC_ID_H264:
        stream_type = STREAM_TYPE_VIDEO_H264;
        break;
    case AV_CODEC_ID_VC1:
        stream_type = STREAM_TYPE_VIDEO_VC1;
        break;
    case AV_CODEC_ID_HEVC:
        stream_type = STREAM_TYPE_VIDEO_HEVC;
        break;
    case AV_CODEC_ID_PCM_BLURAY:
        stream_type = STREAM_TYPE_BLURAY_AUDIO_PCM_BLURAY;
        break;
    case AV_CODEC_ID_AC3:
        stream_type = STREAM_TYPE_BLURAY_AUDIO_AC3;
        break;
    case AV_CODEC_ID_DTS:
        stream_type = (st->codecpar->ch_layout.nb_channels > 6) ?
                        STREAM_TYPE_BLURAY_AUDIO_DTS_HD :
                        STREAM_TYPE_BLURAY_AUDIO_DTS;
        break;
    case AV_CODEC_ID_TRUEHD:
        stream_type = STREAM_TYPE_BLURAY_AUDIO_TRUEHD;
        break;
    case AV_CODEC_ID_EAC3:
        stream_type = STREAM_TYPE_BLURAY_AUDIO_EAC3;
        break;
    case AV_CODEC_ID_HDMV_PGS_SUBTITLE:
        stream_type = STREAM_TYPE_BLURAY_SUBTITLE_PGS;
        break;
    case AV_CODEC_ID_HDMV_TEXT_SUBTITLE:
        stream_type = STREAM_TYPE_BLURAY_SUBTITLE_TEXT;
        break;
    default:
        av_log_once(s, AV_LOG_WARNING, AV_LOG_DEBUG, &ts_st->data_st_warning,
                    "Stream %d, codec %s, is muxed as a private data stream "
                    "and may not be recognized upon reading.\n", st->index,
                    avcodec_get_name(st->codecpar->codec_id));
        stream_type = STREAM_TYPE_PRIVATE_DATA;
        break;
    }

    return stream_type;
}

static int mpegts_write_pmt(AVFormatContext *s, MpegTSService *service)
{
    MpegTSWrite *ts = s->priv_data;
    uint8_t data[SECTION_LENGTH], *q, *desc_length_ptr, *program_info_length_ptr;
    int val, stream_type, i, err = 0;

    q = data;
    put16(&q, 0xe000 | service->pcr_pid);

    program_info_length_ptr = q;
    q += 2; /* patched after */

    /* put program info here */
    if (ts->m2ts_mode) {
        put_registration_descriptor(&q, MKTAG('H', 'D', 'M', 'V'));
        *q++ = 0x88;        // descriptor_tag - hdmv_copy_control_descriptor
        *q++ = 0x04;        // descriptor_length
        put16(&q, 0x0fff);  // CA_System_ID
        *q++ = 0xfc;        // private_data_byte
        *q++ = 0xfc;        // private_data_byte
    }

    val = 0xf000 | (q - program_info_length_ptr - 2);
    program_info_length_ptr[0] = val >> 8;
    program_info_length_ptr[1] = val;

    for (i = 0; i < s->nb_streams; i++) {
        AVStream *st = s->streams[i];
        MpegTSWriteStream *ts_st = st->priv_data;
        AVDictionaryEntry *lang = av_dict_get(st->metadata, "language", NULL, 0);
        const char default_language[] = "und";
        const char *language = lang && strlen(lang->value) >= 3 ? lang->value : default_language;
        enum AVCodecID codec_id = st->codecpar->codec_id;

        if (s->nb_programs) {
            int k, found = 0;
            AVProgram *program = service->program;

            for (k = 0; k < program->nb_stream_indexes; k++)
                if (program->stream_index[k] == i) {
                    found = 1;
                    break;
                }

            if (!found)
                continue;
        }

        if (q - data > SECTION_LENGTH - 32) {
            err = 1;
            break;
        }

        stream_type = ts->m2ts_mode ? get_m2ts_stream_type(s, st) : get_dvb_stream_type(s, st);

        *q++ = stream_type;
        put16(&q, 0xe000 | ts_st->pid);
        desc_length_ptr = q;
        q += 2; /* patched after */

        /* write optional descriptors here */
        switch (st->codecpar->codec_type) {
        case AVMEDIA_TYPE_AUDIO:
            if (codec_id == AV_CODEC_ID_AC3)
                put_registration_descriptor(&q, MKTAG('A', 'C', '-', '3'));
            if (codec_id == AV_CODEC_ID_EAC3)
                put_registration_descriptor(&q, MKTAG('E', 'A', 'C', '3'));
            if (codec_id == AV_CODEC_ID_AAC || codec_id == AV_CODEC_ID_AAC_LATM) {
                /* 4 bytes: tag + len + profile_and_level + flags */
                if (q - data > SECTION_LENGTH - 4) {
                    err = 1;
                    break;
                }
                put_aac_descriptor(&q, st->codecpar);
            }
            if (ts->flags & MPEGTS_FLAG_SYSTEM_B) {
                if (codec_id == AV_CODEC_ID_AC3) {
                    DVBAC3Descriptor *dvb_ac3_desc = ts_st->dvb_ac3_desc;

                    *q++= AC3_DESCRIPTOR; // AC3 descriptor see A038 DVB SI
                    if (dvb_ac3_desc) {
                        int len = 1 +
                                  !!(dvb_ac3_desc->component_type_flag) +
                                  !!(dvb_ac3_desc->bsid_flag) +
                                  !!(dvb_ac3_desc->mainid_flag) +
                                  !!(dvb_ac3_desc->asvc_flag);

                        *q++ = len;
                        *q++ = dvb_ac3_desc->component_type_flag << 7 | dvb_ac3_desc->bsid_flag << 6 |
                               dvb_ac3_desc->mainid_flag         << 5 | dvb_ac3_desc->asvc_flag << 4;

                        if (dvb_ac3_desc->component_type_flag) *q++ = dvb_ac3_desc->component_type;
                        if (dvb_ac3_desc->bsid_flag)           *q++ = dvb_ac3_desc->bsid;
                        if (dvb_ac3_desc->mainid_flag)         *q++ = dvb_ac3_desc->mainid;
                        if (dvb_ac3_desc->asvc_flag)           *q++ = dvb_ac3_desc->asvc;
                    } else {
                        *q++=1; // 1 byte, all flags sets to 0
                        *q++=0; // omit all fields...
                    }
                } else if (codec_id == AV_CODEC_ID_EAC3) {
                    *q++= ENHANCED_AC3_DESCRIPTOR; // EAC3 descriptor see A038 DVB SI
                    *q++=1; // 1 byte, all flags sets to 0
                    *q++=0; // omit all fields...
                }
            }
            if (codec_id == AV_CODEC_ID_S302M)
                put_registration_descriptor(&q, MKTAG('B', 'S', 'S', 'D'));
            if (codec_id == AV_CODEC_ID_OPUS) {
                int ch = st->codecpar->ch_layout.nb_channels;

                /* 6 bytes registration descriptor, 4 bytes Opus audio descriptor */
                if (q - data > SECTION_LENGTH - 6 - 4) {
                    err = 1;
                    break;
                }

                put_registration_descriptor(&q, MKTAG('O', 'p', 'u', 's'));

                *q++ = DVB_EXTENSION_DESCRIPTOR;
                *q++ = 2;
                *q++ = 0x80;

                if (st->codecpar->extradata && st->codecpar->extradata_size >= 19) {
                    if (st->codecpar->extradata[18] == 0 && ch <= 2) {
                        /* RTP mapping family */
                        *q++ = ch;
                    } else if (st->codecpar->extradata[18] == 1 && ch <= 8 &&
                               st->codecpar->extradata_size >= 21 + ch) {
                        static const uint8_t coupled_stream_counts[9] = {
                            1, 0, 1, 1, 2, 2, 2, 3, 3
                        };
                        static const uint8_t channel_map_a[8][8] = {
                            {0},
                            {0, 1},
                            {0, 2, 1},
                            {0, 1, 2, 3},
                            {0, 4, 1, 2, 3},
                            {0, 4, 1, 2, 3, 5},
                            {0, 4, 1, 2, 3, 5, 6},
                            {0, 6, 1, 2, 3, 4, 5, 7},
                        };
                        static const uint8_t channel_map_b[8][8] = {
                            {0},
                            {0, 1},
                            {0, 1, 2},
                            {0, 1, 2, 3},
                            {0, 1, 2, 3, 4},
                            {0, 1, 2, 3, 4, 5},
                            {0, 1, 2, 3, 4, 5, 6},
                            {0, 1, 2, 3, 4, 5, 6, 7},
                        };
                        /* Vorbis mapping family */

                        if (st->codecpar->extradata[19] == ch - coupled_stream_counts[ch] &&
                            st->codecpar->extradata[20] == coupled_stream_counts[ch] &&
                            memcmp(&st->codecpar->extradata[21], channel_map_a[ch - 1], ch) == 0) {
                            *q++ = ch;
                        } else if (ch >= 2 && st->codecpar->extradata[19] == ch &&
                                   st->codecpar->extradata[20] == 0 &&
                                   memcmp(&st->codecpar->extradata[21], channel_map_b[ch - 1], ch) == 0) {
                            *q++ = ch | 0x80;
                        } else {
                            /* Unsupported, could write an extended descriptor here */
                            av_log(s, AV_LOG_ERROR, "Unsupported Opus Vorbis-style channel mapping");
                            *q++ = 0xff;
                        }
                    } else {
                        /* Unsupported */
                        av_log(s, AV_LOG_ERROR, "Unsupported Opus channel mapping for family %d", st->codecpar->extradata[18]);
                        *q++ = 0xff;
                    }
                } else if (ch <= 2) {
                    /* Assume RTP mapping family */
                    *q++ = ch;
                } else {
                    /* Unsupported */
                    av_log(s, AV_LOG_ERROR, "Unsupported Opus channel mapping");
                    *q++ = 0xff;
                }
            }

            if (language != default_language ||
                st->disposition & (AV_DISPOSITION_CLEAN_EFFECTS    |
                                   AV_DISPOSITION_HEARING_IMPAIRED |
                                   AV_DISPOSITION_VISUAL_IMPAIRED)) {
                const char *p, *next;
                uint8_t *len_ptr;

                *q++     = ISO_639_LANGUAGE_DESCRIPTOR;
                len_ptr  = q++;
                *len_ptr = 0;

                for (p = next = language; next && *len_ptr < 255 / 4 * 4; p = next + 1) {
                    if (q - data > SECTION_LENGTH - 4) {
                        err = 1;
                        break;
                    }
                    next = strchr(p, ',');
                    if (strlen(p) != 3 && (!next || next != p + 3))
                        continue; /* not a 3-letter code */

                    *q++ = *p++;
                    *q++ = *p++;
                    *q++ = *p++;

                    if (st->disposition & AV_DISPOSITION_CLEAN_EFFECTS)
                        *q++ = 0x01;
                    else if (st->disposition & AV_DISPOSITION_HEARING_IMPAIRED)
                        *q++ = 0x02;
                    else if (st->disposition & AV_DISPOSITION_VISUAL_IMPAIRED)
                        *q++ = 0x03;
                    else
                        *q++ = 0; /* undefined type */

                    *len_ptr += 4;
                }

                if (*len_ptr == 0)
                    q -= 2; /* no language codes were written */
            }
            break;
        case AVMEDIA_TYPE_SUBTITLE:
           if (codec_id == AV_CODEC_ID_DVB_SUBTITLE) {
               uint8_t *len_ptr;
               int extradata_copied = 0;

               *q++ = SUBTITLING_DESCRIPTOR; /* subtitling_descriptor */
               len_ptr = q++;

               while (strlen(language) >= 3) {
                   if (sizeof(data) - (q - data) < 8) { /* 8 bytes per DVB subtitle substream data */
                       err = 1;
                       break;
                   }
                   *q++ = *language++;
                   *q++ = *language++;
                   *q++ = *language++;
                   /* Skip comma */
                   if (*language != '\0')
                       language++;

                   if (st->codecpar->extradata_size - extradata_copied >= 5) {
                       *q++ = st->codecpar->extradata[extradata_copied + 4]; /* subtitling_type */
                       memcpy(q, st->codecpar->extradata + extradata_copied, 4); /* composition_page_id and ancillary_page_id */
                       extradata_copied += 5;
                       q += 4;
                   } else {
                       /* subtitling_type:
                        * 0x10 - normal with no monitor aspect ratio criticality
                        * 0x20 - for the hard of hearing with no monitor aspect ratio criticality */
                       *q++ = (st->disposition & AV_DISPOSITION_HEARING_IMPAIRED) ? 0x20 : 0x10;
                       if ((st->codecpar->extradata_size == 4) && (extradata_copied == 0)) {
                           /* support of old 4-byte extradata format */
                           memcpy(q, st->codecpar->extradata, 4); /* composition_page_id and ancillary_page_id */
                           extradata_copied += 4;
                           q += 4;
                       } else {
                           put16(&q, 1); /* composition_page_id */
                           put16(&q, 1); /* ancillary_page_id */
                       }
                   }
               }

               *len_ptr = q - len_ptr - 1;
           } else if (codec_id == AV_CODEC_ID_DVB_TELETEXT) {
               uint8_t *len_ptr = NULL;
               int extradata_copied = 0;

               /* The descriptor tag. teletext_descriptor */
               *q++ = TELETEXT_DESCRIPTOR;
               len_ptr = q++;

               while (strlen(language) >= 3 && q - data < sizeof(data) - 6) {
                   *q++ = *language++;
                   *q++ = *language++;
                   *q++ = *language++;
                   /* Skip comma */
                   if (*language != '\0')
                       language++;

                   if (st->codecpar->extradata_size - 1 > extradata_copied) {
                       memcpy(q, st->codecpar->extradata + extradata_copied, 2);
                       extradata_copied += 2;
                       q += 2;
                   } else {
                       /* The Teletext descriptor:
                        * teletext_type: This 5-bit field indicates the type of Teletext page indicated. (0x01 Initial Teletext page)
                        * teletext_magazine_number: This is a 3-bit field which identifies the magazine number.
                        * teletext_page_number: This is an 8-bit field giving two 4-bit hex digits identifying the page number. */
                       *q++ = 0x08;
                       *q++ = 0x00;
                   }
               }

               *len_ptr = q - len_ptr - 1;
            } else if (codec_id == AV_CODEC_ID_ARIB_CAPTION) {
                if (put_arib_caption_descriptor(s, &q, st->codecpar) < 0)
                    break;
            }
        break;
        case AVMEDIA_TYPE_VIDEO:
            if (stream_type == STREAM_TYPE_VIDEO_DIRAC) {
                put_registration_descriptor(&q, MKTAG('d', 'r', 'a', 'c'));
            } else if (stream_type == STREAM_TYPE_VIDEO_VC1) {
                put_registration_descriptor(&q, MKTAG('V', 'C', '-', '1'));
            } else if (stream_type == STREAM_TYPE_VIDEO_HEVC && s->strict_std_compliance <= FF_COMPLIANCE_NORMAL) {
                put_registration_descriptor(&q, MKTAG('H', 'E', 'V', 'C'));
            } else if (stream_type == STREAM_TYPE_VIDEO_CAVS || stream_type == STREAM_TYPE_VIDEO_AVS2 ||
                       stream_type == STREAM_TYPE_VIDEO_AVS3) {
                put_registration_descriptor(&q, MKTAG('A', 'V', 'S', 'V'));
            }
            if (stream_type == STREAM_TYPE_VIDEO_H264) {
                /* AVC_video_descriptor: 6 bytes (tag + len + 4 payload).
                 * Note: in transcode pipelines (e.g. -c:v libx264), codecpar
                 * is filled from the encoder context only after the encoder
                 * produces its first packet, which happens after write_header.
                 * In that case profile/level read as -99 here and we emit no
                 * descriptor. The remux/copy path (e.g. -c copy from an mp4)
                 * has codecpar populated by the demuxer and works correctly. */
                if (q - data > SECTION_LENGTH - 6) {
                    err = 1;
                    break;
                }
                put_avc_video_descriptor(&q, st->codecpar);
            } else if (stream_type == STREAM_TYPE_VIDEO_HEVC) {
                /* HEVC_video_descriptor: 15 bytes (tag + len + 13 payload) */
                if (q - data > SECTION_LENGTH - 15) {
                    err = 1;
                    break;
                }
                put_hevc_video_descriptor(&q, st->codecpar);
            }
            break;
        case AVMEDIA_TYPE_DATA:
            if (codec_id == AV_CODEC_ID_SMPTE_KLV) {
                put_registration_descriptor(&q, MKTAG('K', 'L', 'V', 'A'));
            } else if (codec_id == AV_CODEC_ID_SMPTE_2038) {
                put_registration_descriptor(&q, MKTAG('V', 'A', 'N', 'C'));
            } else if (codec_id == AV_CODEC_ID_TIMED_ID3) {
                const char *tag = "ID3 ";
                *q++ = METADATA_DESCRIPTOR;
                *q++ = 13;
                put16(&q, 0xffff);    /* metadata application format */
                putbuf(&q, tag, strlen(tag));
                *q++ = 0xff;        /* metadata format */
                putbuf(&q, tag, strlen(tag));
                *q++ = 0;            /* metadata service ID */
                *q++ = 0xF;          /* metadata_locator_record_flag|MPEG_carriage_flags|reserved */
            }
            break;
        }

        val = 0xf000 | (q - desc_length_ptr - 2);
        desc_length_ptr[0] = val >> 8;
        desc_length_ptr[1] = val;
    }

    if (err)
        av_log(s, AV_LOG_ERROR,
               "The PMT section cannot fit stream %d and all following streams.\n"
               "Try reducing the number of languages in the audio streams "
               "or the total number of streams.\n", i);

    mpegts_write_section1(&service->pmt, PMT_TID, service->sid, ts->tables_version, 0, 0,
                          data, q - data);
    return 0;
}

static void mpegts_write_sdt(AVFormatContext *s)
{
    MpegTSWrite *ts = s->priv_data;
    MpegTSService *service;
    uint8_t data[SECTION_LENGTH], *q, *desc_list_len_ptr, *desc_len_ptr;
    int i, running_status, free_ca_mode, val;

    q = data;
    put16(&q, ts->original_network_id);
    *q++ = 0xff;
    for (i = 0; i < ts->nb_services; i++) {
        service = ts->services[i];
        put16(&q, service->sid);
        *q++              = 0xfc | 0x00; /* currently no EIT info */
        desc_list_len_ptr = q;
        q                += 2;
        running_status    = 4; /* running */
        free_ca_mode      = 0;

        /* write only one descriptor for the service name and provider */
        *q++         = SERVICE_DESCRIPTOR;
        desc_len_ptr = q;
        q++;
        *q++         = ts->service_type;
        putbuf(&q, service->provider_name, service->provider_name[0] + 1);
        putbuf(&q, service->name, service->name[0] + 1);
        desc_len_ptr[0] = q - desc_len_ptr - 1;

        /* fill descriptor length */
        val = (running_status << 13) | (free_ca_mode << 12) |
              (q - desc_list_len_ptr - 2);
        desc_list_len_ptr[0] = val >> 8;
        desc_list_len_ptr[1] = val;
    }
    mpegts_write_section1(&ts->sdt, SDT_TID, ts->transport_stream_id, ts->tables_version, 0, 0,
                          data, q - data);
}

static void mpegts_write_nit(AVFormatContext *s)
{
    MpegTSWrite *ts = s->priv_data;
    uint8_t data[SECTION_LENGTH], *q, *desc_len_ptr, *loop_len_ptr;

    q = data;

    //network_descriptors_length
    put16(&q, 0xf000 | (ts->provider_name[0] + 2));

    //network_name_descriptor
    *q++ = NETWORK_NAME_DESCRIPTOR;
    putbuf(&q, ts->provider_name, ts->provider_name[0] + 1);

    //transport_stream_loop_length
    loop_len_ptr = q;
    q += 2;

    put16(&q, ts->transport_stream_id);
    put16(&q, ts->original_network_id);

    //transport_descriptors_length
    desc_len_ptr = q;
    q += 2;

    //service_list_descriptor
    *q++ = SERVICE_LIST_DESCRIPTOR;
    *q++ = 3 * ts->nb_services;
    for (int i = 0; i < ts->nb_services; i++) {
        put16(&q, ts->services[i]->sid);
        *q++ = ts->service_type;
    }

    //calculate lengths
    put16(&desc_len_ptr, 0xf000 | q - (desc_len_ptr + 2));
    put16(&loop_len_ptr, 0xf000 | q - (loop_len_ptr + 2));

    mpegts_write_section1(&ts->nit, NIT_TID, ts->original_network_id, ts->tables_version, 0, 0,
                          data, q - data);
}

/* This stores a string in buf with the correct encoding and also sets the
 * first byte as the length. !str is accepted for an empty string.
 * If the string is already encoded, invalid UTF-8 or has no multibyte sequence
 * then we keep it as is, otherwise we signal UTF-8 encoding. */
static int encode_str8(uint8_t *buf, const char *str)
{
    size_t str_len;
    if (!str)
        str = "";
    str_len = strlen(str);
    if (str[0] && (unsigned)str[0] >= 0x20) {   /* Make sure the string is not already encoded. */
        const uint8_t *q = str;
        int has_multibyte = 0;
        while (*q) {
            uint32_t code;
            GET_UTF8(code, *q++, goto invalid;) /* Is it valid UTF-8? */
            has_multibyte |= (code > 127);      /* Does it have multibyte UTF-8 chars in it? */
        }
        if (has_multibyte) {                    /* If we have multibyte chars and valid UTF-8, then encode as such! */
            if (str_len > 254)
                return AVERROR(EINVAL);
            buf[0] = str_len + 1;
            buf[1] = 0x15;
            memcpy(&buf[2], str, str_len);
            return 0;
        }
    }
invalid:
    /* Otherwise let's just encode the string as is! */
    if (str_len > 255)
        return AVERROR(EINVAL);
    buf[0] = str_len;
    memcpy(&buf[1], str, str_len);
    return 0;
}

/* Forward declaration — defined later, needed by interleave_write_raw */
static int write_pcr_bits(uint8_t *buf, int64_t pcr);

/* ------------------------------------------------------------------ */
/* TS packet interleaving — buffer/drain helpers                       */
/* ------------------------------------------------------------------ */

/* Known SI PIDs that route to the SI buffer during interleaving.
 * Null packets (0x1FFF) are NOT routed to SI — they are CBR padding
 * and should be distributed evenly, not bunched with SI tables. */
static int is_si_pid(int pid)
{
    return pid < 0x0020;
}

/* Grow an interleave buffer to hold at least `needed` packets */
static int interleave_buf_grow(uint8_t **buf, int *capacity, int needed)
{
    int new_cap;
    uint8_t *new_buf;

    if (needed <= *capacity)
        return 0;
    new_cap = FFMAX(*capacity * 2, needed);
    new_buf = av_realloc(*buf, (int64_t)new_cap * TS_PACKET_SIZE);
    if (!new_buf)
        return AVERROR(ENOMEM);
    *buf = new_buf;
    *capacity = new_cap;
    return 0;
}

/* Enqueue one 188-byte TS packet into the appropriate interleave buffer.
 * Null packets (0x1FFF) are discarded — they will be regenerated at drain
 * time as needed to maintain CBR rate via the pacing mechanism. */
static void interleave_enqueue(AVFormatContext *s, const uint8_t *packet)
{
    MpegTSWrite *ts = s->priv_data;
    int pid = ((packet[1] & 0x1F) << 8) | packet[2];
    int i;

    /* Null packets (0x1FFF) routing depends on interleave mode:
     * Mode 1 (null-interleave): nulls go to video buffer, audio interleaves
     *   among nulls. Works when Rxn_video > muxrate.
     * Mode 2 (full-interleave): nulls go to SI buffer, so the drain
     *   interleaves audio directly among video packets. Works for all
     *   profiles/levels regardless of Rxn vs muxrate. */
    if (pid == 0x1FFF) {
        if (ts->ts_interleave == 1) {
            /* Mode 1: nulls → video buffer */
            for (i = 0; i < s->nb_streams; i++) {
                MpegTSWriteStream *ts_st = s->streams[i]->priv_data;
                if (ts_st && s->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
                    interleave_buf_grow(&ts_st->interleave_buf,
                                        &ts_st->interleave_buf_capacity,
                                        ts_st->interleave_buf_count + 1);
                    memcpy(ts_st->interleave_buf +
                           (int64_t)ts_st->interleave_buf_count * TS_PACKET_SIZE,
                           packet, TS_PACKET_SIZE);
                    ts_st->interleave_buf_count++;
                    return;
                }
            }
        }
        /* Mode 2 or no video stream: nulls → SI buffer */
    }

    if (is_si_pid(pid)) {
        /* SI (PAT, PMT, SDT, NIT, PCR-only for other streams) → shared SI buffer */
        interleave_buf_grow(&ts->interleave_si_buf,
                            &ts->interleave_si_capacity,
                            ts->interleave_si_count + 1);
        memcpy(ts->interleave_si_buf +
               (int64_t)ts->interleave_si_count * TS_PACKET_SIZE,
               packet, TS_PACKET_SIZE);
        ts->interleave_si_count++;
    } else {
        /* Elementary stream → find per-stream buffer by PID */
        for (i = 0; i < s->nb_streams; i++) {
            MpegTSWriteStream *ts_st = s->streams[i]->priv_data;
            if (ts_st && ts_st->pid == pid) {
                interleave_buf_grow(&ts_st->interleave_buf,
                                    &ts_st->interleave_buf_capacity,
                                    ts_st->interleave_buf_count + 1);
                memcpy(ts_st->interleave_buf +
                       (int64_t)ts_st->interleave_buf_count * TS_PACKET_SIZE,
                       packet, TS_PACKET_SIZE);
                ts_st->interleave_buf_count++;
                return;
            }
        }
        /* PID not found among streams — treat as SI (e.g., PMT PID) */
        interleave_buf_grow(&ts->interleave_si_buf,
                            &ts->interleave_si_capacity,
                            ts->interleave_si_count + 1);
        memcpy(ts->interleave_si_buf +
               (int64_t)ts->interleave_si_count * TS_PACKET_SIZE,
               packet, TS_PACKET_SIZE);
        ts->interleave_si_count++;
    }
}

/* ------------------------------------------------------------------
 * Threaded-output helpers.
 *
 * mpegts_out_emit_paced:  called only by the output thread.  Takes a
 *   raw TS packet from the ring, rewrites PCR based on the current
 *   output byte position, paces to at least one packet-duration after
 *   the previous emit, then writes to avio.  This is the only place
 *   s->pb, interleave_output_bytes, and out_last_emit_us are written
 *   when threaded mode is on.
 *
 * mpegts_out_thread_fn:  top-level output-thread loop.  Dequeues from
 *   the ring until shutdown is signalled AND the ring is empty, then
 *   returns.  Blocks on the not_empty condvar when the ring is
 *   momentarily empty.
 * ------------------------------------------------------------------ */
static void mpegts_out_emit_paced(AVFormatContext *s, uint8_t *packet)
{
    MpegTSWrite *ts = s->priv_data;

    /* Rewrite PCR (same logic as non-threaded interleave_write_raw). */
    {
        int afc = (packet[3] >> 4) & 0x03;
        if ((afc == 0x02 || afc == 0x03) && packet[4] >= 7) {
            int pcr_flag = packet[5] & 0x10;
            if (pcr_flag) {
                int64_t pcr = av_rescale(ts->interleave_output_bytes + 11,
                                         8 * PCR_TIME_BASE, ts->mux_rate) +
                              ts->first_pcr;
                write_pcr_bits(&packet[6], pcr);
            }
        }
    }

    /* Strict pd-spaced CBR pacing: each emit is at least pd after the
     * previous one, clamped so the on-wire rate never exceeds mux_rate.
     * Any wall-clock drift from the mux thread is absorbed by the ring
     * buffer, not by the on-wire schedule.
     *
     * We advance out_last_emit_us by the planned target, not by post-write
     * wall time.  Using wall time would bake avio_write() latency into the
     * spacing and make the emit rate drift below mux_rate. */
    if (ts->mux_rate > 1) {
        int64_t now_us = av_gettime_relative();
        int64_t pd_us = av_rescale(TS_PACKET_SIZE * 8, 1000000LL,
                                   ts->mux_rate);
        int64_t target_us;

        if (ts->out_last_emit_us == 0)
            target_us = now_us;
        else
            target_us = ts->out_last_emit_us + pd_us;

        if (target_us > now_us) {
            /* Default: coarse-sleep the bulk of each wait, spin only
             * the last CBR_PACE_SPIN_US for precision.  This frees the
             * CPU and — counterintuitively — empirically produces
             * fewer late events than pure busy-spin on a stock Linux
             * scheduler, because a cooperative thread is preempted
             * less often than a 100%-CPU hog.  cbr_pacing_spin=1
             * forces pure busy-spin for RT/isolcpus deployments. */
            int64_t wait = target_us - now_us;
            if (wait <= CBR_PACE_MAX_EARLY_US) {
                if (!ts->cbr_pacing_spin && wait > CBR_PACE_SPIN_US)
                    av_usleep((unsigned)(wait - CBR_PACE_SPIN_US));
                while (av_gettime_relative() < target_us)
                    spin_hint();
            }
        } else if (ts->out_last_emit_us != 0) {
            /* Output thread ran late — cap target at now_us to prevent a
             * run-away catch-up burst.  Each subsequent packet will be
             * strictly pd-spaced from its actual emission time. */
            atomic_fetch_add_explicit(&ts->out_ring->late_emits, 1,
                                      memory_order_relaxed);
            target_us = now_us;
        }
        ts->out_last_emit_us = target_us;
        atomic_fetch_add_explicit(&ts->out_ring->total_emits, 1,
                                  memory_order_relaxed);
    }

    if (ts->m2ts_mode) {
        int64_t pcr = av_rescale(ts->interleave_output_bytes + 11,
                                 8 * PCR_TIME_BASE, ts->mux_rate) +
                      ts->first_pcr;
        uint32_t tp_extra_header = pcr % 0x3fffffff;
        tp_extra_header = AV_RB32(&tp_extra_header);
        avio_write(s->pb, (unsigned char *)&tp_extra_header,
                   sizeof(tp_extra_header));
    }
    avio_write(s->pb, packet, TS_PACKET_SIZE);
    ts->interleave_output_bytes += TS_PACKET_SIZE;
}

static void *mpegts_out_thread_fn(void *arg)
{
    AVFormatContext *s = arg;
    MpegTSWrite *ts = s->priv_data;
    MpegTSOutRing *r = ts->out_ring;
    uint8_t packet[TS_PACKET_SIZE];

    for (;;) {
        int head = atomic_load_explicit(&r->head, memory_order_relaxed);

        /* Spin-wait for a packet.  Acquire-load pairs with the
         * producer's release-store on tail.  No kernel entry. */
        while (head == atomic_load_explicit(&r->tail,
                                            memory_order_acquire)) {
            if (atomic_load_explicit(&r->shutdown, memory_order_acquire))
                return NULL;
            atomic_fetch_add_explicit(&r->consumer_empty_spins, 1,
                                      memory_order_relaxed);
            spin_hint();
        }

        memcpy(packet, r->packets[head], TS_PACKET_SIZE);
        atomic_store_explicit(&r->head,
                              (head + 1) & MPEGTS_OUT_RING_MASK,
                              memory_order_release);

        mpegts_out_emit_paced(s, packet);
    }
}

static int mpegts_out_thread_start(AVFormatContext *s)
{
    MpegTSWrite *ts = s->priv_data;
    MpegTSOutRing *r;
    int ret;

    r = av_mallocz(sizeof(*r));
    if (!r)
        return AVERROR(ENOMEM);
    ts->out_ring = r;

    if ((ret = pthread_create(&ts->out_thread, NULL,
                              mpegts_out_thread_fn, s))) {
        av_freep(&ts->out_ring);
        return AVERROR(ret);
    }
    ts->out_thread_started = 1;

    return 0;
}

static void mpegts_out_thread_stop(AVFormatContext *s)
{
    MpegTSWrite *ts = s->priv_data;
    MpegTSOutRing *r = ts->out_ring;

    if (!ts->out_thread_started || !r)
        return;

    /* Signal shutdown — producer/consumer pick it up on next spin. */
    atomic_store_explicit(&r->shutdown, 1, memory_order_release);

    pthread_join(ts->out_thread, NULL);
    ts->out_thread_started = 0;

    {
        int64_t emits = atomic_load_explicit(&r->total_emits,
                                             memory_order_relaxed);
        int64_t late = atomic_load_explicit(&r->late_emits,
                                            memory_order_relaxed);
        int64_t pspins = atomic_load_explicit(&r->producer_full_spins,
                                              memory_order_relaxed);
        int64_t cspins = atomic_load_explicit(&r->consumer_empty_spins,
                                              memory_order_relaxed);
        int hwmark = atomic_load_explicit(&r->max_depth,
                                          memory_order_relaxed);
        av_log(s, AV_LOG_INFO,
               "cbr_pacing_threaded: emits=%lld late=%lld (%.3f%%) "
               "ring hwmark=%d/%d p_spins=%lld c_spins=%lld\n",
               (long long)emits, (long long)late,
               emits ? 100.0 * late / emits : 0.0,
               hwmark, MPEGTS_OUT_RING_CAP,
               (long long)pspins, (long long)cspins);
    }

    av_freep(&ts->out_ring);
}

/* Write one packet directly to output with CBR pacing (used during drain).
 * If the packet contains a PCR, recompute it based on the actual output
 * byte position (interleave_output_bytes) rather than the buffering-time
 * position (total_size).  This is necessary because interleaving changes
 * the byte position of each packet. */
static void interleave_write_raw(AVFormatContext *s, const uint8_t *packet_in)
{
    MpegTSWrite *ts = s->priv_data;
    uint8_t packet[TS_PACKET_SIZE];

    memcpy(packet, packet_in, TS_PACKET_SIZE);

    /* Threaded output path: hand the raw packet (no PCR rewrite, no
     * pacing) to the output thread via the ring buffer.  The output
     * thread owns s->pb and interleave_output_bytes from this point. */
    if (ts->cbr_pacing_threaded && ts->out_ring) {
        MpegTSOutRing *r = ts->out_ring;
        int tail = atomic_load_explicit(&r->tail, memory_order_relaxed);
        int next_tail = (tail + 1) & MPEGTS_OUT_RING_MASK;

        /* Wait for a slot.  PAUSE-spin while full — this is the
         * backpressure signal to the mux thread.  The output thread
         * drains at strict mux_rate so the spin is bounded (<= pd). */
        while (next_tail == atomic_load_explicit(&r->head,
                                                 memory_order_acquire)) {
            if (atomic_load_explicit(&r->shutdown, memory_order_relaxed))
                return;
            atomic_fetch_add_explicit(&r->producer_full_spins, 1,
                                      memory_order_relaxed);
            spin_hint();
        }

        memcpy(r->packets[tail], packet, TS_PACKET_SIZE);
        /* Publish the new tail with release ordering so the output
         * thread sees the packet bytes before the index update. */
        atomic_store_explicit(&r->tail, next_tail, memory_order_release);

        /* Track high-water mark (approximate; relaxed). */
        {
            int head = atomic_load_explicit(&r->head, memory_order_relaxed);
            int depth = (next_tail - head) & MPEGTS_OUT_RING_MASK;
            int cur = atomic_load_explicit(&r->max_depth, memory_order_relaxed);
            if (depth > cur)
                atomic_store_explicit(&r->max_depth, depth,
                                      memory_order_relaxed);
        }
        return;
    }

    /* Rewrite PCR if present.  PCR location:
     *   byte 4 = adaptation_field_length
     *   byte 5 = adaptation flags (bit 4 = PCR_flag)
     *   bytes 6-11 = PCR field (6 bytes) */
    {
        int afc = (packet[3] >> 4) & 0x03;
        if ((afc == 0x02 || afc == 0x03) && packet[4] >= 7) {
            int pcr_flag = packet[5] & 0x10;
            if (pcr_flag) {
                int64_t pcr = av_rescale(ts->interleave_output_bytes + 11,
                                         8 * PCR_TIME_BASE, ts->mux_rate) +
                              ts->first_pcr;
                write_pcr_bits(&packet[6], pcr);
            }
        }
    }

    /* CBR pacing based on output byte count */
    if (ts->cbr_pacing && ts->mux_rate > 1) {
        int64_t now_us, target_us, wait;

        now_us = av_gettime_relative();
        if (ts->pacing_start_us == 0)
            ts->pacing_start_us = now_us;

        target_us = ts->pacing_start_us +
                    av_rescale(ts->interleave_output_bytes, 8000000LL,
                               ts->mux_rate);
        wait = target_us - now_us;

        if (wait > 0 && wait <= CBR_PACE_MAX_EARLY_US) {
            if (wait > CBR_PACE_SPIN_US)
                av_usleep((unsigned)(wait - CBR_PACE_SPIN_US));
            while (av_gettime_relative() < target_us)
                spin_hint();
        }
    }

    if (ts->m2ts_mode) {
        int64_t pcr = av_rescale(ts->interleave_output_bytes + 11,
                                 8 * PCR_TIME_BASE, ts->mux_rate) +
                      ts->first_pcr;
        uint32_t tp_extra_header = pcr % 0x3fffffff;
        tp_extra_header = AV_RB32(&tp_extra_header);
        avio_write(s->pb, (unsigned char *)&tp_extra_header,
                   sizeof(tp_extra_header));
    }
    avio_write(s->pb, packet, TS_PACKET_SIZE);
    ts->interleave_output_bytes += TS_PACKET_SIZE;
}

/* Drain all interleave buffers in proportional round-robin order.
 * Called after mpegts_write_pes() returns — all TS packets for the
 * current PES are buffered and ready to be interleaved with packets
 * from other streams' buffers. */
static void interleave_drain(AVFormatContext *s)
{
    MpegTSWrite *ts = s->priv_data;
    int total, chunk, i;

    /* Count total buffered packets */
    total = ts->interleave_si_count;
    for (i = 0; i < s->nb_streams; i++) {
        MpegTSWriteStream *ts_st = s->streams[i]->priv_data;
        if (!ts_st) continue;
        total += ts_st->interleave_buf_count;
    }

    if (total == 0)
        return;

    chunk = ts->ts_interleave_chunk;

    /* Round-robin drain.  Each round emits, in order:
     *   1) at most 1 SI packet (PAT/PMT/SDT/NIT),
     *   2) up to `chunk` video packets (back-to-back — video Rxn
     *      exceeds muxrate so TBn never accumulates),
     *   3) at most ONE packet from each audio stream that has content.
     *
     * The per-packet round-robin in step 3 enforces the H.222.0 Annex D
     * T-STD constraint: consecutive packets of any single audio PID are
     * separated by at least (other-audio-streams-with-content + chunk + 1)
     * packets, giving TBn time to drain at its per-codec Rxn.
     *
     * Concretely, at muxrate = 23.8 Mbps a single TS packet spans 63.2 µs;
     * AAC 5.1 Rxn per Annex Q (1.2 × 576 kbps × 8 ch) = 5.53 Mbps drains
     * 184 B in 266 µs, so same-PID packets must be spaced ≥ 266 µs apart.
     * A single round of 1 SI + 8 video + N audio = (9 + N) × 63.2 µs ≥
     * 568 µs for N ≥ 0, which satisfies the bound unconditionally.
     *
     * We intentionally do NOT drain multiple packets of the same audio PID
     * in one round — even when the buffer has accumulated (e.g. after a
     * stream-loop boundary).  The outer while-loop re-enters the video
     * and SI phases between each audio pass, which keeps same-PID spacing
     * bounded regardless of how long one stream has been starving
     * relative to others. */
    while (total > 0) {
        int emitted_this_round = 0;
        int non_audio_emitted = 0;

        /* 1) SI packet — emit 1 per round.  The SI buffer holds two
         *    distinct classes of packets, intermixed in FIFO order:
         *
         *     (a) PSI (PAT/PMT/SDT/NIT): decoder-visible, flow through
         *         TBsys with Rxn = 1 Mbps (H.222.0 clause 2.4.2.4).
         *         To satisfy P3.3 for TBsys, a 184-byte PSI payload
         *         needs 184·8/Rxn = 1.472 ms of wall-clock drain time
         *         before the next PSI arrives — otherwise TBsys
         *         accumulates.  At muxrate=23.8 Mbps that translates
         *         to a minimum spacing of ~4400 bytes of output.
         *
         *     (b) Null packets (PID 0x1FFF): never demultiplexed,
         *         never touch any decoder buffer.  Emit freely.
         *
         *    Because PSI always precedes its trailing nulls in the
         *    FIFO (both are enqueued within the same mpegts_write_pes
         *    call, PSI first via retransmit_si_info, nulls after via
         *    mpegts_insert_null_packet), gating PSI on spacing never
         *    stalls a null behind it.  If the front is PSI and spacing
         *    isn't met, we just skip SI emission this round — the PSI
         *    emits in a later round once enough bytes have gone out. */
        {
            int did_si = 0;
            if (ts->interleave_si_count > 0) {
                const uint8_t *front = ts->interleave_si_buf;
                int front_pid = ((front[1] & 0x1f) << 8) | front[2];
                int is_null = (front_pid == 0x1FFF);
                int spacing_ok = 1;
                if (!is_null) {
                    int64_t gap = ts->interleave_output_bytes -
                                  ts->last_psi_output_bytes;
                    int64_t min_gap = (int64_t)TS_PACKET_SIZE *
                                      ts->mux_rate / 1000000;
                    spacing_ok = (gap >= min_gap);
                }
                if (is_null || spacing_ok) {
                    interleave_write_raw(s, ts->interleave_si_buf);
                    ts->interleave_si_count--;
                    if (ts->interleave_si_count > 0)
                        memmove(ts->interleave_si_buf,
                                ts->interleave_si_buf + TS_PACKET_SIZE,
                                (int64_t)ts->interleave_si_count * TS_PACKET_SIZE);
                    total--;
                    emitted_this_round++;
                    did_si = 1;
                    if (!is_null) {
                        non_audio_emitted++;
                        ts->last_psi_output_bytes = ts->interleave_output_bytes;
                    }
                }
            }

            /* 1b) Synthetic-null spacer — when the SI buffer is empty (or
             *    its front is a gated PSI), and we are about to emit video
             *    this round, inject a null to guarantee every video chunk
             *    is preceded by a non-video packet.  Without this, drains
             *    triggered by single-stream overflow (buf_count > chunk*4)
             *    emit chunk after chunk of same-PID video with no spacer,
             *    producing 100+ packet runs of one PID on the wire and
             *    pushing the receiver's video TBn past the 512 B Annex C
             *    limit when UDP cadence is slightly irregular. */
            if (!did_si) {
                int any_video_pending = 0;
                int k;
                for (k = 0; k < s->nb_streams; k++) {
                    MpegTSWriteStream *ts_st2 = s->streams[k]->priv_data;
                    if (ts_st2 && ts_st2->interleave_buf_count > 0 &&
                        s->streams[k]->codecpar->codec_type ==
                            AVMEDIA_TYPE_VIDEO) {
                        any_video_pending = 1;
                        break;
                    }
                }
                if (any_video_pending) {
                    uint8_t null_pkt[TS_PACKET_SIZE];
                    null_pkt[0] = SYNC_BYTE;
                    null_pkt[1] = (NULL_PID >> 8) & 0x1F;
                    null_pkt[2] = NULL_PID & 0xFF;
                    null_pkt[3] = 0x10;  /* payload-only, cc=0 */
                    memset(null_pkt + 4, STUFFING_BYTE, TS_PACKET_SIZE - 4);
                    interleave_write_raw(s, null_pkt);
                    /* Keep total_size in sync with interleave_output_bytes:
                     * both now include this null. */
                    ts->total_size += TS_PACKET_SIZE;
                    emitted_this_round++;
                    /* Do NOT count towards non_audio_emitted — a null alone
                     * is not a valid audio-spacing partner per H.222.0
                     * Annex D (audio still needs SI or video in the round). */
                }
            }
        }

        /* 2) Video packets — up to `chunk` per round from each video
         *    stream.  Video Rxn (≥ 24 Mbps for H.264 High 4.0) exceeds
         *    muxrate so TBn for video never accumulates; back-to-back
         *    emission is T-STD-safe. */
        for (i = 0; i < s->nb_streams; i++) {
            MpegTSWriteStream *ts_st = s->streams[i]->priv_data;
            int emit, j;
            if (!ts_st || ts_st->interleave_buf_count == 0)
                continue;
            if (s->streams[i]->codecpar->codec_type != AVMEDIA_TYPE_VIDEO)
                continue;
            emit = FFMIN(chunk, ts_st->interleave_buf_count);
            for (j = 0; j < emit; j++) {
                interleave_write_raw(s, ts_st->interleave_buf);
                ts_st->interleave_buf_count--;
                if (ts_st->interleave_buf_count > 0)
                    memmove(ts_st->interleave_buf,
                            ts_st->interleave_buf + TS_PACKET_SIZE,
                            (int64_t)ts_st->interleave_buf_count * TS_PACKET_SIZE);
                total--;
                emitted_this_round++;
                non_audio_emitted++;
            }
        }

        /* 3) Audio — emit at most ONE packet per audio stream, but only
         *    if at least one non-audio packet (SI or video) was emitted
         *    in this round.  This is what enforces H.222.0 Annex D T-STD
         *    compliance: without this guard, once the SI and video buffers
         *    deplete mid-drain, subsequent rounds would emit only leftover
         *    audio, producing back-to-back same-PID packets at wire rate
         *    (63.2 µs at 23.8 Mbps).  For AAC 5.1 with Rxn = 5.53 Mbps
         *    (Annex Q: 1.2 × 576 kbps × 8 ch), consecutive same-PID packets
         *    need ≥ 266 µs of separation or TBn grows past the 512 B limit
         *    (TR 101 290 P3.3 Buffer_error).
         *
         *    When we break out of this loop with audio still buffered, the
         *    audio remains in its per-stream buffer and drains on the next
         *    drain call — which runs at the next AVPacket arrival, and by
         *    then either new video has been buffered (restoring spacing)
         *    or the single-stream threshold will re-fire this path. */
        /* Audio guard: audio packets emit only when interleaved with
         * non-audio content (SI/video) in the same round.  This enforces
         * the H.222.0 Annex D T-STD spacing requirement for per-PID
         * transport buffers (TBn) — consecutive same-PID audio packets
         * at wire rate overflow TBn (512 B limit) because the per-codec
         * Rxn drain rate is well below mux_rate.  When no non-audio
         * packet was emitted this round, skip audio — the audio stays
         * buffered for a future drain with non-audio content.  We do NOT
         * break the drain: SI content (particularly null packets that
         * implement CBR padding in interleave mode) must keep flowing,
         * or the SI buffer grows unboundedly. */
        if (non_audio_emitted == 0)
            goto end_round;

        for (i = 0; i < s->nb_streams; i++) {
            MpegTSWriteStream *ts_st = s->streams[i]->priv_data;
            if (!ts_st || ts_st->interleave_buf_count == 0)
                continue;
            if (s->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO)
                continue;
            interleave_write_raw(s, ts_st->interleave_buf);
            ts_st->interleave_buf_count--;
            if (ts_st->interleave_buf_count > 0)
                memmove(ts_st->interleave_buf,
                        ts_st->interleave_buf + TS_PACKET_SIZE,
                        (int64_t)ts_st->interleave_buf_count * TS_PACKET_SIZE);
            total--;
            emitted_this_round++;
        }

end_round:
        /* Safety: if nothing was emitted this round, break to avoid infinite loop */
        if (emitted_this_round == 0)
            break;
    }
}

/* ------------------------------------------------------------------ */

static int64_t get_pcr(const MpegTSWrite *ts)
{
    return av_rescale(ts->total_size + 11, 8 * PCR_TIME_BASE, ts->mux_rate) +
           ts->first_pcr;
}

static void write_packet(AVFormatContext *s, const uint8_t *packet)
{
    MpegTSWrite *ts = s->priv_data;

    /* When interleave buffering is active, enqueue instead of writing.
     * total_size is still advanced so that get_pcr() returns correct
     * values for subsequent packets within the same PES. */
    if (ts->ts_interleave && ts->interleave_buffering) {
        interleave_enqueue(s, packet);
        ts->total_size += TS_PACKET_SIZE;
        return;
    }

    /* CBR output pacing: pace each TS packet to maintain constant bitrate.
     * Uses the same fair-spinning pattern as the IPMX USB sender:
     * coarse-sleep all but the last CBR_PACE_SPIN_US, then busy-spin
     * for sub-millisecond precision.  This prevents burst delivery on UDP
     * and ensures TS packets are sent at a rate matching the declared
     * muxrate.  Without this, the muxer emits all packets for a frame in
     * one burst, then pauses until the next frame — causing receiver buffer
     * overflow on systems with limited de-jitter capacity. */
    if (ts->cbr_pacing && ts->mux_rate > 1) {
        int64_t now_us, target_us, wait;

        now_us = av_gettime_relative();

        if (ts->pacing_start_us == 0)
            ts->pacing_start_us = now_us;

        /* Pacing target derived from total_size — the same byte counter
         * that drives PCR (see get_pcr()).  This guarantees the pacing
         * clock and the PCR clock are locked: zero cumulative drift.
         *
         * target = epoch + total_size * 8 / mux_rate  (in microseconds)
         */
        target_us = ts->pacing_start_us +
                    av_rescale(ts->total_size, 8000000LL, ts->mux_rate);
        wait = target_us - now_us;

        if (wait > 0 && wait <= CBR_PACE_MAX_EARLY_US) {
            /* Coarse sleep for the bulk of the wait */
            if (wait > CBR_PACE_SPIN_US)
                av_usleep((unsigned)(wait - CBR_PACE_SPIN_US));
            /* Busy-spin the last fraction for precision */
            while (av_gettime_relative() < target_us)
                spin_hint();
        }
        /* If wait <= 0: we're behind schedule (demux seek stall, OS
         * scheduling delay).  Send immediately — the constant-rate
         * pacing will naturally re-converge as total_size advances.
         * If wait > CBR_PACE_MAX_EARLY_US: startup transient or clock
         * discontinuity — also send immediately and let it converge. */
    }

    if (ts->m2ts_mode) {
        int64_t pcr = get_pcr(s->priv_data);
        uint32_t tp_extra_header = pcr % 0x3fffffff;
        tp_extra_header = AV_RB32(&tp_extra_header);
        avio_write(s->pb, (unsigned char *) &tp_extra_header,
                   sizeof(tp_extra_header));
    }
    avio_write(s->pb, packet, TS_PACKET_SIZE);
    ts->total_size += TS_PACKET_SIZE;

    /* When interleave mode is active, packets can reach this non-buffered path
     * (e.g., CBR gap-fill null packets inserted before a PES arrives at a
     * stream-loop boundary).  interleave_output_bytes must track the actual
     * avio byte position so that PCR rewrites performed later by
     * interleave_write_raw() reflect the true file-byte position of
     * drained PCR-carrying packets.  Without this, every direct-written
     * packet shifts all subsequent drained PCRs by 188 * 8 / mux_rate
     * (~63 µs at 23.8 Mbps), producing P2.4 PCR_accuracy_error. */
    if (ts->ts_interleave)
        ts->interleave_output_bytes += TS_PACKET_SIZE;
}

static void section_write_packet(MpegTSSection *s, const uint8_t *packet)
{
    AVFormatContext *ctx = s->opaque;
    write_packet(ctx, packet);
}

static MpegTSService *mpegts_add_service(AVFormatContext *s, int sid,
                                         const AVDictionary *metadata,
                                         AVProgram *program)
{
    MpegTSWrite *ts = s->priv_data;
    MpegTSService *service;
    AVDictionaryEntry *title, *provider;
    char default_service_name[32];
    const char *service_name;
    const char *provider_name;

    title = av_dict_get(metadata, "service_name", NULL, 0);
    if (!title)
        title = av_dict_get(metadata, "title", NULL, 0);
    snprintf(default_service_name, sizeof(default_service_name), "%s%02d", DEFAULT_SERVICE_NAME, ts->nb_services + 1);
    service_name  = title ? title->value : default_service_name;
    provider      = av_dict_get(metadata, "service_provider", NULL, 0);
    provider_name = provider ? provider->value : DEFAULT_PROVIDER_NAME;

    service = av_mallocz(sizeof(MpegTSService));
    if (!service)
        return NULL;
    service->pmt.pid       = ts->pmt_start_pid + ts->nb_services;
    service->sid           = sid;
    service->pcr_pid       = 0x1fff;
    if (encode_str8(service->provider_name, provider_name) < 0 ||
        encode_str8(service->name, service_name) < 0) {
        av_log(s, AV_LOG_ERROR, "Too long service or provider name\n");
        goto fail;
    }
    ts->sdt.remaining -= 10 + service->provider_name[0] + service->name[0];
    if (ts->sdt.remaining < 0)
        goto fail;

    if (av_dynarray_add_nofree(&ts->services, &ts->nb_services, service) < 0)
        goto fail;

    service->pmt.write_packet = section_write_packet;
    service->pmt.opaque       = s;
    service->pmt.cc           = 15;
    service->pmt.discontinuity= ts->flags & MPEGTS_FLAG_DISCONT;
    service->program          = program;

    return service;
fail:
    av_free(service);
    return NULL;
}

static void enable_pcr_generation_for_stream(AVFormatContext *s, AVStream *pcr_st)
{
    MpegTSWrite *ts = s->priv_data;
    MpegTSWriteStream *ts_st = pcr_st->priv_data;

    if (ts->mux_rate > 1 || ts->pcr_period_ms >= 0) {
        int pcr_period_ms = ts->pcr_period_ms == -1 ? PCR_RETRANS_TIME : ts->pcr_period_ms;
        ts_st->pcr_period = av_rescale(pcr_period_ms, PCR_TIME_BASE, 1000);
    } else {
        /* By default, for VBR we select the highest multiple of frame duration which is less than 100 ms. */
        int64_t frame_period = 0;
        if (pcr_st->codecpar->codec_type == AVMEDIA_TYPE_AUDIO) {
            int frame_size = av_get_audio_frame_duration2(pcr_st->codecpar, 0);
            if (!frame_size) {
               av_log(s, AV_LOG_WARNING, "frame size not set\n");
               frame_size = 512;
            }
            frame_period = av_rescale_rnd(frame_size, PCR_TIME_BASE, pcr_st->codecpar->sample_rate, AV_ROUND_UP);
        } else if (pcr_st->avg_frame_rate.num) {
            frame_period = av_rescale_rnd(pcr_st->avg_frame_rate.den, PCR_TIME_BASE, pcr_st->avg_frame_rate.num, AV_ROUND_UP);
        }
        if (frame_period > 0 && frame_period <= PCR_TIME_BASE / 10)
            ts_st->pcr_period = frame_period * (PCR_TIME_BASE / 10 / frame_period);
        else
            ts_st->pcr_period = 1;
    }

    // output a PCR as soon as possible
    ts_st->last_pcr   = ts->first_pcr - ts_st->pcr_period;
}

static void select_pcr_streams(AVFormatContext *s)
{
    MpegTSWrite *ts = s->priv_data;

    for (int i = 0; i < ts->nb_services; i++) {
        MpegTSService *service = ts->services[i];
        AVStream *pcr_st = NULL;
        AVProgram *program = service->program;
        int nb_streams = program ? program->nb_stream_indexes : s->nb_streams;

        for (int j = 0; j < nb_streams; j++) {
            AVStream *st = s->streams[program ? program->stream_index[j] : j];
            if (!pcr_st ||
                pcr_st->codecpar->codec_type != AVMEDIA_TYPE_VIDEO && st->codecpar->codec_type == AVMEDIA_TYPE_VIDEO)
            {
                pcr_st = st;
            }
        }

        if (pcr_st) {
            MpegTSWriteStream *ts_st = pcr_st->priv_data;
            service->pcr_pid = ts_st->pid;
            enable_pcr_generation_for_stream(s, pcr_st);
            av_log(s, AV_LOG_VERBOSE, "service %i using PCR in pid=%i, pcr_period=%"PRId64"ms\n",
                service->sid, service->pcr_pid, av_rescale(ts_st->pcr_period, 1000, PCR_TIME_BASE));
        }
    }
}

static int mpegts_init(AVFormatContext *s)
{
    MpegTSWrite *ts = s->priv_data;
    AVDictionaryEntry *provider;
    const char *provider_name;
    int i, j;
    int ret;

    if (ts->m2ts_mode == -1) {
        if (av_match_ext(s->url, "m2ts")) {
            ts->m2ts_mode = 1;
        } else {
            ts->m2ts_mode = 0;
        }
    }

    ts->m2ts_video_pid   = M2TS_VIDEO_PID;
    ts->m2ts_audio_pid   = M2TS_AUDIO_START_PID;
    ts->m2ts_pgssub_pid  = M2TS_PGSSUB_START_PID;
    ts->m2ts_textsub_pid = M2TS_TEXTSUB_PID;

    if (ts->m2ts_mode) {
        ts->pmt_start_pid = M2TS_PMT_PID;
        if (s->nb_programs > 1) {
            av_log(s, AV_LOG_ERROR, "Only one program is allowed in m2ts mode!\n");
            return AVERROR(EINVAL);
        }
    }

    if (s->max_delay < 0) /* Not set by the caller */
        s->max_delay = 0;

    // round up to a whole number of TS packets
    ts->pes_payload_size = (ts->pes_payload_size + 14 + 183) / 184 * 184 - 14;

    ts->sdt.remaining    = SECTION_LENGTH - 3;

    if (!s->nb_programs) {
        /* allocate a single DVB service */
        if (!mpegts_add_service(s, ts->service_id, s->metadata, NULL))
            return AVERROR(ENOMEM);
    } else {
        for (i = 0; i < s->nb_programs; i++) {
            AVProgram *program = s->programs[i];
            if (!mpegts_add_service(s, program->id, program->metadata, program))
                return AVERROR(ENOMEM);
        }
    }

    ts->pat.pid          = PAT_PID;
    /* Initialize at 15 so that it wraps and is equal to 0 for the
     * first packet we write. */
    ts->pat.cc           = 15;
    ts->pat.discontinuity= ts->flags & MPEGTS_FLAG_DISCONT;
    ts->pat.write_packet = section_write_packet;
    ts->pat.opaque       = s;

    ts->sdt.pid          = SDT_PID;
    ts->sdt.cc           = 15;
    ts->sdt.discontinuity= ts->flags & MPEGTS_FLAG_DISCONT;
    ts->sdt.write_packet = section_write_packet;
    ts->sdt.opaque       = s;

    ts->nit.pid          = NIT_PID;
    ts->nit.cc           = 15;
    ts->nit.discontinuity= ts->flags & MPEGTS_FLAG_DISCONT;
    ts->nit.write_packet = section_write_packet;
    ts->nit.opaque       = s;

    ts->pkt = ffformatcontext(s)->pkt;

    /* assign pids to each stream */
    for (i = 0; i < s->nb_streams; i++) {
        AVStream *st = s->streams[i];
        MpegTSWriteStream *ts_st;

        ts_st = av_mallocz(sizeof(MpegTSWriteStream));
        if (!ts_st) {
            return AVERROR(ENOMEM);
        }
        st->priv_data = ts_st;

        avpriv_set_pts_info(st, 33, 1, 90000);

        ts_st->payload = av_mallocz(ts->pes_payload_size);
        if (!ts_st->payload) {
            return AVERROR(ENOMEM);
        }

        /* Allocate interleave buffer if ts_interleave is enabled */
        if (ts->ts_interleave) {
            int initial_cap = (st->codecpar->codec_type == AVMEDIA_TYPE_VIDEO)
                              ? 2048 : 128;
            ts_st->interleave_buf = av_malloc((int64_t)initial_cap * TS_PACKET_SIZE);
            if (!ts_st->interleave_buf)
                return AVERROR(ENOMEM);
            ts_st->interleave_buf_capacity = initial_cap;
            ts_st->interleave_buf_count = 0;
        }

        /* MPEG pid values < 16 are reserved. Applications which set st->id in
         * this range are assigned a calculated pid. */
        if (st->id < 16) {
            if (ts->m2ts_mode) {
                switch (st->codecpar->codec_type) {
                case AVMEDIA_TYPE_VIDEO:
                    ts_st->pid = ts->m2ts_video_pid++;
                    break;
                case AVMEDIA_TYPE_AUDIO:
                    ts_st->pid = ts->m2ts_audio_pid++;
                    break;
                case AVMEDIA_TYPE_SUBTITLE:
                    switch (st->codecpar->codec_id) {
                    case AV_CODEC_ID_HDMV_PGS_SUBTITLE:
                        ts_st->pid = ts->m2ts_pgssub_pid++;
                        break;
                    case AV_CODEC_ID_HDMV_TEXT_SUBTITLE:
                        ts_st->pid = ts->m2ts_textsub_pid++;
                        break;
                    }
                    break;
                }
                if (ts->m2ts_video_pid   > M2TS_VIDEO_PID + 1          ||
                    ts->m2ts_audio_pid   > M2TS_AUDIO_START_PID + 32   ||
                    ts->m2ts_pgssub_pid  > M2TS_PGSSUB_START_PID + 32  ||
                    ts->m2ts_textsub_pid > M2TS_TEXTSUB_PID + 1        ||
                    ts_st->pid < 16) {
                    av_log(s, AV_LOG_ERROR, "Cannot automatically assign PID for stream %d\n", st->index);
                    return AVERROR(EINVAL);
                }
            } else {
                ts_st->pid = ts->start_pid + i;
            }
        } else {
            ts_st->pid = st->id;
        }
        if (ts_st->pid >= 0x1FFF) {
            av_log(s, AV_LOG_ERROR,
                   "Invalid stream id %d, must be less than 8191\n", st->id);
            return AVERROR(EINVAL);
        }
        for (j = 0; j < ts->nb_services; j++) {
            if (ts->services[j]->pmt.pid > LAST_OTHER_PID) {
                av_log(s, AV_LOG_ERROR,
                       "Invalid PMT PID %d, must be less than %d\n", ts->services[j]->pmt.pid, LAST_OTHER_PID + 1);
                return AVERROR(EINVAL);
            }
            if (ts_st->pid == ts->services[j]->pmt.pid) {
                av_log(s, AV_LOG_ERROR, "PID %d cannot be both elementary and PMT PID\n", ts_st->pid);
                return AVERROR(EINVAL);
            }
        }
        for (j = 0; j < i; j++) {
            MpegTSWriteStream *ts_st_prev = s->streams[j]->priv_data;
            if (ts_st_prev->pid == ts_st->pid) {
                av_log(s, AV_LOG_ERROR, "Duplicate stream id %d\n", ts_st->pid);
                return AVERROR(EINVAL);
            }
        }
        ts_st->payload_pts     = AV_NOPTS_VALUE;
        ts_st->payload_dts     = AV_NOPTS_VALUE;
        ts_st->cc              = 15;
        ts_st->discontinuity   = ts->flags & MPEGTS_FLAG_DISCONT;
        if (st->codecpar->codec_id == AV_CODEC_ID_AAC &&
            st->codecpar->extradata_size > 0) {
            AVStream *ast;
            ts_st->amux = avformat_alloc_context();
            if (!ts_st->amux) {
                return AVERROR(ENOMEM);
            }
            ts_st->amux->oformat =
                av_guess_format((ts->flags & MPEGTS_FLAG_AAC_LATM) ? "latm" : "adts",
                                NULL, NULL);
            if (!ts_st->amux->oformat) {
                return AVERROR(EINVAL);
            }
            if (!(ast = avformat_new_stream(ts_st->amux, NULL))) {
                return AVERROR(ENOMEM);
            }
            ret = avcodec_parameters_copy(ast->codecpar, st->codecpar);
            if (ret != 0)
                return ret;
            ast->time_base = st->time_base;
            ret = avformat_write_header(ts_st->amux, NULL);
            if (ret < 0)
                return ret;
        }
        if (st->codecpar->codec_id == AV_CODEC_ID_OPUS) {
            ts_st->opus_pending_trim_start = st->codecpar->initial_padding * 48000 / st->codecpar->sample_rate;
        }
    }

    if (ts->copyts < 1)
        ts->first_pcr = av_rescale(s->max_delay, PCR_TIME_BASE, AV_TIME_BASE);

    select_pcr_streams(s);

    ts->last_pat_ts = AV_NOPTS_VALUE;
    ts->last_sdt_ts = AV_NOPTS_VALUE;
    ts->last_nit_ts = AV_NOPTS_VALUE;
    ts->pat_period = av_rescale(ts->pat_period_us, PCR_TIME_BASE, AV_TIME_BASE);
    ts->pat_min_gap = av_rescale(ts->pat_min_gap_us, PCR_TIME_BASE, AV_TIME_BASE);
    ts->sdt_period = av_rescale(ts->sdt_period_us, PCR_TIME_BASE, AV_TIME_BASE);
    ts->nit_period = av_rescale(ts->nit_period_us, PCR_TIME_BASE, AV_TIME_BASE);

    /* assign provider name */
    provider = av_dict_get(s->metadata, "service_provider", NULL, 0);
    provider_name = provider ? provider->value : DEFAULT_PROVIDER_NAME;
    if (encode_str8(ts->provider_name, provider_name) < 0) {
        av_log(s, AV_LOG_ERROR, "Too long provider name\n");
        return AVERROR(EINVAL);
    }

    if (ts->mux_rate == 1)
        av_log(s, AV_LOG_VERBOSE, "muxrate VBR, ");
    else
        av_log(s, AV_LOG_VERBOSE, "muxrate %d, ", ts->mux_rate);
    av_log(s, AV_LOG_VERBOSE,
           "sdt every %"PRId64" ms, pat/pmt every %"PRId64" ms",
           av_rescale(ts->sdt_period, 1000, PCR_TIME_BASE),
           av_rescale(ts->pat_period, 1000, PCR_TIME_BASE));
    if (ts->flags & MPEGTS_FLAG_NIT)
        av_log(s, AV_LOG_VERBOSE, ", nit every %"PRId64" ms", av_rescale(ts->nit_period, 1000, PCR_TIME_BASE));
    av_log(s, AV_LOG_VERBOSE, "\n");

    /* PEP encryption initialization.
     * Check both muxer private data (ts->pep_psk) and AVFormatContext (s->pep_psk)
     * since the option may be set on either depending on how the CLI routes it. */
    {
        const char *psk = (ts->pep_psk && ts->pep_psk[0]) ? ts->pep_psk :
                          (s->pep_psk && s->pep_psk[0]) ? s->pep_psk : NULL;
        const char *mode = ts->pep_mode ? ts->pep_mode : s->pep_mode;
        const char *iv = ts->pep_iv ? ts->pep_iv : s->pep_iv;
        const char *kg = ts->pep_key_generator ? ts->pep_key_generator : s->pep_key_generator;
        const char *kv = ts->pep_key_version ? ts->pep_key_version : s->pep_key_version;

    if (psk) {
        RtpPepContext *pep_ctx = av_mallocz(sizeof(*pep_ctx));
        if (!pep_ctx)
            return AVERROR(ENOMEM);
        if (rtp_pep_init(pep_ctx, s, psk, mode, iv, kg, kv, s->pep_rtp_kv) < 0 ||
            av_ipmx_pep_mode_has_cmac(pep_ctx->mode)) {
            if (av_ipmx_pep_mode_has_cmac(pep_ctx->mode))
                av_log(s, AV_LOG_ERROR, "PEP: CMAC modes not supported for MPEG2-TS over UDP\n");

            av_free(pep_ctx);
            return AVERROR(EINVAL);
        }
        ts->pep_context = pep_ctx;
        ts->pep_input_ctr = 0;
        ts->pep_key_ver = pep_ctx->key_version;
        /* Wire kv_rotate for MPEG2-TS (boundary = is_start PES packet) */
        if (s->pep_kv_rotate > 0 && s->pep_rtp_kv) {
            pep_ctx->kv_rotate_slices = s->pep_kv_rotate;
            av_log(s, AV_LOG_INFO, "MPEG-TS PEP key rotation every %d PES boundaries\n",
                   pep_ctx->kv_rotate_slices);
        }
        av_log(s, AV_LOG_INFO, "MPEG-TS PEP encryption enabled\n");
    }
    }

    /* Allocate shared SI interleave buffer */
    if (ts->ts_interleave) {
        int si_cap = 512;
        ts->interleave_si_buf = av_malloc((int64_t)si_cap * TS_PACKET_SIZE);
        if (!ts->interleave_si_buf)
            return AVERROR(ENOMEM);
        ts->interleave_si_capacity = si_cap;
        ts->interleave_si_count = 0;
        ts->interleave_output_bytes = 0;
        ts->last_psi_output_bytes = INT64_MIN / 2;
    }

    if (ts->cbr_pacing_threaded) {
        int ret;
        if (!ts->cbr_pacing) {
            av_log(s, AV_LOG_ERROR,
                   "cbr_pacing_threaded requires cbr_pacing=1\n");
            return AVERROR(EINVAL);
        }
        /* Disable libavformat's per-packet FLUSH_POINT marker.  In
         * threaded mode the output thread fills the avio buffer
         * asynchronously, so a flush-after-each-muxer-write_packet
         * cuts UDP datagrams into partials mid-accumulation.  Only
         * override the auto default (-1); respect explicit user
         * choices of 0 or 1. */
        if (s->flush_packets == -1)
            s->flush_packets = 0;
        if ((ret = mpegts_out_thread_start(s)) < 0) {
            av_log(s, AV_LOG_ERROR,
                   "failed to start mpegts output thread: %d\n", ret);
            return ret;
        }
    }

    return 0;
}

/* send SDT, NIT, PAT and PMT tables regularly */
static void retransmit_si_info(AVFormatContext *s, int force_pat, int force_sdt, int force_nit, int64_t pcr)
{
    MpegTSWrite *ts = s->priv_data;
    int i;

    if ((pcr != AV_NOPTS_VALUE && ts->last_sdt_ts == AV_NOPTS_VALUE) ||
        (pcr != AV_NOPTS_VALUE && pcr - ts->last_sdt_ts >= ts->sdt_period) ||
        force_sdt
    ) {
        if (pcr != AV_NOPTS_VALUE)
            ts->last_sdt_ts = FFMAX(pcr, ts->last_sdt_ts);
        mpegts_write_sdt(s);
    }
    if ((pcr != AV_NOPTS_VALUE && ts->last_pat_ts == AV_NOPTS_VALUE) ||
        (pcr != AV_NOPTS_VALUE && pcr - ts->last_pat_ts >= ts->pat_period) ||
        force_pat) {
        if (pcr != AV_NOPTS_VALUE)
            ts->last_pat_ts = FFMAX(pcr, ts->last_pat_ts);
        mpegts_write_pat(s);
        for (i = 0; i < ts->nb_services; i++)
            mpegts_write_pmt(s, ts->services[i]);
    }
    if ((pcr != AV_NOPTS_VALUE && ts->last_nit_ts == AV_NOPTS_VALUE) ||
        (pcr != AV_NOPTS_VALUE && pcr - ts->last_nit_ts >= ts->nit_period) ||
        force_nit
    ) {
        if (pcr != AV_NOPTS_VALUE)
            ts->last_nit_ts = FFMAX(pcr, ts->last_nit_ts);
        if (ts->flags & MPEGTS_FLAG_NIT)
            mpegts_write_nit(s);
    }
}

static int write_pcr_bits(uint8_t *buf, int64_t pcr)
{
    int64_t pcr_low = pcr % SYSTEM_CLOCK_FREQUENCY_DIVISOR, pcr_high = pcr / SYSTEM_CLOCK_FREQUENCY_DIVISOR;

    *buf++ = pcr_high >> 25;
    *buf++ = pcr_high >> 17;
    *buf++ = pcr_high >>  9;
    *buf++ = pcr_high >>  1;
    *buf++ = pcr_high <<  7 | pcr_low >> 8 | 0x7e;
    *buf++ = pcr_low;

    return 6;
}

/* Write a single null transport stream packet */
static void mpegts_insert_null_packet(AVFormatContext *s)
{
    uint8_t *q;
    uint8_t buf[TS_PACKET_SIZE];

    q    = buf;
    *q++ = SYNC_BYTE;
    *q++ = 0x00 | (NULL_PID >> 8);
    *q++ = NULL_PID & 0xff;
    *q++ = 0x10;
    memset(q, STUFFING_BYTE, TS_PACKET_SIZE - (q - buf)); /* data_bytes may be assigned any value */
    write_packet(s, buf);
}

/* Write a single transport stream packet with a PCR and no payload */
static void mpegts_insert_pcr_only(AVFormatContext *s, AVStream *st)
{
    MpegTSWrite *ts = s->priv_data;
    MpegTSWriteStream *ts_st = st->priv_data;
    uint8_t *q;
    uint8_t buf[TS_PACKET_SIZE];

    q    = buf;
    *q++ = SYNC_BYTE;
    *q++ = ts_st->pid >> 8;
    *q++ = ts_st->pid;
    *q++ = 0x20 | ts_st->cc;   /* Adaptation only */
    /* Continuity Count field does not increment (see 13818-1 section 2.4.3.3) */
    *q++ = TS_PACKET_SIZE - 5; /* Adaptation Field Length */
    *q++ = 0x10;               /* Adaptation flags: PCR present */
    if (ts_st->discontinuity) {
        q[-1] |= 0x80;
        ts_st->discontinuity = 0;
    }

    /* PCR coded into 6 bytes */
    q += write_pcr_bits(q, get_pcr(ts));

    /* stuffing bytes */
    memset(q, STUFFING_BYTE, TS_PACKET_SIZE - (q - buf));
    write_packet(s, buf);
}

static void write_pts(uint8_t *q, int fourbits, int64_t pts)
{
    int val;

    val  = fourbits << 4 | (((pts >> 30) & 0x07) << 1) | 1;
    *q++ = val;
    val  = (((pts >> 15) & 0x7fff) << 1) | 1;
    *q++ = val >> 8;
    *q++ = val;
    val  = (((pts) & 0x7fff) << 1) | 1;
    *q++ = val >> 8;
    *q++ = val;
}

/* Set an adaptation field flag in an MPEG-TS packet*/
static void set_af_flag(uint8_t *pkt, int flag)
{
    // expect at least one flag to set
    av_assert0(flag);

    if ((pkt[3] & 0x20) == 0) {
        // no AF yet, set adaptation field flag
        pkt[3] |= 0x20;
        // 1 byte length, no flags
        pkt[4] = 1;
        pkt[5] = 0;
    }
    pkt[5] |= flag;
}

/* Extend the adaptation field by size bytes */
static void extend_af(uint8_t *pkt, int size)
{
    // expect already existing adaptation field
    av_assert0(pkt[3] & 0x20);
    pkt[4] += size;
}

/* Get a pointer to MPEG-TS payload (right after TS packet header) */
static uint8_t *get_ts_payload_start(uint8_t *pkt)
{
    if (pkt[3] & 0x20)
        return pkt + 5 + pkt[4];
    else
        return pkt + 4;
}

static int get_pes_stream_id(AVFormatContext *s, AVStream *st, int stream_id, int *async)
{
    MpegTSWrite *ts = s->priv_data;
    *async = 0;
    if (st->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
        if (st->codecpar->codec_id == AV_CODEC_ID_DIRAC)
            return STREAM_ID_EXTENDED_STREAM_ID;
        else
            return STREAM_ID_VIDEO_STREAM_0;
    } else if (st->codecpar->codec_type == AVMEDIA_TYPE_AUDIO &&
               (st->codecpar->codec_id == AV_CODEC_ID_MP2 ||
                st->codecpar->codec_id == AV_CODEC_ID_MP3 ||
                st->codecpar->codec_id == AV_CODEC_ID_AAC)) {
        return STREAM_ID_AUDIO_STREAM_0;
    } else if (st->codecpar->codec_type == AVMEDIA_TYPE_AUDIO &&
                st->codecpar->codec_id == AV_CODEC_ID_AC3 &&
                ts->m2ts_mode) {
        return STREAM_ID_EXTENDED_STREAM_ID;
    } else if (st->codecpar->codec_type == AVMEDIA_TYPE_DATA &&
               st->codecpar->codec_id == AV_CODEC_ID_TIMED_ID3) {
        return STREAM_ID_PRIVATE_STREAM_1;
    } else if (st->codecpar->codec_type == AVMEDIA_TYPE_DATA) {
        if (st->codecpar->codec_id == AV_CODEC_ID_SMPTE_KLV &&
            stream_id == STREAM_ID_PRIVATE_STREAM_1) /* asynchronous KLV */
            *async = 1;
        return stream_id != -1 ? stream_id : STREAM_ID_METADATA_STREAM;
    } else {
        return STREAM_ID_PRIVATE_STREAM_1;
    }
}

/* Add a PES header to the front of the payload, and segment into an integer
 * number of TS packets. The final TS packet is padded using an oversized
 * adaptation header to exactly fill the last TS packet.
 * NOTE: 'payload' contains a complete PES payload. */
static void mpegts_write_pes(AVFormatContext *s, AVStream *st,
                             const uint8_t *payload, int payload_size,
                             int64_t pts, int64_t dts, int key, int stream_id)
{
    MpegTSWriteStream *ts_st = st->priv_data;
    MpegTSWrite *ts = s->priv_data;
    uint8_t buf[TS_PACKET_SIZE];
    uint8_t *q;
    int val, is_start, is_start_saved, len, header_len, write_pcr, flags;
    int afc_len, stuffing_len;
    int is_dvb_subtitle = (st->codecpar->codec_id == AV_CODEC_ID_DVB_SUBTITLE);
    int is_dvb_teletext = (st->codecpar->codec_id == AV_CODEC_ID_DVB_TELETEXT);
    int64_t delay = av_rescale(s->max_delay, 90000, AV_TIME_BASE);
    int force_pat = st->codecpar->codec_type == AVMEDIA_TYPE_VIDEO && key && !ts_st->prev_payload_key;
    /* TR 101 290 P3.2: minimum SI repetition interval (25 ms by spec).
     * Suppress force_pat if we sent PAT/PMT less than pat_min_gap ago
     * to avoid violating the minimum repetition rate. The floor is
     * configurable via -pat_min_gap (default 25 ms) so deployments with
     * jittery network paths can raise it for arrival-side robustness. */
    int force_sdt = 0;
    int force_nit = 0;
    if (force_pat && ts->last_pat_ts != AV_NOPTS_VALUE) {
        int64_t pcr_now = get_pcr(ts);
        if (pcr_now - ts->last_pat_ts < ts->pat_min_gap)
            force_pat = 0;
    }

    av_assert0(ts_st->payload != buf || st->codecpar->codec_type != AVMEDIA_TYPE_VIDEO);
    if (ts->flags & MPEGTS_FLAG_PAT_PMT_AT_FRAMES && st->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
        force_pat = 1;
    }

    if (ts->flags & MPEGTS_FLAG_REEMIT_PAT_PMT) {
        force_pat = 1;
        force_sdt = 1;
        force_nit = 1;
        ts->flags &= ~MPEGTS_FLAG_REEMIT_PAT_PMT;
    }

    is_start = 1;
    is_start_saved = 0;
    while (payload_size > 0) {
        int64_t pcr = AV_NOPTS_VALUE;
        if (ts->mux_rate > 1)
            pcr = get_pcr(ts);
        else if (dts != AV_NOPTS_VALUE)
            pcr = (dts - delay) * SYSTEM_CLOCK_FREQUENCY_DIVISOR;

        retransmit_si_info(s, force_pat, force_sdt, force_nit, pcr);
        force_pat = 0;
        force_sdt = 0;
        force_nit = 0;

        write_pcr = 0;
        if (ts->mux_rate > 1) {
            /* Send PCR packets for all PCR streams if needed */
            pcr = get_pcr(ts);
            if (pcr >= ts->next_pcr) {
                int64_t next_pcr = INT64_MAX;
                for (int i = 0; i < s->nb_streams; i++) {
                    /* Make the current stream the last, because for that we
                     * can insert the pcr into the payload later */
                    int st2_index = i < st->index ? i : (i + 1 == s->nb_streams ? st->index : i + 1);
                    AVStream *st2 = s->streams[st2_index];
                    MpegTSWriteStream *ts_st2 = st2->priv_data;
                    if (ts_st2->pcr_period) {
                        if (pcr - ts_st2->last_pcr >= ts_st2->pcr_period) {
                            ts_st2->last_pcr = FFMAX(pcr - ts_st2->pcr_period, ts_st2->last_pcr + ts_st2->pcr_period);
                            if (st2 != st) {
                                mpegts_insert_pcr_only(s, st2);
                                pcr = get_pcr(ts);
                            } else {
                                write_pcr = 1;
                            }
                        }
                        next_pcr = FFMIN(next_pcr, ts_st2->last_pcr + ts_st2->pcr_period);
                    }
                }
                ts->next_pcr = next_pcr;
            }
            if (dts != AV_NOPTS_VALUE && (dts - pcr / SYSTEM_CLOCK_FREQUENCY_DIVISOR) > delay) {
                /* pcr insert gets priority over null packet insert */
                if (write_pcr)
                    mpegts_insert_pcr_only(s, st);
                else
                    mpegts_insert_null_packet(s);
                /* recalculate write_pcr and possibly retransmit si_info */
                continue;
            }
        } else if (ts_st->pcr_period && pcr != AV_NOPTS_VALUE) {
            if (pcr - ts_st->last_pcr >= ts_st->pcr_period && is_start) {
                ts_st->last_pcr = FFMAX(pcr - ts_st->pcr_period, ts_st->last_pcr + ts_st->pcr_period);
                write_pcr = 1;
            }
        }

        /* prepare packet header */
        q    = buf;
        *q++ = SYNC_BYTE;
        val  = ts_st->pid >> 8;
        if (ts->m2ts_mode && st->codecpar->codec_id == AV_CODEC_ID_AC3)
            val |= 0x20;
        if (is_start)
            val |= 0x40;
        *q++      = val;
        *q++      = ts_st->pid;
        ts_st->cc = ts_st->cc + 1 & 0xf;
        *q++      = 0x10 | ts_st->cc; // payload indicator + CC
        if (ts_st->discontinuity) {
            set_af_flag(buf, 0x80);
            q = get_ts_payload_start(buf);
            ts_st->discontinuity = 0;
        }
        if (!(ts->flags & MPEGTS_FLAG_OMIT_RAI) &&
            key && is_start && pts != AV_NOPTS_VALUE &&
            !is_dvb_teletext /* adaptation+payload forbidden for teletext (ETSI EN 300 472 V1.3.1 4.1) */) {
            // set Random Access for key frames
            if (ts_st->pcr_period)
                write_pcr = 1;
            set_af_flag(buf, 0x40);
            q = get_ts_payload_start(buf);
        }
        if (write_pcr) {
            set_af_flag(buf, 0x10);
            q = get_ts_payload_start(buf);
            // add 11, pcr references the last byte of program clock reference base
            if (dts != AV_NOPTS_VALUE && dts < pcr / SYSTEM_CLOCK_FREQUENCY_DIVISOR)
                av_log(s, AV_LOG_WARNING, "dts < pcr, TS is invalid\n");
            extend_af(buf, write_pcr_bits(q, pcr));
            q = get_ts_payload_start(buf);
        }
        /* PEP: insert CTR header into adaptation field private data.
         * Only for data PES streams with eligible PIDs.
         * The extend_af shifts data, reducing available payload space;
         * the stuffing logic at packet end handles the remaining space. */
        if (ts->pep_context && ts_st->pid >= 0x0010 && ts_st->pid <= 0x1ffe) {
            int pep_pcr_sz = (buf[3] & 0x20 && buf[4] > 0 && (buf[5] & 0x10)) ? 6 : 0;
            set_af_flag(buf, 0x02); /* transport_private_data_flag */
            if (is_start) {
                /* Full header: private_data_length(1) + key_version(4) + input_ctr(8) */
                buf[6 + pep_pcr_sz] = 12;
                AV_WB32(buf + 6 + pep_pcr_sz + 1, ts->pep_key_ver);
                AV_WB64(buf + 6 + pep_pcr_sz + 5, ts->pep_input_ctr);
                extend_af(buf, 1 + 12);
            } else {
                /* Short header: private_data_length(1) + input_ctr_short(3) */
                buf[6 + pep_pcr_sz] = 3;
                buf[6 + pep_pcr_sz + 1] = (ts->pep_input_ctr >> 16) & 0xFF;
                buf[6 + pep_pcr_sz + 2] = (ts->pep_input_ctr >> 8) & 0xFF;
                buf[6 + pep_pcr_sz + 3] = ts->pep_input_ctr & 0xFF;
                extend_af(buf, 1 + 3);
            }
            q = get_ts_payload_start(buf);
        }
        if (is_start) {
            int pes_extension = 0;
            int pes_header_stuffing_bytes = 0;
            int async;
            /* write PES header */
            *q++ = 0x00;
            *q++ = 0x00;
            *q++ = 0x01;
            *q++ = stream_id = get_pes_stream_id(s, st, stream_id, &async);
            if (async)
                pts = dts = AV_NOPTS_VALUE;

            header_len = 0;

            if (stream_id != STREAM_ID_PROGRAM_STREAM_MAP &&
                stream_id != STREAM_ID_PADDING_STREAM &&
                stream_id != STREAM_ID_PRIVATE_STREAM_2 &&
                stream_id != STREAM_ID_ECM_STREAM &&
                stream_id != STREAM_ID_EMM_STREAM &&
                stream_id != STREAM_ID_PROGRAM_STREAM_DIRECTORY &&
                stream_id != STREAM_ID_DSMCC_STREAM &&
                stream_id != STREAM_ID_TYPE_E_STREAM) {

                flags      = 0;
                if (pts != AV_NOPTS_VALUE) {
                    header_len += 5;
                    flags      |= 0x80;
                }
                if (dts != AV_NOPTS_VALUE && pts != AV_NOPTS_VALUE && dts != pts) {
                    header_len += 5;
                    flags      |= 0x40;
                }
                if (st->codecpar->codec_type == AVMEDIA_TYPE_VIDEO &&
                    st->codecpar->codec_id == AV_CODEC_ID_DIRAC) {
                    /* set PES_extension_flag */
                    pes_extension = 1;
                    flags        |= 0x01;

                    /* One byte for PES2 extension flag +
                     * one byte for extension length +
                     * one byte for extension id */
                    header_len += 3;
                }
                /* for Blu-ray AC3 Audio the PES Extension flag should be as follow
                 * otherwise it will not play sound on blu-ray
                 */
                if (ts->m2ts_mode &&
                    st->codecpar->codec_type == AVMEDIA_TYPE_AUDIO &&
                    st->codecpar->codec_id == AV_CODEC_ID_AC3) {
                    /* set PES_extension_flag */
                    pes_extension = 1;
                    flags |= 0x01;
                    header_len += 3;
                }
                if (is_dvb_teletext) {
                    pes_header_stuffing_bytes = 0x24 - header_len;
                    header_len = 0x24;
                }
                len = payload_size + header_len + 3;
                /* 3 extra bytes should be added to DVB subtitle payload: 0x20 0x00 at the beginning and trailing 0xff */
                if (is_dvb_subtitle) {
                    len += 3;
                    payload_size++;
                }
                if (len > 0xffff)
                    len = 0;
                if (ts->omit_video_pes_length && st->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
                    len = 0;
                }
                *q++ = len >> 8;
                *q++ = len;
                val  = 0x80;
                /* data alignment indicator is required for subtitle and data streams */
                if (st->codecpar->codec_type == AVMEDIA_TYPE_SUBTITLE || st->codecpar->codec_type == AVMEDIA_TYPE_DATA)
                    val |= 0x04;
                *q++ = val;
                *q++ = flags;
                *q++ = header_len;
                if (pts != AV_NOPTS_VALUE) {
                    write_pts(q, flags >> 6, pts);
                    q += 5;
                }
                if (dts != AV_NOPTS_VALUE && pts != AV_NOPTS_VALUE && dts != pts) {
                    write_pts(q, 1, dts);
                    q += 5;
                }
                if (pes_extension && st->codecpar->codec_id == AV_CODEC_ID_DIRAC) {
                    flags = 0x01;  /* set PES_extension_flag_2 */
                    *q++  = flags;
                    *q++  = 0x80 | 0x01; /* marker bit + extension length */
                    /* Set the stream ID extension flag bit to 0 and
                     * write the extended stream ID. */
                    *q++ = 0x00 | 0x60;
                }
                /* For Blu-ray AC3 Audio Setting extended flags */
                if (ts->m2ts_mode &&
                    pes_extension &&
                    st->codecpar->codec_id == AV_CODEC_ID_AC3) {
                    flags = 0x01; /* set PES_extension_flag_2 */
                    *q++ = flags;
                    *q++ = 0x80 | 0x01; /* marker bit + extension length */
                    *q++ = 0x00 | 0x71; /* for AC3 Audio (specifically on blue-rays) */
                }


                if (is_dvb_subtitle) {
                    /* First two fields of DVB subtitles PES data:
                     * data_identifier: for DVB subtitle streams shall be coded with the value 0x20
                     * subtitle_stream_id: for DVB subtitle stream shall be identified by the value 0x00 */
                    *q++ = 0x20;
                    *q++ = 0x00;
                }
                if (is_dvb_teletext) {
                    memset(q, STUFFING_BYTE, pes_header_stuffing_bytes);
                    q += pes_header_stuffing_bytes;
                }
            } else {
                len = payload_size;
                *q++ = len >> 8;
                *q++ = len;
            }
            is_start_saved = 1; /* remember that this TS packet starts a new PES */
            is_start = 0;
        }
        /* header size */
        header_len = q - buf;
        /* data len */
        len = TS_PACKET_SIZE - header_len;
        if (len > payload_size)
            len = payload_size;
        stuffing_len = TS_PACKET_SIZE - header_len - len;
        if (stuffing_len > 0) {
            /* add stuffing with AFC */
            if (buf[3] & 0x20) {
                /* stuffing already present: increase its size */
                afc_len = buf[4] + 1;
                memmove(buf + 4 + afc_len + stuffing_len,
                        buf + 4 + afc_len,
                        header_len - (4 + afc_len));
                buf[4] += stuffing_len;
                memset(buf + 4 + afc_len, STUFFING_BYTE, stuffing_len);
            } else {
                /* add stuffing */
                memmove(buf + 4 + stuffing_len, buf + 4, header_len - 4);
                buf[3] |= 0x20;
                buf[4]  = stuffing_len - 1;
                if (stuffing_len >= 2) {
                    buf[5] = 0x00;
                    memset(buf + 6, STUFFING_BYTE, stuffing_len - 2);
                }
            }
        }

        if (is_dvb_subtitle && payload_size == len) {
            memcpy(buf + TS_PACKET_SIZE - len, payload, len - 1);
            buf[TS_PACKET_SIZE - 1] = 0xff; /* end_of_PES_data_field_marker: an 8-bit field with fixed contents 0xff for DVB subtitle */
        } else {
            memcpy(buf + TS_PACKET_SIZE - len, payload, len);
        }

        payload      += len;
        payload_size -= len;

        /* PEP: encrypt payload portion of TS packet */
        if (ts->pep_context && ts_st->pid >= 0x0010 && ts_st->pid <= 0x1ffe) {
            RtpPepContext *pep = (RtpPepContext *)ts->pep_context;
            int hdr_size = buf[4] + 1 + 4; /* adaptation_field_length + 1 + 4-byte TS header */
            int payload_len = TS_PACKET_SIZE - hdr_size;

            /* KV rotation: if pending and at a PES start boundary, rotate now */
            if (pep->kv_rotate_pending && is_start_saved) {
                uint8_t kv_bytes[4];
                int key_bits = av_ipmx_pep_mode_key_bits(pep->mode);
                pep->key_version = (pep->key_version + 1) & 0xFFFFFFFF;
                AV_WB32(kv_bytes, pep->key_version);
                av_ipmx_pep_derive_key(pep->psk, pep->psk_len,
                                        pep->key_generator, kv_bytes,
                                        pep->key, key_bits);
                ts->pep_key_ver = pep->key_version;
                pep->kv_rotate_pending = 0;
                pep->kv_slices_since_rotate = 0;
                av_log(s, AV_LOG_INFO, "PEP: key_version rotated to %08x\n",
                       pep->key_version);
            }

            if (payload_len > 0) {
                av_ipmx_pep_aes_ctr(pep->key, pep->key_len, pep->iv_prime,
                                     ts->pep_input_ctr,
                                     buf + hdr_size, buf + hdr_size, payload_len);
                ts->pep_input_ctr += av_ipmx_pep_ctr_advance(payload_len);
                /* Count slices and arm rotation */
                if (pep->kv_rotate_slices > 0) {
                    pep->kv_slices_since_rotate += av_ipmx_pep_ctr_advance(payload_len);
                    if (pep->kv_slices_since_rotate >= pep->kv_rotate_slices)
                        pep->kv_rotate_pending = 1;
                }
            }
            is_start_saved = 0;
        }

        write_packet(s, buf);
    }
    ts_st->prev_payload_key = key;
}

static int check_h26x_startcode(AVFormatContext *s, const AVStream *st, const AVPacket *pkt, const char *codec)
{
    if (pkt->size < 5 || AV_RB32(pkt->data) != 0x0000001 && AV_RB24(pkt->data) != 0x000001) {
        if (!st->nb_frames) {
            av_log(s, AV_LOG_ERROR, "%s bitstream malformed, "
                   "no startcode found, use the video bitstream filter '%s_mp4toannexb' to fix it "
                   "('-bsf:v %s_mp4toannexb' option with ffmpeg)\n", codec, codec, codec);
            return AVERROR_INVALIDDATA;
        }
        av_log(s, AV_LOG_WARNING, "%s bitstream error, startcode missing, size %d", codec, pkt->size);
        if (pkt->size)
            av_log(s, AV_LOG_WARNING, " data %08"PRIX32, AV_RB32(pkt->data));
        av_log(s, AV_LOG_WARNING, "\n");
    }
    return 0;
}

int ff_check_h264_startcode(AVFormatContext *s, const AVStream *st, const AVPacket *pkt)
{
    return check_h26x_startcode(s, st, pkt, "h264");
}

/* Based on GStreamer's gst-plugins-base/ext/ogg/gstoggstream.c
 * Released under the LGPL v2.1+, written by
 * Vincent Penquerc'h <vincent.penquerch@collabora.co.uk>
 */
static int opus_get_packet_samples(AVFormatContext *s, AVPacket *pkt)
{
    static const int durations[32] = {
      480, 960, 1920, 2880,       /* Silk NB */
      480, 960, 1920, 2880,       /* Silk MB */
      480, 960, 1920, 2880,       /* Silk WB */
      480, 960,                   /* Hybrid SWB */
      480, 960,                   /* Hybrid FB */
      120, 240, 480, 960,         /* CELT NB */
      120, 240, 480, 960,         /* CELT NB */
      120, 240, 480, 960,         /* CELT NB */
      120, 240, 480, 960,         /* CELT NB */
    };
    int toc, frame_duration, nframes, duration;

    if (pkt->size < 1)
        return 0;

    toc = pkt->data[0];

    frame_duration = durations[toc >> 3];
    switch (toc & 3) {
    case 0:
        nframes = 1;
        break;
    case 1:
        nframes = 2;
        break;
    case 2:
        nframes = 2;
        break;
    case 3:
        if (pkt->size < 2)
            return 0;
        nframes = pkt->data[1] & 63;
        break;
    }

    duration = nframes * frame_duration;
    if (duration > 5760) {
        av_log(s, AV_LOG_WARNING,
               "Opus packet duration > 120 ms, invalid");
        return 0;
    }

    return duration;
}

static uint8_t *h26x_prefix_aud(const uint8_t *aud, const int aud_size,
    const uint8_t *extra_data, const int extra_size, AVPacket *pkt, int *size)
{
    const int sz = 4;       //start code size
    uint8_t *data = av_malloc(pkt->size + sz + aud_size + extra_size);
    if (!data)
        return NULL;
    AV_WB32(data, 0x00000001);
    memcpy(data + sz, aud, aud_size);
    memcpy(data + sz + aud_size, extra_data, extra_size);
    memcpy(data + sz + aud_size + extra_size, pkt->data, pkt->size);
    *size = pkt->size + sz + aud_size + extra_size;
    return data;
}

#define H264_NAL_TYPE(state) (state & 0x1f)
#define HEVC_NAL_TYPE(state) ((state & 0x7e) >> 1)
#define VVC_NAL_TYPE(state)  ((state >> 11) & 0x1f)
static int mpegts_write_packet_internal(AVFormatContext *s, AVPacket *pkt)
{
    AVStream *st = s->streams[pkt->stream_index];
    int size = pkt->size;
    const uint8_t *buf = pkt->data;
    uint8_t *data = NULL;
    MpegTSWrite *ts = s->priv_data;
    MpegTSWriteStream *ts_st = st->priv_data;
    const int64_t delay = av_rescale(s->max_delay, 90000, AV_TIME_BASE) * 2;
    const int64_t max_audio_delay = av_rescale(s->max_delay, 90000, AV_TIME_BASE) / 2;
    int64_t dts = pkt->dts, pts = pkt->pts;
    int opus_samples = 0;
    size_t side_data_size;
    uint8_t *side_data = NULL;
    int stream_id = -1;

    side_data = av_packet_get_side_data(pkt,
                                        AV_PKT_DATA_MPEGTS_STREAM_ID,
                                        &side_data_size);
    if (side_data)
        stream_id = side_data[0];

    if (!ts->first_dts_checked && dts != AV_NOPTS_VALUE) {
        ts->first_pcr += dts * SYSTEM_CLOCK_FREQUENCY_DIVISOR;
        ts->first_dts_checked = 1;
    }

    /* CBR gap fill: if the muxer was idle (e.g., during a -stream_loop
     * seek), wall clock has advanced but no TS packets were emitted.
     * Fill the gap with null packets to maintain continuous CBR output.
     * Without this, the muxer bursts to catch up after the idle period,
     * which can overflow the UDP socket buffer and lose a packet (CC error).
     *
     * Each null packet goes through write_packet() which applies CBR
     * pacing, so they are spaced at the correct bitrate interval. */
    if (ts->cbr_pacing && ts->mux_rate > 1 && ts->pacing_start_us > 0) {
        int64_t now_us = av_gettime_relative();
        int64_t target_bytes = av_rescale(now_us - ts->pacing_start_us,
                                          ts->mux_rate, 8000000LL);
        int64_t gap_bytes = target_bytes - ts->total_size;
        /* Only fill if we're behind by more than a few packets
         * (small gaps self-correct; large gaps need null fill) */
        if (gap_bytes > TS_PACKET_SIZE * 4) {
            int max_fill = gap_bytes / TS_PACKET_SIZE;
            /* Cap to prevent runaway fill on startup or clock jumps */
            if (max_fill > 10000)
                max_fill = 0; /* too large — clock discontinuity, skip */
            while (max_fill-- > 0)
                mpegts_insert_null_packet(s);
        }
    }

    if (ts->copyts < 1) {
        if (pts != AV_NOPTS_VALUE)
            pts += delay;
        if (dts != AV_NOPTS_VALUE)
            dts += delay;
    }

    if (!ts_st->first_timestamp_checked && (pts == AV_NOPTS_VALUE || dts == AV_NOPTS_VALUE)) {
        av_log(s, AV_LOG_ERROR, "first pts and dts value must be set\n");
        return AVERROR_INVALIDDATA;
    }
    ts_st->first_timestamp_checked = 1;

    if (st->codecpar->codec_id == AV_CODEC_ID_H264) {
        const uint8_t *p = buf, *buf_end = p + size;
        const uint8_t *found_aud = NULL, *found_aud_end = NULL;
        int nal_type;
        uint32_t state = -1;
        int extradd = (pkt->flags & AV_PKT_FLAG_KEY) ? st->codecpar->extradata_size : 0;
        int ret = ff_check_h264_startcode(s, st, pkt);
        if (ret < 0)
            return ret;

        if (extradd && AV_RB24(st->codecpar->extradata) > 1)
            extradd = 0;

        /* Ensure that all pictures are prefixed with an AUD, and that
         * IDR pictures are also prefixed with SPS and PPS. SPS and PPS
         * are assumed to be available in 'extradata' if not found in-band. */
        do {
            p = avpriv_find_start_code(p, buf_end, &state);
            nal_type = H264_NAL_TYPE(state);
            av_log(s, AV_LOG_TRACE, "nal %"PRId32"\n", nal_type);
            if (nal_type == H264_NAL_SPS)
                extradd = 0;
            if (nal_type == H264_NAL_AUD) {
                found_aud = p - 4;     // start of the 0x000001 start code.
                found_aud_end = p + 1; // first byte past the AUD.
                if (found_aud < buf)
                    found_aud = buf;
                if (buf_end < found_aud_end)
                    found_aud_end = buf_end;
            }
        } while (p < buf_end
                 && nal_type != H264_NAL_IDR_SLICE
                 && nal_type != H264_NAL_SLICE
                 && (extradd > 0 || !found_aud));
        if (nal_type != H264_NAL_IDR_SLICE)
            extradd = 0;

        if (!found_aud) {
            /* Prefix 'buf' with the missing AUD, and extradata if needed. */
            const uint8_t aud[] = {
                H264_NAL_AUD,
                0xf0, // any slice type (0xe) + rbsp stop one bit
            };
            buf = data = h26x_prefix_aud(aud, FF_ARRAY_ELEMS(aud),
                st->codecpar->extradata, extradd, pkt, &size);
            if (!data)
                return AVERROR(ENOMEM);
        } else if (extradd != 0) {
            /* Move the AUD up to the beginning of the frame, where the H.264
             * spec requires it to appear. Emit the extradata after it. */
            PutByteContext pb;
            const int new_pkt_size = pkt->size + 1 + extradd;
            data = av_malloc(new_pkt_size);
            if (!data)
                return AVERROR(ENOMEM);
            bytestream2_init_writer(&pb, data, new_pkt_size);
            bytestream2_put_byte(&pb, 0x00);
            bytestream2_put_buffer(&pb, found_aud, found_aud_end - found_aud);
            bytestream2_put_buffer(&pb, st->codecpar->extradata, extradd);
            bytestream2_put_buffer(&pb, pkt->data, found_aud - pkt->data);
            bytestream2_put_buffer(&pb, found_aud_end, buf_end - found_aud_end);
            av_assert0(new_pkt_size == bytestream2_tell_p(&pb));
            buf     = data;
            size    = new_pkt_size;
        }
    } else if (st->codecpar->codec_id == AV_CODEC_ID_AAC) {
        if (pkt->size < 2) {
            av_log(s, AV_LOG_ERROR, "AAC packet too short\n");
            return AVERROR_INVALIDDATA;
        }
        if ((AV_RB16(pkt->data) & 0xfff0) != 0xfff0) {
            int ret;
            AVPacket *pkt2 = ts->pkt;

            if (!ts_st->amux) {
                av_log(s, AV_LOG_ERROR, "AAC bitstream not in ADTS format "
                                        "and extradata missing\n");
            } else {
                av_packet_unref(pkt2);
                pkt2->data = pkt->data;
                pkt2->size = pkt->size;
                av_assert0(pkt->dts != AV_NOPTS_VALUE);
                pkt2->dts = av_rescale_q(pkt->dts, st->time_base, ts_st->amux->streams[0]->time_base);

                ret = avio_open_dyn_buf(&ts_st->amux->pb);
                if (ret < 0)
                    return ret;

                ret = av_write_frame(ts_st->amux, pkt2);
                if (ret < 0) {
                    ffio_free_dyn_buf(&ts_st->amux->pb);
                    return ret;
                }
                size            = avio_close_dyn_buf(ts_st->amux->pb, &data);
                ts_st->amux->pb = NULL;
                buf             = data;
            }
        }
    } else if (st->codecpar->codec_id == AV_CODEC_ID_HEVC) {
        const uint8_t *p = buf, *buf_end = p + size;
        uint32_t state = -1;
        int nal_type;
        int extradd = (pkt->flags & AV_PKT_FLAG_KEY) ? st->codecpar->extradata_size : 0;
        int ret = check_h26x_startcode(s, st, pkt, "hevc");
        if (ret < 0)
            return ret;

        if (extradd && AV_RB24(st->codecpar->extradata) > 1)
            extradd = 0;

        do {
            p = avpriv_find_start_code(p, buf_end, &state);
            nal_type = HEVC_NAL_TYPE(state);
            av_log(s, AV_LOG_TRACE, "nal %"PRId32"\n", nal_type);
            if (nal_type == HEVC_NAL_VPS)
                extradd = 0;
        } while (p < buf_end && nal_type != HEVC_NAL_AUD && nal_type >= HEVC_NAL_VPS);

        if (nal_type < HEVC_NAL_BLA_W_LP || nal_type >= HEVC_NAL_RSV_VCL24)
            extradd = 0;
        if (nal_type != HEVC_NAL_AUD) { // AUD NAL
            const uint8_t aud[] = {
                (HEVC_NAL_AUD << 1),
                0x01,
                0x50, // any slice type (0x4) + rbsp stop one bit
            };
            buf = data = h26x_prefix_aud(aud, FF_ARRAY_ELEMS(aud),
                st->codecpar->extradata, extradd, pkt, &size);
            if (!data)
                return AVERROR(ENOMEM);
        }
    } else if (st->codecpar->codec_id == AV_CODEC_ID_VVC) {
      const uint8_t *p = buf, *buf_end = p + size;
      uint32_t state = -1;
      uint32_t nal_type = -1;
      int extradd = (pkt->flags & AV_PKT_FLAG_KEY) ? st->codecpar->extradata_size : 0;
      int ret = check_h26x_startcode(s, st, pkt, "vvc");
      if (ret < 0)
          return ret;

      if (extradd && AV_RB24(st->codecpar->extradata) > 1)
          extradd = 0;

      do {
          p = avpriv_find_start_code(p, buf_end, &state);
          // state contains byte behind start code, p points 2 bytes behind start code
          nal_type = VVC_NAL_TYPE(state);
          av_log(s, AV_LOG_TRACE, "nal %"PRId32"\n", nal_type );
          if (nal_type == VVC_VPS_NUT)
              extradd = 0;
      } while (p < buf_end && nal_type != VVC_AUD_NUT && nal_type >= VVC_OPI_NUT);

      if (nal_type >= VVC_OPI_NUT)
          extradd = 0;
      if (nal_type != VVC_AUD_NUT) { // AUD NAL
            const uint8_t aud[] = {
                0,                                          // forbidden_zero_bit, nuh_reserved_zero_bit, nuh_layer_id
                (VVC_AUD_NUT << 3) | 1,                     // nal_unit_type, nuh_temporal_id_plus1(1)
                (pkt->flags & AV_PKT_FLAG_KEY) << 7 | 0x28, // aud_irap_or_gdr_flag, aud_pic_type(2), rbsp_stop_one_bit
            };
            buf = data = h26x_prefix_aud(aud, FF_ARRAY_ELEMS(aud), st->codecpar->extradata, extradd, pkt, &size);
            if (!data)
                return AVERROR(ENOMEM);
      }
    } else if (st->codecpar->codec_id == AV_CODEC_ID_OPUS) {
        if (pkt->size < 2) {
            av_log(s, AV_LOG_ERROR, "Opus packet too short\n");
            return AVERROR_INVALIDDATA;
        }

        /* Add Opus control header */
        if ((AV_RB16(pkt->data) >> 5) != 0x3ff) {
            uint8_t *side_data;
            size_t side_data_size;
            int i, n;
            int ctrl_header_size;
            int trim_start = 0, trim_end = 0;

            opus_samples = opus_get_packet_samples(s, pkt);

            side_data = av_packet_get_side_data(pkt,
                                                AV_PKT_DATA_SKIP_SAMPLES,
                                                &side_data_size);

            if (side_data && side_data_size >= 10) {
                trim_end = AV_RL32(side_data + 4) * 48000 / st->codecpar->sample_rate;
            }

            ctrl_header_size = pkt->size + 2 + pkt->size / 255 + 1;
            if (ts_st->opus_pending_trim_start)
              ctrl_header_size += 2;
            if (trim_end)
              ctrl_header_size += 2;

            data = av_malloc(ctrl_header_size);
            if (!data)
                return AVERROR(ENOMEM);

            data[0] = 0x7f;
            data[1] = 0xe0;
            if (ts_st->opus_pending_trim_start)
                data[1] |= 0x10;
            if (trim_end)
                data[1] |= 0x08;

            n = pkt->size;
            i = 2;
            do {
                data[i] = FFMIN(n, 255);
                n -= 255;
                i++;
            } while (n >= 0);

            av_assert0(2 + pkt->size / 255 + 1 == i);

            if (ts_st->opus_pending_trim_start) {
                trim_start = FFMIN(ts_st->opus_pending_trim_start, opus_samples);
                AV_WB16(data + i, trim_start);
                i += 2;
                ts_st->opus_pending_trim_start -= trim_start;
            }
            if (trim_end) {
                trim_end = FFMIN(trim_end, opus_samples - trim_start);
                AV_WB16(data + i, trim_end);
                i += 2;
            }

            memcpy(data + i, pkt->data, pkt->size);
            buf     = data;
            size    = ctrl_header_size;
        } else {
            /* TODO: Can we get TS formatted data here? If so we will
             * need to count the samples of that too! */
            av_log(s, AV_LOG_WARNING, "Got MPEG-TS formatted Opus data, unhandled");
        }
    } else if (st->codecpar->codec_id == AV_CODEC_ID_AC3 && !ts_st->dvb_ac3_desc) {
        AC3HeaderInfo *hdr = NULL;

        if (avpriv_ac3_parse_header(&hdr, pkt->data, pkt->size) >= 0) {
            uint8_t number_of_channels_flag;
            uint8_t service_type_flag;
            uint8_t full_service_flag = 1;
            DVBAC3Descriptor *dvb_ac3_desc;

            dvb_ac3_desc = av_mallocz(sizeof(*dvb_ac3_desc));
            if (!dvb_ac3_desc) {
                av_free(hdr);
                return AVERROR(ENOMEM);
            }

            service_type_flag = hdr->bitstream_mode;
            switch (hdr->channel_mode) {
                case AC3_CHMODE_DUALMONO:
                    number_of_channels_flag = 1;
                    break;
                case AC3_CHMODE_MONO:
                    number_of_channels_flag = 0;
                    break;
                case AC3_CHMODE_STEREO:
                    if (hdr->dolby_surround_mode == AC3_DSURMOD_ON)
                        number_of_channels_flag = 3;
                    else
                        number_of_channels_flag = 2;
                    break;
                case AC3_CHMODE_3F:
                case AC3_CHMODE_2F1R:
                case AC3_CHMODE_3F1R:
                case AC3_CHMODE_2F2R:
                case AC3_CHMODE_3F2R:
                    number_of_channels_flag = 4;
                    break;
                default: /* reserved */
                    number_of_channels_flag = 7;
                    break;
            }

            if (service_type_flag  == 1 || service_type_flag == 4 ||
                (service_type_flag == 7 && !number_of_channels_flag))
                full_service_flag = 0;

            dvb_ac3_desc->component_type_flag = 1;
            dvb_ac3_desc->component_type = (full_service_flag << 6)              |
                                           ((service_type_flag      & 0x7) << 3) |
                                           (number_of_channels_flag & 0x7);
            dvb_ac3_desc->bsid_flag   = 1;
            dvb_ac3_desc->bsid        = hdr->bitstream_id;
            dvb_ac3_desc->mainid_flag = 0;
            dvb_ac3_desc->asvc_flag   = 0;

            ts_st->dvb_ac3_desc = dvb_ac3_desc;
        }
        av_free(hdr);
    } else if (st->codecpar->codec_id == AV_CODEC_ID_PCM_BLURAY && ts->m2ts_mode) {
        if (ts->ts_interleave) ts->interleave_buffering = 1;
        mpegts_write_pes(s, st, buf, size, pts, dts,
                         pkt->flags & AV_PKT_FLAG_KEY, stream_id);
        if (ts->ts_interleave) ts->interleave_buffering = 0;
        return 0;
    }

    /* When interleaving is enabled, drain when BOTH video and audio data
     * are buffered.  This ensures we accumulate packets from the current
     * stream, and drain only once we have data from multiple streams to
     * interleave.  For a typical V,V,V,...,A,V,V,...,A pattern from the
     * DTS interleaver, this drains at each audio arrival, interleaving
     * the accumulated video + null packets with audio. */
    if (ts->ts_interleave) {
        int have_video = 0, have_audio = 0;
        int i;
        for (i = 0; i < s->nb_streams; i++) {
            MpegTSWriteStream *ts_st2 = s->streams[i]->priv_data;
            if (ts_st2 && ts_st2->interleave_buf_count > 0) {
                if (s->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO)
                    have_video = 1;
                else
                    have_audio = 1;
            }
        }
        /* Drain when we have both video and audio, or when the current
         * stream already has a large buffer (fallback for single-stream) */
        if ((have_video && have_audio) ||
            ts_st->interleave_buf_count > ts->ts_interleave_chunk * 4)
            interleave_drain(s);
    }

    if (ts_st->payload_size && (ts_st->payload_size + size > ts->pes_payload_size ||
        (dts != AV_NOPTS_VALUE && ts_st->payload_dts != AV_NOPTS_VALUE &&
         dts - ts_st->payload_dts >= max_audio_delay) ||
        ts_st->opus_queued_samples + opus_samples >= 5760 /* 120ms */)) {
        if (ts->ts_interleave) ts->interleave_buffering = 1;
        mpegts_write_pes(s, st, ts_st->payload, ts_st->payload_size,
                         ts_st->payload_pts, ts_st->payload_dts,
                         ts_st->payload_flags & AV_PKT_FLAG_KEY, stream_id);
        if (ts->ts_interleave) ts->interleave_buffering = 0;
        ts_st->payload_size = 0;
        ts_st->opus_queued_samples = 0;
    }

    if (st->codecpar->codec_type != AVMEDIA_TYPE_AUDIO || size > ts->pes_payload_size) {
        av_assert0(!ts_st->payload_size);
        // for video and subtitle, write a single pes packet
        if (ts->ts_interleave) ts->interleave_buffering = 1;
        mpegts_write_pes(s, st, buf, size, pts, dts,
                         pkt->flags & AV_PKT_FLAG_KEY, stream_id);
        if (ts->ts_interleave) ts->interleave_buffering = 0;
        ts_st->opus_queued_samples = 0;
        av_free(data);
        return 0;
    }

    if (!ts_st->payload_size) {
        ts_st->payload_pts   = pts;
        ts_st->payload_dts   = dts;
        ts_st->payload_flags = pkt->flags;
    }

    memcpy(ts_st->payload + ts_st->payload_size, buf, size);
    ts_st->payload_size += size;
    ts_st->opus_queued_samples += opus_samples;

    av_free(data);

    return 0;
}

static void mpegts_write_flush(AVFormatContext *s)
{
    MpegTSWrite *ts = s->priv_data;
    int i;

    /* flush current packets */
    for (i = 0; i < s->nb_streams; i++) {
        AVStream *st = s->streams[i];
        MpegTSWriteStream *ts_st = st->priv_data;
        if (ts_st->payload_size > 0) {
            if (ts->ts_interleave) ts->interleave_buffering = 1;
            mpegts_write_pes(s, st, ts_st->payload, ts_st->payload_size,
                             ts_st->payload_pts, ts_st->payload_dts,
                             ts_st->payload_flags & AV_PKT_FLAG_KEY, -1);
            if (ts->ts_interleave) ts->interleave_buffering = 0;
            ts_st->payload_size = 0;
            ts_st->opus_queued_samples = 0;
        }
    }

    /* Final drain of any remaining interleave-buffered packets */
    if (ts->ts_interleave)
        interleave_drain(s);

    if (ts->m2ts_mode) {
        int packets = (avio_tell(s->pb) / (TS_PACKET_SIZE + 4)) % 32;
        while (packets++ < 32)
            mpegts_insert_null_packet(s);
    }
}

static int mpegts_write_packet(AVFormatContext *s, AVPacket *pkt)
{
    if (!pkt) {
        mpegts_write_flush(s);
        return 1;
    } else {
        return mpegts_write_packet_internal(s, pkt);
    }
}

static int mpegts_write_end(AVFormatContext *s)
{
    if (s->pb)
        mpegts_write_flush(s);

    /* Drain and stop the output thread after the mux thread has pushed
     * the trailing buffered content into the ring. */
    mpegts_out_thread_stop(s);

    return 0;
}

static void mpegts_deinit(AVFormatContext *s)
{
    MpegTSWrite *ts = s->priv_data;
    MpegTSService *service;
    int i;

    /* Safety net: stop the thread if write_end didn't (e.g. error path). */
    mpegts_out_thread_stop(s);

    for (i = 0; i < s->nb_streams; i++) {
        AVStream *st = s->streams[i];
        MpegTSWriteStream *ts_st = st->priv_data;
        if (ts_st) {
            av_freep(&ts_st->dvb_ac3_desc);
            av_freep(&ts_st->payload);
            av_freep(&ts_st->interleave_buf);
            if (ts_st->amux) {
                avformat_free_context(ts_st->amux);
                ts_st->amux = NULL;
            }
        }
    }

    for (i = 0; i < ts->nb_services; i++) {
        service = ts->services[i];
        av_freep(&service);
    }
    av_freep(&ts->services);
    av_freep(&ts->interleave_si_buf);
}

static int mpegts_check_bitstream(AVFormatContext *s, AVStream *st,
                                  const AVPacket *pkt)
{
    const struct Entry {
        enum AVCodecID id;
        const char *bsf_name;
        uint8_t mask;
        uint8_t value;
    } list[] = {
        { AV_CODEC_ID_H264, "h264_mp4toannexb", 0xff, 0x01 /* configurationVersion in AVCDecoderConfigurationRecord  */},
        { AV_CODEC_ID_HEVC, "hevc_mp4toannexb", 0xff, 0x01 /* configurationVersion in HEVCDecoderConfigurationRecord */},
        { AV_CODEC_ID_VVC,  "vvc_mp4toannexb",  0xf8, 0xf8 /* reserved '11111'b    in VVCDecoderConfigurationRecord  */},
    };

    for (int i = 0; i < FF_ARRAY_ELEMS(list); i++) {
        const struct Entry *e = list + i;
        if (e->id == st->codecpar->codec_id &&
                pkt->size >= 5 && AV_RB32(pkt->data) != 0x0000001 &&
                (AV_RB24(pkt->data) != 0x000001 ||
                    (st->codecpar->extradata_size > 0 &&
                        ((st->codecpar->extradata[0] & e->mask) == e->value))))
            return ff_stream_add_bitstream_filter(st, e->bsf_name, NULL);
    }
    return 1;
}

#define OFFSET(x) offsetof(MpegTSWrite, x)
#define ENC AV_OPT_FLAG_ENCODING_PARAM
static const AVOption options[] = {
    { "mpegts_transport_stream_id", "Set transport_stream_id field.",
      OFFSET(transport_stream_id), AV_OPT_TYPE_INT, { .i64 = 0x0001 }, 0x0001, 0xffff, ENC },
    { "mpegts_original_network_id", "Set original_network_id field.",
      OFFSET(original_network_id), AV_OPT_TYPE_INT, { .i64 = DVB_PRIVATE_NETWORK_START }, 0x0001, 0xffff, ENC },
    { "mpegts_service_id", "Set service_id field.",
      OFFSET(service_id), AV_OPT_TYPE_INT, { .i64 = 0x0001 }, 0x0001, 0xffff, ENC },
    { "mpegts_service_type", "Set service_type field.",
      OFFSET(service_type), AV_OPT_TYPE_INT, { .i64 = 0x01 }, 0x01, 0xff, ENC, .unit = "mpegts_service_type" },
    { "digital_tv", "Digital Television.",
      0, AV_OPT_TYPE_CONST, { .i64 = MPEGTS_SERVICE_TYPE_DIGITAL_TV }, 0x01, 0xff, ENC, .unit = "mpegts_service_type" },
    { "digital_radio", "Digital Radio.",
      0, AV_OPT_TYPE_CONST, { .i64 = MPEGTS_SERVICE_TYPE_DIGITAL_RADIO }, 0x01, 0xff, ENC, .unit = "mpegts_service_type" },
    { "teletext", "Teletext.",
      0, AV_OPT_TYPE_CONST, { .i64 = MPEGTS_SERVICE_TYPE_TELETEXT }, 0x01, 0xff, ENC, .unit = "mpegts_service_type" },
    { "advanced_codec_digital_radio", "Advanced Codec Digital Radio.",
      0, AV_OPT_TYPE_CONST, { .i64 = MPEGTS_SERVICE_TYPE_ADVANCED_CODEC_DIGITAL_RADIO }, 0x01, 0xff, ENC, .unit = "mpegts_service_type" },
    { "mpeg2_digital_hdtv", "MPEG2 Digital HDTV.",
      0, AV_OPT_TYPE_CONST, { .i64 = MPEGTS_SERVICE_TYPE_MPEG2_DIGITAL_HDTV }, 0x01, 0xff, ENC, .unit = "mpegts_service_type" },
    { "advanced_codec_digital_sdtv", "Advanced Codec Digital SDTV.",
      0, AV_OPT_TYPE_CONST, { .i64 = MPEGTS_SERVICE_TYPE_ADVANCED_CODEC_DIGITAL_SDTV }, 0x01, 0xff, ENC, .unit = "mpegts_service_type" },
    { "advanced_codec_digital_hdtv", "Advanced Codec Digital HDTV.",
      0, AV_OPT_TYPE_CONST, { .i64 = MPEGTS_SERVICE_TYPE_ADVANCED_CODEC_DIGITAL_HDTV }, 0x01, 0xff, ENC, .unit = "mpegts_service_type" },
    { "hevc_digital_hdtv", "HEVC Digital Television Service.",
      0, AV_OPT_TYPE_CONST, { .i64 = MPEGTS_SERVICE_TYPE_HEVC_DIGITAL_HDTV }, 0x01, 0xff, ENC, .unit = "mpegts_service_type" },
    { "mpegts_pmt_start_pid", "Set the first pid of the PMT.",
      OFFSET(pmt_start_pid), AV_OPT_TYPE_INT, { .i64 = 0x1000 }, FIRST_OTHER_PID, LAST_OTHER_PID, ENC },
    { "mpegts_start_pid", "Set the first pid.",
      OFFSET(start_pid), AV_OPT_TYPE_INT, { .i64 = 0x0100 }, FIRST_OTHER_PID, LAST_OTHER_PID, ENC },
    { "mpegts_m2ts_mode", "Enable m2ts mode.", OFFSET(m2ts_mode), AV_OPT_TYPE_BOOL, { .i64 = -1 }, -1, 1, ENC },
    { "muxrate", NULL, OFFSET(mux_rate), AV_OPT_TYPE_INT, { .i64 = 1 }, 0, INT_MAX, ENC },
    { "pes_payload_size", "Minimum PES packet payload in bytes",
      OFFSET(pes_payload_size), AV_OPT_TYPE_INT, { .i64 = DEFAULT_PES_PAYLOAD_SIZE }, 0, INT_MAX, ENC },
    { "mpegts_flags", "MPEG-TS muxing flags", OFFSET(flags), AV_OPT_TYPE_FLAGS, { .i64 = 0 }, 0, INT_MAX, ENC, .unit = "mpegts_flags" },
    { "resend_headers", "Reemit PAT/PMT before writing the next packet",
      0, AV_OPT_TYPE_CONST, { .i64 = MPEGTS_FLAG_REEMIT_PAT_PMT }, 0, INT_MAX, ENC, .unit = "mpegts_flags" },
    { "latm", "Use LATM packetization for AAC",
      0, AV_OPT_TYPE_CONST, { .i64 = MPEGTS_FLAG_AAC_LATM }, 0, INT_MAX, ENC, .unit = "mpegts_flags" },
    { "pat_pmt_at_frames", "Reemit PAT and PMT at each video frame",
      0, AV_OPT_TYPE_CONST, { .i64 = MPEGTS_FLAG_PAT_PMT_AT_FRAMES}, 0, INT_MAX, ENC, .unit = "mpegts_flags" },
    { "system_b", "Conform to System B (DVB) instead of System A (ATSC)",
      0, AV_OPT_TYPE_CONST, { .i64 = MPEGTS_FLAG_SYSTEM_B }, 0, INT_MAX, ENC, .unit = "mpegts_flags" },
    { "initial_discontinuity", "Mark initial packets as discontinuous",
      0, AV_OPT_TYPE_CONST, { .i64 = MPEGTS_FLAG_DISCONT }, 0, INT_MAX, ENC, .unit = "mpegts_flags" },
    { "nit", "Enable NIT transmission",
      0, AV_OPT_TYPE_CONST, { .i64 = MPEGTS_FLAG_NIT}, 0, INT_MAX, ENC, .unit = "mpegts_flags" },
    { "omit_rai", "Disable writing of random access indicator",
      0, AV_OPT_TYPE_CONST, { .i64 = MPEGTS_FLAG_OMIT_RAI }, 0, INT_MAX, ENC, .unit = "mpegts_flags" },
    { "mpegts_copyts", "don't offset dts/pts", OFFSET(copyts), AV_OPT_TYPE_BOOL, { .i64 = -1 }, -1, 1, ENC },
    { "tables_version", "set PAT, PMT, SDT and NIT version", OFFSET(tables_version), AV_OPT_TYPE_INT, { .i64 = 0 }, 0, 31, ENC },
    { "omit_video_pes_length", "Omit the PES packet length for video packets",
      OFFSET(omit_video_pes_length), AV_OPT_TYPE_BOOL, { .i64 = 1 }, 0, 1, ENC },
    { "cbr_pacing", "Pace TS packet output to match muxrate (for CBR UDP streaming)",
      OFFSET(cbr_pacing), AV_OPT_TYPE_BOOL, { .i64 = 0 }, 0, 1, ENC },
    { "cbr_pacing_threaded", "Pace output in a dedicated thread fed by a ring "
                             "buffer; absorbs mux-thread scheduling stalls and "
                             "emits at strict per-packet-duration spacing to "
                             "avoid downstream T-STD TBn overflow. Requires "
                             "cbr_pacing=1.",
      OFFSET(cbr_pacing_threaded), AV_OPT_TYPE_BOOL, { .i64 = 0 }, 0, 1, ENC },
    { "cbr_pacing_spin", "Threaded pacer only: pure busy-spin between emits "
                         "instead of sleep+tail-spin.  Burns one CPU core at "
                         "100% for maximum timing precision; useful on RT "
                         "kernels with isolated cores.  Default 0 (sleep+"
                         "spin, which is kinder to the scheduler and — on a "
                         "standard Linux kernel — empirically produces "
                         "fewer late events too).",
      OFFSET(cbr_pacing_spin), AV_OPT_TYPE_BOOL, { .i64 = 0 }, 0, 1, ENC },
    { "ts_interleave", "Interleave audio/video TS packets (0=off, 1=null-interleave, 2=full-interleave)",
      OFFSET(ts_interleave), AV_OPT_TYPE_INT, { .i64 = 0 }, 0, 2, ENC },
    { "ts_interleave_chunk", "Video TS packets per interleave round (used by ts_interleave=1 and =2). "
                              "Default 7 = exactly one full UDP datagram of pure video at pkt_size=1316, "
                              "which empirically gives the lowest worst-case video T-STD TBn peak.",
      OFFSET(ts_interleave_chunk), AV_OPT_TYPE_INT, { .i64 = 7 }, 1, 256, ENC },
    { "pep_psk", "PEP pre-shared key (hex) for MPEG-TS encryption",
      OFFSET(pep_psk), AV_OPT_TYPE_STRING, { .str = NULL }, 0, 0, ENC },
    { "pep_mode", "PEP cipher mode (e.g. AES-128-CTR)",
      OFFSET(pep_mode), AV_OPT_TYPE_STRING, { .str = NULL }, 0, 0, ENC },
    { "pep_iv", "PEP base IV (hex)",
      OFFSET(pep_iv), AV_OPT_TYPE_STRING, { .str = NULL }, 0, 0, ENC },
    { "pep_key_generator", "PEP key generator (hex)",
      OFFSET(pep_key_generator), AV_OPT_TYPE_STRING, { .str = NULL }, 0, 0, ENC },
    { "pep_key_version", "PEP key version (hex)",
      OFFSET(pep_key_version), AV_OPT_TYPE_STRING, { .str = NULL }, 0, 0, ENC },
    { "pcr_period", "PCR retransmission time in milliseconds",
      OFFSET(pcr_period_ms), AV_OPT_TYPE_INT, { .i64 = -1 }, -1, INT_MAX, ENC },
    { "pat_period", "PAT/PMT retransmission time limit in seconds",
      OFFSET(pat_period_us), AV_OPT_TYPE_DURATION, { .i64 = PAT_RETRANS_TIME * 1000LL }, 0, INT64_MAX, ENC },
    { "pat_min_gap", "Minimum interval between PAT/PMT sections (TR 101 290 P3.2 floor)",
      OFFSET(pat_min_gap_us), AV_OPT_TYPE_DURATION, { .i64 = 25000 }, 0, 500000, ENC },
    { "sdt_period", "SDT retransmission time limit in seconds",
      OFFSET(sdt_period_us), AV_OPT_TYPE_DURATION, { .i64 = SDT_RETRANS_TIME * 1000LL }, 0, INT64_MAX, ENC },
    { "nit_period", "NIT retransmission time limit in seconds",
      OFFSET(nit_period_us), AV_OPT_TYPE_DURATION, { .i64 = NIT_RETRANS_TIME * 1000LL }, 0, INT64_MAX, ENC },
    { NULL },
};

static const AVClass mpegts_muxer_class = {
    .class_name = "MPEGTS muxer",
    .item_name  = av_default_item_name,
    .option     = options,
    .version    = LIBAVUTIL_VERSION_INT,
};

const FFOutputFormat ff_mpegts_muxer = {
    .p.name         = "mpegts",
    .p.long_name    = NULL_IF_CONFIG_SMALL("MPEG-TS (MPEG-2 Transport Stream)"),
    .p.mime_type    = "video/MP2T",
    .p.extensions   = "ts,m2t,m2ts,mts",
    .priv_data_size = sizeof(MpegTSWrite),
    .p.audio_codec  = AV_CODEC_ID_MP2,
    .p.video_codec  = AV_CODEC_ID_MPEG2VIDEO,
    .init           = mpegts_init,
    .write_packet   = mpegts_write_packet,
    .write_trailer  = mpegts_write_end,
    .deinit         = mpegts_deinit,
    .check_bitstream = mpegts_check_bitstream,
    .p.flags         = AVFMT_VARIABLE_FPS | AVFMT_NODIMENSIONS,
    .flags_internal  = FF_OFMT_FLAG_ALLOW_FLUSH,
    .p.priv_class   = &mpegts_muxer_class,
};
