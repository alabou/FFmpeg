/*
 * IPMX Streaming Muxer (VSF TR-10-1 et al.)
 *
 * Copyright (c) 2026 Matrox Graphics Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *
 * Architecture:
 *   - One -f ipmx muxer instance owns N per-stream `IpmxStreamRT` contexts
 *     (1 video + up to N-1 audio). Each owns its own child ff_rtp_muxer,
 *     paced AVIO, RTP+RTCP UDP sockets, and SR scheduler state.
 *   - The muxer holds the shared Internal Clock anchor (TR-10-1 §7.1, §8.7),
 *     SDP/PCAP file handles, and configurable options.
 *   - Pacing for Step 1/2 is inline per-stream (β=1.10 ST 2110-21 §7.1);
 *     the shared pacer thread (cross-stream min-heap) is added in Step 2.3.
 *
 * Conformance:
 *   - TR-10-1 §7.1, §8.5–§8.7, §8.8, §8.10, §10
 *   - TR-10-2 §10 (uncompressed video Media Info Block, type 0x0001)
 *   - TR-10-3   (PCM audio Media Info Block, type 0x0002)
 *   - SMPTE ST 2110-10/20/30 + ST 2110-21 §6.6.2 / §7.1 (β = 1.10)
 *   - RFC 3550 §6.4.1 SR layout (RC=0 per TR-10-1 §8.7)
 */

#include "config_components.h"

#include <assert.h>
#include <errno.h>
#include <inttypes.h>
#include <math.h>
#include <pthread.h>
#include <stdatomic.h>
#include <string.h>
#include <time.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sched.h>
#include <unistd.h>

#include "libavutil/avstring.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/log.h"
#include "libavutil/mathematics.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/pixdesc.h"
#include "libavutil/random_seed.h"
#include "libavutil/time.h"

#include "libavcodec/defs.h"
#include "libavcodec/get_bits.h"
#include "libavcodec/golomb.h"
#include "libavcodec/packet.h"

#include "avformat.h"
#include "avio_internal.h"
#include "internal.h"
#include "mux.h"
#include "nal.h"
#include "rtpenc.h"
#include "url.h"

extern const FFOutputFormat ff_rtp_muxer;

/* Constants -------------------------------------------------------------- */

#define IPMX_RTCP_TAG               0x5831  /* "X1" — TR-10-1 §8.7 */
#define IPMX_INFO_BLOCK_VERSION     0x01

#define IPMX_MIB_TYPE_VIDEO_RAW     0x0001  /* TR-10-2 §10 */
#define IPMX_MIB_TYPE_AUDIO_PCM     0x0002  /* TR-10-3 */
#define IPMX_MIB_TYPE_VIDEO_JXSV    0x0003
#define IPMX_MIB_TYPE_AUDIO_AM824   0x0004  /* TR-10-12 */
#define IPMX_MIB_TYPE_VIDEO_COMP    0x0005  /* TR-10-7 §12 */
#define IPMX_MIB_TYPE_VIDEO_H265    0x0009  /* TR-10-15b §16 */
#define IPMX_MIB_TYPE_VIDEO_H264    0x000A  /* TR-10-15c §16 */
#define IPMX_MIB_TYPE_PEP           0x0011  /* TR-10-13 */

/* Video MIB rate field: 22-bit frame-rate numerator, 10-bit denominator. */
#define IPMX_MIB_RATE_NUM_MAX       0x3fffff
#define IPMX_MIB_RATE_DEN_MAX       0x3ff

/* Relative tolerance for recognising a nominal frame rate (N or N·1000/1001
 * fps) in a container rate. Container timestamps are rounded: 59.94 fps in
 * a 90 kHz MP4 track reads back as 54000000/900901 (1 ppm from 60000/1001),
 * while neighbouring nominal rates (60 vs 59.94) are 1000 ppm apart. */
#define IPMX_FRAME_RATE_SNAP_TOLERANCE 1e-4

/* RFC 7798 §7.1 tx-mode. The muxer produces a single RTP stream (SRST). The
 * SDP fmtp line and H.265 MIB 0x0009 both carry this value, so TR-10-15b §16
 * "present in the fmtp line ⇒ present in the MIB, same syntax" holds by
 * construction. */
#define IPMX_HEVC_TX_MODE           "SRST"
static_assert(sizeof(IPMX_HEVC_TX_MODE) - 1 == 4, "MIB 0x0009 tx-mode is a 4-byte field");
/* TR-10-15b §16: MIB 0x0009 sprop-vps/sps/pps lengths are 1 byte. */
#define IPMX_H265_SPROP_MAX_CHARS   255

#define RTCP_PT_SR                  200
#define RTCP_PT_SDES                202     /* RFC 3550 §12.1 */
#define SDES_ITEM_CNAME             0x01    /* RFC 3550 §6.5.1 */

/* DSCP per IPMX TP-10-1 §13.1: 36 (AF42) for video, 34 (AF41) for audio.
 * IP_TOS / IP byte 1 expects DSCP shifted left by 2 (low 2 bits = ECN). */
#define IPMX_DSCP_VIDEO_TOS         (36 << 2)
#define IPMX_DSCP_AUDIO_TOS         (34 << 2)

#define IPMX_DEFAULT_BETA           1.10
#define IPMX_RTCP_BUF_MAX           1500
#define IPMX_AVIO_BUF_SZ            65536

#define IPMX_MAX_STREAMS            16

/* Initial AU offset selection: how much idle time to leave between AU 0's
 * encoder pour (= stream_epoch + encoder_delay) and AU 0's first byte on wire.
 *
 * AU_offset_0 ∈ [0, cpb_size/max_bitrate]  per TR-10-15c §15.
 *
 *   IPMX_AU_OFFSET0_ZERO     = 0  (min latency; SR & AU 0 co-located)
 *   IPMX_AU_OFFSET0_SYMMETRIC= cpb_size/max_bitrate − init_delay/90000  (default)
 *                              (symmetric encoder/decoder bucket round trip
 *                               equals cpb_size/max_bitrate; matches x264/x265
 *                               which both compute the SEI offset field as
 *                               cpb_size·90000/bitrate − initial_cpb_removal_delay)
 *   IPMX_AU_OFFSET0_MAX      = cpb_size/max_bitrate  (TR-10-15c upper bound;
 *                              receiver pre-buffers a full CPB worth before
 *                              first AU arrives)
 *   IPMX_AU_OFFSET0_OFFSET   = initial_cpb_removal_delay_offset/90000
 *                              (use the encoder's declared offset verbatim;
 *                               only useful for encoders that compute the
 *                               offset field independently of cpb_size — e.g.
 *                               low_delay_hrd_flag=1 sources where the CPB
 *                               is declared larger than what was used to
 *                               compute the encoder pre-fill. Numerically
 *                               equal to SYMMETRIC for x264/x265 CBR sources.)
 */
typedef enum IpmxAuOffset0Mode {
    IPMX_AU_OFFSET0_ZERO      = 0,
    IPMX_AU_OFFSET0_SYMMETRIC = 1,
    IPMX_AU_OFFSET0_MAX       = 2,
    IPMX_AU_OFFSET0_OFFSET    = 3,
} IpmxAuOffset0Mode;

/* Maximum payload bytes per ring slot (RTP MTU + headroom). */
#define IPMX_PACER_EVT_MAX          2000

/* Per-stream MPSC ring sizing. Sized to absorb each stream's largest
 * natural burst plus a safety factor:
 *   - Video at 640×480p30 with 1400 B MTU → ~336 packets per frame burst.
 *     Round up to 512 (next power of two; head/tail wrap is fast with mask).
 *   - Audio at 1 ms ptime is a 1-packet "burst" but we need slack to absorb
 *     pacer scheduler jitter; 64 covers ~64 ms worst-case backlog.
 *
 * NOTE: IPMX_RING_CAP_* values MUST be powers of two (head & (cap-1) mask). */
/* MPSC ring capacities — must accommodate ffmpeg's `-re` 0.5 s
 * initial_burst plus headroom for AU-drain wall-clock-vs-pacer skew.
 * Undersized rings cause the producer (ffmpeg's main mux loop) to spin
 * on full-slot wait, which back-pressures every input's demux thread
 * and triggers ffmpeg's "Resumed reading at pts X after a lag of Y"
 * warning — observed pre-fix as 2.25 s of cumulative producer wait
 * across 9 streams in a 6 s burst-fixture run (337 RTP push blocks for
 * stream 0 video + 11–20 k for audio streams 1–8).
 *
 *   VIDEO 2048: 4× of 30 fps × max_AU(~80 packets) — absorbs the
 *               worst-case multi-IDR pre-buffer without spinning.
 *   AUDIO 1024: 1024 × 1 ms ptime = ~1 s of audio per stream — more
 *               than enough to absorb a 0.5 s -re initial_burst even
 *               when the pacer is briefly behind on drain. */
#define IPMX_RING_CAP_VIDEO         4096
#define IPMX_RING_CAP_AUDIO         2048
#define IPMX_RING_CAP_SR             512

/* Pacer drain heuristic (per-stream lookahead window) — see plan §2:
 *   K_video × T_frame_us  → 2 frames of packets visible at any time
 *   K_audio × T_ptime_us  → 5 audio packets per stream visible at any time
 * Both are configurable as AVOptions (see IpmxMuxContext). */
#define IPMX_DEFAULT_K_VIDEO        2
#define IPMX_DEFAULT_K_AUDIO        5

/* Idle-poll cap when both heap and rings empty. */
#define IPMX_PACER_IDLE_POLL_US     50

/* ------------------------------------------------------------------------- */

typedef enum IpmxEventKind {
    IPMX_EVT_RTP      = 0,
    IPMX_EVT_RTCP_SR  = 1,
    /* Synthetic kind: not produced by the producer side, only emitted into
     * the trace ring by the pacer when a slot is dropped because it was
     * stale beyond ipmx_drop_late_us. Lets post-mortem analysis see exactly
     * which packets were dropped vs dispatched. */
    IPMX_EVT_DROP_RTP = 2,
    IPMX_EVT_DROP_SR  = 3,
    /* Synthetic kind: logged by the pacer dispatch loop each time the
     * preempt branch fires (best.target > pacer_spin_us in the future →
     * av_usleep(pacer_spin_us) then re-loop). Lets post-mortem trace
     * analysis see exactly when the pacer entered an outer-sleep wait
     * and how long it slept relative to the next dispatched event. */
    IPMX_EVT_PREEMPT  = 4,
} IpmxEventKind;

typedef struct IpmxEvent {
    int64_t   target_us;          /* β-paced or SR target wall-clock time */
    uint64_t  seq;                /* tie-breaker for equal target_us, FIFO */
    int       stream_idx;         /* which IpmxStreamRT this targets */
    IpmxEventKind kind;
    int       len;
    int       dscp_tos;           /* IP_TOS for the synthesised PCAP IP header */
    uint8_t   data[IPMX_PACER_EVT_MAX];
} IpmxEvent;

/* Per-stream MPSC ring slot. Producer-multi-thread, consumer-single (pacer). */
typedef struct IpmxRingSlot {
    atomic_int  ready;            /* 0 = free (writable by producer);
                                     1 = filled (readable by pacer) */
    IpmxEvent   evt;              /* event payload inline (no malloc) */
} IpmxRingSlot;

/* Per-stream MPSC ring. Multiple producer threads (FFmpeg's mux thread,
 * occasionally the main thread for trailer flushes) reserve slots via
 * atomic fetch-add on `head`. The single consumer (pacer) drains slots
 * in FIFO order via `tail` (consumer-only, not atomic — only the pacer
 * touches it). */
typedef struct IpmxStreamRing {
    IpmxRingSlot *slots;
    int           cap;            /* power of two */
    atomic_uint   head;           /* producer reserves: fetch_add(&head, 1) */
    unsigned      tail;           /* consumer-only */
} IpmxStreamRing;

/* In-memory trace ring (single-thread writer = pacer; dump on close).
 * Used for realtime-pacer tuning per the user's "in-memory ring buffer"
 * suggestion. Disabled when ipmx_pacer_trace = 0 (default). */
typedef struct IpmxTraceEntry {
    int64_t  t_us;            /* av_gettime_relative() at log time (= pre-send) */
    int64_t  target_us;       /* effective target after pacer_offset_us applied */
    int64_t  raw_target_us;   /* producer-side target (without offset) */
    int64_t  send_done_us;    /* av_gettime_relative() after ffurl_write returns */
    int64_t  pcap_done_us;    /* av_gettime_relative() after pcap_emit_udp returns */
    int      stream_idx;
    int      kind;            /* 0=RTP, 1=SR */
    int      late_us;         /* t_us - target_us; positive = late */
} IpmxTraceEntry;

/* In-memory HRD pacing trace — one record per compressed-video AU, written
 * by the mux thread at the AU's marker packet (single writer per stream).
 * Records the inputs and the outcome of the inter-AU pacing decision so the
 * HRD transmitter schedule can be checked against the wire. Times are
 * relative to the stream epoch. Disabled when ipmx_hrd_trace_cap = 0. */
typedef struct IpmxHrdTraceEntry {
    int      au_idx;
    int64_t  rtp_payload_bytes;   /* RTP payload bytes, RFC 6184/7798 payload headers included */
    int64_t  nal_unit_bytes;      /* NAL unit bytes (H.264 eq C-6 b(n) / H.265 eq C-8 sizeInbits, ÷8) */
    int64_t  pace_us;             /* first-packet target of this AU (after eq C-3 stall and HRD ceiling) */
    int64_t  t_af_advance_us;     /* drain time used for the inter-AU advance */
    int64_t  t_r_n_us;            /* t_r,n of this AU (pic_timing SEI) */
    int64_t  frame_epoch_next_us; /* encoder pour time of the next AU */
    int64_t  pace_next_us;        /* lower bound max(frame_epoch, t_af) set for the next AU,
                                   * before its eq C-3 stall and HRD ceiling */
    int64_t  clamp_us;            /* HRD-ceiling reduction of this AU's pace_us (0 = none) */
} IpmxHrdTraceEntry;

/* Pacer thread state. No heap, no pool — just the thread + trace ring.
 * All event storage lives in per-stream rings (IpmxStreamRT.ring). */
typedef struct IpmxPacer {
    pthread_t        thread;
    int              started;
    atomic_int       stop;

    /* Optional trace ring (size = power of two; head increments forever). */
    IpmxTraceEntry  *trace;
    int              trace_cap;
    int              trace_mask;
    atomic_uint      trace_head;

    /* Startup gate state — see ipmx_pacer_thread.
     *
     * On thread entry, gate_start_us is captured. Dispatch is held until
     * either (a) every registered stream has set its epoch_set flag — i.e.
     * each stream's first packet is on its ring — or (b) ipmx_pacer_gate_us
     * has elapsed. At release, T_anchor = av_gettime_relative() and each
     * primed stream gets pacer_offset_us = T_anchor - epoch_us. From that
     * point on, find_two_earliest and dispatch use the effective target
     *   effective_target_us = slot.target_us + st->pacer_offset_us
     * which anchors every stream's pacing schedule to the same wall-clock
     * reference, eliminating the startup burst seen when each stream's
     * producer-side epoch is set independently at first-packet time. */
    int64_t          gate_start_us;
    int              gate_open;
} IpmxPacer;

/* ------------------------------------------------------------------------- */

struct IpmxMuxContext;  /* forward */

typedef struct IpmxStreamRT {
    /* Back-pointer to the owning muxer context. */
    struct IpmxMuxContext *mc;

    /* Index of this stream in AVFormatContext->streams[]. */
    int      src_stream_index;

    /* Stream type. */
    int      is_audio;              /* 0 = video, 1 = audio */
    int      is_compressed_video;   /* 0 = raw / N/A, 1 = H.264 or H.265 (MIB 0x0005) */
    int      compressed_codec_id;   /* AV_CODEC_ID_H264 or AV_CODEC_ID_HEVC when is_compressed_video */

    /* Distinct dynamic RTP payload type per stream. Required for an
     * unambiguous bundle SDP: any RFC 3551 receiver dispatches fmtp
     * parameters by payload type (RTSPState.parse_fmtp iterates all
     * streams sharing the PT), so two m= blocks with the same PT cause
     * fmtp to be re-applied across contexts — re-entrant per-signal
     * stream allocation in rtpdec_am824 was the visible symptom. Video
     * uses ctx->video_payload_type (default 96); each audio stream gets
     * ctx->audio_payload_type + audio_idx (default 97, 98, 99, …). */
    int      payload_type;

    /* Per-stream RTP packetiser (a child ff_rtp_muxer instance). */
    AVFormatContext *child_rtp_ctx;

    /* Per-stream UDP sockets (RTP + RTCP, RTCP port = RTP port + 1). */
    URLContext *udp_h;
    URLContext *udp_rtcp_h;

    /* Per-stream AVIO interception (custom write callback feeds packet bytes
     * into ipmx_avio_write so we can pace + send + capture-to-PCAP). */
    AVIOContext *paced_avio;
    uint8_t     *avio_buf;

    /* Working buffer for assembling RTP datagrams between flushes. */
    uint8_t  *pkt_buf;
    int       pkt_buf_size;
    int       pkt_buf_capacity;

    /* Resolved destination IP/port (used in SDP and PCAP synthesis). */
    char      dst_ip_resolved[64];
    int       dst_port_resolved;
    int       dst_rtcp_port_resolved;

    /* Wall-clock anchor (av_gettime_relative() µs at first packet).
     * epoch_us is written by the producer thread once on the first packet,
     * BEFORE the release-store on epoch_set. The pacer thread (gate logic)
     * reads epoch_set with acquire, then epoch_us — so it sees a consistent
     * value pair. */
    int64_t     epoch_us;
    atomic_int  epoch_set;

    /* Pacer-side offset applied at dispatch: effective_target_us =
     * slot.target_us + pacer_offset_us. Set once when the pacer's startup
     * gate releases (see IpmxPacer.gate_open). 0 by default — when the gate
     * is disabled (ipmx_pacer_gate_us=0) or a stream primes after the gate
     * has already released, the effective target equals the producer-side
     * target. Written exclusively by the pacer thread; read by the pacer
     * thread; producers do not touch it. */
    int64_t     pacer_offset_us;

    /* Initial RTP TS sampled from Internal Clock at write_header (TR-10-1 §8.6). */
    uint32_t rtp_ts_base;

    /* Internal Clock value at the rtp_ts_base sampling instant — anchor for
     * NTP timestamps in RTCP SRs (paired with RTP-TS per TR-10-1 §8.7). */
    int64_t  internal_clock_at_epoch_ns;

    /* Per-frame pacing state (video). */
    int64_t  frame_epoch_us;        /* target time of frame's first packet */
    int      packet_in_frame;       /* index within the current frame */
    int      packets_per_frame;     /* RFC 4175 framing estimate */
    int64_t  t_frame_us;            /* nominal frame period in µs */

    /* Frame counter (presentation order). */
    int64_t  frame_idx;

    /* Marker tracking — RFC 4175 marker bit set on last packet of a frame. */
    int      saw_marker_pending;

    /* RTCP SR scheduling. */
    int      sr_due;
    uint32_t pending_sr_rtp_ts;

    /* Compressed-video SR rtp_ts is derived from the OBSERVED wire AU
     * timestamp delta, not from avg_frame_rate metadata. Container metadata
     * can lie (e.g. raw .h264 with H.264 SPS num_units_in_tick=25 fps yet
     * actual bitstream cadence 30 fps), in which case `frame_idx ×
     * (90000/avg_rate)` no longer matches the wire AU rtp_ts and SR-to-AU
     * association in the validator fails. We track each AU's first-packet
     * rtp_ts and the inter-AU step; SR_{N+1} emitted at AU_N's marker uses
     * `prev_au_first_rtp_ts + observed_au_ts_step`. The very first marker
     * (no step observed yet) defers SR_1 to the first packet of frame 1
     * where the actual rtp_ts is read from the wire. */
    uint32_t prev_au_first_rtp_ts;
    uint32_t observed_au_ts_step;

    /* RFC 4175 / video state. */
    int      width, height;
    int      pix_fmt;               /* AVPixelFormat */
    int      pgroup_octets;
    int      pgroup_pixels;
    int      bit_depth;
    int      payload_size;          /* RTP payload MTU (excluding IP/UDP/RTP) */
    int      bytes_per_frame;
    int      rate_num;              /* exact frame rate numerator   (e.g. 30) */
    int      rate_den;              /* exact frame rate denominator (e.g. 1)  */

    /* Compressed video state (H.264 / H.265). */
    int64_t  comp_max_bitrate_bps;  /* max_bitrate including IP/UDP/RTP per TR-10-7 §11.
                                     * Set from SPS-VUI HRD `bit_rate_value × 2^(6+bit_rate_scale)`
                                     * when `hrd_present == 1`; falls back to codecpar->bit_rate
                                     * otherwise (preserves legacy behaviour for non-HRD sources). */
    int64_t  comp_cum_payload_bytes;/* cumulative RTP payload bytes dispatched (whole stream — for diagnostics) */
    int64_t  comp_au_cum_bytes;     /* RTP payload bytes within current AU (resets on marker) */
    int64_t  comp_au_rtp_payload_bytes; /* RTP payload bytes within current AU (RTP header and extension
                                     * excluded, RFC 6184/7798 payload-format headers included). HRD
                                     * trace only. Resets on marker. */
    int64_t  comp_au_nal_unit_bytes;/* NAL unit bytes of the current AU, counted by ipmx_write_packet
                                     * from the AVPacket handed to the RTP packetizer. Used at marker
                                     * for the HRD drain time b(n)/BitRate (H.264 eq C-6, H.265 eq C-8)
                                     * of the inter-AU advance. BitRate counts the Type II bitstream,
                                     * so payload-format headers (FU, AP, STAP-A) must not be counted.
                                     * Resets on marker. */
    IpmxHrdTraceEntry *hrd_trace;   /* HRD pacing trace ring (ipmx_hrd_trace_cap entries; NULL = off) */
    unsigned hrd_trace_count;       /* records written (ring index = count % cap) */

    /* Parameter sets cached from codecpar->extradata for MIB 0x000A / 0x0009. */
    uint8_t *sps_data;   int sps_len;
    uint8_t *pps_data;   int pps_len;
    uint8_t *vps_data;   int vps_len;   /* H.265 only */
    int      h264_packetization_mode;   /* RFC 6184 §6.2 (1 = non-interleaved) */

    /* HEVC profile_tier_level fields, parsed once from the cached SPS RBSP at
     * write_header and reused by both the MIB builder (ipmx_build_h265_mib)
     * and the SDP fmtp augmentation in ipmx_sdp_emit_video_compressed_block.
     * All values are RFC 7798 fmtp parameter sources per AMWA BCP-006-03:
     *   general_profile_space  → profile-space=
     *   general_profile_idc    → profile-id=
     *   general_tier_flag      → tier-flag=
     *   general_level_idc      → level-id=
     *   general_profile_compatibility_flag[32] (bit-reversed) → profile-compatibility-indicator=
     *   general_constraint_indicator_flags (48 bits)          → interop-constraints=
     * hevc_ptl_valid is set to 1 when the parse succeeded (SPS present and
     * ≥ 13 bytes of RBSP). */
    uint8_t  hevc_profile_space;
    uint8_t  hevc_profile_idc;
    uint8_t  hevc_tier_flag;
    uint8_t  hevc_level_idc;
    uint32_t hevc_profile_compat;        /* already bit-reversed, ready to format as %08X */
    uint8_t  hevc_interop_constraints[6];/* big-endian, 48-bit field; format as 12 hex chars */
    int      hevc_ptl_valid;

    /* HRD state — populated at write_header by parsing SPS VUI HRD parameters.
     * Per TR-10-15c §15 / TR-10-15b §15 + H.264 §C.1.1 / H.265 §C.2.1.1.
     * If hrd_present == 0, the pacer falls back to the legacy `i·T` schedule
     * using codecpar->bit_rate (preserves backwards-compat for non-HRD inputs). */
    int      hrd_present;                       /* 1 if SPS provided nal_hrd_parameters */
    int      hrd_cbr_flag;                      /* cbr_flag[0] — selects CBR (eq C-2) vs VBR (eq C-3) branch */
    int      hrd_cpb_removal_delay_length;      /* SEI bit width for cpb_removal_delay (H.264) / au_cpb_removal_delay_minus1 (H.265) */
    int      hrd_initial_cpb_removal_delay_length; /* SEI bit width for initial_cpb_removal_delay[] */
    int64_t  hrd_max_bitrate_bps;               /* (bit_rate_value_minus1[0]+1) · 2^(6+bit_rate_scale) */
    int64_t  hrd_cpb_size_bits;                 /* (cpb_size_value_minus1[0]+1) · 2^(4+cpb_size_scale) */
    uint32_t vui_num_units_in_tick;             /* SPS VUI timing: clock tick t_c = num_units_in_tick / time_scale s; */
    uint32_t vui_time_scale;                    /* also the frame rate (ipmx_select_video_rate). H.264: 2 ticks per frame */

    unsigned hrd_bp_sei_count;                  /* buffering_period SEI parsed (HRD summary) */
    unsigned hrd_pt_sei_count;                  /* pic_timing SEI parsed (HRD summary) */

    /* Per-AU HRD runtime state — updated by the SEI walker on the clear access
     * unit (ipmx_write_packet); consumed by the conformance check at marker and (when the
     * VBR branch lands) by frame_epoch_us computation. Times in µs
     * relative to stream_epoch_us. The t_r,n schedule itself is kept in
     * clock ticks from AU 0's removal time (eq C-8 / C-9 only add
     * cpb_removal_delay ticks), so it accumulates no rounding;
     * hrd_t_r_n_j_us is the µs value of hrd_t_r_n_j_ticks. */
    int64_t  hrd_init_delay_us;                 /* most recent BP SEI initial_cpb_removal_delay (used by VBR `tai_earliest`) */
    int64_t  hrd_init_offset_us;                /* most recent BP SEI initial_cpb_removal_delay_offset (VBR only) */
    int64_t  hrd_init_delay_first_us;           /* cached AU-0's init_delay; for diagnostic logging at loop detection */
    int64_t  hrd_init_offset_first_us;          /* cached AU-0's init_offset; for diagnostic logging at loop detection */
    int64_t  hrd_t_r_n_base_us;                 /* t_r,n(0) = AU 0's initial_cpb_removal_delay (eq C-7) */
    int64_t  hrd_t_r_n_n_b_ticks;               /* t_r,n of the first AU of the CURRENT buffering period (C-9 anchor) */
    int64_t  hrd_t_r_n_n_b_prev_ticks;          /* t_r,n of the first AU of the PREVIOUS buffering period (C-8 anchor) */
    int64_t  hrd_t_r_n_j_ticks;                 /* t_r,n(j) of the CURRENT AU (just-seen pic_timing SEI) */
    int64_t  hrd_t_r_n_prev_au_ticks;           /* t_r,n(j-1) — monotonicity reference for loop / splice detection */
    int64_t  hrd_t_r_n_step_ticks;              /* last t_r,n(j) − t_r,n(j-1) > 0: the encoder's per-AU removal
                                                 * interval (re-anchor step; 0 until two AUs were seen) */
    int64_t  hrd_t_r_n_j_us;                    /* t_r,n(j) of the CURRENT AU, µs */
    int64_t  hrd_t_af_prev_us;                  /* t_af(j-1) — final-arrival time of previous AU's last byte (β-free) */
    int64_t  hrd_anchor_offset_us;              /* cumulative offset added to every t_r,n; bumped at loop / splice */
    int64_t  hrd_au_offset0_us;                 /* TR-10-15c §15 AU_offset(0): idle shift applied to AU 0's
                                                 * first byte on the wire (= encoder pre-fill drain duration).
                                                 * Set at first write_packet based on ctx->au_offset0_mode:
                                                 *   ZERO=0, MAX=cpb_drain, SYMMETRIC=cpb_drain-init_delay.
                                                 * SYMMETRIC requires init_delay from AU 0's BP SEI; computed
                                                 * lazily once the SEI walker has parsed it. AU 0 packets are
                                                 * buffered (hrd_au0_buf) until that point.
                                                 * Range: [0, cpb_size/max_bitrate] per TR-10-15c §15. */
    int      hrd_au_offset0_applied;            /* 1 once AU_offset(0) has been computed AND comp_pace_epoch_us
                                                 * has been set accordingly. Initially 0 for SYMMETRIC mode;
                                                 * 1 immediately for ZERO/MAX. */
    /* AU 0 RTP packet buffer — used only when au_offset0_mode == SYMMETRIC and BP SEI
     * has not yet been parsed. Holds packets that should be on the wire but whose
     * target_us depends on a still-unknown AU_offset(0). Flushed (= pushed to st->ring
     * with correct target_us) as soon as the SEI walker populates hrd_init_delay_first_us
     * (the BP SEI was found inside AU 0) — typically by AU 0's 3rd or 4th RTP packet. */
    struct IpmxAuBufferedPkt {
        uint8_t *data;          /* malloc'd copy of the RTP packet */
        int      len;
        int64_t  cum_bytes;     /* comp_au_cum_bytes through this packet (= wire bytes for within-AU pacing) */
        int      marker;        /* RTP marker bit captured from packet header */
        uint16_t seq;
        uint32_t rtp_ts;
    } *hrd_au0_buf;
    int       hrd_au0_buf_count;
    int       hrd_au0_buf_cap;
    int64_t  comp_pace_epoch_us;                /* H.264 §C.1.1 / H.265 §C.2.1.1: eq C-2 (CBR) or eq C-3 (VBR) value
                                                 * of t_ai(j) — the wallclock at which AU j's first byte hits the wire.
                                                 * Used by ipmx_comp_target_us_for_pkt for per-packet pacing.
                                                 * Distinct from frame_epoch_us (which carries the j·T SR cadence
                                                 * per TR-10-15c §15 schedule rule 1). For CBR these two values
                                                 * coincide for tight AU sizing; for VBR they differ on stalls. */
    int      hrd_au_idx;                        /* running AU counter since stream start (0-based) */
    int      hrd_at_new_bp;                     /* set when current AU is first of a new BP — controls C-7/C-8 vs C-9 */
    int      hrd_saw_bp_in_this_au;             /* per-AU scratch for §15 BP+recovery_point cross-check */
    int      hrd_saw_recovery_point_in_this_au; /* per-AU scratch (same) */
    int      hrd_saw_pt_in_this_au;             /* per-AU scratch: hrd_t_r_n_j_us is this AU's t_r,n */
    int64_t  hrd_tai_earliest_wall_us;          /* per-AU scratch: epoch_us + eq C-3 tai_earliest (VBR, AU > 0); 0 = none */
    int64_t  hrd_clamp_this_au_us;              /* HRD-ceiling reduction of this AU's comp_pace_epoch_us (trace) */
    int      hrd_seen_first_au;                 /* 0 until first BP/pic_timing seen (gates t_r,n computation) */

    /* Audio state (ST 2110-30 / TR-10-3 for PCM, ST 2110-31 / TR-10-12 for AM824). */
    int      audio_sample_rate;     /* e.g. 48000 */
    int      audio_channels;        /* e.g. 2 (stereo) */
    int      audio_sample_size;     /* bits per sample (e.g. 24 for L24) */
    int      audio_ptime_us;        /* nominal packet time in µs (e.g. 1000) */
    int      audio_sr_period_pkts;  /* SR every N packets, N = floor(10000/ptime_us) */
    int      audio_payload_bytes;   /* expected per-packet RTP payload */
    int      audio_periods_per_pkt; /* exact number of audio sample periods per RTP
                                     * packet — derived from ptime_us and sample_rate
                                     * at init. Used for sample-count and time
                                     * arithmetic that must match the actual on-wire
                                     * packet rate. ptime_us alone drifts at non-
                                     * integer-period ptimes (e.g. 1090 µs at
                                     * 44.1 kHz: ptime_us*sample_rate ≠ 48 *
                                     * sample_rate per packet). See SR scheduling
                                     * in `ipmx_avio_write` and the audio target_us
                                     * helper. */
    int      audio_is_am824;        /* 1 if wire format is AM824 (TR-10-12) */
    int      audio_is_non_pcm;      /* 1 if AM824 carries non-PCM (S337M / IEC 61937 burst, e.g. AAC) */
    char     audio_channel_order[32]; /* SDP/MIB channel-order, e.g. "SMPTE2110.(ST)" or "SMPTE2110.(51)" */
    int64_t  audio_pkt_idx;         /* running audio packet count */
    int64_t  audio_packet_epoch_us; /* wall-clock target of next audio pkt */

    /* RTP runtime — extracted from intercepted RTP header. */
    uint32_t ssrc;
    uint16_t last_seq;
    uint32_t last_rtp_ts;

    /* Cumulative for SR. */
    uint32_t pkt_count;
    uint32_t octet_count;

    /* Staleness-drop counters (see ipmx_drop_late_us in IpmxMuxContext).
     * Incremented by the pacer thread when a slot is dropped because its
     * effective target_us is older than now - drop_late_us. Reported on
     * muxer close; visible per-event in the in-memory trace ring as
     * IPMX_EVT_DROP_RTP / IPMX_EVT_DROP_SR. */
    uint64_t dropped_late_rtp;
    uint64_t dropped_late_sr;

    /* Producer-side back-pressure diagnostics. Incremented inside
     * ipmx_ring_push_into when the slot reservation has to spin waiting
     * for the pacer to release a slot. Used at write_trailer to surface
     * how often the producer thread (ffmpeg's main mux loop) was blocked
     * by a full ring — which back-pressures `-re` demux pacing and
     * shows up as ffmpeg's "Resumed reading at pts X after a lag of Y"
     * warning. */
    uint64_t ring_push_blocked_calls;
    uint64_t ring_push_spin_iters;
    int64_t  ring_push_blocked_us;
    int64_t  ring_push_blocked_max_us;
    uint64_t sr_ring_push_blocked_calls;
    int64_t  sr_ring_push_blocked_us;

    /* Per-stream MPSC ring (producer→pacer) for RTP packets. */
    IpmxStreamRing ring;
    /* Separate ring for RTCP SRs. SRs have their own target_us cadence
     * (TR-10-15c §15) which can be earlier than late RTP packets queued
     * mid-IDR-drain. Putting SRs in a separate ring lets the pacer's
     * min-of-N peek select them in target order independent of the RTP
     * FIFO ordering. */
    IpmxStreamRing sr_ring;

    /* Per-stream lookahead window for the pacer drain heuristic. Computed at
     * setup as K × natural_period (K_video × t_frame_us for video,
     * K_audio × audio_ptime_us for audio). Ensures the heap holds roughly
     * one frame-period worth of timeline per stream — bursts on a single
     * stream don't crowd out other streams' near-due events. */
    int64_t  lookahead_us;
} IpmxStreamRT;

typedef struct IpmxMuxContext {
    const AVClass *av_class;

    IpmxStreamRT     streams[IPMX_MAX_STREAMS];
    int              nb_streams;

    /* Internal Clock anchor (TR-10-1 §7.1) — same bit layout as PTPv2
     * truncated 64-bit (TR-10-1 §8.7 Figure 2). Sampled at write_header. */
    int64_t          internal_clock_anchor_unix_ns;
    int64_t          internal_clock_anchor_relative_us;

    /* AVOptions. */
    char            *refclk;
    char            *mediaclk;
    double           pace_beta;
    int              au_offset0_mode;   /* IpmxAuOffset0Mode — initial AU offset
                                         * selection (see enum docs). Applied at
                                         * first write_packet for compressed
                                         * video streams when hrd_present == 1. */
    int              pacer_spin_us;
    /* Late-burst fairness threshold. When the earliest pending packet has
     * target_us < now (late) AND another stream's ring head has target_us
     * within [now, now + pacer_imminent_us], dispatch *both* in sequence
     * (late first, imminent next) instead of letting subsequent late
     * packets from the same stream monopolise the dispatcher. Default
     * ~20 µs ≈ typical sendto + PCAP-write cost; 0 disables the feature. */
    int              pacer_imminent_us;
    /* Startup gate window (µs). At pacer thread start, dispatch is held
     * until every stream has its first packet on its ring OR this many µs
     * have elapsed; on release, all primed streams' targets are anchored
     * to a common wall-clock reference (T_anchor) via per-stream
     * pacer_offset_us. 0 disables the gate (back to per-stream-epoch
     * behavior). Default 50 ms — well above ffmpeg's pipeline-priming
     * window and well below user-perceptible startup latency. */
    int              pacer_gate_us;
    /* Staleness drop gate (µs). When > 0, the pacer drops any slot whose
     * effective target_us is older than now - drop_late_us instead of
     * dispatching it. Defensive backstop for fault scenarios — kernel
     * latency spike, CPU saturation, malformed input causing producer
     * stall — where the queue can otherwise accumulate hundreds of ms of
     * backlog and snowball into a long catch-up burst that wastes wire
     * bandwidth, mis-aligns receiver A/V (stale RTCP SRs), and starves
     * other streams. 0 (default) = disabled, every queued packet is
     * dispatched regardless of lateness. Recommended values when enabled:
     * audio = AES67 §7.5 SHALL bound (min(17·ptime, 17 ms) → 17000 µs);
     * video = one frame period (33 333 µs at 30 fps). Drops are counted
     * per-stream and logged on muxer close; trace ring records each drop
     * as IPMX_EVT_DROP_RTP / IPMX_EVT_DROP_SR with the same target/late
     * columns as a normal dispatch event. */
    int              drop_late_us;
    int              video_payload_type;
    int              video_pkt_size;
    int              audio_payload_type;
    int              audio_ptime_us;
    int              audio_base_port;    /* base RTP port for audio streams[1..] */
    /* PEP (TR-10-13) — when AVFormatContext::privacy_encrypt is on, the
     * RFC 8285 one-byte extension header carrying the IV/counter is added
     * by rtpenc.c; we record its extmap id so the IPMX SDP and MIB 0x0011
     * report the same values as the on-wire ext header. */
    int              pep_extmap_id_full;  /* SDP a=extmap id for PEP-Full-IV-Counter (1..14) */
    int              pep_extmap_id_short; /* SDP a=extmap id for PEP-Short-IV-Counter (1..14) */
    int              privacy_encrypt;     /* mirror of s->privacy_encrypt for SR builder use */
    /* Resolved hex strings used in SDP a=privacy line. Generated at
     * write_header time when the user doesn't override them, so the SDP
     * exactly matches the rtp_pep_init values inside the child rtp_ctx. */
    char             pep_iv_hex[17];           /* 16 hex chars + NUL  (8 bytes) */
    char             pep_key_generator_hex[33];/* 32 hex chars + NUL (16 bytes) */
    char             pep_key_version_hex[9];   /* 8 hex chars + NUL  (4 bytes) */
    char             pep_key_id_hex[17];       /* 16 hex chars + NUL (8 bytes) */
    char            *sdp_file;
    char            *pcap_file;
    char            *sender_mac;
    char            *src_ip;
    char            *cname;              /* RTCP SDES CNAME; default ipmx@<src-ip> */
    int              src_port_base;      /* synthesised PCAP src port; +i per stream */

    /* Pacer drain heuristic — per-stream lookahead window multipliers. */
    int              k_video;            /* lookahead_us(video) = k_video × t_frame_us */
    int              k_audio;            /* lookahead_us(audio) = k_audio × audio_ptime_us */
    int              idle_poll_us;       /* max sleep when all rings empty */
    int              pacer_trace_cap;    /* in-memory trace ring size (0 = disabled) */
    char            *pacer_trace_file;   /* dump path for trace at close (NULL = stderr) */
    int              hrd_trace_cap;      /* per-stream HRD pacing trace size, in AUs (0 = disabled) */
    char            *hrd_trace_file;     /* dump path for the HRD trace at close (NULL = stderr) */

    /* PCAP file handle. */
    FILE            *pcap_fp;
    uint16_t         pcap_ip_id_counter;

    /* Single shared pacer (per the plan). */
    IpmxPacer        pacer;
} IpmxMuxContext;

/* ─────────────── helpers ─────────────── */

static void ipmx_default_sender_mac(char buf[18])
{
    uint64_t r = av_get_random_seed();
    snprintf(buf, 18, "02-00-FC-%02X-%02X-%02X",
             (unsigned)(r & 0xff),
             (unsigned)((r >> 8) & 0xff),
             (unsigned)((r >> 16) & 0xff));
}

/* Exact packets-per-frame count for RFC 4175 raw video.
 *
 * The earlier closed-form `(line_bytes / per_pkt_pixels) × height` formula
 * structurally overestimates because rtpenc_rfc4175.c packs MULTIPLE scanline
 * segments per RTP packet via the RFC 4175 continuation flag whenever there is
 * room left after the previous scanline's data + 6-byte scanline header. With
 * 640×480 YUV420P at MTU 1400 the simple formula returns 480; the actual
 * count is 336 — which fed a wrong t_drain into the pacer for frame 0 (the
 * estimate is corrected at the first marker, so only frame 0 was affected).
 *
 * To get the right count up front we simulate ff_rtp_send_raw_rfc4175's
 * inner do-while loop. Constants and step rules are taken verbatim from
 * src/libavformat/rtpenc_rfc4175.c. */
static int ipmx_compute_packets_per_frame(int width, int height,
                                          int pgroup_octets, int pgroup_xinc,
                                          int pgroup_yinc,
                                          int payload_size)
{
    const int interlaced = 0;            /* IPMX project scope: progressive only. */
    const int xinc       = pgroup_xinc;
    const int yinc       = pgroup_yinc << interlaced;
    const int pgroup     = pgroup_octets;
    const int head_size  = 6;
    int i = 0;
    int offset = 0;
    int packets = 0;
    int guard = width * height;          /* termination safety */

    if (width <= 0 || height <= 0 || xinc <= 0 || pgroup <= 0 || payload_size <= 8)
        return 1;

    while (i < height && guard-- > 0) {
        int left = payload_size - 2;     /* 2-byte extended sequence number */
        int next_line, length, cont, pixels;
        do {
            pixels = width - offset;
            length = (pixels * pgroup) / xinc;
            left -= head_size;
            if (left >= length) {
                next_line = 1;
            } else {
                pixels  = (left / pgroup) * xinc;
                length  = (pixels * pgroup) / xinc;
                next_line = 0;
            }
            left -= length;
            if (next_line) i += yinc;
            cont = (left > (head_size + pgroup) && i < height) ? 1 : 0;
            if (next_line)
                offset = 0;
            else
                offset += pixels;
        } while (cont);
        packets++;
    }
    return packets > 0 ? packets : 1;
}

static void ipmx_set_dscp(URLContext *h, int tos)
{
    int fd;
    if (!h) return;
    fd = ffurl_get_file_handle(h);
    if (fd < 0) return;
    {
        int t = tos;
        (void)setsockopt(fd, IPPROTO_IP, IP_TOS, &t, sizeof(t));
    }
}

static void ipmx_clock_anchor_init(IpmxMuxContext *ctx)
{
    struct timespec ts;
    int64_t now_us = av_gettime_relative();
    if (clock_gettime(CLOCK_REALTIME, &ts) == 0) {
        ctx->internal_clock_anchor_unix_ns =
            (int64_t)ts.tv_sec * 1000000000LL + (int64_t)ts.tv_nsec;
    } else {
        ctx->internal_clock_anchor_unix_ns = (int64_t)time(NULL) * 1000000000LL;
    }
    ctx->internal_clock_anchor_relative_us = now_us;
}

static int64_t ipmx_clock_now_ns(const IpmxMuxContext *ctx)
{
    int64_t rel_us = av_gettime_relative();
    return ctx->internal_clock_anchor_unix_ns
         + (rel_us - ctx->internal_clock_anchor_relative_us) * 1000LL;
}

/* ─────────────── parameter-set parsing (H.264 / H.265) ─────────────── */

/* Walk an Annex B byte stream and dispatch each NAL to a callback.
 * Recognises both 3-byte (0x00 00 01) and 4-byte (0x00 00 00 01) start codes. */
static void ipmx_walk_annexb(const uint8_t *buf, int len,
                             void (*cb)(const uint8_t *nal, int nal_len, void *opaque),
                             void *opaque)
{
    int i = 0;
    while (i < len) {
        int sc_len = 0;
        int start, end;
        if (i + 3 < len && buf[i] == 0 && buf[i+1] == 0 && buf[i+2] == 0 && buf[i+3] == 1) sc_len = 4;
        else if (i + 2 < len && buf[i] == 0 && buf[i+1] == 0 && buf[i+2] == 1) sc_len = 3;
        if (!sc_len) { i++; continue; }
        start = i + sc_len;
        end = len;
        for (i = start; i + 2 < len; i++) {
            if (buf[i] == 0 && buf[i+1] == 0 && (buf[i+2] == 0 || buf[i+2] == 1)) {
                end = i;
                break;
            }
        }
        if (i + 2 >= len) { end = len; i = len; }
        if (end > start) cb(buf + start, end - start, opaque);
    }
}

typedef struct IpmxNalCacheCtx {
    IpmxStreamRT *st;
    int           is_h265;
} IpmxNalCacheCtx;

static void ipmx_cache_nal_cb(const uint8_t *nal, int nal_len, void *opaque)
{
    IpmxNalCacheCtx *cx = opaque;
    IpmxStreamRT *st = cx->st;
    int nal_type;
    if (nal_len < 1) return;
    if (cx->is_h265) {
        if (nal_len < 2) return;
        nal_type = (nal[0] >> 1) & 0x3f;
        if (nal_type == 32 /* VPS */) {
            av_freep(&st->vps_data);
            st->vps_data = av_malloc(nal_len);
            if (st->vps_data) { memcpy(st->vps_data, nal, nal_len); st->vps_len = nal_len; }
        } else if (nal_type == 33 /* SPS */) {
            av_freep(&st->sps_data);
            st->sps_data = av_malloc(nal_len);
            if (st->sps_data) { memcpy(st->sps_data, nal, nal_len); st->sps_len = nal_len; }
        } else if (nal_type == 34 /* PPS */) {
            av_freep(&st->pps_data);
            st->pps_data = av_malloc(nal_len);
            if (st->pps_data) { memcpy(st->pps_data, nal, nal_len); st->pps_len = nal_len; }
        }
    } else {
        nal_type = nal[0] & 0x1f;
        if (nal_type == 7 /* SPS */) {
            av_freep(&st->sps_data);
            st->sps_data = av_malloc(nal_len);
            if (st->sps_data) { memcpy(st->sps_data, nal, nal_len); st->sps_len = nal_len; }
        } else if (nal_type == 8 /* PPS */) {
            av_freep(&st->pps_data);
            st->pps_data = av_malloc(nal_len);
            if (st->pps_data) { memcpy(st->pps_data, nal, nal_len); st->pps_len = nal_len; }
        }
    }
}

/* Walk an AVCC (H.264 avcC box) or HVCC (H.265 hvcC box) extradata blob and
 * dispatch each parameter-set NAL through ipmx_cache_nal_cb. */
static void ipmx_walk_avcc(const uint8_t *buf, int len, int is_h265,
                           void (*cb)(const uint8_t *nal, int nal_len, void *opaque),
                           void *opaque)
{
    int i;
    if (len < 7 || buf[0] != 0x01) return;
    if (is_h265) {
        /* HVCC: skip 22 bytes of fixed header, then numOfArrays. */
        if (len < 23) return;
        {
            int n_arr = buf[22];
            i = 23;
            for (int a = 0; a < n_arr && i + 3 <= len; a++) {
                int n_nalus;
                /* array_completeness:1, reserved:1, NAL_unit_type:6 (1 byte),
                 * num_nalus:2 — the per-array header is 3 bytes. */
                i += 1;  /* skip first byte (NAL type) */
                if (i + 2 > len) return;
                n_nalus = AV_RB16(buf + i); i += 2;
                for (int u = 0; u < n_nalus && i + 2 <= len; u++) {
                    int sz = AV_RB16(buf + i); i += 2;
                    if (i + sz > len) return;
                    cb(buf + i, sz, opaque);
                    i += sz;
                }
            }
        }
    } else {
        /* AVCC: skip 5-byte header, then 1-byte (3-msb reserved + 5-bit
         * numOfSPS), then SPS list, then 1-byte numOfPPS, then PPS list. */
        int n_sps, n_pps;
        i = 5;
        n_sps = buf[i] & 0x1f; i += 1;
        for (int u = 0; u < n_sps && i + 2 <= len; u++) {
            int sz = AV_RB16(buf + i); i += 2;
            if (i + sz > len) return;
            cb(buf + i, sz, opaque);
            i += sz;
        }
        if (i >= len) return;
        n_pps = buf[i]; i += 1;
        for (int u = 0; u < n_pps && i + 2 <= len; u++) {
            int sz = AV_RB16(buf + i); i += 2;
            if (i + sz > len) return;
            cb(buf + i, sz, opaque);
            i += sz;
        }
    }
}

static void ipmx_cache_param_sets(IpmxStreamRT *st,
                                  const uint8_t *buf, int len, int is_h265)
{
    IpmxNalCacheCtx cx = { .st = st, .is_h265 = is_h265 };
    /* AVCC / HVCC extradata begins with configurationVersion = 0x01.
     * Annex B begins with a start code (0x00000001 or 0x000001). */
    if (len > 0 && buf[0] == 0x01)
        ipmx_walk_avcc(buf, len, is_h265, ipmx_cache_nal_cb, &cx);
    else
        ipmx_walk_annexb(buf, len, ipmx_cache_nal_cb, &cx);
}

/* ─────────────────── SPS VUI HRD parameter extraction ───────────────────
 *
 * Parses the SPS NAL unit's VUI HRD section (H.264 Annex E / H.265 Annex E)
 * and captures the fields needed by the HRD-aware pacer:
 *   - num_units_in_tick / time_scale → vui_num_units_in_tick / vui_time_scale
 *   - nal_hrd_parameters.bit_rate_value_minus1[0] / bit_rate_scale → hrd_max_bitrate_bps
 *   - nal_hrd_parameters.cpb_size_value_minus1[0] / cpb_size_scale → hrd_cpb_size_bits
 *   - cbr_flag[0] → hrd_cbr_flag (selects CBR eq C-2 vs VBR eq C-3 branch)
 *   - initial_cpb_removal_delay_length_minus1 → hrd_initial_cpb_removal_delay_length (SEI bit widths)
 *   - cpb_removal_delay_length_minus1 / au_cpb_removal_delay_length_minus1 → hrd_cpb_removal_delay_length
 *
 * Pattern mirrors libavformat/avc.c:ff_avc_decode_sps (H.264) and
 * libavformat/hevc.c:skip_hrd_parameters (H.265) — uses get_bits.h for the
 * bit reader, with TR-10-15c/b §15-mandated simplifications:
 *   - Only nal_hrd_parameters is used (Type-II HRD, vcl_hrd ignored).
 *   - cpb_cnt_minus1 must be 0 (single CPB).
 *   - sub_pic_hrd_params_present_flag = 1 returns AVERROR(ENOSYS).
 *
 * On success: returns 0 with all hrd_* fields populated and hrd_present = 1.
 * On absence of nal_hrd: returns 0 with hrd_present = 0 (pacer falls back).
 * On unsupported mode: returns AVERROR(ENOSYS). */

/* H.264 only: skip past the SPS prelude (profile/level/scaling lists/etc.)
 * to reach vui_parameters_present_flag. Returns < 0 on parse error. */
static int ipmx_h264_skip_to_vui(GetBitContext *gb)
{
    unsigned int profile_idc, chroma_format_idc, pic_order_cnt_type;
    unsigned int num_ref_frames_in_pic_order_cnt_cycle;

    profile_idc = get_bits(gb, 8);
    skip_bits(gb, 8);                                       /* constraint_set flags + reserved */
    skip_bits(gb, 8);                                       /* level_idc */
    get_ue_golomb(gb);                                      /* seq_parameter_set_id */

    if (profile_idc == 100 || profile_idc == 110 || profile_idc == 122 ||
        profile_idc == 244 || profile_idc == 44  || profile_idc == 83  ||
        profile_idc == 86  || profile_idc == 118 || profile_idc == 128 ||
        profile_idc == 138 || profile_idc == 139 || profile_idc == 134) {
        chroma_format_idc = get_ue_golomb(gb);
        if (chroma_format_idc == 3)
            skip_bits1(gb);                                 /* separate_colour_plane_flag */
        get_ue_golomb(gb);                                  /* bit_depth_luma_minus8 */
        get_ue_golomb(gb);                                  /* bit_depth_chroma_minus8 */
        skip_bits1(gb);                                     /* qpprime_y_zero_transform_bypass_flag */
        if (get_bits1(gb)) {                                /* seq_scaling_matrix_present_flag */
            int i, j, sizeOfScalingList, nextScale, lastScale, delta_scale;
            int lists = (chroma_format_idc != 3) ? 8 : 12;
            for (i = 0; i < lists; i++) {
                if (!get_bits1(gb))
                    continue;
                lastScale = 8; nextScale = 8;
                sizeOfScalingList = i < 6 ? 16 : 64;
                for (j = 0; j < sizeOfScalingList; j++) {
                    if (nextScale != 0) {
                        delta_scale = get_se_golomb(gb);
                        nextScale = (lastScale + delta_scale) & 0xff;
                    }
                    lastScale = nextScale == 0 ? lastScale : nextScale;
                }
            }
        }
    }

    get_ue_golomb(gb);                                      /* log2_max_frame_num_minus4 */
    pic_order_cnt_type = get_ue_golomb(gb);
    if (pic_order_cnt_type == 0) {
        get_ue_golomb(gb);                                  /* log2_max_pic_order_cnt_lsb_minus4 */
    } else if (pic_order_cnt_type == 1) {
        skip_bits1(gb);                                     /* delta_pic_order_always_zero_flag */
        get_se_golomb(gb);                                  /* offset_for_non_ref_pic */
        get_se_golomb(gb);                                  /* offset_for_top_to_bottom_field */
        num_ref_frames_in_pic_order_cnt_cycle = get_ue_golomb(gb);
        if (num_ref_frames_in_pic_order_cnt_cycle > 255)
            return AVERROR_INVALIDDATA;
        for (unsigned int i = 0; i < num_ref_frames_in_pic_order_cnt_cycle; i++)
            get_se_golomb(gb);                              /* offset_for_ref_frame */
    }
    get_ue_golomb(gb);                                      /* max_num_ref_frames */
    skip_bits1(gb);                                         /* gaps_in_frame_num_value_allowed_flag */
    get_ue_golomb(gb);                                      /* pic_width_in_mbs_minus1 */
    get_ue_golomb(gb);                                      /* pic_height_in_map_units_minus1 */
    {
        int frame_mbs_only_flag = get_bits1(gb);
        if (!frame_mbs_only_flag)
            skip_bits1(gb);                                 /* mb_adaptive_frame_field_flag */
    }
    skip_bits1(gb);                                         /* direct_8x8_inference_flag */
    if (get_bits1(gb)) {                                    /* frame_cropping_flag */
        get_ue_golomb(gb);                                  /* frame_crop_left_offset */
        get_ue_golomb(gb);                                  /* frame_crop_right_offset */
        get_ue_golomb(gb);                                  /* frame_crop_top_offset */
        get_ue_golomb(gb);                                  /* frame_crop_bottom_offset */
    }
    return 0;
}

/* H.264 / H.265 share an identical hrd_parameters() bit layout. Parses one
 * hrd_parameters() block, capturing the fields we need on success. */
static int ipmx_extract_hrd_parameters_block(GetBitContext *gb, IpmxStreamRT *st,
                                             void *log_ctx)
{
    unsigned int cpb_cnt_minus1, bit_rate_scale, cpb_size_scale;
    unsigned int br_val, cs_val;
    unsigned int initial_len, cpb_removal_len;

    cpb_cnt_minus1 = get_ue_golomb(gb);
    if (cpb_cnt_minus1 != 0) {
        av_log(log_ctx, AV_LOG_WARNING,
               "IPMX HRD: cpb_cnt_minus1 = %u; TR-10-15c §15 mandates 0. Falling back to legacy schedule.\n",
               cpb_cnt_minus1);
        return AVERROR(ENOSYS);
    }

    bit_rate_scale = get_bits(gb, 4);
    cpb_size_scale = get_bits(gb, 4);

    /* get_ue_golomb_long for these — typical values (e.g. 15624 for 4 Mbps
     * @ scale=2) exceed get_ue_golomb's 8190 ceiling. */
    br_val = get_ue_golomb_long(gb);                        /* bit_rate_value_minus1[0] */
    cs_val = get_ue_golomb_long(gb);                        /* cpb_size_value_minus1[0] */
    st->hrd_cbr_flag = get_bits1(gb);                       /* cbr_flag[0] */

    initial_len     = get_bits(gb, 5);                      /* initial_cpb_removal_delay_length_minus1 */
    cpb_removal_len = get_bits(gb, 5);                      /* cpb_removal_delay_length_minus1 */
    skip_bits(gb, 5);                                       /* dpb_output_delay_length_minus1 */
    skip_bits(gb, 5);                                       /* time_offset_length */

    st->hrd_max_bitrate_bps                   = (int64_t)(br_val + 1) << (6 + bit_rate_scale);
    st->hrd_cpb_size_bits                     = (int64_t)(cs_val + 1) << (4 + cpb_size_scale);
    st->hrd_initial_cpb_removal_delay_length  = (int)(initial_len + 1);
    st->hrd_cpb_removal_delay_length          = (int)(cpb_removal_len + 1);
    return 0;
}

static int ipmx_extract_sps_hrd_h264(IpmxStreamRT *st, void *log_ctx)
{
    GetBitContext gb;
    uint8_t *rbsp;
    uint32_t rbsp_len;
    int ret;

    if (!st->sps_data || st->sps_len < 1)
        return 0;                                           /* no SPS — hrd_present stays 0 */

    /* SPS NAL byte 0 is the nal_header (type=7). Strip it and de-escape. */
    rbsp = ff_nal_unit_extract_rbsp(st->sps_data + 1, st->sps_len - 1, &rbsp_len, 0);
    if (!rbsp)
        return AVERROR(ENOMEM);

    ret = init_get_bits8(&gb, rbsp, rbsp_len);
    if (ret < 0)
        goto end;

    ret = ipmx_h264_skip_to_vui(&gb);
    if (ret < 0)
        goto end;

    if (!get_bits1(&gb)) {                                  /* vui_parameters_present_flag */
        ret = 0;                                            /* no VUI; hrd_present stays 0 */
        goto end;
    }

    /* VUI: walk fields until we reach timing_info / hrd_parameters. */
    if (get_bits1(&gb)) {                                   /* aspect_ratio_info_present_flag */
        unsigned int idc = get_bits(&gb, 8);
        if (idc == 0xff) skip_bits_long(&gb, 32);           /* sar_width:u16, sar_height:u16 */
    }
    if (get_bits1(&gb))  skip_bits1(&gb);                   /* overscan_info_present + overscan_appropriate */
    if (get_bits1(&gb)) {                                   /* video_signal_type_present_flag */
        skip_bits(&gb, 4);                                  /* video_format(3) + video_full_range_flag(1) */
        if (get_bits1(&gb))                                 /* colour_description_present_flag */
            skip_bits(&gb, 24);                             /* colour_primaries + transfer + matrix */
    }
    if (get_bits1(&gb)) {                                   /* chroma_loc_info_present_flag */
        get_ue_golomb(&gb);
        get_ue_golomb(&gb);
    }
    if (get_bits1(&gb)) {                                   /* timing_info_present_flag */
        unsigned int num_units = get_bits_long(&gb, 32);
        unsigned int time_scale = get_bits_long(&gb, 32);
        skip_bits1(&gb);                                    /* fixed_frame_rate_flag */
        /* H.264 convention: time_scale = 2 × frame_rate; t_c = num_units_in_tick / time_scale. */
        if (time_scale > 0 && num_units > 0) {
            st->vui_num_units_in_tick = num_units;
            st->vui_time_scale        = time_scale;
        }
    }
    if (get_bits1(&gb)) {                                   /* nal_hrd_parameters_present_flag */
        ret = ipmx_extract_hrd_parameters_block(&gb, st, log_ctx);
        if (ret < 0)
            goto end;
        st->hrd_present = 1;
    }
    /* vcl_hrd_parameters / low_delay_hrd_flag / pic_struct_present_flag /
     * bitstream_restriction_flag: we don't need them for the pacer. */

end:
    av_free(rbsp);
    return ret;
}

/* H.265 helper: skip through profile_tier_level(maxNumSubLayersMinus1).
 * Mirrors libavformat/hevc.c's parse_ptl. */
static void ipmx_h265_skip_ptl(GetBitContext *gb, unsigned int max_sub_layers_minus1)
{
    int i;
    skip_bits(gb, 2);                                       /* general_profile_space */
    skip_bits1(gb);                                         /* general_tier_flag */
    skip_bits(gb, 5);                                       /* general_profile_idc */
    skip_bits_long(gb, 32);                                 /* general_profile_compatibility_flag[32] */
    skip_bits_long(gb, 32);                                 /* general_progressive..frame_only_constraint_flag + reserved (48 bits) */
    skip_bits(gb, 16);
    skip_bits(gb, 8);                                       /* general_level_idc */

    {
        int sub_layer_profile_present[8] = {0};
        int sub_layer_level_present[8]   = {0};
        for (i = 0; i < (int)max_sub_layers_minus1; i++) {
            sub_layer_profile_present[i] = get_bits1(gb);
            sub_layer_level_present[i]   = get_bits1(gb);
        }
        if (max_sub_layers_minus1 > 0)
            for (i = (int)max_sub_layers_minus1; i < 8; i++)
                skip_bits(gb, 2);                           /* reserved_zero_2bits */
        for (i = 0; i < (int)max_sub_layers_minus1; i++) {
            if (sub_layer_profile_present[i])
                skip_bits_long(gb, 88);                     /* same shape as general (8+32+32+16) */
            if (sub_layer_level_present[i])
                skip_bits(gb, 8);
        }
    }
}

/* H.265 short_term_ref_pic_set parser — borrowed from libavformat/hevc.c
 * (parse_rps at line 561). Walks past the variable-length RPS to reach
 * fields beyond. Returns 0 on success. */
#define IPMX_H265_MAX_RPS  64
static int ipmx_h265_parse_rps(GetBitContext *gb, unsigned int rps_idx,
                               unsigned int num_rps,
                               unsigned int *num_delta_pocs)
{
    unsigned int i;
    if (rps_idx && get_bits1(gb)) {                         /* inter_ref_pic_set_prediction_flag */
        if (rps_idx >= num_rps)
            return AVERROR_INVALIDDATA;
        skip_bits1(gb);                                     /* delta_rps_sign */
        get_ue_golomb_long(gb);                             /* abs_delta_rps_minus1 */
        num_delta_pocs[rps_idx] = 0;
        for (i = 0; i <= num_delta_pocs[rps_idx - 1]; i++) {
            uint8_t use_delta_flag = 0;
            uint8_t used = get_bits1(gb);
            if (!used) use_delta_flag = get_bits1(gb);
            if (used || use_delta_flag) num_delta_pocs[rps_idx]++;
        }
    } else {
        unsigned int num_neg = get_ue_golomb_long(gb);
        unsigned int num_pos = get_ue_golomb_long(gb);
        if ((num_pos + (uint64_t)num_neg) * 2 > (uint64_t)get_bits_left(gb))
            return AVERROR_INVALIDDATA;
        num_delta_pocs[rps_idx] = num_neg + num_pos;
        for (i = 0; i < num_neg; i++) { get_ue_golomb_long(gb); skip_bits1(gb); }
        for (i = 0; i < num_pos; i++) { get_ue_golomb_long(gb); skip_bits1(gb); }
    }
    return 0;
}

/* H.265: skip the SPS prelude up to vui_parameters_present_flag. */
static int ipmx_h265_skip_to_vui(GetBitContext *gb, unsigned int *out_max_sub_layers_minus1)
{
    unsigned int max_sub_layers_minus1;
    unsigned int chroma_format_idc;
    unsigned int log2_max_poc_lsb_m4;
    unsigned int num_rps;
    unsigned int num_delta_pocs[IPMX_H265_MAX_RPS] = {0};
    int i;

    skip_bits(gb, 4);                                       /* sps_video_parameter_set_id */
    max_sub_layers_minus1 = get_bits(gb, 3);
    if (max_sub_layers_minus1 > 6)
        return AVERROR_INVALIDDATA;
    *out_max_sub_layers_minus1 = max_sub_layers_minus1;
    skip_bits1(gb);                                         /* sps_temporal_id_nesting_flag */
    ipmx_h265_skip_ptl(gb, max_sub_layers_minus1);
    get_ue_golomb(gb);                                      /* sps_seq_parameter_set_id */
    chroma_format_idc = get_ue_golomb(gb);
    if (chroma_format_idc == 3)
        skip_bits1(gb);                                     /* separate_colour_plane_flag */
    get_ue_golomb(gb);                                      /* pic_width_in_luma_samples */
    get_ue_golomb(gb);                                      /* pic_height_in_luma_samples */
    if (get_bits1(gb)) {                                    /* conformance_window_flag */
        get_ue_golomb(gb); get_ue_golomb(gb);
        get_ue_golomb(gb); get_ue_golomb(gb);
    }
    get_ue_golomb(gb);                                      /* bit_depth_luma_minus8 */
    get_ue_golomb(gb);                                      /* bit_depth_chroma_minus8 */
    log2_max_poc_lsb_m4 = get_ue_golomb(gb);                /* log2_max_pic_order_cnt_lsb_minus4 */
    if (log2_max_poc_lsb_m4 > 12) return AVERROR_INVALIDDATA;
    {
        int sub_layer_ordering_info_present = get_bits1(gb);
        int start = sub_layer_ordering_info_present ? 0 : (int)max_sub_layers_minus1;
        for (i = start; i <= (int)max_sub_layers_minus1; i++) {
            get_ue_golomb_long(gb);                         /* sps_max_dec_pic_buffering_minus1 */
            get_ue_golomb_long(gb);                         /* sps_max_num_reorder_pics */
            get_ue_golomb_long(gb);                         /* sps_max_latency_increase_plus1 */
        }
    }
    get_ue_golomb(gb);                                      /* log2_min_luma_coding_block_size_minus3 */
    get_ue_golomb(gb);                                      /* log2_diff_max_min_luma_coding_block_size */
    get_ue_golomb(gb);                                      /* log2_min_luma_transform_block_size_minus2 */
    get_ue_golomb(gb);                                      /* log2_diff_max_min_luma_transform_block_size */
    get_ue_golomb(gb);                                      /* max_transform_hierarchy_depth_inter */
    get_ue_golomb(gb);                                      /* max_transform_hierarchy_depth_intra */
    if (get_bits1(gb)) {                                    /* scaling_list_enabled_flag */
        if (get_bits1(gb)) {                                /* sps_scaling_list_data_present_flag */
            int sizeId, matrixId;
            for (sizeId = 0; sizeId < 4; sizeId++) {
                int matrix_max = (sizeId == 3) ? 2 : 6;
                for (matrixId = 0; matrixId < matrix_max; matrixId++) {
                    if (!get_bits1(gb)) {
                        get_ue_golomb_long(gb);             /* scaling_list_pred_matrix_id_delta */
                    } else {
                        int coefNum = FFMIN(64, (1 << (4 + (sizeId << 1))));
                        if (sizeId > 1) get_se_golomb_long(gb);
                        for (int k = 0; k < coefNum; k++)
                            get_se_golomb_long(gb);
                    }
                }
            }
        }
    }
    skip_bits1(gb);                                         /* amp_enabled_flag */
    skip_bits1(gb);                                         /* sample_adaptive_offset_enabled_flag */
    if (get_bits1(gb)) {                                    /* pcm_enabled_flag */
        skip_bits(gb, 4);                                   /* pcm_sample_bit_depth_luma_minus1 */
        skip_bits(gb, 4);                                   /* pcm_sample_bit_depth_chroma_minus1 */
        get_ue_golomb(gb);                                  /* log2_min_pcm_luma_coding_block_size_minus3 */
        get_ue_golomb(gb);                                  /* log2_diff_max_min_pcm_luma_coding_block_size */
        skip_bits1(gb);                                     /* pcm_loop_filter_disabled_flag */
    }

    /* num_short_term_ref_pic_sets — walk via parse_rps. */
    num_rps = get_ue_golomb(gb);
    if (num_rps > IPMX_H265_MAX_RPS)
        return AVERROR_INVALIDDATA;
    for (i = 0; i < (int)num_rps; i++) {
        int ret = ipmx_h265_parse_rps(gb, (unsigned int)i, num_rps, num_delta_pocs);
        if (ret < 0) return ret;
    }

    if (get_bits1(gb)) {                                    /* long_term_ref_pics_present_flag */
        unsigned int num_lt = get_ue_golomb(gb);
        unsigned int bits_per_lsb = log2_max_poc_lsb_m4 + 4;
        for (unsigned int j = 0; j < num_lt; j++) {
            skip_bits(gb, bits_per_lsb);                    /* lt_ref_pic_poc_lsb_sps */
            skip_bits1(gb);                                 /* used_by_curr_pic_lt_sps_flag */
        }
    }
    skip_bits1(gb);                                         /* sps_temporal_mvp_enabled_flag */
    skip_bits1(gb);                                         /* strong_intra_smoothing_enabled_flag */
    return 0;
}

static int ipmx_extract_sps_hrd_h265(IpmxStreamRT *st, void *log_ctx)
{
    GetBitContext gb;
    uint8_t *rbsp;
    uint32_t rbsp_len;
    unsigned int max_sub_layers_minus1 = 0;
    int ret;

    if (!st->sps_data || st->sps_len < 2)
        return 0;

    /* H.265 NAL header is 2 bytes. Strip + de-escape. */
    rbsp = ff_nal_unit_extract_rbsp(st->sps_data + 2, st->sps_len - 2, &rbsp_len, 0);
    if (!rbsp)
        return AVERROR(ENOMEM);

    ret = init_get_bits8(&gb, rbsp, rbsp_len);
    if (ret < 0)
        goto end;

    ret = ipmx_h265_skip_to_vui(&gb, &max_sub_layers_minus1);
    if (ret < 0)
        goto end;

    if (!get_bits1(&gb)) {                                  /* vui_parameters_present_flag */
        ret = 0;
        goto end;
    }

    /* H.265 VUI prelude — walk to vui_timing_info_present_flag. */
    if (get_bits1(&gb)) {                                   /* aspect_ratio_info_present_flag */
        unsigned int idc = get_bits(&gb, 8);
        if (idc == 0xff) skip_bits_long(&gb, 32);
    }
    if (get_bits1(&gb)) skip_bits1(&gb);                    /* overscan_info + overscan_appropriate */
    if (get_bits1(&gb)) {                                   /* video_signal_type_present_flag */
        skip_bits(&gb, 4);                                  /* video_format + video_full_range_flag */
        if (get_bits1(&gb))                                 /* colour_description_present_flag */
            skip_bits(&gb, 24);
    }
    if (get_bits1(&gb)) {                                   /* chroma_loc_info_present_flag */
        get_ue_golomb(&gb); get_ue_golomb(&gb);
    }
    skip_bits1(&gb);                                        /* neutral_chroma_indication_flag */
    skip_bits1(&gb);                                        /* field_seq_flag */
    skip_bits1(&gb);                                        /* frame_field_info_present_flag */
    if (get_bits1(&gb)) {                                   /* default_display_window_flag */
        get_ue_golomb(&gb); get_ue_golomb(&gb);
        get_ue_golomb(&gb); get_ue_golomb(&gb);
    }
    if (get_bits1(&gb)) {                                   /* vui_timing_info_present_flag */
        unsigned int num_units = get_bits_long(&gb, 32);
        unsigned int time_scale = get_bits_long(&gb, 32);
        /* H.265 convention: t_c = num_units_in_tick / time_scale directly. */
        if (time_scale > 0 && num_units > 0) {
            st->vui_num_units_in_tick = num_units;
            st->vui_time_scale        = time_scale;
        }
        if (get_bits1(&gb))                                 /* vui_poc_proportional_to_timing_flag */
            get_ue_golomb_long(&gb);                        /* vui_num_ticks_poc_diff_one_minus1 */
        if (get_bits1(&gb)) {                               /* vui_hrd_parameters_present_flag */
            /* Refuse sub_pic_hrd_params_present_flag = 1 per plan. */
            int nal_hrd_present, vcl_hrd_present, sub_pic;
            nal_hrd_present = get_bits1(&gb);
            vcl_hrd_present = get_bits1(&gb);
            sub_pic = 0;
            if (nal_hrd_present || vcl_hrd_present) {
                sub_pic = get_bits1(&gb);
                if (sub_pic) {
                    av_log(log_ctx, AV_LOG_ERROR,
                           "IPMX TR-10-15b sub-picture HRD not supported (sub_pic_hrd_params_present_flag = 1)\n");
                    ret = AVERROR(ENOSYS);
                    goto end;
                }
                /* bit_rate_scale(4) + cpb_size_scale(4) = 8 bits before sub_pic block (not taken). */
                {
                    unsigned int bit_rate_scale = get_bits(&gb, 4);
                    unsigned int cpb_size_scale = get_bits(&gb, 4);
                    unsigned int initial_len, cpb_removal_len;
                    /* sub_pic_hrd_params_present_flag is 0 here → no cpb_size_du_scale. */
                    initial_len     = get_bits(&gb, 5);     /* initial_cpb_removal_delay_length_minus1 */
                    cpb_removal_len = get_bits(&gb, 5);     /* au_cpb_removal_delay_length_minus1 */
                    skip_bits(&gb, 5);                      /* dpb_output_delay_length_minus1 */
                    st->hrd_initial_cpb_removal_delay_length = (int)(initial_len + 1);
                    st->hrd_cpb_removal_delay_length         = (int)(cpb_removal_len + 1);
                    /* Now walk the per-sub-layer hrd_parameters() block, capturing
                     * fields from sub-layer 0 (the only one TR-10-15b requires). */
                    {
                        unsigned int sl;
                        for (sl = 0; sl <= max_sub_layers_minus1; sl++) {
                            int fixed_pic_rate_general = get_bits1(&gb);
                            int fixed_pic_rate_within_cvs = fixed_pic_rate_general;
                            int low_delay_hrd = 0;
                            unsigned int cpb_cnt = 0;
                            if (!fixed_pic_rate_general)
                                fixed_pic_rate_within_cvs = get_bits1(&gb);
                            if (fixed_pic_rate_within_cvs)
                                get_ue_golomb(&gb);         /* elemental_duration_in_tc_minus1 */
                            else
                                low_delay_hrd = get_bits1(&gb);
                            if (!low_delay_hrd)
                                cpb_cnt = get_ue_golomb(&gb);
                            if (cpb_cnt != 0) {
                                av_log(log_ctx, AV_LOG_WARNING,
                                       "IPMX HRD: cpb_cnt_minus1 = %u; TR-10-15b §15 mandates 0.\n",
                                       cpb_cnt);
                                ret = AVERROR(ENOSYS);
                                goto end;
                            }
                            if (nal_hrd_present) {
                                unsigned int br_val, cs_val;
                                /* get_ue_golomb_long: HRD values commonly exceed
                                 * get_ue_golomb's 8190 ceiling. */
                                br_val = get_ue_golomb_long(&gb);
                                cs_val = get_ue_golomb_long(&gb);
                                /* sub_pic_hrd flag is 0 → no cpb_size_du / bit_rate_du fields. */
                                {
                                    int cbr_flag = get_bits1(&gb);
                                    if (sl == 0) {
                                        st->hrd_cbr_flag        = cbr_flag;
                                        st->hrd_max_bitrate_bps = (int64_t)(br_val + 1) << (6 + bit_rate_scale);
                                        st->hrd_cpb_size_bits   = (int64_t)(cs_val + 1) << (4 + cpb_size_scale);
                                    }
                                }
                            }
                            if (vcl_hrd_present) {
                                get_ue_golomb_long(&gb);     /* bit_rate_value_minus1 */
                                get_ue_golomb_long(&gb);     /* cpb_size_value_minus1 */
                                skip_bits1(&gb);             /* cbr_flag */
                            }
                        }
                    }
                    if (nal_hrd_present)
                        st->hrd_present = 1;
                }
            }
        }
    }

end:
    av_free(rbsp);
    return ret;
}

/* Dispatcher — called once at write_header per compressed-video stream after
 * ipmx_cache_param_sets has populated st->sps_data. */
static int ipmx_extract_sps_hrd(IpmxStreamRT *st, int codec_id, void *log_ctx)
{
    int ret;
    if (codec_id == AV_CODEC_ID_H264)
        ret = ipmx_extract_sps_hrd_h264(st, log_ctx);
    else if (codec_id == AV_CODEC_ID_HEVC)
        ret = ipmx_extract_sps_hrd_h265(st, log_ctx);
    else
        ret = 0;

    if (ret == AVERROR(ENOSYS)) {
        /* Spec-unsupported HRD configuration (sub-pic HRD, non-zero cpb_cnt,
         * etc.). Caller decides whether to abort or fall back. For the muxer
         * we treat this as "HRD-unaware fallback" and proceed with the legacy
         * codecpar->bit_rate schedule. */
        st->hrd_present = 0;
        ret = 0;
    }
    return ret;
}

/* ─────────────────── Per-AU SEI walker (hot path) ───────────────────────
 *
 * Hand-rolled, zero-allocation parser for the three SEI payload types that
 * carry HRD timing data:
 *   - payload_type 0 (buffering_period):
 *       H.264 §D.1.1 — initial_cpb_removal_delay[0], initial_cpb_removal_delay_offset[0]
 *       H.265 §D.2.2 — nal_initial_cpb_removal_delay[0], nal_initial_cpb_removal_offset[0],
 *                       concatenation_flag, irap_cpb_params_present_flag
 *   - payload_type 1 (pic_timing): cpb_removal_delay (H.264) / au_cpb_removal_delay_minus1+1 (H.265)
 *   - payload_type 6 (recovery_point): recovery_frame_cnt (diagnostic only)
 *
 * Invoked from ipmx_avio_write on each RTP packet's NAL data when a stream
 * has hrd_present == 1. Walks the RTP packet's NAL units (RFC 6184 / 7798
 * single-NAL or aggregated forms), skipping VCL by nal_unit_type, and only
 * decodes the three payload types above. Updates `st->hrd_*` runtime fields
 * in place. */

/* Common SEI payload extractor. Returns < 0 on parse error. The `payload`
 * buffer is the raw SEI payload bytes WITHOUT the payload_type/payload_size
 * prefix — we de-escape it via ff_nal_unit_extract_rbsp so emulation_prevention
 * bytes don't break the bit reader. */

static int ipmx_hrd_extract_bp(IpmxStreamRT *st, int is_h265,
                               const uint8_t *payload, int payload_size,
                               void *log_ctx)
{
    GetBitContext gb;
    uint8_t *rbsp;
    uint32_t rbsp_len;
    int ret = 0;
    int len_bits;

    rbsp = ff_nal_unit_extract_rbsp(payload, payload_size, &rbsp_len, 0);
    if (!rbsp)
        return AVERROR(ENOMEM);
    ret = init_get_bits8(&gb, rbsp, rbsp_len);
    if (ret < 0)
        goto end;

    len_bits = st->hrd_initial_cpb_removal_delay_length;
    if (len_bits <= 0 || len_bits > 32) {
        ret = AVERROR_INVALIDDATA;
        goto end;
    }

    if (is_h265) {
        /* H.265 §D.2.2: bp_seq_parameter_set_id (ue), irap_cpb_params_present_flag (u1),
         * optional cpb_delay_offset + dpb_delay_offset (each u(au_cpb_removal_delay_length)),
         * concatenation_flag (u1), au_cpb_removal_delay_delta_minus1 (u(au_cpb_removal_delay_length)),
         * then nal_initial_cpb_removal_delay[0] and nal_initial_cpb_removal_offset[0]. */
        unsigned int irap_cpb;
        get_ue_golomb(&gb);                                     /* bp_seq_parameter_set_id */
        irap_cpb = get_bits1(&gb);                              /* irap_cpb_params_present_flag */
        if (irap_cpb) {
            av_log(log_ctx, AV_LOG_ERROR,
                   "IPMX TR-10-15b: BLA / IRAP alt-CPB params not supported "
                   "(irap_cpb_params_present_flag = 1 in BP SEI)\n");
            ret = AVERROR(ENOSYS);
            goto end;
        }
        /* Per H.265 §D.2.2, when irap_cpb_params_present_flag = 0, the
         * cpb_delay_offset / dpb_delay_offset fields are NOT present. */
        {
            int concat = get_bits1(&gb);                        /* concatenation_flag */
            int aulen = st->hrd_cpb_removal_delay_length > 0 ? st->hrd_cpb_removal_delay_length : 1;
            if (aulen > 0 && aulen <= 32) skip_bits(&gb, aulen);/* au_cpb_removal_delay_delta_minus1 */
            else skip_bits1(&gb);                               /* defensive */
            if (concat) {
                av_log(log_ctx, AV_LOG_WARNING,
                       "IPMX TR-10-15b: concatenation_flag = 1 in BP SEI at AU=%d — "
                       "max-of-two-bases timing formula (C-10) not implemented; "
                       "falling back to simple C-8 anchor (may produce off-by-N timing).\n",
                       st->hrd_au_idx);
                /* Continue with simple branch — non-fatal for our IPMX use case. */
            }
        }
        /* nal_initial_cpb_removal_delay[0] + nal_initial_cpb_removal_offset[0],
         * each u(initial_cpb_removal_delay_length) bits. */
        {
            uint32_t init_delay  = get_bits_long(&gb, len_bits);
            uint32_t init_offset = get_bits_long(&gb, len_bits);
            st->hrd_init_delay_us  = (int64_t)init_delay  * 1000000 / 90000;
            st->hrd_init_offset_us = (int64_t)init_offset * 1000000 / 90000;
        }
    } else {
        /* H.264 §D.1.1: seq_parameter_set_id (ue), then for each SchedSelIdx
         * up to cpb_cnt_minus1 (= 0 per TR-10-15c §15):
         *   initial_cpb_removal_delay  u(initial_cpb_removal_delay_length)
         *   initial_cpb_removal_delay_offset  u(same length)
         */
        get_ue_golomb(&gb);                                     /* seq_parameter_set_id */
        {
            uint32_t init_delay  = get_bits_long(&gb, len_bits);
            uint32_t init_offset = get_bits_long(&gb, len_bits);
            st->hrd_init_delay_us  = (int64_t)init_delay  * 1000000 / 90000;
            st->hrd_init_offset_us = (int64_t)init_offset * 1000000 / 90000;
        }
    }
    st->hrd_saw_bp_in_this_au = 1;
    st->hrd_at_new_bp = 1;
    st->hrd_bp_sei_count++;
    if (!st->hrd_seen_first_au) {
        st->hrd_init_delay_first_us = st->hrd_init_delay_us;
        st->hrd_init_offset_first_us = st->hrd_init_offset_us;
    }
end:
    av_free(rbsp);
    return ret;
}

/* t_r,n in µs relative to the stream epoch for a schedule position in clock
 * ticks from AU 0 (t_c = num_units_in_tick / time_scale), rounded once. */
static int64_t ipmx_hrd_t_r_n_us(const IpmxStreamRT *st, int64_t ticks)
{
    if (!st->vui_time_scale)
        return st->hrd_t_r_n_base_us;
    return st->hrd_t_r_n_base_us +
           av_rescale(ticks, (int64_t)st->vui_num_units_in_tick * 1000000, st->vui_time_scale);
}

static int ipmx_hrd_extract_pic_timing(IpmxStreamRT *st, int is_h265,
                                       const uint8_t *payload, int payload_size,
                                       void *log_ctx)
{
    GetBitContext gb;
    uint8_t *rbsp;
    uint32_t rbsp_len;
    int ret = 0;
    int len_bits;
    int64_t cpb_removal_delay = 0;        /* raw SEI value (H.265: +1 already applied), in clock ticks */
    int64_t prospective_ticks;

    rbsp = ff_nal_unit_extract_rbsp(payload, payload_size, &rbsp_len, 0);
    if (!rbsp)
        return AVERROR(ENOMEM);
    ret = init_get_bits8(&gb, rbsp, rbsp_len);
    if (ret < 0)
        goto end;

    len_bits = st->hrd_cpb_removal_delay_length;
    if (len_bits <= 0 || len_bits > 32) {
        ret = AVERROR_INVALIDDATA;
        goto end;
    }

    if (is_h265) {
        /* H.265 pic_timing payload starts with au_cpb_removal_delay_minus1
         * (u(au_cpb_removal_delay_length_minus1+1)). The +1 converts spec's
         * "minus1" to the actual cpb_removal_delay we use. */
        uint32_t au_cpb_minus1 = get_bits_long(&gb, len_bits);
        cpb_removal_delay = (int64_t)au_cpb_minus1 + 1;
    } else {
        /* H.264 §D.1.2 pic_timing: cpc_dpb_delays_present implies the first
         * field is cpb_removal_delay u(cpb_removal_delay_length_minus1+1). */
        uint32_t cpb_rm = get_bits_long(&gb, len_bits);
        cpb_removal_delay = (int64_t)cpb_rm;
    }
    st->hrd_pt_sei_count++;
    st->hrd_saw_pt_in_this_au = 1;

    /* Compute t_r,n(j) per H.264 §C.1.2 / H.265 §C.2.3 (non-concatenation branch).
     * Three cases:
     *   - First AU ever (HRD init):           t_r,n(0) = init_delay_us  (eq C-7)
     *   - AU j > 0, first of NEW BP:          t_r,n(j) = t_r,n(n_b_prev) + cpb_removal_delay·t_c (eq C-8)
     *   - AU j > 0, NOT first of BP:          t_r,n(j) = t_r,n(n_b)      + cpb_removal_delay·t_c (eq C-9)
     */
    if (!st->hrd_seen_first_au) {
        /* AU 0 — HRD init. Apply C-7. */
        st->hrd_t_r_n_base_us        = st->hrd_init_delay_us;
        st->hrd_t_r_n_j_ticks        = 0;
        st->hrd_t_r_n_n_b_ticks      = 0;
        st->hrd_t_r_n_n_b_prev_ticks = 0;
        st->hrd_seen_first_au = 1;
    } else if (st->hrd_at_new_bp) {
        /* Eq C-8: anchored at PREVIOUS BP's first-AU t_r,n (a re-anchored
         * one included). */
        st->hrd_t_r_n_n_b_prev_ticks = st->hrd_t_r_n_n_b_ticks;
        prospective_ticks = st->hrd_t_r_n_n_b_prev_ticks + cpb_removal_delay;
        /* Loop / splice detection: if the C-8 result would be NON-monotonic
         * relative to the previous AU's t_r,n, the bitstream has restarted
         * (stream_loop wrap). Re-anchor: this AU continues the removal
         * schedule one AU interval past the previous AU — the last observed
         * t_r,n step, i.e. the encoder's own cpb_removal_delay increment.
         * One clock tick is not one AU: for H.264, time_scale usually
         * counts fields (2 ticks per frame). hrd_anchor_offset_us reports
         * the shift relative to eq C-7 for this AU's init_delay. */
        if (prospective_ticks <= st->hrd_t_r_n_prev_au_ticks) {
            int64_t step_ticks = st->hrd_t_r_n_step_ticks > 0 ? st->hrd_t_r_n_step_ticks : 1;
            st->hrd_t_r_n_j_ticks    = st->hrd_t_r_n_prev_au_ticks + step_ticks;
            st->hrd_t_r_n_n_b_ticks  = st->hrd_t_r_n_j_ticks;
            st->hrd_anchor_offset_us = ipmx_hrd_t_r_n_us(st, st->hrd_t_r_n_j_ticks) -
                                       st->hrd_init_delay_us;
            av_log(log_ctx, AV_LOG_VERBOSE,
                   "IPMX HRD re-anchor at AU=%d; step=%lld ticks anchor_offset=%lld µs, "
                   "init_delay matches first=%s\n",
                   st->hrd_au_idx, (long long)step_ticks, (long long)st->hrd_anchor_offset_us,
                   (st->hrd_init_delay_us == st->hrd_init_delay_first_us) ? "yes" : "no");
        } else {
            st->hrd_t_r_n_j_ticks   = prospective_ticks;
            st->hrd_t_r_n_n_b_ticks = st->hrd_t_r_n_j_ticks;
        }
    } else {
        /* Eq C-9: anchored at CURRENT BP's first-AU t_r,n. */
        st->hrd_t_r_n_j_ticks = st->hrd_t_r_n_n_b_ticks + cpb_removal_delay;
    }
    st->hrd_t_r_n_j_us = ipmx_hrd_t_r_n_us(st, st->hrd_t_r_n_j_ticks);

    /* Refine comp_pace_epoch_us per H.264 §C.1.1 / H.265 §C.2.1.1:
     *   - AU 0 (hrd_au_idx == 0): pacing epoch already set to stream epoch
     *     (epoch_us) at first-packet init — eq C-2/C-3 give t_ai(0) = 0
     *     relative to stream_epoch, i.e. comp_pace_epoch_us = epoch_us.
     *   - CBR (cbr_flag = 1, eq C-2): t_ai(j) = t_af(j-1). Already set by
     *     the marker handler to epoch_us + hrd_t_af_prev_us — nothing to do.
     *   - VBR (cbr_flag = 0, eq C-3): t_ai(j) = max(t_af(j-1), tai_earliest(j)).
     *     Compute tai_earliest from t_r,n(j), init_delay, and (when not first
     *     AU of a new BP) init_offset. Override comp_pace_epoch_us if the max
     *     picks tai_earliest (encoder declared a stall).
     */
    if (st->hrd_seen_first_au && st->hrd_au_idx > 0 && !st->hrd_cbr_flag) {
        int64_t tai_earliest_us;
        if (st->hrd_at_new_bp) {
            /* C-5 (H.264) / C-7 (H.265): tai_earliest = t_r,n(j) − init_delay/90000 */
            tai_earliest_us = st->hrd_t_r_n_j_us - st->hrd_init_delay_us;
        } else {
            /* C-4 (H.264) / C-6 (H.265): tai_earliest = t_r,n(j) − (init_delay + init_offset)/90000 */
            tai_earliest_us = st->hrd_t_r_n_j_us - st->hrd_init_delay_us - st->hrd_init_offset_us;
        }
        /* tai_earliest_us is relative to stream_epoch (t_r,n is relative too
         * after anchor_offset application). t_af_prev_us is also relative to
         * stream_epoch. So both are in the same frame. The HRD ceiling
         * (ipmx_hrd_apply_ceiling) never pulls the pace below it. */
        st->hrd_tai_earliest_wall_us = st->epoch_us + tai_earliest_us;
        if (tai_earliest_us > st->hrd_t_af_prev_us) {
            /* Encoder declared a stall — wire goes idle until tai_earliest(j).
             * MUST clamp upward only: the marker handler in the previous AU
             * advanced comp_pace_epoch_us by max(t_frame_us, t_af_advance_us),
             * which may already exceed epoch_us + tai_earliest_us if prior
             * AUs accumulated lag (e.g., long mandelbrot drains followed by
             * tiny black-segment AUs whose tai_earliest sits back near T_pour
             * cadence). Setting comp_pace_epoch backward would make AU j+1
             * fire earlier than frame_epoch_us — frame_epoch only advances
             * by T per marker, so SR(j+1) (target = frame_epoch − 50 µs)
             * would land AFTER wire AU j+1, breaking TR-10-15c-131 / -134a
             * SR-before-AU. Observed empirically on h264_burst with small
             * vbv-bufsize. */
            int64_t new_pace = st->epoch_us + tai_earliest_us;
            if (new_pace > st->comp_pace_epoch_us) {
                int64_t advanced_by = new_pace - st->comp_pace_epoch_us;
                st->comp_pace_epoch_us = new_pace;
                av_log(log_ctx, AV_LOG_VERBOSE,
                       "IPMX HRD eq C-3 stall at AU=%d: tai_earliest=%lld µs > t_af_prev=%lld µs "
                       "→ comp_pace_epoch advanced by %lld µs\n",
                       st->hrd_au_idx, (long long)tai_earliest_us,
                       (long long)st->hrd_t_af_prev_us,
                       (long long)advanced_by);
            }
        }
        /* else: t_af_prev >= tai_earliest → max picks t_af_prev → the marker
         * handler already set comp_pace_epoch_us = epoch_us + t_af_prev. */
    }

    st->hrd_at_new_bp = 0;      /* consumed */

end:
    av_free(rbsp);
    return ret;
}

static int ipmx_hrd_extract_recovery_point(IpmxStreamRT *st, int is_h265,
                                           const uint8_t *payload, int payload_size,
                                           void *log_ctx)
{
    GetBitContext gb;
    uint8_t *rbsp;
    uint32_t rbsp_len;
    int ret = 0;

    rbsp = ff_nal_unit_extract_rbsp(payload, payload_size, &rbsp_len, 0);
    if (!rbsp)
        return AVERROR(ENOMEM);
    ret = init_get_bits8(&gb, rbsp, rbsp_len);
    if (ret < 0)
        goto end;

    /* recovery_point: recovery_frame_cnt (H.264 ue) or recovery_poc_cnt (H.265 se),
     * then flags. Diagnostic only — we don't use this for timing. */
    if (is_h265) {
        int recovery_poc_cnt = get_se_golomb(&gb);
        av_log(log_ctx, AV_LOG_VERBOSE,
               "IPMX HRD: recovery_point at AU=%d, recovery_poc_cnt=%d\n",
               st->hrd_au_idx, recovery_poc_cnt);
    } else {
        unsigned int recovery_frame_cnt = get_ue_golomb_long(&gb);
        av_log(log_ctx, AV_LOG_VERBOSE,
               "IPMX HRD: recovery_point at AU=%d, recovery_frame_cnt=%u\n",
               st->hrd_au_idx, recovery_frame_cnt);
    }
    st->hrd_saw_recovery_point_in_this_au = 1;
end:
    av_free(rbsp);
    return ret;
}

/* Walk a SEI NAL unit's payload list per H.264/H.265 §7.3.2.x. Each payload
 * is preceded by a payload_type and payload_size that are encoded as
 * 0xff-extension sequences. Only payload_types 0 / 1 / 6 dispatch to our
 * extractors; everything else is skipped. */
static int ipmx_hrd_walk_sei(IpmxStreamRT *st, int is_h265,
                             const uint8_t *payload, int len, void *log_ctx)
{
    int i = 0;
    while (i < len) {
        int payload_type = 0, payload_size = 0;
        while (i < len && payload[i] == 0xff) { payload_type += 255; i++; }
        if (i < len) { payload_type += payload[i]; i++; }
        while (i < len && payload[i] == 0xff) { payload_size += 255; i++; }
        if (i < len) { payload_size += payload[i]; i++; }
        if (payload_size < 0 || i + payload_size > len) break;

        switch (payload_type) {
        case 0:  /* buffering_period */
            ipmx_hrd_extract_bp(st, is_h265, payload + i, payload_size, log_ctx);
            break;
        case 1:  /* pic_timing */
            ipmx_hrd_extract_pic_timing(st, is_h265, payload + i, payload_size, log_ctx);
            break;
        case 6:  /* recovery_point */
            ipmx_hrd_extract_recovery_point(st, is_h265, payload + i, payload_size, log_ctx);
            break;
        default:
            break;  /* skip — we only care about 3 payload types */
        }
        i += payload_size;
        /* rbsp_trailing_bits within the SEI payload list — if the next byte
         * is 0x80 (sole rbsp_trailing_bit '1' + zeros), we're done. */
        if (i < len && payload[i] == 0x80) break;
    }
    return 0;
}

/* Per-NAL callback dispatched by ipmx_walk_annexb. Filters VCL NAL types by
 * nal_unit_type and only invokes the SEI walker for SEI NAL types. */
typedef struct IpmxHrdAuCtx {
    IpmxStreamRT *st;
    int           is_h265;
    void         *log_ctx;
} IpmxHrdAuCtx;

static void ipmx_hrd_au_nal_cb(const uint8_t *nal, int nal_len, void *opaque)
{
    IpmxHrdAuCtx *cx = opaque;
    int nal_type;
    if (nal_len < 1) return;
    if (cx->is_h265) {
        if (nal_len < 2) return;
        nal_type = (nal[0] >> 1) & 0x3f;
        /* H.265 PREFIX_SEI_NUT = 39. SUFFIX_SEI_NUT = 40 — we ignore that
         * (post-decoding info, not HRD timing). VCL types are 0-31; skip. */
        if (nal_type != 39) return;
        /* H.265 SEI NAL: 2-byte header + payload list. */
        ipmx_hrd_walk_sei(cx->st, 1, nal + 2, nal_len - 2, cx->log_ctx);
    } else {
        nal_type = nal[0] & 0x1f;
        /* H.264 SEI NUT = 6. VCL types: 1-5, 19, 20. Skip everything but SEI. */
        if (nal_type != 6) return;
        ipmx_hrd_walk_sei(cx->st, 0, nal + 1, nal_len - 1, cx->log_ctx);
    }
}

/* Walk the NAL units of one H.264/H.265 access unit (an AVPacket), split
 * exactly as ff_rtp_send_h264_hevc() splits it: length-prefixed when
 * nal_length_size > 0, Annex B otherwise. This is the clear access unit; RTP
 * packetisation and PEP / HKEP encryption happen afterwards, inside the
 * child RTP muxer. */
static void ipmx_walk_au_nal_units(const uint8_t *buf, int size, int nal_length_size,
                                   void (*cb)(const uint8_t *nal, int nal_len, void *opaque),
                                   void *opaque)
{
    const uint8_t *r, *end = buf + size;

    if (nal_length_size)
        r = ff_nal_mp4_find_startcode(buf, end, nal_length_size) ? buf : end;
    else
        r = ff_nal_find_startcode(buf, end);
    while (r < end) {
        const uint8_t *r1;
        if (nal_length_size) {
            r1 = ff_nal_mp4_find_startcode(r, end, nal_length_size);
            if (!r1)
                r1 = end;
            r += nal_length_size;
        } else {
            while (!*(r++));
            r1 = ff_nal_find_startcode(r, end);
        }
        cb(r, r1 - r, opaque);
        r = r1;
    }
}

/* Per-NAL work on the clear access unit (ipmx_write_packet):
 *   - NAL unit byte count: b(n) of H.264 eq C-6 / sizeInbits of H.265 eq C-8
 *     (÷ 8) at the Type II conformance point, which counts "all bytes of the
 *     Type II bitstream, which may be a NAL unit stream or a byte stream"
 *     (H.264 / H.265 §C.1). RFC 6184 / RFC 7798 carry this NAL unit stream;
 *     start codes, length prefixes, payload-format headers (FU, STAP-A, AP)
 *     and PEP MACs are not part of it.
 *   - in-band parameter-set cache refresh (MIB 0x000A / 0x0009);
 *   - HRD SEI walker (buffering_period / pic_timing / recovery_point). */
typedef struct IpmxAuNalCtx {
    IpmxStreamRT *st;
    int           is_h265;
    void         *log_ctx;
    int64_t       nal_unit_bytes;
} IpmxAuNalCtx;

static void ipmx_au_nal_cb(const uint8_t *nal, int nal_len, void *opaque)
{
    IpmxAuNalCtx *cx = opaque;
    IpmxNalCacheCtx cache = { .st = cx->st, .is_h265 = cx->is_h265 };

    cx->nal_unit_bytes += nal_len;
    ipmx_cache_nal_cb(nal, nal_len, &cache);
    if (cx->st->hrd_present) {
        IpmxHrdAuCtx hrd = { .st = cx->st, .is_h265 = cx->is_h265, .log_ctx = cx->log_ctx };
        ipmx_hrd_au_nal_cb(nal, nal_len, &hrd);
    }
}

/* HRD ceiling for AU j > 0 (ipmx_write_packet, after the AU's NAL walk and
 * before its first RTP packet). The previous AU's marker set the pace to the
 * lower bound max(frame_epoch(j), t_af(j-1)), and the pic_timing walker may
 * have raised it to the eq C-3 tai_earliest(j). That cumulative MAX can walk
 * upward without bound, so the pace is capped where AU j's last byte still
 * arrives by its removal time:
 *
 *   ceiling(j) = epoch + t_r,n(j) − drain(j),  drain(j) = b(j)·8 / BitRate
 *
 * with this AU's own t_r,n (pic_timing SEI) and NAL unit bytes. Capping below
 * t_af(j-1) starts AU j before AU j-1's last byte (eq C-2 / C-3 not met) to
 * keep t_af(j) ≤ t_r,n(j). The pace never goes below frame_epoch(j)
 * (SR-before-AU) or tai_earliest(j); when the ceiling lies below those, AU
 * j's last byte is late and the marker's "HRD violation" diagnostic reports
 * it. */
static void ipmx_hrd_apply_ceiling(IpmxStreamRT *st, void *log_ctx)
{
    int64_t drain_us, ceiling_us, capped_us;

    st->hrd_clamp_this_au_us = 0;
    if (!st->hrd_present || st->hrd_au_idx == 0 || !st->hrd_saw_pt_in_this_au ||
        st->hrd_max_bitrate_bps <= 0)
        return;
    drain_us   = (int64_t)((double)st->comp_au_nal_unit_bytes * 8.0 * 1e6 /
                           (double)st->hrd_max_bitrate_bps);
    ceiling_us = st->epoch_us + st->hrd_t_r_n_j_us - drain_us;
    capped_us  = FFMAX(ceiling_us, FFMAX(st->frame_epoch_us, st->hrd_tai_earliest_wall_us));
    if (capped_us >= st->comp_pace_epoch_us)
        return;
    st->hrd_clamp_this_au_us = st->comp_pace_epoch_us - capped_us;
    av_log(log_ctx, AV_LOG_VERBOSE,
           "IPMX HRD-ceiling clamp at AU=%d: pace reduced by %lld µs to keep t_af ≤ t_r,n "
           "(ceiling %lld µs below the pace)\n",
           st->hrd_au_idx, (long long)st->hrd_clamp_this_au_us,
           (long long)(st->comp_pace_epoch_us - ceiling_us));
    st->comp_pace_epoch_us = capped_us;
}

/* Base64 encoder (RFC 4648, no line breaks) — writes encoded bytes into out
 * and returns the encoded length. Caller must size out for ((len+2)/3)*4. */
static int ipmx_b64_encode(char *out, int max_out, const uint8_t *in, int len)
{
    static const char tab[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    int i, o = 0;
    int need = ((len + 2) / 3) * 4;
    if (max_out < need) return -1;
    for (i = 0; i + 2 < len; i += 3) {
        uint32_t v = ((uint32_t)in[i] << 16) | ((uint32_t)in[i+1] << 8) | (uint32_t)in[i+2];
        out[o++] = tab[(v >> 18) & 0x3f];
        out[o++] = tab[(v >> 12) & 0x3f];
        out[o++] = tab[(v >> 6)  & 0x3f];
        out[o++] = tab[(v)       & 0x3f];
    }
    if (i < len) {
        uint32_t v = (uint32_t)in[i] << 16;
        if (i + 1 < len) v |= (uint32_t)in[i+1] << 8;
        out[o++] = tab[(v >> 18) & 0x3f];
        out[o++] = tab[(v >> 12) & 0x3f];
        out[o++] = (i + 1 < len) ? tab[(v >> 6) & 0x3f] : '=';
        out[o++] = '=';
    }
    return o;
}

/* ─────────────── MIB builders ─────────────── */

/* H.264 MIB type 0x000A (TR-10-15c §16) — byte layout mirrors
 * streams/ipmx_sender_report.py H264MediaInfoBlock.to_bytes() and the
 * sprop-content convention used by streams/ipmx_add_sender_reports_pcap.py
 * (raw SPS NAL bytes in sprop_parameter_sets, raw PPS NAL bytes in
 * sprop_level_parameter_sets). */
static int ipmx_build_h264_mib(uint8_t *p, int max, IpmxStreamRT *st)
{
    int payload_off = 4;
    uint32_t mask = 0;
    int profile_level_id_present;
    int sps_len = st->sps_len, pps_len = st->pps_len;
    int total_payload, total_bytes, words_minus_1;
    int pkt_mode = st->h264_packetization_mode > 0 ? st->h264_packetization_mode : 1;

    /* Build the canonical RFC 6184 §8.1 sprop-parameter-sets value:
     *   "<base64(SPS)>,<base64(PPS)>"
     * stored as ASCII in MIB sprop_parameter_sets (TR-10-15c §16: "the same
     * syntax as in the 'a=fmtp' line"). MIB sprop_level_parameter_sets is
     * for alternative-level SPS NALs only, which our streams don't carry. */
    char sprop_b64[1024];
    int  sprop_len = 0;
    if (sps_len > 0) {
        int n = ipmx_b64_encode(sprop_b64, sizeof(sprop_b64) - 2, st->sps_data, sps_len);
        if (n < 0) return AVERROR(ENOSPC);
        sprop_len += n;
    }
    if (pps_len > 0) {
        if (sprop_len > 0 && sprop_len < (int)sizeof(sprop_b64)) sprop_b64[sprop_len++] = ',';
        if (sprop_len + (((pps_len + 2) / 3) * 4) >= (int)sizeof(sprop_b64))
            return AVERROR(ENOSPC);
        {
            int n = ipmx_b64_encode(sprop_b64 + sprop_len,
                                    sizeof(sprop_b64) - sprop_len,
                                    st->pps_data, pps_len);
            if (n < 0) return AVERROR(ENOSPC);
            sprop_len += n;
        }
    }
    /* Single-byte length field in the MIB layout caps at 255 ASCII chars.
     * For our 640×480 streams the SPS+PPS base64 is well under 100 bytes,
     * so this isn't tight in practice. */
    if (sprop_len > 0xff) sprop_len = 0xff;

    profile_level_id_present = (sps_len >= 4);  /* need 3 profile bytes after NAL header */

    /* Field-presence bitmask (FIELD_BITS in streams/ipmx_sender_report.py:407): */
    if (profile_level_id_present)   mask |= 1u << 0;   /* profile_level_id */
    /* packetization_mode always present */            mask |= 1u << 1;
    if (sprop_len > 0)              mask |= 1u << 6;   /* sprop_parameter_sets */

    if (max < payload_off + 4) return AVERROR(ENOSPC);
    AV_WB32(p + payload_off, mask);            payload_off += 4;

    /* profile_level_id (3 bytes from SPS bytes 1..3 — profile_idc, constraint, level_idc). */
    if (max < payload_off + 3) return AVERROR(ENOSPC);
    if (profile_level_id_present) {
        p[payload_off + 0] = st->sps_data[1];
        p[payload_off + 1] = st->sps_data[2];
        p[payload_off + 2] = st->sps_data[3];
    } else {
        memset(p + payload_off, 0, 3);
    }
    payload_off += 3;

    if (max < payload_off + 1) return AVERROR(ENOSPC);
    p[payload_off++] = (uint8_t)pkt_mode;            /* packetization_mode */

    /* sprop_max_don_diff (2), sprop_interleaving_depth (2),
     * sprop_deint_buf_req (4), sprop_init_buf_time (4) — all zero (mode 1). */
    if (max < payload_off + 12) return AVERROR(ENOSPC);
    memset(p + payload_off, 0, 12); payload_off += 12;

    /* Length descriptors: 1B sprop_len, 1B level_sets_len, 1B extra_len, 1B reserved. */
    if (max < payload_off + 4 + sprop_len) return AVERROR(ENOSPC);
    p[payload_off++] = (uint8_t)sprop_len;          /* sprop_parameter_sets length */
    p[payload_off++] = 0;                            /* sprop_level_parameter_sets length (empty) */
    p[payload_off++] = 0;                            /* extra_bytes length */
    p[payload_off++] = 0;                            /* reserved padding */
    if (sprop_len > 0) { memcpy(p + payload_off, sprop_b64, sprop_len); payload_off += sprop_len; }

    /* word-boundary pad to 32-bit alignment. */
    while ((payload_off - 4) & 3) {
        if (payload_off >= max) return AVERROR(ENOSPC);
        p[payload_off++] = 0;
    }

    /* Header (2 bytes type, 2 bytes length-words-minus-1) at p[0..3]. */
    total_payload = payload_off - 4;
    total_bytes   = 4 + total_payload;
    words_minus_1 = (total_bytes / 4) - 1;
    AV_WB16(p,     IPMX_MIB_TYPE_VIDEO_H264);
    AV_WB16(p + 2, words_minus_1);
    return total_bytes;
}

/* Reverse the bits of a 32-bit integer — H.265 SPS stores
 * profile_compatibility_flag[j] at bit (31-j) in big-endian read; the MIB
 * convention puts flag[j] at bit j. Mirrors
 * streams/ipmx_add_sender_reports_pcap.py:_reverse_bits_32. */
static uint32_t ipmx_reverse_bits32(uint32_t v)
{
    uint32_t r = 0;
    int i;
    for (i = 0; i < 32; i++) r = (r << 1) | ((v >> i) & 1);
    return r;
}

/* Parse the HEVC profile_tier_level block from the cached SPS RBSP into the
 * IpmxStreamRT.hevc_* fields. Same byte-walk as the inline parse that was in
 * ipmx_build_h265_mib (now reused both by the MIB builder and the SDP fmtp
 * augmentation in ipmx_sdp_emit_video_compressed_block, which needs the PTL
 * values to emit the RFC 7798 / BCP-006-03 NMOS fmtp parameters
 *   profile-space=, profile-id=, tier-flag=, level-id=,
 *   profile-compatibility-indicator=, interop-constraints=
 *
 * Steps (per H.265 §7.3.2.1.1 / §7.3.3 + RFC 7798 §7.2):
 *   1. skip 2-byte NAL header
 *   2. strip emulation-prevention bytes (00 00 03 → 00 00)
 *   3. rbsp[1]            = (profile_space << 6) | (tier_flag << 5) | profile_idc
 *      rbsp[2:6]          = profile_compatibility_flag[32] (MSB-first)
 *      rbsp[6:12]         = general_constraint_indicator_flags (48 bits, big-endian)
 *      rbsp[12]           = general_level_idc
 *
 * On success populates st->hevc_* and sets st->hevc_ptl_valid = 1.
 * On absent/malformed SPS leaves the fields zero and hevc_ptl_valid = 0. */
static void ipmx_extract_h265_ptl(IpmxStreamRT *st)
{
    uint8_t rbsp[64];
    int rbsp_len = 0;
    int i;

    st->hevc_ptl_valid = 0;
    if (!st->sps_data || st->sps_len < 15)
        return;

    for (i = 2; i < st->sps_len && rbsp_len < (int)sizeof(rbsp); ) {
        if (i + 2 < st->sps_len &&
            st->sps_data[i] == 0 && st->sps_data[i+1] == 0 && st->sps_data[i+2] == 3) {
            if (rbsp_len + 2 <= (int)sizeof(rbsp)) {
                rbsp[rbsp_len++] = 0;
                rbsp[rbsp_len++] = 0;
            }
            i += 3;
        } else {
            rbsp[rbsp_len++] = st->sps_data[i++];
        }
    }
    if (rbsp_len < 13)
        return;

    {
        uint8_t  ptl    = rbsp[1];
        uint32_t compat = AV_RB32(rbsp + 2);
        st->hevc_profile_space  = (ptl >> 6) & 0x03;
        st->hevc_tier_flag      = (ptl >> 5) & 0x01;
        st->hevc_profile_idc    = ptl & 0x1f;
        st->hevc_profile_compat = ipmx_reverse_bits32(compat);
        memcpy(st->hevc_interop_constraints, rbsp + 6, 6);
        st->hevc_level_idc      = rbsp[12];
        st->hevc_ptl_valid      = 1;
    }
}

/* Base64-encode one cached H.265 parameter-set NAL unit into out (sized
 * IPMX_H265_SPROP_MAX_CHARS) — the same characters stock sdp.c emits for
 * sprop-vps/sps/pps. Returns the length; 0 when the NAL unit is absent or its
 * base64 form exceeds the 1-byte MIB length field. */
static int ipmx_h265_sprop_b64(char *out, const uint8_t *nal, int nal_len)
{
    int n;
    if (!nal || nal_len <= 0)
        return 0;
    n = ipmx_b64_encode(out, IPMX_H265_SPROP_MAX_CHARS, nal, nal_len);
    return n < 0 ? 0 : n;
}

/* Warn at write_header when a parameter set is too long for MIB 0x0009: the
 * SDP still lists it, but TR-10-15b §16 caps each sprop at 255 characters, so
 * ipmx_build_h265_mib leaves it out instead of truncating the base64. */
static void ipmx_warn_h265_sprop_overflow(AVFormatContext *s,
                                          const IpmxStreamRT *st, int stream_index)
{
    static const char *const names[3] = { "sprop-vps", "sprop-sps", "sprop-pps" };
    const int nal_len[3] = { st->vps_len, st->sps_len, st->pps_len };
    int i;
    for (i = 0; i < 3; i++) {
        int chars = ((nal_len[i] + 2) / 3) * 4;
        if (chars > IPMX_H265_SPROP_MAX_CHARS)
            av_log(s, AV_LOG_WARNING,
                   "IPMX stream %d: %s is %d base64 characters, more than the %d "
                   "MIB 0x0009 can carry (TR-10-15b §16); omitted from the MIB\n",
                   stream_index, names[i], chars, IPMX_H265_SPROP_MAX_CHARS);
    }
}

/* H.265 MIB type 0x0009 (TR-10-15b §16) — byte layout mirrors
 * streams/ipmx_sender_report.py H265MediaInfoBlock.to_bytes(). The PTL
 * extraction from the SPS RBSP follows
 * streams/ipmx_add_sender_reports_pcap.py:_parse_h265_sps_ptl (EPB-stripped
 * RBSP, MIB convention with reversed compatibility-flag bit order). */
static int ipmx_build_h265_mib(uint8_t *p, int max, IpmxStreamRT *st)
{
    int payload_off = 4;
    uint32_t mask = 0;
    uint8_t profile_space = 0, profile_id = 0, level_id = 0, tier_flag = 0;
    uint32_t profile_compat = 0;
    uint8_t  interop_constraints[6] = { 0 };
    /* sprop_{vps,sps,pps} in TR-10-15b §16 syntax: the base64 characters as
     * represented in the SDP fmtp line. */
    char sprop[3][IPMX_H265_SPROP_MAX_CHARS];
    int  sprop_len[3];
    int total_payload, total_bytes, words_minus_1;
    int i;

    /* Read PTL fields from the IpmxStreamRT cache populated by
     * ipmx_extract_h265_ptl at write_header. The same fields are reused
     * by the SDP fmtp augmentation in ipmx_sdp_emit_video_compressed_block,
     * so the SPS RBSP parse happens once. */
    if (st->hevc_ptl_valid) {
        profile_space  = st->hevc_profile_space;
        tier_flag      = st->hevc_tier_flag;
        profile_id     = st->hevc_profile_idc;
        profile_compat = st->hevc_profile_compat;
        memcpy(interop_constraints, st->hevc_interop_constraints, 6);
        level_id       = st->hevc_level_idc;
    }

    sprop_len[0] = ipmx_h265_sprop_b64(sprop[0], st->vps_data, st->vps_len);
    sprop_len[1] = ipmx_h265_sprop_b64(sprop[1], st->sps_data, st->sps_len);
    sprop_len[2] = ipmx_h265_sprop_b64(sprop[2], st->pps_data, st->pps_len);

    /* Field-presence bitmask (FIELD_BITS in streams/ipmx_sender_report.py:254): */
    mask |= 1u << 0;   /* profile_space */
    mask |= 1u << 1;   /* profile_id */
    mask |= 1u << 2;   /* level_id */
    mask |= 1u << 3;   /* tier_flag */
    mask |= 1u << 4;   /* profile_compatibility_indicator */
    mask |= 1u << 5;   /* interop_constraints */
    mask |= 1u << 7;   /* tx_mode */
    if (sprop_len[0] > 0) mask |= 1u << 13;   /* sprop_vps */
    if (sprop_len[1] > 0) mask |= 1u << 14;   /* sprop_sps */
    if (sprop_len[2] > 0) mask |= 1u << 15;   /* sprop_pps */

    if (max < payload_off + 4 + 1+1+1+1 + 4 + 6 + 2 + 4 + 4 + 2 + 2 + 1+1 + 2 + 4)
        return AVERROR(ENOSPC);
    AV_WB32(p + payload_off, mask);                       payload_off += 4;
    p[payload_off++] = profile_space;
    p[payload_off++] = profile_id;
    p[payload_off++] = level_id;
    p[payload_off++] = tier_flag;
    AV_WB32(p + payload_off, profile_compat);             payload_off += 4;
    memcpy(p + payload_off, interop_constraints, 6);      payload_off += 6;
    AV_WB16(p + payload_off, 0);                          payload_off += 2;   /* sprop_max_don_diff */
    memcpy(p + payload_off, IPMX_HEVC_TX_MODE, 4);        payload_off += 4;   /* tx_mode (4 ASCII chars, as in the fmtp line) */
    AV_WB32(p + payload_off, 0);                          payload_off += 4;   /* sprop_depack_buf_bytes */
    AV_WB16(p + payload_off, 0);                          payload_off += 2;   /* sprop_depack_buf_nalus */
    AV_WB16(p + payload_off, 0);                          payload_off += 2;   /* sprop_spatial_segmentation_idc */
    p[payload_off++] = 0;                                                   /* sprop_sub_layer_id */
    p[payload_off++] = 0;                                                   /* sprop_segmentation_id */
    AV_WB16(p + payload_off, 0);                          payload_off += 2;   /* reserved */

    /* sprop_{vps,sps,pps}: 1-byte lengths (then extra_bytes length), then
     * the base64 characters. */
    if (max < payload_off + 4 + sprop_len[0] + sprop_len[1] + sprop_len[2])
        return AVERROR(ENOSPC);
    for (i = 0; i < 3; i++)
        p[payload_off++] = (uint8_t)sprop_len[i];
    p[payload_off++] = 0;                                                   /* extra_bytes length */
    for (i = 0; i < 3; i++) {
        memcpy(p + payload_off, sprop[i], sprop_len[i]);
        payload_off += sprop_len[i];
    }

    /* word-boundary pad to 32-bit alignment. */
    while ((payload_off - 4) & 3) {
        if (payload_off >= max) return AVERROR(ENOSPC);
        p[payload_off++] = 0;
    }

    total_payload = payload_off - 4;
    total_bytes   = 4 + total_payload;
    words_minus_1 = (total_bytes / 4) - 1;
    AV_WB16(p,     IPMX_MIB_TYPE_VIDEO_H265);
    AV_WB16(p + 2, words_minus_1);
    return total_bytes;
}

/* Video MIB (type 0x0001 / 0x0003 / 0x0005) — layout in
 * streams/ipmx_sender_report.py _decode_video_media_info; 92 bytes total. */
static int ipmx_build_video_mib(uint8_t *p, int max,
                                int mib_type,
                                const char *sampling_str,
                                int width, int height,
                                int rate_num, int rate_den,
                                int bit_depth, int packing_mode, int interlace,
                                const char *range_str,
                                const char *colorimetry_str,
                                const char *tcs_str,
                                uint64_t meas_pixclk,
                                int htotal, int vtotal)
{
    uint32_t packed;
    uint32_t rate_field;
    int len_words_minus1 = 22;

    if (max < 92) return AVERROR(ENOSPC);

    AV_WB16(p, mib_type);
    AV_WB16(p + 2, len_words_minus1);

    memset(p + 4, 0, 16);
    if (sampling_str) av_strlcpy((char *)(p + 4), sampling_str, 16);

    packed = ((uint32_t)(bit_depth & 0x7f) << 24)
           | ((uint32_t)(packing_mode ? 1 : 0) << 23)
           | ((uint32_t)(interlace ? 1 : 0) << 22)
           | ((uint32_t)1 << 8)
           | ((uint32_t)1);
    AV_WB32(p + 20, packed);

    memset(p + 24, 0, 12);
    if (range_str)       av_strlcpy((char *)(p + 24), range_str, 12);
    memset(p + 36, 0, 20);
    if (colorimetry_str) av_strlcpy((char *)(p + 36), colorimetry_str, 20);
    memset(p + 56, 0, 16);
    if (tcs_str)         av_strlcpy((char *)(p + 56), tcs_str, 16);

    AV_WB16(p + 72, width);
    AV_WB16(p + 74, height);
    rate_field = ((uint32_t)(rate_num & IPMX_MIB_RATE_NUM_MAX) << 10)
               |  (uint32_t)(rate_den & IPMX_MIB_RATE_DEN_MAX);
    AV_WB32(p + 76, rate_field);
    AV_WB32(p + 80, (uint32_t)(meas_pixclk >> 32));
    AV_WB32(p + 84, (uint32_t)(meas_pixclk & 0xffffffffu));
    AV_WB16(p + 88, htotal);
    AV_WB16(p + 90, vtotal);

    return 92;
}

/* Audio MIB (type 0x0002 / 0x0004) — layout in
 * streams/ipmx_sender_report.py _decode_audio_media_info. */
static int ipmx_build_audio_mib(uint8_t *p, int max,
                                int mib_type,
                                int sampling_rate, int sample_size_bits,
                                int channel_count, int packet_time_us,
                                int measured_sample_rate,
                                const char *channel_order_str)
{
    int co_len = (int)strlen(channel_order_str);
    int padded = (co_len + 3) & ~3;
    int channel_words = padded / 4;
    int payload_len = 16 + padded;
    int total_bytes = 4 + payload_len;
    int len_words_minus1 = (total_bytes / 4) - 1;

    if (max < total_bytes) return AVERROR(ENOSPC);

    AV_WB16(p,      mib_type);
    AV_WB16(p + 2,  len_words_minus1);
    AV_WB32(p + 4,  sampling_rate);
    p[8]  = (uint8_t)(sample_size_bits & 0xff);
    p[9]  = (uint8_t)(channel_count    & 0xff);
    AV_WB16(p + 10, packet_time_us);
    AV_WB32(p + 12, measured_sample_rate);
    AV_WB32(p + 16, channel_words);
    memset(p + 20, 0, padded);
    memcpy(p + 20, channel_order_str, co_len);

    return total_bytes;
}

/* Build a complete RTCP SR datagram for stream st (video or audio).
 * Returns datagram length in bytes, or < 0 on error. */
static int ipmx_build_sr(IpmxStreamRT *st,
                         uint8_t *out, int max,
                         uint32_t rtp_ts_to_carry,
                         int64_t  internal_clock_ns_at_sample,
                         uint32_t pkt_count, uint32_t octet_count)
{
    IpmxMuxContext *ctx = st->mc;
    int p = 0;
    int info_off, total_words, mib_len;
    int ntp_msw = (int)(internal_clock_ns_at_sample / 1000000000LL);
    int ntp_lsw = (int)(internal_clock_ns_at_sample % 1000000000LL);

    if (max < 28) return AVERROR(ENOSPC);

    /* RTCP header (8 bytes) + sender info (20 bytes), 28 total. */
    out[p++] = 0x80;                /* V=2, P=0, RC=0 (TR-10-1 §8.7) */
    out[p++] = RTCP_PT_SR;
    out[p++] = 0; out[p++] = 0;     /* length: filled later */
    AV_WB32(out + p, st->ssrc);            p += 4;
    AV_WB32(out + p, (uint32_t)ntp_msw);   p += 4;
    AV_WB32(out + p, (uint32_t)ntp_lsw);   p += 4;
    AV_WB32(out + p, rtp_ts_to_carry);     p += 4;
    AV_WB32(out + p, pkt_count);           p += 4;
    AV_WB32(out + p, octet_count);         p += 4;

    /* IPMX Info Block (TR-10-1 §8.7). */
    info_off = p;
    AV_WB16(out + p, IPMX_RTCP_TAG); p += 2;
    AV_WB16(out + p, 0);             p += 2;
    out[p++] = IPMX_INFO_BLOCK_VERSION;
    out[p++] = 0; out[p++] = 0; out[p++] = 0;

    /* ts-refclk string (64 bytes, zero-padded). TR-10-1 §10.4. */
    {
        char tsref[64] = { 0 };
        if (ctx->refclk && ctx->refclk[0])
            av_strlcpy(tsref, ctx->refclk, sizeof(tsref));
        else
            snprintf(tsref, sizeof(tsref), "localmac=%s",
                     ctx->sender_mac ? ctx->sender_mac : "00-20-FC-00-00-00");
        if (max - p < 64) return AVERROR(ENOSPC);
        memcpy(out + p, tsref, 64);
        p += 64;
    }

    /* mediaclk string (12 bytes, zero-padded). TR-10-1 §10.5. */
    {
        char mc[12] = { 0 };
        av_strlcpy(mc, ctx->mediaclk ? ctx->mediaclk : "sender", sizeof(mc));
        if (max - p < 12) return AVERROR(ENOSPC);
        memcpy(out + p, mc, 12);
        p += 12;
    }

    /* Media Info Block — type depends on stream kind. */
    if (st->is_audio) {
        const char *channel_order_str = st->audio_channel_order[0]
                                          ? st->audio_channel_order
                                          : "SMPTE2110.(ST)";
        int audio_mib_type = st->audio_is_am824 ? IPMX_MIB_TYPE_AUDIO_AM824
                                                 : IPMX_MIB_TYPE_AUDIO_PCM;
        /* MIB packet_time must be the ACTUAL packet duration (periods ×
         * 1e6 / sample_rate), not the user-supplied option value. They
         * differ at sample rates where the option-value µs doesn't divide
         * cleanly into samples (e.g. 44.1 kHz: option 1090 µs → 48 samples
         * → actual 1088.435 µs ≈ 1088). The validator's MIB check matches
         * against the exact rounded value. */
        int mib_periods = (st->audio_payload_bytes > 0 && st->audio_channels > 0)
                          ? st->audio_payload_bytes / (st->audio_channels *
                              (st->audio_is_am824 ? 4 : (st->audio_sample_size / 8)))
                          : 0;
        int mib_packet_time_us = (mib_periods > 0 && st->audio_sample_rate > 0)
                                 ? (int)((int64_t)mib_periods * 1000000 /
                                         st->audio_sample_rate)
                                 : st->audio_ptime_us;
        mib_len = ipmx_build_audio_mib(out + p, max - p,
                                       audio_mib_type,
                                       st->audio_sample_rate,
                                       st->audio_sample_size,
                                       st->audio_channels,
                                       mib_packet_time_us,
                                       /* measured_sample_rate: TR-10-1 §10.3
                                        * makes this conditional on baseband
                                        * input. We're non-baseband, but the
                                        * streams/ reference and the conformance
                                        * validator (TR-10-1-MIB-SIG) still
                                        * expect a non-zero value — and there
                                        * is no asynchronous baseband sampling
                                        * to "measure" anyway. Mirror the
                                        * streams/ convention (ipmx_pcm.py:342:
                                        * media.measured_sample_rate =
                                        * config.sample_rate) and emit the
                                        * nominal sample_rate. The same nominal
                                        * value is emitted in the SDP fmtp
                                        * `measuredsamplerate=` field below so
                                        * the IPMX↔SDP cross-check passes. */
                                       st->audio_sample_rate,
                                       channel_order_str);
    } else {
        /* Video MIB. Type depends on whether the stream is raw (RFC 4175
         * → 0x0001, TR-10-2 §10) or compressed (H.264/H.265 → 0x0005,
         * TR-10-7 §12, with content shared with 0x0001 per TR-10-7). */
        const char *sampling_str = "YCbCr-4:2:0";
        int mib_type = st->is_compressed_video ? IPMX_MIB_TYPE_VIDEO_COMP
                                               : IPMX_MIB_TYPE_VIDEO_RAW;
        int bit_depth = st->bit_depth > 0 ? st->bit_depth : 8;
        switch (st->pix_fmt) {
        case AV_PIX_FMT_UYVY422:
        case AV_PIX_FMT_YUV422P:
        case AV_PIX_FMT_YUV422P10:  sampling_str = "YCbCr-4:2:2"; break;
        case AV_PIX_FMT_YUV444P:    sampling_str = "YCbCr-4:4:4"; break;
        case AV_PIX_FMT_YUV420P:
        case AV_PIX_FMT_YUV420P10:  sampling_str = "YCbCr-4:2:0"; break;
        case AV_PIX_FMT_RGB24:      sampling_str = "RGB";         break;
        default:                    sampling_str = "YCbCr-4:2:0"; break;
        }
        {
            /* MIB baseband signal parameters. For non-baseband sources
             * (file/generator/network input), TR-10-1 §10.2 makes htotal /
             * vtotal / measuredpixclk conditional, but the conformance
             * validator (TR-10-1-MIB-SIG) and the streams/-side reference
             * (generate_video_test_streams.py:621 — htotal defaults to
             * cfg.width) require htotal >= width, vtotal >= height, and
             * measured_pixclk = htotal·vtotal·fps. We therefore emit
             * synthesised values that satisfy the equality at minimum
             * geometry: htotal = width, vtotal = height, measured_pixclk
             * = width·height·fps. */
            int rn = st->rate_num > 0 ? st->rate_num : 30;
            int rd = st->rate_den > 0 ? st->rate_den : 1;
            int htot = st->width;
            int vtot = st->height;
            uint64_t pixclk = (uint64_t)htot * (uint64_t)vtot *
                              (uint64_t)rn / (uint64_t)rd;
            mib_len = ipmx_build_video_mib(out + p, max - p,
                                           mib_type,
                                           sampling_str,
                                           st->width, st->height,
                                           rn, rd,
                                           bit_depth, 0, 0,
                                           "NARROW", "BT709", "SDR",
                                           pixclk, htot, vtot);
        }
    }
    if (mib_len < 0) return mib_len;
    p += mib_len;

    /* Compressed video — TR-10-15c §16 (H.264 → 0x000A) and TR-10-15b §16
     * (H.265 → 0x0009) require an additional codec-specific MIB *immediately*
     * following the 0x0005 video MIB. */
    if (!st->is_audio && st->is_compressed_video) {
        int extra;
        if (st->compressed_codec_id == AV_CODEC_ID_H264)
            extra = ipmx_build_h264_mib(out + p, max - p, st);
        else if (st->compressed_codec_id == AV_CODEC_ID_HEVC)
            extra = ipmx_build_h265_mib(out + p, max - p, st);
        else
            extra = 0;
        if (extra < 0) return extra;
        p += extra;
    }

    /* TR-10-13 PEP MIB (type 0x0011) — emitted on every stream when PEP
     * encryption is enabled. Layout per streams/ipmx_sender_report.py:
     * privacy_version (8b) | rsvd (4b) | f_id (4b) | rsvd (4b) | s_id (4b) | rsvd (8b). */
    if (ctx->privacy_encrypt) {
        if (max - p < 8) return AVERROR(ENOSPC);
        AV_WB16(out + p,     IPMX_MIB_TYPE_PEP);
        AV_WB16(out + p + 2, 1);  /* length-words-minus-1 (8-byte total) */
        /* privacy_version=0, f_id (Full-IV ext-map id) and s_id (Short-IV
         * ext-map id) — we emit Full-IV with extmap id = ctx->pep_extmap_id_full
         * (RFC 8285 one-byte ext header). The validator checks consistency
         * with the SDP a=extmap. */
        AV_WB32(out + p + 4,
                ((uint32_t)0 << 24) |          /* privacy_version */
                ((uint32_t)(ctx->pep_extmap_id_full & 0x0F) << 16) |  /* f_id */
                ((uint32_t)(ctx->pep_extmap_id_short & 0x0F) << 8));  /* s_id */
        p += 8;
    }

    /* IPMX Info Block length (32-bit words minus 1). */
    {
        int info_block_bytes = p - info_off;
        int info_words = (info_block_bytes + 3) / 4;
        AV_WB16(out + info_off + 2, info_words - 1);
    }

    /* RTCP length (32-bit words minus 1). */
    total_words = (p + 3) / 4;
    AV_WB16(out + 2, total_words - 1);

    /* Pad to 32-bit boundary if needed. */
    while (p & 3) {
        if (p >= max) return AVERROR(ENOSPC);
        out[p++] = 0;
    }

    /* Compound RTCP packet (RFC 3550 §6.1 / TR-10-1 §8.7): append an SDES
     * packet carrying a CNAME item after the SR.  Stock ffmpeg (rtpenc.c)
     * emits the SDES only when a cname is configured; IPMX *mandates* the
     * compound packet, so we synthesise a default CNAME (ipmx@<src-ip>) when
     * none is set. The SR length word above already covers only the SR, so
     * the SDES stands as its own RTCP packet in the same datagram. */
    {
        const char *cname = (ctx->cname && ctx->cname[0]) ? ctx->cname : NULL;
        char cname_buf[128];
        int clen, sdes_off;

        if (!cname) {
            snprintf(cname_buf, sizeof(cname_buf), "ipmx@%s",
                     (ctx->src_ip && ctx->src_ip[0]) ? ctx->src_ip : "127.0.0.1");
            cname = cname_buf;
        }
        clen = (int)strlen(cname);
        if (clen > 255)
            clen = 255;
        /* header(4) + SSRC(4) + item type(1) + item len(1) + text + null + pad(≤3) */
        if (max - p < 10 + clen + 1 + 3)
            return AVERROR(ENOSPC);

        sdes_off = p;
        out[p++] = (2 << 6) | 0x01;      /* V=2, P=0, SC=1 */
        out[p++] = RTCP_PT_SDES;         /* 202 */
        out[p++] = 0; out[p++] = 0;      /* length: filled later */
        AV_WB32(out + p, st->ssrc);      p += 4;
        out[p++] = SDES_ITEM_CNAME;      /* item type = 1 (CNAME) */
        out[p++] = (uint8_t)clen;        /* item length (text only) */
        memcpy(out + p, cname, clen);    p += clen;
        out[p++] = 0;                    /* null item ends the chunk */
        while ((p - sdes_off) & 3)       /* pad chunk to 32-bit boundary */
            out[p++] = 0;
        AV_WB16(out + sdes_off + 2, ((p - sdes_off) / 4) - 1);
    }

    return p;
}

/* ─────────────── DLT_EN10MB PCAP writer ─────────────── */

#define PCAP_MAGIC_LE   0xa1b2c3d4
#define PCAP_DLT_EN10MB 1

static void ipmx_pcap_write_header(FILE *fp)
{
    uint8_t hdr[24];
    AV_WL32(hdr,      PCAP_MAGIC_LE);
    AV_WL16(hdr + 4,  2);
    AV_WL16(hdr + 6,  4);
    AV_WL32(hdr + 8,  0);
    AV_WL32(hdr + 12, 0);
    AV_WL32(hdr + 16, 65535);
    AV_WL32(hdr + 20, PCAP_DLT_EN10MB);
    fwrite(hdr, 1, 24, fp);
}

static void ipmx_mac_for_ipv4(uint32_t ipv4_be, uint8_t mac[6])
{
    uint8_t a = (ipv4_be >> 24) & 0xff;
    uint8_t b = (ipv4_be >> 16) & 0xff;
    uint8_t c = (ipv4_be >>  8) & 0xff;
    uint8_t d =  ipv4_be        & 0xff;
    if ((a & 0xf0) == 0xe0) {
        mac[0] = 0x01; mac[1] = 0x00; mac[2] = 0x5e;
        mac[3] = b & 0x7f; mac[4] = c; mac[5] = d;
    } else {
        mac[0] = 0x02; mac[1] = 0x00; mac[2] = 0xfc;
        mac[3] = b;    mac[4] = c;    mac[5] = d;
    }
}

static uint32_t ipmx_inet_pton4(const char *str)
{
    int o[4] = { 127, 0, 0, 1 };
    if (str && str[0])
        sscanf(str, "%d.%d.%d.%d", &o[0], &o[1], &o[2], &o[3]);
    return ((uint32_t)(o[0] & 0xff) << 24) |
           ((uint32_t)(o[1] & 0xff) << 16) |
           ((uint32_t)(o[2] & 0xff) << 8)  |
            (uint32_t)(o[3] & 0xff);
}

static void ipmx_pcap_emit_udp(IpmxMuxContext *ctx,
                               const uint8_t *payload, int payload_len,
                               uint32_t src_ipv4, uint16_t src_port,
                               uint32_t dst_ipv4, uint16_t dst_port,
                               int dscp_tos)
{
    uint8_t hdr[16], eth[14], ip[20], udp[8];
    uint8_t src_mac[6] = { 0x02, 0x00, 0xfc, 0xde, 0xad, 0xbe };
    uint8_t dst_mac[6];
    int     ip_total_len = 20 + 8 + payload_len;
    int     pkt_len = 14 + ip_total_len;
    int64_t now_ns = ipmx_clock_now_ns(ctx);
    uint32_t ts_sec  = (uint32_t)(now_ns / 1000000000LL);
    uint32_t ts_usec = (uint32_t)((now_ns / 1000) % 1000000);
    uint16_t ip_csum;
    uint32_t s = 0;
    int i;

    if (!ctx->pcap_fp) return;

    AV_WL32(hdr,      ts_sec);
    AV_WL32(hdr + 4,  ts_usec);
    AV_WL32(hdr + 8,  pkt_len);
    AV_WL32(hdr + 12, pkt_len);

    ipmx_mac_for_ipv4(dst_ipv4, dst_mac);
    memcpy(eth,     dst_mac, 6);
    memcpy(eth + 6, src_mac, 6);
    AV_WB16(eth + 12, 0x0800);

    ip[0] = 0x45;
    ip[1] = (uint8_t)(dscp_tos & 0xff);
    AV_WB16(ip + 2,  ip_total_len);
    AV_WB16(ip + 4,  ctx->pcap_ip_id_counter++);
    AV_WB16(ip + 6,  0x4000);
    ip[8]  = 64;
    ip[9]  = 17;
    AV_WB16(ip + 10, 0);
    AV_WB32(ip + 12, src_ipv4);
    AV_WB32(ip + 16, dst_ipv4);
    for (i = 0; i < 20; i += 2)
        s += (ip[i] << 8) | ip[i + 1];
    while (s >> 16) s = (s & 0xffff) + (s >> 16);
    ip_csum = (uint16_t)~s;
    AV_WB16(ip + 10, ip_csum);

    AV_WB16(udp,     src_port);
    AV_WB16(udp + 2, dst_port);
    AV_WB16(udp + 4, 8 + payload_len);
    AV_WB16(udp + 6, 0);

    fwrite(hdr, 1, 16, ctx->pcap_fp);
    fwrite(eth, 1, 14, ctx->pcap_fp);
    fwrite(ip,  1, 20, ctx->pcap_fp);
    fwrite(udp, 1,  8, ctx->pcap_fp);
    if (payload_len > 0) fwrite(payload, 1, payload_len, ctx->pcap_fp);
}

/* Sleep until an absolute CLOCK_MONOTONIC deadline expressed in microseconds.
 *
 * av_gettime_relative() returns CLOCK_MONOTONIC microseconds on Linux, so we
 * can pass av_gettime_relative()-derived values directly. clock_nanosleep
 * with TIMER_ABSTIME is preferred over av_usleep / nanosleep (relative) for
 * the pacer's outer preempt sleep:
 *   - Absolute deadline → no "now + delta" math each call → fewer drift
 *     opportunities under preemption between gettime and the sleep syscall.
 *   - Linux HRTIMER backend on TIMER_ABSTIME has tighter wake-up tail than
 *     the relative variant on some kernels (incl. WSL2 where av_usleep
 *     showed rare 1 ms+ oversleeps in pacer trace data).
 *   - Returns EINTR on signal; we retry, otherwise treat any non-zero
 *     return as a no-op (avoid busy-looping inside the pacer thread). */
static inline void ipmx_clock_nanosleep_until_us(int64_t target_us)
{
    struct timespec ts;
    ts.tv_sec  =  target_us / 1000000;
    ts.tv_nsec = (target_us % 1000000) * 1000;
    while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL) == EINTR)
        ;
}

/* ─────────────── shared pacer (single thread, cross-stream min-heap) ─────────────── */

static void ipmx_pace_wait(int64_t target_us, int spin_us)
{
    int64_t now = av_gettime_relative();
    int64_t delta = target_us - now;
    if (delta <= 0) return;
    if (delta > spin_us) av_usleep((unsigned)(delta - spin_us));
    while (av_gettime_relative() < target_us) {
#if defined(__x86_64__) || defined(__i386__)
        __asm__ volatile("pause" ::: "memory");
#endif
    }
}

/* Producer side: enqueue an event into the stream's MPSC ring.
 *
 * MPSC reservation: atomic_fetch_add(&head, 1) gives this producer a
 * unique slot index. We then wait (sched_yield) until that slot is free
 * (slot.ready==0) — i.e., the pacer has finished consuming an earlier
 * occupant. After populating the event payload, the slot.ready release-store
 * publishes it for the pacer to read. No mutex, no condvar.
 *
 * Backpressure: if a producer is much faster than the pacer can drain
 * (pathological case), the wait loop spins+yields. Under expected load
 * the slot is already free at fetch_add time. */
static int ipmx_ring_push_into(IpmxStreamRT *st, IpmxStreamRing *ring,
                               IpmxEventKind kind, int dscp_tos,
                               int64_t target_us,
                               const uint8_t *data, int len)
{
    IpmxRingSlot *slot;
    unsigned h;
    int spin = 0;
    int64_t wait_start_us = 0;
    uint64_t wait_iters = 0;
    int waited = 0;

    if (len < 0 || len > IPMX_PACER_EVT_MAX) return AVERROR(EINVAL);

    h = atomic_fetch_add_explicit(&ring->head, 1, memory_order_acq_rel);
    slot = &ring->slots[h & (ring->cap - 1)];

    /* Wait for slot to be free (consumer-released). */
    while (atomic_load_explicit(&slot->ready, memory_order_acquire) != 0) {
        if (atomic_load_explicit(&st->mc->pacer.stop, memory_order_relaxed))
            return AVERROR_EOF;
        if (!waited) { wait_start_us = av_gettime_relative(); waited = 1; }
        wait_iters++;
        if (++spin > 64) { sched_yield(); spin = 0; }
#if defined(__x86_64__) || defined(__i386__)
        else __asm__ volatile("pause" ::: "memory");
#endif
    }
    if (waited) {
        int64_t waited_us = av_gettime_relative() - wait_start_us;
        if (ring == &st->sr_ring) {
            st->sr_ring_push_blocked_calls++;
            st->sr_ring_push_blocked_us += waited_us;
        } else {
            st->ring_push_blocked_calls++;
            st->ring_push_spin_iters    += wait_iters;
            st->ring_push_blocked_us    += waited_us;
            if (waited_us > st->ring_push_blocked_max_us)
                st->ring_push_blocked_max_us = waited_us;
        }
    }

    /* Populate event in-place inside the slot — zero-copy beyond the payload. */
    slot->evt.target_us  = target_us;
    slot->evt.stream_idx = st->src_stream_index;
    slot->evt.kind       = kind;
    slot->evt.len        = len;
    slot->evt.dscp_tos   = dscp_tos;
    if (len > 0) memcpy(slot->evt.data, data, len);

    /* Release-store publishes the slot to the pacer. */
    atomic_store_explicit(&slot->ready, 1, memory_order_release);
    return 0;
}

/* Convenience wrapper: push to the stream's RTP ring (the default ring). */
static int ipmx_ring_push(IpmxStreamRT *st,
                          IpmxEventKind kind, int dscp_tos,
                          int64_t target_us,
                          const uint8_t *data, int len)
{
    return ipmx_ring_push_into(st, &st->ring, kind, dscp_tos,
                               target_us, data, len);
}

/* In-memory trace ring helper (single-writer = pacer thread). */
static void ipmx_trace_log(IpmxPacer *p, int64_t now_us,
                           int64_t target_us, int64_t raw_target_us,
                           int stream_idx, int kind)
{
    unsigned idx;
    IpmxTraceEntry *e;
    if (!p->trace || p->trace_cap == 0) return;
    idx = atomic_fetch_add_explicit(&p->trace_head, 1, memory_order_relaxed);
    e = &p->trace[idx & p->trace_mask];
    e->t_us          = now_us;
    e->target_us     = target_us;
    e->raw_target_us = raw_target_us;
    e->send_done_us  = 0;
    e->pcap_done_us  = 0;
    e->stream_idx    = stream_idx;
    e->kind          = kind;
    e->late_us       = (int)(now_us - target_us);
}

/* Fill in the post-send timestamps for the most-recently-logged trace entry.
 * Called immediately after ipmx_pacer_dispatch_slot returns so we can
 * separately measure (a) pre-dispatch latency = t_us - target_us, (b) sendto
 * latency = send_done_us - t_us, (c) pcap-write latency = pcap_done_us -
 * send_done_us. Visible in the trace dump as extra columns. */
static void ipmx_trace_fill_send(IpmxPacer *p,
                                 int64_t send_done_us, int64_t pcap_done_us)
{
    unsigned head_idx;
    IpmxTraceEntry *e;
    if (!p->trace || p->trace_cap == 0) return;
    head_idx = atomic_load_explicit(&p->trace_head, memory_order_relaxed);
    if (head_idx == 0) return;
    e = &p->trace[(head_idx - 1) & p->trace_mask];
    e->send_done_us = send_done_us;
    e->pcap_done_us = pcap_done_us;
}

/* Pacer-pick descriptor — both the "best" (= strict global min effective
 * target_us) and the "best-alt" (= min effective target_us among rings of
 * streams *other* than best->st) returned by ipmx_pacer_find_two_earliest.
 *
 * "Effective" target = slot.target_us + st->pacer_offset_us. The offset is
 * 0 until the pacer's startup gate releases (see IpmxPacer.gate_open). */
typedef struct IpmxPacerPick {
    IpmxStreamRT   *st;        /* NULL if no candidate */
    IpmxStreamRing *ring;
    int64_t         target_us;     /* effective target (INT64_MAX if no candidate) */
    int64_t         raw_target_us; /* producer-side target (for trace) */
} IpmxPacerPick;

/* Multiplexing step.
 *
 * Returns the strict global minimum (best) and the minimum across rings of
 * streams *other* than best->st (best_alt). The pacer uses best_alt to
 * implement the late-burst fairness rule — see ipmx_pacer_thread.
 *
 * Why no heap: with N streams (≤ ~16), min-of-N is N atomic loads + N
 * comparisons — ~150 ns total at N=9. A per-event heap push/pop is ~50 ns
 * but is paid PER PACKET (~18 K/s here = ~900 µs/s), AND requires a drain
 * step that happens in big bursts (e.g., 336 video packets at frame
 * boundary), pulling the pacer off its tight pacing loop. The MPEG-TS
 * pacer uses a single SPSC FIFO with no heap and achieves sub-2 ms
 * variance for the same reason. */
static void ipmx_pacer_find_two_earliest(IpmxMuxContext *ctx,
                                         IpmxPacerPick *best,
                                         IpmxPacerPick *best_alt)
{
    int i;

    best->st = NULL;     best->ring = NULL;
    best->target_us     = INT64_MAX;
    best->raw_target_us = INT64_MAX;
    best_alt->st = NULL; best_alt->ring = NULL;
    best_alt->target_us     = INT64_MAX;
    best_alt->raw_target_us = INT64_MAX;

    for (i = 0; i < ctx->nb_streams; i++) {
        IpmxStreamRT *st = &ctx->streams[i];
        /* Each stream contributes up to two rings: RTP (data) and SR (RTCP).
         * Both are checked so the pacer dispatches in target order across
         * RTP and SR queues, not in producer-FIFO order. */
        IpmxStreamRing *rings[2] = { &st->ring, st->sr_ring.cap ? &st->sr_ring : NULL };
        int r;
        for (r = 0; r < 2; r++) {
            IpmxStreamRing *ring = rings[r];
            IpmxRingSlot *slot;
            int64_t raw_t, t;
            if (!ring || ring->cap == 0) continue;
            slot = &ring->slots[ring->tail & (ring->cap - 1)];
            if (atomic_load_explicit(&slot->ready, memory_order_acquire) != 1)
                continue;
            raw_t = slot->evt.target_us;
            t     = raw_t + st->pacer_offset_us;
            if (t < best->target_us) {
                /* New global min — demote previous best to best_alt only if
                 * its stream is different from the new best's stream. */
                if (best->st && best->st != st) {
                    best_alt->st            = best->st;
                    best_alt->ring          = best->ring;
                    best_alt->target_us     = best->target_us;
                    best_alt->raw_target_us = best->raw_target_us;
                }
                best->st            = st;
                best->ring          = ring;
                best->target_us     = t;
                best->raw_target_us = raw_t;
            } else if (st != best->st && t < best_alt->target_us) {
                /* Same stream as current best is excluded from best_alt. */
                best_alt->st            = st;
                best_alt->ring          = ring;
                best_alt->target_us     = t;
                best_alt->raw_target_us = raw_t;
            }
        }
    }
}

/* Pacer-thread dispatch — sends the event from the slot, writes PCAP if
 * enabled, then releases the ring slot for the producer. */
static void ipmx_pacer_dispatch_slot(IpmxMuxContext *ctx, IpmxStreamRT *st,
                                     IpmxStreamRing *ring, IpmxRingSlot *slot,
                                     int64_t *out_send_done_us,
                                     int64_t *out_pcap_done_us)
{
    IpmxEvent *e = &slot->evt;
    URLContext *socket_h;
    uint16_t    dst_port;

    if (e->kind == IPMX_EVT_RTCP_SR) {
        socket_h = st->udp_rtcp_h;
        dst_port = (uint16_t)st->dst_rtcp_port_resolved;
    } else {
        socket_h = st->udp_h;
        dst_port = (uint16_t)st->dst_port_resolved;
    }

    if (socket_h)
        (void)ffurl_write(socket_h, e->data, e->len);
    if (out_send_done_us) *out_send_done_us = av_gettime_relative();

    if (ctx->pcap_fp) {
        uint32_t src = ipmx_inet_pton4(ctx->src_ip ? ctx->src_ip : "127.0.0.1");
        uint32_t dst = ipmx_inet_pton4(st->dst_ip_resolved);
        ipmx_pcap_emit_udp(ctx, e->data, e->len,
                           src, (uint16_t)(ctx->src_port_base + st->src_stream_index),
                           dst, dst_port, e->dscp_tos);
    }
    if (out_pcap_done_us) *out_pcap_done_us = av_gettime_relative();

    /* Release the slot for the producer. */
    atomic_store_explicit(&slot->ready, 0, memory_order_release);
    ring->tail++;
}

/* Startup gate — held until every registered stream has its first packet
 * on its ring (epoch_set==1) OR ipmx_pacer_gate_us has elapsed. On release,
 * T_anchor is captured and each primed stream's pacer_offset_us is set to
 *   pacer_offset_us = T_anchor - epoch_us
 * so all streams' producer-side targets are translated into a common
 * wall-clock reference. Late-priming streams (those that miss the gate)
 * keep pacer_offset_us=0; their producer-side targets are already real
 * wall-clock instants by definition (epoch_us = av_gettime_relative() at
 * first write_packet), so 0 offset is correct.
 *
 * Returns 1 if the gate was just released this tick, 0 if still gated. */
static int ipmx_pacer_check_gate(IpmxMuxContext *ctx)
{
    IpmxPacer *p = &ctx->pacer;
    int gate_us  = ctx->pacer_gate_us;
    int all_primed = 1;
    int i, n_primed = 0;
    int64_t now_us;

    if (p->gate_open) return 0;

    if (gate_us <= 0) {
        /* Gate disabled — open immediately, leaving every stream's offset
         * at 0 (original per-stream-epoch behavior). */
        p->gate_open = 1;
        return 1;
    }

    for (i = 0; i < ctx->nb_streams; i++) {
        if (atomic_load_explicit(&ctx->streams[i].epoch_set, memory_order_acquire))
            n_primed++;
        else
            all_primed = 0;
    }
    now_us = av_gettime_relative();
    if (!all_primed && (now_us - p->gate_start_us) <= (int64_t)gate_us)
        return 0;

    {
        int64_t t_anchor = now_us;
        for (i = 0; i < ctx->nb_streams; i++) {
            IpmxStreamRT *st = &ctx->streams[i];
            if (atomic_load_explicit(&st->epoch_set, memory_order_acquire))
                st->pacer_offset_us = t_anchor - st->epoch_us;
            else
                st->pacer_offset_us = 0;
        }
        p->gate_open = 1;
        av_log(ctx, AV_LOG_INFO,
               "IPMX pacer gate released after %"PRId64" µs (%d/%d streams primed%s)\n",
               now_us - p->gate_start_us, n_primed, ctx->nb_streams,
               all_primed ? "" : ", timeout");
    }
    return 1;
}

static void *ipmx_pacer_thread(void *arg)
{
    IpmxMuxContext *ctx = arg;
    IpmxPacer *p = &ctx->pacer;
    int idle_poll_us  = ctx->idle_poll_us  > 0 ? ctx->idle_poll_us  : IPMX_PACER_IDLE_POLL_US;
    int imminent_us   = ctx->pacer_imminent_us;

    p->gate_start_us = av_gettime_relative();
    p->gate_open     = 0;

    /* Loop until BOTH stop is signalled AND all rings are empty — so
     * end-of-stream packets are still dispatched on their proper schedule
     * (otherwise drain-without-pacing compresses the last frame into a
     * burst, showing up as a final-window TR-10-9 §11.2 variance spike). */
    for (;;) {
        IpmxPacerPick best, best_alt;
        IpmxRingSlot *slot;
        int64_t now_us;

        /* Startup gate: hold dispatch until every stream is primed or the
         * gate timeout elapses. While gated, sleep idle_poll_us between
         * checks so we don't burn CPU. */
        if (!p->gate_open) {
            if (atomic_load_explicit(&p->stop, memory_order_acquire))
                break;
            (void)ipmx_pacer_check_gate(ctx);
            if (!p->gate_open) {
                av_usleep((unsigned)idle_poll_us);
                continue;
            }
        }

        ipmx_pacer_find_two_earliest(ctx, &best, &best_alt);

        if (!best.st) {
            if (atomic_load_explicit(&p->stop, memory_order_acquire))
                break;  /* stop signalled and rings drained — done */
            /* Otherwise sleep briefly, watching for new arrivals or stop. */
            av_usleep((unsigned)idle_poll_us);
            continue;
        }

        slot = &best.ring->slots[best.ring->tail & (best.ring->cap - 1)];

        /* ── Staleness drop gate ──────────────────────────────────────────
         * Defensive backstop: if the slot we're about to dispatch is older
         * than (now - drop_late_us), drop it instead. Prevents fault-mode
         * snowballs (kernel preempt, sendto stall, malformed input) from
         * accumulating hundreds of ms of backlog and saturating the wire
         * with packets past their presentation deadline.
         *
         * Sampled BEFORE pace_wait so we don't waste a spin on a slot
         * we're going to drop. drop_late_us=0 disables the gate (every
         * packet is dispatched regardless of lateness). */
        if (ctx->drop_late_us > 0) {
            int64_t now_pre = av_gettime_relative();
            if (best.target_us < now_pre - (int64_t)ctx->drop_late_us) {
                IpmxEventKind drop_kind =
                    (slot->evt.kind == IPMX_EVT_RTCP_SR)
                    ? IPMX_EVT_DROP_SR : IPMX_EVT_DROP_RTP;
                if (slot->evt.kind == IPMX_EVT_RTCP_SR)
                    best.st->dropped_late_sr++;
                else
                    best.st->dropped_late_rtp++;
                ipmx_trace_log(p, now_pre, best.target_us, best.raw_target_us,
                               best.st->src_stream_index, drop_kind);
                /* Free the slot for the producer; advance tail. */
                atomic_store_explicit(&slot->ready, 0, memory_order_release);
                best.ring->tail++;
                continue;
            }
        }

        /* Pace and dispatch. If the chosen best is far in the future
         * (e.g. an AU first-packet whose t_af-extended target sits seconds
         * out for a VBR IDR drain), sleep only pacer_spin_us µs and
         * re-enter the dispatch loop. That lets later-arriving but
         * earlier-targeted events — most importantly RTCP Sender Reports
         * pushed at j·T encoder-pou r cadence — preempt the wait instead of
         * being serialised behind the long drain. Only commit to the exact
         * pace_wait spin once target is within the preempt window.
         * TR-10-15c §15 SR-before-AU is met by SR's target = T_pour(j) − ε
         * being unconditionally earlier than the corresponding AU's wire
         * arrival; this re-check window is what makes that ordering hold on
         * the wire even when an AU's drain extends past several T's. */
        {
            int64_t now_pre = av_gettime_relative();
            int64_t delta   = best.target_us - now_pre;
            if (delta > ctx->pacer_spin_us) {
                /* Trace the preempt-sleep event BEFORE sleeping. send_done_us
                 * and pcap_done_us are back-filled AFTER the sleep returns so
                 * the trace dump shows the actual wallclock the sleep landed,
                 * making any kernel oversleep visible in post-mortem analysis.
                 *
                 * Sleep uses clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME)
                 * targeting an absolute deadline = now_pre + spin_us. On WSL2
                 * this is empirically tighter than av_usleep (which uses a
                 * relative timespec via nanosleep) — av_usleep showed rare
                 * ~1 ms+ oversleeps in pacer trace data while the absolute
                 * variant binds the wake target to a specific CLOCK_MONOTONIC
                 * instant, avoiding the now+delta recomputation drift. */
                int64_t pre_sleep_us  = now_pre;
                int64_t deadline_us   = now_pre + ctx->pacer_spin_us;
                ipmx_trace_log(p, pre_sleep_us, best.target_us,
                               best.raw_target_us, best.st->src_stream_index,
                               IPMX_EVT_PREEMPT);
                ipmx_clock_nanosleep_until_us(deadline_us);
                {
                    int64_t post_sleep_us = av_gettime_relative();
                    /* Re-use send_done_us / pcap_done_us columns for
                     * (sleep-end, idle). Their difference vs t_us measures
                     * actual sleep duration including kernel oversleep. */
                    ipmx_trace_fill_send(p, post_sleep_us, post_sleep_us);
                }
                continue;
            }
        }
        ipmx_pace_wait(best.target_us, ctx->pacer_spin_us);
        now_us = av_gettime_relative();
        ipmx_trace_log(p, now_us, best.target_us, best.raw_target_us,
                       best.st->src_stream_index, slot->evt.kind);

        {
            int64_t send_done_us = 0, pcap_done_us = 0;
            ipmx_pacer_dispatch_slot(ctx, best.st, best.ring, slot,
                                     &send_done_us, &pcap_done_us);
            ipmx_trace_fill_send(p, send_done_us, pcap_done_us);
        }

        /* ── Late-burst fairness ──────────────────────────────────────────
         * If the just-dispatched packet was late (target_us < now) AND
         * another stream has a packet whose target is "imminent" — i.e.
         * within [now, now + imminent_us] of right-now — service that
         * packet immediately too, before any *next* late packet from the
         * first stream can monopolise the dispatcher. This keeps streams
         * with on-time pacing from drifting out of cadence while a peer
         * stream's late burst is being drained.
         *
         * Conditions are conservative: imminent_us defaults to ~20 µs
         * (typical sendto + PCAP-write cost). When all streams are late,
         * best_alt.target_us is also < now → the comparison fails and
         * the pacer correctly drains in target order one packet per loop. */
        if (imminent_us > 0 && best.target_us < now_us && best_alt.st) {
            int64_t alt_lead = best_alt.target_us - now_us;
            if (alt_lead >= 0 && alt_lead <= imminent_us) {
                IpmxRingSlot *alt_slot = &best_alt.ring->slots[
                    best_alt.ring->tail & (best_alt.ring->cap - 1)];
                /* Re-check the slot is still ready (defensive — producer
                 * cannot reclaim a slot in flight). */
                if (atomic_load_explicit(&alt_slot->ready, memory_order_acquire) == 1) {
                    int64_t now2 = av_gettime_relative();
                    int64_t alt_send_done = 0, alt_pcap_done = 0;
                    ipmx_trace_log(p, now2,
                                   best_alt.target_us, best_alt.raw_target_us,
                                   best_alt.st->src_stream_index,
                                   alt_slot->evt.kind);
                    ipmx_pacer_dispatch_slot(ctx, best_alt.st, best_alt.ring, alt_slot,
                                             &alt_send_done, &alt_pcap_done);
                    ipmx_trace_fill_send(p, alt_send_done, alt_pcap_done);
                }
            }
        }
    }

    return NULL;
}

/* Round up to next power of two (≥ 1). */
static int ipmx_round_up_pow2(int n)
{
    int p = 1;
    while (p < n) p <<= 1;
    return p;
}

static int ipmx_pacer_init(IpmxMuxContext *ctx)
{
    IpmxPacer *p = &ctx->pacer;
    int rc;

    atomic_init(&p->stop, 0);
    atomic_init(&p->trace_head, 0);

    /* Optional in-memory trace ring (see ipmx_pacer_trace_cap AVOption). */
    if (ctx->pacer_trace_cap > 0) {
        int cap = ipmx_round_up_pow2(ctx->pacer_trace_cap);
        p->trace = av_calloc(cap, sizeof(IpmxTraceEntry));
        if (!p->trace) return AVERROR(ENOMEM);
        p->trace_cap  = cap;
        p->trace_mask = cap - 1;
    }

    rc = pthread_create(&p->thread, NULL, ipmx_pacer_thread, ctx);
    if (rc != 0) {
        av_freep(&p->trace);
        return AVERROR(rc);
    }
    p->started = 1;
    return 0;
}

static void ipmx_pacer_dump_trace(IpmxMuxContext *ctx, AVFormatContext *s)
{
    IpmxPacer *p = &ctx->pacer;
    FILE *fp = NULL;
    unsigned head, count, start, i;

    if (!p->trace || p->trace_cap == 0) return;
    head = atomic_load_explicit(&p->trace_head, memory_order_relaxed);
    count = head < (unsigned)p->trace_cap ? head : (unsigned)p->trace_cap;
    start = head < (unsigned)p->trace_cap ? 0 : (head - p->trace_cap);

    if (ctx->pacer_trace_file && ctx->pacer_trace_file[0]) {
        fp = fopen(ctx->pacer_trace_file, "w");
        if (!fp) {
            av_log(s, AV_LOG_WARNING, "Cannot open pacer trace file '%s'\n",
                   ctx->pacer_trace_file);
            return;
        }
    } else {
        fp = stderr;
    }
    fprintf(fp, "# IPMX pacer trace — %u events (capacity %d)\n", count, p->trace_cap);
    fprintf(fp, "# t_us\ttarget_us\traw_target_us\tsend_done_us\tpcap_done_us\tlate_us\tstream\tkind\n");
    for (i = 0; i < count; i++) {
        IpmxTraceEntry *e = &p->trace[(start + i) & p->trace_mask];
        const char *kind_str;
        switch (e->kind) {
        case IPMX_EVT_RTP:      kind_str = "RTP";      break;
        case IPMX_EVT_RTCP_SR:  kind_str = "SR";       break;
        case IPMX_EVT_DROP_RTP: kind_str = "DROP_RTP"; break;
        case IPMX_EVT_DROP_SR:  kind_str = "DROP_SR";  break;
        case IPMX_EVT_PREEMPT:  kind_str = "PREEMPT";  break;
        default:                kind_str = "?";        break;
        }
        fprintf(fp, "%" PRId64 "\t%" PRId64 "\t%" PRId64 "\t%" PRId64 "\t%" PRId64 "\t%d\t%d\t%s\n",
                e->t_us, e->target_us, e->raw_target_us,
                e->send_done_us, e->pcap_done_us,
                e->late_us, e->stream_idx, kind_str);
    }
    if (fp != stderr) fclose(fp);
}

/* Dump every compressed-video stream's HRD pacing trace (oldest record first). */
static void ipmx_hrd_dump_trace(IpmxMuxContext *ctx, AVFormatContext *s)
{
    FILE *fp;
    int i;

    if (ctx->hrd_trace_cap <= 0) return;
    if (ctx->hrd_trace_file && ctx->hrd_trace_file[0]) {
        fp = fopen(ctx->hrd_trace_file, "w");
        if (!fp) {
            av_log(s, AV_LOG_WARNING, "Cannot open HRD trace file '%s'\n",
                   ctx->hrd_trace_file);
            return;
        }
    } else {
        fp = stderr;
    }
    fprintf(fp, "# IPMX HRD pacing trace (times in µs relative to the stream epoch)\n");
    fprintf(fp, "# stream\tau\trtp_payload_bytes\tnal_unit_bytes\tpace_us\tt_af_advance_us"
                "\tt_r_n_us\tframe_epoch_next_us\tpace_next_us\tclamp_us\n");
    for (i = 0; i < ctx->nb_streams; i++) {
        const IpmxStreamRT *st = &ctx->streams[i];
        unsigned cap = (unsigned)ctx->hrd_trace_cap, count, start, k;
        if (!st->hrd_trace) continue;
        count = FFMIN(st->hrd_trace_count, cap);
        start = st->hrd_trace_count - count;
        for (k = 0; k < count; k++) {
            const IpmxHrdTraceEntry *e = &st->hrd_trace[(start + k) % cap];
            fprintf(fp, "%d\t%d\t%" PRId64 "\t%" PRId64 "\t%" PRId64 "\t%" PRId64
                        "\t%" PRId64 "\t%" PRId64 "\t%" PRId64 "\t%" PRId64 "\n",
                    i, e->au_idx, e->rtp_payload_bytes, e->nal_unit_bytes,
                    e->pace_us, e->t_af_advance_us, e->t_r_n_us,
                    e->frame_epoch_next_us, e->pace_next_us, e->clamp_us);
        }
    }
    if (fp != stderr) fclose(fp);
}

static void ipmx_pacer_close(IpmxMuxContext *ctx)
{
    IpmxPacer *p = &ctx->pacer;
    if (!p->started) return;
    atomic_store_explicit(&p->stop, 1, memory_order_release);
    pthread_join(p->thread, NULL);
    p->started = 0;
}

/* Per-stream ring init/teardown. Must be called per-stream from
 * ipmx_setup_stream / ipmx_write_trailer. */
static int ipmx_ring_init(IpmxStreamRing *ring, int cap)
{
    int i;
    /* cap must be a power of two for fast head/tail mask. */
    if (cap <= 0 || (cap & (cap - 1)) != 0) return AVERROR(EINVAL);
    ring->slots = av_calloc(cap, sizeof(IpmxRingSlot));
    if (!ring->slots) return AVERROR(ENOMEM);
    ring->cap   = cap;
    atomic_init(&ring->head, 0);
    ring->tail  = 0;
    for (i = 0; i < cap; i++)
        atomic_init(&ring->slots[i].ready, 0);
    return 0;
}

static void ipmx_ring_close(IpmxStreamRing *ring)
{
    av_freep(&ring->slots);
    ring->cap = 0;
}

/* β-paced target time within a frame for raw video (RFC 4175). */
static int64_t ipmx_video_target_us_for_pkt(IpmxStreamRT *st, double beta)
{
    double t_drain_us;
    if (st->packets_per_frame <= 0) return st->frame_epoch_us;
    t_drain_us = (double)st->t_frame_us / (double)st->packets_per_frame / beta;
    return st->frame_epoch_us + (int64_t)((double)st->packet_in_frame * t_drain_us);
}

/* Cumulative-byte target for compressed video (H.264 RFC 6184 / H.265 RFC 7798).
 *
 * Per TR-10-7 §10 + ST 2110-21 §7.1: pace at MaxRate × β where MaxRate is the
 * encoder's maximum target bitrate including IP/UDP/RTP overhead. Each packet's
 * target_us = stream_epoch_us + (cumulative_bytes_to_packet_end × 8 × 1e6) /
 *             (max_bitrate_bps × β).
 *
 * This produces a smooth bitrate over time that stays within MaxRate × β,
 * absorbs AU-size variation (I-frame vs P-frame), and gives the receiver
 * predictable arrival timing per the TR-10-1 §8.1 Network Compatibility Model.
 *
 * Caller passes the cumulative bytes including the current packet (i.e. the
 * byte offset of the packet's LAST byte from stream start). */
static int64_t ipmx_comp_target_us_for_pkt(IpmxStreamRT *st, double beta,
                                           int64_t au_cum_bytes_through_pkt)
{
    /* TR-10-15c §15 / TR-10-15b §15 AU pacing — H.264 §C.1.1 / H.265 §C.2.1.1:
     *
     *   CBR (cbr_flag = 1, eq C-2):
     *     t_ai(j) = t_af(j-1) = t_ai(j-1) + AU_size(j-1)·8 / max_bitrate
     *
     *   VBR (cbr_flag = 0, eq C-3):
     *     t_ai(j) = max(t_af(j-1), tai_earliest(j))
     *     tai_earliest(j) = t_r,n(j) - init_delay/90000        if first AU of new BP (C-5/C-7)
     *                     = t_r,n(j) - (init_delay + init_offset)/90000  otherwise (C-4/C-6)
     *
     * `comp_pace_epoch_us` is the wallclock at which AU j's first byte hits
     * the wire — i.e. exactly t_ai(j) translated to our muxer's relative
     * time. It is set by the SEI walker (ipmx_hrd_extract_pic_timing) when
     * hrd_present == 1, using the encoder's HRD trace from pic_timing SEI
     * + buffering_period SEI. For the HRD-not-present fallback, comp_pace_epoch_us
     * mirrors frame_epoch_us (the legacy j·T cadence).
     *
     * Within an AU, packets are paced at max_bitrate × β per ST 2110-21 §7.1.
     * β creates the small idle gap between actual_t_af(j) and t_ai(j+1) that
     * legitimately falls in §15's allowed AU_offset range. */
    double rate_bps = (double)st->comp_max_bitrate_bps * beta;
    if (rate_bps <= 0) return st->comp_pace_epoch_us;
    return st->comp_pace_epoch_us +
           (int64_t)((double)au_cum_bytes_through_pkt * 8.0 * 1000000.0 / rate_bps);
}

/* Audio packet target — uniform ptime cadence (TR-10-1 §8.10.1). */
static int64_t ipmx_audio_target_us_for_pkt(IpmxStreamRT *st)
{
    /* Use periods_per_pkt × 1e6 / sample_rate, not ptime_us directly, so
     * the pacer's wall-clock targets stay aligned with the RTP packetiser's
     * actual packet rate at non-integer-period ptimes (e.g. 44.1 kHz). */
    return st->audio_packet_epoch_us +
           ((int64_t)st->audio_pkt_idx * st->audio_periods_per_pkt *
            1000000LL) / st->audio_sample_rate;
}

/* ─────────────── core per-stream AVIO write callback ─────────────── */

/* Each call delivers one full RTP datagram from the child rtp muxer
 * (avio_flush is issued per ff_rtp_send_data; AVIO direct=0 so the buffer
 * accumulates the full packet between flushes). We pace, send, capture. */
static int ipmx_avio_write(void *opaque, const uint8_t *buf, int buf_size)
{
    IpmxStreamRT *st = opaque;
    IpmxMuxContext *ctx;
    int64_t target_us;
    int     marker;
    int     ret;

    if (!st || buf_size <= 0) return buf_size;
    ctx = st->mc;

    /* Accumulate into per-stream packet buffer (defensive — should be 1 call/packet). */
    if (st->pkt_buf_size + buf_size > st->pkt_buf_capacity) {
        int new_cap = st->pkt_buf_size + buf_size + 1024;
        uint8_t *nb = av_realloc(st->pkt_buf, new_cap);
        if (!nb) return AVERROR(ENOMEM);
        st->pkt_buf = nb;
        st->pkt_buf_capacity = new_cap;
    }
    memcpy(st->pkt_buf + st->pkt_buf_size, buf, buf_size);
    st->pkt_buf_size += buf_size;

    if (st->pkt_buf_size < 12) return buf_size;  /* RTP header is at least 12 bytes */

    /* Initialise per-stream wall-clock pacing epoch on first packet.
     *
     * epoch_us / frame_epoch_us / audio_packet_epoch_us must be written
     * BEFORE the release-store on epoch_set so the pacer's gate-check
     * thread sees consistent values (it reads epoch_set with acquire and
     * then reads epoch_us). */
    if (!atomic_load_explicit(&st->epoch_set, memory_order_relaxed)) {
        uint32_t first_rtp_ts = AV_RB32(st->pkt_buf + 4);
        st->epoch_us = av_gettime_relative();
        st->frame_epoch_us = st->epoch_us;
        /* AU_offset_0 selection: shift AU 0's wire transmission by this
         * amount from epoch_us per TR-10-15c §15's symmetric-CPB model:
         *   AU_offset_0 = cpb_size/max_bitrate − init_delay/90000.
         *
         * ZERO  : AU_offset_0 = 0 (no shift; AU 0 fires immediately)
         * MAX   : AU_offset_0 = cpb_size/max_bitrate (known at init from SPS)
         * SYMM. : AU_offset_0 = cpb_size/max_bitrate − init_delay/90000
         *           init_delay comes from AU 0's BP SEI (parsed from the clear
         *           access unit in ipmx_write_packet); the AU 0 buffering block
         *           in the compressed-video processing path below applies it,
         *           or falls back to 0 when AU 0 carries no BP SEI. */
        st->hrd_au_offset0_us = 0;
        if (st->hrd_present && st->hrd_max_bitrate_bps > 0 && st->hrd_cpb_size_bits > 0) {
            int64_t cpb_drain_us = (int64_t)((double)st->hrd_cpb_size_bits * 1e6 /
                                             (double)st->hrd_max_bitrate_bps);
            switch (ctx->au_offset0_mode) {
            case IPMX_AU_OFFSET0_MAX:
                st->hrd_au_offset0_us = cpb_drain_us;
                st->hrd_au_offset0_applied = 1;
                break;
            case IPMX_AU_OFFSET0_SYMMETRIC:
            case IPMX_AU_OFFSET0_OFFSET:
                /* Don't apply yet — SYMMETRIC needs init_delay and OFFSET
                 * needs init_offset, both from AU 0's BP SEI. AU 0 packets
                 * will be buffered until BP SEI is parsed, then flushed
                 * with the correct AU_offset_0. */
                st->hrd_au_offset0_applied = 0;
                break;
            case IPMX_AU_OFFSET0_ZERO:
            default:
                st->hrd_au_offset0_us = 0;
                st->hrd_au_offset0_applied = 1;
                break;
            }
        } else {
            /* No HRD on this stream — no offset, no buffering. */
            st->hrd_au_offset0_applied = 1;
        }
        st->comp_pace_epoch_us = st->epoch_us + st->hrd_au_offset0_us;
        st->audio_packet_epoch_us = st->epoch_us;
        st->ssrc        = AV_RB32(st->pkt_buf + 8);
        st->last_rtp_ts = first_rtp_ts;
        st->sr_due            = 1;
        /* TR-10-1 §8.6: rtp_ts_base and internal_clock_at_epoch_ns were
         * paired together at write_header time (see ipmx_setup_stream),
         * so the §8.7 invariant NTP·Fs ≡ RTP-TS (mod 2^32) already holds
         * for the first SR provided the on-wire first RTP-TS equals the
         * configured base_timestamp. For H.264/H.265 with -c:v copy and
         * AV_NOPTS_VALUE packets, the rtp packetiser uses base_timestamp
         * unchanged on the first packet, so we don't re-anchor here. */
        st->pending_sr_rtp_ts = first_rtp_ts;
        /* Release-store publishes the epoch tuple to the pacer. */
        atomic_store_explicit(&st->epoch_set, 1, memory_order_release);
    }

    if (!st->is_audio) {
        /* ── Video path ─────────────────────────────────────────────────
         * SR cadence is anchored at the stream epoch and runs at the
         * nominal frame period T (TR-10-15c §15 schedule rule 1). To keep
         * the cadence regular even when AU drains span more than T (large
         * IDRs vs small P/B), SR_{n+1} is enqueued at AU_n's marker
         * boundary (end of this function) — well before AU_{n+1}'s first
         * packet enters the muxer. The sr_due block below fires for SR_0
         * (set in the epoch_set initialisation above) AND for SR_1 in the
         * compressed-video case (deferred from the first marker, where no
         * inter-AU rtp_ts step has been observed yet). */

        /* First packet of a video frame (raw or compressed): capture its
         * rtp_ts so SR_N's rtp_ts matches the wire's AU rtp_ts. Both RFC
         * 4175 and RFC 6184/7798 use a single rtp_ts per access unit, so
         * the first packet's rtp_ts is the AU's rtp_ts. Container metadata
         * (avg_frame_rate) can lie about cadence — using the wire value
         * directly avoids the SR↔AU rtp_ts mismatch that breaks
         * TR-10-1-SR-MAP / TR-10-15c-135b. */
        if (st->packet_in_frame == 0 && st->pkt_buf_size >= 12) {
            uint32_t this_au_rtp_ts = AV_RB32(st->pkt_buf + 4);
            if (st->frame_idx > 0)
                st->observed_au_ts_step = this_au_rtp_ts - st->prev_au_first_rtp_ts;
            st->prev_au_first_rtp_ts = this_au_rtp_ts;
            /* If a deferred SR is queued for emission (first marker has
             * fired OR initial sr_due set during epoch init), align its
             * rtp_ts to this packet's actual on-wire rtp_ts. */
            if (st->sr_due)
                st->pending_sr_rtp_ts = this_au_rtp_ts;
        }
        if (st->sr_due) {
            uint8_t srbuf[IPMX_RTCP_BUF_MAX];
            int64_t ntp_for_sr_ns;
            int64_t sr_target_us = st->frame_epoch_us - 50;
            int srlen;
            /* TR-10-1 §8.7: NTP and RTP-TS in an SR shall be sampled at the
             * same Internal-Clock instant. With our local Internal Clock
             * (TR-10-1 §7.1) and the §8.6 RTP-base anchor, the relationship
             * NTP·Fs = RTP-TS (mod 2^32) holds by construction. */
            int32_t rtp_delta = (int32_t)(st->pending_sr_rtp_ts - st->rtp_ts_base);
            ntp_for_sr_ns = st->internal_clock_at_epoch_ns +
                            (int64_t)rtp_delta * 1000000000LL / 90000LL;
            srlen = ipmx_build_sr(st, srbuf, sizeof(srbuf),
                                  st->pending_sr_rtp_ts, ntp_for_sr_ns,
                                  st->pkt_count, st->octet_count);
            if (srlen > 0)
                (void)ipmx_ring_push_into(st, &st->sr_ring,
                                          IPMX_EVT_RTCP_SR, IPMX_DSCP_VIDEO_TOS,
                                          sr_target_us, srbuf, srlen);
            st->sr_due = 0;
        }

        /* In-band parameter sets (cache refresh) and the HRD SEI are read
         * from the clear access unit in ipmx_write_packet, not from these RTP
         * packets: under PEP / HKEP their payload is already ciphertext. */
        if (st->is_compressed_video) {
            /* Compressed video — per-AU pacing per TR-10-15c §15: the AU's
             * first packet is anchored at i·T from epoch (frame_epoch_us)
             * and subsequent packets within the AU are spaced at
             * max_bitrate·β. comp_au_cum_bytes resets to 0 on each marker
             * boundary; whole-stream comp_cum_payload_bytes is kept for
             * diagnostics. */
            st->comp_au_cum_bytes      += st->pkt_buf_size;
            st->comp_cum_payload_bytes += st->pkt_buf_size;
            /* RTP payload bytes of the AU, for the HRD trace only. The HRD
             * drain time uses comp_au_nal_unit_bytes instead: this payload
             * still carries the RFC 6184/7798 payload-format headers (and,
             * when encrypted, the PEP MAC), which are not NAL unit bytes.
             * RTP header = 12 + 4·X-hdr-words. */
            {
                int rtp_hlen = 12;
                if ((st->pkt_buf[0] & 0x10) && st->pkt_buf_size >= 16) {
                    int xlen_words = AV_RB16(st->pkt_buf + 14);
                    rtp_hlen += 4 + xlen_words * 4;
                }
                if (st->pkt_buf_size > rtp_hlen)
                    st->comp_au_rtp_payload_bytes += (st->pkt_buf_size - rtp_hlen);
            }
            /* AU 0 buffering (SYMMETRIC mode only — see au_offset0_mode init at
             * first-packet block above): when AU_offset_0 has not yet been
             * computed (= !hrd_au_offset0_applied), the wire target_us for
             * this packet would use an uninitialised comp_pace_epoch_us. So we
             * buffer the packet locally and defer target_us computation until
             * hrd_init_delay_first_us is known from AU 0's BP SEI — parsed
             * from the clear access unit before its first packet, so the
             * buffer is flushed at that first packet. Then we compute
             * AU_offset_0 = cpb_size/max_bitrate − init_delay/90000, set
             * comp_pace_epoch_us = epoch_us + AU_offset_0, and flush the
             * buffer (push every buffered packet to st->ring with its correct
             * target_us). */
            if (st->hrd_present && !st->hrd_au_offset0_applied) {
                struct IpmxAuBufferedPkt *bp;
                int pkt_marker = (st->pkt_buf[1] & 0x80) ? 1 : 0;
                int have_bp;
                if (st->hrd_au0_buf_count >= st->hrd_au0_buf_cap) {
                    int new_cap = st->hrd_au0_buf_cap
                                ? st->hrd_au0_buf_cap * 2 : 16;
                    struct IpmxAuBufferedPkt *nb = av_realloc(
                        st->hrd_au0_buf,
                        new_cap * sizeof(struct IpmxAuBufferedPkt));
                    if (!nb) return AVERROR(ENOMEM);
                    st->hrd_au0_buf     = nb;
                    st->hrd_au0_buf_cap = new_cap;
                }
                bp = &st->hrd_au0_buf[st->hrd_au0_buf_count++];
                bp->data = av_malloc(st->pkt_buf_size);
                if (!bp->data) return AVERROR(ENOMEM);
                memcpy(bp->data, st->pkt_buf, st->pkt_buf_size);
                bp->len       = st->pkt_buf_size;
                bp->cum_bytes = st->comp_au_cum_bytes;
                bp->marker    = pkt_marker;
                bp->seq       = AV_RB16(st->pkt_buf + 2);
                bp->rtp_ts    = AV_RB32(st->pkt_buf + 4);

                /* AU 0's BP SEI (hrd_init_delay_first_us, parsed from the
                 * clear access unit in ipmx_write_packet) gives AU_offset_0;
                 * compute it and flush. If AU 0 ends without one, fall back
                 * to AU_offset_0 = 0 instead of holding the stream back. */
                have_bp = st->hrd_init_delay_first_us > 0 &&
                          st->hrd_max_bitrate_bps > 0 && st->hrd_cpb_size_bits > 0;
                if (have_bp || pkt_marker) {
                    int64_t cpb_drain_us = st->hrd_max_bitrate_bps > 0
                        ? (int64_t)((double)st->hrd_cpb_size_bits * 1e6 /
                                    (double)st->hrd_max_bitrate_bps)
                        : 0;
                    const char *mode_name;
                    if (!have_bp) {
                        av_log(ctx, AV_LOG_WARNING,
                               "IPMX: no buffering_period SEI in AU 0; "
                               "AU_offset_0 falls back to 0\n");
                        st->hrd_au_offset0_us = 0;
                        mode_name = "zero (no buffering_period SEI in AU 0)";
                    } else if (ctx->au_offset0_mode == IPMX_AU_OFFSET0_OFFSET) {
                        /* Use BP SEI initial_cpb_removal_delay_offset
                         * verbatim. Intended for encoders where the offset
                         * field is computed independently of cpb_size — e.g.
                         * low_delay_hrd_flag=1 sources that declare a CPB
                         * larger than what was used in the encoder pre-fill
                         * calculation. For x264/x265 CBR sources, this yields
                         * the same value as SYMMETRIC (per their definition
                         * offset = cpb_size·90000/bitrate − init_delay). */
                        st->hrd_au_offset0_us = st->hrd_init_offset_first_us;
                        mode_name = "offset";
                    } else {
                        /* SYMMETRIC: cpb_size/bitrate − init_delay/90000. */
                        st->hrd_au_offset0_us =
                            cpb_drain_us - st->hrd_init_delay_first_us;
                        mode_name = "symmetric";
                    }
                    if (st->hrd_au_offset0_us < 0)
                        st->hrd_au_offset0_us = 0;
                    st->comp_pace_epoch_us = st->epoch_us + st->hrd_au_offset0_us;
                    st->hrd_au_offset0_applied = 1;
                    av_log(ctx, AV_LOG_INFO,
                           "IPMX AU_offset_0 applied: mode=%s "
                           "offset=%lld µs (cpb_drain=%lld, init_delay=%lld, "
                           "init_offset=%lld)\n",
                           mode_name,
                           (long long)st->hrd_au_offset0_us,
                           (long long)cpb_drain_us,
                           (long long)st->hrd_init_delay_first_us,
                           (long long)st->hrd_init_offset_first_us);

                    /* Flush all buffered AU 0 packets: compute target_us,
                     * push to ring, update counters, free buffer entries.
                     * NOTE: do NOT run the marker handler here — it'll run
                     * normally when the marker packet's write_packet call
                     * comes through (which is later than the BP SEI packet
                     * in our test fixtures). */
                    {
                        int i;
                        for (i = 0; i < st->hrd_au0_buf_count; i++) {
                            struct IpmxAuBufferedPkt *e = &st->hrd_au0_buf[i];
                            int64_t t_us = st->comp_pace_epoch_us +
                                (int64_t)((double)e->cum_bytes * 8.0 * 1e6 /
                                          ((double)st->comp_max_bitrate_bps *
                                           ctx->pace_beta));
                            ret = ipmx_ring_push(st, IPMX_EVT_RTP,
                                                 IPMX_DSCP_VIDEO_TOS,
                                                 t_us, e->data, e->len);
                            if (ret < 0) {
                                av_log(NULL, AV_LOG_ERROR,
                                       "Ring push (AU 0 flush) failed: %d\n", ret);
                                return ret;
                            }
                            st->pkt_count++;
                            {
                                int rtp_hlen = 12;
                                if ((e->data[0] & 0x10) && e->len >= 16) {
                                    int xlen = AV_RB16(e->data + 14);
                                    rtp_hlen += 4 + xlen * 4;
                                }
                                if (e->len > rtp_hlen)
                                    st->octet_count +=
                                        (uint32_t)(e->len - rtp_hlen);
                            }
                            st->last_seq    = e->seq;
                            st->last_rtp_ts = e->rtp_ts;
                            st->packet_in_frame++;
                            av_freep(&e->data);
                        }
                        st->hrd_au0_buf_count = 0;
                    }
                }
                /* Whether we just flushed or are still buffering, do NOT
                 * fall through to the normal target/push/counter code for
                 * the current packet — its data was already added to the
                 * buffer (and pushed if we flushed). Skip to end of
                 * write_packet, where the marker handler may fire if the
                 * marker bit was set. The marker logic only runs for the
                 * last packet of AU 0 anyway, which by construction comes
                 * AFTER the BP SEI packet (= after the flush). */
                marker = pkt_marker;
                goto skip_video_push;
            }

            target_us = ipmx_comp_target_us_for_pkt(st, ctx->pace_beta,
                                                    st->comp_au_cum_bytes);
        } else {
            target_us = ipmx_video_target_us_for_pkt(st, ctx->pace_beta);
        }
        ret = ipmx_ring_push(st,
                             IPMX_EVT_RTP, IPMX_DSCP_VIDEO_TOS,
                             target_us, st->pkt_buf, st->pkt_buf_size);
        if (ret < 0) {
            av_log(NULL, AV_LOG_ERROR, "Ring push (video RTP) failed: %d\n", ret);
            return ret;
        }
        st->pkt_count++;
        /* RFC 3550 §6.4.1: octet_count is "the total number of payload
         * octets (i.e., not including header or padding) transmitted".
         * For RTP packets with the X bit set (PEP/HDCP RFC 8285 ext
         * header), subtract the ext-header bytes from the delta too. */
        {
            int rtp_hlen = 12;
            if ((st->pkt_buf[0] & 0x10) && st->pkt_buf_size >= 16) {
                int xlen_words = AV_RB16(st->pkt_buf + 14);
                rtp_hlen += 4 + xlen_words * 4;
            }
            if (st->pkt_buf_size > rtp_hlen)
                st->octet_count += (uint32_t)(st->pkt_buf_size - rtp_hlen);
        }

        marker = (st->pkt_buf[1] & 0x80) ? 1 : 0;
        st->last_seq    = AV_RB16(st->pkt_buf + 2);
        st->last_rtp_ts = AV_RB32(st->pkt_buf + 4);
        st->packet_in_frame++;
skip_video_push:

        if (marker) {
            /* TR-10-15c §15 schedule rule 1: SR cadence is the nominal
             * frame period T anchored at the stream epoch — independent of
             * AU drain time. SRs are pegged to wire rtp_ts (so the
             * validator's SR↔AU association by rtp_timestamp matches), so
             * we DEFER SR_{n+1} emission to the first packet of frame n+1
             * — see the compressed-video first-packet block above, which
             * sets pending_sr_rtp_ts to the on-wire AU rtp_ts and fires
             * the sr_due emission block.
             *
             * Setting sr_due=1 here flags the deferred emission. The
             * sr_target_us is still frame_epoch_{n+1} - 50 (TR-10-15c §15:
             * SR shall precede the AU's first packet); when AU_n drains
             * faster than T the SR fires at its nominal target, when AU_n
             * drains slower than T the SR target is in the past and the
             * pacer dispatches it ASAP — but always before the AU_{n+1}
             * first packet (whose target is frame_epoch_{n+1}). */
            st->saw_marker_pending = 0;
            if (st->packet_in_frame > 0)
                st->packets_per_frame = st->packet_in_frame;
            st->frame_idx++;
            st->packet_in_frame = 0;
            /* Per-marker HRD conformance check (diagnostic only — never aborts).
             * Per H.264 §C.1.1 / H.265 §C.2.1.1: AU j's last byte must arrive
             * at the decoder CPB by t_r,n(j). t_af(j) (β-free spec-side final
             * arrival) = comp_pace_epoch_us(j) − epoch_us + NAL_unit_bytes·8/max_bitrate.
             * Compare against the cached hrd_t_r_n_j_us computed from the AU's
             * pic_timing SEI. NAL unit bytes, not RTP payload bytes (H.264 eq
             * C-6 b(n) / H.265 eq C-8 sizeInbits). */
            if (st->hrd_present && st->hrd_seen_first_au && st->hrd_max_bitrate_bps > 0) {
                int64_t t_af_us_rel = (int64_t)((double)st->comp_au_nal_unit_bytes * 8.0 * 1e6 /
                                                (double)st->hrd_max_bitrate_bps);
                int64_t t_af_us = st->comp_pace_epoch_us - st->epoch_us + t_af_us_rel;
                st->hrd_t_af_prev_us = t_af_us;
                if (t_af_us > st->hrd_t_r_n_j_us + 500 /* 0.5 ms slack */) {
                    av_log(ctx, AV_LOG_WARNING,
                           "IPMX HRD violation at AU=%d: t_af=%lld µs > t_r,n=%lld µs "
                           "(delta=%lld µs). Encoder bug or our pacer too slow.\n",
                           st->hrd_au_idx, (long long)t_af_us,
                           (long long)st->hrd_t_r_n_j_us,
                           (long long)(t_af_us - st->hrd_t_r_n_j_us));
                }
            }
            /* §15 cross-check: BP SEI shall accompany every recovery_point. */
            if (st->hrd_present && st->hrd_saw_recovery_point_in_this_au && !st->hrd_saw_bp_in_this_au) {
                av_log(ctx, AV_LOG_WARNING,
                       "TR-10-15c §15 violation: recovery_point SEI without accompanying "
                       "buffering_period SEI at AU=%d\n", st->hrd_au_idx);
            }
            /* Cache previous-AU t_r,n for monotonicity / loop detection, and
             * the per-AU removal interval the re-anchor continues with. */
            if (st->hrd_present) {
                if (st->hrd_au_idx > 0 && st->hrd_t_r_n_j_ticks > st->hrd_t_r_n_prev_au_ticks)
                    st->hrd_t_r_n_step_ticks = st->hrd_t_r_n_j_ticks - st->hrd_t_r_n_prev_au_ticks;
                st->hrd_t_r_n_prev_au_ticks = st->hrd_t_r_n_j_ticks;
            }
            /* Clear per-AU scratch flags and advance AU index. */
            st->hrd_saw_bp_in_this_au = 0;
            st->hrd_saw_recovery_point_in_this_au = 0;
            st->hrd_saw_pt_in_this_au = 0;
            st->hrd_tai_earliest_wall_us = 0;
            st->hrd_au_idx++;
            /* TR-10-15c §15 schedule rule 1: SR cadence at the nominal frame
             * period T from stream_epoch. frame_epoch_us drives SR target
             * computation; this advance is independent of HRD per §15.
             * Computed from the frame index at the exact rate, not by adding
             * the µs-rounded t_frame_us, which ran the wire ahead of the RTP
             * clock (−10 ppm at 30 fps, −40 ppm at 60 fps). */
            st->frame_epoch_us = st->epoch_us +
                                 av_rescale(st->frame_idx, (int64_t)st->rate_den * 1000000,
                                            st->rate_num);
            /* Compressed-video AU pacing — sender-side three-way max,
             * combining the encoder pour timeline (TR-10-15c §15) with the
             * HRD model (H.264 §C.1.1 / H.265 §C.2.1.1):
             *
             *   wire(j) = max( T_pour(j) ,                   // encoder availability
             *                  t_af(j-1) ,                   // HRD eq C-2 / C-3
             *                  tai_earliest(j) )             // HRD eq C-3 lower bound
             *
             * Implemented across three sites:
             *
             *   1. AU_offset_0 shift applied at first write_packet:
             *      AU 0 fires at epoch_us + AU_offset_0 (symmetric CPB
             *      model). Anchors encoder pour timeline relative to the
             *      decoder's t_r,n(0).
             *
             *   2. Marker advance below: comp_pace_epoch_us +=
             *      max(t_frame_us, t_af_advance_us). Combines the encoder-
             *      availability arm (T_pour(j) − T_pour(j-1) = T) with the
             *      HRD eq (C-3) t_af lower bound (NAL bytes / max_bitrate).
             *      For tight CBR they coincide; for VBR small AUs T_pour
             *      dominates and protects SR-before-AU (TR-10-15c-131/134a)
             *      and CMAX (TR-10-1 §8.1).
             *
             *   3. SEI walker's tai_earliest override
             *      (ipmx_hrd_extract_pic_timing): if tai_earliest(j) >
             *      current comp_pace_epoch_us, overrides → adds eq (C-3)'s
             *      max-with-tai_earliest arm.
             *
             *   4. HRD ceiling (ipmx_hrd_apply_ceiling, ipmx_write_packet):
             *      caps the result at epoch + t_r,n(j) − drain(j) once AU j's
             *      SEI and NAL unit bytes are known — see below.
             *
             * SR cadence is locked to the encoder pour timeline at
             * T_pour(j) − 50 µs via frame_epoch_us (advanced by t_frame_us
             * above). Because wire(j) ≥ T_pour(j) by construction,
             * SR-before-AU holds unconditionally.
             *
             * HRD-TIME-EQC3 strict-equality would NOT hold when T_pour(j)
             * > max(t_af(j-1), tai_earliest(j)) — that gap is the
             * legitimate AU_offset_j ∈ [0, cpb_size/max_bitrate] window
             * TR-10-15c §15 permits, not a violation. The validator's
             * HRD-TIME-EQC3 check enforces eq (C-3) as a LOWER BOUND
             * (with 2 ms tolerance — aligned with TR-10-9 §11.2b's
             * SR-cadence jitter cap); the corresponding upper bound
             * (t_af(j) ≤ t_r,n(j)) is enforced separately by
             * HRD-TIME-01 / H264-HRD-TIME-01. Together they cover the
             * full permissive band without flagging the spec-allowed
             * wire-lateness as a failure. See
             * streams/ipmx_validate_hrd.py:filter_capture_boundary_orphan_srs
             * and the HRD-TIME-EQC3 implementation at line ~1189. */
            {
                /* Cumulative MAX of T_pour(j+1) and t_af(j); AU j+1 caps it
                 * at an absolute HRD ceiling derived from the encoder's
                 * declared t_r,n schedule.
                 *
                 * The floor MAX(frame_epoch, t_af_j) realises the spec's
                 * lower bound (encoder-pour timeline OR HRD eq C-2 chain).
                 * The tai_earliest arm of eq C-3 is added separately by
                 * the pic_timing SEI walker (ipmx_hrd_extract_pic_timing)
                 * when AU j+1 is written.
                 *
                 * The cumulative MAX alone admits a *reflected random
                 * walk* in G = pace − frame_epoch when the encoder has
                 * non-zero per-AU size variance. The walk is pinned at 0
                 * (small AUs let pace snap back to frame_epoch) but free
                 * to drift upward by σ√(2j/π) — bytes^2/AU variance
                 * accumulates without a sink. With NVENC's HEVC drain
                 * stddev ≈ 6 ms, the walk reaches the 1 s init_delay
                 * threshold in roughly init_delay²/(2σ²) ≈ 12k AUs ≈
                 * 7 minutes, after which the per-AU t_af > t_r,n
                 * diagnostic fires.
                 *
                 * To bound that walk, we cap pace at an HRD ceiling
                 * derived from the encoder's *absolute* t_r,n schedule
                 * (the PT SEI cpb_removal_delay sequence the encoder
                 * publishes — fixed per-AU, not accumulated):
                 *
                 *   hrd_ceiling(j+1) = epoch + t_r,n(j+1) − drain(j+1)
                 *
                 * Both terms are AU j+1's own: ipmx_hrd_apply_ceiling
                 * applies it in ipmx_write_packet, after AU j+1's pic_timing
                 * SEI and NAL unit bytes are parsed and before its first
                 * packet. (An estimate made here at AU j's marker — AU j's
                 * drain for drain(j+1), t_r,n(j) + one clock tick for
                 * t_r,n(j+1), which is half a frame for H.264 — clamped the
                 * small AU after a large I-frame by ~110 ms.) The clamp may
                 * start AU j+1 before AU j's last byte (strict t_ai =
                 * t_af_prev rule not met) but bounds the walk so t_af stays
                 * at or below t_r,n. It never goes below frame_epoch(j+1)
                 * (SR-before-AU) or tai_earliest(j+1).
                 *
                 * A CBR source whose long-term NAL rate exceeds BitRate
                 * also makes G grow: the t_af chain then sets the pace on
                 * every AU and G rises by (rate excess) × elapsed time
                 * until the ceiling triggers. The H.265 640x480p30 4M CBR
                 * fixture (+0.22 % over BitRate) does this — G measured
                 * from 100 ms to 152 ms over 10 s. G stays bounded by
                 * t_r,n − frame_epoch − drain either way.
                 */
                int64_t t_af_advance_us = (st->hrd_present && st->hrd_max_bitrate_bps > 0)
                    ? (int64_t)((double)st->comp_au_nal_unit_bytes * 8.0 * 1e6 /
                                (double)st->hrd_max_bitrate_bps)
                    : st->t_frame_us;
                int64_t t_af_j_us       = st->comp_pace_epoch_us + t_af_advance_us;
                int64_t pace_next_us    = FFMAX(st->frame_epoch_us, t_af_j_us);

                if (st->hrd_trace) {
                    IpmxHrdTraceEntry *e =
                        &st->hrd_trace[st->hrd_trace_count++ % (unsigned)ctx->hrd_trace_cap];
                    e->au_idx              = st->hrd_au_idx - 1;
                    e->rtp_payload_bytes   = st->comp_au_rtp_payload_bytes;
                    e->nal_unit_bytes      = st->comp_au_nal_unit_bytes;
                    e->pace_us             = st->comp_pace_epoch_us - st->epoch_us;
                    e->t_af_advance_us     = t_af_advance_us;
                    e->t_r_n_us            = st->hrd_t_r_n_j_us;
                    e->frame_epoch_next_us = st->frame_epoch_us - st->epoch_us;
                    e->pace_next_us        = pace_next_us - st->epoch_us;
                    e->clamp_us            = st->hrd_clamp_this_au_us;
                }

                st->comp_pace_epoch_us = pace_next_us;
            }
            /* Reset per-AU cumulative byte counters — the next AU's first
             * packet anchors at the new pacing epoch. */
            st->comp_au_cum_bytes = 0;
            st->comp_au_rtp_payload_bytes = 0;
            st->comp_au_nal_unit_bytes = 0;
            st->sr_due = 1;
        }
    } else {
        /* ── Audio path (TR-10-1 §8.10.1) ─────────────────────────────── */
        if (st->sr_due) {
            uint8_t srbuf[IPMX_RTCP_BUF_MAX];
            int64_t  ntp_for_sr_ns;
            uint32_t rtp_for_sr;
            /* All sample-count and wall-time arithmetic for SRs must use
             * audio_periods_per_pkt (the integer-truncated periods/packet
             * actually emitted by the RTP packetiser), NOT audio_ptime_us
             * directly. At non-integer-period ptimes (e.g. 44.1 kHz with
             * 1090 µs option → 48 periods → actual 1088.435 µs/packet),
             * pkt_idx × ptime_us drifts by ~1.5 µs/packet from the actual
             * wire packet rate. The drift causes SR rtp_timestamp values
             * not to match any RTP packet's timestamp in the validator's
             * association lookup. The periods-per-packet form matches the
             * RTP packetiser's actual TS progression exactly. */
            int64_t periods_per_pkt = st->audio_periods_per_pkt;
            int64_t pkt_us_num = (int64_t)st->audio_pkt_idx * periods_per_pkt * 1000000LL;
            int64_t sr_target_us = st->audio_packet_epoch_us +
                                    pkt_us_num / st->audio_sample_rate - 30;
            int srlen;
            uint64_t samples_in = (uint64_t)st->audio_pkt_idx *
                                  (uint64_t)periods_per_pkt;
            uint64_t rtp64 = (uint64_t)st->rtp_ts_base + samples_in;
            rtp_for_sr = (uint32_t)(rtp64 & 0xffffffffu);
            ntp_for_sr_ns = st->internal_clock_at_epoch_ns +
                            (int64_t)st->audio_pkt_idx * periods_per_pkt *
                            1000000000LL / st->audio_sample_rate;
            srlen = ipmx_build_sr(st, srbuf, sizeof(srbuf),
                                  rtp_for_sr, ntp_for_sr_ns,
                                  st->pkt_count, st->octet_count);
            if (srlen > 0)
                (void)ipmx_ring_push_into(st, &st->sr_ring,
                                          IPMX_EVT_RTCP_SR, IPMX_DSCP_AUDIO_TOS,
                                          sr_target_us, srbuf, srlen);
            st->sr_due = 0;
        }

        target_us = ipmx_audio_target_us_for_pkt(st);
        ret = ipmx_ring_push(st,
                             IPMX_EVT_RTP, IPMX_DSCP_AUDIO_TOS,
                             target_us, st->pkt_buf, st->pkt_buf_size);
        if (ret < 0) {
            av_log(NULL, AV_LOG_ERROR, "Ring push (audio RTP) failed: %d\n", ret);
            return ret;
        }
        st->pkt_count++;
        /* RFC 3550 §6.4.1: octet_count is "the total number of payload
         * octets (i.e., not including header or padding) transmitted".
         * For RTP packets with the X bit set (PEP/HDCP RFC 8285 ext
         * header), subtract the ext-header bytes from the delta too. */
        {
            int rtp_hlen = 12;
            if ((st->pkt_buf[0] & 0x10) && st->pkt_buf_size >= 16) {
                int xlen_words = AV_RB16(st->pkt_buf + 14);
                rtp_hlen += 4 + xlen_words * 4;
            }
            if (st->pkt_buf_size > rtp_hlen)
                st->octet_count += (uint32_t)(st->pkt_buf_size - rtp_hlen);
        }
        st->last_seq    = AV_RB16(st->pkt_buf + 2);
        st->last_rtp_ts = AV_RB32(st->pkt_buf + 4);

        st->audio_pkt_idx++;
        if (st->audio_sr_period_pkts > 0 &&
            (st->audio_pkt_idx % (int64_t)st->audio_sr_period_pkts) == 0) {
            st->sr_due = 1;
        }
    }

    {
        int rc = st->pkt_buf_size;
        st->pkt_buf_size = 0;
        return rc;
    }
}

/* ─────────────── SDP file writer (per-stream m= blocks) ─────────────── */

/* Forward declaration — definition is below `ipmx_write_one_sdp_file`,
 * but the per-codec block emitters above call it. */
static void ipmx_sdp_emit_source_filter(FILE *fp,
                                        const char *dst_ip,
                                        const char *src_ip);

/* Compute "ts-refclk:<value>" body once for re-use. */
static void ipmx_sdp_compute_tsref(IpmxMuxContext *ctx, char *out, int outsize)
{
    if (ctx->refclk && ctx->refclk[0]) {
        av_strlcpy(out, ctx->refclk, outsize);
    } else if (ctx->sender_mac && ctx->sender_mac[0]) {
        snprintf(out, outsize, "localmac=%s", ctx->sender_mac);
    } else {
        av_strlcpy(out, "localmac=00-20-FC-00-00-00", outsize);
    }
}

/* fmtp exactframerate value, ST 2110-20 §7.2: "Integer frame rates shall be
 * signaled as a single decimal number (e.g. "25") whilst non-integer frame
 * rates shall be signaled as a ratio of two integer decimal numbers
 * separated by a "forward-slash" character (e.g. "30000/1001"), utilizing
 * the numerically smallest numerator value possible." Used by the raw
 * (ST 2110-20) and the compressed (ST 2110-22) video blocks alike, from the
 * same rate the video MIB carries. */
static void ipmx_sdp_format_exactframerate(char *out, int outsize, int rate_num, int rate_den)
{
    int num, den;

    av_reduce(&num, &den, rate_num, rate_den, INT_MAX);
    if (den == 1)
        snprintf(out, outsize, "%d", num);
    else
        snprintf(out, outsize, "%d/%d", num, den);
}

static void ipmx_sdp_emit_video_raw_block(FILE *fp, IpmxStreamRT *st, int payload_type)
{
    IpmxMuxContext *ctx = st->mc;
    const char *sampling = "YCbCr-4:2:0";
    int rate_num = st->rate_num > 0 ? st->rate_num : 30;
    int rate_den = st->rate_den > 0 ? st->rate_den : 1;
    char exactframerate[32];
    char tsref[96];

    switch (st->pix_fmt) {
    case AV_PIX_FMT_UYVY422:
    case AV_PIX_FMT_YUV422P:
    case AV_PIX_FMT_YUV422P10:  sampling = "YCbCr-4:2:2"; break;
    case AV_PIX_FMT_YUV444P:    sampling = "YCbCr-4:4:4"; break;
    case AV_PIX_FMT_YUV420P:
    case AV_PIX_FMT_YUV420P10:  sampling = "YCbCr-4:2:0"; break;
    case AV_PIX_FMT_RGB24:      sampling = "RGB";         break;
    default:                    sampling = "YCbCr-4:2:0"; break;
    }

    ipmx_sdp_compute_tsref(ctx, tsref, sizeof(tsref));
    ipmx_sdp_format_exactframerate(exactframerate, sizeof(exactframerate), rate_num, rate_den);

    /* Synthesised non-baseband signal-timing fields. TR-10-1 §10.2 makes
     * `measuredpixclk`, `htotal`, `vtotal` conditional on baseband input;
     * for our non-baseband sources (file/lavfi/network) the spec language
     * doesn't require them, but the conformance suite and the streams/
     * reference (generate_video_test_streams.py:621-624 — htotal defaults
     * to cfg.width, vtotal to cfg.height, measured_pixclk = htotal·vtotal·fps)
     * expect a coherent set of values. Mirror that convention so:
     *   - the MIB fields (already emitted with the same numbers in
     *     ipmx_build_video_mib) match the SDP, and
     *   - validators that cross-check IPMX↔SDP (TP-10-1Sec13.3a, etc.)
     *     pass instead of flagging the absence as a "missing" field. */
    {
        uint64_t pixclk = (uint64_t)st->width * (uint64_t)st->height *
                          (uint64_t)rate_num / (uint64_t)rate_den;
        fprintf(fp,
                "m=video %d RTP/AVP %d\r\n"
                "c=IN IP4 %s/64\r\n"
                "a=rtcp:%d\r\n"
                "a=rtpmap:%d raw/90000\r\n"
                "a=fmtp:%d sampling=%s; width=%d; height=%d; "
                "exactframerate=%s; depth=%d; colorimetry=BT709; "
                "PM=2110GPM; SSN=ST2110-20:2017; TP=2110TPW; IPMX; "
                "TCS=SDR; RANGE=NARROW; "
                "measuredpixclk=%" PRIu64 "; htotal=%d; vtotal=%d\r\n"
                "a=mediaclk:%s\r\n"
                "a=ts-refclk:%s\r\n",
                st->dst_port_resolved, payload_type,
                st->dst_ip_resolved,
                st->dst_rtcp_port_resolved,
                payload_type, payload_type,
                sampling, st->width, st->height,
                exactframerate, st->bit_depth,
                pixclk, st->width, st->height,
                ctx->mediaclk ? ctx->mediaclk : "sender",
                tsref);
    }
    ipmx_sdp_emit_source_filter(fp, st->dst_ip_resolved, ctx->src_ip);
}

/* Emit H.264 / H.265 video SDP block.
 *
 * Calls FFmpeg's av_sdp_create() on the child rtp_ctx (which has the right
 * codec params + extradata). FFmpeg formats the rtpmap/fmtp lines correctly
 * for H.264 (RFC 6184: profile-level-id, packetization-mode, sprop-parameter-sets)
 * or H.265 (RFC 7798: profile-id, level-id, sprop-vps/sps/pps). We then strip
 * everything except the m=, a=rtpmap, a=fmtp lines and append IPMX-specific
 * attributes (the IPMX flag in fmtp, mediaclk, ts-refclk). */
static void ipmx_sdp_emit_video_compressed_block(FILE *fp, IpmxStreamRT *st,
                                                 int payload_type)
{
    IpmxMuxContext *ctx = st->mc;
    char  sdp_buf[8192];
    char *line, *saveptr = NULL;
    char  rtpmap_line[512] = "";
    char  fmtp_line[2048] = "";
    char  exactframerate[32];
    char  tsref[96];
    int   rate_num = st->rate_num > 0 ? st->rate_num : 30;
    int   rate_den = st->rate_den > 0 ? st->rate_den : 1;
    AVFormatContext *arr[1] = { st->child_rtp_ctx };

    if (!st->child_rtp_ctx ||
        av_sdp_create(arr, 1, sdp_buf, sizeof(sdp_buf)) < 0) {
        av_log(ctx, AV_LOG_WARNING, "Stream %d: av_sdp_create failed\n",
               st->src_stream_index);
        return;
    }

    /* Pluck the rtpmap and fmtp lines for this stream from the SDP. */
    for (line = strtok_r(sdp_buf, "\r\n", &saveptr); line;
         line = strtok_r(NULL, "\r\n", &saveptr)) {
        if (!strncmp(line, "a=rtpmap:", 9))
            av_strlcpy(rtpmap_line, line, sizeof(rtpmap_line));
        else if (!strncmp(line, "a=fmtp:", 7))
            av_strlcpy(fmtp_line, line, sizeof(fmtp_line));
    }

    ipmx_sdp_compute_tsref(ctx, tsref, sizeof(tsref));
    ipmx_sdp_format_exactframerate(exactframerate, sizeof(exactframerate), rate_num, rate_den);

    /* Emit the IPMX-conformant block. The IPMX flag goes into fmtp; if FFmpeg
     * already emitted a fmtp line we append "; IPMX" to it, otherwise we
     * synthesise a minimal one. */
    /* RFC 4566 §5 media-description ordering: m=, then i= (optional),
     * c=, b=*, k= (optional), a=*. b= MUST precede every a= line. The
     * grammar isn't ambiguous about this — `b=` belongs between `c=`
     * and the first `a=`. */
    fprintf(fp,
            "m=video %d RTP/AVP %d\r\n"
            "c=IN IP4 %s/64\r\n"
            "b=AS:%" PRId64 "\r\n"
            "a=rtcp:%d\r\n",
            st->dst_port_resolved, payload_type,
            st->dst_ip_resolved,
            (st->comp_max_bitrate_bps + 999) / 1000,  /* kbps, rounded up */
            st->dst_rtcp_port_resolved);

    if (rtpmap_line[0])
        fprintf(fp, "%s\r\n", rtpmap_line);
    else
        fprintf(fp, "a=rtpmap:%d %s/90000\r\n", payload_type,
                st->compressed_codec_id == AV_CODEC_ID_H264 ? "H264" : "H265");

    if (fmtp_line[0]) {
        /* Ensure the fmtp body ends with the IPMX keyword. RFC 6184 §8.1:
         * sprop-parameter-sets carries any sequence-and-picture parameter
         * set NAL units as a comma-separated base64 list — so for H.264
         * the FFmpeg-generated "SPS_b64,PPS_b64" is exactly what the
         * receiver and the streams/-side validator expect (TR-10-15c §16
         * inherits this syntax verbatim). We pass it through unchanged.
         *
         * Append measuredpixclk/htotal/vtotal so the IPMX↔SDP cross-check
         * sees the same nominal values that the MIB carries (see the
         * raw-video block above for the rationale; non-baseband but the
         * conformance reference still expects these fields). */
        char with_ipmx[2400];
        uint64_t pixclk = (uint64_t)st->width * (uint64_t)st->height *
                          (uint64_t)rate_num / (uint64_t)rate_den;
        av_strlcpy(with_ipmx, fmtp_line, sizeof(with_ipmx));
        /* ST 2110-22 requires explicit width/height in the SDP fmtp body
         * (MatroxSdpCheck::check_sdp_st2110_22 raises "invalid width"/
         * "invalid height" if md.width / md.height parse to 0). FFmpeg's
         * H.264/H.265 fmtp line normally carries dimensions only inside
         * the SPS/sprop-parameter-sets, so we add them explicitly here.
         * The frame rate is the fmtp exactframerate parameter, as for raw
         * video (ST 2110-22 also accepts a=framerate; the TR-10-15b / -15c
         * §16.1 example SDPs use exactframerate).
         * htotal/vtotal stay (TR-10-1 §10.2 IPMX-specific, separate). */
        av_strlcatf(with_ipmx, sizeof(with_ipmx),
                    "; width=%d; height=%d; exactframerate=%s; TP=2110TPW; measuredpixclk=%" PRIu64
                    "; htotal=%d; vtotal=%d",
                    st->width, st->height, exactframerate,
                    pixclk, st->width, st->height);

        /* H.265 NMOS / BCP-006-03 fmtp augmentation. RFC 7798 §7.1 and
         * AMWA BCP-006-03 require the SDP to carry the SPS profile_tier_level
         * fields plus tx-mode so a Receiver / NMOS Controller can negotiate
         * stream compatibility without parsing the bitstream. FFmpeg's stock
         * av_sdp_create() emits sprop-vps / sprop-sps / sprop-pps but none
         * of these PTL fields; we add them here.
         *
         * Policy: ALWAYS emit (unconditional) — matches existing IPMX-side
         * SDP convention (raw video's colorimetry=BT709 / TCS=SDR / RANGE=NARROW
         * / measuredpixclk / htotal / vtotal, audio's measuredsamplerate;
         * all hard-coded without default-omission). BCP-006-03 explicitly
         * permits omitting defaults, but emitting unconditionally is
         * over-compliant and gives diagnostic clarity. strstr() guards keep
         * the patch idempotent if FFmpeg's stock emitter ever adds these.
         *
         * tx-mode=SRST is mandatory per BCP-006-03 (no default — Single RTP
         * Stream Transport is the only mode the spec defines for declarative
         * SDP). */
        if (st->compressed_codec_id == AV_CODEC_ID_HEVC) {
            if (!strstr(with_ipmx, "tx-mode="))
                av_strlcatf(with_ipmx, sizeof(with_ipmx), "; tx-mode=" IPMX_HEVC_TX_MODE);
            if (st->hevc_ptl_valid) {
                if (!strstr(with_ipmx, "profile-space="))
                    av_strlcatf(with_ipmx, sizeof(with_ipmx),
                                "; profile-space=%u", st->hevc_profile_space);
                if (!strstr(with_ipmx, "profile-id="))
                    av_strlcatf(with_ipmx, sizeof(with_ipmx),
                                "; profile-id=%u", st->hevc_profile_idc);
                if (!strstr(with_ipmx, "tier-flag="))
                    av_strlcatf(with_ipmx, sizeof(with_ipmx),
                                "; tier-flag=%u", st->hevc_tier_flag);
                if (!strstr(with_ipmx, "level-id="))
                    av_strlcatf(with_ipmx, sizeof(with_ipmx),
                                "; level-id=%u", st->hevc_level_idc);
                if (!strstr(with_ipmx, "profile-compatibility-indicator="))
                    av_strlcatf(with_ipmx, sizeof(with_ipmx),
                                "; profile-compatibility-indicator=%08X",
                                st->hevc_profile_compat);
                if (!strstr(with_ipmx, "interop-constraints="))
                    av_strlcatf(with_ipmx, sizeof(with_ipmx),
                                "; interop-constraints=%02X%02X%02X%02X%02X%02X",
                                st->hevc_interop_constraints[0],
                                st->hevc_interop_constraints[1],
                                st->hevc_interop_constraints[2],
                                st->hevc_interop_constraints[3],
                                st->hevc_interop_constraints[4],
                                st->hevc_interop_constraints[5]);
            }
        }

        if (!strstr(with_ipmx, " IPMX") && !strstr(with_ipmx, ";IPMX"))
            av_strlcatf(with_ipmx, sizeof(with_ipmx), "; IPMX");
        fprintf(fp, "%s\r\n", with_ipmx);
    } else {
        /* Fallback path — av_sdp_create() failed to emit an a=fmtp line.
         * Synthesise a minimal one with the IPMX-side extras, plus the H.265
         * NMOS PTL fields when applicable. In practice this branch is
         * unreachable for H.264/H.265 with non-empty extradata. */
        uint64_t pixclk = (uint64_t)st->width * (uint64_t)st->height *
                          (uint64_t)rate_num / (uint64_t)rate_den;
        char synth_fmtp[1024];
        snprintf(synth_fmtp, sizeof(synth_fmtp),
                 "a=fmtp:%d width=%d; height=%d; exactframerate=%s; TP=2110TPW; measuredpixclk=%" PRIu64
                 "; htotal=%d; vtotal=%d",
                 payload_type, st->width, st->height, exactframerate,
                 pixclk, st->width, st->height);
        if (st->compressed_codec_id == AV_CODEC_ID_HEVC) {
            av_strlcatf(synth_fmtp, sizeof(synth_fmtp), "; tx-mode=" IPMX_HEVC_TX_MODE);
            if (st->hevc_ptl_valid) {
                av_strlcatf(synth_fmtp, sizeof(synth_fmtp),
                            "; profile-space=%u; profile-id=%u; tier-flag=%u; level-id=%u"
                            "; profile-compatibility-indicator=%08X"
                            "; interop-constraints=%02X%02X%02X%02X%02X%02X",
                            st->hevc_profile_space, st->hevc_profile_idc,
                            st->hevc_tier_flag,    st->hevc_level_idc,
                            st->hevc_profile_compat,
                            st->hevc_interop_constraints[0],
                            st->hevc_interop_constraints[1],
                            st->hevc_interop_constraints[2],
                            st->hevc_interop_constraints[3],
                            st->hevc_interop_constraints[4],
                            st->hevc_interop_constraints[5]);
            }
        }
        av_strlcatf(synth_fmtp, sizeof(synth_fmtp), "; IPMX");
        fprintf(fp, "%s\r\n", synth_fmtp);
    }

    fprintf(fp,
            "a=mediaclk:%s\r\n"
            "a=ts-refclk:%s\r\n",
            ctx->mediaclk ? ctx->mediaclk : "sender",
            tsref);
    ipmx_sdp_emit_source_filter(fp, st->dst_ip_resolved, ctx->src_ip);
}

static void ipmx_sdp_emit_video_block(FILE *fp, IpmxStreamRT *st, int payload_type)
{
    if (st->is_compressed_video)
        ipmx_sdp_emit_video_compressed_block(fp, st, payload_type);
    else
        ipmx_sdp_emit_video_raw_block(fp, st, payload_type);
}

static void ipmx_sdp_emit_audio_block(FILE *fp, IpmxStreamRT *st, int payload_type)
{
    IpmxMuxContext *ctx = st->mc;
    char tsrefbuf[96];
    const char *tsref;
    double ptime_ms = (double)st->audio_ptime_us / 1000.0;
    const char *channel_order = st->audio_channel_order[0]
                                  ? st->audio_channel_order
                                  : "SMPTE2110.(ST)";
    char rtpmap_line[64];

    if (ctx->refclk && ctx->refclk[0]) {
        tsref = ctx->refclk;
    } else if (ctx->sender_mac && ctx->sender_mac[0]) {
        snprintf(tsrefbuf, sizeof(tsrefbuf), "localmac=%s", ctx->sender_mac);
        tsref = tsrefbuf;
    } else {
        tsref = "localmac=00-20-FC-00-00-00";
    }

    /* rtpmap encoding name — "AM824" for ST 2110-31 (TR-10-12 §10), "L24" /
     * "L16" for plain PCM (ST 2110-30). */
    if (st->audio_is_am824) {
        snprintf(rtpmap_line, sizeof(rtpmap_line),
                 "AM824/%d/%d",
                 st->audio_sample_rate, st->audio_channels);
    } else {
        snprintf(rtpmap_line, sizeof(rtpmap_line),
                 "L%d/%d/%d",
                 st->audio_sample_size, st->audio_sample_rate, st->audio_channels);
    }

    /* `measuredsamplerate` is conditional on baseband input per TR-10-1
     * §10.3, but our streams/ reference (ipmx_pcm.py:342) and the
     * conformance suite expect a non-zero nominal value for non-baseband
     * sources too. Mirror the streams/ convention: emit the nominal
     * sample_rate so the SDP matches the audio MIB's measured_sample_rate
     * field (also nominal, see ipmx_build_audio_mib call site). */
    fprintf(fp,
            "m=audio %d RTP/AVP %d\r\n"
            "c=IN IP4 %s/64\r\n"
            "a=rtcp:%d\r\n"
            "a=rtpmap:%d %s\r\n"
            "a=fmtp:%d channel-order=%s; measuredsamplerate=%d; IPMX\r\n"
            "a=ptime:%g\r\n"
            "a=mediaclk:%s\r\n"
            "a=ts-refclk:%s\r\n",
            st->dst_port_resolved, payload_type,
            st->dst_ip_resolved,
            st->dst_rtcp_port_resolved,
            payload_type,
            rtpmap_line,
            payload_type,
            channel_order,
            st->audio_sample_rate,
            ptime_ms,
            ctx->mediaclk ? ctx->mediaclk : "sender",
            tsref);
    ipmx_sdp_emit_source_filter(fp, st->dst_ip_resolved, ctx->src_ip);
}

/* Emit SDP files. The streams/ MatroxSdp parser (and per-stream IPMX
 * validators) accept a single-media SDP per file (TR-10-1 §10 + ST 2110-10:
 * "one SDP object per RTP Stream"). We therefore write one SDP per stream:
 *   - If `ctx->sdp_file` is "/path/foo.sdp", we emit /path/foo_<idx>.sdp
 *     for each stream (idx = 0 video, 1..N audio).
 *   - We also emit /path/foo.sdp as a multi-media bundle for human reading
 *     (validators that require single-media use the per-stream files). */
static void ipmx_write_one_sdp_file(IpmxMuxContext *ctx, AVFormatContext *s,
                                    const char *path, int single_stream_idx)
{
    FILE *fp = fopen(path, "w");
    int i;
    if (!fp) {
        av_log(s, AV_LOG_WARNING, "Cannot open SDP file '%s'\n", path);
        return;
    }
    {
        int64_t sess = ctx->internal_clock_anchor_unix_ns / 1000;
        const char *src_for_origin = (ctx->src_ip && ctx->src_ip[0]) ? ctx->src_ip : "127.0.0.1";
        fprintf(fp,
                "v=0\r\n"
                "o=- %" PRId64 " %" PRId64 " IN IP4 %s\r\n"
                "s=IPMX media out\r\n"
                "t=0 0\r\n",
                sess, sess, src_for_origin);
    }
    for (i = 0; i < ctx->nb_streams; i++) {
        IpmxStreamRT *st = &ctx->streams[i];
        if (single_stream_idx >= 0 && i != single_stream_idx) continue;
        /* Use the per-stream PT resolved at setup time so each m= block
         * carries its own distinct dynamic PT (96..127). */
        if (st->is_audio)
            ipmx_sdp_emit_audio_block(fp, st, st->payload_type);
        else
            ipmx_sdp_emit_video_block(fp, st, st->payload_type);
        /* TR-10-13 PEP: a=privacy declares the cipher mode and IV; a=extmap
         * declares the RFC 8285 one-byte ext-header URNs the receiver must
         * use to recover the IV/counter for each packet. Emitted once per
         * m= section. */
        if (s->privacy_encrypt) {
            const char *protocol = s->pep_rtp_kv ? "RTP_KV" : "RTP";
            const char *mode = (s->pep_mode && s->pep_mode[0]) ? s->pep_mode : "AES-128-CTR";
            fprintf(fp,
                    "a=privacy:protocol=%s; mode=%s; iv=%s; "
                    "key_generator=%s; key_version=%s; key_id=%s\r\n"
                    "a=extmap:%d urn:ietf:params:rtp-hdrext:PEP-Full-IV-Counter\r\n"
                    "a=extmap:%d urn:ietf:params:rtp-hdrext:PEP-Short-IV-Counter\r\n",
                    protocol, mode,
                    ctx->pep_iv_hex,
                    ctx->pep_key_generator_hex,
                    ctx->pep_key_version_hex,
                    ctx->pep_key_id_hex,
                    ctx->pep_extmap_id_full, ctx->pep_extmap_id_short);
        }
    }
    fclose(fp);
}

/**
 * RFC 4570 source-filter line: emit `a=source-filter: incl IN IP4 <dst> <src>`
 * iff `dst_ip` is in 224.0.0.0/4 (= IPv4 multicast). For unicast m= blocks
 * the line MUST NOT be emitted (RFC 4570 only applies to multicast). This
 * is required by TR-10-9 §17 SHALL — without it, IPMX receivers can't
 * issue an SSM (S,G) IGMPv3 join for the stream.
 */
static void ipmx_sdp_emit_source_filter(FILE *fp,
                                        const char *dst_ip,
                                        const char *src_ip)
{
    struct in_addr addr;
    if (!fp || !dst_ip || !dst_ip[0] || !src_ip || !src_ip[0])
        return;
    if (inet_pton(AF_INET, dst_ip, &addr) != 1)
        return;
    /* IPv4 multicast = 224.0.0.0/4 (top 4 bits == 1110 == 0xE). */
    if ((ntohl(addr.s_addr) >> 28) != 0xE)
        return;
    fprintf(fp, "a=source-filter: incl IN IP4 %s %s\r\n", dst_ip, src_ip);
}

/**
 * Auto-detect the muxer's outgoing-interface IPv4 address and store it
 * into ctx->src_ip. Required for TR-10-9 §17 (the SDP a=source-filter
 * line on multicast streams MUST carry the actual sender source) and
 * cleans up the SDP `o=` origin line and PCAP IP synthesis at the same
 * time.
 *
 * Method: open a throwaway AF_INET SOCK_DGRAM socket and connect() it to
 * the bundle's destination. UDP connect() is a route-lookup primitive —
 * it sends no traffic, it just records the destination on the socket and
 * forces the kernel to choose an outgoing interface. getsockname() then
 * returns that interface's IP. The throwaway socket is closed
 * immediately; no traffic is generated.
 *
 * Resolution order:
 *   1. If ctx->src_ip is set to anything other than the AVOption default
 *      "127.0.0.1", honour it (explicit user override; useful for NAT,
 *      test fixtures, multi-NIC tie-breaking).
 *   2. Otherwise run the connect+getsockname dance; on success, replace
 *      ctx->src_ip.
 *   3. On failure, leave ctx->src_ip alone and log a warning — the SDP
 *      can still be emitted (validators will flag the gap if it's wrong).
 */
static void ipmx_resolve_src_ip(IpmxMuxContext *ctx, AVFormatContext *s)
{
    char detected[INET_ADDRSTRLEN] = "";
    int  tmp_fd;
    struct sockaddr_in dst, local;
    socklen_t local_len = sizeof(local);

    /* Honour explicit user override. The AVOption default is "127.0.0.1";
     * anything else means the user asked for a specific source. */
    if (ctx->src_ip && ctx->src_ip[0] &&
        strcmp(ctx->src_ip, "127.0.0.1") != 0) {
        return;
    }
    if (ctx->nb_streams <= 0)
        return;

    tmp_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (tmp_fd < 0) {
        av_log(s, AV_LOG_WARNING,
               "IPMX: source-IP auto-detect failed (socket: %s)\n",
               strerror(errno));
        return;
    }
    memset(&dst, 0, sizeof(dst));
    dst.sin_family = AF_INET;
    dst.sin_port   = htons((uint16_t)ctx->streams[0].dst_port_resolved);
    if (inet_pton(AF_INET, ctx->streams[0].dst_ip_resolved,
                  &dst.sin_addr) == 1
        && connect(tmp_fd, (struct sockaddr *)&dst, sizeof(dst)) == 0
        && getsockname(tmp_fd, (struct sockaddr *)&local, &local_len) == 0
        && local.sin_family == AF_INET
        && local.sin_addr.s_addr != htonl(INADDR_ANY)) {
        inet_ntop(AF_INET, &local.sin_addr, detected, sizeof(detected));
    }
    close(tmp_fd);

    if (detected[0]) {
        av_freep(&ctx->src_ip);
        ctx->src_ip = av_strdup(detected);
        av_log(s, AV_LOG_VERBOSE,
               "IPMX: auto-detected source IP %s for destination %s\n",
               detected, ctx->streams[0].dst_ip_resolved);
    } else {
        av_log(s, AV_LOG_WARNING,
               "IPMX: could not auto-detect source IP for destination %s; "
               "keeping %s. SDP a=source-filter on multicast streams may "
               "be wrong — set -ipmx_src_ip explicitly to override.\n",
               ctx->streams[0].dst_ip_resolved,
               (ctx->src_ip && ctx->src_ip[0]) ? ctx->src_ip : "(none)");
    }
}

static void ipmx_write_sdp_file(IpmxMuxContext *ctx, AVFormatContext *s)
{
    char path[1024];
    char base[1024];
    char *dot;
    int  i;

    if (!ctx->sdp_file || !ctx->sdp_file[0]) return;

    /* Resolve ctx->src_ip from the kernel before any SDP writer runs, so
     * the o= origin line and the per-m= a=source-filter lines (TR-10-9
     * §17) all reflect the real sending interface. */
    ipmx_resolve_src_ip(ctx, s);

    /* Bundle SDP: all streams in one file (for human reading). */
    ipmx_write_one_sdp_file(ctx, s, ctx->sdp_file, /*single_stream_idx=*/-1);
    av_log(s, AV_LOG_INFO, "IPMX bundle SDP → %s\n", ctx->sdp_file);

    /* Per-stream SDPs: foo.sdp → foo_<idx>.sdp for each stream. */
    av_strlcpy(base, ctx->sdp_file, sizeof(base));
    dot = strrchr(base, '.');
    if (dot && !strcmp(dot, ".sdp")) *dot = '\0';
    for (i = 0; i < ctx->nb_streams; i++) {
        snprintf(path, sizeof(path), "%s_%d.sdp", base, i);
        ipmx_write_one_sdp_file(ctx, s, path, i);
        av_log(s, AV_LOG_INFO, "IPMX stream-%d SDP → %s\n", i, path);
    }
}

/* ─────────────── per-stream setup ─────────────── */

/* Nominal frame rate (N or N·1000/1001 fps) within
 * IPMX_FRAME_RATE_SNAP_TOLERANCE of num/den, if any. */
static int ipmx_snap_frame_rate(int64_t num, int64_t den, int *out_num, int *out_den)
{
    double fps    = (double)num / (double)den;
    double n_int  = round(fps);
    double n_1001 = round(fps * 1.001);

    if (n_int >= 1 && n_int <= IPMX_MIB_RATE_NUM_MAX &&
        fabs(fps - n_int) <= IPMX_FRAME_RATE_SNAP_TOLERANCE * fps) {
        *out_num = (int)n_int;
        *out_den = 1;
        return 1;
    }
    if (n_1001 >= 1 && n_1001 * 1000 <= IPMX_MIB_RATE_NUM_MAX &&
        fabs(fps - n_1001 * 1000.0 / 1001.0) <= IPMX_FRAME_RATE_SNAP_TOLERANCE * fps) {
        *out_num = (int)n_1001 * 1000;
        *out_den = 1001;
        return 1;
    }
    return 0;
}

/* Frame rate of a video stream: the video MIB rate field, the SDP
 * exactframerate and the frame epoch cadence all use it.
 *   - H.264 / H.265: the SPS VUI timing, which TR-10-15c / -15b require to
 *     carry the frame rate (H.264: num_units_in_tick = frame rate
 *     denominator, time_scale = twice its numerator; H.265:
 *     vui_num_units_in_tick = denominator, vui_time_scale = numerator).
 *   - Otherwise (no VUI timing, raw video): the container rate
 *     (avg_frame_rate, else r_frame_rate), taken as the nominal N or
 *     N·1000/1001 rate it is within IPMX_FRAME_RATE_SNAP_TOLERANCE of.
 * A rate the MIB rate field cannot carry is approximated, with a warning. */
static void ipmx_select_video_rate(AVFormatContext *s, IpmxStreamRT *st,
                                   const AVStream *src_st, int codec_id)
{
    AVRational container = src_st->avg_frame_rate;
    const char *source;
    int num = 30, den = 1;

    if (container.num <= 0 || container.den <= 0)
        container = src_st->r_frame_rate;

    if (st->is_compressed_video && st->vui_time_scale && st->vui_num_units_in_tick) {
        int64_t ticks_per_frame = codec_id == AV_CODEC_ID_H264 ? 2 : 1;
        av_reduce(&num, &den, st->vui_time_scale,
                  ticks_per_frame * st->vui_num_units_in_tick, INT_MAX);
        source = "SPS VUI timing";
        if (container.num > 0 && container.den > 0 &&
            fabs(av_q2d(container) - (double)num / den) >
            IPMX_FRAME_RATE_SNAP_TOLERANCE * ((double)num / den))
            av_log(s, AV_LOG_WARNING,
                   "IPMX stream=%d: SPS VUI timing gives %d/%d fps but the container "
                   "%d/%d fps; using the SPS rate\n",
                   (int)(st - st->mc->streams), num, den, container.num, container.den);
    } else if (container.num > 0 && container.den > 0) {
        source = "container (nominal rate)";
        if (!ipmx_snap_frame_rate(container.num, container.den, &num, &den)) {
            av_reduce(&num, &den, container.num, container.den, INT_MAX);
            source = "container";
        }
    } else {
        source = "default (no rate signalled)";
    }

    if (num > IPMX_MIB_RATE_NUM_MAX || den > IPMX_MIB_RATE_DEN_MAX) {
        int approx_num, approx_den;
        if (!ipmx_snap_frame_rate(num, den, &approx_num, &approx_den))
            av_reduce(&approx_num, &approx_den, num, den, IPMX_MIB_RATE_DEN_MAX);
        av_log(s, AV_LOG_WARNING,
               "IPMX stream=%d: frame rate %d/%d does not fit the video MIB rate field "
               "(22-bit numerator, 10-bit denominator); using %d/%d\n",
               (int)(st - st->mc->streams), num, den, approx_num, approx_den);
        num = approx_num;
        den = approx_den;
    }

    st->rate_num   = num;
    st->rate_den   = den;
    st->t_frame_us = (int64_t)1000000 * den / num;
    av_log(s, AV_LOG_VERBOSE, "IPMX stream=%d: frame rate %d/%d fps from %s\n",
           (int)(st - st->mc->streams), num, den, source);
}

static int ipmx_setup_stream(AVFormatContext *s, IpmxStreamRT *st,
                             AVStream *src_st, int idx,
                             const char *host, int port,
                             int payload_type)
{
    IpmxMuxContext *ctx = s->priv_data;
    AVFormatContext *rtp_ctx = NULL;
    AVStream *rtp_st = NULL;
    char rtp_url[1024], rtcp_url[1024];
    int  rtcp_port = port + 1;
    int  ret;
    int  is_video = (src_st->codecpar->codec_type == AVMEDIA_TYPE_VIDEO);
    int  pkt_size_for_url = is_video ? ctx->video_pkt_size : 1500;

    st->mc = ctx;
    st->src_stream_index = idx;
    st->is_audio = !is_video;

    /* Configure stream-format-specific knobs. */
    if (is_video) {
        int codec_id = src_st->codecpar->codec_id;
        st->width  = src_st->codecpar->width;
        st->height = src_st->codecpar->height;
        st->pix_fmt = src_st->codecpar->format;

        if (codec_id == AV_CODEC_ID_H264 || codec_id == AV_CODEC_ID_HEVC) {
            /* Compressed video — H.264 (RFC 6184 / TR-10-15c) or H.265
             * (RFC 7798 / TR-10-15b). Pace per packet at the encoder's
             * declared bit_rate × β (TR-10-7 §10 + ST 2110-21 §7.1). */
            st->is_compressed_video = 1;
            st->compressed_codec_id = codec_id;
            st->h264_packetization_mode = 1;  /* RFC 6184 non-interleaved */
            st->payload_size = ctx->video_pkt_size;
            /* Seed parameter-set cache from codecpar->extradata (Annex B
             * SPS/PPS/VPS as written by the H.264/HEVC demuxer for raw
             * .h264/.h265 inputs). The bitstream walk on each packet then
             * keeps the cache fresh if the source updates SPS/PPS in-band. */
            if (src_st->codecpar->extradata && src_st->codecpar->extradata_size > 0) {
                ipmx_cache_param_sets(st,
                                      src_st->codecpar->extradata,
                                      src_st->codecpar->extradata_size,
                                      codec_id == AV_CODEC_ID_HEVC);
            }
            /* HEVC: parse the SPS profile_tier_level block into the
             * st->hevc_* cache. Used by ipmx_build_h265_mib (replaces its
             * inline parse) and by the SDP fmtp augmentation. */
            if (codec_id == AV_CODEC_ID_HEVC) {
                ipmx_extract_h265_ptl(st);
                ipmx_warn_h265_sprop_overflow(s, st, (int)(st - ctx->streams));
            }
            if (ctx->hrd_trace_cap > 0) {
                st->hrd_trace = av_calloc(ctx->hrd_trace_cap, sizeof(*st->hrd_trace));
                if (!st->hrd_trace)
                    return AVERROR(ENOMEM);
            }
            /* Parse SPS VUI HRD parameters (TR-10-15c / TR-10-15b §15,
             * H.264 §C.1.1 / H.265 §C.2.1.1). On success populates
             * st->hrd_* fields and sets st->hrd_present = 1. On failure or
             * absence of HRD, hrd_present stays 0 and the pacer falls back
             * to the legacy codecpar->bit_rate schedule. */
            {
                int hret = ipmx_extract_sps_hrd(st, codec_id, s);
                if (hret < 0)
                    return hret;
            }
            /* comp_max_bitrate_bps: prefer the SPS-VUI HRD-declared rate
             * (TR-10-15c §15 / H.264 §C.1.1 eq C-2 require the HRD max
             * for inter-AU pacing); fall back to codecpar->bit_rate (and
             * then to a 4 Mbps default for our demo media) when HRD is
             * not present in the bitstream. */
            if (st->hrd_present && st->hrd_max_bitrate_bps > 0) {
                st->comp_max_bitrate_bps = st->hrd_max_bitrate_bps;
                av_log(s, AV_LOG_VERBOSE,
                       "IPMX HRD: stream=%d codec=%s cbr_flag=%d "
                       "max_bitrate=%lld bps cpb_size=%lld bits clock_tick=%u/%u s\n",
                       (int)(st - ctx->streams),
                       codec_id == AV_CODEC_ID_H264 ? "H.264" : "H.265",
                       st->hrd_cbr_flag,
                       (long long)st->hrd_max_bitrate_bps,
                       (long long)st->hrd_cpb_size_bits,
                       st->vui_num_units_in_tick, st->vui_time_scale);
            } else if (src_st->codecpar->bit_rate > 0) {
                st->comp_max_bitrate_bps = src_st->codecpar->bit_rate;
                av_log(s, AV_LOG_WARNING,
                       "IPMX HRD: stream=%d %s — VUI HRD not present; falling back to "
                       "codecpar->bit_rate (%lld bps). Wire pacing will not be bit-exact "
                       "to TR-10-15c §15 eq C-2.\n",
                       (int)(st - ctx->streams),
                       codec_id == AV_CODEC_ID_H264 ? "H.264" : "H.265",
                       (long long)st->comp_max_bitrate_bps);
            } else {
                st->comp_max_bitrate_bps = 4000000;
                av_log(s, AV_LOG_WARNING,
                       "IPMX HRD: stream=%d %s — neither VUI HRD nor codecpar->bit_rate "
                       "available; defaulting to 4 Mbps. Wire pacing will not match the "
                       "encoder's actual rate.\n",
                       (int)(st - ctx->streams),
                       codec_id == AV_CODEC_ID_H264 ? "H.264" : "H.265");
            }
            /* For RFC 6184/7798 there's no fixed "packets per frame" — so we
             * skip the RFC 4175 pgroup math. packets_per_frame stays 0; the
             * compressed pacer uses cumulative-byte timing instead. */
        } else {
            /* Raw uncompressed video (RFC 4175). All pgroup parameters come
             * from the rfc4175 packetizer's table — single source of truth,
             * keyed on (codec_id, pix_fmt) so memory-packed pix_fmts
             * (UYVY422, RGB24) are accepted under either RAWVIDEO or
             * BITPACKED, and planar ones (YUV420P, YUV422P, YUV422P10,
             * YUV444P, YUV420P10) require BITPACKED upstream. */
            const FFRFC4175Format *rf =
                ff_rfc4175_get_format(src_st->codecpar->codec_id,
                                      src_st->codecpar->format);
            if (!rf) {
                av_log(s, AV_LOG_ERROR,
                       "Stream %d: RFC 4175 / IPMX raw video does not "
                       "support pixel format %s under codec %s. Use "
                       "-c:v rawvideo for memory-packed pix_fmts (uyvy422 / "
                       "rgb24) or -c:v bitpacked for planar "
                       "pix_fmts (yuv420p / yuv422p / yuv422p10 / yuv444p).\n",
                       idx, av_get_pix_fmt_name(src_st->codecpar->format),
                       avcodec_get_name(src_st->codecpar->codec_id));
                return AVERROR(EINVAL);
            }
            st->pgroup_octets = rf->pgroup;
            st->pgroup_pixels = rf->pixels;
            st->bit_depth     = rf->depth;
            st->payload_size  = ctx->video_pkt_size;
            st->bytes_per_frame = (st->width * st->height * st->pgroup_octets) /
                                  st->pgroup_pixels;
            st->packets_per_frame = ipmx_compute_packets_per_frame(
                    st->width, st->height, st->pgroup_octets,
                    rf->xinc, rf->yinc, st->payload_size);
        }
        /* After ipmx_extract_sps_hrd: H.264 / H.265 take it from the SPS. */
        ipmx_select_video_rate(s, st, src_st, codec_id);
    } else {
        /* Audio stream (TR-10-1 §8.10.1, AES67 §7.5). */
        int codec_id = src_st->codecpar->codec_id;
        st->audio_sample_rate = src_st->codecpar->sample_rate ? src_st->codecpar->sample_rate : 48000;
        st->audio_channels    = src_st->codecpar->ch_layout.nb_channels
                                  ? src_st->codecpar->ch_layout.nb_channels : 2;
        st->audio_is_am824    = (codec_id == AV_CODEC_ID_AM824);
        /* For AM824, the encoder publishes wire-content metadata
         * (sample_size, non-PCM mask, fmtp channel-order) as
         * AV_PKT_DATA_AM824_STREAM_INFO on AVStream::codecpar->coded_side_data.
         * That's the single source of truth for SDP fmtp + audio MIB. */
        const AVAM824StreamInfo *am824_info = NULL;
        if (st->audio_is_am824) {
            const AVPacketSideData *sd = av_packet_side_data_get(
                    src_st->codecpar->coded_side_data,
                    src_st->codecpar->nb_coded_side_data,
                    AV_PKT_DATA_AM824_STREAM_INFO);
            if (sd && sd->size >= sizeof(AVAM824StreamInfo))
                am824_info = (const AVAM824StreamInfo *)sd->data;
            st->audio_sample_size = am824_info ? am824_info->sample_size_bits : 24;
        } else {
            st->audio_sample_size = (codec_id == AV_CODEC_ID_PCM_S24BE) ? 24 :
                                    (codec_id == AV_CODEC_ID_PCM_S16BE) ? 16 : 24;
        }
        /* Default ptime: 1 ms for both PCM (TR-10-3) and AM824 (TR-10-12 /
         * ST 2110-31 §6.1 Table 1 lists 1 ms / 125 µs / 83 µs as permitted;
         * 1 ms is interoperable with AES67 §7.2.1 "Required packet time"
         * and matches the SHOULD jitter budget of 1 packet time = 1 ms).
         * Lower ptimes are available via -ipmx_audio_ptime_us when the
         * channel count needs more headroom in the standard 1500-byte UDP
         * MTU (e.g. 7.1 AM824 = 8 ch × 4 B × 48 periods/ms = 1536 B which
         * exceeds the MTU at 1 ms — drop to 125 µs to fit). */
        if (ctx->audio_ptime_us > 0)
            st->audio_ptime_us = ctx->audio_ptime_us;
        else
            st->audio_ptime_us = 1000;
        /* SR cadence: every N packets where N = floor(10 ms / ptime).
         * (TR-10-1 §8.10.1: "every N packets after [the first], where N is
         * the integer result of dividing 10 milliseconds by the nominal
         * packet time".) */
        st->audio_sr_period_pkts = 10000 / st->audio_ptime_us;
        if (st->audio_sr_period_pkts <= 0) st->audio_sr_period_pkts = 1;
        {
            /* periods_per_pkt = floor(ptime_us × sample_rate / 1e6). At
             * non-integer-period ptimes (e.g. 1090 µs at 44.1 kHz: 48.069
             * → 48), the truncation determines the actual on-wire RTP-TS
             * delta per packet. We store this so all downstream sample-
             * count and wall-clock-per-packet arithmetic matches what the
             * RTP packetiser actually emits. */
            int periods = (int)((int64_t)st->audio_ptime_us *
                                st->audio_sample_rate / 1000000LL);
            if (periods <= 0) periods = 1;
            st->audio_periods_per_pkt = periods;
            if (st->audio_is_am824) {
                st->audio_payload_bytes = periods * st->audio_channels * 4;
            } else {
                st->audio_payload_bytes = periods * st->audio_channels *
                                          (st->audio_sample_size / 8);
            }
        }
        /* SDP/MIB channel-order — see ST 2110-30 §6.2.2 / TR-10-12. For
         * AM824 streams the encoder publishes the fmtp channel-order
         * string and non-PCM flag via AV_PKT_DATA_AM824_STREAM_INFO; we
         * just consume it. Non-AM824 (PCM L24/L16) defaults from the
         * stream's channel count to the matching SMPTE 2110-30 multichannel
         * token (M / ST / 51 / 71). Anything outside those falls back to
         * `(ST)` for compatibility with the legacy assumption. */
        if (st->audio_is_am824 && am824_info) {
            st->audio_is_non_pcm = am824_info->is_non_pcm;
            av_strlcpy(st->audio_channel_order, am824_info->channel_order,
                       sizeof(st->audio_channel_order));
        } else {
            const char *ord;
            st->audio_is_non_pcm = 0;
            switch (st->audio_channels) {
            case 1:  ord = "SMPTE2110.(M)";    break;
            case 2:  ord = "SMPTE2110.(ST)";   break;
            case 6:  ord = "SMPTE2110.(51)";   break;
            case 8:  ord = "SMPTE2110.(71)";   break;
            case 24: ord = "SMPTE2110.(222)";  break;
            default: ord = "SMPTE2110.(ST)";   break;
            }
            av_strlcpy(st->audio_channel_order, ord,
                       sizeof(st->audio_channel_order));
        }
    }

    /* Per-stream lookahead window for the pacer drain heuristic. Sized to
     * the stream's natural period so the heap holds roughly K periods of
     * timeline per stream — bursts on one stream don't crowd out other
     * streams' near-due events. */
    {
        int k_video = ctx->k_video > 0 ? ctx->k_video : IPMX_DEFAULT_K_VIDEO;
        int k_audio = ctx->k_audio > 0 ? ctx->k_audio : IPMX_DEFAULT_K_AUDIO;
        if (is_video)
            st->lookahead_us = (int64_t)k_video * st->t_frame_us;
        else
            st->lookahead_us = (int64_t)k_audio * (int64_t)st->audio_ptime_us;
        if (st->lookahead_us <= 0) st->lookahead_us = 1000;  /* sanity */
    }

    /* Per-stream MPSC ring (producer→pacer). Video gets the larger ring
     * because it bursts ~336 packets per frame; audio's predictable 1-packet
     * cadence only needs a small ring with slack for scheduler jitter. */
    {
        int ring_cap = is_video ? IPMX_RING_CAP_VIDEO : IPMX_RING_CAP_AUDIO;
        if ((ret = ipmx_ring_init(&st->ring, ring_cap)) < 0) {
            av_log(s, AV_LOG_ERROR, "Stream %d: cannot allocate ring (%d)\n", idx, ret);
            return ret;
        }
        /* Dedicated ring for RTCP SRs. Must be large enough to absorb
         * ffmpeg's `-re` 0.5 s initial_burst worth of SRs without
         * back-pressuring the producer thread (which then back-pressures
         * the demux thread's PTS-pace pulling, surfacing as "Resumed
         * reading" lag warnings). For audio at 1 ms ptime the muxer
         * schedules one SR every 10 ms (audio_sr_period_pkts), so
         * 0.5 s burst = 50 SRs; cap 256 gives 2.5 s of headroom. Video
         * fires one SR per AU (33 ms) so cap 256 is multi-second
         * headroom there too. */
        if ((ret = ipmx_ring_init(&st->sr_ring, IPMX_RING_CAP_SR)) < 0) {
            av_log(s, AV_LOG_ERROR, "Stream %d: cannot allocate SR ring (%d)\n", idx, ret);
            return ret;
        }
    }

    /* Open per-stream UDP sockets (RTP + RTCP). */
    snprintf(rtp_url,  sizeof(rtp_url),  "udp://%s:%d?pkt_size=%d", host, port, pkt_size_for_url);
    snprintf(rtcp_url, sizeof(rtcp_url), "udp://%s:%d?pkt_size=%d", host, rtcp_port, IPMX_RTCP_BUF_MAX);
    if ((ret = ffurl_open_whitelist(&st->udp_h, rtp_url, AVIO_FLAG_WRITE,
                                    &s->interrupt_callback, NULL,
                                    s->protocol_whitelist, s->protocol_blacklist, NULL)) < 0) {
        av_log(s, AV_LOG_ERROR, "Stream %d: cannot open RTP UDP %s: %d\n", idx, rtp_url, ret);
        return ret;
    }
    if ((ret = ffurl_open_whitelist(&st->udp_rtcp_h, rtcp_url, AVIO_FLAG_WRITE,
                                    &s->interrupt_callback, NULL,
                                    s->protocol_whitelist, s->protocol_blacklist, NULL)) < 0) {
        av_log(s, AV_LOG_ERROR, "Stream %d: cannot open RTCP UDP %s: %d\n", idx, rtcp_url, ret);
        return ret;
    }
    ipmx_set_dscp(st->udp_h,      is_video ? IPMX_DSCP_VIDEO_TOS : IPMX_DSCP_AUDIO_TOS);
    ipmx_set_dscp(st->udp_rtcp_h, is_video ? IPMX_DSCP_VIDEO_TOS : IPMX_DSCP_AUDIO_TOS);

    av_strlcpy(st->dst_ip_resolved, host, sizeof(st->dst_ip_resolved));
    st->dst_port_resolved      = port;
    st->dst_rtcp_port_resolved = rtcp_port;

    /* Allocate child rtp_ctx + paced AVIO for this stream. */
    rtp_ctx = avformat_alloc_context();
    if (!rtp_ctx) return AVERROR(ENOMEM);
    rtp_ctx->oformat = &ff_rtp_muxer.p;
    rtp_ctx->max_delay = s->max_delay;
    /* Propagate PEP encryption settings (TR-10-13). The fields live on
     * AVFormatContext (declared in avformat.h alongside `privacy_encrypt`)
     * and are read by rtpenc.c at write_header time to wire the cipher
     * callbacks through ff_rtp_send_data_packet. */
    rtp_ctx->privacy_encrypt   = s->privacy_encrypt;
    rtp_ctx->privacy_scramble  = s->privacy_scramble;
    /* AVOption strings on AVFormatContext are owned by the OPT layer and
     * av_freed on context destroy, so we must hand the child its own
     * av_strdup'd copies (otherwise libavformat double-frees on close). */
    rtp_ctx->pep_psk           = s->pep_psk           ? av_strdup(s->pep_psk)           : NULL;
    rtp_ctx->pep_mode          = s->pep_mode          ? av_strdup(s->pep_mode)          : NULL;
    /* Use the resolved hex strings (so the SDP a=privacy line matches
     * exactly what the child rtp_ctx feeds into rtp_pep_init). */
    rtp_ctx->pep_iv            = s->privacy_encrypt ? av_strdup(ctx->pep_iv_hex)
                                                    : (s->pep_iv ? av_strdup(s->pep_iv) : NULL);
    rtp_ctx->pep_key_generator = s->privacy_encrypt ? av_strdup(ctx->pep_key_generator_hex)
                                                    : (s->pep_key_generator ? av_strdup(s->pep_key_generator) : NULL);
    rtp_ctx->pep_key_version   = s->privacy_encrypt ? av_strdup(ctx->pep_key_version_hex)
                                                    : (s->pep_key_version ? av_strdup(s->pep_key_version) : NULL);
    rtp_ctx->pep_rtp_kv        = s->pep_rtp_kv;
    rtp_ctx->pep_kv_rotate     = s->pep_kv_rotate;
    rtp_st = avformat_new_stream(rtp_ctx, NULL);
    if (!rtp_st) { ret = AVERROR(ENOMEM); goto fail; }
    rtp_st->time_base.num = 1;
    rtp_st->time_base.den = is_video ? 90000 : st->audio_sample_rate;
    avcodec_parameters_copy(rtp_st->codecpar, src_st->codecpar);
    rtp_st->avg_frame_rate = src_st->avg_frame_rate;
    /* Preserve the source codec_id (RAWVIDEO or BITPACKED) on the rtp child.
     * Both arms call ff_rtp_send_raw_rfc4175() in rtpenc.c, so the codec_id
     * label has no effect on the wire-format path itself. The label DOES
     * matter for the rfc4175 validation gate (rtpenc.c rt_write_header):
     * memory-packed pix_fmts (UYVY422, RGB24) are valid under either
     * codec_id, but planar pix_fmts (YUV420P, YUV422P, YUV444P, YUV422P10,
     * YUV420P10) require BITPACKED so the bitpacked encoder did the
     * planar→wire packing. Forcing BITPACKED here would mask the case
     * where a user picks `-c:v rawvideo` with a planar pix_fmt and silently
     * ship the planar bytes onto the wire mislabelled as RFC 4175 — which
     * is exactly the bug class the validation gate exists to catch. */

    st->avio_buf = av_mallocz(IPMX_AVIO_BUF_SZ);
    if (!st->avio_buf) { ret = AVERROR(ENOMEM); goto fail; }
    st->paced_avio = avio_alloc_context(st->avio_buf, IPMX_AVIO_BUF_SZ, 1,
                                        st, NULL, ipmx_avio_write, NULL);
    if (!st->paced_avio) { ret = AVERROR(ENOMEM); goto fail; }
    /* For audio, constrain max_packet_size so ff_rtp_send_samples emits
     * exactly one ptime worth of audio per RTP packet (TR-10-1 §8.10.1
     * + AES67 §7.5). Without this, the rtp muxer packs as many samples
     * as fit in MTU (~5 ms at 48 kHz/24 b/2 ch over 1500 B), which would
     * misalign the SR cadence and inflate audio latency.
     *
     * When PEP encryption is on, rtpenc.c reserves 20 bytes (RTP_ExtHdrFull)
     * + privacy_mac_size from max_payload_size for the RFC 8285 ext header.
     * If we don't expand max_packet_size accordingly, ff_rtp_send_samples
     * fragments the audio frame into two RTP packets per ptime, doubling
     * the wire packet rate and breaking the SR cadence. Reserve the same
     * upper bound as rtpenc.c so the budget after subtraction equals the
     * un-encrypted budget. */
    if (is_video) {
        st->paced_avio->max_packet_size = ctx->video_pkt_size;
    } else {
        int extra = 0;
        if (s->privacy_encrypt) {
            /* sizeof(struct RTP_ExtHdrFull) = 20 bytes (rtp.h). The MAC tag
             * is 8 bytes for AES-128-CTR_CMAC-64 modes; add unconditionally
             * to keep the calculation independent of mode at setup time. */
            extra = 20 + 8;
        }
        st->paced_avio->max_packet_size = st->audio_payload_bytes + 12 + extra;
    }
    rtp_ctx->pb = st->paced_avio;

    st->pkt_buf_capacity = 65536;
    st->pkt_buf = av_malloc(st->pkt_buf_capacity);
    if (!st->pkt_buf) { ret = AVERROR(ENOMEM); goto fail; }

    /* Initialise child rtp_ctx and seed its base_timestamp with our
     * Internal-Clock-derived value (TR-10-1 §8.6). */
    {
        AVDictionary *opts = NULL;
        char ptbuf[16], szbuf[16], ssrcbuf[24];
        int media_rate   = is_video ? 90000 : st->audio_sample_rate;
        st->payload_type = payload_type;
        int64_t now_ns = ipmx_clock_now_ns(ctx);
        uint64_t now_sec = (uint64_t)(now_ns / 1000000000LL);
        uint64_t now_ns_sub = (uint64_t)(now_ns % 1000000000LL);
        uint64_t ticks = (now_sec * (uint64_t)media_rate)
                       + (now_ns_sub * (uint64_t)media_rate) / 1000000000ULL;
        uint32_t initial_ts = (uint32_t)(ticks & 0xffffffffu);
        uint32_t initial_ssrc = av_get_random_seed();

        snprintf(ptbuf,   sizeof(ptbuf),   "%d", payload_type);
        snprintf(szbuf,   sizeof(szbuf),   "%d",
                 is_video ? ctx->video_pkt_size : (st->audio_payload_bytes + 12));
        snprintf(ssrcbuf, sizeof(ssrcbuf), "%u", initial_ssrc);
        av_dict_set(&opts, "payload_type", ptbuf,   0);
        av_dict_set(&opts, "pkt_size",     szbuf,   0);
        av_dict_set(&opts, "rtpflags",     "+skip_rtcp", 0);
        av_dict_set(&opts, "ssrc",         ssrcbuf, 0);
        av_dict_set(&opts, "seq",          "0",     0);

        ret = avformat_write_header(rtp_ctx, &opts);
        av_dict_free(&opts);
        if (ret < 0) {
            av_log(s, AV_LOG_ERROR, "Stream %d: child rtp_ctx write_header: %d\n", idx, ret);
            goto fail;
        }
        {
            RTPMuxContext *rmx = (RTPMuxContext *)rtp_ctx->priv_data;
            rmx->base_timestamp = initial_ts;
            rmx->timestamp      = initial_ts;
            rmx->cur_timestamp  = 0;
        }
        st->rtp_ts_base = initial_ts;
        st->ssrc        = initial_ssrc;
        st->internal_clock_at_epoch_ns = now_ns;
    }

    st->child_rtp_ctx = rtp_ctx;
    return 0;

fail:
    if (rtp_ctx) avformat_free_context(rtp_ctx);
    if (st->paced_avio) {
        av_freep(&st->paced_avio->buffer);
        avio_context_free(&st->paced_avio);
    }
    av_freep(&st->pkt_buf);
    av_freep(&st->hrd_trace);
    if (st->hrd_au0_buf) {
        int i;
        for (i = 0; i < st->hrd_au0_buf_count; i++)
            av_freep(&st->hrd_au0_buf[i].data);
        av_freep(&st->hrd_au0_buf);
        st->hrd_au0_buf_count = 0;
        st->hrd_au0_buf_cap = 0;
    }
    if (st->udp_h)      ffurl_close(st->udp_h);
    if (st->udp_rtcp_h) ffurl_close(st->udp_rtcp_h);
    return ret;
}

/* ─────────────── write_header ─────────────── */

static int ipmx_write_header(AVFormatContext *s)
{
    IpmxMuxContext *ctx = s->priv_data;
    char default_mac[18];
    int ret, i;
    char proto[16], host[64], path2[64];
    int  base_port = 0;

    if (s->nb_streams < 1 || s->nb_streams > IPMX_MAX_STREAMS) {
        av_log(s, AV_LOG_ERROR, "IPMX muxer: %d streams (must be 1..%d)\n",
               s->nb_streams, IPMX_MAX_STREAMS);
        return AVERROR(EINVAL);
    }

    ipmx_clock_anchor_init(ctx);

    if (!ctx->sender_mac) {
        ipmx_default_sender_mac(default_mac);
        ctx->sender_mac = av_strdup(default_mac);
    }
    if (!ctx->mediaclk) ctx->mediaclk = av_strdup("sender");
    if (ctx->pace_beta < 1.0 || ctx->pace_beta > 1.10) ctx->pace_beta = IPMX_DEFAULT_BETA;
    /* Mirror parent privacy flag for the SR-builder hot path. */
    ctx->privacy_encrypt = s->privacy_encrypt;
    if (s->privacy_encrypt) {
        /* Resolve PEP IV / key_generator / key_version / key_id once per
         * muxer so the SDP a=privacy line exactly matches the values that
         * rtp_pep_init writes into the child rtp_ctx. User-provided hex
         * strings (via -pep_iv / -pep_key_generator / -pep_key_version)
         * win; otherwise we generate random values here and propagate them
         * via av_dict_set when opening each child rtp_ctx. */
        uint8_t buf[16];
        int j;
        if (s->pep_iv && s->pep_iv[0])
            av_strlcpy(ctx->pep_iv_hex, s->pep_iv, sizeof(ctx->pep_iv_hex));
        else {
            for (j = 0; j < 8; j++) buf[j] = (uint8_t)(av_get_random_seed() & 0xff);
            for (j = 0; j < 8; j++) snprintf(ctx->pep_iv_hex + j*2, 3, "%02x", buf[j]);
        }
        if (s->pep_key_generator && s->pep_key_generator[0])
            av_strlcpy(ctx->pep_key_generator_hex, s->pep_key_generator,
                       sizeof(ctx->pep_key_generator_hex));
        else {
            for (j = 0; j < 16; j++) buf[j] = (uint8_t)(av_get_random_seed() & 0xff);
            for (j = 0; j < 16; j++) snprintf(ctx->pep_key_generator_hex + j*2, 3, "%02x", buf[j]);
        }
        if (s->pep_key_version && s->pep_key_version[0])
            av_strlcpy(ctx->pep_key_version_hex, s->pep_key_version,
                       sizeof(ctx->pep_key_version_hex));
        else {
            for (j = 0; j < 4; j++) buf[j] = (uint8_t)(av_get_random_seed() & 0xff);
            for (j = 0; j < 4; j++) snprintf(ctx->pep_key_version_hex + j*2, 3, "%02x", buf[j]);
            /* key_version=0 is reserved (rtp_pep.c:48) — force non-zero. */
            if (!strcmp(ctx->pep_key_version_hex, "00000000"))
                av_strlcpy(ctx->pep_key_version_hex, "00000001",
                           sizeof(ctx->pep_key_version_hex));
        }
        /* key_id is informational only; emit a fixed sequential value. */
        av_strlcpy(ctx->pep_key_id_hex, "0001020304050607",
                   sizeof(ctx->pep_key_id_hex));
    }

    if (!s->url || !s->url[0]) {
        av_log(s, AV_LOG_ERROR, "IPMX muxer needs a destination URL\n");
        return AVERROR(EINVAL);
    }
    av_url_split(proto, sizeof(proto), NULL, 0, host, sizeof(host),
                 &base_port, path2, sizeof(path2), s->url);
    if (base_port <= 0) base_port = 5000;

    /* Open PCAP if requested (single shared file across all streams). */
    if (ctx->pcap_file && ctx->pcap_file[0]) {
        ctx->pcap_fp = fopen(ctx->pcap_file, "wb");
        if (!ctx->pcap_fp) {
            av_log(s, AV_LOG_WARNING, "Cannot open PCAP '%s'\n", ctx->pcap_file);
        } else {
            ipmx_pcap_write_header(ctx->pcap_fp);
            av_log(s, AV_LOG_INFO, "IPMX PCAP capture → %s\n", ctx->pcap_file);
        }
    }

    /* Set up each stream. Video gets base_port (RTP) / +1 (RTCP).
     * Audio streams use audio_base_port + 2*(i-1) / ...+1. */
    {
        int audio_idx = 0;
        for (i = 0; i < s->nb_streams; i++) {
            AVStream *src_st = s->streams[i];
            int port_for_stream;
            int pt_for_stream;
            if (src_st->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
                port_for_stream = base_port;
                pt_for_stream   = ctx->video_payload_type;
            } else if (src_st->codecpar->codec_type == AVMEDIA_TYPE_AUDIO) {
                int abase = ctx->audio_base_port > 0 ? ctx->audio_base_port : (base_port + 6);
                port_for_stream = abase + 2 * audio_idx;
                /* Distinct dynamic PT per audio m= block. Without this, all
                 * audio streams would advertise the same PT and an RFC 3551
                 * receiver's fmtp-by-PT lookup would conflate parameters
                 * across them — e.g. lavf's parse_fmtp dispatches the line
                 * to every stream sharing the PT, which re-runs per-signal
                 * stream creation inside rtpdec_am824. */
                pt_for_stream   = ctx->audio_payload_type + audio_idx;
                if (pt_for_stream > 127) {
                    av_log(s, AV_LOG_ERROR,
                           "Too many audio streams: PT %d would exceed the "
                           "dynamic range [96..127]. audio_payload_type=%d, "
                           "audio_idx=%d\n",
                           pt_for_stream, ctx->audio_payload_type, audio_idx);
                    return AVERROR(EINVAL);
                }
                if (pt_for_stream == ctx->video_payload_type) {
                    av_log(s, AV_LOG_ERROR,
                           "Audio PT %d collides with video_payload_type. "
                           "Move video_payload_type out of the audio range "
                           "or lower audio_payload_type.\n", pt_for_stream);
                    return AVERROR(EINVAL);
                }
                audio_idx++;
            } else {
                av_log(s, AV_LOG_ERROR, "Stream %d: unsupported media type\n", i);
                return AVERROR(EINVAL);
            }
            if ((ret = ipmx_setup_stream(s, &ctx->streams[i], src_st, i,
                                         host, port_for_stream,
                                         pt_for_stream)) < 0)
                return ret;
            ctx->nb_streams++;
        }
    }

    /* Start the single shared pacer thread AFTER all streams are set up so
     * the pacer's startup gate sees the final nb_streams count and can
     * track first-packet arrival on every stream. All RTP packets and RTCP
     * SRs across all streams will be enqueued onto per-stream rings and
     * dispatched here in target-time order. */
    if ((ret = ipmx_pacer_init(ctx)) < 0) {
        av_log(s, AV_LOG_ERROR, "Cannot start IPMX pacer thread: %d\n", ret);
        return ret;
    }

    ipmx_write_sdp_file(ctx, s);
    return 0;
}

/* ─────────────── write_packet ─────────────── */

static int ipmx_write_packet(AVFormatContext *s, AVPacket *pkt)
{
    IpmxMuxContext *ctx = s->priv_data;
    IpmxStreamRT *st;
    AVPacket local;
    int ret;

    if (pkt->stream_index < 0 || pkt->stream_index >= ctx->nb_streams)
        return AVERROR(EINVAL);
    st = &ctx->streams[pkt->stream_index];
    if (!st->child_rtp_ctx) return AVERROR(EINVAL);

    /* One AVPacket is one access unit. Read it here, in the clear, before the
     * child RTP muxer packetises (and, with PEP / HKEP, encrypts) it: its NAL
     * unit byte count, in-band parameter sets and HRD SEI, from which its pace
     * is capped at the HRD ceiling. The packetizer emits all of its packets
     * (marker on the last) inside the av_write_frame() call below, so the
     * per-packet path and the marker handler see this AU's state. */
    if (st->is_compressed_video) {
        const RTPMuxContext *rmx = st->child_rtp_ctx->priv_data;
        IpmxAuNalCtx cx = {
            .st      = st,
            .is_h265 = st->compressed_codec_id == AV_CODEC_ID_HEVC,
            .log_ctx = s,
        };
        ipmx_walk_au_nal_units(pkt->data, pkt->size, rmx->nal_length_size,
                               ipmx_au_nal_cb, &cx);
        st->comp_au_nal_unit_bytes = cx.nal_unit_bytes;
        ipmx_hrd_apply_ceiling(st, s);
    }

    av_packet_ref(&local, pkt);
    local.stream_index = 0;
    if (pkt->pts != AV_NOPTS_VALUE)
        local.pts = av_rescale_q(pkt->pts,
                                 s->streams[pkt->stream_index]->time_base,
                                 st->child_rtp_ctx->streams[0]->time_base);
    if (pkt->dts != AV_NOPTS_VALUE)
        local.dts = av_rescale_q(pkt->dts,
                                 s->streams[pkt->stream_index]->time_base,
                                 st->child_rtp_ctx->streams[0]->time_base);

    ret = av_write_frame(st->child_rtp_ctx, &local);
    av_packet_unref(&local);
    return ret;
}

/* ─────────────── write_trailer ─────────────── */

static int ipmx_write_trailer(AVFormatContext *s)
{
    IpmxMuxContext *ctx = s->priv_data;
    int i;

    /* Drain producers (child rtp muxers) so any in-flight packets enter the
     * pacer queue, then stop the pacer (which drains its remaining heap). */
    for (i = 0; i < ctx->nb_streams; i++) {
        IpmxStreamRT *st = &ctx->streams[i];
        if (st->child_rtp_ctx) {
            av_write_trailer(st->child_rtp_ctx);
            avformat_free_context(st->child_rtp_ctx);
            st->child_rtp_ctx = NULL;
        }
    }
    ipmx_pacer_close(ctx);

    /* Per-stream HRD summary (compressed video only) — confirms SEI walker
     * was active and reports the final t_r,n / loop-anchor state. */
    for (i = 0; i < ctx->nb_streams; i++) {
        IpmxStreamRT *st = &ctx->streams[i];
        if (!st->is_compressed_video || !st->hrd_present) continue;
        av_log(s, AV_LOG_INFO,
               "IPMX HRD summary stream=%d: %s cbr_flag=%d AUs=%d "
               "final_t_r_n=%lld µs anchor_offset=%lld µs init_delay_first=%lld µs "
               "bp_sei=%u pt_sei=%u\n",
               i, st->compressed_codec_id == AV_CODEC_ID_H264 ? "H.264" : "H.265",
               st->hrd_cbr_flag, st->hrd_au_idx,
               (long long)st->hrd_t_r_n_j_us,
               (long long)st->hrd_anchor_offset_us,
               (long long)st->hrd_init_delay_first_us,
               st->hrd_bp_sei_count, st->hrd_pt_sei_count);
        if (st->hrd_au_idx > 0 && (!st->hrd_bp_sei_count || !st->hrd_pt_sei_count))
            av_log(s, AV_LOG_WARNING,
                   "IPMX HRD stream=%d: the SPS declares HRD parameters but no %s SEI "
                   "was parsed; HRD timing (t_r,n, ceiling, diagnostics) was not applied\n",
                   i, !st->hrd_bp_sei_count ? "buffering_period" : "pic_timing");
    }

    /* Per-stream producer back-pressure summary. Logs only when the
     * producer (ffmpeg's main mux loop) had to spin on a full ring slot —
     * which is the signature of MPSC ring overflow and the most likely
     * cause of ffmpeg's "Resumed reading at pts X after a lag of Y"
     * warning, since the back-pressure starves the demux thread's `-re`
     * burst budget. */
    for (i = 0; i < ctx->nb_streams; i++) {
        IpmxStreamRT *st = &ctx->streams[i];
        if (st->ring_push_blocked_calls == 0 && st->sr_ring_push_blocked_calls == 0)
            continue;
        av_log(s, AV_LOG_WARNING,
               "IPMX ring-push back-pressure stream=%d (%s): RTP ring "
               "blocked %llu calls, %lld µs total, %lld µs max-call, "
               "%llu spin iters; SR ring blocked %llu calls, %lld µs total\n",
               i,
               st->is_compressed_video ? "compressed video"
                 : (st->is_audio ? "audio" : "raw video"),
               (unsigned long long)st->ring_push_blocked_calls,
               (long long)st->ring_push_blocked_us,
               (long long)st->ring_push_blocked_max_us,
               (unsigned long long)st->ring_push_spin_iters,
               (unsigned long long)st->sr_ring_push_blocked_calls,
               (long long)st->sr_ring_push_blocked_us);
    }

    /* Per-stream staleness-drop summary. Logs only when drops actually
     * happened — silent on healthy runs (which are the vast majority).
     * The per-event detail is in the trace ring (kind = DROP_RTP /
     * DROP_SR) when -ipmx_pacer_trace_cap was set. */
    if (ctx->drop_late_us > 0) {
        uint64_t total_rtp = 0, total_sr = 0;
        for (i = 0; i < ctx->nb_streams; i++) {
            total_rtp += ctx->streams[i].dropped_late_rtp;
            total_sr  += ctx->streams[i].dropped_late_sr;
        }
        if (total_rtp || total_sr) {
            av_log(s, AV_LOG_INFO,
                   "IPMX pacer dropped %"PRIu64" stale RTP packet(s) and "
                   "%"PRIu64" stale RTCP SR(s) (drop_late_us=%d):\n",
                   total_rtp, total_sr, ctx->drop_late_us);
            for (i = 0; i < ctx->nb_streams; i++) {
                IpmxStreamRT *st = &ctx->streams[i];
                if (st->dropped_late_rtp || st->dropped_late_sr) {
                    av_log(s, AV_LOG_INFO,
                           "  stream %d: %"PRIu64" RTP, %"PRIu64" SR\n",
                           st->src_stream_index,
                           st->dropped_late_rtp, st->dropped_late_sr);
                }
            }
        }
    }

    /* Dump in-memory pacer and HRD traces (if enabled) before freeing. */
    ipmx_pacer_dump_trace(ctx, s);
    av_freep(&ctx->pacer.trace);
    ipmx_hrd_dump_trace(ctx, s);

    for (i = 0; i < ctx->nb_streams; i++) {
        IpmxStreamRT *st = &ctx->streams[i];
        if (st->paced_avio) {
            av_freep(&st->paced_avio->buffer);
            avio_context_free(&st->paced_avio);
        }
        av_freep(&st->pkt_buf);
        av_freep(&st->sps_data);
        av_freep(&st->pps_data);
        av_freep(&st->vps_data);
        av_freep(&st->hrd_trace);
        if (st->hrd_au0_buf) {
            int k;
            for (k = 0; k < st->hrd_au0_buf_count; k++)
                av_freep(&st->hrd_au0_buf[k].data);
            av_freep(&st->hrd_au0_buf);
            st->hrd_au0_buf_count = 0;
            st->hrd_au0_buf_cap = 0;
        }
        ipmx_ring_close(&st->ring);
        if (st->sr_ring.cap) ipmx_ring_close(&st->sr_ring);
        if (st->udp_h)      { ffurl_close(st->udp_h);      st->udp_h = NULL; }
        if (st->udp_rtcp_h) { ffurl_close(st->udp_rtcp_h); st->udp_rtcp_h = NULL; }
    }
    if (ctx->pcap_fp) { fclose(ctx->pcap_fp); ctx->pcap_fp = NULL; }
    return 0;
}

/* ─────────────── AVOptions ─────────────── */

#define OFFSET(x) offsetof(IpmxMuxContext, x)
#define E AV_OPT_FLAG_ENCODING_PARAM

static const AVOption options[] = {
    { "ipmx_refclk",         "ts-refclk attribute body (e.g. \"localmac=AA-BB-CC-DD-EE-FF\" or \"ptp=...\")",
      OFFSET(refclk),        AV_OPT_TYPE_STRING, { .str = NULL }, 0, 0, E },
    { "ipmx_mediaclk",       "mediaclk attribute body (e.g. \"sender\" or \"direct=0\")",
      OFFSET(mediaclk),      AV_OPT_TYPE_STRING, { .str = NULL }, 0, 0, E },
    { "cname",               "CNAME for the RTCP SDES item (default: ipmx@<src-ip>)",
      OFFSET(cname),         AV_OPT_TYPE_STRING, { .str = NULL }, 0, 0, E },
    { "ipmx_pace_beta",      "ST 2110-21 §7.1 β factor (1.00..1.10)",
      OFFSET(pace_beta),     AV_OPT_TYPE_DOUBLE, { .dbl = IPMX_DEFAULT_BETA }, 1.00, 1.10, E },
    { "ipmx_au_offset0_mode",
      "Initial AU offset (TR-10-15c §15 AU_offset_0): 0=zero (min latency), "
      "1=symmetric (cpb_size/max_bitrate - init_delay/90000) — default, "
      "matches the canonical encoder/decoder bucket model both x264 and "
      "x265 declare in their BP SEIs; "
      "2=max (cpb_size/max_bitrate, max receiver pre-buffer); "
      "3=offset (use BP SEI initial_cpb_removal_delay_offset verbatim — "
      "for non-x264/x265 encoders where the offset field is computed "
      "independently of cpb_size, e.g. low_delay_hrd_flag=1 sources)",
      OFFSET(au_offset0_mode), AV_OPT_TYPE_INT, { .i64 = IPMX_AU_OFFSET0_SYMMETRIC },
      IPMX_AU_OFFSET0_ZERO, IPMX_AU_OFFSET0_OFFSET, E, .unit = "au_offset0_mode" },
    { "zero",      "AU_offset_0 = 0 (min latency)",
      0, AV_OPT_TYPE_CONST, { .i64 = IPMX_AU_OFFSET0_ZERO },
      0, 0, E, .unit = "au_offset0_mode" },
    { "symmetric", "AU_offset_0 = cpb_size/max_bitrate - init_delay/90000 (symmetric bucket model)",
      0, AV_OPT_TYPE_CONST, { .i64 = IPMX_AU_OFFSET0_SYMMETRIC },
      0, 0, E, .unit = "au_offset0_mode" },
    { "max",       "AU_offset_0 = cpb_size/max_bitrate (TR-10-15c §15 upper bound)",
      0, AV_OPT_TYPE_CONST, { .i64 = IPMX_AU_OFFSET0_MAX },
      0, 0, E, .unit = "au_offset0_mode" },
    { "offset",    "AU_offset_0 = initial_cpb_removal_delay_offset/90000 (encoder-declared offset; for non-x264/x265 encoders)",
      0, AV_OPT_TYPE_CONST, { .i64 = IPMX_AU_OFFSET0_OFFSET },
      0, 0, E, .unit = "au_offset0_mode" },
    { "ipmx_pacer_spin_us",  "busy-spin tail threshold (µs); also the outer "
      "preempt-sleep duration (the dispatch loop uses av_usleep(pacer_spin_us) "
      "for any wait larger than pacer_spin_us, then re-runs find_two_earliest).",
      OFFSET(pacer_spin_us), AV_OPT_TYPE_INT, { .i64 = 200 }, 0, 100000, E },
    { "ipmx_pacer_imminent_us",
      "Late-burst fairness threshold (µs). When the earliest pending packet is late, "
      "an imminent packet from another stream within this window of \"now\" is dispatched "
      "back-to-back to avoid late-burst monopolising the pacer (0 = disabled).",
      OFFSET(pacer_imminent_us), AV_OPT_TYPE_INT, { .i64 = 20 }, 0, 1000, E },
    { "ipmx_pacer_gate_us",
      "Startup gate window (µs). Pacer holds dispatch until every stream has "
      "its first packet enqueued OR this many µs elapsed; on release, all "
      "streams' targets are anchored to a common wall-clock reference "
      "(eliminates the startup burst from independent per-stream epochs). "
      "0 = disabled.",
      OFFSET(pacer_gate_us), AV_OPT_TYPE_INT, { .i64 = 50000 }, 0, 1000000, E },
    { "ipmx_drop_late_us",
      "Staleness drop gate (µs). When > 0, the pacer drops any queued RTP "
      "packet or RTCP SR whose effective target is older than (now - this) "
      "instead of dispatching it. Defensive backstop for fault scenarios "
      "(kernel preempt, sendto stall, malformed input) — without it, a "
      "200 ms producer stall snowballs into a catch-up burst that wastes "
      "wire bandwidth and mis-aligns receiver A/V via stale RTCP SRs. "
      "Recommended when enabled: audio AES67 §7.5 SHALL bound "
      "(min(17·ptime, 17 ms) → 17000 µs); video one frame period "
      "(33333 µs at 30 fps). Drops counted per-stream and logged on close. "
      "0 (default) = disabled — every queued packet is dispatched.",
      OFFSET(drop_late_us), AV_OPT_TYPE_INT, { .i64 = 0 }, 0, 10000000, E },
    { "ipmx_video_pt",       "Dynamic RTP payload type for video",
      OFFSET(video_payload_type), AV_OPT_TYPE_INT, { .i64 = 96 }, 96, 127, E },
    { "ipmx_video_pkt_size", "RTP MTU payload size for video (bytes)",
      OFFSET(video_pkt_size),     AV_OPT_TYPE_INT, { .i64 = 1400 }, 200, 8000, E },
    { "ipmx_audio_pt",       "Dynamic RTP payload type for audio",
      OFFSET(audio_payload_type), AV_OPT_TYPE_INT, { .i64 = 97 }, 96, 127, E },
    { "ipmx_audio_ptime_us", "Audio nominal packet time (µs); SR cadence = floor(10000/this)",
      OFFSET(audio_ptime_us),     AV_OPT_TYPE_INT, { .i64 = 1000 }, 83, 4000, E },
    { "ipmx_audio_base_port","Base RTP port for audio streams (default video_port+6); each audio uses +2 from previous",
      OFFSET(audio_base_port),    AV_OPT_TYPE_INT, { .i64 = 0 }, 0, 65535, E },
    { "ipmx_pep_extmap_full",  "RFC 8285 one-byte ext-map id for PEP-Full-IV-Counter (TR-10-13)",
      OFFSET(pep_extmap_id_full),  AV_OPT_TYPE_INT, { .i64 = 1 }, 1, 14, E },
    { "ipmx_pep_extmap_short", "RFC 8285 one-byte ext-map id for PEP-Short-IV-Counter (TR-10-13)",
      OFFSET(pep_extmap_id_short), AV_OPT_TYPE_INT, { .i64 = 2 }, 1, 14, E },
    { "ipmx_pacer_video_lookahead_au",
      "Pacer drain window for video, in AUs (frames). Heap holds K_video × T_frame_us of timeline ahead.",
      OFFSET(k_video),       AV_OPT_TYPE_INT, { .i64 = IPMX_DEFAULT_K_VIDEO }, 1, 16, E },
    { "ipmx_pacer_audio_lookahead_pkts",
      "Pacer drain window for audio, in packets. Heap holds K_audio × ptime_us of timeline ahead.",
      OFFSET(k_audio),       AV_OPT_TYPE_INT, { .i64 = IPMX_DEFAULT_K_AUDIO }, 1, 64, E },
    { "ipmx_pacer_idle_poll_us",
      "Pacer idle-poll cap (µs); only used when all rings empty.",
      OFFSET(idle_poll_us),  AV_OPT_TYPE_INT, { .i64 = IPMX_PACER_IDLE_POLL_US }, 1, 100000, E },
    { "ipmx_pacer_trace_cap",
      "In-memory pacer trace ring capacity (0 = disabled). Power-of-two rounded up; ring is a sliding window.",
      OFFSET(pacer_trace_cap), AV_OPT_TYPE_INT, { .i64 = 0 }, 0, 1<<20, E },
    { "ipmx_pacer_trace_file",
      "File to dump the pacer trace ring to on muxer close (NULL = stderr)",
      OFFSET(pacer_trace_file), AV_OPT_TYPE_STRING, { .str = NULL }, 0, 0, E },
    { "ipmx_hrd_trace_cap",
      "Per-stream HRD pacing trace capacity in access units (0 = disabled); ring keeps the latest entries.",
      OFFSET(hrd_trace_cap), AV_OPT_TYPE_INT, { .i64 = 0 }, 0, 1<<22, E },
    { "ipmx_hrd_trace_file",
      "File to dump the HRD pacing trace to on muxer close (NULL = stderr)",
      OFFSET(hrd_trace_file), AV_OPT_TYPE_STRING, { .str = NULL }, 0, 0, E },
    { "ipmx_sdp_file",       "Path to write generated IPMX SDP",
      OFFSET(sdp_file),      AV_OPT_TYPE_STRING, { .str = NULL }, 0, 0, E },
    { "ipmx_pcap_file",      "Path to write captured PCAP (DLT_EN10MB) of all RTP+RTCP packets",
      OFFSET(pcap_file),     AV_OPT_TYPE_STRING, { .str = NULL }, 0, 0, E },
    { "ipmx_sender_mac",     "Sender MAC for ts-refclk:localmac (default synthesised)",
      OFFSET(sender_mac),    AV_OPT_TYPE_STRING, { .str = NULL }, 0, 0, E },
    { "ipmx_src_ip",         "Source IPv4 address embedded in synthesised PCAP IP header",
      OFFSET(src_ip),        AV_OPT_TYPE_STRING, { .str = "127.0.0.1" }, 0, 0, E },
    { "ipmx_src_port",       "Base source UDP port for PCAP synthesis; +stream_index per stream",
      OFFSET(src_port_base), AV_OPT_TYPE_INT, { .i64 = 50000 }, 1, 65535, E },
    { NULL }
};

static const AVClass ipmx_muxer_class = {
    .class_name = "IPMX muxer",
    .item_name  = av_default_item_name,
    .option     = options,
    .version    = LIBAVUTIL_VERSION_INT,
};

const FFOutputFormat ff_ipmx_muxer = {
    .p.name            = "ipmx",
    .p.long_name       = NULL_IF_CONFIG_SMALL("IPMX (TR-10) RTP+RTCP streaming"),
    .priv_data_size    = sizeof(IpmxMuxContext),
    .p.audio_codec     = AV_CODEC_ID_PCM_S24BE,
    .p.video_codec     = AV_CODEC_ID_BITPACKED,
    .p.flags           = AVFMT_NOFILE,
    .write_header      = ipmx_write_header,
    .write_packet      = ipmx_write_packet,
    .write_trailer     = ipmx_write_trailer,
    .p.priv_class      = &ipmx_muxer_class,
};
