/*
 * IPMX pipeline timing instrumentation.
 *
 * Activated at runtime by setting IPMX_TIMING=1.  When disabled, all
 * functions are inlined no-ops with zero overhead.
 *
 * Timestamps use av_gettime_relative() (CLOCK_MONOTONIC, microseconds),
 * comparable across processes on the same host.
 *
 * Copyright (C) 2026 Matrox Graphics Inc.
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
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef IPMX_TIMING_H
#define IPMX_TIMING_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "libavutil/time.h"
#include "libavutil/buffer.h"
#include "libavcodec/packet.h"

/* Layout stored in AVPacket.opaque_ref.  First field matches ffplay's
 * FrameData so both coexist without conflict.
 *
 * NOTE: opaque_ref is destroyed by fftools (ffmpeg_demux.c replaces it
 * with DemuxPktData).  Use IpmxUsbTimingSideData (below) for any data
 * that must survive the demuxer→muxer pipeline. */
typedef struct IpmxTimingOpaque {
    int64_t pkt_pos;
    int64_t wall_us;
    int64_t sender_us;     /* presentation timestamp (corrected or raw) for E2E display */
    int64_t sender_raw_us; /* raw absolute STARTFRAME * 1000 (for clock recovery filter) */
    int64_t capture_wall_us; /* wall clock at USB capture — stamped once, never
                              * overwritten.  On same-machine runs this shares
                              * CLOCK_MONOTONIC with the receiver, giving exact
                              * pipeline latency without clock offset estimation. */
    uint32_t scr_stc;      /* receiver-side: SCR STC from decoded UVC frames */
    uint16_t scr_sof;      /* receiver-side: SCR SOF from decoded UVC frames */
    uint8_t  has_scr;      /* non-zero if scr_stc/scr_sof are valid */
} IpmxTimingOpaque;

/*
 * Per-packet USB device timing, attached as AVPacket side_data
 * (AV_PKT_DATA_IPMX_USB_TIMING).  Unlike opaque_ref, side_data is
 * preserved through the fftools pipeline (streamcopy, scheduler, etc.).
 *
 * Written by ipmx_usb_local (capture), read by ipmx_usb_sender.
 */
typedef struct IpmxUsbTimingSideData {
    uint32_t device_pts;       /* raw UVC PTS (device clock ticks, 32-bit) */
    uint32_t scr_stc;          /* UVC SCR: Source Time Clock (device clock ticks) */
    uint16_t scr_sof;          /* UVC SCR: 11-bit USB SOF frame number */
    uint32_t frame_number;     /* USB frame number (1 ms ticks from capture epoch) */
    uint32_t clock_frequency;  /* dwClockFrequency from device descriptors (Hz) */
    int64_t  capture_wall_us;  /* CLOCK_MONOTONIC at USB capture (µs).
                                * Survives fftools pipeline via side_data. */
    uint8_t  has_device_pts;
    uint8_t  has_scr;
} IpmxUsbTimingSideData;

static inline int ipmx_timing_enabled(void)
{
    static int cached = -1;
    if (cached < 0) {
        const char *env = getenv("IPMX_TIMING");
        cached = (env && env[0] == '1');
    }
    return cached;
}

static inline FILE *ipmx_timing_open(const char *path)
{
    FILE *fp;
    if (!ipmx_timing_enabled())
        return NULL;
    fp = fopen(path, "w");
    if (fp)
        setvbuf(fp, NULL, _IOLBF, 0);
    return fp;
}

/* Unconditionally stamp the current wall-clock time into pkt->opaque_ref.
 * Merges into existing IpmxTimingOpaque when present. */
static inline void ipmx_timing_force_stamp_pkt(AVPacket *pkt)
{
    if (pkt->opaque_ref &&
        pkt->opaque_ref->size >= (int)sizeof(IpmxTimingOpaque)) {
        IpmxTimingOpaque *t = (IpmxTimingOpaque *)pkt->opaque_ref->data;
        t->wall_us = av_gettime_relative();
    } else {
        AVBufferRef *buf = av_buffer_allocz(sizeof(IpmxTimingOpaque));
        IpmxTimingOpaque *t;
        if (!buf)
            return;
        t = (IpmxTimingOpaque *)buf->data;
        t->pkt_pos = -1;
        t->wall_us = av_gettime_relative();
        av_buffer_unref(&pkt->opaque_ref);
        pkt->opaque_ref = buf;
    }
}

/* Stamp only when IPMX_TIMING=1 is set. */
static inline void ipmx_timing_stamp_pkt(AVPacket *pkt)
{
    if (!ipmx_timing_enabled())
        return;
    ipmx_timing_force_stamp_pkt(pkt);
}

/* Read the wall-clock timestamp from pkt->opaque_ref.  Returns 0 if absent. */
static inline int64_t ipmx_timing_read_pkt(const AVPacket *pkt)
{
    if (pkt && pkt->opaque_ref &&
        pkt->opaque_ref->size >= (int)sizeof(IpmxTimingOpaque)) {
        const IpmxTimingOpaque *t =
            (const IpmxTimingOpaque *)pkt->opaque_ref->data;
        return t->wall_us;
    }
    return 0;
}

/* Read the capture wall-clock timestamp.  Returns 0 if absent. */
static inline int64_t ipmx_timing_read_capture(const AVPacket *pkt)
{
    if (pkt && pkt->opaque_ref &&
        pkt->opaque_ref->size >= (int)sizeof(IpmxTimingOpaque)) {
        const IpmxTimingOpaque *t =
            (const IpmxTimingOpaque *)pkt->opaque_ref->data;
        return t->capture_wall_us;
    }
    return 0;
}

/* Read the absolute sender timestamp. Returns 0 if absent.
 * Prefers sender_raw_us if present (receiver-side), else sender_us. */
static inline int64_t ipmx_timing_read_sender(const AVPacket *pkt)
{
    if (pkt && pkt->opaque_ref &&
        pkt->opaque_ref->size >= (int)sizeof(IpmxTimingOpaque)) {
        const IpmxTimingOpaque *t =
            (const IpmxTimingOpaque *)pkt->opaque_ref->data;
        if (t->sender_raw_us > 0)
            return t->sender_raw_us;
        return t->sender_us;
    }
    return 0;
}

/* -----------------------------------------------------------------------
 * Side-data readers: read IpmxUsbTimingSideData from AV_PKT_DATA_IPMX_USB_TIMING.
 * These survive the fftools pipeline (unlike opaque_ref).
 * ----------------------------------------------------------------------- */

static inline const IpmxUsbTimingSideData *
ipmx_timing_get_side_data(const AVPacket *pkt)
{
    size_t sz = 0;
    const uint8_t *raw;
    if (!pkt)
        return NULL;
    raw = av_packet_get_side_data(pkt, AV_PKT_DATA_IPMX_USB_TIMING, &sz);
    if (raw && sz >= sizeof(IpmxUsbTimingSideData))
        return (const IpmxUsbTimingSideData *)raw;
    return NULL;
}

static inline uint32_t ipmx_timing_read_frame_number(const AVPacket *pkt)
{
    const IpmxUsbTimingSideData *sd = ipmx_timing_get_side_data(pkt);
    return sd ? sd->frame_number : 0;
}

static inline int ipmx_timing_read_device_pts(const AVPacket *pkt,
                                               uint32_t *out_pts)
{
    const IpmxUsbTimingSideData *sd = ipmx_timing_get_side_data(pkt);
    if (sd && sd->has_device_pts) {
        *out_pts = sd->device_pts;
        return 1;
    }
    return 0;
}

static inline int ipmx_timing_read_scr(const AVPacket *pkt,
                                        uint32_t *out_stc, uint16_t *out_sof)
{
    const IpmxUsbTimingSideData *sd = ipmx_timing_get_side_data(pkt);
    if (sd && sd->has_scr) {
        *out_stc = sd->scr_stc;
        *out_sof = sd->scr_sof;
        return 1;
    }
    return 0;
}

static inline uint32_t ipmx_timing_read_clock_frequency(const AVPacket *pkt)
{
    const IpmxUsbTimingSideData *sd = ipmx_timing_get_side_data(pkt);
    return sd ? sd->clock_frequency : 0;
}

static inline int64_t ipmx_timing_read_capture_wall(const AVPacket *pkt)
{
    const IpmxUsbTimingSideData *sd = ipmx_timing_get_side_data(pkt);
    return sd ? sd->capture_wall_us : 0;
}

/* -----------------------------------------------------------------------
 * Side-data writer: attach IpmxUsbTimingSideData to a packet.
 * ----------------------------------------------------------------------- */

static inline IpmxUsbTimingSideData *
ipmx_timing_add_side_data(AVPacket *pkt)
{
    uint8_t *raw = av_packet_new_side_data(
        pkt, AV_PKT_DATA_IPMX_USB_TIMING, sizeof(IpmxUsbTimingSideData));
    if (!raw)
        return NULL;
    memset(raw, 0, sizeof(IpmxUsbTimingSideData));
    return (IpmxUsbTimingSideData *)raw;
}

/* Read from an AVFrame (after COPY_OPAQUE propagation). */
static inline int64_t ipmx_timing_read_frame(const void *opaque_ref_data,
                                              int opaque_ref_size)
{
    if (opaque_ref_data &&
        opaque_ref_size >= (int)sizeof(IpmxTimingOpaque)) {
        const IpmxTimingOpaque *t =
            (const IpmxTimingOpaque *)opaque_ref_data;
        return t->wall_us;
    }
    return 0;
}

/*
 * Clock recovery stats shared between the receiver library and ffplay.
 * The receiver writes these fields; ffplay reads them for the status line.
 * No lock needed: single writer, single reader, all fields are atomic-width
 * on x86_64.
 */
typedef struct IpmxClockRecStats {
    double   drift_ppm;
    double   asrc_ppm;          /* drift + queue correction */
    int64_t  video_skipped;
    int64_t  video_repeated;
    int      calibrated;        /* non-zero once clock recovery is ready */
    int64_t  clock_offset_us;   /* global: add to sender_us to get receiver wall clock */
    int64_t  clock_offset_video_us; /* per-stream: min(arrival - sender_us) for video */
    int64_t  clock_offset_audio_us; /* per-stream: min(arrival - sender_us) for audio */
    int      has_video_offset;
    int      has_audio_offset;
} IpmxClockRecStats;

/*
 * Defined in ipmx_usb_clockrec.c (libavdevice).
 * ffplay reads this via the shared library symbol.
 * Named av_* to pass the libavdevice version-script export filter.
 */
extern IpmxClockRecStats av_ipmx_clockrec_stats;

#endif /* IPMX_TIMING_H */
