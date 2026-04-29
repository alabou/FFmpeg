/*
 * IPMX USB Clock Recovery and Latency Measurement
 *
 * USB 2.0-compliant clock recovery for IPMX USB over IP (TR-10-14):
 *
 *   Primary PLL: linear regression on STARTFRAME (USB frame number in ms)
 *     vs local arrival time.  Recovers the sender's USB bus frame clock
 *     rate relative to the receiver's wall clock.
 *
 *   SCR correlator (optional): when UVC Source Clock Reference is present
 *     in video payloads, correlates the camera's device clock (STC) with
 *     the USB SOF counter for full device-to-receiver clock chain recovery.
 *
 *   Full affine model: r̂(s) = â·s + b̂ recovers both frequency (â)
 *     and phase (b̂) from the regression, enabling constant-latency
 *     delivery without epoch warmup or pacer feedback PI loops.
 *
 *   Adaptive audio resampling via libswresample swr_set_compensation().
 *
 * Two receiver modes:
 *   Mode A (default): arrival-based playback, no PLL, no resampling.
 *   Mode B (--clock-recovery): full PLL + resampling + video skip/repeat.
 *
 * Direct pipeline latency measurement for same-machine benchmarking:
 *   - Computes arrival_us - sender_us for every packet.
 *   - Running statistics (mean, stddev, min, max) with sub-ms precision
 *     despite STARTFRAME ms-quantization (stderr = 289us / sqrt(N)).
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

#ifndef AVDEVICE_IPMX_USB_CLOCKREC_H
#define AVDEVICE_IPMX_USB_CLOCKREC_H

#include <stdint.h>
#include <stdio.h>
#include "libavutil/samplefmt.h"

struct AVPacket;
struct SwrContext;
struct AVFormatContext;

#define CLOCKREC_WINDOW_SIZE            512
#define CLOCKREC_CALIB_MIN_SPAN_US 2000000   /* min 2 s of real-time for full cal. */
#define CLOCKREC_MID_CALIB_SPAN_US 1000000  /* 1 s mid-cal: start pacing early */
#define CLOCKREC_LOG_INTERVAL_US    5000000   /* 5 s */

#define CLOCKREC_MAX_QUEUE_PPM        500.0
#define CLOCKREC_RATE_WINDOW_US   1000000   /* 1 s observer window */
#define CLOCKREC_RATE_EWMA_ALPHA      0.1   /* smoothing for rate mismatch */

/* Timing-based video skip/repeat thresholds (fractions of T_video).
 *
 * e = now - pts  (positive = late, negative = early)
 *
 *   e > +SKIP  * T_video   → frame is stale, skip it
 *   e < -REPEAT * T_video  → too early, repeat previous frame
 *   otherwise               → output normally (sleep remainder if early)
 *
 * The normal window is (SKIP + REPEAT) * T_video = 1.0 * T_video wide.
 * Consecutive-skip limit prevents dropping entire GOPs.
 */
#define CLOCKREC_VIDEO_SKIP_FRAC            0.75  /* skip  when e > +0.75 * T_video */
#define CLOCKREC_VIDEO_REPEAT_FRAC          0.25  /* repeat when e < -0.25 * T_video */
#define CLOCKREC_VIDEO_MAX_CONSECUTIVE_SKIP 2     /* never skip more than this in a row */

/* ASRC feedback loop — set to 0 to revert to open-loop for analysis.
 *
 * Saturating I-controller on the cumulative sample imbalance
 * (total_out - total_in).  The error in samples is scaled by
 * CLOCKREC_FB_KP (ppm per sample of imbalance) and clamped to
 * ±CLOCKREC_FB_MAX_PPM.  This gives constant correction authority
 * regardless of elapsed time — unlike a ratio-based approach whose
 * denominator grows and kills its own gain.
 *
 * With KP = 5.0 and MAX = 5.0, the linear zone covers ±1 sample.
 * Beyond that the controller saturates, driving the imbalance back
 * to zero at the maximum allowed rate.  The clamp prevents the
 * feedback from fighting the PLL during transients.
 *
 * The PLL handles short-term drift tracking; this feedback corrects
 * any long-term systematic bias (PLL estimation error, swr rounding,
 * etc.) that would otherwise accumulate in downstream audio buffers.
 */
#define CLOCKREC_ASRC_FEEDBACK       1
#define CLOCKREC_FB_WARMUP_US        5000000  /* 5 s after calibration */
#define CLOCKREC_FB_KP               5.0      /* ppm per sample of imbalance */
#define CLOCKREC_FB_MAX_PPM          5.0      /* saturation clamp (±) */

typedef struct IpmxLatencyStats {
    int64_t  count;
    double   sum;
    double   sum_sq;
    int64_t  min;
    int64_t  max;
} IpmxLatencyStats;

typedef struct IpmxClockRecovery {
    /* --- Mode flags (set from CLI options before init) --- */
    int      clock_recovery;
    int      latency_report;

    /* --- Simulated clock drift for same-machine testing --- */
    double   clock_sim_drift_ppm;   /* injected frequency offset (PPM) */
    int      clock_sim_offset_ms;   /* injected phase offset (ms) */
    int64_t  sim_epoch_us;          /* first local_us seen (for drift ramp) */

    /* --- Self-calibrated clock offset (sender vs receiver epoch) --- */
    int64_t  clock_offset_us;       /* global min(local - sender) at startup */
    int      clock_offset_set;

    /* Per-stream running minimum offsets for accurate e2e display.
     * Eliminates cross-stream contamination in split-device mode where
     * audio and video channels start at different times. */
    int64_t  stream_offset_video;
    int64_t  stream_offset_audio;
    int      has_stream_offset_video;
    int      has_stream_offset_audio;

    /* --- Audio format (set before init, needed for SwrContext) --- */
    int                  sample_rate;
    int                  channels;
    int                  bytes_per_sample;
    enum AVSampleFormat  sample_fmt;
    int                  is_pcm;         /* non-zero if raw PCM / float */

    /* --- Primary PLL: STARTFRAME (USB frame number) vs arrival time ---
     *
     * NTP/PTP-inspired minimum-delay clock filter with stream separation:
     *
     *   Stage 0 (stream selection): audio-only PLL when audio is
     *     present; video fallback when no audio.  This prevents
     *     regression bias from mixing streams with different delivery
     *     delays (split-device mode: video paced at frame rate adds
     *     ~33 ms offset vs ~100 µs for audio).
     *
     *   Stage 1 (pre-filter): reject stale sender_us values that have
     *     gone backward by > 1 second (catches protocol-layer artifacts).
     *
     *   Stage 2 (quality gate): per-stream sliding-window minimum of
     *     one-way delay.  Reject samples whose delay exceeds
     *     min + CLOCKREC_JITTER_GATE_US — keeps only the "best path"
     *     samples (NTP clock filter principle).
     *
     *   Stage 3 (regression): two-pass centered linear regression on
     *     the admitted samples for slope → raw drift.
     *
     *   Stage 4 (EWMA smoothing): exponential filter on drift_ppm to
     *     prevent step changes when the regression window rolls over.
     */
#define CLOCKREC_JITTER_GATE_US       1000  /* admit samples within 1 ms of min-delay */
#define CLOCKREC_DRIFT_EWMA_ALPHA    0.005  /* smoothing factor for drift_ppm */
#define CLOCKREC_DRIFT_SLEW_PPM_PER_S  50.0 /* max drift_ppm rate of change (ppm/s) */
#define CLOCKREC_RAMP_DURATION_US   500000  /* 500 ms ramp after calibration */
#define CLOCKREC_DRIFT_OUTLIER_PPM  50.0   /* reject regression if |raw - ewma| exceeds this */

    int64_t  sender_us[CLOCKREC_WINDOW_SIZE];
    int64_t  local_us[CLOCKREC_WINDOW_SIZE];
    int      ts_count;
    int      ts_write_idx;
    int      pll_has_audio;         /* 1 once first audio sample is seen */

    /* Per-stream sliding-window minimum-offset trackers */
#define CLOCKREC_OFFSET_WINDOW  64
    int64_t  offset_ring_a[CLOCKREC_OFFSET_WINDOW];  /* audio */
    int64_t  offset_ring_v[CLOCKREC_OFFSET_WINDOW];  /* video */
    int      offset_count_a, offset_count_v;
    int      offset_widx_a, offset_widx_v;
    int64_t  min_offset_a, min_offset_v;

    /* Drift output */
    double   drift_ppm;             /* EWMA-smoothed + slew-limited drift */
    double   drift_ppm_raw;         /* latest regression output (unsmoothed) */
    double   drift_ppm_ewma;        /* EWMA output before slew limiting */
    int64_t  drift_last_update_us;  /* wall time of last pll_update */
    int64_t  drift_slew_clamp_count; /* times slew limiter fired */
    int64_t  drift_outlier_count;   /* regression outliers rejected */

    /* --- Affine model: r̂(s) = pll_slope * s + pll_intercept ---
     * Maps sender timestamps (s) to receiver wall-clock coordinates (r̂).
     * pll_slope  = â = 1 - drift_ppm/1e6 (from EWMA-smoothed drift)
     * pll_intercept = b̂ = mean_y - â * mean_x (phase, from current means)
     * Together they give constant-latency delivery without epoch warmup. */
    double   pll_slope;             /* â — smoothed clock ratio */
    double   pll_intercept;         /* b̂ — phase offset (µs) */
    double   pll_mean_x;            /* latest regression mean(sender_us) */
    double   pll_mean_y;            /* latest regression mean(local_us) */

    /* Filter statistics */
    int64_t  last_accepted_sender_us;
    int64_t  pll_rejected;          /* stale sender_us (stage 1) */
    int64_t  pll_jitter_filtered;   /* jitter-gated samples (stage 2) */
    int64_t  pll_stream_skipped;    /* video samples skipped when audio present */
    int64_t  pll_gate_accepted;     /* jitter-gate accepted samples */
    int64_t  pll_gate_accept_margin_sum;
    int64_t  pll_gate_accept_margin_max;
    int64_t  pll_gate_reject_margin_sum;
    int64_t  pll_gate_reject_margin_min;
    int64_t  pll_gate_reject_margin_max;
    int64_t  pll_last_offset_us;
    int64_t  pll_last_min_offset_us;
    int64_t  pll_last_margin_us;
    int      pll_last_sample_audio;
    int      pll_last_sample_accepted;

    /* --- SCR correlator: UVC device clock vs USB SOF (optional) ---
     * When available, provides a second drift measurement that
     * correlates the camera's STC with the USB bus SOF counter.
     * Combined with the primary PLL, this yields a full
     * camera → sender bus → network → receiver clock chain. */
#define CLOCKREC_SCR_WINDOW_SIZE 64
    uint32_t scr_stc[CLOCKREC_SCR_WINDOW_SIZE];
    uint16_t scr_sof[CLOCKREC_SCR_WINDOW_SIZE];
    int      scr_count;
    int      scr_write_idx;
    int      has_scr;
    double   scr_drift_ppm;
    double   scr_slope;         /* raw regression slope (device ticks / ms) */
    uint32_t scr_clock_freq;    /* dwClockFrequency (Hz); 0 = raw slope only */
    uint16_t scr_last_sof;     /* for unwrapping 11-bit SOF counter */
    int64_t  scr_sof_unwrapped; /* monotonic SOF in ms */

    /* --- Fast rate-mismatch correction (receive rate vs handoff rate) --- */
    int      target_depth;
    double   queue_correction_ppm;
    double   rate_correction_ppm_raw;
    int64_t  queue_last_update_us;
    int64_t  rate_prev_rx_samples;
    int64_t  rate_prev_tx_samples;
    int64_t  rate_rx_samples;
    int64_t  rate_tx_samples;

    /* Target depth warmup: collect samples before enabling the PI controller
     * so target_depth reflects steady-state, not a transient snapshot. */
#define CLOCKREC_TARGET_WARMUP_SIZE 64
    int      target_warmup_samples[CLOCKREC_TARGET_WARMUP_SIZE];
    int      target_warmup_count;
    int      target_warmup_done;

    /* --- Calibration --- */
    int      mid_calibrated;        /* 1 after 1s mid-cal (pacing can start) */
    int      calibrated;
    int64_t  calib_first_local_us;  /* first local_us, for span check */
    int64_t  calib_min_diff;        /* running minimum of (local - sender) */
    int64_t  ramp_start_us;         /* wall time at calibration; drives ASRC ramp-in */

    /* Explicit latency target from CLI (microseconds, 0 = auto-detect via warmup) */
    int64_t  latency_target_us;

    /* --- Per-stream latency statistics (same-machine benchmarking) --- */
    IpmxLatencyStats lat_video;
    IpmxLatencyStats lat_audio;
    int64_t  lat_last_log_us;

    /* --- Resampler (allocated only when clock_recovery) --- */
    struct SwrContext *swr;
    double   delta_accumulator;     /* sigma-delta carry for fractional ASRC */
    uint8_t *resample_buf;
    int      resample_buf_size;
    int64_t  resample_last_log_us;  /* own log timestamp for CLOCKREC line */

    /* --- Video frame skip/repeat --- */
    int64_t  video_frames_skipped;
    int64_t  video_frames_repeated;
    int      video_consecutive_skips;
    int64_t  video_repeat_pts_inc;  /* PTS increment per frame (us), from fps */
    int64_t  last_dequeue_us;       /* wall time of last successful dequeue */

    /* --- Frame counters (video: input vs delivered) --- */
    int64_t  video_rx_frames;       /* video frames dequeued from pkt_queue */
    int64_t  video_tx_frames;       /* video frames delivered to ffplay */

    /* --- ASRC feedback loop (compile-gated by CLOCKREC_ASRC_FEEDBACK) ---
     *
     * Compares input sample rate (samples / arrival time span) to output
     * sample rate (samples / PTS time span).  The rate error is smoothed
     * by a fixed-point Q32 EWMA and applied as a correction to
     * effective_ppm.  This compensates for any long-term bias (PLL
     * estimation error, swr rounding, etc.) without needing an external
     * clock reference.
     *
     * Arrival time = receiver wall clock when packet entered pkt_queue.
     * PTS = intended pacer delivery time (deterministic, no poll jitter).
     */
#if CLOCKREC_ASRC_FEEDBACK
    int64_t  fb_in_samples;         /* total_samples_in snapshot at warmup end */
    int64_t  fb_out_samples;        /* total_samples_out snapshot at warmup end */
    int64_t  fb_cal_wall_us;        /* wall_us when calibration completed */
    int      fb_active;             /* 1 once warmup has elapsed */
    double   fb_correction_ppm;     /* bias correction applied to effective_ppm */
#endif

    /* --- Diagnostics --- */
    FILE    *log_fp;
    int      verbose_log;   /* 1 = PLL spike dumps + periodic PLL log (IPMX_TIMING=1) */
    int64_t  total_samples_in;
    int64_t  total_samples_out;
} IpmxClockRecovery;

/**
 * Initialise clock recovery / latency reporter.
 *
 * Call after audio stream parameters are known.  If only latency_report
 * is set, no SwrContext is allocated and no resampling occurs.
 *
 * @param s   AVFormatContext for av_log (may be NULL).
 * @return 0 on success, negative AVERROR on failure.
 */
int  ipmx_clockrec_init(IpmxClockRecovery *cr, struct AVFormatContext *s);

/**
 * Release all resources.  Logs final latency summary if latency_report
 * was active.
 */
void ipmx_clockrec_free(IpmxClockRecovery *cr, struct AVFormatContext *s);

/**
 * Feed a (sender_timestamp, local_arrival) pair for the primary PLL.
 *
 * sender_us comes from STARTFRAME * 1000 (USB frame number in µs).
 * The module self-calibrates the epoch offset at startup.
 * If clock_sim_drift_ppm or clock_sim_offset_ms are set, local_us is
 * warped before processing to simulate cross-machine clock differences.
 *
 * @param is_audio non-zero for audio packets, zero for video.
 */
void ipmx_clockrec_add_timestamp(IpmxClockRecovery *cr,
                                  int64_t sender_us, int64_t local_us,
                                  int is_audio,
                                  struct AVFormatContext *s);

/**
 * Feed a UVC Source Clock Reference sample for the SCR correlator.
 *
 * @param stc  Source Time Clock (device clock ticks, from UVC SCR).
 * @param sof  11-bit USB SOF frame number (from UVC SCR).
 */
void ipmx_clockrec_add_scr(IpmxClockRecovery *cr,
                            uint32_t stc, uint16_t sof,
                            struct AVFormatContext *s);

/**
 * Update fast correction from cumulative audio sample counts.
 *
 * rx_samples is the cumulative number of input audio samples received by
 * ipmx_usb. tx_samples is the cumulative number of output audio samples
 * handed off by ipmx_usb after ASRC. The function derives a smoothed signed
 * ppm mismatch term from their rates.
 */
void ipmx_clockrec_update_audio_rate(IpmxClockRecovery *cr,
                                     int64_t rx_samples,
                                     int64_t tx_samples);

/**
 * ASRC feedback: update cumulative sample balance correction.
 *
 * Computes (total_out - total_in) since the warmup snapshot, expresses
 * it as ppm, and applies it directly as fb_correction_ppm (Kp = 1.0).
 * Called from the pacer delivery point for each audio packet.
 *
 * Only active when CLOCKREC_ASRC_FEEDBACK is 1.
 */
#if CLOCKREC_ASRC_FEEDBACK
void ipmx_clockrec_fb_update(IpmxClockRecovery *cr);
#endif

/**
 * Resample an audio packet in-place to compensate for clock drift.
 *
 * Only active when clock_recovery is enabled, the module is calibrated,
 * and the audio codec is raw PCM.  For compressed audio the packet is
 * untouched and the caller should rely on PTS adjustment alone.
 *
 * @return 0 on success or no-op, negative AVERROR on failure.
 */
int  ipmx_clockrec_resample_audio(IpmxClockRecovery *cr,
                                   struct AVPacket *pkt);

/**
 * Timing-based video pace decision.
 *
 * Given a video packet's absolute PTS (wall-clock delivery time) and the
 * current wall time, returns one of:
 *   +1  skip   — frame is stale  (e > +0.75 * T_video)
 *    0  normal — output frame    (sleep remainder if early)
 *   -1  repeat — too early       (e < -0.25 * T_video)
 *
 * Subject to a consecutive-skip limit to avoid dropping entire GOPs.
 * The caller must increment video_consecutive_skips on skip and reset
 * it to 0 when a frame is delivered.
 */
int  ipmx_clockrec_video_pace_decision(IpmxClockRecovery *cr,
                                        int64_t pts, int64_t now_us);

/**
 * Reconstruct a receiver-local wall-clock timestamp from a sender-side
 * STARTFRAME-based timestamp using the full affine model:
 *
 *   r̂(s) = pll_slope * s + pll_intercept
 *
 * Before mid-calibration (first 1 second), returns raw sender_us.
 * After mid-calibration, maps sender's clock domain directly to
 * receiver wall-clock coordinates.  Both audio and video use the
 * same affine transform — no per-media state needed.
 *
 * @param sender_us  Absolute sender timestamp (STARTFRAME * 1000).
 * @param is_audio   Non-zero for audio streams, zero for video.
 * @return Receiver-local wall-clock timestamp in µs.
 */
int64_t ipmx_clockrec_reconstruct_ts(IpmxClockRecovery *cr,
                                     int64_t sender_us, int is_audio);

#endif /* AVDEVICE_IPMX_USB_CLOCKREC_H */
