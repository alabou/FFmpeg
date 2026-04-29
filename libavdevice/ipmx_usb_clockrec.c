/*
 * IPMX USB Clock Recovery and Latency Measurement
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

#include "ipmx_usb_clockrec.h"
#include "ipmx_timing.h"

#include <math.h>
#include <string.h>

#include "libavutil/avutil.h"
#include "libavutil/channel_layout.h"
#include "libavutil/error.h"
#include "libavutil/log.h"
#include "libavutil/mem.h"
#include "libavutil/time.h"
#include "libavcodec/packet.h"
#include "libswresample/swresample.h"
#include "libavformat/avformat.h"

IpmxClockRecStats av_ipmx_clockrec_stats;

/* -----------------------------------------------------------------------
 * Helpers
 * ----------------------------------------------------------------------- */

static int codec_is_pcm(enum AVSampleFormat fmt)
{
    switch (fmt) {
    case AV_SAMPLE_FMT_U8:
    case AV_SAMPLE_FMT_S16:
    case AV_SAMPLE_FMT_S32:
    case AV_SAMPLE_FMT_FLT:
    case AV_SAMPLE_FMT_DBL:
    case AV_SAMPLE_FMT_U8P:
    case AV_SAMPLE_FMT_S16P:
    case AV_SAMPLE_FMT_S32P:
    case AV_SAMPLE_FMT_FLTP:
    case AV_SAMPLE_FMT_DBLP:
    case AV_SAMPLE_FMT_S64:
    case AV_SAMPLE_FMT_S64P:
        return 1;
    default:
        return 0;
    }
}

static void lat_stat_init(IpmxLatencyStats *st)
{
    st->count  = 0;
    st->sum    = 0.0;
    st->sum_sq = 0.0;
    st->min    = INT64_MAX;
    st->max    = INT64_MIN;
}

static void lat_stat_add(IpmxLatencyStats *st, int64_t delay_us)
{
    st->count++;
    st->sum    += (double)delay_us;
    st->sum_sq += (double)delay_us * (double)delay_us;
    if (delay_us < st->min) st->min = delay_us;
    if (delay_us > st->max) st->max = delay_us;
}

static void lat_stat_log(const IpmxLatencyStats *st, const char *label,
                          int final, FILE *fp, AVFormatContext *s)
{
    double mean, variance, stddev, stderr_val;
    const char *fin;

    if (st->count < 2)
        return;

    mean     = st->sum / (double)st->count;
    variance = (st->sum_sq / (double)st->count) - mean * mean;
    if (variance < 0.0) variance = 0.0;
    stddev     = sqrt(variance);
    stderr_val = stddev / sqrt((double)st->count);
    fin = final ? " FINAL" : "";

    if (fp)
        fprintf(fp,
                "LATENCY%s [%s]: n=%lld mean=%.0fus stddev=%.0fus "
                "min=%lldus max=%lldus stderr=%.1fus\n",
                fin, label,
                (long long)st->count, mean, stddev,
                (long long)st->min, (long long)st->max,
                stderr_val);

    if (s)
        av_log(s, final ? AV_LOG_INFO : AV_LOG_VERBOSE,
               "LATENCY%s [%s]: n=%lld mean=%.0fus stddev=%.0fus "
               "min=%lldus max=%lldus stderr=%.1fus\n",
               fin, label,
               (long long)st->count, mean, stddev,
               (long long)st->min, (long long)st->max,
               stderr_val);
}

/* -----------------------------------------------------------------------
 * Per-stream sliding-window minimum-offset tracker.
 *
 * Maintains a ring buffer of recent (local - sender) offset values and
 * recomputes the minimum over the window.  The minimum-delay sample is
 * the one least affected by network jitter, congestion, or scheduling
 * delay — a core principle from NTP's clock filter algorithm.
 *
 * Audio and video have separate trackers because their delivery delays
 * differ significantly in split-device mode (audio ~100 µs, video ~33 ms
 * due to frame-rate pacing).
 * ----------------------------------------------------------------------- */

static void offset_tracker_update(int64_t *ring, int *count, int *widx,
                                  int64_t *min_out, int64_t offset)
{
    int i, m;
    int64_t mn = INT64_MAX;

    ring[*widx] = offset;
    *widx = (*widx + 1) % CLOCKREC_OFFSET_WINDOW;
    if (*count < CLOCKREC_OFFSET_WINDOW)
        (*count)++;

    m = *count < CLOCKREC_OFFSET_WINDOW ? *count : CLOCKREC_OFFSET_WINDOW;
    for (i = 0; i < m; i++) {
        if (ring[i] < mn)
            mn = ring[i];
    }
    *min_out = mn;
}

/* -----------------------------------------------------------------------
 * Primary PLL: NTP/PTP-inspired minimum-delay clock filter.
 *
 *   y = local_us (receiver wall clock, µs)
 *   x = sender_us (STARTFRAME * 1000, i.e. USB frame number in µs)
 *   slope = cov(x,y) / var(x)
 *   drift_ppm_raw = (1 - slope) * 1e6
 *   drift_ppm = EWMA(drift_ppm_raw)
 *
 * Two-pass centered regression on the admitted ring buffer.  The ring
 * buffer only contains quality-gated samples (close to minimum delay),
 * so the regression is already resistant to jitter outliers.
 *
 * EWMA smoothing prevents step changes when the window composition
 * shifts (e.g. a burst of good samples rolls off the window).
 *
 * Positive drift_ppm  =>  sender USB frame clock runs fast relative to
 *                         receiver (queue grows, need to consume faster).
 * ----------------------------------------------------------------------- */

static void pll_update(IpmxClockRecovery *cr)
{
    int n = cr->ts_count < CLOCKREC_WINDOW_SIZE
            ? cr->ts_count : CLOCKREC_WINDOW_SIZE;
    double mean_x = 0.0, mean_y = 0.0;
    double cov_xy = 0.0, var_x = 0.0;
    double slope, raw;
    int i, idx;

    if (n < 10)
        return;

    /* Pass 1: means */
    for (i = 0; i < n; i++) {
        idx = (cr->ts_write_idx - n + i + CLOCKREC_WINDOW_SIZE)
              % CLOCKREC_WINDOW_SIZE;
        mean_x += (double)cr->sender_us[idx];
        mean_y += (double)cr->local_us[idx];
    }
    mean_x /= n;
    mean_y /= n;

    /* Pass 2: centered covariance and variance */
    for (i = 0; i < n; i++) {
        double dx, dy;
        idx = (cr->ts_write_idx - n + i + CLOCKREC_WINDOW_SIZE)
              % CLOCKREC_WINDOW_SIZE;
        dx = (double)cr->sender_us[idx] - mean_x;
        dy = (double)cr->local_us[idx]  - mean_y;
        var_x  += dx * dx;
        cov_xy += dx * dy;
    }

    if (var_x < 1e-6)
        return;

    slope = cov_xy / var_x;
    raw   = (1.0 - slope) * 1e6;
    cr->drift_ppm_raw = raw;
    cr->pll_mean_x = mean_x;
    cr->pll_mean_y = mean_y;

    /* EWMA smoothing — prevents step changes when window composition
     * shifts.  First valid measurement seeds the filter directly. */
    if (cr->drift_ppm_ewma == 0.0 && cr->drift_ppm == 0.0 &&
        fabs(raw) < 500.0) {
        cr->drift_ppm_ewma = raw;
        cr->drift_ppm      = raw;
        cr->pll_slope     = 1.0 - raw / 1e6;
        cr->pll_intercept = mean_y - cr->pll_slope * mean_x;
    } else {
        int64_t now_us = av_gettime_relative();
        double  dt_s, max_step, ewma_candidate, step;
        int     slew_clamped = 0;

        /* Regression outlier rejection: if the raw regression result
         * deviates too far from the current EWMA, discard it.  This
         * prevents transient scheduling/network delays from corrupting
         * the drift estimate — they shift local_us for a batch of
         * samples, causing the regression to report a false slope. */
        if (fabs(raw - cr->drift_ppm_ewma) > CLOCKREC_DRIFT_OUTLIER_PPM) {
            cr->drift_outlier_count++;
            goto pll_log;
        }

        cr->drift_ppm_ewma += CLOCKREC_DRIFT_EWMA_ALPHA *
                              (raw - cr->drift_ppm_ewma);
        ewma_candidate = cr->drift_ppm_ewma;

        /* Slew-rate limiter: cap how fast drift_ppm can change.
         * Prevents PLL noise spikes from creating ASRC phase jumps. */
        dt_s = (cr->drift_last_update_us > 0)
               ? (double)(now_us - cr->drift_last_update_us) / 1e6
               : 0.001;
        if (dt_s < 0.0001)
            dt_s = 0.0001;
        max_step = CLOCKREC_DRIFT_SLEW_PPM_PER_S * dt_s;
        step = ewma_candidate - cr->drift_ppm;
        if (step > max_step) {
            cr->drift_ppm += max_step;
            cr->drift_slew_clamp_count++;
            slew_clamped = 1;
        } else if (step < -max_step) {
            cr->drift_ppm -= max_step;
            cr->drift_slew_clamp_count++;
            slew_clamped = 1;
        } else {
            cr->drift_ppm = ewma_candidate;
        }
        cr->drift_last_update_us = now_us;

        /* Update affine model: slope from smoothed drift, intercept
         * EWMA-smoothed to prevent unbounded drift.
         *
         * Raw intercept = mean_y - slope * mean_x.  Because the slope
         * is itself EWMA-smoothed, any residual slope error epsilon
         * causes the raw intercept to drift by epsilon * mean_x, which
         * grows linearly with elapsed time (mean_x tracks sender
         * timestamps).  Smoothing the intercept with the same alpha
         * as the slope prevents this.
         *
         * Exception: while the slew-rate limiter is actively clamping
         * (slope still converging after calibration or a transient),
         * the intercept must track immediately — EWMA lag during rapid
         * slope changes causes massive PTS errors at startup. */
        cr->pll_slope = 1.0 - cr->drift_ppm / 1e6;
        {
            double new_intercept = cr->pll_mean_y -
                                   cr->pll_slope * cr->pll_mean_x;
            if (slew_clamped) {
                cr->pll_intercept = new_intercept;
            } else {
                cr->pll_intercept += CLOCKREC_DRIFT_EWMA_ALPHA *
                                     (new_intercept - cr->pll_intercept);
            }
        }
    }

pll_log:
    if (cr->log_fp && cr->verbose_log && (cr->ts_count % 1000 == 0))
        fprintf(cr->log_fp,
                "wall_us=%lld PLL: n=%d slope=%.9f raw=%.2f ewma=%.2f "
                "slew=%.2f clamp=%lld "
                "rej_stale=%lld rej_jitter=%lld "
                "last=%s/%s off=%lld min=%lld margin=%lld\n",
                (long long)av_gettime_relative(),
                n, slope, raw, cr->drift_ppm_ewma,
                cr->drift_ppm,
                (long long)cr->drift_slew_clamp_count,
                (long long)cr->pll_rejected,
                (long long)cr->pll_jitter_filtered,
                cr->pll_last_sample_audio ? "audio" : "video",
                cr->pll_last_sample_accepted ? "accept" : "reject",
                (long long)cr->pll_last_offset_us,
                (long long)cr->pll_last_min_offset_us,
                (long long)cr->pll_last_margin_us);

    if (cr->log_fp && cr->verbose_log && fabs(raw) > 100.0) {
        int dump_n = n < 32 ? n : 32;
        int64_t wall_us = av_gettime_relative();

        fprintf(cr->log_fp,
                "wall_us=%lld PLL_SPIKE: n=%d slope=%.9f raw=%.2f ewma=%.2f "
                "slew=%.2f dump_n=%d last=%s/%s off=%lld min=%lld margin=%lld\n",
                (long long)wall_us, n, slope, raw, cr->drift_ppm_ewma,
                cr->drift_ppm, dump_n,
                cr->pll_last_sample_audio ? "audio" : "video",
                cr->pll_last_sample_accepted ? "accept" : "reject",
                (long long)cr->pll_last_offset_us,
                (long long)cr->pll_last_min_offset_us,
                (long long)cr->pll_last_margin_us);
        for (i = 0; i < dump_n; i++) {
            int ring_idx = (cr->ts_write_idx - dump_n + i + CLOCKREC_WINDOW_SIZE)
                           % CLOCKREC_WINDOW_SIZE;
            int prev_idx = (ring_idx - 1 + CLOCKREC_WINDOW_SIZE)
                           % CLOCKREC_WINDOW_SIZE;
            int64_t sender = cr->sender_us[ring_idx];
            int64_t local  = cr->local_us[ring_idx];
            int64_t raw_diff = local - sender;
            int64_t dsender = 0;
            int64_t dlocal = 0;

            if (i > 0) {
                dsender = sender - cr->sender_us[prev_idx];
                dlocal  = local  - cr->local_us[prev_idx];
            }

            fprintf(cr->log_fp,
                    "wall_us=%lld PLL_SAMPLE: i=%d ring=%d sender_us=%lld "
                    "local_us=%lld raw_diff=%lld dsender=%lld dlocal=%lld\n",
                    (long long)wall_us, i, ring_idx,
                    (long long)sender, (long long)local,
                    (long long)raw_diff, (long long)dsender,
                    (long long)dlocal);
        }
    }
}

/* -----------------------------------------------------------------------
 * SCR correlator: UVC device clock (STC) vs USB SOF frame number.
 *
 * The 11-bit SOF wraps every ~2048 ms.  We unwrap it into a monotonic
 * millisecond counter for regression.
 *
 *   y = STC (device clock ticks)
 *   x = SOF_unwrapped (ms)
 *   slope = cov(x,y) / var(x) = device_ticks_per_ms
 *   expected = dwClockFrequency / 1000
 *   scr_drift_ppm = (slope / expected - 1) * 1e6
 * ----------------------------------------------------------------------- */

static void scr_correlator_update(IpmxClockRecovery *cr)
{
    int n = cr->scr_count < CLOCKREC_SCR_WINDOW_SIZE
            ? cr->scr_count : CLOCKREC_SCR_WINDOW_SIZE;
    double sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0;
    double mean_x, mean_y, cov_xy, var_x, slope;
    int i, idx;

    if (n < 10)
        return;

    for (i = 0; i < n; i++) {
        double x, y;
        idx = (cr->scr_write_idx - n + i + CLOCKREC_SCR_WINDOW_SIZE)
              % CLOCKREC_SCR_WINDOW_SIZE;
        x = (double)cr->scr_sof[idx];
        y = (double)cr->scr_stc[idx];
        sx  += x;
        sy  += y;
        sxx += x * x;
        sxy += x * y;
    }

    mean_x = sx / n;
    mean_y = sy / n;
    cov_xy = (sxy / n) - mean_x * mean_y;
    var_x  = (sxx / n) - mean_x * mean_x;

    if (var_x < 1e-6)
        return;

    slope = cov_xy / var_x;
    cr->scr_slope = slope;

    if (cr->scr_clock_freq > 0) {
        double expected = (double)cr->scr_clock_freq / 1000.0;
        cr->scr_drift_ppm = (slope / expected - 1.0) * 1e6;
    } else {
        cr->scr_drift_ppm = slope;
    }
}

/* -----------------------------------------------------------------------
 * Public API
 * ----------------------------------------------------------------------- */

int ipmx_clockrec_init(IpmxClockRecovery *cr, AVFormatContext *s)
{
    if (!cr->clock_recovery && !cr->latency_report)
        return 0;

    cr->is_pcm = codec_is_pcm(cr->sample_fmt);

    cr->ts_count     = 0;
    cr->ts_write_idx = 0;
    cr->pll_has_audio = 0;
    cr->drift_ppm     = 0.0;
    cr->drift_ppm_raw = 0.0;
    cr->drift_ppm_ewma = 0.0;
    cr->drift_last_update_us = 0;
    cr->drift_slew_clamp_count = 0;
    cr->last_accepted_sender_us = 0;
    cr->pll_rejected = 0;
    cr->pll_jitter_filtered = 0;
    cr->pll_stream_skipped = 0;
    cr->pll_gate_accepted = 0;
    cr->pll_gate_accept_margin_sum = 0;
    cr->pll_gate_accept_margin_max = 0;
    cr->pll_gate_reject_margin_sum = 0;
    cr->pll_gate_reject_margin_min = INT64_MAX;
    cr->pll_gate_reject_margin_max = 0;
    cr->pll_last_offset_us = 0;
    cr->pll_last_min_offset_us = 0;
    cr->pll_last_margin_us = 0;
    cr->pll_last_sample_audio = 0;
    cr->pll_last_sample_accepted = 1;
    cr->offset_count_a = cr->offset_count_v = 0;
    cr->offset_widx_a  = cr->offset_widx_v  = 0;
    cr->min_offset_a   = INT64_MAX;
    cr->min_offset_v   = INT64_MAX;

    cr->target_depth          = 0;
    cr->queue_correction_ppm  = 0.0;
    cr->rate_correction_ppm_raw = 0.0;
    cr->queue_last_update_us  = 0;
    cr->rate_prev_rx_samples  = 0;
    cr->rate_prev_tx_samples  = 0;
    cr->rate_rx_samples       = 0;
    cr->rate_tx_samples       = 0;
    cr->target_warmup_count   = 0;
    cr->target_warmup_done    = 0;
    cr->delta_accumulator     = 0.0;

    cr->mid_calibrated   = 0;
    cr->calibrated       = 0;
    cr->clock_offset_us  = 0;
    cr->clock_offset_set = 0;
    cr->pll_slope        = 1.0;
    cr->pll_intercept    = 0.0;
    cr->pll_mean_x       = 0.0;
    cr->pll_mean_y       = 0.0;
    cr->sim_epoch_us     = 0;
    cr->calib_first_local_us = 0;
    cr->calib_min_diff   = INT64_MAX;
    cr->ramp_start_us    = 0;

    cr->scr_count        = 0;
    cr->scr_write_idx    = 0;
    cr->has_scr          = 0;
    cr->scr_drift_ppm    = 0.0;
    cr->scr_slope        = 0.0;
    /* scr_clock_freq is set by caller before init */
    cr->scr_last_sof     = 0;
    cr->scr_sof_unwrapped = 0;

    lat_stat_init(&cr->lat_video);
    lat_stat_init(&cr->lat_audio);
    cr->lat_last_log_us = 0;

    cr->total_samples_in  = 0;
    cr->total_samples_out = 0;

    cr->video_frames_skipped  = 0;
    cr->video_frames_repeated = 0;
    cr->video_consecutive_skips = 0;
    cr->last_dequeue_us       = 0;
    cr->video_rx_frames       = 0;
    cr->video_tx_frames       = 0;

#if CLOCKREC_ASRC_FEEDBACK
    cr->fb_in_samples     = 0;
    cr->fb_out_samples    = 0;
    cr->fb_cal_wall_us    = 0;
    cr->fb_active         = 0;
    cr->fb_correction_ppm = 0.0;
#endif

    cr->verbose_log = ipmx_timing_enabled();

    /* Open clockrec log only if --log (verbose) or --latency-report.
     * No --log and no --latency-report → no log file at all. */
    if (cr->verbose_log || cr->latency_report) {
        cr->log_fp = fopen("/tmp/ipmx_clockrec.log", "w");
        if (cr->log_fp)
            setvbuf(cr->log_fp, NULL, _IOLBF, 0);
    }

    if (cr->clock_recovery && cr->is_pcm && cr->sample_rate > 0) {
        AVChannelLayout ch_layout = { 0 };
        int ret;

        av_channel_layout_default(&ch_layout, cr->channels);

        ret = swr_alloc_set_opts2(&cr->swr,
                                  &ch_layout, cr->sample_fmt, cr->sample_rate,
                                  &ch_layout, cr->sample_fmt, cr->sample_rate,
                                  0, NULL);
        av_channel_layout_uninit(&ch_layout);
        if (ret < 0 || !cr->swr) {
            av_log(s, AV_LOG_ERROR,
                   "clockrec: swr_alloc_set_opts2 failed: %s\n",
                   av_err2str(ret));
            return ret;
        }

        ret = swr_init(cr->swr);
        if (ret < 0) {
            av_log(s, AV_LOG_ERROR,
                   "clockrec: swr_init failed: %s\n", av_err2str(ret));
            swr_free(&cr->swr);
            return ret;
        }

        /*
         * Activate the resampler in passthrough mode so that subsequent
         * swr_set_compensation() calls can adjust the ratio smoothly.
         */
        swr_set_compensation(cr->swr, 0, cr->sample_rate);

        cr->resample_buf          = NULL;
        cr->resample_buf_size     = 0;
        cr->resample_last_log_us  = 0;

        av_log(s, AV_LOG_INFO,
               "clockrec: clock recovery enabled (%d Hz, %d ch, %s)\n",
               cr->sample_rate, cr->channels,
               av_get_sample_fmt_name(cr->sample_fmt));
    }

    if (cr->latency_report)
        av_log(s, AV_LOG_INFO,
               "clockrec: latency reporting enabled\n");

    return 0;
}

void ipmx_clockrec_free(IpmxClockRecovery *cr, AVFormatContext *s)
{
    if (!cr->clock_recovery && !cr->latency_report)
        return;

    if (cr->latency_report) {
        lat_stat_log(&cr->lat_video, "video", 1, cr->log_fp, s);
        lat_stat_log(&cr->lat_audio, "audio", 1, cr->log_fp, s);
    }

    if (cr->clock_recovery && cr->log_fp) {
        if (cr->total_samples_in > 0) {
            double ratio = (double)cr->total_samples_out /
                           (double)cr->total_samples_in;
            fprintf(cr->log_fp,
                    "CLOCKREC FINAL: samples_in=%lld samples_out=%lld "
                    "ratio=%.9f drift_ppm=%.2f\n",
                    (long long)cr->total_samples_in,
                    (long long)cr->total_samples_out,
                    ratio, cr->drift_ppm);
        }
        fprintf(cr->log_fp,
                "VIDEO FINAL: skipped=%lld repeated=%lld "
                "rx_frames=%lld tx_frames=%lld\n",
                (long long)cr->video_frames_skipped,
                (long long)cr->video_frames_repeated,
                (long long)cr->video_rx_frames,
                (long long)cr->video_tx_frames);
        fprintf(cr->log_fp,
                "PLL FILTER: stale_rejected=%lld jitter_filtered=%lld "
                "stream_skipped=%lld pll_stream=%s "
                "min_a=%lldus min_v=%lldus\n",
                (long long)cr->pll_rejected,
                (long long)cr->pll_jitter_filtered,
                (long long)cr->pll_stream_skipped,
                cr->pll_has_audio ? "audio" : "video",
                cr->min_offset_a < INT64_MAX
                    ? (long long)cr->min_offset_a : -1LL,
                cr->min_offset_v < INT64_MAX
                    ? (long long)cr->min_offset_v : -1LL);
        fprintf(cr->log_fp,
                "PLL GATE: accepted=%lld avg_accept_margin=%.1fus "
                "max_accept_margin=%lldus rejected=%lld "
                "avg_reject_margin=%.1fus min_reject_margin=%lldus "
                "max_reject_margin=%lldus gate=%dus "
                "last=%s/%s off=%lldus min=%lldus margin=%lldus\n",
                (long long)cr->pll_gate_accepted,
                cr->pll_gate_accepted > 0
                    ? (double)cr->pll_gate_accept_margin_sum /
                      (double)cr->pll_gate_accepted
                    : 0.0,
                (long long)cr->pll_gate_accept_margin_max,
                (long long)cr->pll_jitter_filtered,
                cr->pll_jitter_filtered > 0
                    ? (double)cr->pll_gate_reject_margin_sum /
                      (double)cr->pll_jitter_filtered
                    : 0.0,
                cr->pll_jitter_filtered > 0
                    ? (long long)cr->pll_gate_reject_margin_min
                    : -1LL,
                (long long)cr->pll_gate_reject_margin_max,
                CLOCKREC_JITTER_GATE_US,
                cr->pll_last_sample_audio ? "audio" : "video",
                cr->pll_last_sample_accepted ? "accept" : "reject",
                (long long)cr->pll_last_offset_us,
                (long long)cr->pll_last_min_offset_us,
                (long long)cr->pll_last_margin_us);
        fprintf(cr->log_fp,
                "PLL SLEW: limit=%.1f ppm/s clamped=%lld "
                "final_ewma=%.2f final_slew=%.2f\n",
                CLOCKREC_DRIFT_SLEW_PPM_PER_S,
                (long long)cr->drift_slew_clamp_count,
                cr->drift_ppm_ewma, cr->drift_ppm);
        if (cr->has_scr)
            fprintf(cr->log_fp,
                    "SCR FINAL: samples=%d slope=%.1f drift_ppm=%.2f clk=%u\n",
                    cr->scr_count, cr->scr_slope,
                    cr->scr_drift_ppm, cr->scr_clock_freq);
        av_log(s, AV_LOG_INFO,
               "clockrec: video frames skipped=%lld repeated=%lld%s\n",
               (long long)cr->video_frames_skipped,
               (long long)cr->video_frames_repeated,
               cr->has_scr ? " (SCR active)" : "");
    }

    if (cr->log_fp) {
        fclose(cr->log_fp);
        cr->log_fp = NULL;
    }

    if (cr->swr)
        swr_free(&cr->swr);

    av_freep(&cr->resample_buf);
    cr->resample_buf_size = 0;
}

void ipmx_clockrec_add_timestamp(IpmxClockRecovery *cr,
                                  int64_t sender_us, int64_t local_us,
                                  int is_audio,
                                  AVFormatContext *s)
{
    int64_t delay_us;

    if (!cr->clock_recovery && !cr->latency_report)
        return;

    /*
     * Simulated clock warp: inject a phase offset and a frequency drift
     * into the receiver's view of local time.  This lets us exercise the
     * full PLL + resampling pipeline on a single machine.
     *
     *   warped = local + offset_ms*1000
     *            + (local - epoch) * drift_ppm / 1e6
     */
    if (cr->clock_sim_drift_ppm != 0.0 || cr->clock_sim_offset_ms != 0) {
        if (!cr->sim_epoch_us)
            cr->sim_epoch_us = local_us;
        local_us += (int64_t)cr->clock_sim_offset_ms * 1000;
        local_us += (int64_t)((double)(local_us - cr->sim_epoch_us) *
                              cr->clock_sim_drift_ppm / 1e6);
    }

    /*
     * Self-calibrate the clock offset between sender epoch (USB frame
     * counter starting near 0, converted to µs) and receiver epoch
     * (CLOCK_MONOTONIC).
     *
     * The sender may have a startup backlog (buffered frames from before
     * the receiver connected).  During backlog drain, (local - sender) is
     * artificially high because sender_us corresponds to old frames while
     * local_us is current.  The true steady-state offset is the MINIMUM
     * of (local - sender), reached once the backlog is drained.
     *
     * We require at least CLOCKREC_CALIB_MIN_SPAN_US of real-time data
     * to ensure the backlog has cleared before finalizing calibration.
     */
    /*
     * Two-phase calibration:
     *   Phase 1 (mid-cal, 1s): compute initial affine model, start pacing.
     *   Phase 2 (full-cal, 2s): refine with more data, lock in.
     *
     * Pre-calibration packets are discarded by the pacer (no valid b̂ yet).
     * Mid-calibration starts pacing after only 1s of startup delay.
     */
    if (!cr->calibrated) {
        int64_t raw_diff = local_us - sender_us;
        int64_t span_us;

        if (!cr->calib_first_local_us)
            cr->calib_first_local_us = local_us;
        if (raw_diff < cr->calib_min_diff)
            cr->calib_min_diff = raw_diff;

        /* Fill the ring buffer for PLL even before calibration. */
        cr->sender_us[cr->ts_write_idx] = sender_us;
        cr->local_us[cr->ts_write_idx]  = local_us;
        cr->ts_write_idx = (cr->ts_write_idx + 1) % CLOCKREC_WINDOW_SIZE;
        if (cr->ts_count < CLOCKREC_WINDOW_SIZE)
            cr->ts_count++;

        span_us = local_us - cr->calib_first_local_us;

        /* Mid-calibration at 1s OR full calibration at 2s */
        if ((!cr->mid_calibrated && span_us >= CLOCKREC_MID_CALIB_SPAN_US) ||
            (cr->mid_calibrated && !cr->calibrated &&
             span_us >= CLOCKREC_CALIB_MIN_SPAN_US)) {
            int is_mid = !cr->mid_calibrated;
            int is_full = cr->mid_calibrated;

            cr->clock_offset_us  = cr->calib_min_diff;
            cr->clock_offset_set = 1;

            if (is_mid) {
                cr->mid_calibrated = 1;
                cr->ramp_start_us  = av_gettime_relative();
            }
            if (is_full)
                cr->calibrated = 1;

            /* When an explicit latency target is set, lock in the target
             * depth immediately and skip the warmup phase entirely. */
            if (cr->latency_target_us > 0 && !cr->target_warmup_done) {
                cr->target_depth = (int)(cr->latency_target_us / 8000);
                if (cr->target_depth < 2)
                    cr->target_depth = 2;
                cr->target_warmup_done = 1;
            }

            /* Reset PLL drift at calibration boundary. */
            cr->drift_ppm     = 0.0;
            cr->drift_ppm_raw = 0.0;
            cr->drift_ppm_ewma = 0.0;
            cr->drift_last_update_us = 0;
            cr->drift_slew_clamp_count = 0;
            cr->queue_correction_ppm  = 0.0;
            cr->rate_correction_ppm_raw = 0.0;
            cr->queue_last_update_us  = 0;
            cr->rate_prev_rx_samples  = 0;
            cr->rate_prev_tx_samples  = 0;
            cr->delta_accumulator     = 0.0;

            /* Retroactive jitter gate. */
            {
                int64_t gate_max = cr->clock_offset_us + CLOCKREC_JITTER_GATE_US;
                int src, dst = 0, old_count = cr->ts_count;

                for (src = 0; src < old_count; src++) {
                    int ring = (cr->ts_write_idx - old_count + src
                                + CLOCKREC_WINDOW_SIZE) % CLOCKREC_WINDOW_SIZE;
                    int64_t offset = cr->local_us[ring] - cr->sender_us[ring];
                    if (offset <= gate_max) {
                        int dst_ring = dst % CLOCKREC_WINDOW_SIZE;
                        cr->sender_us[dst_ring] = cr->sender_us[ring];
                        cr->local_us[dst_ring]  = cr->local_us[ring];
                        dst++;
                    }
                }
                cr->ts_count = dst < CLOCKREC_WINDOW_SIZE ? dst : CLOCKREC_WINDOW_SIZE;
                cr->ts_write_idx = dst % CLOCKREC_WINDOW_SIZE;

                if (cr->log_fp)
                    fprintf(cr->log_fp,
                            "PLL: retroactive gate kept %d of %d pre-cal samples "
                            "(gate=%d us)\n",
                            dst, old_count, CLOCKREC_JITTER_GATE_US);
            }

            /* Seed the affine model from initial regression. */
            pll_update(cr);
            cr->pll_slope     = 1.0 - cr->drift_ppm / 1e6;
            cr->pll_intercept = cr->pll_mean_y - cr->pll_slope * cr->pll_mean_x;

            if (cr->log_fp)
                fprintf(cr->log_fp,
                        "%s: clock_offset=%lldus target_depth=%d "
                        "span_us=%lld slope=%.9f intercept=%.0f "
                        "target_source=%s\n",
                        is_mid ? "MID_CALIBRATED" : "CALIBRATED",
                        (long long)cr->clock_offset_us, cr->target_depth,
                        (long long)span_us,
                        cr->pll_slope, cr->pll_intercept,
                        cr->latency_target_us > 0 ? "explicit" : "auto");
            av_log(s, AV_LOG_INFO,
                   "clockrec: %s — clock offset %lld us, "
                   "slope %.9f, intercept %.0f\n",
                   is_mid ? "mid-calibrated (1s)" : "calibrated (2s)",
                   (long long)cr->clock_offset_us,
                   cr->pll_slope, cr->pll_intercept);

            if (cr->clock_sim_drift_ppm != 0.0 || cr->clock_sim_offset_ms != 0)
                av_log(s, AV_LOG_INFO,
                       "clockrec: simulated drift %.1f PPM, offset %d ms\n",
                       cr->clock_sim_drift_ppm, cr->clock_sim_offset_ms);

            /* Between mid and full cal, fall through to post-cal path. */
            if (is_mid)
                return;
        }

        /* Not yet mid-calibrated — still collecting. */
        if (!cr->mid_calibrated)
            return;

        /* Between mid and full calibration: fall through to post-cal. */
    }

    /*
     * Post-calibration: track per-stream minimum offsets for accurate e2e.
     * In split-device mode, audio and video channels start independently
     * so a single global offset contaminates one stream's e2e with the
     * other's pipeline delay.  Per-stream offsets eliminate this.
     *
     * Also refine the global offset (used as fallback / PLL baseline).
     * Only the display is affected — the PLL depends on slope, not intercept.
     */
    if (cr->clock_offset_set) {
        int64_t raw_diff = local_us - sender_us;
        if (raw_diff > 0 && raw_diff < cr->clock_offset_us)
            cr->clock_offset_us = raw_diff;

        if (raw_diff > 0) {
            if (is_audio) {
                if (!cr->has_stream_offset_audio ||
                    raw_diff < cr->stream_offset_audio) {
                    cr->stream_offset_audio     = raw_diff;
                    cr->has_stream_offset_audio = 1;
                }
            } else {
                if (!cr->has_stream_offset_video ||
                    raw_diff < cr->stream_offset_video) {
                    cr->stream_offset_video     = raw_diff;
                    cr->has_stream_offset_video = 1;
                }
            }
        }
    }

    delay_us = (local_us - sender_us) - cr->clock_offset_us;

    /* --- Per-stream latency statistics (relative to baseline) --- */
    if (cr->latency_report) {
        lat_stat_add(is_audio ? &cr->lat_audio : &cr->lat_video, delay_us);

        if (local_us - cr->lat_last_log_us >= CLOCKREC_LOG_INTERVAL_US) {
            lat_stat_log(&cr->lat_video, "video", 0, cr->log_fp, s);
            lat_stat_log(&cr->lat_audio, "audio", 0, cr->log_fp, s);
            if (cr->clock_recovery && cr->log_fp)
                fprintf(cr->log_fp,
                        "VIDEO: skipped=%lld repeated=%lld drift=%.2f ppm\n",
                        (long long)cr->video_frames_skipped,
                        (long long)cr->video_frames_repeated,
                        cr->drift_ppm);
            cr->lat_last_log_us = local_us;
        }
    }

    /* Update shared stats for ffplay's status line */
    if (cr->clock_recovery && cr->calibrated) {
        av_ipmx_clockrec_stats.drift_ppm       = cr->drift_ppm;
        av_ipmx_clockrec_stats.asrc_ppm        = cr->drift_ppm +
                                                  cr->queue_correction_ppm;
        av_ipmx_clockrec_stats.video_skipped   = cr->video_frames_skipped;
        av_ipmx_clockrec_stats.video_repeated  = cr->video_frames_repeated;
        av_ipmx_clockrec_stats.clock_offset_us = cr->clock_offset_us;
        av_ipmx_clockrec_stats.calibrated      = 1;

        if (cr->has_stream_offset_video) {
            av_ipmx_clockrec_stats.clock_offset_video_us =
                cr->stream_offset_video;
            av_ipmx_clockrec_stats.has_video_offset = 1;
        }
        if (cr->has_stream_offset_audio) {
            av_ipmx_clockrec_stats.clock_offset_audio_us =
                cr->stream_offset_audio;
            av_ipmx_clockrec_stats.has_audio_offset = 1;
        }
    }

    /*
     * NTP/PTP-inspired multi-stage sample filter:
     *
     *   Stage 0 — Stream separation:
     *     Once audio samples appear, only audio enters the PLL.  This
     *     prevents regression bias from mixing streams with different
     *     constant delivery delays (critical for split-device mode
     *     where video pacing adds ~33 ms of delay vs ~100 µs for audio).
     *     Audio is preferred because it has higher sample rate (~125 Hz),
     *     lower and more consistent delivery delay, and no pacing jitter.
     *
     *   Stage 1 — Stale-value pre-filter:
     *     Reject samples whose sender_us has jumped backward by > 1 s.
     *
     *   Stage 2 — Per-stream minimum-delay quality gate:
     *     Track sliding-window minimum offset per stream type.  Reject
     *     samples exceeding min + CLOCKREC_JITTER_GATE_US.
     *
     *   Samples passing all stages enter the PLL ring buffer for
     *   centered regression → EWMA-smoothed drift_ppm.
     */
    {
        int64_t offset = local_us - sender_us;
        int64_t min_off;
        int64_t margin;

        /* Stage 0: stream separation — prefer audio for PLL.
         * Keep existing video samples in the ring buffer: they share the
         * same sender_us epoch (STARTFRAME) and give the regression a
         * solid baseline.  New audio samples will naturally dilute and
         * replace them as the ring buffer wraps. */
        if (is_audio && !cr->pll_has_audio) {
            cr->pll_has_audio = 1;
            if (cr->log_fp)
                fprintf(cr->log_fp,
                        "PLL: switching to audio-only (kept %d video samples)\n",
                        cr->ts_count);
        }
        if (!is_audio && cr->pll_has_audio) {
            cr->pll_stream_skipped++;
            offset_tracker_update(cr->offset_ring_v, &cr->offset_count_v,
                                  &cr->offset_widx_v, &cr->min_offset_v,
                                  offset);
            return;
        }

        /* Stage 1: stale sender_us rejection */
        if (cr->last_accepted_sender_us > 0 &&
            sender_us < cr->last_accepted_sender_us - 1000000) {
            cr->pll_rejected++;
            return;
        }
        if (sender_us > cr->last_accepted_sender_us)
            cr->last_accepted_sender_us = sender_us;

        /* Update the per-stream sliding-window minimum-offset tracker */
        if (is_audio)
            offset_tracker_update(cr->offset_ring_a, &cr->offset_count_a,
                                  &cr->offset_widx_a, &cr->min_offset_a,
                                  offset);
        else
            offset_tracker_update(cr->offset_ring_v, &cr->offset_count_v,
                                  &cr->offset_widx_v, &cr->min_offset_v,
                                  offset);

        min_off = is_audio ? cr->min_offset_a : cr->min_offset_v;
        margin = min_off < INT64_MAX ? offset - min_off : 0;
        cr->pll_last_sample_audio = is_audio;
        cr->pll_last_offset_us = offset;
        cr->pll_last_min_offset_us = min_off < INT64_MAX ? min_off : 0;
        cr->pll_last_margin_us = margin;

        /* Stage 2: jitter quality gate — only during steady state */
        if (cr->calibrated && min_off < INT64_MAX &&
            offset > min_off + CLOCKREC_JITTER_GATE_US) {
            cr->pll_jitter_filtered++;
            cr->pll_gate_reject_margin_sum += margin;
            if (margin < cr->pll_gate_reject_margin_min)
                cr->pll_gate_reject_margin_min = margin;
            if (margin > cr->pll_gate_reject_margin_max)
                cr->pll_gate_reject_margin_max = margin;
            cr->pll_last_sample_accepted = 0;
            return;
        }

        cr->pll_gate_accepted++;
        cr->pll_gate_accept_margin_sum += margin;
        if (margin > cr->pll_gate_accept_margin_max)
            cr->pll_gate_accept_margin_max = margin;
        cr->pll_last_sample_accepted = 1;
    }

    cr->sender_us[cr->ts_write_idx] = sender_us;
    cr->local_us[cr->ts_write_idx]  = local_us;
    cr->ts_write_idx = (cr->ts_write_idx + 1) % CLOCKREC_WINDOW_SIZE;
    if (cr->ts_count < CLOCKREC_WINDOW_SIZE)
        cr->ts_count++;

    /* --- Steady-state PLL update --- */
    if (cr->clock_recovery)
        pll_update(cr);
}

void ipmx_clockrec_add_scr(IpmxClockRecovery *cr,
                           uint32_t stc, uint16_t sof,
                           AVFormatContext *s)
{
    int16_t delta;

    if (!cr->clock_recovery)
        return;

    /* Unwrap the 11-bit SOF counter (wraps every 2048 ms) into a
     * monotonic millisecond value.  Handles both forward wrap and
     * small backward jitter. */
    delta = (int16_t)((sof - cr->scr_last_sof) & 0x07FF);
    if (delta > 1024)
        delta -= 2048;
    cr->scr_sof_unwrapped += delta;
    cr->scr_last_sof = sof;

    cr->scr_stc[cr->scr_write_idx] = stc;
    cr->scr_sof[cr->scr_write_idx] = (uint16_t)(cr->scr_sof_unwrapped & 0xFFFF);
    cr->scr_write_idx = (cr->scr_write_idx + 1) % CLOCKREC_SCR_WINDOW_SIZE;
    if (cr->scr_count < CLOCKREC_SCR_WINDOW_SIZE)
        cr->scr_count++;

    if (!cr->has_scr) {
        cr->has_scr = 1;
        av_log(s, AV_LOG_INFO,
               "clockrec: UVC SCR detected (STC=%u SOF=%u)\n",
               stc, sof);
    }

    scr_correlator_update(cr);

    if (cr->log_fp && cr->scr_count % 30 == 0)
        fprintf(cr->log_fp,
                "SCR: stc=%u sof=%u unwrapped=%lld slope=%.1f drift_ppm=%.2f\n",
                stc, sof,
                (long long)cr->scr_sof_unwrapped,
                cr->scr_slope, cr->scr_drift_ppm);
}

void ipmx_clockrec_update_audio_rate(IpmxClockRecovery *cr,
                                     int64_t rx_samples,
                                     int64_t tx_samples)
{
    double dt_s, raw_ppm, alpha;
    int64_t now;
    int64_t dt_us, d_rx, d_tx;

    if (!cr->clock_recovery || !cr->calibrated || cr->sample_rate <= 0)
        return;

    now = av_gettime_relative();
    cr->rate_rx_samples = rx_samples;
    cr->rate_tx_samples = tx_samples;

    if (cr->queue_last_update_us == 0) {
        cr->queue_last_update_us = now;
        cr->rate_prev_rx_samples = rx_samples;
        cr->rate_prev_tx_samples = tx_samples;
        return;
    }

    dt_us = now - cr->queue_last_update_us;
    if (dt_us < CLOCKREC_RATE_WINDOW_US)
        return;

    d_rx = rx_samples - cr->rate_prev_rx_samples;
    d_tx = tx_samples - cr->rate_prev_tx_samples;
    cr->rate_prev_rx_samples = rx_samples;
    cr->rate_prev_tx_samples = tx_samples;
    cr->queue_last_update_us = now;

    if (d_rx <= 0 && d_tx <= 0)
        return;

    dt_s = (double)dt_us / 1e6;
    raw_ppm = 1000000.0 * ((double)(d_rx - d_tx)) /
              ((double)cr->sample_rate * dt_s);
    cr->rate_correction_ppm_raw = raw_ppm;

    if (raw_ppm >  CLOCKREC_MAX_QUEUE_PPM)
        raw_ppm =  CLOCKREC_MAX_QUEUE_PPM;
    if (raw_ppm < -CLOCKREC_MAX_QUEUE_PPM)
        raw_ppm = -CLOCKREC_MAX_QUEUE_PPM;

    alpha = CLOCKREC_RATE_EWMA_ALPHA;
    cr->queue_correction_ppm =
        (1.0 - alpha) * cr->queue_correction_ppm + alpha * raw_ppm;
    if (cr->queue_correction_ppm >  CLOCKREC_MAX_QUEUE_PPM)
        cr->queue_correction_ppm =  CLOCKREC_MAX_QUEUE_PPM;
    if (cr->queue_correction_ppm < -CLOCKREC_MAX_QUEUE_PPM)
        cr->queue_correction_ppm = -CLOCKREC_MAX_QUEUE_PPM;

}

static double get_effective_ppm(IpmxClockRecovery *cr)
{
    double effective_ppm;

    if (!cr->calibrated)
        return 0.0;

    effective_ppm = cr->drift_ppm;

    if (cr->ramp_start_us > 0) {
        int64_t elapsed = av_gettime_relative() - cr->ramp_start_us;
        if (elapsed < CLOCKREC_RAMP_DURATION_US) {
            double alpha = (double)elapsed /
                           (double)CLOCKREC_RAMP_DURATION_US;
            effective_ppm *= alpha;
        }
        /* Note: we don't clear ramp_start_us here because this is a getter
         * called from multiple places. It is cleared in resample_audio. */
    }

    /* ASRC feedback: add the measured rate error so the ASRC corrects
     * its output rate.  Positive fb_correction_ppm (output faster than
     * input) makes effective_ppm less negative, reducing surplus. */
#if CLOCKREC_ASRC_FEEDBACK
    if (cr->fb_active)
        effective_ppm += cr->fb_correction_ppm;
#endif

    return effective_ppm;
}

int ipmx_clockrec_resample_audio(IpmxClockRecovery *cr, AVPacket *pkt)
{
    double effective_ppm;
    int sample_delta, in_samples, max_out, out_count;
    int frame_size;
    int new_size;
    AVBufferRef *new_buf;
    const uint8_t *in_ptr;
    uint8_t *out_ptr;

    if (!cr->clock_recovery || !cr->swr || !cr->is_pcm)
        return 0;
    if (!pkt || pkt->size <= 0)
        return 0;

    frame_size = cr->channels * cr->bytes_per_sample;
    if (frame_size <= 0)
        return 0;

    in_samples = pkt->size / frame_size;
    if (in_samples <= 0)
        return 0;

    /*
     * Always route audio through the resampler, even before calibration.
     * Before calibration completes, use zero compensation (1:1 passthrough
     * rate).  This primes the resampler's internal polyphase filter state
     * so that when calibration completes and drift corrections begin, the
     * ~16-sample group delay has already been absorbed — no pop/click at
     * the transition.
     *
     * After calibration, ramp effective_ppm from 0 to the full correction
     * over CLOCKREC_RAMP_DURATION_US.  This prevents an audible pitch
     * transient when the ASRC activates — even a small step change in
     * resampling ratio produces an audible glitch on a pure tone.
     */
    effective_ppm = get_effective_ppm(cr);

    /* Clear the ramp trigger once duration elapsed */
    if (cr->ramp_start_us > 0 &&
        (av_gettime_relative() - cr->ramp_start_us) >= CLOCKREC_RAMP_DURATION_US) {
        cr->ramp_start_us = 0;
    }

    /*
     * Sigma-delta ASRC: accumulate the exact fractional correction per
     * packet and emit an integer sample_delta only when the accumulator
     * crosses ±0.5.  This achieves sub-quantum average rate correction
     * (1/48000 = 20.83 ppm per integer step) by rapidly switching between
     * adjacent delta values — analogous to a first-order sigma-delta DAC.
     *
     * swr_set_compensation(delta, distance):
     *   Over the next `distance` input samples, produce
     *   `distance + delta` output samples.
     *
     * To consume faster (sender fast, positive drift_ppm):
     *   fewer output samples => delta < 0.
     */
    cr->delta_accumulator += -(effective_ppm * (double)in_samples / 1e6);
    sample_delta = (int)lround(cr->delta_accumulator);
    cr->delta_accumulator -= (double)sample_delta;

    swr_set_compensation(cr->swr, sample_delta, in_samples);

    max_out = swr_get_out_samples(cr->swr, in_samples);
    if (max_out <= 0)
        max_out = in_samples + 16;

    new_size = max_out * frame_size;
    if (new_size > cr->resample_buf_size) {
        uint8_t *tmp = av_realloc(cr->resample_buf, new_size);
        if (!tmp)
            return AVERROR(ENOMEM);
        cr->resample_buf      = tmp;
        cr->resample_buf_size = new_size;
    }

    in_ptr  = pkt->data;
    out_ptr = cr->resample_buf;
    out_count = swr_convert(cr->swr,
                            &out_ptr, max_out,
                            &in_ptr,  in_samples);
    if (out_count < 0)
        return out_count;
    if (out_count == 0)
        return 0;

    if (cr->calibrated) {
        cr->total_samples_in  += in_samples;
        cr->total_samples_out += out_count;

        /* ASRC feedback update is called from the pacer delivery
         * point (ipmx_clockrec_fb_update), not here — it needs the
         * PTS which is only known at delivery time. */
    }

    new_size = out_count * frame_size;
    new_buf  = av_buffer_alloc(new_size + AV_INPUT_BUFFER_PADDING_SIZE);
    if (!new_buf)
        return AVERROR(ENOMEM);
    memcpy(new_buf->data, cr->resample_buf, new_size);
    memset(new_buf->data + new_size, 0, AV_INPUT_BUFFER_PADDING_SIZE);

    av_buffer_unref(&pkt->buf);
    pkt->buf  = new_buf;
    pkt->data = new_buf->data;
    pkt->size = new_size;

    /* Log periodically (only after calibration when data is meaningful). */
    if (cr->log_fp && cr->calibrated) {
        int64_t now = av_gettime_relative();
        if (cr->resample_last_log_us == 0 ||
            now - cr->resample_last_log_us >= CLOCKREC_LOG_INTERVAL_US) {
            cr->resample_last_log_us = now;
            fprintf(cr->log_fp,
                    "wall_us=%lld CLOCKREC: drift=%.2f ppm  rate_corr=%.2f ppm  "
                    "rate_raw=%.2f ppm  effective=%.2f ppm  "
                    "slope=%.9f  intercept=%.0f  "
                    "delta=%d/%d  accum=%.4f  "
                    "in=%lld out=%lld  rx=%lld tx=%lld  "
                    "vrx=%lld vtx=%lld  fb_corr=%.2f ppm  ramp=%s\n",
                    (long long)now,
                    cr->drift_ppm,
                    cr->queue_correction_ppm,
                    cr->rate_correction_ppm_raw,
                    effective_ppm,
                    cr->pll_slope, cr->pll_intercept,
                    sample_delta, in_samples,
                    cr->delta_accumulator,
                    (long long)cr->total_samples_in,
                    (long long)cr->total_samples_out,
                    (long long)cr->rate_rx_samples,
                    (long long)cr->rate_tx_samples,
                    (long long)cr->video_rx_frames,
                    (long long)cr->video_tx_frames,
#if CLOCKREC_ASRC_FEEDBACK
                    cr->fb_correction_ppm,
#else
                    0.0,
#endif
                    cr->ramp_start_us > 0 ? "active" : "done");
        }
    }

    return 0;
}

/* -----------------------------------------------------------------------
 * ASRC feedback: rate-based bias correction
 *
 * Compares input sample rate (samples / arrival-time span) to output
 * sample rate (samples / PTS span).  The rate error in ppm is fed to a
 * fixed-point Q32 EWMA.  After warmup, the smoothed error is applied
 * as a correction to effective_ppm.
 *
 * Called from the pacer delivery point for each audio packet, where
 * both arrival_us (from opaque_ref) and pts (absolute) are known.
 * ----------------------------------------------------------------------- */

#if CLOCKREC_ASRC_FEEDBACK
void ipmx_clockrec_fb_update(IpmxClockRecovery *cr)
{
    int64_t in_since, out_since;

    if (!cr->calibrated)
        return;

    /* Record calibration time on first call */
    if (!cr->fb_cal_wall_us)
        cr->fb_cal_wall_us = av_gettime_relative();

    /* Warmup: let PLL and ASRC settle before measuring */
    if (!cr->fb_active) {
        if (av_gettime_relative() - cr->fb_cal_wall_us < CLOCKREC_FB_WARMUP_US)
            return;
        /* Warmup complete — snapshot the starting point. */
        cr->fb_in_samples  = cr->total_samples_in;
        cr->fb_out_samples = cr->total_samples_out;
        cr->fb_active      = 1;
        return;
    }

    /* Saturating I-controller on cumulative sample imbalance.
     *
     * (out_since - in_since) is the integral of the instantaneous rate
     * error — every excess or deficit sample is counted.  Scaling by a
     * fixed gain (CLOCKREC_FB_KP) and clamping to ±CLOCKREC_FB_MAX_PPM
     * gives constant correction authority: 1 excess sample produces the
     * same ppm nudge whether it happened 1 second or 1 hour ago.
     *
     * The clamp makes this a bang-bang controller beyond ±1 sample of
     * error (with KP=5, MAX=5).  The resulting limit-cycle has ~1 sample
     * amplitude and a period of seconds — well below audibility. */
    in_since  = cr->total_samples_in  - cr->fb_in_samples;
    out_since = cr->total_samples_out - cr->fb_out_samples;
    if (in_since <= 0)
        return;

    cr->fb_correction_ppm = (double)(out_since - in_since) * CLOCKREC_FB_KP;
    if (cr->fb_correction_ppm > CLOCKREC_FB_MAX_PPM)
        cr->fb_correction_ppm = CLOCKREC_FB_MAX_PPM;
    if (cr->fb_correction_ppm < -CLOCKREC_FB_MAX_PPM)
        cr->fb_correction_ppm = -CLOCKREC_FB_MAX_PPM;
}
#endif

/* -----------------------------------------------------------------------
 * Video frame skip / repeat decisions
 * ----------------------------------------------------------------------- */

int ipmx_clockrec_video_pace_decision(IpmxClockRecovery *cr,
                                      int64_t pts, int64_t now_us)
{
    int64_t t_video;
    int64_t e;

    if (!cr->clock_recovery || !cr->calibrated)
        return 0;
    if (cr->video_repeat_pts_inc <= 0)
        return 0;

    t_video = cr->video_repeat_pts_inc;  /* frame period in us */
    e = now_us - pts;                    /* positive = late, negative = early */

    /* Skip: frame is stale */
    if (e > (int64_t)(CLOCKREC_VIDEO_SKIP_FRAC * t_video)) {
        if (cr->video_consecutive_skips >= CLOCKREC_VIDEO_MAX_CONSECUTIVE_SKIP)
            return 0;  /* consecutive limit: deliver instead */
        return 1;
    }

    /* Repeat: too early, hold back */
    if (e < -(int64_t)(CLOCKREC_VIDEO_REPEAT_FRAC * t_video))
        return -1;

    /* Normal: on time (or within the 1-T_video window) */
    return 0;
}

int64_t ipmx_clockrec_reconstruct_ts(IpmxClockRecovery *cr,
                                     int64_t sender_us, int is_audio)
{
    (void)is_audio;  /* affine model is shared across audio/video */

    if (!cr->clock_recovery || !cr->mid_calibrated)
        return sender_us;

    /*
     * Full affine model: r̂(s) = â · s + b̂
     *
     * Maps the sender's STARTFRAME-based timestamp directly to the
     * receiver's wall-clock domain.  Both slope (â = frequency ratio)
     * and intercept (b̂ = phase offset) are recovered from the regression.
     *
     * No per-media state, no accumulator, no epoch needed.
     */
    return (int64_t)lround(cr->pll_slope * (double)sender_us +
                           cr->pll_intercept);
}

/* Pacer feedback removed: the affine model's intercept (b_hat) provides
 * continuous phase tracking, eliminating the need for a PI feedback loop. */
