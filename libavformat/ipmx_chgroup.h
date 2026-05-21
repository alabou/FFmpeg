/*
 * IPMX channel-order group helper (SMPTE ST 2110-30 / ST 2110-31).
 *
 * Parses the SMPTE2110.(token,token,…) body of the SDP fmtp `channel-order=`
 * parameter into an array of "signal groups", and helps depacketizers
 * allocate one AVStream per group.
 *
 * Each token in the channel-order denotes a logical signal (one program /
 * one carrier). The mapping is:
 *
 *   M    → 1 group: PCM,  1 ch (mono)
 *   ST   → 1 group: PCM,  2 ch (stereo)
 *   DM   → 2 groups: PCM,  1 ch each (two independent mono signals)
 *   LtRt → 1 group: PCM,  2 ch (matrix-encoded stereo)
 *   51   → 1 group: PCM,  6 ch (5.1)
 *   71   → 1 group: PCM,  8 ch (7.1)
 *   222  → 1 group: PCM, 24 ch (22.2)
 *   SGRP → 2 groups: PCM,  2 ch each (SDI group: two stereo pairs)
 *   AES3 → 1 group: non-PCM, 2 ch (S337M-wrapped compressed audio)
 *
 * Used by:
 *   - rtpdec_am824.c (always; AM824 is a SMPTE 2110-31 / IPMX-only payload).
 *   - The IPMX-aware L24/L16 path (only when AVFormatContext::ipmx == 1).
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

#ifndef AVFORMAT_IPMX_CHGROUP_H
#define AVFORMAT_IPMX_CHGROUP_H

#include "libavutil/channel_layout.h"
#include "libavcodec/codec_id.h"
#include "avformat.h"

#define IPMX_MAX_CHGROUPS 32  /* well above any realistic SMPTE 2110-31 layout */

typedef enum IpmxChGroupKind {
    IPMX_CHGROUP_PCM,    /**< linear PCM, channel layout known from token */
    IPMX_CHGROUP_AES3,   /**< AES3 pair, codec NONE until S337M-detected */
} IpmxChGroupKind;

typedef struct IpmxChGroup {
    IpmxChGroupKind   kind;
    int               nchannels;       /**< channels per signal in this group */
    int               channel_offset;  /**< first channel of this group within
                                         *  the full interleaved wire frame */
    AVChannelLayout   layout;          /**< layout for PCM groups (AES3 = stereo) */
    int               stream_idx;      /**< AVStream index allocated by
                                         *  ff_ipmx_alloc_group_streams (-1
                                         *  before allocation) */
} IpmxChGroup;

/**
 * Parse the body of `SMPTE2110.(…)` (the substring between `(` and `)`) into
 * `groups[]`. Returns the number of groups produced (>= 0) on success or a
 * negative AVERROR. `*total_channels` (if non-NULL) is filled with the sum
 * of channels across all groups, which must match the rtpmap channel count.
 *
 * Unknown tokens are tolerated: a one-line warning is logged via @p log_ctx
 * (may be NULL) and the token contributes one stereo PCM group, mirroring
 * the legacy depacketizer behaviour.
 */
int ff_ipmx_parse_chorder(void *log_ctx, const char *body,
                          IpmxChGroup *groups, int max_groups,
                          int *total_channels);

/**
 * Allocate one AVStream per group. Reuses `st0` for groups[0]; calls
 * `avformat_new_stream` for groups[1..n_groups-1]. Sets per-stream
 * codec_type / sample_rate / time_base / ch_layout / codec_id. PCM groups
 * get `pcm_codec_id` (e.g. AV_CODEC_ID_PCM_S32BE) plus the supplied
 * bits_per_coded_sample / bits_per_raw_sample. AES3 groups get
 * codec_id = AV_CODEC_ID_NONE (the depacketizer is expected to upgrade
 * the codec_id once the S337M data-type field is decoded from the first
 * burst).
 *
 * After this function each group's `stream_idx` is set to the AVStream
 * index of the matching stream.
 */
int ff_ipmx_alloc_group_streams(AVFormatContext *s, AVStream *st0,
                                IpmxChGroup *groups, int n_groups,
                                enum AVCodecID pcm_codec_id,
                                int bits_per_coded_sample,
                                int bits_per_raw_sample);

#endif /* AVFORMAT_IPMX_CHGROUP_H */
