/*
 * IPMX channel-order group helper (SMPTE ST 2110-30 / ST 2110-31).
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

#include "ipmx_chgroup.h"
#include "internal.h"
#include "libavutil/avstring.h"
#include "libavutil/channel_layout.h"
#include "libavutil/error.h"
#include "libavutil/log.h"

/* Token table for SMPTE 2110-31 §6.2.2 channel-order groups. The set is
 * intentionally small — these are the only tokens defined by the spec and
 * the only ones the IPMX sample muxer emits. Unrecognised tokens fall back
 * to a single stereo PCM group (compat with the legacy AM824 depacketizer
 * behaviour, which counted unknown tokens as one pair). */
typedef struct ChToken {
    const char       *name;
    int               len;
    int               n_groups;        /* DM / SGRP expand to >1 group */
    int               channels_per_grp;
    IpmxChGroupKind   kind;
    AVChannelLayout   layout;
} ChToken;

#define LAYOUT_MONO     AV_CHANNEL_LAYOUT_MONO
#define LAYOUT_STEREO   AV_CHANNEL_LAYOUT_STEREO
#define LAYOUT_5POINT1  AV_CHANNEL_LAYOUT_5POINT1
#define LAYOUT_7POINT1  AV_CHANNEL_LAYOUT_7POINT1
#define LAYOUT_22POINT2 AV_CHANNEL_LAYOUT_22POINT2

static const ChToken ch_tokens[] = {
    { "M",    1, 1, 1, IPMX_CHGROUP_PCM,  LAYOUT_MONO     },
    { "ST",   2, 1, 2, IPMX_CHGROUP_PCM,  LAYOUT_STEREO   },
    { "DM",   2, 2, 1, IPMX_CHGROUP_PCM,  LAYOUT_MONO     },
    { "LtRt", 4, 1, 2, IPMX_CHGROUP_PCM,  LAYOUT_STEREO   },
    { "51",   2, 1, 6, IPMX_CHGROUP_PCM,  LAYOUT_5POINT1  },
    { "71",   2, 1, 8, IPMX_CHGROUP_PCM,  LAYOUT_7POINT1  },
    { "222",  3, 1,24, IPMX_CHGROUP_PCM,  LAYOUT_22POINT2 },
    { "SGRP", 4, 2, 2, IPMX_CHGROUP_PCM,  LAYOUT_STEREO   },
    { "AES3", 4, 1, 2, IPMX_CHGROUP_AES3, LAYOUT_STEREO   },
    { NULL }
};

static const ChToken fallback_token = {
    "?",  1, 1, 2, IPMX_CHGROUP_PCM, LAYOUT_STEREO
};

static const ChToken *match_token(const char *p, int len)
{
    int i;
    for (i = 0; ch_tokens[i].name; i++) {
        if (ch_tokens[i].len == len &&
            !av_strncasecmp(p, ch_tokens[i].name, len))
            return &ch_tokens[i];
    }
    return NULL;
}

int ff_ipmx_parse_chorder(void *log_ctx, const char *body,
                          IpmxChGroup *groups, int max_groups,
                          int *total_channels)
{
    int n_groups = 0;
    int ch_offset = 0;
    const char *p = body;

    if (!body || !groups || max_groups <= 0)
        return AVERROR(EINVAL);

    while (*p && *p != ')') {
        const char *end;
        const ChToken *tok;
        int g;

        /* Skip leading whitespace inside the parens (some encoders insert it). */
        while (*p == ' ' || *p == '\t')
            p++;
        if (*p == ')' || !*p)
            break;

        end = p;
        while (*end && *end != ',' && *end != ')')
            end++;

        tok = match_token(p, (int)(end - p));
        if (!tok) {
            char tokbuf[16];
            int tlen = (int)(end - p);
            if (tlen > (int)sizeof(tokbuf) - 1)
                tlen = sizeof(tokbuf) - 1;
            memcpy(tokbuf, p, tlen);
            tokbuf[tlen] = '\0';
            av_log(log_ctx, AV_LOG_WARNING,
                   "IPMX channel-order: unknown token \"%s\" — treating as "
                   "stereo PCM group\n", tokbuf);
            tok = &fallback_token;
        }

        for (g = 0; g < tok->n_groups; g++) {
            if (n_groups >= max_groups) {
                av_log(log_ctx, AV_LOG_ERROR,
                       "IPMX channel-order: too many groups (max %d)\n",
                       max_groups);
                return AVERROR_INVALIDDATA;
            }
            groups[n_groups].kind            = tok->kind;
            groups[n_groups].nchannels       = tok->channels_per_grp;
            groups[n_groups].channel_offset  = ch_offset;
            groups[n_groups].layout          = tok->layout;
            groups[n_groups].stream_idx      = -1;
            ch_offset += tok->channels_per_grp;
            n_groups++;
        }

        p = end;
        if (*p == ',')
            p++;
    }

    if (total_channels)
        *total_channels = ch_offset;
    return n_groups;
}

int ff_ipmx_alloc_group_streams(AVFormatContext *s, AVStream *st0,
                                IpmxChGroup *groups, int n_groups,
                                enum AVCodecID pcm_codec_id,
                                int bits_per_coded_sample,
                                int bits_per_raw_sample)
{
    int i, ret;
    int sr;

    if (!s || !st0 || !groups || n_groups <= 0)
        return AVERROR(EINVAL);

    sr = st0->codecpar->sample_rate;

    for (i = 0; i < n_groups; i++) {
        AVStream *st;

        if (i == 0) {
            st = st0;
        } else {
            st = avformat_new_stream(s, NULL);
            if (!st)
                return AVERROR(ENOMEM);
            st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
            st->codecpar->sample_rate = sr;
            avpriv_set_pts_info(st, 32, 1, sr);
        }

        groups[i].stream_idx = st->index;

        ret = av_channel_layout_copy(&st->codecpar->ch_layout,
                                     &groups[i].layout);
        if (ret < 0)
            return ret;

        if (groups[i].kind == IPMX_CHGROUP_AES3) {
            /* The S337M data-type field in the first burst will upgrade
             * codec_id to AC-3 / AAC / EAC3 / etc. */
            st->codecpar->codec_id = AV_CODEC_ID_NONE;
        } else {
            st->codecpar->codec_id              = pcm_codec_id;
            st->codecpar->bits_per_coded_sample = bits_per_coded_sample;
            st->codecpar->bits_per_raw_sample   = bits_per_raw_sample;
        }
    }

    return 0;
}
