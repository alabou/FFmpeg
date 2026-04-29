/*
 * IPMX UVC (USB Video Class) protocol handling for FFmpeg.
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

#include "ipmx_usb_uvc.h"
#include "ipmx_usb_quirks.h"
#include <stdlib.h>
#include <string.h>

/* USB descriptor types */
#define USB_DT_INTERFACE            0x04
#define USB_DT_ENDPOINT             0x05
#define USB_DT_CS_INTERFACE         0x24

/* UVC payload header bits (bmHeaderInfo) */
#define UVC_HDR_FID                 0x01
#define UVC_HDR_EOF                 0x02
#define UVC_HDR_PTS                 0x04
#define UVC_HDR_SCR                 0x08
#define UVC_HDR_STI                 0x20

/* 100ns units per second */
#define UVC_100NS_PER_SEC           10000000

/* ---------------------------------------------------------------------------
 * ipmx_uvc_parse_config
 * --------------------------------------------------------------------------- */

int ipmx_uvc_parse_config(IpmxUvcContext *ctx, const uint8_t *desc, int desc_len)
{
    int pos = 0;
    int in_vc = 0;
    int in_vs_alt = 0;
    uint8_t cur_format_idx = 0;
    uint8_t cur_frame_count = 0;

    if (!ctx || !desc || desc_len < 2)
        return -1;

    memset(ctx, 0, sizeof(*ctx));
    ctx->vc_interface = 0xFF;
    ctx->vs_interface = 0xFF;
    ctx->vs_endpoint = 0xFF;

    while (pos + 2 <= desc_len) {
        uint8_t bLength = desc[pos];
        uint8_t bDescriptorType = desc[pos + 1];

        if (bLength < 2 || pos + bLength > desc_len)
            break;

        if (bDescriptorType == USB_DT_INTERFACE) {
            uint8_t bInterfaceNumber, bInterfaceClass, bInterfaceSubClass;
            if (bLength < 9)
                goto next;
            bInterfaceNumber = desc[pos + 2];
            bInterfaceClass = desc[pos + 5];
            bInterfaceSubClass = desc[pos + 6];

            if (bInterfaceClass != UVC_CC_VIDEO) {
                in_vc = 0;
                in_vs_alt = 0;
                goto next;
            }

            if (bInterfaceSubClass == UVC_SC_VIDEOCONTROL) {
                ctx->vc_interface = bInterfaceNumber;
                in_vc = 1;
                in_vs_alt = 0;
            } else if (bInterfaceSubClass == UVC_SC_VIDEOSTREAMING) {
                ctx->vs_interface = bInterfaceNumber;
                in_vc = 0;
                in_vs_alt = 1;
                cur_format_idx = 0;
                cur_frame_count = 0;
            } else {
                in_vc = 0;
                in_vs_alt = 0;
            }
        } else if (bDescriptorType == USB_DT_ENDPOINT && in_vs_alt) {
            uint8_t bEndpointAddress, bmAttributes;
            uint16_t wMaxPacketSize;
            if (bLength < 7)
                goto next;
            bEndpointAddress = desc[pos + 2];
            bmAttributes = desc[pos + 3];
            wMaxPacketSize = desc[pos + 4] | (desc[pos + 5] << 8);

            /* Isochronous IN endpoint: (bmAttributes & 0x03) == 0x01 or 0x05 */
            if ((bmAttributes & 0x03) == 0x01 && (bEndpointAddress & 0x80)) {
                ctx->vs_endpoint = bEndpointAddress & 0x0F;
                ctx->max_packet_size = wMaxPacketSize;
            }
        } else if (bDescriptorType == USB_DT_CS_INTERFACE && in_vc) {
            /* VC Header (subtype 0x01): extract dwClockFrequency at bytes 7-10 */
            if (bLength >= 13 && desc[pos + 2] == UVC_DT_HEADER) {
                ctx->clock_frequency = (uint32_t)desc[pos + 7]
                                     | ((uint32_t)desc[pos + 8]  << 8)
                                     | ((uint32_t)desc[pos + 9]  << 16)
                                     | ((uint32_t)desc[pos + 10] << 24);
            }
        } else if (bDescriptorType == USB_DT_CS_INTERFACE && in_vs_alt) {
            uint8_t bDescriptorSubtype;
            if (bLength < 3)
                goto next;
            bDescriptorSubtype = desc[pos + 2];

            switch (bDescriptorSubtype) {
            case UVC_VS_INPUT_HEADER:
                /* Just marks start of VS descriptors; no extra parsing needed */
                break;

            case UVC_VS_FORMAT_MJPEG:
                if (ctx->num_formats >= IPMX_UVC_MAX_FORMATS)
                    break;
                ctx->formats[ctx->num_formats].format_index =
                    (bLength >= 4) ? desc[pos + 3] : (ctx->num_formats + 1);
                ctx->formats[ctx->num_formats].format_type = UVC_VS_FORMAT_MJPEG;
                ctx->formats[ctx->num_formats].num_frames =
                    (bLength >= 5) ? desc[pos + 4] : 0;
                cur_format_idx = ctx->num_formats;
                cur_frame_count = 0;
                ctx->num_formats++;
                break;

            case UVC_VS_FRAME_MJPEG: {
                IpmxUvcFormatInfo *fmt;
                if (cur_format_idx >= ctx->num_formats)
                    break;
                fmt = &ctx->formats[cur_format_idx];
                if (fmt->format_type != UVC_VS_FORMAT_MJPEG ||
                    cur_frame_count >= IPMX_UVC_MAX_FRAMES)
                    break;
                if (bLength >= 26) {
                    IpmxUvcFrameInfo *fi = &fmt->frames[cur_frame_count];
                    fi->width = desc[pos + 5] | (desc[pos + 6] << 8);
                    fi->height = desc[pos + 7] | (desc[pos + 8] << 8);
                    fi->max_frame_size = desc[pos + 17] | (desc[pos + 18] << 8) |
                        (desc[pos + 19] << 16) | (desc[pos + 20] << 24);
                    fi->default_interval = desc[pos + 21] | (desc[pos + 22] << 8) |
                        (desc[pos + 23] << 16) | (desc[pos + 24] << 24);
                    cur_frame_count++;
                }
                break;
            }

            case UVC_VS_FORMAT_UNCOMPRESSED:
                if (ctx->num_formats >= IPMX_UVC_MAX_FORMATS)
                    break;
                ctx->formats[ctx->num_formats].format_index =
                    (bLength >= 4) ? desc[pos + 3] : (ctx->num_formats + 1);
                ctx->formats[ctx->num_formats].format_type = UVC_VS_FORMAT_UNCOMPRESSED;
                if (bLength >= 24) {
                    memcpy(ctx->formats[ctx->num_formats].guid, &desc[pos + 5], 16);
                    ctx->formats[ctx->num_formats].bits_per_pixel = desc[pos + 21];
                }
                ctx->formats[ctx->num_formats].num_frames =
                    (bLength >= 24) ? desc[pos + 22] : 0;
                cur_format_idx = ctx->num_formats;
                cur_frame_count = 0;
                ctx->num_formats++;
                break;

            case UVC_VS_FRAME_UNCOMPRESSED: {
                IpmxUvcFormatInfo *fmt;
                if (cur_format_idx >= ctx->num_formats)
                    break;
                fmt = &ctx->formats[cur_format_idx];
                if (fmt->format_type != UVC_VS_FORMAT_UNCOMPRESSED ||
                    cur_frame_count >= IPMX_UVC_MAX_FRAMES)
                    break;
                if (bLength >= 26) {
                    IpmxUvcFrameInfo *fi = &fmt->frames[cur_frame_count];
                    fi->width = desc[pos + 5] | (desc[pos + 6] << 8);
                    fi->height = desc[pos + 7] | (desc[pos + 8] << 8);
                    fi->max_frame_size = desc[pos + 17] | (desc[pos + 18] << 8) |
                        (desc[pos + 19] << 16) | (desc[pos + 20] << 24);
                    fi->default_interval = desc[pos + 21] | (desc[pos + 22] << 8) |
                        (desc[pos + 23] << 16) | (desc[pos + 24] << 24);
                    cur_frame_count++;
                }
                break;
            }

            case UVC_VS_FORMAT_FRAME_BASED:
                if (ctx->num_formats >= IPMX_UVC_MAX_FORMATS)
                    break;
                ctx->formats[ctx->num_formats].format_index =
                    (bLength >= 4) ? desc[pos + 3] : (ctx->num_formats + 1);
                ctx->formats[ctx->num_formats].format_type = UVC_VS_FORMAT_FRAME_BASED;
                if (bLength >= 24) {
                    memcpy(ctx->formats[ctx->num_formats].guid, &desc[pos + 5], 16);
                    ctx->formats[ctx->num_formats].bits_per_pixel = desc[pos + 21];
                }
                ctx->formats[ctx->num_formats].num_frames =
                    (bLength >= 5) ? desc[pos + 4] : 0;
                cur_format_idx = ctx->num_formats;
                cur_frame_count = 0;
                ctx->num_formats++;
                break;

            case UVC_VS_FRAME_FRAME_BASED: {
                IpmxUvcFormatInfo *fmt;
                if (cur_format_idx >= ctx->num_formats)
                    break;
                fmt = &ctx->formats[cur_format_idx];
                if (fmt->format_type != UVC_VS_FORMAT_FRAME_BASED ||
                    cur_frame_count >= IPMX_UVC_MAX_FRAMES)
                    break;
                /* Frame-based layout differs: offset 17 = dwDefaultFrameInterval
                 * (no dwMaxVideoFrameBufferSize field) */
                if (bLength >= 22) {
                    IpmxUvcFrameInfo *fi = &fmt->frames[cur_frame_count];
                    fi->width = desc[pos + 5] | (desc[pos + 6] << 8);
                    fi->height = desc[pos + 7] | (desc[pos + 8] << 8);
                    fi->max_frame_size = 0;
                    fi->default_interval = desc[pos + 17] | (desc[pos + 18] << 8) |
                        (desc[pos + 19] << 16) | (desc[pos + 20] << 24);
                    cur_frame_count++;
                }
                break;
            }

            case UVC_VS_COLOR_MATCHING:
                /* Ignore */
                break;

            default:
                break;
            }
        }

    next:
        pos += bLength;
    }

    if (ctx->vc_interface == 0xFF || ctx->vs_interface == 0xFF ||
        ctx->vs_endpoint == 0xFF || ctx->num_formats == 0)
        return -1;

    return 0;
}

/* ---------------------------------------------------------------------------
 * ipmx_uvc_build_probe_control
 * --------------------------------------------------------------------------- */

int ipmx_uvc_build_probe_control(uint8_t *buf, int buf_size,
                                 uint8_t format_idx, uint8_t frame_idx,
                                 uint32_t interval)
{
    if (!buf || buf_size < 26)
        return -1;

    memset(buf, 0, 26);
    buf[2] = format_idx;
    buf[3] = frame_idx;
    buf[4] = (uint8_t)(interval);
    buf[5] = (uint8_t)(interval >> 8);
    buf[6] = (uint8_t)(interval >> 16);
    buf[7] = (uint8_t)(interval >> 24);
    return 26;
}

/* ---------------------------------------------------------------------------
 * ipmx_uvc_build_set_cur_req / ipmx_uvc_build_get_cur_req
 * --------------------------------------------------------------------------- */

int ipmx_uvc_build_set_cur_req(uint8_t *buf, uint8_t cs, uint8_t interface,
                               uint16_t length)
{
    if (!buf)
        return -1;

    buf[0] = 0x21;  /* Host-to-device, Class, Interface */
    buf[1] = UVC_SET_CUR;
    buf[2] = 0;           /* wValue low byte */
    buf[3] = cs;          /* wValue high byte = (cs << 8) */
    buf[4] = interface;  /* wIndex low byte */
    buf[5] = 0;           /* wIndex high byte */
    buf[6] = length & 0xFF;
    buf[7] = (length >> 8) & 0xFF;
    return 8;
}

int ipmx_uvc_build_get_cur_req(uint8_t *buf, uint8_t cs, uint8_t interface,
                               uint16_t length)
{
    if (!buf)
        return -1;

    buf[0] = 0xA1;  /* Device-to-host, Class, Interface */
    buf[1] = UVC_GET_CUR;
    buf[2] = 0;           /* wValue low byte */
    buf[3] = cs;          /* wValue high byte = (cs << 8) */
    buf[4] = interface;  /* wIndex low byte */
    buf[5] = 0;           /* wIndex high byte */
    buf[6] = length & 0xFF;
    buf[7] = (length >> 8) & 0xFF;
    return 8;
}

int ipmx_uvc_build_get_def_req(uint8_t *buf, uint8_t cs, uint8_t interface,
                               uint16_t length)
{
    if (!buf)
        return -1;

    buf[0] = 0xA1;
    buf[1] = UVC_GET_DEF;
    buf[2] = 0;
    buf[3] = cs;
    buf[4] = interface;
    buf[5] = 0;
    buf[6] = length & 0xFF;
    buf[7] = (length >> 8) & 0xFF;
    return 8;
}

int ipmx_uvc_build_get_min_req(uint8_t *buf, uint8_t cs, uint8_t interface,
                               uint16_t length)
{
    if (!buf)
        return -1;

    buf[0] = 0xA1;
    buf[1] = UVC_GET_MIN;
    buf[2] = 0;
    buf[3] = cs;
    buf[4] = interface;
    buf[5] = 0;
    buf[6] = length & 0xFF;
    buf[7] = (length >> 8) & 0xFF;
    return 8;
}

int ipmx_uvc_build_get_max_req(uint8_t *buf, uint8_t cs, uint8_t interface,
                               uint16_t length)
{
    if (!buf)
        return -1;

    buf[0] = 0xA1;
    buf[1] = UVC_GET_MAX;
    buf[2] = 0;
    buf[3] = cs;
    buf[4] = interface;
    buf[5] = 0;
    buf[6] = length & 0xFF;
    buf[7] = (length >> 8) & 0xFF;
    return 8;
}

/* ---------------------------------------------------------------------------
 * Reassembler init / free
 * --------------------------------------------------------------------------- */

int ipmx_uvc_init_reassembler(IpmxUvcContext *ctx)
{
    if (!ctx)
        return -1;

    ipmx_uvc_free_reassembler(ctx);
    ctx->frame_buf = (uint8_t *)malloc(IPMX_UVC_MAX_FRAME_BUF);
    ctx->done_buf  = (uint8_t *)malloc(IPMX_UVC_MAX_FRAME_BUF);
    if (!ctx->frame_buf || !ctx->done_buf) {
        free(ctx->frame_buf);
        free(ctx->done_buf);
        ctx->frame_buf = NULL;
        ctx->done_buf  = NULL;
        return -1;
    }

    ctx->frame_buf_size = IPMX_UVC_MAX_FRAME_BUF;
    ctx->frame_buf_pos  = 0;
    ctx->done_buf_size  = IPMX_UVC_MAX_FRAME_BUF;
    ctx->done_buf_pos   = 0;
    ctx->last_fid       = 0;
    ctx->frame_complete = 0;
    return 0;
}

void ipmx_uvc_free_reassembler(IpmxUvcContext *ctx)
{
    if (!ctx)
        return;
    free(ctx->frame_buf);
    free(ctx->done_buf);
    ctx->frame_buf  = NULL;
    ctx->done_buf   = NULL;
    ctx->frame_buf_size = 0;
    ctx->frame_buf_pos  = 0;
    ctx->done_buf_size  = 0;
    ctx->done_buf_pos   = 0;
    ctx->frame_complete = 0;
}

/* ---------------------------------------------------------------------------
 * ipmx_uvc_process_payload
 * --------------------------------------------------------------------------- */

static void complete_frame(IpmxUvcContext *ctx)
{
    if (ctx->frame_buf_pos > 0 && ctx->frame_buf_pos <= ctx->done_buf_size) {
        memcpy(ctx->done_buf, ctx->frame_buf, ctx->frame_buf_pos);
        ctx->done_buf_pos = ctx->frame_buf_pos;
    }
    ctx->done_frame_pts     = ctx->cur_frame_pts;
    ctx->done_frame_has_pts = ctx->cur_frame_has_pts;
    ctx->done_frame_stc     = ctx->cur_frame_stc;
    ctx->done_frame_sof     = ctx->cur_frame_sof;
    ctx->done_frame_has_scr = ctx->cur_frame_has_scr;
    ctx->frame_complete = 1;
}

static void start_new_frame(IpmxUvcContext *ctx)
{
    ctx->frame_buf_pos      = 0;
    ctx->cur_frame_has_pts  = 0;
    ctx->cur_frame_pts      = 0;
    ctx->cur_frame_has_scr  = 0;
    ctx->cur_frame_stc      = 0;
    ctx->cur_frame_sof      = 0;
}

int ipmx_uvc_process_payload(IpmxUvcContext *ctx, const uint8_t *data,
                             int data_len)
{
    int header_len;
    uint8_t bm_header_info;
    uint8_t fid;
    int eof;
    const uint8_t *payload;
    int payload_len;
    uint32_t quirks;

    if (!ctx || !data || data_len < 2 || !ctx->frame_buf)
        return -1;

    quirks = ctx->quirks;
    header_len = data[0];
    if (header_len < 2 || header_len > data_len)
        return -1;

    bm_header_info = data[1];
    fid = (bm_header_info & UVC_HDR_FID) ? 1 : 0;
    eof = (bm_header_info & UVC_HDR_EOF) ? 1 : 0;

    /* Extract PTS if present (bmHeaderInfo bit 2, header must be >= 6 bytes) */
    if ((bm_header_info & UVC_HDR_PTS) && header_len >= 6 && !ctx->cur_frame_has_pts) {
        ctx->cur_frame_pts = (uint32_t)data[2]
                           | ((uint32_t)data[3] << 8)
                           | ((uint32_t)data[4] << 16)
                           | ((uint32_t)data[5] << 24);
        ctx->cur_frame_has_pts = 1;
    }

    /* Extract SCR if present (bmHeaderInfo bit 3).
     * SCR layout (6 bytes, little-endian) follows PTS when present:
     *   PTS+SCR: bytes 6-9 = STC, bytes 10-11 = SOF  (header >= 12)
     *   SCR only: bytes 2-5 = STC, bytes 6-7 = SOF   (header >= 8)
     * SOF: bits 10:0 = 11-bit USB frame number, bits 15:11 = reserved */
    if ((bm_header_info & UVC_HDR_SCR) &&
        header_len >= ((bm_header_info & UVC_HDR_PTS) ? 12 : 8) &&
        !ctx->cur_frame_has_scr) {
        int scr_off = (bm_header_info & UVC_HDR_PTS) ? 6 : 2;
        ctx->cur_frame_stc = (uint32_t)data[scr_off]
                           | ((uint32_t)data[scr_off + 1] << 8)
                           | ((uint32_t)data[scr_off + 2] << 16)
                           | ((uint32_t)data[scr_off + 3] << 24);
        ctx->cur_frame_sof = ((uint16_t)data[scr_off + 4]
                           | ((uint16_t)data[scr_off + 5] << 8)) & 0x07FF;
        ctx->cur_frame_has_scr = 1;
    }

    payload = data + header_len;
    payload_len = data_len - header_len;

    if (payload_len <= 0)
        return 0;

    /* MJPEG_NO_EOF quirk: detect frame boundary via JPEG SOI (0xFF 0xD8) */
    if ((quirks & IPMX_UVC_QUIRK_MJPEG_NO_EOF) && ctx->frame_buf_pos > 0) {
        int i;
        for (i = 0; i < payload_len - 1; i++) {
            if (payload[i] == 0xFF && payload[i + 1] == 0xD8) {
                /* Append bytes before the SOI to the current frame */
                if (i > 0 &&
                    ctx->frame_buf_pos + i <= ctx->frame_buf_size) {
                    memcpy(ctx->frame_buf + ctx->frame_buf_pos,
                           payload, i);
                    ctx->frame_buf_pos += i;
                }
                complete_frame(ctx);
                start_new_frame(ctx);
                payload += i;
                payload_len -= i;
                break;
            }
        }
    }

    /* STREAM_NO_FID quirk: detect frame boundary via EOF only */
    if (quirks & IPMX_UVC_QUIRK_STREAM_NO_FID) {
        if (eof) {
            if (ctx->frame_buf_pos + payload_len > ctx->frame_buf_size)
                return -1;
            memcpy(ctx->frame_buf + ctx->frame_buf_pos, payload, payload_len);
            ctx->frame_buf_pos += payload_len;
            complete_frame(ctx);
        } else {
            if (ctx->frame_buf_pos + payload_len > ctx->frame_buf_size)
                return -1;
            memcpy(ctx->frame_buf + ctx->frame_buf_pos, payload, payload_len);
            ctx->frame_buf_pos += payload_len;
        }
        return 0;
    }

    /* Standard FID-based frame boundary detection */
    if (fid != ctx->last_fid) {
        /* FID toggled -> new frame; complete previous if we had data */
        if (ctx->frame_buf_pos > 0)
            complete_frame(ctx);
        start_new_frame(ctx);
        ctx->last_fid = fid;
    }

    if (ctx->frame_buf_pos + payload_len > ctx->frame_buf_size)
        return -1;
    memcpy(ctx->frame_buf + ctx->frame_buf_pos, payload, payload_len);
    ctx->frame_buf_pos += payload_len;

    if (eof)
        complete_frame(ctx);

    return 0;
}

/* ---------------------------------------------------------------------------
 * ipmx_uvc_frame_ready / ipmx_uvc_get_frame
 * --------------------------------------------------------------------------- */

int ipmx_uvc_frame_ready(IpmxUvcContext *ctx)
{
    return ctx ? ctx->frame_complete : 0;
}

int ipmx_uvc_get_frame(IpmxUvcContext *ctx, uint8_t **data, int *size)
{
    if (!ctx || !data || !size)
        return -1;

    *data = ctx->done_buf;
    *size = ctx->done_buf_pos;
    ctx->done_buf_pos = 0;
    ctx->frame_complete = 0;
    return 0;
}

int ipmx_uvc_get_frame_pts(IpmxUvcContext *ctx, uint32_t *pts_ticks)
{
    if (!ctx || !pts_ticks)
        return 0;
    if (ctx->done_frame_has_pts) {
        *pts_ticks = ctx->done_frame_pts;
        return 1;
    }
    return 0;
}

int ipmx_uvc_get_frame_scr(IpmxUvcContext *ctx, uint32_t *stc, uint16_t *sof)
{
    if (!ctx || !stc || !sof)
        return 0;
    if (ctx->done_frame_has_scr) {
        *stc = ctx->done_frame_stc;
        *sof = ctx->done_frame_sof;
        return 1;
    }
    return 0;
}

/* ---------------------------------------------------------------------------
 * ipmx_uvc_select_format
 * --------------------------------------------------------------------------- */

static uint32_t interval_from_fps(int fps)
{
    if (fps <= 0)
        return 0;
    return (uint32_t)(UVC_100NS_PER_SEC / fps);
}

static int format_type_bonus(const IpmxUvcFormatInfo *fmt)
{
    /* MJPEG > H.264 frame-based > uncompressed */
    if (fmt->format_type == UVC_VS_FORMAT_MJPEG)
        return 2000;
    if (fmt->format_type == UVC_VS_FORMAT_FRAME_BASED)
        return 1000;
    return 0;
}

static int score_format(const IpmxUvcFormatInfo *fmt, const IpmxUvcFrameInfo *fi,
                        int width, int height, int fps, uint32_t *out_interval)
{
    int dim_match;
    uint32_t want_interval, def_interval, chosen;

    dim_match = (fi->width == (uint16_t)width && fi->height == (uint16_t)height);
    if (!dim_match)
        return -1;

    want_interval = interval_from_fps(fps);
    def_interval = fi->default_interval;

    chosen = (want_interval > 0 && want_interval == def_interval)
        ? want_interval : def_interval;
    *out_interval = chosen;

    return 10000 + format_type_bonus(fmt) + (dim_match ? 100 : 0);
}

int ipmx_uvc_select_format(IpmxUvcContext *ctx, int width, int height, int fps)
{
    int best_fmt = -1;
    int best_frame = -1;
    int best_score = -1;
    uint32_t best_interval = 0;
    int f, r;

    if (!ctx || width <= 0 || height <= 0)
        return -1;

    for (f = 0; f < ctx->num_formats; f++) {
        const IpmxUvcFormatInfo *fmt = &ctx->formats[f];
        for (r = 0; r < fmt->num_frames; r++) {
            const IpmxUvcFrameInfo *fi = &fmt->frames[r];
            uint32_t interval;
            int score = score_format(fmt, fi, width, height, fps, &interval);
            if (score > best_score) {
                best_score = score;
                best_fmt = f;
                best_frame = r;
                best_interval = interval;
            }
        }
    }

    if (best_fmt < 0 || best_frame < 0)
        return -1;

    ctx->active_format = ctx->formats[best_fmt].format_index;
    ctx->active_frame = (uint8_t)(best_frame + 1);  /* UVC bFrameIndex is 1-based */
    ctx->active_interval = best_interval;
    return 0;
}

/* ---------------------------------------------------------------------------
 * Format identification helpers
 * --------------------------------------------------------------------------- */

int ipmx_uvc_is_h264(const IpmxUvcFormatInfo *fmt)
{
    if (!fmt || fmt->format_type != UVC_VS_FORMAT_FRAME_BASED)
        return 0;
    return memcmp(fmt->guid, UVC_GUID_FORMAT_H264, 16) == 0;
}

int ipmx_uvc_is_hevc(const IpmxUvcFormatInfo *fmt)
{
    if (!fmt || fmt->format_type != UVC_VS_FORMAT_FRAME_BASED)
        return 0;
    return memcmp(fmt->guid, UVC_GUID_FORMAT_HEVC, 16) == 0;
}

const char *ipmx_uvc_format_name(const IpmxUvcFormatInfo *fmt)
{
    if (!fmt)
        return "unknown";
    switch (fmt->format_type) {
    case UVC_VS_FORMAT_MJPEG:
        return "MJPEG";
    case UVC_VS_FORMAT_FRAME_BASED:
        if (memcmp(fmt->guid, UVC_GUID_FORMAT_H264, 16) == 0)
            return "H.264";
        if (memcmp(fmt->guid, UVC_GUID_FORMAT_HEVC, 16) == 0)
            return "HEVC";
        return "FrameBased";
    case UVC_VS_FORMAT_UNCOMPRESSED:
        if (memcmp(fmt->guid, UVC_GUID_FORMAT_YUY2, 16) == 0)
            return "YUY2";
        if (memcmp(fmt->guid, UVC_GUID_FORMAT_NV12, 16) == 0)
            return "NV12";
        return "RAW";
    default:
        return "unknown";
    }
}
