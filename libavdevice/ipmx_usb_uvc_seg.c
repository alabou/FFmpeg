/*
 * IPMX USB UVC frame segmenter for Sender device.
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

#include "ipmx_usb_uvc_seg.h"
#include "ipmx_usb_quirks.h"
#include <string.h>

#define UVC_HDR_SIZE_NOPT     2   /* bHeaderLength + bmHeaderInfo */
#define UVC_HDR_SIZE_PTS      6   /* + 4-byte dwPresentationTime */
#define UVC_HDR_SIZE_SCR      8   /* + 6-byte SCR (4 STC + 2 SOF) */
#define UVC_HDR_SIZE_PTS_SCR 12   /* + 4-byte PTS + 6-byte SCR */

void ipmx_uvc_seg_init(IpmxUvcSegmenter *seg, uint16_t max_packet_size,
                        uint32_t quirks)
{
    memset(seg, 0, sizeof(*seg));
    seg->max_packet_size = max_packet_size > UVC_HDR_SIZE_PTS_SCR
                           ? max_packet_size : 1024;
    seg->fid    = 0;
    seg->quirks = quirks;
}

void ipmx_uvc_seg_begin_frame(IpmxUvcSegmenter *seg,
                               const uint8_t *frame_data, int frame_size,
                               int64_t pts_us,
                               int has_scr, uint32_t stc, uint16_t sof)
{
    seg->frame_data   = frame_data;
    seg->frame_size   = frame_size;
    seg->frame_offset = 0;
    seg->frame_done   = 0;
    seg->has_pts      = (pts_us >= 0);
    seg->frame_pts    = (uint32_t)(pts_us & 0xFFFFFFFF);
    seg->has_scr      = has_scr;
    seg->frame_stc    = stc;
    seg->frame_sof    = sof;
    if (!(seg->quirks & IPMX_UVC_QUIRK_STREAM_NO_FID))
        seg->fid ^= 1;
}

int ipmx_uvc_seg_next_packet(IpmxUvcSegmenter *seg,
                              uint8_t *out_buf, int out_buf_size,
                              int *out_len, int *is_last)
{
    int hdr_size, payload_capacity, chunk, remaining;
    uint8_t bfh;

    *out_len = 0;
    *is_last = 0;

    if (seg->frame_done)
        return 1;

    if (seg->has_pts && seg->has_scr)
        hdr_size = UVC_HDR_SIZE_PTS_SCR;
    else if (seg->has_pts)
        hdr_size = UVC_HDR_SIZE_PTS;
    else if (seg->has_scr)
        hdr_size = UVC_HDR_SIZE_SCR;
    else
        hdr_size = UVC_HDR_SIZE_NOPT;

    if (out_buf_size < hdr_size)
        return -1;

    payload_capacity = seg->max_packet_size - hdr_size;
    if (payload_capacity <= 0)
        payload_capacity = 1;
    if (payload_capacity > out_buf_size - hdr_size)
        payload_capacity = out_buf_size - hdr_size;

    remaining = seg->frame_size - seg->frame_offset;
    chunk = remaining < payload_capacity ? remaining : payload_capacity;

    bfh = UVC_SEG_BFH_EOH;
    if (!(seg->quirks & IPMX_UVC_QUIRK_STREAM_NO_FID))
        bfh |= (seg->fid & 1);

    if (seg->frame_offset + chunk >= seg->frame_size) {
        if (!(seg->quirks & IPMX_UVC_QUIRK_MJPEG_NO_EOF))
            bfh |= UVC_SEG_BFH_EOF;
        *is_last = 1;
        seg->frame_done = 1;
    }

    {
        int pos = 2;
        if (seg->has_pts) {
            bfh |= UVC_SEG_BFH_PTS;
            out_buf[pos++] = (uint8_t)(seg->frame_pts >>  0);
            out_buf[pos++] = (uint8_t)(seg->frame_pts >>  8);
            out_buf[pos++] = (uint8_t)(seg->frame_pts >> 16);
            out_buf[pos++] = (uint8_t)(seg->frame_pts >> 24);
        }
        if (seg->has_scr) {
            bfh |= UVC_SEG_BFH_SCR;
            out_buf[pos++] = (uint8_t)(seg->frame_stc >>  0);
            out_buf[pos++] = (uint8_t)(seg->frame_stc >>  8);
            out_buf[pos++] = (uint8_t)(seg->frame_stc >> 16);
            out_buf[pos++] = (uint8_t)(seg->frame_stc >> 24);
            out_buf[pos++] = (uint8_t)(seg->frame_sof >>  0);
            out_buf[pos++] = (uint8_t)(seg->frame_sof >>  8);
        }
        out_buf[0] = (uint8_t)hdr_size;
        out_buf[1] = bfh;
    }

    if (chunk > 0)
        memcpy(out_buf + hdr_size,
               seg->frame_data + seg->frame_offset, chunk);

    seg->frame_offset += chunk;
    *out_len = hdr_size + chunk;
    return 0;
}
