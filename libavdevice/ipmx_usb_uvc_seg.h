/*
 * IPMX USB UVC frame segmenter for Sender device.
 *
 * Inverse of ipmx_uvc_process_payload() (reassembler): takes a complete
 * video frame and fragments it into UVC isochronous payloads with 2-byte
 * payload headers, FID toggle, and EOF markers.
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

#ifndef AVDEVICE_IPMX_USB_UVC_SEG_H
#define AVDEVICE_IPMX_USB_UVC_SEG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* UVC payload header bits (bmHeaderInfo) */
#define UVC_SEG_BFH_FID  0x01
#define UVC_SEG_BFH_EOF  0x02
#define UVC_SEG_BFH_PTS  0x04
#define UVC_SEG_BFH_SCR  0x08
#define UVC_SEG_BFH_ERR  0x40
#define UVC_SEG_BFH_EOH  0x80

typedef struct IpmxUvcSegmenter {
    uint16_t max_packet_size;
    uint8_t  fid;            /* toggles per frame (0 or 1) */
    uint32_t quirks;         /* IPMX_UVC_QUIRK_* flags */

    /* Current frame state (set by begin_frame) */
    const uint8_t *frame_data;
    int            frame_size;
    int            frame_offset;
    int            frame_done;

    /* Per-frame PTS (microseconds, truncated to 32-bit) */
    uint32_t frame_pts;
    int      has_pts;

    /* Per-frame SCR (Source Clock Reference) */
    uint32_t frame_stc;
    uint16_t frame_sof;
    int      has_scr;
} IpmxUvcSegmenter;

void ipmx_uvc_seg_init(IpmxUvcSegmenter *seg, uint16_t max_packet_size,
                        uint32_t quirks);

/**
 * Begin segmentation of one video frame. Toggles FID.
 * @param pts_us   Presentation timestamp in microseconds, or < 0 if absent.
 *                 Truncated to 32-bit for the UVC PTS field.
 * @param has_scr  Non-zero to include SCR in payload headers.
 * @param stc      Source Time Clock (device clock ticks), used when has_scr != 0.
 * @param sof      11-bit USB SOF frame number, used when has_scr != 0.
 */
void ipmx_uvc_seg_begin_frame(IpmxUvcSegmenter *seg,
                               const uint8_t *frame_data, int frame_size,
                               int64_t pts_us,
                               int has_scr, uint32_t stc, uint16_t sof);

/**
 * Get next ISO packet payload.
 *
 * @param out_buf     Output buffer (must be >= max_packet_size)
 * @param out_buf_size Buffer size
 * @param out_len     Written length
 * @param is_last     Set to 1 if this is the last packet for the frame
 * @return 0 on success, 1 if no more packets (frame already fully emitted)
 */
int ipmx_uvc_seg_next_packet(IpmxUvcSegmenter *seg,
                              uint8_t *out_buf, int out_buf_size,
                              int *out_len, int *is_last);

#ifdef __cplusplus
}
#endif

#endif /* AVDEVICE_IPMX_USB_UVC_SEG_H */
