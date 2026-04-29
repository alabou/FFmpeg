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

#ifndef AVDEVICE_IPMX_USB_UVC_H
#define AVDEVICE_IPMX_USB_UVC_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* UVC descriptor types (VC = VideoControl, VS = VideoStreaming) */
#define UVC_DT_HEADER               0x01
#define UVC_DT_INPUT_TERMINAL       0x02
#define UVC_DT_OUTPUT_TERMINAL      0x03
#define UVC_DT_PROCESSING_UNIT      0x05
#define UVC_VS_INPUT_HEADER         0x01
#define UVC_VS_FORMAT_MJPEG         0x06
#define UVC_VS_FORMAT_UNCOMPRESSED  0x04
#define UVC_VS_FRAME_MJPEG          0x07
#define UVC_VS_FRAME_UNCOMPRESSED   0x05
#define UVC_VS_COLOR_MATCHING       0x0D
#define UVC_VS_FORMAT_FRAME_BASED   0x10
#define UVC_VS_FRAME_FRAME_BASED    0x11

#define UVC_CC_VIDEO                0x0E
#define UVC_SC_VIDEOCONTROL         0x01
#define UVC_SC_VIDEOSTREAMING       0x02

#define UVC_SET_CUR                 0x01
#define UVC_GET_CUR                 0x81
#define UVC_GET_MIN                 0x82
#define UVC_GET_MAX                 0x83
#define UVC_GET_DEF                 0x87
#define UVC_VS_PROBE_CONTROL        0x01
#define UVC_VS_COMMIT_CONTROL       0x02

/* H.264 Payload GUID: {34363248-0000-0010-8000-00AA00389B71} */
static const uint8_t UVC_GUID_FORMAT_H264[16] = {
    0x48, 0x32, 0x36, 0x34, 0x00, 0x00, 0x10, 0x00,
    0x80, 0x00, 0x00, 0xAA, 0x00, 0x38, 0x9B, 0x71
};

/* HEVC (H.265) Payload GUID: {35363248-0000-0010-8000-00AA00389B71} */
static const uint8_t UVC_GUID_FORMAT_HEVC[16] = {
    0x48, 0x32, 0x36, 0x35, 0x00, 0x00, 0x10, 0x00,
    0x80, 0x00, 0x00, 0xAA, 0x00, 0x38, 0x9B, 0x71
};

/* Common uncompressed GUIDs */
static const uint8_t UVC_GUID_FORMAT_YUY2[16] = {
    0x59, 0x55, 0x59, 0x32, 0x00, 0x00, 0x10, 0x00,
    0x80, 0x00, 0x00, 0xAA, 0x00, 0x38, 0x9B, 0x71
};
static const uint8_t UVC_GUID_FORMAT_NV12[16] = {
    0x4E, 0x56, 0x31, 0x32, 0x00, 0x00, 0x10, 0x00,
    0x80, 0x00, 0x00, 0xAA, 0x00, 0x38, 0x9B, 0x71
};

#define IPMX_UVC_MAX_FRAMES 16

typedef struct IpmxUvcFrameInfo {
    uint16_t width;
    uint16_t height;
    uint32_t default_interval;  /* 100ns units */
    uint32_t max_frame_size;
} IpmxUvcFrameInfo;

typedef struct IpmxUvcFormatInfo {
    uint8_t  format_index;
    uint8_t  format_type;      /* FORMAT_MJPEG, FORMAT_UNCOMPRESSED, or FORMAT_FRAME_BASED */
    uint8_t  num_frames;
    IpmxUvcFrameInfo frames[IPMX_UVC_MAX_FRAMES];
    uint8_t  guid[16];         /* uncompressed or frame-based: payload GUID */
    uint8_t  bits_per_pixel;
} IpmxUvcFormatInfo;

#define IPMX_UVC_MAX_FORMATS 8
#define IPMX_UVC_MAX_FRAME_BUF (2 * 1024 * 1024)

typedef struct IpmxUvcContext {
    /* Parsed from descriptors */
    uint8_t  vc_interface;     /* VideoControl interface number */
    uint8_t  vs_interface;      /* VideoStreaming interface number */
    uint8_t  vs_endpoint;       /* Isochronous IN endpoint number */
    uint16_t max_packet_size;
    uint32_t clock_frequency;   /* dwClockFrequency from VC Header (Hz, 0 = unknown) */
    uint8_t  num_formats;
    IpmxUvcFormatInfo formats[IPMX_UVC_MAX_FORMATS];

    /* Negotiated */
    uint8_t  active_format;
    uint8_t  active_frame;
    uint32_t active_interval;

    /* Frame reassembly (double-buffered for SOI-based completion) */
    uint8_t *frame_buf;
    int      frame_buf_size;
    int      frame_buf_pos;
    uint8_t *done_buf;
    int      done_buf_size;
    int      done_buf_pos;
    uint8_t  last_fid;
    int      frame_complete;

    /* Quirks */
    uint32_t quirks;

    /* PTS extracted from UVC payload headers (device clock ticks, 32-bit wrap).
     * Units are defined by dwClockFrequency; convert to µs with:
     *   pts_us = pts_ticks * 1000000 / clock_frequency */
    uint32_t cur_frame_pts;   /* PTS captured at start of current frame */
    int      cur_frame_has_pts;
    uint32_t done_frame_pts;  /* PTS of the most recently completed frame */
    int      done_frame_has_pts;

    /* SCR (Source Clock Reference) extracted from UVC payload headers.
     * STC is in the same clock domain as PTS (device clock ticks).
     * SOF is the 11-bit USB frame number sampled when STC was captured. */
    uint32_t cur_frame_stc;
    uint16_t cur_frame_sof;
    int      cur_frame_has_scr;
    uint32_t done_frame_stc;
    uint16_t done_frame_sof;
    int      done_frame_has_scr;
} IpmxUvcContext;

/**
 * Parse USB configuration descriptor chain and populate UVC context.
 * Returns 0 on success, negative on error.
 */
int ipmx_uvc_parse_config(IpmxUvcContext *ctx, const uint8_t *desc, int desc_len);

/**
 * Build 26-byte UVC Probe/Commit control data.
 * Returns 26 on success, negative on error.
 */
int ipmx_uvc_build_probe_control(uint8_t *buf, int buf_size,
                                uint8_t format_idx, uint8_t frame_idx,
                                uint32_t interval);

/**
 * Build 8-byte USB SET_CUR setup packet.
 * Returns 8 on success, negative on error.
 */
int ipmx_uvc_build_set_cur_req(uint8_t *buf, uint8_t cs, uint8_t interface,
                                uint16_t length);

/**
 * Build 8-byte USB GET_CUR setup packet.
 * Returns 8 on success, negative on error.
 */
int ipmx_uvc_build_get_cur_req(uint8_t *buf, uint8_t cs, uint8_t interface,
                               uint16_t length);
int ipmx_uvc_build_get_def_req(uint8_t *buf, uint8_t cs, uint8_t interface,
                               uint16_t length);
int ipmx_uvc_build_get_min_req(uint8_t *buf, uint8_t cs, uint8_t interface,
                                uint16_t length);
int ipmx_uvc_build_get_max_req(uint8_t *buf, uint8_t cs, uint8_t interface,
                               uint16_t length);

/**
 * Initialize frame reassembly buffer.
 * Returns 0 on success, negative on error.
 */
int ipmx_uvc_init_reassembler(IpmxUvcContext *ctx);

/**
 * Free frame reassembly buffer.
 */
void ipmx_uvc_free_reassembler(IpmxUvcContext *ctx);

/**
 * Process one isochronous packet payload.
 * Returns 0 on success, negative on error.
 */
int ipmx_uvc_process_payload(IpmxUvcContext *ctx, const uint8_t *data,
                             int data_len);

/**
 * Check if a complete frame is ready.
 * Returns 1 if ready, 0 otherwise.
 */
int ipmx_uvc_frame_ready(IpmxUvcContext *ctx);

/**
 * Get completed frame data and reset reassembler state.
 * Sets *data and *size. Returns 0 on success.
 */
int ipmx_uvc_get_frame(IpmxUvcContext *ctx, uint8_t **data, int *size);

/**
 * Retrieve PTS of the last completed frame.
 * @param pts_ticks  Output: PTS in device clock ticks (dwClockFrequency units).
 * @return 1 if PTS was present, 0 if absent.
 */
int ipmx_uvc_get_frame_pts(IpmxUvcContext *ctx, uint32_t *pts_ticks);

/**
 * Retrieve SCR (Source Clock Reference) of the last completed frame.
 * @param stc  Output: Source Time Clock (device clock ticks, same domain as PTS).
 * @param sof  Output: 11-bit USB SOF frame number sampled at STC capture time.
 * @return 1 if SCR was present, 0 if absent.
 */
int ipmx_uvc_get_frame_scr(IpmxUvcContext *ctx, uint32_t *stc, uint16_t *sof);

/**
 * Select best matching format/frame for width x height @ fps.
 * Prefers MJPEG > H.264 > uncompressed.
 * Sets active_format, active_frame, active_interval.
 * Returns 0 if found, -1 if no match.
 */
int ipmx_uvc_select_format(IpmxUvcContext *ctx, int width, int height, int fps);

/**
 * Check if a format's GUID identifies H.264.
 */
int ipmx_uvc_is_h264(const IpmxUvcFormatInfo *fmt);

/**
 * Check if a format's GUID identifies HEVC (H.265).
 */
int ipmx_uvc_is_hevc(const IpmxUvcFormatInfo *fmt);

/**
 * Return a human-readable format name for logging.
 */
const char *ipmx_uvc_format_name(const IpmxUvcFormatInfo *fmt);

#ifdef __cplusplus
}
#endif

#endif /* AVDEVICE_IPMX_USB_UVC_H */
