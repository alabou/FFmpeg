/*
 * Local USB device input using libusb — shares UVC/UAC with IPMX USB receiver.
 *
 * Bypasses the TR-10-14 TCP layer and communicates directly with a locally
 * attached USB webcam/microphone via libusb.  The UVC frame reassembly,
 * UAC audio extraction, and quirk handling are identical to the IPMX USB
 * receiver; only the transport changes.
 *
 * Usage:
 *   ffmpeg -f ipmx_usb_local -usb_device 0c45:636b -i "" -c:v copy out.mkv
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

#include "config_components.h"

#include <libusb-1.0/libusb.h>

#include "libavformat/avformat.h"
#include "libavformat/demux.h"
#include "libavformat/internal.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/time.h"
#include "libavutil/log.h"
#include "libavutil/pixdesc.h"
#include "avdevice.h"
#include "ipmx_usb_uvc.h"
#include "ipmx_usb_uac.h"
#include "ipmx_usb_quirks.h"
#include "ipmx_timing.h"

#include <string.h>

/* USB standard descriptor types */
#define USB_DT_DEVICE    0x01
#define USB_DT_CONFIG    0x02
#define USB_DT_STRING    0x03

#define CONFIG_DESC_MAX  4096
#define PCM_BUF_SIZE     (48000 * 8 * 4)

/* Isochronous transfer parameters */
#define ISO_VIDEO_NUM_PACKETS   32
#define ISO_AUDIO_NUM_PACKETS   8
#define ISO_MAX_PACKET_SIZE     3072
#define ISO_TRANSFERS_VIDEO     4
#define ISO_TRANSFERS_AUDIO     4

#define CTRL_TIMEOUT_MS  5000

/* ---------------------------------------------------------------------------
 * Async ISO transfer ring
 * --------------------------------------------------------------------------- */

typedef struct IsoRing {
    struct libusb_transfer **xfers;
    int      count;
    int      next_reap;
    volatile int completed;
    uint8_t  endpoint;
    int      num_packets;
    uint16_t max_pkt_size;
} IsoRing;

/* ---------------------------------------------------------------------------
 * Private context
 * --------------------------------------------------------------------------- */

typedef struct IpmxUsbLocalContext {
    AVClass *class;

    /* CLI options */
    char *usb_device;
    char *video_size_str;
    char *framerate_str;
    char *stream_type;
    int   sample_rate;

    /* libusb state */
    libusb_context       *usb_ctx;
    libusb_device_handle *dev_handle;
    uint16_t dev_vid;
    uint16_t dev_pid;

    /* Claimed interfaces (for cleanup) */
    int ifaces_claimed[8];
    int num_ifaces_claimed;

    /* Device descriptors */
    uint8_t config_desc[CONFIG_DESC_MAX];
    int     config_desc_len;

    /* UVC / UAC shared modules */
    IpmxUvcContext uvc;
    IpmxUacContext uac;
    int has_video;
    int has_audio;
    int video_stream_idx;
    int audio_stream_idx;

    /* ISO transfer rings */
    IsoRing video_ring;
    IsoRing audio_ring;

    /* PTS counters (diagnostics only) */
    int64_t video_frame_count;
    int64_t audio_sample_count;

    /* Wall-clock PTS epoch (microseconds, CLOCK_MONOTONIC) */
    int64_t epoch_us;

    /* UVC PTS unwrapping for source-aware pacing (32-bit wraps every ~71 min) */
    int      uvc_pts_epoch_set;
    uint32_t uvc_pts_epoch;     /* first UVC PTS seen */
    uint32_t uvc_pts_prev;      /* previous raw UVC PTS for wraparound detection */
    int64_t  uvc_pts_wrap_acc;  /* accumulated wraparound offset */

    /* Source SOF unwrapping for frame_number derivation (11-bit wraps every ~2048 ms) */
    int      has_source_sof;
    uint16_t first_sof;
    int64_t  sof_unwrapped;

    /* Round-robin: alternate between video and audio delivery */
    int next_is_audio;

    /* Shutdown */
    int stop;

    /* Timing instrumentation (IPMX_TIMING=1) */
    FILE *timing_fp;
    int64_t timing_vid_cnt;
    int64_t timing_aud_cnt;

    /* Scratch buffer for control IN transfers (avoids 4 KiB on the stack) */
    uint8_t ctrl_in_buf[4096];
} IpmxUsbLocalContext;

static int64_t capture_pts_us(IpmxUsbLocalContext *ctx)
{
    int64_t now = av_gettime_relative();
    if (!ctx->epoch_us)
        ctx->epoch_us = now;
    return now - ctx->epoch_us;
}

/*
 * Derive a synthetic USB frame number from wall clock.  Since this process
 * IS the USB host, av_gettime_relative() at 1 ms resolution is effectively
 * the SOF counter.  The value is monotonic and does not wrap for ~25 days
 * (31-bit range in TR-10-14 STARTFRAME).
 */
static uint32_t capture_frame_number(IpmxUsbLocalContext *ctx)
{
    int64_t elapsed_us = capture_pts_us(ctx);
    return (uint32_t)(elapsed_us / 1000);
}

/*
 * Unwrap a 32-bit UVC device PTS into a monotonic offset in device clock
 * ticks (dwClockFrequency units) relative to the first PTS ever seen.
 * Handles the 32-bit wraparound (~143 s at 30 MHz, ~71.6 min at 1 MHz).
 */
static int64_t unwrap_uvc_pts(IpmxUsbLocalContext *ctx, uint32_t raw_pts)
{
    if (!ctx->uvc_pts_epoch_set) {
        ctx->uvc_pts_epoch     = raw_pts;
        ctx->uvc_pts_prev      = raw_pts;
        ctx->uvc_pts_wrap_acc  = 0;
        ctx->uvc_pts_epoch_set = 1;
        return 0;
    }
    if (raw_pts < ctx->uvc_pts_prev &&
        (ctx->uvc_pts_prev - raw_pts) > (UINT32_MAX / 2))
        ctx->uvc_pts_wrap_acc += (int64_t)UINT32_MAX + 1;
    ctx->uvc_pts_prev = raw_pts;
    return ctx->uvc_pts_wrap_acc + (int64_t)raw_pts - (int64_t)ctx->uvc_pts_epoch;
}

/* ---------------------------------------------------------------------------
 * Options
 * --------------------------------------------------------------------------- */

static const AVOption ipmx_usb_local_options[] = {
    { "usb_device", "USB VID:PID (hex, e.g. 0c45:636b)",
      offsetof(IpmxUsbLocalContext, usb_device),
      AV_OPT_TYPE_STRING, {.str = NULL}, 0, 0, AV_OPT_FLAG_DECODING_PARAM },
    { "video_size", "Preferred resolution (e.g. 1280x720)",
      offsetof(IpmxUsbLocalContext, video_size_str),
      AV_OPT_TYPE_STRING, {.str = NULL}, 0, 0, AV_OPT_FLAG_DECODING_PARAM },
    { "framerate", "Preferred frame rate",
      offsetof(IpmxUsbLocalContext, framerate_str),
      AV_OPT_TYPE_STRING, {.str = NULL}, 0, 0, AV_OPT_FLAG_DECODING_PARAM },
    { "stream_type", "Stream type: video, audio, both",
      offsetof(IpmxUsbLocalContext, stream_type),
      AV_OPT_TYPE_STRING, {.str = "both"}, 0, 0, AV_OPT_FLAG_DECODING_PARAM },
    { "sample_rate", "Preferred audio sample rate",
      offsetof(IpmxUsbLocalContext, sample_rate),
      AV_OPT_TYPE_INT, {.i64 = 48000}, 8000, 384000, AV_OPT_FLAG_DECODING_PARAM },
    { NULL },
};

static const AVClass ipmx_usb_local_class = {
    .class_name = "ipmx_usb_local indev",
    .item_name  = av_default_item_name,
    .option     = ipmx_usb_local_options,
    .version    = LIBAVUTIL_VERSION_INT,
    .category   = AV_CLASS_CATEGORY_DEVICE_VIDEO_INPUT,
};

/* ---------------------------------------------------------------------------
 * Parse VID:PID string
 * --------------------------------------------------------------------------- */

static int parse_vid_pid(AVFormatContext *s, const char *str,
                         uint16_t *vid, uint16_t *pid)
{
    unsigned int v, p;
    if (!str || sscanf(str, "%x:%x", &v, &p) != 2) {
        av_log(s, AV_LOG_ERROR,
               "Invalid -usb_device '%s'; expected VID:PID (e.g. 0c45:636b)\n",
               str ? str : "(null)");
        return AVERROR(EINVAL);
    }
    *vid = (uint16_t)v;
    *pid = (uint16_t)p;
    return 0;
}

/* ---------------------------------------------------------------------------
 * Helper: claim an interface (detaching kernel driver if needed)
 * --------------------------------------------------------------------------- */

static int claim_interface(AVFormatContext *s, IpmxUsbLocalContext *ctx,
                           int iface)
{
    int i, ret;

    for (i = 0; i < ctx->num_ifaces_claimed; i++)
        if (ctx->ifaces_claimed[i] == iface)
            return 0;

    ret = libusb_claim_interface(ctx->dev_handle, iface);
    if (ret < 0) {
        av_log(s, AV_LOG_ERROR, "Cannot claim interface %d: %s\n",
               iface, libusb_strerror(ret));
        return AVERROR(EIO);
    }
    if (ctx->num_ifaces_claimed < 8)
        ctx->ifaces_claimed[ctx->num_ifaces_claimed++] = iface;
    av_log(s, AV_LOG_DEBUG, "Claimed interface %d\n", iface);
    return 0;
}

/* ---------------------------------------------------------------------------
 * libusb control transfer wrapper matching the 8-byte setup packet convention
 * --------------------------------------------------------------------------- */

static int local_ctrl_transfer(AVFormatContext *s, IpmxUsbLocalContext *ctx,
                               const uint8_t *setup,
                               const uint8_t *data_out, int data_out_len,
                               uint8_t *data_in, int *data_in_len)
{
    uint8_t  bmRequestType = setup[0];
    uint8_t  bRequest      = setup[1];
    uint16_t wValue        = setup[2] | ((uint16_t)setup[3] << 8);
    uint16_t wIndex        = setup[4] | ((uint16_t)setup[5] << 8);
    uint16_t wLength       = setup[6] | ((uint16_t)setup[7] << 8);
    int ret;

    if (bmRequestType & 0x80) {
        /* IN transfer */
        int len = data_in_len ? *data_in_len : wLength;
        uint8_t *buf = data_in ? data_in : ctx->ctrl_in_buf;
        int buf_size = data_in ? len : (int)sizeof(ctx->ctrl_in_buf);

        if (buf_size > (int)sizeof(ctx->ctrl_in_buf) && !data_in)
            buf_size = (int)sizeof(ctx->ctrl_in_buf);

        ret = libusb_control_transfer(ctx->dev_handle,
                                      bmRequestType, bRequest,
                                      wValue, wIndex,
                                      buf, (uint16_t)buf_size,
                                      CTRL_TIMEOUT_MS);
        if (ret < 0) {
            if (ctx->uac.quirks & IPMX_UAC_QUIRK_IGNORE_CTL_ERROR) {
                av_log(s, AV_LOG_WARNING, "Control IN transfer failed (ignored, quirk): %s\n",
                       libusb_strerror(ret));
                if (data_in_len)
                    *data_in_len = 0;
                return 0;
            }
            av_log(s, AV_LOG_DEBUG, "Control IN transfer failed: %s\n",
                   libusb_strerror(ret));
            return AVERROR(EIO);
        }
        if (data_in_len)
            *data_in_len = ret;
    } else {
        /* OUT transfer */
        ret = libusb_control_transfer(ctx->dev_handle,
                                      bmRequestType, bRequest,
                                      wValue, wIndex,
                                      (uint8_t *)(data_out ? data_out : (const uint8_t *)""),
                                      data_out ? (uint16_t)data_out_len : 0,
                                      CTRL_TIMEOUT_MS);
        if (ret < 0) {
            if (ctx->uac.quirks & IPMX_UAC_QUIRK_IGNORE_CTL_ERROR) {
                av_log(s, AV_LOG_WARNING, "Control OUT transfer failed (ignored, quirk): %s\n",
                       libusb_strerror(ret));
                return 0;
            }
            av_log(s, AV_LOG_DEBUG, "Control OUT transfer failed: %s\n",
                   libusb_strerror(ret));
            return AVERROR(EIO);
        }
    }
    return 0;
}

/* ---------------------------------------------------------------------------
 * USB enumeration (reads descriptors via libusb, parses with shared modules)
 * --------------------------------------------------------------------------- */

static int local_enumerate(AVFormatContext *s, IpmxUsbLocalContext *ctx)
{
    struct libusb_device *dev = libusb_get_device(ctx->dev_handle);
    struct libusb_device_descriptor dev_desc;
    struct libusb_config_descriptor *cfg = NULL;
    int ret;

    ret = libusb_get_device_descriptor(dev, &dev_desc);
    if (ret < 0) {
        av_log(s, AV_LOG_ERROR, "Cannot read device descriptor: %s\n",
               libusb_strerror(ret));
        return AVERROR(EIO);
    }
    ctx->dev_vid = dev_desc.idVendor;
    ctx->dev_pid = dev_desc.idProduct;
    av_log(s, AV_LOG_INFO, "USB device VID:PID = %04x:%04x\n",
           ctx->dev_vid, ctx->dev_pid);

    /* Read string descriptors */
    {
        unsigned char str_buf[256];
        if (dev_desc.iManufacturer &&
            libusb_get_string_descriptor_ascii(ctx->dev_handle,
                                               dev_desc.iManufacturer,
                                               str_buf, sizeof(str_buf)) > 0)
            av_log(s, AV_LOG_INFO, "Manufacturer: %s\n", str_buf);
        if (dev_desc.iProduct &&
            libusb_get_string_descriptor_ascii(ctx->dev_handle,
                                               dev_desc.iProduct,
                                               str_buf, sizeof(str_buf)) > 0)
            av_log(s, AV_LOG_INFO, "Product: %s\n", str_buf);
        if (dev_desc.iSerialNumber &&
            libusb_get_string_descriptor_ascii(ctx->dev_handle,
                                               dev_desc.iSerialNumber,
                                               str_buf, sizeof(str_buf)) > 0)
            av_log(s, AV_LOG_INFO, "Serial: %s\n", str_buf);
    }

    ret = libusb_get_active_config_descriptor(dev, &cfg);
    if (ret < 0) {
        av_log(s, AV_LOG_ERROR, "Cannot read config descriptor: %s\n",
               libusb_strerror(ret));
        return AVERROR(EIO);
    }

    /* Copy raw descriptor bytes for UVC/UAC parsers */
    {
        int raw_len;
        struct libusb_config_descriptor *raw_cfg = NULL;
        ret = libusb_get_config_descriptor(dev, 0, &raw_cfg);
        if (ret < 0) {
            libusb_free_config_descriptor(cfg);
            return AVERROR(EIO);
        }
        raw_len = raw_cfg->wTotalLength;
        if (raw_len > CONFIG_DESC_MAX)
            raw_len = CONFIG_DESC_MAX;

        /* libusb doesn't expose the raw bytes directly; re-read via control */
        {
            uint8_t setup[8];
            int xfer_len = raw_len;
            setup[0] = 0x80; /* IN | STANDARD | DEVICE */
            setup[1] = 0x06; /* GET_DESCRIPTOR */
            setup[2] = 0;    /* desc_index */
            setup[3] = USB_DT_CONFIG;
            setup[4] = 0;
            setup[5] = 0;
            setup[6] = raw_len & 0xFF;
            setup[7] = (raw_len >> 8) & 0xFF;
            ret = local_ctrl_transfer(s, ctx, setup, NULL, 0,
                                      ctx->config_desc, &xfer_len);
            if (ret < 0) {
                libusb_free_config_descriptor(raw_cfg);
                libusb_free_config_descriptor(cfg);
                return ret;
            }
            ctx->config_desc_len = xfer_len;
        }
        libusb_free_config_descriptor(raw_cfg);
    }

    libusb_free_config_descriptor(cfg);

    av_log(s, AV_LOG_INFO, "Config descriptor: %d bytes\n",
           ctx->config_desc_len);

    /* Parse with shared UVC/UAC modules */
    ret = ipmx_uvc_parse_config(&ctx->uvc, ctx->config_desc,
                                ctx->config_desc_len);
    if (ret < 0)
        av_log(s, AV_LOG_INFO, "No UVC interface found\n");
    else
        av_log(s, AV_LOG_INFO,
               "UVC: vc_iface=%d vs_iface=%d ep=0x%02x formats=%d\n",
               ctx->uvc.vc_interface, ctx->uvc.vs_interface,
               ctx->uvc.vs_endpoint, ctx->uvc.num_formats);

    ret = ipmx_uac_parse_config(&ctx->uac, ctx->config_desc,
                                ctx->config_desc_len);
    if (ret < 0)
        av_log(s, AV_LOG_INFO, "No UAC interface found\n");
    else
        av_log(s, AV_LOG_INFO,
               "UAC: ac_iface=%d as_iface=%d ep=0x%02x ch=%d rate=%u\n",
               ctx->uac.ac_interface, ctx->uac.as_interface,
               ctx->uac.as_endpoint, ctx->uac.channels,
               ctx->uac.sample_rates[0]);

    ctx->uvc.quirks = ipmx_uvc_quirks_lookup(ctx->dev_vid, ctx->dev_pid);
    ctx->uac.quirks = ipmx_uac_quirks_lookup(ctx->dev_vid, ctx->dev_pid);

    return 0;
}

/* ---------------------------------------------------------------------------
 * Find best ISO alt setting for an interface.
 *
 * Scans the raw config descriptor to enumerate all alternate settings of
 * the given interface, looks for isochronous IN endpoints, and returns the
 * alt setting with the highest actual bandwidth (decoding the high-speed
 * wMaxPacketSize multiplier in bits 12:11).
 *
 * On success stores the alt setting number in *best_alt, the decoded
 * per-transaction byte count in *best_pkt_size, and returns 0.
 * --------------------------------------------------------------------------- */

static int find_best_iso_alt(AVFormatContext *s,
                             const uint8_t *desc, int desc_len,
                             int target_iface,
                             int *best_alt, int *best_pkt_size,
                             int use_second_best)
{
    int pos = 0;
    int cur_iface = -1, cur_alt = -1;
    int second_alt = -1, second_pkt = 0;

    *best_alt = -1;
    *best_pkt_size = 0;

    while (pos + 2 <= desc_len) {
        uint8_t bLength = desc[pos];
        uint8_t bDescriptorType = desc[pos + 1];

        if (bLength < 2 || pos + bLength > desc_len)
            break;

        if (bDescriptorType == 0x04 && bLength >= 9) { /* INTERFACE */
            cur_iface = desc[pos + 2];
            cur_alt   = desc[pos + 3];
        } else if (bDescriptorType == 0x05 && bLength >= 7 &&
                   cur_iface == target_iface) { /* ENDPOINT */
            uint8_t  bEndpointAddress = desc[pos + 2];
            uint8_t  bmAttributes     = desc[pos + 3];
            uint16_t wMaxPacketSize   = desc[pos + 4] |
                                        ((uint16_t)desc[pos + 5] << 8);

            if ((bmAttributes & 0x03) == 0x01 && (bEndpointAddress & 0x80)) {
                int base_size  = wMaxPacketSize & 0x07FF;
                int mult       = ((wMaxPacketSize >> 11) & 0x03) + 1;
                int total_size = base_size * mult;

                if (total_size > *best_pkt_size) {
                    if (use_second_best) {
                        second_alt = *best_alt;
                        second_pkt = *best_pkt_size;
                    }
                    *best_pkt_size = total_size;
                    *best_alt      = cur_alt;
                } else if (use_second_best && total_size > second_pkt) {
                    second_alt = cur_alt;
                    second_pkt = total_size;
                }
                av_log(s, AV_LOG_DEBUG,
                       "  iface %d alt %d ep 0x%02x: %d x %d = %d bytes\n",
                       target_iface, cur_alt, bEndpointAddress,
                       base_size, mult, total_size);
            }
        }
        pos += bLength;
    }

    if (*best_alt < 0) {
        av_log(s, AV_LOG_ERROR,
               "No ISO IN endpoint found for interface %d\n", target_iface);
        return AVERROR(ENODEV);
    }
    if (use_second_best && second_alt >= 0) {
        *best_alt = second_alt;
        *best_pkt_size = second_pkt;
        av_log(s, AV_LOG_DEBUG,
               "FIX_BANDWIDTH quirk: using second-best alt %d, pkt %d\n",
               *best_alt, *best_pkt_size);
    }
    av_log(s, AV_LOG_INFO,
           "Interface %d: best alt %d, max ISO pkt %d bytes\n",
           target_iface, *best_alt, *best_pkt_size);
    return 0;
}

/* ---------------------------------------------------------------------------
 * UVC probe/commit via libusb
 * --------------------------------------------------------------------------- */

static int local_uvc_negotiate(AVFormatContext *s, IpmxUsbLocalContext *ctx)
{
    uint8_t probe_data[26];
    uint8_t setup[8];
    int probe_len, ret, best_alt, best_pkt;

    ret = claim_interface(s, ctx, ctx->uvc.vc_interface);
    if (ret < 0) return ret;
    ret = claim_interface(s, ctx, ctx->uvc.vs_interface);
    if (ret < 0) return ret;

    if (ctx->uvc.quirks & IPMX_UVC_QUIRK_PROBE_DEF) {
        ipmx_uvc_build_get_def_req(setup, UVC_VS_PROBE_CONTROL,
                                    ctx->uvc.vs_interface, 26);
        probe_len = 26;
        ret = local_ctrl_transfer(s, ctx, setup, NULL, 0,
                                  probe_data, &probe_len);
        if (ret < 0 || probe_len != 26) {
            av_log(s, AV_LOG_DEBUG, "GET_DEF(PROBE) failed, using built probe\n");
            ret = ipmx_uvc_build_probe_control(probe_data, sizeof(probe_data),
                                               ctx->uvc.active_format,
                                               ctx->uvc.active_frame,
                                               ctx->uvc.active_interval);
            if (ret < 0) return AVERROR_EXIT;
        }
    } else if (ctx->uvc.quirks & IPMX_UVC_QUIRK_PROBE_MINMAX) {
        uint8_t min_data[26], max_data[26];
        uint8_t fmt_clamp, frame_clamp;
        uint32_t interval_min, interval_max, interval_clamp;

        ipmx_uvc_build_get_min_req(setup, UVC_VS_PROBE_CONTROL,
                                    ctx->uvc.vs_interface, 26);
        probe_len = 26;
        ret = local_ctrl_transfer(s, ctx, setup, NULL, 0, min_data, &probe_len);
        if (ret < 0 || probe_len != 26) {
            av_log(s, AV_LOG_DEBUG, "GET_MIN(PROBE) failed, using built probe\n");
            ret = ipmx_uvc_build_probe_control(probe_data, sizeof(probe_data),
                                               ctx->uvc.active_format,
                                               ctx->uvc.active_frame,
                                               ctx->uvc.active_interval);
            if (ret < 0) return AVERROR_EXIT;
        } else {
            ipmx_uvc_build_get_max_req(setup, UVC_VS_PROBE_CONTROL,
                                       ctx->uvc.vs_interface, 26);
            probe_len = 26;
            ret = local_ctrl_transfer(s, ctx, setup, NULL, 0, max_data, &probe_len);
            if (ret < 0 || probe_len != 26) {
                av_log(s, AV_LOG_DEBUG, "GET_MAX(PROBE) failed, using built probe\n");
                ret = ipmx_uvc_build_probe_control(probe_data, sizeof(probe_data),
                                                   ctx->uvc.active_format,
                                                   ctx->uvc.active_frame,
                                                   ctx->uvc.active_interval);
                if (ret < 0) return AVERROR_EXIT;
            } else {
                interval_min = min_data[4] | ((uint32_t)min_data[5] << 8) |
                              ((uint32_t)min_data[6] << 16) | ((uint32_t)min_data[7] << 24);
                interval_max = max_data[4] | ((uint32_t)max_data[5] << 8) |
                              ((uint32_t)max_data[6] << 16) | ((uint32_t)max_data[7] << 24);
                fmt_clamp = (uint8_t)av_clip(ctx->uvc.active_format, min_data[2], max_data[2]);
                frame_clamp = (uint8_t)av_clip(ctx->uvc.active_frame, min_data[3], max_data[3]);
                interval_clamp = (uint32_t)av_clip((int64_t)ctx->uvc.active_interval,
                                                   (int64_t)interval_min,
                                                   (int64_t)interval_max);
                ret = ipmx_uvc_build_probe_control(probe_data, sizeof(probe_data),
                                                   fmt_clamp, frame_clamp, interval_clamp);
                if (ret < 0) return AVERROR_EXIT;
            }
        }
    } else {
        ret = ipmx_uvc_build_probe_control(probe_data, sizeof(probe_data),
                                           ctx->uvc.active_format,
                                           ctx->uvc.active_frame,
                                           ctx->uvc.active_interval);
        if (ret < 0) return AVERROR_EXIT;
    }

    /* SET_CUR(PROBE) */
    ipmx_uvc_build_set_cur_req(setup, UVC_VS_PROBE_CONTROL,
                               ctx->uvc.vs_interface, 26);
    ret = local_ctrl_transfer(s, ctx, setup, probe_data, 26, NULL, NULL);
    if (ret < 0) {
        av_log(s, AV_LOG_ERROR, "SET_CUR(PROBE) failed\n");
        return ret;
    }

    /* GET_CUR(PROBE) */
    ipmx_uvc_build_get_cur_req(setup, UVC_VS_PROBE_CONTROL,
                               ctx->uvc.vs_interface, 26);
    probe_len = 26;
    ret = local_ctrl_transfer(s, ctx, setup, NULL, 0,
                              probe_data, &probe_len);
    if (ret < 0) {
        av_log(s, AV_LOG_ERROR, "GET_CUR(PROBE) failed\n");
        return ret;
    }

    /* SET_CUR(COMMIT) */
    ipmx_uvc_build_set_cur_req(setup, UVC_VS_COMMIT_CONTROL,
                               ctx->uvc.vs_interface, 26);
    ret = local_ctrl_transfer(s, ctx, setup, probe_data, 26, NULL, NULL);
    if (ret < 0) {
        av_log(s, AV_LOG_ERROR, "SET_CUR(COMMIT) failed\n");
        return ret;
    }

    /* Select the highest-bandwidth alt setting for the VS interface */
    ret = find_best_iso_alt(s, ctx->config_desc, ctx->config_desc_len,
                            ctx->uvc.vs_interface, &best_alt, &best_pkt,
                            (ctx->uvc.quirks & IPMX_UVC_QUIRK_FIX_BANDWIDTH) ? 1 : 0);
    if (ret < 0)
        return ret;

    ret = libusb_set_interface_alt_setting(ctx->dev_handle,
                                           ctx->uvc.vs_interface, best_alt);
    if (ret < 0) {
        av_log(s, AV_LOG_ERROR, "SET_INTERFACE(%d, %d) failed: %s\n",
               ctx->uvc.vs_interface, best_alt, libusb_strerror(ret));
        return AVERROR(EIO);
    }
    ctx->uvc.max_packet_size = best_pkt;

    av_log(s, AV_LOG_INFO,
           "UVC probe/commit negotiation complete (alt=%d pkt=%d)\n",
           best_alt, best_pkt);
    return 0;
}

/* ---------------------------------------------------------------------------
 * UAC activation via libusb
 * --------------------------------------------------------------------------- */

static int local_uac_activate(AVFormatContext *s, IpmxUsbLocalContext *ctx)
{
    int ret, best_alt, best_pkt;

    ret = claim_interface(s, ctx, ctx->uac.ac_interface);
    if (ret < 0) return ret;
    ret = claim_interface(s, ctx, ctx->uac.as_interface);
    if (ret < 0) return ret;

    ret = find_best_iso_alt(s, ctx->config_desc, ctx->config_desc_len,
                            ctx->uac.as_interface, &best_alt, &best_pkt, 0);
    if (ret < 0)
        return ret;

    ret = libusb_set_interface_alt_setting(ctx->dev_handle,
                                           ctx->uac.as_interface, best_alt);
    if (ret < 0) {
        av_log(s, AV_LOG_ERROR, "SET_INTERFACE(%d, %d) for audio failed: %s\n",
               ctx->uac.as_interface, best_alt, libusb_strerror(ret));
        return AVERROR(EIO);
    }
    ctx->uac.max_packet_size = best_pkt;

    if (ctx->uac.uac_version == 2 && ctx->uac.clock_source_id != 0 &&
        !(ctx->uac.quirks & IPMX_UAC_QUIRK_SKIP_CLOCK_SELECTOR)) {
        /* UAC 2.0: negotiate sample rate via Clock Source entity */
        uint8_t range_buf[2 + 12];
        uint16_t windex_cs = (uint16_t)ctx->uac.clock_source_id |
                             ((uint16_t)ctx->uac.ac_interface << 8);

        ret = libusb_control_transfer(ctx->dev_handle,
                                      0xA1, UAC2_RANGE,
                                      (UAC2_CS_SAM_FREQ_CONTROL << 8),
                                      windex_cs,
                                      range_buf, sizeof(range_buf),
                                      CTRL_TIMEOUT_MS);
        if (ret >= 14) {
            ctx->uac.clock_freq_min = range_buf[2] | (range_buf[3] << 8) |
                                      (range_buf[4] << 16) | (range_buf[5] << 24);
            ctx->uac.clock_freq_max = range_buf[6] | (range_buf[7] << 8) |
                                      (range_buf[8] << 16) | (range_buf[9] << 24);
            av_log(s, AV_LOG_INFO,
                   "UAC 2.0 Clock Source range: %u-%u Hz\n",
                   ctx->uac.clock_freq_min, ctx->uac.clock_freq_max);
        } else {
            av_log(s, AV_LOG_WARNING,
                   "UAC 2.0 GET_RANGE on Clock Source failed: %s\n",
                   libusb_strerror(ret));
        }

        if (ctx->uac.active_sample_rate > 0) {
            uint8_t rate_data[4];
            uint32_t rate = ctx->uac.active_sample_rate;
            rate_data[0] = (uint8_t)(rate);
            rate_data[1] = (uint8_t)(rate >> 8);
            rate_data[2] = (uint8_t)(rate >> 16);
            rate_data[3] = (uint8_t)(rate >> 24);

            ret = libusb_control_transfer(ctx->dev_handle,
                                          0x21, UAC2_CUR,
                                          (UAC2_CS_SAM_FREQ_CONTROL << 8),
                                          windex_cs,
                                          rate_data, 4, CTRL_TIMEOUT_MS);
            if (ret < 0)
                av_log(s, AV_LOG_WARNING,
                       "UAC 2.0 SET_CUR on Clock Source failed: %s\n",
                       libusb_strerror(ret));
            else
                av_log(s, AV_LOG_INFO,
                       "UAC 2.0 sample rate set to %u Hz\n",
                       ctx->uac.active_sample_rate);
        }
    } else if (ctx->uac.num_sample_rates > 1 && ctx->uac.active_sample_rate > 0) {
        /* UAC 1.0: SET_CUR Sampling Frequency on endpoint */
        uint8_t rate_data[3];
        uint32_t rate = ctx->uac.active_sample_rate;
        rate_data[0] = (uint8_t)(rate);
        rate_data[1] = (uint8_t)(rate >> 8);
        rate_data[2] = (uint8_t)(rate >> 16);

        ret = libusb_control_transfer(ctx->dev_handle,
                                      0x22, 0x01,
                                      0x0100, ctx->uac.as_endpoint | 0x80,
                                      rate_data, 3, CTRL_TIMEOUT_MS);
        if (ret < 0)
            av_log(s, AV_LOG_WARNING,
                   "SET_CUR Sampling Frequency failed (non-fatal): %s\n",
                   libusb_strerror(ret));
        else
            av_log(s, AV_LOG_INFO, "UAC sample rate set to %u Hz\n", rate);
    }

    av_log(s, AV_LOG_INFO, "UAC streaming interface activated\n");
    return 0;
}

/* ---------------------------------------------------------------------------
 * ISO transfer completion callback
 * --------------------------------------------------------------------------- */

static void LIBUSB_CALL iso_callback(struct libusb_transfer *xfer)
{
    IsoRing *ring = (IsoRing *)xfer->user_data;
    ring->completed++;
}

/* ---------------------------------------------------------------------------
 * ISO ring setup / teardown
 * --------------------------------------------------------------------------- */

static int iso_ring_init(AVFormatContext *s, IpmxUsbLocalContext *ctx,
                         IsoRing *ring, uint8_t endpoint,
                         int num_packets, uint16_t max_pkt_size,
                         int num_transfers)
{
    int i, ret;

    ring->endpoint     = endpoint;
    ring->num_packets  = num_packets;
    ring->max_pkt_size = max_pkt_size;
    ring->count        = num_transfers;
    ring->next_reap    = 0;
    ring->completed    = 0;

    ring->xfers = av_calloc(num_transfers,
                            sizeof(struct libusb_transfer *));
    if (!ring->xfers)
        return AVERROR(ENOMEM);

    for (i = 0; i < num_transfers; i++) {
        int pkt_buf_size = num_packets * max_pkt_size;
        struct libusb_transfer *xfer =
            libusb_alloc_transfer(num_packets);
        if (!xfer)
            return AVERROR(ENOMEM);

        xfer->dev_handle = ctx->dev_handle;
        xfer->endpoint   = endpoint;
        xfer->type       = LIBUSB_TRANSFER_TYPE_ISOCHRONOUS;
        xfer->timeout    = 0;
        xfer->buffer     = av_mallocz(pkt_buf_size);
        if (!xfer->buffer) {
            libusb_free_transfer(xfer);
            return AVERROR(ENOMEM);
        }
        xfer->length       = pkt_buf_size;
        xfer->num_iso_packets = num_packets;
        xfer->callback     = iso_callback;
        xfer->user_data    = ring;

        libusb_set_iso_packet_lengths(xfer, max_pkt_size);

        ret = libusb_submit_transfer(xfer);
        if (ret < 0) {
            av_log(s, AV_LOG_ERROR,
                   "Failed to submit ISO transfer for ep 0x%02x: %s\n",
                   endpoint, libusb_strerror(ret));
            av_free(xfer->buffer);
            libusb_free_transfer(xfer);
            return AVERROR(EIO);
        }
        ring->xfers[i] = xfer;
    }

    av_log(s, AV_LOG_INFO,
           "ISO ring: ep=0x%02x transfers=%d pkts=%d max_pkt=%d\n",
           endpoint, num_transfers, num_packets, max_pkt_size);
    return 0;
}

static void iso_ring_cancel(IsoRing *ring)
{
    int i;
    if (!ring->xfers)
        return;
    for (i = 0; i < ring->count; i++) {
        if (ring->xfers[i])
            libusb_cancel_transfer(ring->xfers[i]);
    }
}

static void iso_ring_free(IsoRing *ring)
{
    int i;
    if (!ring->xfers)
        return;
    for (i = 0; i < ring->count; i++) {
        struct libusb_transfer *xfer = ring->xfers[i];
        if (xfer) {
            av_free(xfer->buffer);
            libusb_free_transfer(xfer);
        }
    }
    av_freep(&ring->xfers);
    ring->count = 0;
}

/* ---------------------------------------------------------------------------
 * Preference parsing helpers (shared logic with TCP receiver)
 * --------------------------------------------------------------------------- */

static void parse_video_size(const char *str, int *width, int *height)
{
    *width  = 1920;
    *height = 1080;
    if (str && str[0] && sscanf(str, "%dx%d", width, height) != 2) {
        *width  = 1920;
        *height = 1080;
    }
}

static int parse_framerate(const char *str)
{
    int fps = 30;
    if (str && str[0])
        fps = atoi(str);
    return (fps > 0) ? fps : 30;
}

/* ---------------------------------------------------------------------------
 * read_header
 * --------------------------------------------------------------------------- */

static int ipmx_usb_local_read_header(AVFormatContext *s)
{
    IpmxUsbLocalContext *ctx = s->priv_data;
    int ret, pref_w, pref_h, pref_fps;
    int want_video = 1, want_audio = 1;

    ctx->video_stream_idx = -1;
    ctx->audio_stream_idx = -1;

    /* Parse VID:PID */
    if (!ctx->usb_device) {
        av_log(s, AV_LOG_ERROR,
               "No USB device specified; use -usb_device VID:PID\n");
        return AVERROR(EINVAL);
    }
    ret = parse_vid_pid(s, ctx->usb_device, &ctx->dev_vid, &ctx->dev_pid);
    if (ret < 0)
        return ret;

    /* Initialize libusb */
    ret = libusb_init(&ctx->usb_ctx);
    if (ret < 0) {
        av_log(s, AV_LOG_ERROR, "libusb_init failed: %s\n",
               libusb_strerror(ret));
        return AVERROR(EIO);
    }

    ctx->dev_handle = libusb_open_device_with_vid_pid(
        ctx->usb_ctx, ctx->dev_vid, ctx->dev_pid);
    if (!ctx->dev_handle) {
        av_log(s, AV_LOG_ERROR,
               "Cannot open USB device %04x:%04x "
               "(not found or permission denied)\n",
               ctx->dev_vid, ctx->dev_pid);
        libusb_exit(ctx->usb_ctx);
        ctx->usb_ctx = NULL;
        return AVERROR(ENODEV);
    }

    libusb_set_auto_detach_kernel_driver(ctx->dev_handle, 1);

    ret = libusb_reset_device(ctx->dev_handle);
    if (ret < 0) {
        av_log(s, AV_LOG_WARNING, "USB device reset failed: %s (continuing)\n",
               libusb_strerror(ret));
    }
    av_log(s, AV_LOG_INFO, "Opened USB device %04x:%04x\n",
           ctx->dev_vid, ctx->dev_pid);

    /* Enumerate: read descriptors, parse UVC/UAC */
    ret = local_enumerate(s, ctx);
    if (ret < 0)
        goto fail;

    /* Resolve stream type preference */
    if (ctx->stream_type) {
        if (!strcmp(ctx->stream_type, "video"))
            want_audio = 0;
        else if (!strcmp(ctx->stream_type, "audio"))
            want_video = 0;
    }
    ctx->has_video = want_video && (ctx->uvc.num_formats > 0);
    ctx->has_audio = want_audio && (ctx->uac.channels > 0);
    if (!ctx->has_video && !ctx->has_audio) {
        ctx->has_video = (ctx->uvc.num_formats > 0);
        ctx->has_audio = (ctx->uac.channels > 0);
    }
    if (!ctx->has_video && !ctx->has_audio) {
        av_log(s, AV_LOG_ERROR, "No video or audio capabilities found\n");
        ret = AVERROR(ENODEV);
        goto fail;
    }

    /* UVC negotiation */
    if (ctx->has_video) {
        parse_video_size(ctx->video_size_str, &pref_w, &pref_h);
        pref_fps = parse_framerate(ctx->framerate_str);

        ret = ipmx_uvc_select_format(&ctx->uvc, pref_w, pref_h, pref_fps);
        if (ret < 0) {
            av_log(s, AV_LOG_WARNING,
                   "Could not match %dx%d@%d, using first format\n",
                   pref_w, pref_h, pref_fps);
            ctx->uvc.active_format  = ctx->uvc.formats[0].format_index;
            ctx->uvc.active_frame   = 1;
            ctx->uvc.active_interval =
                ctx->uvc.formats[0].frames[0].default_interval;
        }
        if (ctx->uvc.quirks & IPMX_UVC_QUIRK_RESTRICT_FRAME_RATE) {
            for (int f = 0; f < ctx->uvc.num_formats; f++) {
                if (ctx->uvc.formats[f].format_index == ctx->uvc.active_format &&
                    ctx->uvc.formats[f].num_frames > 0) {
                    ctx->uvc.active_interval =
                        ctx->uvc.formats[f].frames[0].default_interval;
                    av_log(s, AV_LOG_DEBUG,
                           "RESTRICT_FRAME_RATE quirk: interval=%u\n",
                           (unsigned)ctx->uvc.active_interval);
                    break;
                }
            }
        }

        ret = local_uvc_negotiate(s, ctx);
        if (ret < 0) {
            av_log(s, AV_LOG_ERROR, "UVC negotiation failed\n");
            goto fail;
        }

        ret = ipmx_uvc_init_reassembler(&ctx->uvc);
        if (ret < 0) {
            av_log(s, AV_LOG_ERROR, "Failed to init UVC reassembler\n");
            goto fail;
        }
    }

    /* UAC activation */
    if (ctx->has_audio) {
        if (ipmx_uac_select_rate(&ctx->uac, ctx->sample_rate) < 0) {
            if (ctx->uac.sample_rates[0] > 0)
                ctx->uac.active_sample_rate = ctx->uac.sample_rates[0];
            else
                ctx->uac.active_sample_rate = 8000;
        }
        if (ctx->uac.quirks & IPMX_UAC_QUIRK_RATE_FIX && ctx->uac.active_sample_rate == 0)
            ctx->uac.active_sample_rate = 48000;

        ret = local_uac_activate(s, ctx);
        if (ret < 0) {
            av_log(s, AV_LOG_ERROR, "UAC activation failed\n");
            goto fail;
        }
    }

    /* Create video AVStream */
    if (ctx->has_video) {
        AVStream *vst = avformat_new_stream(s, NULL);
        int sel_fmt = -1, sel_frm = -1, f, r;
        const IpmxUvcFormatInfo *active_fmt_info = NULL;

        if (!vst) { ret = AVERROR(ENOMEM); goto fail; }
        ctx->video_stream_idx = vst->index;
        vst->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;

        for (f = 0; f < ctx->uvc.num_formats; f++) {
            if (ctx->uvc.formats[f].format_index == ctx->uvc.active_format) {
                sel_fmt = f;
                break;
            }
        }
        if (sel_fmt >= 0) {
            active_fmt_info = &ctx->uvc.formats[sel_fmt];
            for (r = 0; r < ctx->uvc.formats[sel_fmt].num_frames; r++) {
                if (r + 1 == ctx->uvc.active_frame) {
                    sel_frm = r;
                    break;
                }
            }
        }

        if (active_fmt_info && ipmx_uvc_is_h264(active_fmt_info))
            vst->codecpar->codec_id = AV_CODEC_ID_H264;
        else if (active_fmt_info && ipmx_uvc_is_hevc(active_fmt_info))
            vst->codecpar->codec_id = AV_CODEC_ID_HEVC;
        else if (active_fmt_info &&
                 active_fmt_info->format_type == UVC_VS_FORMAT_UNCOMPRESSED) {
            vst->codecpar->codec_id = AV_CODEC_ID_RAWVIDEO;
            if (memcmp(active_fmt_info->guid, UVC_GUID_FORMAT_YUY2, 16) == 0)
                vst->codecpar->format = AV_PIX_FMT_YUYV422;
            else if (memcmp(active_fmt_info->guid, UVC_GUID_FORMAT_NV12, 16) == 0)
                vst->codecpar->format = AV_PIX_FMT_NV12;
            else
                vst->codecpar->format = AV_PIX_FMT_YUYV422;
        } else {
            vst->codecpar->codec_id = AV_CODEC_ID_MJPEG;
        }

        if (sel_fmt >= 0 && sel_frm >= 0) {
            vst->codecpar->width  =
                ctx->uvc.formats[sel_fmt].frames[sel_frm].width;
            vst->codecpar->height =
                ctx->uvc.formats[sel_fmt].frames[sel_frm].height;
        } else {
            vst->codecpar->width  = 1920;
            vst->codecpar->height = 1080;
        }

        avpriv_set_pts_info(vst, 64, 1, 1000000);

        if (ctx->uvc.clock_frequency > 0) {
            char buf[32];
            snprintf(buf, sizeof(buf), "%u", ctx->uvc.clock_frequency);
            av_dict_set(&vst->metadata, "uvc_clock_frequency", buf, 0);
        }

        av_log(s, AV_LOG_INFO, "Video stream[%d]: %dx%d %s\n",
               vst->index, vst->codecpar->width, vst->codecpar->height,
               active_fmt_info ? ipmx_uvc_format_name(active_fmt_info)
                               : "MJPEG");
    }

    /* Create audio AVStream */
    if (ctx->has_audio) {
        AVStream *ast = avformat_new_stream(s, NULL);
        int bps;

        if (!ast) { ret = AVERROR(ENOMEM); goto fail; }
        ctx->audio_stream_idx = ast->index;
        ast->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;

        bps = ctx->uac.subframe_size ? ctx->uac.subframe_size : 2;
        if (ctx->uac.format_tag == UAC_FORMAT_IEEE_FLOAT && bps == 4)
            ast->codecpar->codec_id = AV_CODEC_ID_PCM_F32LE;
        else if (bps == 4)
            ast->codecpar->codec_id = AV_CODEC_ID_PCM_S32LE;
        else if (bps == 3)
            ast->codecpar->codec_id = AV_CODEC_ID_PCM_S24LE;
        else {
            ast->codecpar->codec_id = AV_CODEC_ID_PCM_S16LE;
            bps = 2;
        }

        ast->codecpar->sample_rate = ctx->uac.active_sample_rate
                                     ? ctx->uac.active_sample_rate : 48000;
        ast->codecpar->ch_layout.nb_channels = ctx->uac.channels
                                               ? ctx->uac.channels : 2;
        ast->codecpar->ch_layout.order = AV_CHANNEL_ORDER_UNSPEC;
        ast->codecpar->bits_per_coded_sample = ctx->uac.bit_resolution
                                               ? ctx->uac.bit_resolution : 16;
        ast->codecpar->block_align =
            ast->codecpar->ch_layout.nb_channels * bps;
        ast->codecpar->bit_rate =
            ast->codecpar->sample_rate * ast->codecpar->block_align * 8;

        avpriv_set_pts_info(ast, 64, 1, 1000000);

        av_log(s, AV_LOG_INFO, "Audio stream[%d]: %d Hz, %d ch\n",
               ast->index, ast->codecpar->sample_rate,
               ast->codecpar->ch_layout.nb_channels);
    }

    /* Submit initial ISO transfers.
     * The UVC/UAC parsers strip the IN direction bit from endpoint addresses
     * (they only store the endpoint number).  libusb needs the full address
     * with direction bit 7 set for IN endpoints. */
    if (ctx->has_video) {
        uint16_t vid_pkt = ctx->uvc.max_packet_size
                           ? ctx->uvc.max_packet_size : ISO_MAX_PACKET_SIZE;
        ret = iso_ring_init(s, ctx, &ctx->video_ring,
                            ctx->uvc.vs_endpoint | 0x80,
                            ISO_VIDEO_NUM_PACKETS, vid_pkt,
                            ISO_TRANSFERS_VIDEO);
        if (ret < 0) goto fail;
    }
    if (ctx->has_audio) {
        uint16_t aud_pkt = ctx->uac.max_packet_size
                           ? ctx->uac.max_packet_size : 192;
        ret = iso_ring_init(s, ctx, &ctx->audio_ring,
                            ctx->uac.as_endpoint | 0x80,
                            ISO_AUDIO_NUM_PACKETS, aud_pkt,
                            ISO_TRANSFERS_AUDIO);
        if (ret < 0) goto fail;
    }

    ctx->timing_fp = ipmx_timing_open("/tmp/ipmx_timing_capture.log");
    if (ctx->timing_fp)
        fprintf(ctx->timing_fp, "# wall_us\tevent\tmedia\tcounter\tcapture_us\tqdepth\n");

    av_log(s, AV_LOG_INFO, "Local USB device ready: %d stream(s)\n",
           s->nb_streams);
    return 0;

fail:
    return ret;
}

/* ---------------------------------------------------------------------------
 * Process one completed ISO transfer for video
 * --------------------------------------------------------------------------- */

static int process_video_iso(AVFormatContext *s, IpmxUsbLocalContext *ctx,
                             struct libusb_transfer *xfer, AVPacket *pkt)
{
    int i, got_frame = 0;

    for (i = 0; i < xfer->num_iso_packets; i++) {
        struct libusb_iso_packet_descriptor *iso = &xfer->iso_packet_desc[i];
        if (iso->status != LIBUSB_TRANSFER_COMPLETED || iso->actual_length <= 0)
            continue;

        {
            uint8_t *data = libusb_get_iso_packet_buffer_simple(xfer, i);
            ipmx_uvc_process_payload(&ctx->uvc, data,
                                     iso->actual_length);
        }

        if (!got_frame && ipmx_uvc_frame_ready(&ctx->uvc)) {
            uint8_t *frame_data;
            int frame_size;
            if (ipmx_uvc_get_frame(&ctx->uvc, &frame_data, &frame_size) == 0
                && frame_size > 0) {
                uint32_t uvc_pts_val;
                uint32_t scr_stc;
                uint16_t scr_sof;
                int have_uvc_pts = ipmx_uvc_get_frame_pts(&ctx->uvc, &uvc_pts_val);
                int have_scr     = ipmx_uvc_get_frame_scr(&ctx->uvc, &scr_stc, &scr_sof);
                uint32_t fn      = capture_frame_number(ctx);

                int ret = av_new_packet(pkt, frame_size);
                if (ret < 0) return ret;
                memcpy(pkt->data, frame_data, frame_size);
                pkt->stream_index = ctx->video_stream_idx;

                if (have_uvc_pts && ctx->uvc.clock_frequency > 0)
                    pkt->pts = unwrap_uvc_pts(ctx, uvc_pts_val)
                               * 1000000LL / ctx->uvc.clock_frequency;
                else
                    pkt->pts = capture_pts_us(ctx);

                {
                    IpmxUsbTimingSideData *sd = ipmx_timing_add_side_data(pkt);
                    if (sd) {
                        sd->frame_number     = fn;
                        sd->clock_frequency  = ctx->uvc.clock_frequency;
                        sd->capture_wall_us  = av_gettime_relative();
                        if (have_uvc_pts) {
                            sd->device_pts     = uvc_pts_val;
                            sd->has_device_pts = 1;
                        }
                        if (have_scr) {
                            sd->scr_stc = scr_stc;
                            sd->scr_sof = scr_sof;
                            sd->has_scr = 1;
                        }
                    }
                }

                ctx->video_frame_count++;
                pkt->flags |= AV_PKT_FLAG_KEY;
                /* Stamp capture wall time into opaque_ref.  This field
                 * survives the sender→receiver pipeline (ENQ and NET_RCV
                 * overwrite wall_us but leave capture_wall_us alone). */
                {
                    int64_t cap_us = av_gettime_relative();
                    ipmx_timing_force_stamp_pkt(pkt);
                    if (pkt->opaque_ref &&
                        pkt->opaque_ref->size >= (int)sizeof(IpmxTimingOpaque)) {
                        IpmxTimingOpaque *t =
                            (IpmxTimingOpaque *)pkt->opaque_ref->data;
                        t->capture_wall_us = cap_us;
                    }
                    if (ctx->timing_fp) {
                        fprintf(ctx->timing_fp,
                                "%lld\tCAP\tV\t%lld\t%lld\t-\n",
                                (long long)cap_us, (long long)ctx->timing_vid_cnt,
                                (long long)cap_us);
                        ctx->timing_vid_cnt++;
                    }
                }
                got_frame = 1;
            }
        }
    }

    return got_frame ? 0 : AVERROR(EAGAIN);
}

/* ---------------------------------------------------------------------------
 * Process one completed ISO transfer for audio
 * --------------------------------------------------------------------------- */

static int process_audio_iso(AVFormatContext *s, IpmxUsbLocalContext *ctx,
                             struct libusb_transfer *xfer, AVPacket *pkt)
{
    uint8_t pcm_buf[PCM_BUF_SIZE];
    int pcm_total = 0;
    int i;

    for (i = 0; i < xfer->num_iso_packets; i++) {
        struct libusb_iso_packet_descriptor *iso = &xfer->iso_packet_desc[i];
        if (iso->status != LIBUSB_TRANSFER_COMPLETED || iso->actual_length <= 0)
            continue;

        {
            uint8_t *data = libusb_get_iso_packet_buffer_simple(xfer, i);
            int n = ipmx_uac_extract_audio(&ctx->uac, data,
                                           iso->actual_length,
                                           pcm_buf + pcm_total,
                                           PCM_BUF_SIZE - pcm_total);
            if (n > 0)
                pcm_total += n;
        }
    }

    if (pcm_total > 0) {
        uint32_t fn = capture_frame_number(ctx);
        int bytes_per_sample = ctx->uac.channels
                               * (ctx->uac.bit_resolution / 8);
        int num_samples = bytes_per_sample > 0
                          ? pcm_total / bytes_per_sample : 0;
        uint32_t rate = ctx->uac.active_sample_rate;
        int ret = av_new_packet(pkt, pcm_total);
        if (ret < 0)
            return ret;
        memcpy(pkt->data, pcm_buf, pcm_total);
        pkt->stream_index = ctx->audio_stream_idx;

        if (rate > 0)
            pkt->pts = ctx->audio_sample_count * 1000000LL / rate;
        else
            pkt->pts = capture_pts_us(ctx);

        {
            IpmxUsbTimingSideData *sd = ipmx_timing_add_side_data(pkt);
            if (sd) {
                sd->frame_number    = fn;
                sd->capture_wall_us = av_gettime_relative();
            }
        }
        ctx->audio_sample_count += num_samples;
        {
            int64_t cap_us = av_gettime_relative();
            ipmx_timing_force_stamp_pkt(pkt);
            if (pkt->opaque_ref &&
                pkt->opaque_ref->size >= (int)sizeof(IpmxTimingOpaque)) {
                IpmxTimingOpaque *t =
                    (IpmxTimingOpaque *)pkt->opaque_ref->data;
                t->capture_wall_us = cap_us;
            }
            if (ctx->timing_fp) {
                fprintf(ctx->timing_fp,
                        "%lld\tCAP\tA\t%lld\t%lld\t-\n",
                        (long long)cap_us, (long long)ctx->timing_aud_cnt,
                        (long long)cap_us);
                ctx->timing_aud_cnt++;
            }
        }
        return 0;
    }

    return AVERROR(EAGAIN);
}

/* ---------------------------------------------------------------------------
 * Reap one completed transfer from a ring, resubmit it
 * --------------------------------------------------------------------------- */

static int iso_ring_reap(AVFormatContext *s, IpmxUsbLocalContext *ctx,
                         IsoRing *ring, AVPacket *pkt, int is_video)
{
    struct libusb_transfer *xfer;
    int ret;

    if (ring->completed <= 0)
        return AVERROR(EAGAIN);

    xfer = ring->xfers[ring->next_reap];
    ring->next_reap = (ring->next_reap + 1) % ring->count;
    ring->completed--;

    if (xfer->status != LIBUSB_TRANSFER_COMPLETED &&
        xfer->status != LIBUSB_TRANSFER_TIMED_OUT) {
        av_log(s, AV_LOG_DEBUG, "ISO transfer status %d on ep 0x%02x\n",
               xfer->status, ring->endpoint);
    }

    if (is_video)
        ret = process_video_iso(s, ctx, xfer, pkt);
    else
        ret = process_audio_iso(s, ctx, xfer, pkt);

    /* Reset and resubmit */
    libusb_set_iso_packet_lengths(xfer, ring->max_pkt_size);
    {
        int submit_ret = libusb_submit_transfer(xfer);
        if (submit_ret < 0) {
            av_log(s, AV_LOG_ERROR, "Failed to resubmit ISO: %s\n",
                   libusb_strerror(submit_ret));
        }
    }

    return ret;
}

/* ---------------------------------------------------------------------------
 * read_packet
 * --------------------------------------------------------------------------- */

static int ipmx_usb_local_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    IpmxUsbLocalContext *ctx = s->priv_data;
    struct timeval tv;
    int ret, pass;

    if (ctx->stop)
        return AVERROR_EOF;

    for (pass = 0; pass < 200; pass++) {
        tv.tv_sec  = 0;
        tv.tv_usec = 10000;
        ret = libusb_handle_events_timeout(ctx->usb_ctx, &tv);
        if (ret < 0 && ret != LIBUSB_ERROR_TIMEOUT) {
            av_log(s, AV_LOG_ERROR,
                   "libusb_handle_events error: %s\n", libusb_strerror(ret));
        }

        /* Round-robin between video and audio to prevent starvation */
        if (ctx->next_is_audio && ctx->has_audio) {
            ret = iso_ring_reap(s, ctx, &ctx->audio_ring, pkt, 0);
            if (ret == 0) {
                ctx->next_is_audio = 0;
                return 0;
            }
        }

        if (ctx->has_video) {
            ret = iso_ring_reap(s, ctx, &ctx->video_ring, pkt, 1);
            if (ret == 0) {
                ctx->next_is_audio = ctx->has_audio;
                return 0;
            }
        }

        if (!ctx->next_is_audio && ctx->has_audio) {
            ret = iso_ring_reap(s, ctx, &ctx->audio_ring, pkt, 0);
            if (ret == 0) {
                ctx->next_is_audio = 0;
                return 0;
            }
        }
    }

    return AVERROR(EAGAIN);
}

/* ---------------------------------------------------------------------------
 * read_close
 * --------------------------------------------------------------------------- */

static int ipmx_usb_local_read_close(AVFormatContext *s)
{
    IpmxUsbLocalContext *ctx = s->priv_data;
    int i;

    if (ctx->timing_fp) {
        fclose(ctx->timing_fp);
        ctx->timing_fp = NULL;
    }

    ctx->stop = 1;

    /* Cancel all in-flight ISO transfers */
    iso_ring_cancel(&ctx->video_ring);
    iso_ring_cancel(&ctx->audio_ring);

    /* Drain libusb events so cancellation callbacks complete */
    if (ctx->usb_ctx) {
        struct timeval tv = { 0, 200000 };
        libusb_handle_events_timeout(ctx->usb_ctx, &tv);
    }

    /* Now safe to free transfer memory */
    iso_ring_free(&ctx->video_ring);
    iso_ring_free(&ctx->audio_ring);

    /* Reset alt settings to 0 before releasing */
    if (ctx->dev_handle) {
        if (ctx->has_video)
            libusb_set_interface_alt_setting(ctx->dev_handle,
                                             ctx->uvc.vs_interface, 0);
        if (ctx->has_audio)
            libusb_set_interface_alt_setting(ctx->dev_handle,
                                             ctx->uac.as_interface, 0);
    }

    ipmx_uvc_free_reassembler(&ctx->uvc);

    /* Release claimed interfaces (auto-detach reattaches kernel drivers) */
    for (i = 0; i < ctx->num_ifaces_claimed; i++)
        libusb_release_interface(ctx->dev_handle, ctx->ifaces_claimed[i]);

    if (ctx->dev_handle) {
        libusb_close(ctx->dev_handle);
        ctx->dev_handle = NULL;
    }
    if (ctx->usb_ctx) {
        libusb_exit(ctx->usb_ctx);
        ctx->usb_ctx = NULL;
    }

    av_log(s, AV_LOG_INFO, "Local USB device closed\n");
    return 0;
}

/* ---------------------------------------------------------------------------
 * FFInputFormat registration
 * --------------------------------------------------------------------------- */

const FFInputFormat ff_ipmx_usb_local_demuxer = {
    .p.name         = "ipmx_usb_local",
    .p.long_name    = NULL_IF_CONFIG_SMALL("Local USB device via libusb "
                                           "(shares UVC/UAC with IPMX USB)"),
    .p.flags        = AVFMT_NOFILE,
    .p.priv_class   = &ipmx_usb_local_class,
    .priv_data_size = sizeof(IpmxUsbLocalContext),
    .read_header    = ipmx_usb_local_read_header,
    .read_packet    = ipmx_usb_local_read_packet,
    .read_close     = ipmx_usb_local_read_close,
};
