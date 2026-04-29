/*
 * IPMX USB virtual USB descriptor builder for Sender device.
 *
 * Generates USB device/config/string descriptor chains at runtime
 * from AVStream codec parameters, emulating a UVC webcam + UAC microphone.
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

#ifndef AVDEVICE_IPMX_USB_VIRT_DESC_H
#define AVDEVICE_IPMX_USB_VIRT_DESC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IPMX_VIRT_DESC_MAX_CONFIG  2048
#define IPMX_VIRT_DESC_MAX_STRING  256

enum IpmxUsbVideoCodec {
    IPMX_USB_VCODEC_MJPEG = 0,
    IPMX_USB_VCODEC_H264,
    IPMX_USB_VCODEC_HEVC,
};

/* Virtual device parameters */
typedef struct IpmxUsbVirtDescParams {
    uint16_t vid;
    uint16_t pid;
    const char *manufacturer;
    const char *product;
    const char *serial;

    /* Video stream (0 to omit) */
    int video_width;
    int video_height;
    int video_fps;
    enum IpmxUsbVideoCodec video_codec;
    uint16_t video_max_pkt_size; /* ISO endpoint max_packet_size */
    uint32_t video_clock_freq;   /* dwClockFrequency (Hz); 0 = use default 1 MHz */

    /* Audio stream (0 channels to omit) */
    uint32_t audio_sample_rate;
    uint8_t  audio_channels;
    uint8_t  audio_bit_depth;    /* 16, 24, or 32 */
    uint16_t audio_max_pkt_size;
    int      audio_uac2;         /* 0 = UAC 1.0 (default), 1 = UAC 2.0 */
} IpmxUsbVirtDescParams;

/* Pre-built descriptor set */
typedef struct IpmxUsbVirtDescSet {
    uint8_t device_desc[18];
    uint8_t config_desc[IPMX_VIRT_DESC_MAX_CONFIG];
    int     config_desc_len;

    /* Endpoint addresses assigned during build */
    uint8_t video_ep;    /* e.g. 0x81 */
    uint8_t audio_ep;    /* e.g. 0x82 */
    uint8_t video_vs_iface;
    uint8_t audio_as_iface;
} IpmxUsbVirtDescSet;

/**
 * Build complete USB descriptor set for a virtual webcam/microphone.
 * Returns 0 on success, negative on error.
 */
int ipmx_usb_virt_desc_build(IpmxUsbVirtDescSet *out,
                              const IpmxUsbVirtDescParams *params);

/**
 * Build a USB string descriptor for the given index.
 *
 * Index 0 returns the language ID descriptor (English US).
 * Index 1 = manufacturer, 2 = product, 3 = serial.
 *
 * @param buf     Output buffer (at least IPMX_VIRT_DESC_MAX_STRING bytes)
 * @param buf_size Buffer size
 * @param index   String descriptor index (0..3)
 * @param params  Virtual device params (for manufacturer/product/serial)
 * @return Number of bytes written, or negative on error.
 */
int ipmx_usb_virt_desc_string(uint8_t *buf, int buf_size,
                               uint8_t index,
                               const IpmxUsbVirtDescParams *params);

#ifdef __cplusplus
}
#endif

#endif /* AVDEVICE_IPMX_USB_VIRT_DESC_H */
