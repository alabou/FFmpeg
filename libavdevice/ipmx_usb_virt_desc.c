/*
 * IPMX USB virtual USB descriptor builder for Sender device.
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

#include "ipmx_usb_virt_desc.h"
#include <string.h>

/* USB descriptor type constants */
#define DT_DEVICE        0x01
#define DT_CONFIG        0x02
#define DT_STRING        0x03
#define DT_INTERFACE     0x04
#define DT_ENDPOINT      0x05
#define DT_IAD           0x0B
#define DT_CS_INTERFACE  0x24
#define DT_CS_ENDPOINT   0x25

/* UVC class-specific subtypes */
#define UVC_VC_HEADER           0x01
#define UVC_VC_INPUT_TERMINAL   0x02
#define UVC_VC_OUTPUT_TERMINAL  0x03
#define UVC_VC_PROCESSING_UNIT  0x05
#define UVC_VS_INPUT_HEADER     0x01
#define UVC_VS_FORMAT_MJPEG     0x06
#define UVC_VS_FRAME_MJPEG      0x07
#define UVC_VS_FORMAT_FRAME     0x10
#define UVC_VS_FRAME_FRAME      0x11
#define UVC_VS_COLOR_MATCHING   0x0D

/* UAC 1.0 class-specific subtypes */
#define UAC_AC_HEADER           0x01
#define UAC_AC_INPUT_TERMINAL   0x02
#define UAC_AC_OUTPUT_TERMINAL  0x03
#define UAC_AC_FEATURE_UNIT     0x06
#define UAC_AS_GENERAL          0x01
#define UAC_AS_FORMAT_TYPE      0x02

/* Terminal types */
#define ITT_CAMERA              0x0201
#define OTT_STREAMING           0x0101
#define ITT_MICROPHONE          0x0201  /* UAC input terminal type */
#define OTT_USB_STREAMING       0x0101  /* UAC output terminal type */

/* H.264 Payload GUID: {34363248-0000-0010-8000-00AA00389B71} */
static const uint8_t GUID_H264[16] = {
    0x48, 0x32, 0x36, 0x34, 0x00, 0x00, 0x10, 0x00,
    0x80, 0x00, 0x00, 0xAA, 0x00, 0x38, 0x9B, 0x71
};

/* HEVC (H.265) Payload GUID: {35363248-0000-0010-8000-00AA00389B71} */
static const uint8_t GUID_HEVC[16] = {
    0x48, 0x32, 0x36, 0x35, 0x00, 0x00, 0x10, 0x00,
    0x80, 0x00, 0x00, 0xAA, 0x00, 0x38, 0x9B, 0x71
};

/* Helper to append bytes and advance cursor */
static int put(uint8_t *buf, int pos, int max, const uint8_t *data, int len)
{
    if (pos + len > max)
        return -1;
    memcpy(buf + pos, data, len);
    return pos + len;
}

static int put8(uint8_t *buf, int pos, int max, uint8_t v)
{
    if (pos + 1 > max) return -1;
    buf[pos] = v;
    return pos + 1;
}

static int put16le(uint8_t *buf, int pos, int max, uint16_t v)
{
    if (pos + 2 > max) return -1;
    buf[pos]     = v & 0xFF;
    buf[pos + 1] = (v >> 8) & 0xFF;
    return pos + 2;
}

static int put32le(uint8_t *buf, int pos, int max, uint32_t v)
{
    if (pos + 4 > max) return -1;
    buf[pos]     = v & 0xFF;
    buf[pos + 1] = (v >> 8) & 0xFF;
    buf[pos + 2] = (v >> 16) & 0xFF;
    buf[pos + 3] = (v >> 24) & 0xFF;
    return pos + 4;
}

/* Patch a 16-bit LE value at a known offset */
static void patch16le(uint8_t *buf, int offset, uint16_t v)
{
    buf[offset]     = v & 0xFF;
    buf[offset + 1] = (v >> 8) & 0xFF;
}

int ipmx_usb_virt_desc_build(IpmxUsbVirtDescSet *out,
                              const IpmxUsbVirtDescParams *params)
{
    int has_video = (params->video_width > 0 && params->video_height > 0);
    int has_audio = (params->audio_channels > 0 && params->audio_sample_rate > 0);
    int num_ifaces = 0;
    uint8_t iface_vc = 0, iface_vs = 0, iface_ac = 0, iface_as = 0;
    uint8_t ep_video = 0x81, ep_audio = 0x82;
    int p, cfg_start;
    uint16_t vid = params->vid  ? params->vid  : 0xFF01;
    uint16_t pid = params->pid  ? params->pid  : 0x0001;
    uint16_t v_max_pkt = params->video_max_pkt_size ? params->video_max_pkt_size : 1024;
    uint16_t a_max_pkt = params->audio_max_pkt_size;
    int bps, bytes_per_ms;

    if (!has_video && !has_audio)
        return -1;

    memset(out, 0, sizeof(*out));

    /* --- Device Descriptor (18 bytes) --- */
    {
        uint8_t *d = out->device_desc;
        d[0]  = 18;            /* bLength */
        d[1]  = DT_DEVICE;
        d[2]  = 0x00; d[3] = 0x02; /* bcdUSB = 2.00 */
        d[4]  = 0xEF;          /* bDeviceClass = Misc (IAD) */
        d[5]  = 0x02;          /* bDeviceSubClass = Common Class */
        d[6]  = 0x01;          /* bDeviceProtocol = IAD */
        d[7]  = 0x40;          /* bMaxPacketSize0 = 64 */
        d[8]  = vid & 0xFF;
        d[9]  = (vid >> 8) & 0xFF;
        d[10] = pid & 0xFF;
        d[11] = (pid >> 8) & 0xFF;
        d[12] = 0x00; d[13] = 0x01; /* bcdDevice = 1.00 */
        d[14] = 1;             /* iManufacturer = string 1 */
        d[15] = 2;             /* iProduct = string 2 */
        d[16] = 3;             /* iSerialNumber = string 3 */
        d[17] = 1;             /* bNumConfigurations */
    }

    /* Assign interface numbers */
    if (has_video) {
        iface_vc = num_ifaces++;
        iface_vs = num_ifaces++;
    }
    if (has_audio) {
        iface_ac = num_ifaces++;
        iface_as = num_ifaces++;
    }
    if (has_video && !has_audio) {
        ep_video = 0x81;
    } else if (!has_video && has_audio) {
        ep_audio = 0x81;
    } else {
        ep_video = 0x81;
        ep_audio = 0x82;
    }

    out->video_ep       = has_video ? ep_video : 0;
    out->audio_ep       = has_audio ? ep_audio : 0;
    out->video_vs_iface = has_video ? iface_vs : 0xFF;
    out->audio_as_iface = has_audio ? iface_as : 0xFF;

    /* Compute audio bytes per ms for max_packet_size default */
    bps = params->audio_bit_depth ? (params->audio_bit_depth / 8) : 2;
    bytes_per_ms = (params->audio_sample_rate * params->audio_channels * bps) / 1000;
    if (!a_max_pkt && has_audio)
        a_max_pkt = (uint16_t)(bytes_per_ms + bps * params->audio_channels);

    /* --- Configuration Descriptor chain --- */
    p = 0;
    cfg_start = 0;
    {
        uint8_t *c = out->config_desc;
        int max = IPMX_VIRT_DESC_MAX_CONFIG;

        /* Configuration descriptor (9 bytes) — wTotalLength patched later */
        uint8_t cfg_hdr[9] = {
            9, DT_CONFIG, 0, 0,  /* wTotalLength placeholder */
            num_ifaces, 1, 0, 0xA0, 0xFA
        };
        p = put(c, p, max, cfg_hdr, 9);
        if (p < 0) return -1;
        cfg_start = 0;

        /* ============ VIDEO ============ */
        if (has_video) {
            uint32_t interval;
            uint32_t max_frame_size;
            int vc_total;

            /* IAD for Video (8) */
            {
                uint8_t iad[8] = {
                    8, DT_IAD, iface_vc, 2, 0x0E, 0x03, 0x00, 0x02
                };
                p = put(c, p, max, iad, 8);
                if (p < 0) return -1;
            }

            /* VC Interface (9) */
            {
                uint8_t vi[9] = {
                    9, DT_INTERFACE, iface_vc, 0, 0, 0x0E, 0x01, 0x00, 0x02
                };
                p = put(c, p, max, vi, 9);
                if (p < 0) return -1;
            }

            /* VC Header (13) */
            vc_total = 13 + 18 + 11 + 9; /* header + IT + PU + OT */
            {
                uint32_t clk = params->video_clock_freq
                               ? params->video_clock_freq : 1000000;
                uint8_t vch[13] = {
                    13, DT_CS_INTERFACE, UVC_VC_HEADER,
                    0x10, 0x01,   /* bcdUVC = 1.10 */
                    (uint8_t)(vc_total & 0xFF), (uint8_t)((vc_total >> 8) & 0xFF),
                    (uint8_t)(clk >>  0), (uint8_t)(clk >>  8),
                    (uint8_t)(clk >> 16), (uint8_t)(clk >> 24),
                    0x01, iface_vs
                };
                p = put(c, p, max, vch, 13);
                if (p < 0) return -1;
            }

            /* Input Terminal Camera (18) */
            {
                uint8_t it[18] = {
                    18, DT_CS_INTERFACE, UVC_VC_INPUT_TERMINAL,
                    0x01,         /* bTerminalID = 1 */
                    0x01, 0x02,   /* wTerminalType = ITT_CAMERA */
                    0x00, 0x00,   /* bAssocTerminal, iTerminal */
                    0x00, 0x00,   /* wObjectiveFocalLengthMin */
                    0x00, 0x00,   /* wObjectiveFocalLengthMax */
                    0x00, 0x00,   /* wOcularFocalLength */
                    0x03,         /* bControlSize = 3 */
                    0x00, 0x00, 0x00  /* bmControls */
                };
                p = put(c, p, max, it, 18);
                if (p < 0) return -1;
            }

            /* Processing Unit (11) */
            {
                uint8_t pu[11] = {
                    11, DT_CS_INTERFACE, UVC_VC_PROCESSING_UNIT,
                    0x02, 0x01,   /* bUnitID=2, bSourceID=1 */
                    0x00, 0x00,   /* wMaxMultiplier */
                    0x02,         /* bControlSize = 2 */
                    0x01, 0x00,   /* bmControls = brightness */
                    0x00          /* iProcessing */
                };
                p = put(c, p, max, pu, 11);
                if (p < 0) return -1;
            }

            /* Output Terminal (9) */
            {
                uint8_t ot[9] = {
                    9, DT_CS_INTERFACE, UVC_VC_OUTPUT_TERMINAL,
                    0x03, 0x01, 0x01, /* bTerminalID=3, wTerminalType=OTT_STREAMING */
                    0x00, 0x02, 0x00  /* bAssocTerminal, bSourceID=2(PU), iTerminal */
                };
                p = put(c, p, max, ot, 9);
                if (p < 0) return -1;
            }

            /* VS Interface Alt0 (9) — zero-bandwidth */
            {
                uint8_t vs0[9] = {
                    9, DT_INTERFACE, iface_vs, 0, 0, 0x0E, 0x02, 0x00, 0x00
                };
                p = put(c, p, max, vs0, 9);
                if (p < 0) return -1;
            }

            /* VS Input Header (13) */
            {
                uint8_t vsh[13] = {
                    13, DT_CS_INTERFACE, UVC_VS_INPUT_HEADER,
                    0x01,         /* bNumFormats = 1 */
                    0x00, 0x00,   /* wTotalLength placeholder — patched below */
                    ep_video,     /* bEndpointAddress */
                    0x00,         /* bmInfo */
                    0x03,         /* bTerminalLink = OT(3) */
                    0x00,         /* bStillCaptureMethod */
                    0x00,         /* bTriggerSupport */
                    0x00,         /* bTriggerUsage */
                    0x01          /* bControlSize = 1 */
                };
                int vs_hdr_off = p;
                p = put(c, p, max, vsh, 13);
                if (p < 0) return -1;

                /* VS Format + Frame + Color Matching will follow;
                 * compute VS total after writing all VS class-specific */
                (void)vs_hdr_off;
            }

            interval = params->video_fps > 0
                       ? 10000000U / params->video_fps
                       : 333333;
            max_frame_size = (uint32_t)params->video_width *
                             params->video_height * 2;

            if (params->video_codec == IPMX_USB_VCODEC_H264 ||
                params->video_codec == IPMX_USB_VCODEC_HEVC) {
                const uint8_t *guid = (params->video_codec == IPMX_USB_VCODEC_HEVC)
                                      ? GUID_HEVC : GUID_H264;

                /* VS Format Frame-Based (28) */
                int fmt_off = p;
                p = put8(c, p, max, 28);                   /* bLength */
                p = put8(c, p, max, DT_CS_INTERFACE);
                p = put8(c, p, max, UVC_VS_FORMAT_FRAME);
                p = put8(c, p, max, 1);                    /* bFormatIndex */
                p = put8(c, p, max, 1);                    /* bNumFrameDescriptors */
                p = put(c, p, max, guid, 16);              /* guidFormat */
                p = put8(c, p, max, 0);                    /* bBitsPerPixel */
                p = put8(c, p, max, 1);                    /* bDefaultFrameIndex */
                p = put8(c, p, max, 0);                    /* bAspectRatioX */
                p = put8(c, p, max, 0);                    /* bAspectRatioY */
                p = put8(c, p, max, 0);                    /* bmInterlaceFlags */
                p = put8(c, p, max, 0);                    /* bCopyProtect */
                p = put8(c, p, max, 0);                    /* bVariableSize */
                if (p < 0) return -1;
                (void)fmt_off;

                /* VS Frame Frame-Based (30) */
                p = put8(c, p, max, 30);                   /* bLength */
                p = put8(c, p, max, DT_CS_INTERFACE);
                p = put8(c, p, max, UVC_VS_FRAME_FRAME);
                p = put8(c, p, max, 1);                    /* bFrameIndex */
                p = put8(c, p, max, 0x00);                 /* bmCapabilities */
                p = put16le(c, p, max, (uint16_t)params->video_width);
                p = put16le(c, p, max, (uint16_t)params->video_height);
                p = put32le(c, p, max, max_frame_size * 8 * params->video_fps);
                p = put32le(c, p, max, max_frame_size * 8 * params->video_fps);
                p = put32le(c, p, max, max_frame_size);
                p = put32le(c, p, max, interval);
                p = put8(c, p, max, 0x01);                 /* bFrameIntervalType=1(discrete) */
                p = put32le(c, p, max, interval);
                if (p < 0) return -1;
            } else {
                /* VS Format MJPEG (11) */
                p = put8(c, p, max, 11);
                p = put8(c, p, max, DT_CS_INTERFACE);
                p = put8(c, p, max, UVC_VS_FORMAT_MJPEG);
                p = put8(c, p, max, 1);                    /* bFormatIndex */
                p = put8(c, p, max, 1);                    /* bNumFrameDescriptors */
                p = put8(c, p, max, 0x01);                 /* bmFlags */
                p = put8(c, p, max, 1);                    /* bDefaultFrameIndex */
                p = put8(c, p, max, 0);                    /* bAspectRatioX */
                p = put8(c, p, max, 0);                    /* bAspectRatioY */
                p = put8(c, p, max, 0);                    /* bmInterlaceFlags */
                p = put8(c, p, max, 0);                    /* bCopyProtect */
                if (p < 0) return -1;

                /* VS Frame MJPEG (30) */
                p = put8(c, p, max, 30);
                p = put8(c, p, max, DT_CS_INTERFACE);
                p = put8(c, p, max, UVC_VS_FRAME_MJPEG);
                p = put8(c, p, max, 1);                    /* bFrameIndex */
                p = put8(c, p, max, 0x00);                 /* bmCapabilities */
                p = put16le(c, p, max, (uint16_t)params->video_width);
                p = put16le(c, p, max, (uint16_t)params->video_height);
                p = put32le(c, p, max, max_frame_size * 8 * params->video_fps);
                p = put32le(c, p, max, max_frame_size * 8 * params->video_fps);
                p = put32le(c, p, max, max_frame_size);
                p = put32le(c, p, max, interval);
                p = put8(c, p, max, 0x01);                 /* bFrameIntervalType=1 */
                p = put32le(c, p, max, interval);
                if (p < 0) return -1;
            }

            /* VS Color Matching (6) */
            {
                uint8_t cm[6] = {
                    6, DT_CS_INTERFACE, UVC_VS_COLOR_MATCHING,
                    0x01, 0x01, 0x04
                };
                p = put(c, p, max, cm, 6);
                if (p < 0) return -1;
            }

            /* VS Interface Alt1 (9) — active with ISO endpoint */
            {
                uint8_t vs1[9] = {
                    9, DT_INTERFACE, iface_vs, 1, 1, 0x0E, 0x02, 0x00, 0x00
                };
                p = put(c, p, max, vs1, 9);
                if (p < 0) return -1;
            }

            /* ISO Endpoint (7) */
            {
                uint8_t ep[7] = {
                    7, DT_ENDPOINT, ep_video,
                    0x05,   /* bmAttributes = isochronous, async */
                    (uint8_t)(v_max_pkt & 0xFF), (uint8_t)((v_max_pkt >> 8) & 0xFF),
                    0x01    /* bInterval = 1 */
                };
                p = put(c, p, max, ep, 7);
                if (p < 0) return -1;
            }
        }

        /* ============ AUDIO ============ */
        if (has_audio) {
            /* IAD for Audio (8) */
            {
                uint8_t iad[8] = {
                    8, DT_IAD, iface_ac, 2, 0x01, 0x00, 0x00, 0x00
                };
                p = put(c, p, max, iad, 8);
                if (p < 0) return -1;
            }

            /* AC Interface (9) */
            {
                uint8_t ai[9] = {
                    9, DT_INTERFACE, iface_ac, 0, 0, 0x01, 0x01, 0x00, 0x00
                };
                p = put(c, p, max, ai, 9);
                if (p < 0) return -1;
            }

            if (params->audio_uac2) {
                /* ------- UAC 2.0 AudioControl descriptors ------- */
                /* Entity IDs: Clock Source = 0x09, IT = 0x01, FU = 0x02, OT = 0x03 */
                int ac_hdr_off = p;

                /* AC Header (9 bytes, bcdADC = 2.00) */
                {
                    uint8_t ach[9] = {
                        9, DT_CS_INTERFACE, UAC_AC_HEADER,
                        0x00, 0x02,   /* bcdADC = 2.00 */
                        0x01,         /* bCategory = MICROPHONE (Desktop Microphone) */
                        0x00, 0x00,   /* wTotalLength (patched below) */
                        0x00          /* bmControls */
                    };
                    p = put(c, p, max, ach, 9);
                    if (p < 0) return -1;
                }

                /* Clock Source (8 bytes) */
                {
                    uint8_t cs[8] = {
                        8, DT_CS_INTERFACE, 0x0A,  /* UAC2_AC_CLOCK_SOURCE */
                        0x09,         /* bClockID = 9 */
                        0x01,         /* bmAttributes: internal fixed clock */
                        0x07,         /* bmControls: freq r/w + validity r */
                        0x00,         /* bAssocTerminal */
                        0x00          /* iClockSource */
                    };
                    p = put(c, p, max, cs, 8);
                    if (p < 0) return -1;
                }

                /* Input Terminal Microphone (17 bytes, UAC 2.0) */
                {
                    uint8_t it[17] = {
                        17, DT_CS_INTERFACE, UAC_AC_INPUT_TERMINAL,
                        0x01,         /* bTerminalID = 1 */
                        0x01, 0x02,   /* wTerminalType = ITT_MICROPHONE */
                        0x00,         /* bAssocTerminal */
                        0x09,         /* bCSourceID = Clock Source(9) */
                        params->audio_channels, /* bNrChannels */
                        0x00, 0x00, 0x00, 0x00, /* bmChannelConfig (unspecified) */
                        0x00,         /* iChannelNames */
                        0x00, 0x00,   /* bmControls */
                        0x00          /* iTerminal */
                    };
                    p = put(c, p, max, it, 17);
                    if (p < 0) return -1;
                }

                /* Feature Unit (10 + 4*(channels+1) bytes, UAC 2.0)
                 * Simplified: master channel only */
                {
                    uint8_t fu[14] = {
                        14, DT_CS_INTERFACE, UAC_AC_FEATURE_UNIT,
                        0x02,         /* bUnitID = 2 */
                        0x01,         /* bSourceID = IT(1) */
                        0x03, 0x00, 0x00, 0x00,  /* bmaControls(0): mute + volume */
                        0x00, 0x00, 0x00, 0x00,  /* bmaControls(1): none */
                        0x00          /* iFeature */
                    };
                    p = put(c, p, max, fu, 14);
                    if (p < 0) return -1;
                }

                /* Output Terminal USB Streaming (12 bytes, UAC 2.0) */
                {
                    uint8_t ot[12] = {
                        12, DT_CS_INTERFACE, UAC_AC_OUTPUT_TERMINAL,
                        0x03,         /* bTerminalID = 3 */
                        0x01, 0x01,   /* wTerminalType = OTT_USB_STREAMING */
                        0x00,         /* bAssocTerminal */
                        0x02,         /* bSourceID = FU(2) */
                        0x09,         /* bCSourceID = Clock Source(9) */
                        0x00, 0x00,   /* bmControls */
                        0x00          /* iTerminal */
                    };
                    p = put(c, p, max, ot, 12);
                    if (p < 0) return -1;
                }

                /* Patch wTotalLength in AC Header */
                {
                    uint16_t ac_total = (uint16_t)(p - ac_hdr_off);
                    patch16le(c, ac_hdr_off + 6, ac_total);
                }

                /* AS Interface Alt0 (9) — zero-bandwidth */
                {
                    uint8_t as0[9] = {
                        9, DT_INTERFACE, iface_as, 0, 0, 0x01, 0x02, 0x20, 0x00
                    };
                    p = put(c, p, max, as0, 9);
                    if (p < 0) return -1;
                }

                /* AS Interface Alt1 (9) — active */
                {
                    uint8_t as1[9] = {
                        9, DT_INTERFACE, iface_as, 1, 1, 0x01, 0x02, 0x20, 0x00
                    };
                    p = put(c, p, max, as1, 9);
                    if (p < 0) return -1;
                }

                /* AS General (16 bytes, UAC 2.0) */
                {
                    uint8_t asg[16] = {
                        16, DT_CS_INTERFACE, UAC_AS_GENERAL,
                        0x03,         /* bTerminalLink = OT(3) */
                        0x00,         /* bmControls */
                        0x01,         /* bFormatType = FORMAT_TYPE_I */
                        0x01, 0x00, 0x00, 0x00,  /* bmFormats: bit 0 = PCM */
                        params->audio_channels,
                        0x00, 0x00, 0x00, 0x00,  /* bmChannelConfig */
                        0x00          /* iChannelNames */
                    };
                    p = put(c, p, max, asg, 16);
                    if (p < 0) return -1;
                }

                /* AS Format Type I (6 bytes, UAC 2.0 — no rate list) */
                {
                    uint8_t ft[6] = {
                        6, DT_CS_INTERFACE, UAC_AS_FORMAT_TYPE,
                        0x01,         /* bFormatType = FORMAT_TYPE_I */
                        (uint8_t)bps, /* bSubslotSize */
                        params->audio_bit_depth  /* bBitResolution */
                    };
                    p = put(c, p, max, ft, 6);
                    if (p < 0) return -1;
                }

                /* Standard ISO Endpoint (7) */
                {
                    uint8_t ep[7] = {
                        7, DT_ENDPOINT, ep_audio,
                        0x05,   /* bmAttributes = isochronous, async */
                        (uint8_t)(a_max_pkt & 0xFF), (uint8_t)((a_max_pkt >> 8) & 0xFF),
                        0x01    /* bInterval = 1 */
                    };
                    p = put(c, p, max, ep, 7);
                    if (p < 0) return -1;
                }

                /* CS Audio Data Endpoint (8 bytes, UAC 2.0) */
                {
                    uint8_t csep[8] = {
                        8, DT_CS_ENDPOINT, 0x01, /* EP_GENERAL */
                        0x00,         /* bmAttributes */
                        0x00,         /* bmControls */
                        0x00,         /* bLockDelayUnits */
                        0x00, 0x00    /* wLockDelay */
                    };
                    p = put(c, p, max, csep, 8);
                    if (p < 0) return -1;
                }
            } else {
                /* ------- UAC 1.0 AudioControl descriptors ------- */
                int ac_total = 9 + 12 + 9 + 9; /* header + IT + FU + OT */

                /* AC Header (9) */
                {
                    uint8_t ach[9] = {
                        9, DT_CS_INTERFACE, UAC_AC_HEADER,
                        0x00, 0x01,   /* bcdADC = 1.00 */
                        (uint8_t)(ac_total & 0xFF), (uint8_t)((ac_total >> 8) & 0xFF),
                        0x01, iface_as
                    };
                    p = put(c, p, max, ach, 9);
                    if (p < 0) return -1;
                }

                /* Input Terminal Microphone (12) */
                {
                    uint8_t it[12] = {
                        12, DT_CS_INTERFACE, UAC_AC_INPUT_TERMINAL,
                        0x01,         /* bTerminalID = 1 */
                        0x01, 0x02,   /* wTerminalType = ITT_MICROPHONE */
                        0x00,         /* bAssocTerminal */
                        params->audio_channels,
                        0x00, 0x00,   /* wChannelConfig */
                        0x00, 0x00    /* iChannelNames, iTerminal */
                    };
                    p = put(c, p, max, it, 12);
                    if (p < 0) return -1;
                }

                /* Feature Unit (9) */
                {
                    uint8_t fu[9] = {
                        9, DT_CS_INTERFACE, UAC_AC_FEATURE_UNIT,
                        0x02, 0x01,   /* bUnitID=2, bSourceID=1 */
                        0x01,         /* bControlSize = 1 */
                        0x03, 0x00,   /* bmaControls: mute + volume */
                        0x00          /* iFeature */
                    };
                    p = put(c, p, max, fu, 9);
                    if (p < 0) return -1;
                }

                /* Output Terminal USB Streaming (9) */
                {
                    uint8_t ot[9] = {
                        9, DT_CS_INTERFACE, UAC_AC_OUTPUT_TERMINAL,
                        0x03,         /* bTerminalID = 3 */
                        0x01, 0x01,   /* wTerminalType = OTT_USB_STREAMING */
                        0x00,         /* bAssocTerminal */
                        0x02,         /* bSourceID = FU(2) */
                        0x00          /* iTerminal */
                    };
                    p = put(c, p, max, ot, 9);
                    if (p < 0) return -1;
                }

                /* AS Interface Alt0 (9) — zero-bandwidth */
                {
                    uint8_t as0[9] = {
                        9, DT_INTERFACE, iface_as, 0, 0, 0x01, 0x02, 0x00, 0x00
                    };
                    p = put(c, p, max, as0, 9);
                    if (p < 0) return -1;
                }

                /* AS Interface Alt1 (9) — active */
                {
                    uint8_t as1[9] = {
                        9, DT_INTERFACE, iface_as, 1, 1, 0x01, 0x02, 0x00, 0x00
                    };
                    p = put(c, p, max, as1, 9);
                    if (p < 0) return -1;
                }

                /* AS General (7) */
                {
                    uint8_t asg[7] = {
                        7, DT_CS_INTERFACE, UAC_AS_GENERAL,
                        0x03,         /* bTerminalLink = OT(3) */
                        0x01,         /* bDelay = 1 frame */
                        0x01, 0x00    /* wFormatTag = PCM */
                    };
                    p = put(c, p, max, asg, 7);
                    if (p < 0) return -1;
                }

                /* AS Format Type I (11) — single discrete rate */
                {
                    uint32_t rate = params->audio_sample_rate;
                    uint8_t ft[11] = {
                        11, DT_CS_INTERFACE, UAC_AS_FORMAT_TYPE,
                        0x01,         /* bFormatType = FORMAT_TYPE_I */
                        params->audio_channels,
                        (uint8_t)bps, /* bSubframeSize */
                        params->audio_bit_depth,
                        0x01,         /* bSamFreqType = 1 (discrete) */
                        (uint8_t)(rate & 0xFF),
                        (uint8_t)((rate >> 8) & 0xFF),
                        (uint8_t)((rate >> 16) & 0xFF)
                    };
                    p = put(c, p, max, ft, 11);
                    if (p < 0) return -1;
                }

                /* Standard ISO Endpoint (7) */
                {
                    uint8_t ep[7] = {
                        7, DT_ENDPOINT, ep_audio,
                        0x05,   /* bmAttributes = isochronous, async */
                        (uint8_t)(a_max_pkt & 0xFF), (uint8_t)((a_max_pkt >> 8) & 0xFF),
                        0x01    /* bInterval = 1 */
                    };
                    p = put(c, p, max, ep, 7);
                    if (p < 0) return -1;
                }

                /* CS Audio Endpoint (7) */
                {
                    uint8_t csep[7] = {
                        7, DT_CS_ENDPOINT, 0x01,
                        0x01,         /* bmAttributes = sampling freq control */
                        0x00,         /* bLockDelayUnits */
                        0x00, 0x00    /* wLockDelay */
                    };
                    p = put(c, p, max, csep, 7);
                    if (p < 0) return -1;
                }
            }
        }

        /* Patch wTotalLength in config descriptor */
        out->config_desc_len = p;
        patch16le(out->config_desc, cfg_start + 2, (uint16_t)p);
    }

    return 0;
}

int ipmx_usb_virt_desc_string(uint8_t *buf, int buf_size,
                               uint8_t index,
                               const IpmxUsbVirtDescParams *params)
{
    const char *str;
    int slen, desc_len, i;

    if (index == 0) {
        /* Language ID descriptor: English (US) */
        if (buf_size < 4) return -1;
        buf[0] = 4;
        buf[1] = DT_STRING;
        buf[2] = 0x09;
        buf[3] = 0x04;
        return 4;
    }

    switch (index) {
    case 1: str = params->manufacturer ? params->manufacturer : "FFmpeg"; break;
    case 2: str = params->product ? params->product : "IPMX USB Sender"; break;
    case 3: str = params->serial ? params->serial : "0000000000"; break;
    default: return -1;
    }

    slen = (int)strlen(str);
    desc_len = 2 + slen * 2;
    if (desc_len > 255) desc_len = 255;
    if (desc_len > buf_size) return -1;

    buf[0] = (uint8_t)desc_len;
    buf[1] = DT_STRING;
    for (i = 0; i < slen && (2 + i * 2 + 1) < desc_len; i++) {
        buf[2 + i * 2]     = (uint8_t)str[i];
        buf[2 + i * 2 + 1] = 0;
    }

    return desc_len;
}
