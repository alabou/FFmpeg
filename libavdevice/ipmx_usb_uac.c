/*
 * IPMX USB UAC (USB Audio Class) 1.0 / 2.0 protocol handling for FFmpeg.
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

#include "ipmx_usb_uac.h"
#include "ipmx_usb_quirks.h"
#include <stdlib.h>
#include <string.h>

/* USB descriptor types */
#define USB_DT_INTERFACE            0x04
#define USB_DT_ENDPOINT             0x05
#define USB_DT_CS_INTERFACE         0x24

/* UAC 1.0 bcdADC values */
#define UAC_1_0_BCD                 0x0100
#define UAC_2_0_BCD                 0x0200

/* ---------------------------------------------------------------------------
 * ipmx_uac_parse_config
 * --------------------------------------------------------------------------- */

int ipmx_uac_parse_config(IpmxUacContext *ctx, const uint8_t *desc, int desc_len)
{
    int pos = 0;
    int in_ac = 0;
    int in_as_alt = 0;

    if (!ctx || !desc || desc_len < 2)
        return -1;

    memset(ctx, 0, sizeof(*ctx));
    ctx->ac_interface = 0xFF;
    ctx->as_interface = 0xFF;
    ctx->as_endpoint = 0xFF;
    ctx->uac_version = 1;

    while (pos + 2 <= desc_len) {
        uint8_t bLength = desc[pos];
        uint8_t bDescriptorType = desc[pos + 1];

        if (bLength < 2 || pos + bLength > desc_len)
            break;

        if (bDescriptorType == USB_DT_INTERFACE) {
            uint8_t bInterfaceNumber, bAlternateSetting, bInterfaceClass, bInterfaceSubClass;
            if (bLength < 9)
                goto next;
            bInterfaceNumber = desc[pos + 2];
            bAlternateSetting = desc[pos + 3];
            bInterfaceClass = desc[pos + 5];
            bInterfaceSubClass = desc[pos + 6];

            if (bInterfaceClass != UAC_CC_AUDIO) {
                in_ac = 0;
                in_as_alt = 0;
                goto next;
            }

            if (bInterfaceSubClass == UAC_SC_AUDIOCONTROL) {
                ctx->ac_interface = bInterfaceNumber;
                in_ac = 1;
                in_as_alt = 0;
            } else if (bInterfaceSubClass == UAC_SC_AUDIOSTREAMING) {
                in_ac = 0;
                /* Only process alternate setting > 0 (alt 0 = zero-bandwidth) */
                in_as_alt = (bAlternateSetting > 0);
                if (in_as_alt) {
                    ctx->as_interface = bInterfaceNumber;
                }
            } else {
                in_ac = 0;
                in_as_alt = 0;
            }
        } else if (bDescriptorType == USB_DT_CS_INTERFACE && in_ac) {
            uint8_t bDescriptorSubtype;
            if (bLength < 3)
                goto next;
            bDescriptorSubtype = desc[pos + 2];

            if (bDescriptorSubtype == UAC_AC_HEADER && bLength >= 8) {
                /* bcdADC at offset 3-4: 0x0100 = UAC 1.0, 0x0200 = UAC 2.0 */
                uint16_t bcdADC = desc[pos + 3] | (desc[pos + 4] << 8);
                if (bcdADC == UAC_2_0_BCD)
                    ctx->uac_version = 2;
                else
                    ctx->uac_version = 1;
            } else if (bDescriptorSubtype == UAC2_AC_CLOCK_SOURCE && bLength >= 8) {
                /* UAC 2.0 Clock Source descriptor: bClockID at pos+3 */
                ctx->clock_source_id = desc[pos + 3];
            } else if (bDescriptorSubtype == UAC_AC_INPUT_TERMINAL) {
                if (ctx->uac_version == 2 && bLength >= 17) {
                    /* UAC 2.0 IT: bNrChannels at pos+9 (not pos+7 as in UAC 1.0) */
                    uint8_t nr_ch = desc[pos + 9];
                    if (ctx->channels == 0)
                        ctx->channels = nr_ch;
                }
            }
        } else if (bDescriptorType == USB_DT_CS_INTERFACE && in_as_alt) {
            uint8_t bDescriptorSubtype;
            if (bLength < 3)
                goto next;
            bDescriptorSubtype = desc[pos + 2];

            if (bDescriptorSubtype == UAC_AS_GENERAL) {
                if (ctx->uac_version == 2) {
                    /* UAC 2.0 AS General: 16 bytes, bmFormats at pos+5..8 */
                    if (bLength >= 16) {
                        uint32_t bmFormats = desc[pos + 5] |
                            (desc[pos + 6] << 8) |
                            (desc[pos + 7] << 16) |
                            (desc[pos + 8] << 24);
                        if (bmFormats & 0x01)
                            ctx->format_tag = UAC_FORMAT_PCM;
                    }
                } else {
                    /* UAC 1.0 AS General: 7 bytes, wFormatTag at offset 5 */
                    if (bLength >= 7)
                        ctx->format_tag = desc[pos + 5] | (desc[pos + 6] << 8);
                }
            } else if (bDescriptorSubtype == UAC_AS_FORMAT_TYPE) {
                uint8_t bFormatType = (bLength >= 4) ? desc[pos + 3] : 0;
                if (bFormatType != UAC_FORMAT_TYPE_I)
                    goto next;

                if (ctx->uac_version == 2) {
                    /* UAC 2.0 Format Type I: 6 bytes, no sample rate list.
                     * bSubslotSize at pos+4, bBitResolution at pos+5 */
                    if (bLength >= 6) {
                        ctx->subframe_size = desc[pos + 4];
                        ctx->bit_resolution = desc[pos + 5];
                    }
                } else {
                    /* UAC 1.0 Format Type I: variable length with rate list */
                    uint8_t bSamFreqType;
                    if (bLength < 8)
                        goto next;
                    ctx->channels = desc[pos + 4];
                    ctx->subframe_size = desc[pos + 5];
                    ctx->bit_resolution = desc[pos + 6];
                    bSamFreqType = desc[pos + 7];
                    if (bSamFreqType > 0 && bLength >= 8 + 3 * bSamFreqType) {
                        int n = bSamFreqType;
                        if (n > IPMX_UAC_MAX_SAMPLE_RATES)
                            n = IPMX_UAC_MAX_SAMPLE_RATES;
                        ctx->num_sample_rates = n;
                        for (int i = 0; i < n; i++) {
                            int off = 8 + i * 3;
                            ctx->sample_rates[i] = desc[pos + off] |
                                (desc[pos + off + 1] << 8) |
                                (desc[pos + off + 2] << 16);
                        }
                    }
                }
            }
        } else if (bDescriptorType == USB_DT_ENDPOINT && in_as_alt) {
            uint8_t bEndpointAddress, bmAttributes;
            uint16_t wMaxPacketSize;
            if (bLength < 7)
                goto next;
            bEndpointAddress = desc[pos + 2];
            bmAttributes = desc[pos + 3];
            wMaxPacketSize = desc[pos + 4] | (desc[pos + 5] << 8);

            /* Isochronous IN endpoint: (bmAttributes & 0x03) == 0x01, direction IN */
            if ((bmAttributes & 0x03) == 0x01 && (bEndpointAddress & 0x80)) {
                ctx->as_endpoint = bEndpointAddress & 0x0F;
                ctx->max_packet_size = wMaxPacketSize;
            }
        }

next:
        pos += bLength;
    }

    /* Validate we found a usable config */
    if (ctx->ac_interface == 0xFF || ctx->as_interface == 0xFF ||
        ctx->as_endpoint == 0xFF || ctx->subframe_size == 0)
        return -1;

    /* UAC 1.0 requires channels from Format Type; UAC 2.0 may get them from IT */
    if (ctx->channels == 0)
        return -1;

    return 0;
}

/* ---------------------------------------------------------------------------
 * ipmx_uac_build_set_interface_req
 * --------------------------------------------------------------------------- */

int ipmx_uac_build_set_interface_req(uint8_t *buf, uint8_t interface, uint8_t alt_setting)
{
    if (!buf)
        return -1;
    /* bmRequestType=0x01 (class, interface, host-to-device), bRequest=0x0B (SET_INTERFACE) */
    buf[0] = 0x01;
    buf[1] = 0x0B;
    buf[2] = alt_setting;
    buf[3] = 0x00;
    buf[4] = interface;
    buf[5] = 0x00;
    buf[6] = 0x00;
    buf[7] = 0x00;
    return 8;
}

/* ---------------------------------------------------------------------------
 * ipmx_uac_extract_audio
 * --------------------------------------------------------------------------- */

int ipmx_uac_extract_audio(IpmxUacContext *ctx, const uint8_t *iso_data, int iso_data_len,
                           uint8_t *pcm_out, int pcm_out_size)
{
    int copy_len, skip = 0;

    if (!ctx || !iso_data || !pcm_out || iso_data_len < 0 || pcm_out_size < 0)
        return -1;

    /* ALIGN_TRANSFER quirk: some devices send misaligned isochronous payload;
     * skip leading bytes so the first sample is aligned. */
    if (ctx->quirks & IPMX_UAC_QUIRK_ALIGN_TRANSFER) {
        unsigned align = (ctx->subframe_size > 0) ? (unsigned)ctx->subframe_size : 4;
        skip = (int)((uintptr_t)iso_data % align);
        if (skip >= iso_data_len)
            return 0;
        iso_data += skip;
        iso_data_len -= skip;
    }

    /* For UAC 1.0 isochronous IN endpoints, the payload IS the raw PCM data */
    copy_len = iso_data_len;
    if (copy_len > pcm_out_size)
        copy_len = pcm_out_size;
    memcpy(pcm_out, iso_data, copy_len);
    return copy_len;
}

/* ---------------------------------------------------------------------------
 * ipmx_uac_select_rate
 * --------------------------------------------------------------------------- */

int ipmx_uac_select_rate(IpmxUacContext *ctx, uint32_t preferred_rate)
{
    uint32_t best;
    uint64_t best_diff;
    int i;

    if (!ctx || ctx->num_sample_rates == 0)
        return -1;

    /* Exact match */
    for (i = 0; i < ctx->num_sample_rates; i++) {
        if (ctx->sample_rates[i] == preferred_rate) {
            ctx->active_sample_rate = preferred_rate;
            return 0;
        }
    }

    /* Closest available */
    best = ctx->sample_rates[0];
    best_diff = (uint64_t)(preferred_rate > best ?
        preferred_rate - best : best - preferred_rate);

    for (i = 1; i < ctx->num_sample_rates; i++) {
        uint32_t rate = ctx->sample_rates[i];
        uint64_t diff = (uint64_t)(preferred_rate > rate ?
            preferred_rate - rate : rate - preferred_rate);
        if (diff < best_diff) {
            best_diff = diff;
            best = rate;
        }
    }
    ctx->active_sample_rate = best;
    return 0;
}

/* ---------------------------------------------------------------------------
 * ipmx_uac_bytes_per_ms
 * --------------------------------------------------------------------------- */

int ipmx_uac_bytes_per_ms(IpmxUacContext *ctx)
{
    if (!ctx || ctx->active_sample_rate == 0 || ctx->channels == 0 || ctx->subframe_size == 0)
        return 0;
    return (int)((uint64_t)ctx->active_sample_rate * ctx->channels * ctx->subframe_size / 1000);
}

/* ---------------------------------------------------------------------------
 * UAC 2.0 Clock Source control transfer helpers
 * --------------------------------------------------------------------------- */

int ipmx_uac2_build_clock_get_range_req(uint8_t *buf, uint8_t clock_id, uint8_t ac_interface)
{
    if (!buf)
        return -1;
    buf[0] = 0xA1;  /* Device-to-host | Class | Interface */
    buf[1] = UAC2_RANGE;
    buf[2] = 0x00;
    buf[3] = UAC2_CS_SAM_FREQ_CONTROL;
    buf[4] = (uint8_t)((clock_id << 0) & 0xFF);
    buf[5] = ac_interface;
    /* wIndex per UAC 2.0 5.2.2: low byte = entity ID, high byte = interface */
    /* Actually the spec says wIndex = (Interface << 8) | Entity for CUR/RANGE
     * but the convention used by real devices: low = entityID, high = interface */
    buf[4] = clock_id;
    buf[5] = ac_interface;
    buf[6] = 0xFF;  /* wLength = 255 (max, device returns actual) */
    buf[7] = 0x00;
    return 8;
}

int ipmx_uac2_build_clock_set_cur_req(uint8_t *buf, uint8_t clock_id, uint8_t ac_interface)
{
    if (!buf)
        return -1;
    buf[0] = 0x21;  /* Host-to-device | Class | Interface */
    buf[1] = UAC2_CUR;
    buf[2] = 0x00;
    buf[3] = UAC2_CS_SAM_FREQ_CONTROL;
    buf[4] = clock_id;
    buf[5] = ac_interface;
    buf[6] = 0x04;  /* wLength = 4 (dSAMFREQ is 4 bytes LE in UAC 2.0) */
    buf[7] = 0x00;
    return 8;
}
