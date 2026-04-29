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

#ifndef AVDEVICE_IPMX_USB_UAC_H
#define AVDEVICE_IPMX_USB_UAC_H

#include <stdint.h>
#include <stddef.h>

/* UAC class and subclass */
#define UAC_CC_AUDIO               0x01
#define UAC_SC_AUDIOCONTROL        0x01
#define UAC_SC_AUDIOSTREAMING      0x02

/* AudioControl CS_INTERFACE subtypes (UAC 1.0) */
#define UAC_AC_HEADER              0x01
#define UAC_AC_INPUT_TERMINAL      0x02
#define UAC_AC_OUTPUT_TERMINAL     0x03
#define UAC_AC_FEATURE_UNIT        0x06

/* AudioControl CS_INTERFACE subtypes (UAC 2.0 additions) */
#define UAC2_AC_CLOCK_SOURCE       0x0A
#define UAC2_AC_CLOCK_SELECTOR     0x0B

/* AudioStreaming CS_INTERFACE subtypes */
#define UAC_AS_GENERAL             0x01
#define UAC_AS_FORMAT_TYPE         0x02
#define UAC_FORMAT_TYPE_I          0x01

/* UAC 2.0 Clock Source control selectors */
#define UAC2_CS_SAM_FREQ_CONTROL   0x01

/* UAC 2.0 bRequest codes */
#define UAC2_CUR                   0x01
#define UAC2_RANGE                 0x02

/* Format tags */
#define UAC_FORMAT_PCM             0x0001
#define UAC_FORMAT_IEEE_FLOAT      0x0003

#define IPMX_UAC_MAX_SAMPLE_RATES  8

typedef struct IpmxUacContext {
    /* Parsed from descriptors */
    uint8_t  ac_interface;         /* AudioControl interface number */
    uint8_t  as_interface;         /* AudioStreaming interface number */
    uint8_t  as_endpoint;          /* Isochronous IN endpoint number */
    uint16_t max_packet_size;

    /* Audio format */
    uint16_t format_tag;           /* UAC_FORMAT_PCM etc. */
    uint8_t  channels;
    uint8_t  subframe_size;        /* bytes per sample per channel */
    uint8_t  bit_resolution;
    uint8_t  num_sample_rates;
    uint32_t sample_rates[IPMX_UAC_MAX_SAMPLE_RATES];

    /* Active state */
    uint32_t active_sample_rate;

    /* Quirks */
    uint32_t quirks;

    /* UAC version detected (1 or 2) */
    uint8_t  uac_version;

    /* UAC 2.0 Clock Source */
    uint8_t  clock_source_id;      /* Clock Source entity ID (0 if absent) */
    uint32_t clock_freq_min;       /* from GET_RANGE (0 = unknown) */
    uint32_t clock_freq_max;
} IpmxUacContext;

/**
 * Parse USB configuration descriptor chain and extract UAC 1.0 audio config.
 * Returns 0 on success, negative on error.
 */
int ipmx_uac_parse_config(IpmxUacContext *ctx, const uint8_t *desc, int desc_len);

/**
 * Build 8-byte USB SET_INTERFACE setup packet.
 * Returns 8 (bytes written).
 */
int ipmx_uac_build_set_interface_req(uint8_t *buf, uint8_t interface, uint8_t alt_setting);

/**
 * Extract PCM from UAC 1.0 isochronous IN payload.
 * Returns number of bytes copied to pcm_out.
 */
int ipmx_uac_extract_audio(IpmxUacContext *ctx, const uint8_t *iso_data, int iso_data_len,
                           uint8_t *pcm_out, int pcm_out_size);

/**
 * Select closest matching sample rate. Sets ctx->active_sample_rate.
 * Returns 0 on success, negative if no rates available.
 */
int ipmx_uac_select_rate(IpmxUacContext *ctx, uint32_t preferred_rate);

/**
 * Compute bytes per millisecond for active format.
 */
int ipmx_uac_bytes_per_ms(IpmxUacContext *ctx);

/**
 * Build 8-byte UAC 2.0 Clock Source GET_RANGE setup packet.
 * wIndex encodes (clock_id << 8) | ac_interface per UAC 2.0 Section 5.2.2.
 * Returns 8 (bytes written).
 */
int ipmx_uac2_build_clock_get_range_req(uint8_t *buf, uint8_t clock_id, uint8_t ac_interface);

/**
 * Build 8-byte UAC 2.0 Clock Source SET_CUR setup packet.
 * Data phase is 4 bytes (dSAMFREQ, little-endian).
 * Returns 8 (bytes written).
 */
int ipmx_uac2_build_clock_set_cur_req(uint8_t *buf, uint8_t clock_id, uint8_t ac_interface);

#endif /* AVDEVICE_IPMX_USB_UAC_H */
