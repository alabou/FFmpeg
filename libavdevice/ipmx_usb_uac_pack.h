/*
 * IPMX USB UAC audio packer for Sender device.
 *
 * Inverse of ipmx_uac_extract_audio(): takes PCM audio samples and packs
 * them into UAC isochronous payloads of ~1ms each.
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

#ifndef AVDEVICE_IPMX_USB_UAC_PACK_H
#define AVDEVICE_IPMX_USB_UAC_PACK_H

#include <stdint.h>
#include "ipmx_usb_proto.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IPMX_UAC_PACK_MAX_PACKETS 256

typedef struct IpmxUacPacker {
    uint16_t max_packet_size;
    uint32_t sample_rate;
    uint8_t  channels;
    uint8_t  subframe_size;   /* bytes per sample per channel */
    int      bytes_per_ms;    /* sample_rate * channels * subframe_size / 1000 */
} IpmxUacPacker;

void ipmx_uac_pack_init(IpmxUacPacker *pack, uint16_t max_packet_size,
                          uint32_t sample_rate, uint8_t channels,
                          uint8_t subframe_size);

/**
 * Pack PCM data into N isochronous packets (1ms chunks).
 *
 * @param pcm_data   Input PCM buffer
 * @param pcm_len    Length in bytes
 * @param out_buf    Output buffer for concatenated ISO payloads
 * @param out_buf_size  Output buffer capacity
 * @param iso_descs  Array of ISO packet descriptors to fill
 * @param max_packets  Maximum number of packets to generate
 * @param num_packets  Output: actual number of packets generated
 * @param total_transfer_len  Output: total bytes written to out_buf
 * @return 0 on success, negative on error
 */
int ipmx_uac_pack_packets(IpmxUacPacker *pack,
                            const uint8_t *pcm_data, int pcm_len,
                            uint8_t *out_buf, int out_buf_size,
                            IpmxUsbIsoPacket *iso_descs, int max_packets,
                            int *num_packets, int *total_transfer_len);

#ifdef __cplusplus
}
#endif

#endif /* AVDEVICE_IPMX_USB_UAC_PACK_H */
