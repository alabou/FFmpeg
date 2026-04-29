/*
 * IPMX USB UAC audio packer for Sender device.
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

#include "ipmx_usb_uac_pack.h"
#include <string.h>

void ipmx_uac_pack_init(IpmxUacPacker *pack, uint16_t max_packet_size,
                          uint32_t sample_rate, uint8_t channels,
                          uint8_t subframe_size)
{
    memset(pack, 0, sizeof(*pack));
    pack->max_packet_size = max_packet_size;
    pack->sample_rate     = sample_rate;
    pack->channels        = channels;
    pack->subframe_size   = subframe_size ? subframe_size : 2;
    pack->bytes_per_ms    = (sample_rate * channels * pack->subframe_size) / 1000;
    if (pack->bytes_per_ms <= 0)
        pack->bytes_per_ms = 1;
}

int ipmx_uac_pack_packets(IpmxUacPacker *pack,
                            const uint8_t *pcm_data, int pcm_len,
                            uint8_t *out_buf, int out_buf_size,
                            IpmxUsbIsoPacket *iso_descs, int max_packets,
                            int *num_packets, int *total_transfer_len)
{
    int offset = 0;
    int out_pos = 0;
    int pkt_count = 0;

    *num_packets = 0;
    *total_transfer_len = 0;

    while (offset < pcm_len && pkt_count < max_packets) {
        int chunk = pack->bytes_per_ms;
        int remaining = pcm_len - offset;

        if (chunk > remaining)
            chunk = remaining;
        if (chunk > pack->max_packet_size)
            chunk = pack->max_packet_size;
        if (out_pos + chunk > out_buf_size)
            break;

        memcpy(out_buf + out_pos, pcm_data + offset, chunk);

        iso_descs[pkt_count].actuallength = (uint16_t)chunk;
        iso_descs[pkt_count].isostatus    = 0;

        out_pos += chunk;
        offset  += chunk;
        pkt_count++;
    }

    *num_packets = pkt_count;
    *total_transfer_len = out_pos;
    return 0;
}
