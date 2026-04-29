/*
 * IPMX USB Privacy Encryption Protocol (PEP)
 * VSF TR-10-13 (2026-02-17), VSF TR-10-14 (2024-09-24) Section 12
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

#include "ipmx_usb_pep.h"
#include "libavutil/mem.h"
#include "libavutil/intreadwrite.h"
#include <string.h>

/*
 * Core crypto (KDF, AES-CTR, CMAC-64, IV compute) is now provided by
 * libavutil/ipmx_pep.c via the #define aliases in ipmx_usb_pep.h.
 * This file retains only USB-envelope-specific functions.
 */

/* Big-endian helper for reading CTR from USB message header */
static uint64_t get_be64(const uint8_t *p)
{
    return AV_RB64(p);
}

int ipmx_pep_usb_ctr_advance(int msg_length)
{
    int encrypted_len = msg_length - IPMX_PEP_USB_HEADER_SIZE;
    if (encrypted_len <= 0)
        return 0;
    return (encrypted_len + IPMX_PEP_AES_BLOCK_SIZE - 1) / IPMX_PEP_AES_BLOCK_SIZE;
}

/* ---------------------------------------------------------------------------
 * USB message encrypt (mac-then-encrypt)
 * --------------------------------------------------------------------------- */

int ipmx_pep_encrypt_usb_msg(IpmxPepContext *ctx, uint64_t iv_prime,
                             uint8_t *msg, int msg_len)
{
    const uint8_t *header;
    uint8_t *plain_data;
    int plain_len;
    uint64_t ctr;
    uint8_t tag_buf[IPMX_PEP_CMAC_TAG_SIZE];

    if (!ctx || !msg)
        return -1;
    if (msg_len < IPMX_PEP_USB_HEADER_SIZE + IPMX_PEP_USB_MAC_SIZE)
        return -1;

    header = msg;
    plain_data = msg + IPMX_PEP_USB_HEADER_SIZE;
    plain_len = msg_len - IPMX_PEP_USB_HEADER_SIZE - IPMX_PEP_USB_MAC_SIZE;
    ctr = get_be64(header);

    /* Compute CMAC-64 tag */
    if (av_ipmx_pep_mode_has_cmac(ctx->mode)) {
        if (av_ipmx_pep_mode_has_aad(ctx->mode)) {
            if (ipmx_pep_cmac64(ctx->key, ctx->key_len,
                               plain_data, plain_len,
                               header, IPMX_PEP_USB_HEADER_SIZE,
                               tag_buf) < 0)
                return -1;
        } else {
            /* Non-AAD: CMAC over header || plain_data as contiguous data */
            if (ipmx_pep_cmac64(ctx->key, ctx->key_len,
                               msg, IPMX_PEP_USB_HEADER_SIZE + plain_len,
                               NULL, 0,
                               tag_buf) < 0)
                return -1;
        }
        memcpy(msg + msg_len - IPMX_PEP_USB_MAC_SIZE, tag_buf,
               IPMX_PEP_USB_MAC_SIZE);
    } else {
        memset(msg + msg_len - IPMX_PEP_USB_MAC_SIZE, 0, IPMX_PEP_USB_MAC_SIZE);
    }

    /* AES-CTR encrypt msg[16:msg_len] (data + MAC) */
    return ipmx_pep_aes_ctr(ctx->key, ctx->key_len, iv_prime, ctr,
                            msg + IPMX_PEP_USB_HEADER_SIZE,
                            msg + IPMX_PEP_USB_HEADER_SIZE,
                            msg_len - IPMX_PEP_USB_HEADER_SIZE);
}

/* ---------------------------------------------------------------------------
 * USB message decrypt (decrypt-then-verify)
 * --------------------------------------------------------------------------- */

int ipmx_pep_decrypt_usb_msg(IpmxPepContext *ctx, uint64_t iv_prime,
                             uint8_t *msg, int msg_len, int *mac_ok)
{
    const uint8_t *header;
    uint8_t *encrypted;
    int encrypted_len;
    uint64_t ctr;
    uint8_t *plain_data;
    uint8_t recovered_mac[IPMX_PEP_CMAC_TAG_SIZE];
    uint8_t expected_mac[IPMX_PEP_CMAC_TAG_SIZE];
    int mac_matches = 1;

    if (!ctx || !msg || !mac_ok)
        return -1;
    if (msg_len < IPMX_PEP_USB_HEADER_SIZE + IPMX_PEP_USB_MAC_SIZE)
        return -1;

    header = msg;
    encrypted = msg + IPMX_PEP_USB_HEADER_SIZE;
    encrypted_len = msg_len - IPMX_PEP_USB_HEADER_SIZE;
    ctr = get_be64(header);

    /* AES-CTR decrypt */
    if (ipmx_pep_aes_ctr(ctx->key, ctx->key_len, iv_prime, ctr,
                         encrypted, encrypted, encrypted_len) < 0)
        return -1;

    plain_data = msg + IPMX_PEP_USB_HEADER_SIZE;
    memcpy(recovered_mac, msg + msg_len - IPMX_PEP_USB_MAC_SIZE,
           IPMX_PEP_USB_MAC_SIZE);

    /* Verify CMAC */
    if (av_ipmx_pep_mode_has_cmac(ctx->mode)) {
        int plain_len = msg_len - IPMX_PEP_USB_HEADER_SIZE - IPMX_PEP_USB_MAC_SIZE;
        if (av_ipmx_pep_mode_has_aad(ctx->mode)) {
            if (ipmx_pep_cmac64(ctx->key, ctx->key_len,
                               plain_data, plain_len,
                               header, IPMX_PEP_USB_HEADER_SIZE,
                               expected_mac) < 0)
                return -1;
        } else {
            /* Non-AAD: CMAC over header || plain_data as contiguous data */
            if (ipmx_pep_cmac64(ctx->key, ctx->key_len,
                               msg, IPMX_PEP_USB_HEADER_SIZE + plain_len,
                               NULL, 0,
                               expected_mac) < 0)
                return -1;
        }
        mac_matches = (memcmp(recovered_mac, expected_mac,
                             IPMX_PEP_CMAC_TAG_SIZE) == 0);
    }

    *mac_ok = mac_matches;

    /* Zero out MAC bytes in decrypted message */
    memset(msg + msg_len - IPMX_PEP_USB_MAC_SIZE, 0, IPMX_PEP_USB_MAC_SIZE);

    return 0;
}
