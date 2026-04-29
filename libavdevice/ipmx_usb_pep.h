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

#ifndef AVFORMAT_IPMX_USB_PEP_H
#define AVFORMAT_IPMX_USB_PEP_H

#include <stdint.h>
#include <stddef.h>
#include "libavutil/ipmx_pep.h"

#ifdef __cplusplus
extern "C" {
#endif

/* USB-specific constants */
#define IPMX_PEP_USB_HEADER_SIZE 16
#define IPMX_PEP_USB_MAC_SIZE    8

/* Aliases: delegate core crypto names to shared libavutil module */
#define IPMX_PEP_AES_BLOCK_SIZE  AV_IPMX_PEP_AES_BLOCK_SIZE
#define IPMX_PEP_CMAC_TAG_SIZE   AV_IPMX_PEP_CMAC_TAG_SIZE

#define ipmx_pep_derive_key      av_ipmx_pep_derive_key
#define ipmx_pep_compute_iv_prime av_ipmx_pep_compute_iv_prime
#define ipmx_pep_aes_ctr         av_ipmx_pep_aes_ctr
#define ipmx_pep_cmac64          av_ipmx_pep_cmac64

typedef struct IpmxPepContext {
    uint8_t key[32];        /* privacy_key (16 or 32 bytes) */
    int key_len;            /* 16 or 32 */
    enum IpmxPepMode mode;
    uint64_t iv;            /* base IV from SDP */
    uint32_t key_version;   /* current key version */
} IpmxPepContext;

/**
 * CTR advance: number of AES blocks consumed by a USB message.
 * Subtracts the 16-byte USB header before computing ceil(len/16).
 */
int ipmx_pep_usb_ctr_advance(int msg_length);

/**
 * Encrypt a USB message (mac-then-encrypt, TR-10-14 Section 12)
 *
 * @param ctx     PEP context with key and mode
 * @param iv_prime Effective IV (base_iv + substreamid)
 * @param msg     Message buffer (header + data + MAC space), modified in-place
 * @param msg_len Total message length
 * @return 0 on success, -1 on error
 */
int ipmx_pep_encrypt_usb_msg(IpmxPepContext *ctx, uint64_t iv_prime,
                             uint8_t *msg, int msg_len);

/**
 * Decrypt a USB message
 *
 * @param ctx     PEP context with key and mode
 * @param iv_prime Effective IV (base_iv + substreamid)
 * @param msg     Message buffer (header + encrypted data + MAC), modified in-place
 * @param msg_len Total message length
 * @param mac_ok  Output: 1 if MAC verified, 0 if verification failed
 * @return 0 on success, -1 on error
 */
int ipmx_pep_decrypt_usb_msg(IpmxPepContext *ctx, uint64_t iv_prime,
                             uint8_t *msg, int msg_len, int *mac_ok);

/*
 * ipmx_pep_aes_ctr() and ipmx_pep_cmac64() are provided via #define aliases
 * to av_ipmx_pep_aes_ctr() and av_ipmx_pep_cmac64() from libavutil/ipmx_pep.h.
 * ipmx_pep_derive_key() and ipmx_pep_compute_iv_prime() likewise.
 */

#ifdef __cplusplus
}
#endif

#endif /* AVFORMAT_IPMX_USB_PEP_H */
