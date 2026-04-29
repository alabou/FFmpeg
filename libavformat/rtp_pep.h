/*
 * RTP Privacy Encryption Protocol (PEP) cipher callback
 * VSF TR-10-13 (2026-02-17) Section 20 — RTP transport adaptation
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

#ifndef AVFORMAT_RTP_PEP_H
#define AVFORMAT_RTP_PEP_H

#include "libavutil/ipmx_pep.h"
#include <stdint.h>

typedef struct AVFormatContext AVFormatContext;

/**
 * RTP PEP context — stored in AVCipherCB.opaque.
 *
 * Holds the derived key, IV, and state needed by the cipher callback.
 * For RTP_KV, also holds PSK + key_generator for on-the-fly key re-derivation.
 */
typedef struct RtpPepContext {
    uint8_t key[32];           /**< derived privacy_key (16 or 32 bytes) */
    int key_len;               /**< 16 or 32 */
    enum IpmxPepMode mode;     /**< encryption mode (CTR-only in this version) */
    uint64_t iv_prime;         /**< effective IV = base_iv + substreamid */
    uint32_t key_version;      /**< current key_version */
    int is_rtp_kv;             /**< 1 = RTP_KV protocol (dynamic key versioning) */

    /* For RTP_KV key re-derivation */
    uint8_t psk[64];
    int psk_len;
    uint8_t key_generator[16];

    /* Dynamic key_version rotation (debug/test, only valid with *_KV protocols) */
    int kv_rotate_slices;      /**< 0 = disabled, >0 = rotate after N AES-CTR slices */
    int kv_slices_since_rotate; /**< slices encrypted since last rotation */
    int kv_rotate_pending;     /**< 1 = rotation armed, trigger at next valid boundary */
    struct AVCipherCB *owner_cb; /**< back-pointer to update .param on rotation (RTP) */

    /* MAC tag from last encrypt (CMAC modes only) — written by callback,
     * read by sender to append to the RTP packet separately. */
    uint8_t last_mac[8];
    int last_mac_len;          /**< 0 or 8 */
} RtpPepContext;

/**
 * Initialize an RtpPepContext from parameters.
 *
 * If pep_iv / pep_key_generator / pep_key_version are NULL, random values
 * are auto-generated per TR-10-13 Section 13. key_generator and key_version
 * are process-global (generated once); iv is per-stream.
 *
 * The auto-generated hex values are written back to the corresponding
 * AVFormatContext string fields so the SDP writer can emit them.
 *
 * @param ctx               Context to initialize
 * @param s                 AVFormatContext for logging and writing back auto-generated values
 * @param psk_hex           PSK hex string (required)
 * @param mode_str          Mode string (e.g. "AES-128-CTR"); NULL defaults to AES-128-CTR
 * @param iv_hex            Base IV hex string; NULL = auto-generate
 * @param key_generator_hex Key generator hex string; NULL = use process-global
 * @param key_version_hex   Key version hex string; NULL = use process-global
 * @param is_rtp_kv         1 = RTP_KV, 0 = RTP
 * @return 0 on success, negative AVERROR on failure
 */
int rtp_pep_init(RtpPepContext *ctx, AVFormatContext *s,
                 const char *psk_hex, const char *mode_str,
                 const char *iv_hex, const char *key_generator_hex,
                 const char *key_version_hex, int is_rtp_kv);

/**
 * AVCipherCB callback for PEP AES-CTR encryption/decryption.
 *
 * AES-CTR is symmetric (encrypt == decrypt), so the same callback is used
 * for both sender and receiver. Counter block: iv'(64-bit) || ctr(64-bit).
 *
 * For RTP_KV: if param differs from ctx->key_version and AVCIPHERCB_VALIDPARAM
 * is set, the key is re-derived before encrypting.
 *
 * @param opaque    RtpPepContext*
 * @param param     key_version (from RTP extension header inband_ctr field)
 * @param input_ctr 64-bit counter (from RTP extension header)
 * @param buf       Payload buffer (encrypted/decrypted in-place)
 * @param len       Payload length
 * @param flags     AVCIPHERCB_VALIDPARAM | AVCIPHERCB_VALIDINPUTCTR for Full header
 * @return 0 on success, negative on error
 */
int rtp_pep_cipher_callback(void *opaque, uint32_t param,
                            uint64_t input_ctr, uint8_t *buf,
                            size_t len, int flags);

/**
 * Free an RtpPepContext (zeroes key material).
 */
void rtp_pep_free(RtpPepContext *ctx);

/**
 * Parse an SDP "a=privacy:" line into AVFormatContext PEP fields.
 * Sets s->privacy_encrypt = 1 if parsing succeeds.
 */
int ff_rtp_pep_parse_sdp_privacy(AVFormatContext *s, const char *line);

#endif /* AVFORMAT_RTP_PEP_H */
