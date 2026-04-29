/*
 * IPMX Privacy Encryption Protocol (PEP) - Shared Crypto Module
 * VSF TR-10-13 (2026-02-17)
 *
 * Core cryptographic operations shared by USB (libavdevice) and RTP (libavformat)
 * transport adaptations.
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

#ifndef AVUTIL_IPMX_PEP_H
#define AVUTIL_IPMX_PEP_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AV_IPMX_PEP_AES_BLOCK_SIZE  16
#define AV_IPMX_PEP_CMAC_TAG_SIZE   8

/**
 * PEP encryption modes (TR-10-13 Section 13)
 */
enum IpmxPepMode {
    IPMX_PEP_AES_128_CTR = 0,
    IPMX_PEP_AES_256_CTR,
    IPMX_PEP_AES_128_CTR_CMAC_64,
    IPMX_PEP_AES_256_CTR_CMAC_64,
    IPMX_PEP_AES_128_CTR_CMAC_64_AAD,
    IPMX_PEP_AES_256_CTR_CMAC_64_AAD,
};

/* ---------------------------------------------------------------------------
 * Mode helpers
 * --------------------------------------------------------------------------- */

/**
 * Return 128 or 256 based on mode.
 */
int av_ipmx_pep_mode_key_bits(enum IpmxPepMode mode);

/**
 * Return 1 if mode includes CMAC authentication.
 */
int av_ipmx_pep_mode_has_cmac(enum IpmxPepMode mode);

/**
 * Return 1 if mode uses Additional Authenticated Data.
 */
int av_ipmx_pep_mode_has_aad(enum IpmxPepMode mode);

/* ---------------------------------------------------------------------------
 * Key Derivation (NIST SP 800-108 KDF, TR-10-13 Section 12)
 * --------------------------------------------------------------------------- */

/**
 * Derive a privacy_key from PSK, key_generator, and key_version.
 *
 * 128-bit key: AES-CMAC with 16-byte PSK.
 * 256-bit key + 16/32-byte PSK: two AES-CMAC rounds (0xAB, 0xCD).
 * 256-bit key + 64-byte PSK: HMAC-SHA-512/256.
 *
 * @param psk               Pre-Shared Key (16, 32, or 64 bytes)
 * @param psk_len           Length of PSK in bytes
 * @param key_generator     16-byte key generator from SDP
 * @param key_version_bytes 4-byte key version (big-endian)
 * @param out_key           Output buffer for derived key
 * @param key_bits          128 or 256
 * @return 0 on success, -1 on error
 */
int av_ipmx_pep_derive_key(const uint8_t *psk, int psk_len,
                            const uint8_t *key_generator,
                            const uint8_t *key_version_bytes,
                            uint8_t *out_key, int key_bits);

/* ---------------------------------------------------------------------------
 * IV and CTR
 * --------------------------------------------------------------------------- */

/**
 * Compute effective IV: iv' = (base_iv + substreamid) mod 2^64
 * (TR-10-13 Section 14)
 */
uint64_t av_ipmx_pep_compute_iv_prime(uint64_t base_iv, uint16_t substreamid);

/**
 * Number of AES blocks consumed by encrypting payload_len bytes.
 * = ceil(payload_len / 16)
 */
int av_ipmx_pep_ctr_advance(int payload_len);

/* ---------------------------------------------------------------------------
 * Low-level AES-CTR (TR-10-13 Section 15)
 * --------------------------------------------------------------------------- */

/**
 * AES-128/256-CTR encrypt or decrypt in-place or to a separate buffer.
 * Counter block: iv_prime(64-bit BE) || ctr(64-bit BE)
 *
 * @param key       16 or 32-byte key
 * @param key_len   16 or 32
 * @param iv_prime  64-bit IV (host order, stored big-endian in counter block)
 * @param ctr       64-bit counter (host order, stored big-endian in counter block)
 * @param in        Input buffer
 * @param out       Output buffer (may equal in for in-place)
 * @param len       Length in bytes
 * @return 0 on success, -1 on error
 */
int av_ipmx_pep_aes_ctr(const uint8_t *key, int key_len,
                         uint64_t iv_prime, uint64_t ctr,
                         const uint8_t *in, uint8_t *out, int len);

/* ---------------------------------------------------------------------------
 * CMAC-64 (TR-10-13 Section 15, NIST SP 800-38b)
 * --------------------------------------------------------------------------- */

/**
 * Compute 64-bit truncated CMAC.
 * CMAC input: aad || data (aad may be NULL/0 for non-AAD modes)
 *
 * @param key       16 or 32-byte key
 * @param key_len   16 or 32
 * @param data      Data to authenticate
 * @param data_len  Length of data
 * @param aad       Additional authenticated data (may be NULL if aad_len == 0)
 * @param aad_len   Length of AAD
 * @param tag       Output: 8-byte CMAC-64 tag
 * @return 0 on success, -1 on error
 */
int av_ipmx_pep_cmac64(const uint8_t *key, int key_len,
                        const uint8_t *data, int data_len,
                        const uint8_t *aad, int aad_len,
                        uint8_t *tag);

/* ---------------------------------------------------------------------------
 * Utility
 * --------------------------------------------------------------------------- */

/**
 * Decode a hex string to bytes.
 * @param hex     Hex string (e.g. "0a1b2c...")
 * @param out     Output byte buffer
 * @param max_len Maximum bytes to decode
 * @return Number of bytes decoded, or -1 on error
 */
int av_ipmx_pep_hex_decode(const char *hex, uint8_t *out, int max_len);

/**
 * Parse a PEP mode string (e.g. "AES-128-CTR", "AES-256-CTR_CMAC-64-AAD")
 * to the corresponding enum value.
 * @return IpmxPepMode value (defaults to IPMX_PEP_AES_128_CTR for unknown)
 */
enum IpmxPepMode av_ipmx_pep_parse_mode_str(const char *val);

#ifdef __cplusplus
}
#endif

#endif /* AVUTIL_IPMX_PEP_H */
