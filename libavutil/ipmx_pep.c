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

#include "ipmx_pep.h"
#include "intreadwrite.h"
#include <openssl/evp.h>
#include <openssl/cmac.h>
#include <openssl/hmac.h>
#include <string.h>

/* ---------------------------------------------------------------------------
 * Mode helpers
 * --------------------------------------------------------------------------- */

int av_ipmx_pep_mode_key_bits(enum IpmxPepMode mode)
{
    return (mode == IPMX_PEP_AES_128_CTR ||
            mode == IPMX_PEP_AES_128_CTR_CMAC_64 ||
            mode == IPMX_PEP_AES_128_CTR_CMAC_64_AAD) ? 128 : 256;
}

int av_ipmx_pep_mode_has_cmac(enum IpmxPepMode mode)
{
    return mode >= IPMX_PEP_AES_128_CTR_CMAC_64;
}

int av_ipmx_pep_mode_has_aad(enum IpmxPepMode mode)
{
    return mode == IPMX_PEP_AES_128_CTR_CMAC_64_AAD ||
           mode == IPMX_PEP_AES_256_CTR_CMAC_64_AAD;
}

/* ---------------------------------------------------------------------------
 * Key Derivation (NIST SP 800-108 CMAC-based KDF, TR-10-13 Section 12)
 * --------------------------------------------------------------------------- */

#define KDF_PREFIX_AB 0xAB
#define KDF_PREFIX_CD 0xCD

/**
 * One round of CMAC-based KDF.
 * msg = prefix(1) || key_generator(16) || key_version(4) || key_pfs
 */
static int kdf_cmac_round(const uint8_t *psk, int psk_len,
                           uint8_t prefix,
                           const uint8_t *key_generator,
                           const uint8_t *key_version_bytes,
                           const uint8_t *key_pfs, int key_pfs_len,
                           uint8_t *out, int out_len)
{
    CMAC_CTX *ctx = NULL;
    const EVP_CIPHER *cipher;
    uint8_t msg[1 + 16 + 4 + 32]; /* prefix + kg + kv + max pfs */
    int msg_len;
    size_t mac_len;
    int ret = -1;

    if (psk_len != 16 && psk_len != 32)
        return -1;
    if (out_len != 16 && out_len != 32)
        return -1;

    cipher = (psk_len == 16) ? EVP_aes_128_cbc() : EVP_aes_256_cbc();

    msg[0] = prefix;
    memcpy(msg + 1, key_generator, 16);
    memcpy(msg + 17, key_version_bytes, 4);
    msg_len = 1 + 16 + 4 + key_pfs_len;
    if (key_pfs_len > 0)
        memcpy(msg + 21, key_pfs, key_pfs_len);

    ctx = CMAC_CTX_new();
    if (!ctx)
        goto fail;

    if (!CMAC_Init(ctx, psk, (size_t)psk_len, cipher, NULL))
        goto fail;
    if (!CMAC_Update(ctx, msg, (size_t)msg_len))
        goto fail;
    if (!CMAC_Final(ctx, out, &mac_len))
        goto fail;

    if (mac_len < (size_t)out_len)
        goto fail;
    ret = 0;

fail:
    CMAC_CTX_free(ctx);
    return ret;
}

/**
 * HMAC-SHA-512/256 based KDF for 512-bit PSK (NIST SP 800-108, TR-10-13).
 * msg = prefix(1) || key_generator(16) || key_version(4)
 * Output: 32 bytes (256-bit key).
 */
static int kdf_hmac_sha512_256(const uint8_t *psk,
                                uint8_t prefix,
                                const uint8_t *key_generator,
                                const uint8_t *key_version_bytes,
                                uint8_t *out)
{
    uint8_t msg[1 + 16 + 4];
    unsigned int out_len = 0;
    uint8_t *result;

    msg[0] = prefix;
    memcpy(msg + 1, key_generator, 16);
    memcpy(msg + 17, key_version_bytes, 4);

    result = HMAC(EVP_sha512_256(), psk, 64, msg, sizeof(msg), out, &out_len);
    if (!result || out_len < 32)
        return -1;
    return 0;
}

int av_ipmx_pep_derive_key(const uint8_t *psk, int psk_len,
                            const uint8_t *key_generator,
                            const uint8_t *key_version_bytes,
                            uint8_t *out_key, int key_bits)
{
    if (!psk || !key_generator || !key_version_bytes || !out_key)
        return -1;
    if (psk_len != 16 && psk_len != 32 && psk_len != 64)
        return -1;
    if (key_bits != 128 && key_bits != 256)
        return -1;

    if (key_bits == 128) {
        if (psk_len != 16)
            return -1;
        return kdf_cmac_round(psk, psk_len, KDF_PREFIX_AB,
                              key_generator, key_version_bytes,
                              NULL, 0, out_key, 16);
    }

    /* 256-bit key with 512-bit PSK: HMAC-SHA-512/256 */
    if (psk_len == 64)
        return kdf_hmac_sha512_256(psk, KDF_PREFIX_AB,
                                    key_generator, key_version_bytes,
                                    out_key);

    /* 256-bit key with 128 or 256-bit PSK: two CMAC rounds (0xAB, 0xCD) */
    if (kdf_cmac_round(psk, psk_len, KDF_PREFIX_AB,
                       key_generator, key_version_bytes,
                       NULL, 0, out_key, 16) < 0)
        return -1;
    if (kdf_cmac_round(psk, psk_len, KDF_PREFIX_CD,
                       key_generator, key_version_bytes,
                       NULL, 0, out_key + 16, 16) < 0)
        return -1;
    return 0;
}

/* ---------------------------------------------------------------------------
 * IV and CTR
 * --------------------------------------------------------------------------- */

uint64_t av_ipmx_pep_compute_iv_prime(uint64_t base_iv, uint16_t substreamid)
{
    return (base_iv + substreamid) & 0xFFFFFFFFFFFFFFFFULL;
}

int av_ipmx_pep_ctr_advance(int payload_len)
{
    if (payload_len <= 0)
        return 0;
    return (payload_len + AV_IPMX_PEP_AES_BLOCK_SIZE - 1) / AV_IPMX_PEP_AES_BLOCK_SIZE;
}

/* ---------------------------------------------------------------------------
 * Low-level AES-CTR
 * --------------------------------------------------------------------------- */

/* Per-thread cache for OpenSSL cipher contexts.
 *
 * Allocating + initialising a fresh EVP_CIPHER_CTX per RTP packet is a
 * substantial system-time cost — measured at ~1.7 s extra system time per
 * 3 s of 110 Mbps raw video (130%-> 200%+ wall-clock slowdown) before this
 * change. AES-CTR itself is essentially free with AES-NI; the bottleneck
 * is OpenSSL's EVP_CIPHER_CTX_new()/free() pair (malloc + internal locks).
 *
 * We keep one EVP_CIPHER_CTX per thread per cipher (AES-128-CTR vs
 * AES-256-CTR) and re-bind only the IV via EVP_EncryptInit_ex(ctx, NULL,
 * NULL, NULL, iv). The key is also rebound when the caller's key bytes
 * change (rare — only on KV rotation). Threads that never touch PEP pay
 * nothing; threads that do amortise the setup cost across all calls.
 */
typedef struct ipmx_aes_ctr_cache {
    EVP_CIPHER_CTX *ctx;
    int             cipher_key_len;     /* 0 = unbound, 16 or 32 */
    uint8_t         cipher_key[32];     /* last key bound to ctx */
} ipmx_aes_ctr_cache_t;

static __thread ipmx_aes_ctr_cache_t s_aes_ctr_cache;

int av_ipmx_pep_aes_ctr(const uint8_t *key, int key_len,
                         uint64_t iv_prime, uint64_t ctr,
                         const uint8_t *in, uint8_t *out, int len)
{
    ipmx_aes_ctr_cache_t *cache = &s_aes_ctr_cache;
    const EVP_CIPHER *cipher;
    uint8_t iv_ctr[16];
    int outl;
    int rebind_key;

    if (!key || !in || !out || len < 0)
        return -1;
    if (key_len != 16 && key_len != 32)
        return -1;

    cipher = (key_len == 16) ? EVP_aes_128_ctr() : EVP_aes_256_ctr();
    AV_WB64(iv_ctr, iv_prime);
    AV_WB64(iv_ctr + 8, ctr);

    if (!cache->ctx) {
        cache->ctx = EVP_CIPHER_CTX_new();
        if (!cache->ctx) return -1;
        cache->cipher_key_len = 0;
        /* Per-thread leak at thread exit is harmless: process termination
         * reclaims it. The cache is essentially zero-cost amortised. */
    }

    rebind_key = (cache->cipher_key_len != key_len) ||
                  memcmp(cache->cipher_key, key, key_len) != 0;

    if (rebind_key) {
        /* Bind cipher + key + IV. */
        if (!EVP_EncryptInit_ex(cache->ctx, cipher, NULL, key, iv_ctr))
            return -1;
        cache->cipher_key_len = key_len;
        memcpy(cache->cipher_key, key, key_len);
    } else {
        /* Hot path: same key as last call — only re-init the IV. */
        if (!EVP_EncryptInit_ex(cache->ctx, NULL, NULL, NULL, iv_ctr))
            return -1;
    }

    if (!EVP_EncryptUpdate(cache->ctx, out, &outl, in, len))
        return -1;
    if (outl != len)
        return -1;
    if (!EVP_EncryptFinal_ex(cache->ctx, out + outl, &outl))
        return -1;
    if (outl != 0)
        return -1;

    return 0;
}

/* ---------------------------------------------------------------------------
 * CMAC-64
 * --------------------------------------------------------------------------- */

/* Per-thread CMAC context cache — same rationale as the AES-CTR cache:
 * CMAC_CTX_new()/free() per packet adds ~µs each that becomes minutes/hour
 * at high packet rates. Re-key via CMAC_Init when the caller's key changes. */
typedef struct ipmx_cmac_cache {
    CMAC_CTX *ctx;
    int       key_len;
    uint8_t   key[32];
} ipmx_cmac_cache_t;

static __thread ipmx_cmac_cache_t s_cmac_cache;

int av_ipmx_pep_cmac64(const uint8_t *key, int key_len,
                        const uint8_t *data, int data_len,
                        const uint8_t *aad, int aad_len,
                        uint8_t *tag)
{
    ipmx_cmac_cache_t *cache = &s_cmac_cache;
    const EVP_CIPHER *cipher;
    uint8_t full_mac[AV_IPMX_PEP_AES_BLOCK_SIZE];
    size_t mac_len;
    int rebind_key;

    if (!key || !tag)
        return -1;
    if (key_len != 16 && key_len != 32)
        return -1;
    if ((data_len < 0) || (aad_len < 0))
        return -1;
    if (data_len > 0 && !data)
        return -1;
    if (aad_len > 0 && !aad)
        return -1;

    cipher = (key_len == 16) ? EVP_aes_128_cbc() : EVP_aes_256_cbc();

    if (!cache->ctx) {
        cache->ctx = CMAC_CTX_new();
        if (!cache->ctx) return -1;
        cache->key_len = 0;
    }

    rebind_key = (cache->key_len != key_len) ||
                  memcmp(cache->key, key, key_len) != 0;

    if (rebind_key) {
        if (!CMAC_Init(cache->ctx, key, (size_t)key_len, cipher, NULL))
            return -1;
        cache->key_len = key_len;
        memcpy(cache->key, key, key_len);
    } else {
        /* CMAC_Init with NULL key/cipher resets state for the next message
         * while reusing the existing key+cipher binding. */
        if (!CMAC_Init(cache->ctx, NULL, 0, NULL, NULL))
            return -1;
    }

    if (aad_len > 0 && !CMAC_Update(cache->ctx, aad, (size_t)aad_len))
        return -1;
    if (data_len > 0 && !CMAC_Update(cache->ctx, data, (size_t)data_len))
        return -1;
    if (!CMAC_Final(cache->ctx, full_mac, &mac_len))
        return -1;

    if (mac_len < AV_IPMX_PEP_CMAC_TAG_SIZE)
        return -1;

    /* Truncate to CMAC-64: first 8 bytes */
    memcpy(tag, full_mac, AV_IPMX_PEP_CMAC_TAG_SIZE);
    return 0;
}

/* ---------------------------------------------------------------------------
 * Utility
 * --------------------------------------------------------------------------- */

int av_ipmx_pep_hex_decode(const char *hex, uint8_t *out, int max_len)
{
    int len = 0;

    if (!hex || !out)
        return -1;

    while (*hex && len < max_len) {
        char pair[3];
        unsigned int v;

        pair[0] = *hex++;
        pair[1] = *hex ? *hex++ : '0';
        pair[2] = '\0';
        if (sscanf(pair, "%02x", &v) != 1)
            return -1;
        out[len++] = (uint8_t)v;
    }
    return len;
}

enum IpmxPepMode av_ipmx_pep_parse_mode_str(const char *val)
{
    if (!val)
        return IPMX_PEP_AES_128_CTR;
    if (strstr(val, "AES-256") && strstr(val, "AAD"))
        return IPMX_PEP_AES_256_CTR_CMAC_64_AAD;
    if (strstr(val, "AES-256") && strstr(val, "CMAC"))
        return IPMX_PEP_AES_256_CTR_CMAC_64;
    if (strstr(val, "AES-256"))
        return IPMX_PEP_AES_256_CTR;
    if (strstr(val, "AAD"))
        return IPMX_PEP_AES_128_CTR_CMAC_64_AAD;
    if (strstr(val, "CMAC"))
        return IPMX_PEP_AES_128_CTR_CMAC_64;
    return IPMX_PEP_AES_128_CTR;
}
