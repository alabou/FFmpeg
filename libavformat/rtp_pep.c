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

#include "rtp_pep.h"
#include "avformat.h"
#include "libavutil/avstring.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/log.h"
#include "libavutil/mem.h"
#include <openssl/rand.h>
#include <string.h>
#include <stdio.h>

/* ---------------------------------------------------------------------------
 * Process-global auto-generated parameters (TR-10-13 Section 13)
 *
 * key_generator and key_version are generated ONCE per FFmpeg process.
 * iv is generated per-stream (see rtp_pep_init).
 * --------------------------------------------------------------------------- */

static int       s_globals_initialized;
static uint8_t   s_key_generator[16];
static uint32_t  s_key_version;

static void ensure_globals_initialized(void)
{
    if (s_globals_initialized)
        return;

    RAND_bytes(s_key_generator, 16);
    RAND_bytes((uint8_t *)&s_key_version, 4);
    /* key_version = 0 is reserved in some implementations; ensure non-zero */
    if (s_key_version == 0)
        s_key_version = 1;

    s_globals_initialized = 1;
}

/* ---------------------------------------------------------------------------
 * Hex encode helper (for writing back auto-generated values to AVFormatContext)
 * --------------------------------------------------------------------------- */

static void hex_encode(const uint8_t *data, int len, char *out, int out_size)
{
    int i;
    for (i = 0; i < len && (i * 2 + 2) < out_size; i++)
        snprintf(out + i * 2, out_size - i * 2, "%02x", data[i]);
}

/* ---------------------------------------------------------------------------
 * rtp_pep_init
 * --------------------------------------------------------------------------- */

int rtp_pep_init(RtpPepContext *ctx, AVFormatContext *s,
                 const char *psk_hex, const char *mode_str,
                 const char *iv_hex, const char *key_generator_hex,
                 const char *key_version_hex, int is_rtp_kv)
{
    uint8_t kv_bytes[4];
    int key_bits;

    if (!ctx || !s)
        return AVERROR(EINVAL);

    memset(ctx, 0, sizeof(*ctx));

    /* Parse PSK (required) */
    if (!psk_hex || !psk_hex[0]) {
        av_log(s, AV_LOG_ERROR, "PEP: pep_psk is required\n");
        return AVERROR(EINVAL);
    }
    ctx->psk_len = av_ipmx_pep_hex_decode(psk_hex, ctx->psk, 64);
    if (ctx->psk_len != 16 && ctx->psk_len != 32 && ctx->psk_len != 64) {
        av_log(s, AV_LOG_ERROR,
               "PEP: PSK must be 16, 32, or 64 bytes (got %d)\n", ctx->psk_len);
        return AVERROR(EINVAL);
    }

    /* Parse mode — default to AES-128-CTR */
    ctx->mode = av_ipmx_pep_parse_mode_str(mode_str);

    /* Ensure process-global parameters are initialized */
    ensure_globals_initialized();

    /* Key generator: CLI override or process-global */
    if (key_generator_hex && key_generator_hex[0]) {
        if (av_ipmx_pep_hex_decode(key_generator_hex, ctx->key_generator, 16) != 16) {
            av_log(s, AV_LOG_ERROR, "PEP: key_generator must be 16 bytes\n");
            return AVERROR(EINVAL);
        }
    } else {
        memcpy(ctx->key_generator, s_key_generator, 16);
        /* Write back to AVFormatContext for SDP emission */
        if (!s->pep_key_generator) {
            char buf[33];
            hex_encode(s_key_generator, 16, buf, sizeof(buf));
            s->pep_key_generator = av_strdup(buf);
        }
    }

    /* Key version: CLI override or process-global */
    if (key_version_hex && key_version_hex[0]) {
        ctx->key_version = (uint32_t)strtoul(key_version_hex, NULL, 16);
    } else {
        ctx->key_version = s_key_version;
        /* Write back for SDP */
        if (!s->pep_key_version) {
            char buf[9];
            snprintf(buf, sizeof(buf), "%08x", s_key_version);
            s->pep_key_version = av_strdup(buf);
        }
    }

    /* IV: CLI override or random per-stream */
    if (iv_hex && iv_hex[0]) {
        ctx->iv_prime = strtoull(iv_hex, NULL, 16);
    } else {
        uint8_t iv_bytes[8];
        RAND_bytes(iv_bytes, 8);
        ctx->iv_prime = AV_RB64(iv_bytes);
        /* Write back for SDP */
        if (!s->pep_iv) {
            char buf[17];
            snprintf(buf, sizeof(buf), "%016"PRIx64, ctx->iv_prime);
            s->pep_iv = av_strdup(buf);
        }
    }

    ctx->is_rtp_kv = is_rtp_kv;

    /* Write back mode string for SDP emission */
    if (!s->pep_mode) {
        const char *mode_names[] = {
            "AES-128-CTR", "AES-256-CTR",
            "AES-128-CTR_CMAC-64", "AES-256-CTR_CMAC-64",
            "AES-128-CTR_CMAC-64-AAD", "AES-256-CTR_CMAC-64-AAD"
        };
        s->pep_mode = av_strdup(mode_names[ctx->mode]);
    }

    /* Derive initial key */
    key_bits = av_ipmx_pep_mode_key_bits(ctx->mode);
    ctx->key_len = key_bits / 8;
    AV_WB32(kv_bytes, ctx->key_version);
    if (av_ipmx_pep_derive_key(ctx->psk, ctx->psk_len,
                                ctx->key_generator, kv_bytes,
                                ctx->key, key_bits) < 0) {
        av_log(s, AV_LOG_ERROR, "PEP: key derivation failed\n");
        return AVERROR(EIO);
    }

    av_log(s, AV_LOG_INFO,
           "PEP: initialized mode=%s iv=%016"PRIx64" key_version=%08x rtp_kv=%d\n",
           mode_str ? mode_str : "AES-128-CTR",
           ctx->iv_prime, ctx->key_version, ctx->is_rtp_kv);

    return 0;
}

/* ---------------------------------------------------------------------------
 * rtp_pep_cipher_callback
 * --------------------------------------------------------------------------- */

int rtp_pep_cipher_callback(void *opaque, uint32_t param,
                            uint64_t input_ctr, uint8_t *buf,
                            size_t len, int flags)
{
    RtpPepContext *ctx = (RtpPepContext *)opaque;

    if (!ctx || !buf || len == 0)
        return -1;

    /* RTP_KV sender-side: rotate key_version after N AES slices, at the next
     * valid boundary (Full Extension Header = frame/field/packet boundary).
     * Per TR-10-13 Section 20.3: changes at natural boundaries only. */
    if (ctx->kv_rotate_pending && (flags & AVCIPHERCB_VALIDPARAM)) {
        uint8_t kv_bytes[4];
        int key_bits = av_ipmx_pep_mode_key_bits(ctx->mode);

        ctx->key_version = (ctx->key_version + 1) & 0xFFFFFFFF;
        AV_WB32(kv_bytes, ctx->key_version);
        if (av_ipmx_pep_derive_key(ctx->psk, ctx->psk_len,
                                    ctx->key_generator, kv_bytes,
                                    ctx->key, key_bits) < 0)
            return -1;
        if (ctx->owner_cb)
            ctx->owner_cb->param = ctx->key_version;
        ctx->kv_rotate_pending = 0;
        ctx->kv_slices_since_rotate = 0;
        av_log(NULL, AV_LOG_INFO, "PEP: key_version rotated to %08x\n",
               ctx->key_version);
    }

    /* RTP_KV receiver-side: re-derive key if incoming dynamic_key_version changed */
    if (ctx->is_rtp_kv && (flags & AVCIPHERCB_VALIDPARAM) && param != ctx->key_version
        && ctx->kv_rotate_slices == 0) {
        uint8_t kv_bytes[4];
        int key_bits = av_ipmx_pep_mode_key_bits(ctx->mode);

        AV_WB32(kv_bytes, param);
        if (av_ipmx_pep_derive_key(ctx->psk, ctx->psk_len,
                                    ctx->key_generator, kv_bytes,
                                    ctx->key, key_bits) < 0)
            return -1;
        ctx->key_version = param;
        av_log(NULL, AV_LOG_INFO, "PEP: key_version re-derived for %08x\n", param);
    }

    /* --- Encrypt or decrypt with optional CMAC --- */
    if (av_ipmx_pep_mode_has_cmac(ctx->mode)) {
        /* Build AAD (TR-10-13 Section 20) */
        uint8_t aad[16];
        int aad_len = 0;
        int has_aad = av_ipmx_pep_mode_has_aad(ctx->mode);

        if (has_aad) {
            memset(aad, 0, sizeof(aad));
            if (flags & AVCIPHERCB_VALIDINPUTCTR) {
                /* Full header AAD: 00000000 || dkv || ctr_high || ctr_low */
                AV_WB32(aad + 4, ctx->is_rtp_kv ? param : 0);
                AV_WB32(aad + 8, (uint32_t)(input_ctr >> 32));
                AV_WB32(aad + 12, (uint32_t)(input_ctr & 0xFFFFFFFF));
            } else {
                /* Short header AAD: 00..00 || ctr_short(3 bytes) */
                aad[13] = (input_ctr >> 16) & 0xFF;
                aad[14] = (input_ctr >> 8) & 0xFF;
                aad[15] = input_ctr & 0xFF;
            }
            aad_len = 16;
        }

        if (flags & AVCIPHERCB_ENCRYPT) {
            /* mac-then-encrypt: CMAC over plaintext, append tag, encrypt all.
             * Caller MUST ensure buf has 8 bytes of headroom past len. */
            uint8_t tag[AV_IPMX_PEP_CMAC_TAG_SIZE];
            if (av_ipmx_pep_cmac64(ctx->key, ctx->key_len,
                                    buf, (int)len,
                                    has_aad ? aad : NULL, aad_len,
                                    tag) < 0)
                return -1;
            memcpy(buf + len, tag, AV_IPMX_PEP_CMAC_TAG_SIZE);
            /* Encrypt data + MAC together in one pass */
            if (av_ipmx_pep_aes_ctr(ctx->key, ctx->key_len, ctx->iv_prime,
                                     input_ctr, buf, buf,
                                     (int)len + AV_IPMX_PEP_CMAC_TAG_SIZE) < 0)
                return -1;
        } else {
            /* decrypt-then-verify: decrypt all, extract MAC, verify */
            if (av_ipmx_pep_aes_ctr(ctx->key, ctx->key_len, ctx->iv_prime,
                                     input_ctr, buf, buf, (int)len) < 0)
                return -1;
            {
                int data_len = (int)len - AV_IPMX_PEP_CMAC_TAG_SIZE;
                uint8_t expected[AV_IPMX_PEP_CMAC_TAG_SIZE];
                if (data_len < 0)
                    return AVERROR(EILSEQ);
                if (av_ipmx_pep_cmac64(ctx->key, ctx->key_len,
                                        buf, data_len,
                                        has_aad ? aad : NULL, aad_len,
                                        expected) < 0)
                    return -1;
                if (memcmp(buf + data_len, expected, AV_IPMX_PEP_CMAC_TAG_SIZE) != 0)
                    return AVERROR(EILSEQ); /* MAC verification failed */
            }
        }
    } else {
        /* CTR-only: symmetric encrypt/decrypt in-place */
        if (av_ipmx_pep_aes_ctr(ctx->key, ctx->key_len, ctx->iv_prime,
                                  input_ctr, buf, buf, (int)len) < 0)
            return -1;
    }

    /* After encrypt/decrypt: count slices and arm KV rotation */
    if (ctx->kv_rotate_slices > 0) {
        int enc_len = (int)len;
        if (av_ipmx_pep_mode_has_cmac(ctx->mode) && (flags & AVCIPHERCB_ENCRYPT))
            enc_len += AV_IPMX_PEP_CMAC_TAG_SIZE;
        ctx->kv_slices_since_rotate += av_ipmx_pep_ctr_advance(enc_len);
        if (ctx->kv_slices_since_rotate >= ctx->kv_rotate_slices)
            ctx->kv_rotate_pending = 1;
    }

    return 0;
}

/* ---------------------------------------------------------------------------
 * rtp_pep_free
 * --------------------------------------------------------------------------- */

void rtp_pep_free(RtpPepContext *ctx)
{
    if (ctx) {
        memset(ctx->key, 0, sizeof(ctx->key));
        memset(ctx->psk, 0, sizeof(ctx->psk));
    }
}

/* ---------------------------------------------------------------------------
 * SDP "a=privacy:" parser
 * --------------------------------------------------------------------------- */

int ff_rtp_pep_parse_sdp_privacy(AVFormatContext *s, const char *line)
{
    char key[64], val[256];
    const char *p = line;

    if (!s || !line)
        return AVERROR(EINVAL);

    /* Parse semicolon-delimited key=value pairs:
     * a=privacy:protocol=RTP;mode=AES-128-CTR;iv=...;key_generator=...;key_version=...
     */
    while (*p) {
        char *eq;
        int klen;
        const char *semi = strchr(p, ';');
        int seg_len = semi ? (int)(semi - p) : (int)strlen(p);

        if (seg_len >= (int)sizeof(val))
            seg_len = sizeof(val) - 1;

        memcpy(val, p, seg_len);
        val[seg_len] = '\0';

        eq = strchr(val, '=');
        if (eq) {
            klen = (int)(eq - val);
            if (klen >= (int)sizeof(key))
                klen = sizeof(key) - 1;
            memcpy(key, val, klen);
            key[klen] = '\0';
            /* trim leading whitespace from key */
            {
                char *k = key;
                while (*k == ' ') k++;
                if (k != key) memmove(key, k, strlen(k) + 1);
            }
            av_strlcpy(val, eq + 1, sizeof(val));

            if (!strcmp(key, "protocol")) {
                if (strstr(val, "RTP_KV"))
                    s->pep_rtp_kv = 1;
                /* "RTP" is default */
            } else if (!strcmp(key, "mode")) {
                av_freep(&s->pep_mode);
                s->pep_mode = av_strdup(val);
            } else if (!strcmp(key, "iv")) {
                av_freep(&s->pep_iv);
                s->pep_iv = av_strdup(val);
            } else if (!strcmp(key, "key_generator")) {
                av_freep(&s->pep_key_generator);
                s->pep_key_generator = av_strdup(val);
            } else if (!strcmp(key, "key_version")) {
                av_freep(&s->pep_key_version);
                s->pep_key_version = av_strdup(val);
            }
        }

        if (!semi)
            break;
        p = semi + 1;
    }

    s->privacy_encrypt = 1;
    return 0;
}
