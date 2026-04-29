#include <string.h>
#include "rtp_cipher.h"

/*
 * Two dummy XOR ciphers with distinct key derivation so that enabling both
 * HDCP and privacy simultaneously does not cancel out to plaintext.
 * These are NOT cryptographically secure — they exist solely to produce
 * properly-framed encrypted RTP streams for integration testing.
 */

static void derive_hdcp_key(uint8_t key[16], uint32_t param, uint64_t input_ctr)
{
    static const uint8_t salt[4] = { 0x48, 0x44, 0x43, 0x50 }; /* "HDCP" */
    uint8_t p[4], c[8];
    int i;

    p[0] = (param >>  0) & 0xff;
    p[1] = (param >>  8) & 0xff;
    p[2] = (param >> 16) & 0xff;
    p[3] = (param >> 24) & 0xff;

    for (i = 0; i < 8; i++)
        c[i] = (input_ctr >> (i * 8)) & 0xff;

    for (i = 0; i < 4; i++)
        key[i] = p[i] ^ salt[i];
    for (i = 0; i < 4; i++)
        key[4 + i] = p[i] ^ salt[i] ^ 0xff;
    for (i = 0; i < 8; i++)
        key[8 + i] = c[i] ^ salt[i & 3];
}

static void derive_privacy_key(uint8_t key[16], uint32_t param, uint64_t input_ctr)
{
    static const uint8_t salt[4] = { 0x50, 0x45, 0x50, 0x21 }; /* "PEP!" */
    uint8_t p[4], c[8];
    int i;

    /* byte-reverse param relative to HDCP derivation */
    p[0] = (param >> 24) & 0xff;
    p[1] = (param >> 16) & 0xff;
    p[2] = (param >>  8) & 0xff;
    p[3] = (param >>  0) & 0xff;

    /* byte-reverse input_ctr relative to HDCP derivation */
    for (i = 0; i < 8; i++)
        c[i] = (input_ctr >> ((7 - i) * 8)) & 0xff;

    for (i = 0; i < 8; i++)
        key[i] = c[i] ^ salt[i & 3];
    for (i = 0; i < 4; i++)
        key[8 + i] = p[i] ^ salt[i] ^ 0xa5;
    for (i = 0; i < 4; i++)
        key[12 + i] = p[i] ^ salt[i] ^ 0x5a;
}

int dummy_xor_hdcp_cipher(void *opaque, uint32_t param,
    uint64_t input_ctr, uint8_t *buf, size_t len, int flags)
{
    uint8_t key[16];
    size_t i;

    derive_hdcp_key(key, param, input_ctr);
    for (i = 0; i < len; i++)
        buf[i] ^= key[i & 15];

    return 0;
}

int dummy_xor_privacy_cipher(void *opaque, uint32_t param,
    uint64_t input_ctr, uint8_t *buf, size_t len, int flags)
{
    uint8_t key[16];
    size_t i;

    derive_privacy_key(key, param, input_ctr);
    for (i = 0; i < len; i++)
        buf[i] ^= key[i & 15];

    return 0;
}
