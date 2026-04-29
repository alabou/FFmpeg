#ifndef AVFORMAT_RTP_CIPHER_H
#define AVFORMAT_RTP_CIPHER_H

#include <stddef.h>
#include <stdint.h>

int dummy_xor_hdcp_cipher(void *opaque, uint32_t param,
    uint64_t input_ctr, uint8_t *buf, size_t len, int flags);

int dummy_xor_privacy_cipher(void *opaque, uint32_t param,
    uint64_t input_ctr, uint8_t *buf, size_t len, int flags);

#endif /* AVFORMAT_RTP_CIPHER_H */
