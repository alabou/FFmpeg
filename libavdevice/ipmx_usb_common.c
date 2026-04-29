/*
 * IPMX USB (TR-10-14) shared helpers for FFmpeg input/output devices.
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

#include "ipmx_usb_common.h"
#include "libavutil/mem.h"
#include "libavutil/time.h"
#include "libavutil/avstring.h"
#include "libavutil/log.h"
#include "libavutil/error.h"

#include <openssl/rand.h>

#include <poll.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* ---------------------------------------------------------------------------
 * TCP helpers
 * --------------------------------------------------------------------------- */

int ipmx_usb_tcp_connect(const char *host, int port)
{
    struct sockaddr_in addr;
    int fd, flag;

    fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0)
        return -1;

    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(port);
    if (inet_pton(AF_INET, host, &addr.sin_addr) != 1) {
        close(fd);
        return -1;
    }

    if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        close(fd);
        return -1;
    }

    flag = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));
    return fd;
}

int ipmx_usb_tcp_listen(int port)
{
    struct sockaddr_in addr;
    int fd, opt = 1;

    fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0)
        return -1;

    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(port);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        close(fd);
        return -1;
    }

    if (listen(fd, 4) < 0) {
        close(fd);
        return -1;
    }

    return fd;
}

int ipmx_usb_tcp_accept(int listen_fd, int timeout_ms)
{
    struct pollfd pfd;
    int fd, flag;

    pfd.fd     = listen_fd;
    pfd.events = POLLIN;
    if (poll(&pfd, 1, timeout_ms) <= 0)
        return -1;

    fd = accept(listen_fd, NULL, NULL);
    if (fd < 0)
        return -1;

    flag = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));
    return fd;
}

int ipmx_usb_send_all(int sock, const uint8_t *buf, int len)
{
    int sent = 0;
    while (sent < len) {
        int n = send(sock, buf + sent, len - sent, MSG_NOSIGNAL);
        if (n <= 0) {
            if (n < 0 && (errno == EINTR || errno == EAGAIN))
                continue;
            return -1;
        }
        sent += n;
    }
    return 0;
}

int ipmx_usb_recv_fill(uint8_t *rbuf, int *rbuf_len, int *rbuf_pos,
                        int sock, int timeout_ms)
{
    struct pollfd pfd;
    int space, n;

    if (*rbuf_pos > 0 && *rbuf_len > *rbuf_pos) {
        memmove(rbuf, rbuf + *rbuf_pos, *rbuf_len - *rbuf_pos);
        *rbuf_len -= *rbuf_pos;
        *rbuf_pos = 0;
    } else if (*rbuf_pos > 0) {
        *rbuf_len = 0;
        *rbuf_pos = 0;
    }

    pfd.fd     = sock;
    pfd.events = POLLIN;
    if (poll(&pfd, 1, timeout_ms) <= 0)
        return AVERROR(EAGAIN);

    space = IPMX_USB_RECV_BUF_SIZE - *rbuf_len;
    if (space <= 0)
        return AVERROR(ENOMEM);

    n = recv(sock, rbuf + *rbuf_len, space, 0);
    if (n == 0)
        return AVERROR_EOF;
    if (n < 0) {
        if (errno == EINTR || errno == EAGAIN)
            return AVERROR(EAGAIN);
        return AVERROR(errno);
    }
    *rbuf_len += n;
    return 0;
}

/* ---------------------------------------------------------------------------
 * PEP forward-progress checks (TR-10-13 Section 18)
 * --------------------------------------------------------------------------- */

/*
 * CTR: "shall be larger to the last ctr value received ...
 *  taking into account the 2^64 wrap around and the monotonic
 *  incremental nature of the value."
 */
static int ctr_is_forward(uint64_t incoming, uint64_t last)
{
    uint64_t delta = incoming - last;
    return (delta > 0) && (delta < (UINT64_C(1) << 63));
}

/*
 * key_version: "shall be larger or equal to the last key_version
 *  value received ... taking into account the 2^32 wrap around ...
 *  and that the effective value cannot be farther than 2^31 units."
 */
static int kv_is_forward(uint32_t incoming, uint32_t last)
{
    uint32_t delta;
    if (incoming == last)
        return 1;
    delta = incoming - last;
    return (delta > 0) && (delta <= (UINT32_C(1) << 31));
}

/* ---------------------------------------------------------------------------
 * Message receive with framing, timeout, and optional PEP decryption
 * --------------------------------------------------------------------------- */

int ipmx_usb_recv_message(void *log_ctx, IpmxUsbPepState *pep,
                           int sock,
                           uint8_t *rbuf, int *rbuf_len, int *rbuf_pos,
                           uint16_t s2r_ssid, uint64_t *ctr_s2r,
                           IpmxPepForwardProgress *fp,
                           IpmxUsbMsg *msg, int timeout_ms)
{
    int64_t deadline = av_gettime_relative() + (int64_t)timeout_ms * 1000;
    int ret;

    for (;;) {
        int avail = *rbuf_len - *rbuf_pos;
        if (avail > 0) {
            uint32_t msg_len;
            if (ipmx_usb_peek_length(rbuf + *rbuf_pos,
                                     avail, &msg_len) == 0) {
                if ((int)msg_len <= avail) {
                    uint8_t *frame = rbuf + *rbuf_pos;

                    if (pep && pep->encrypted) {
                        int mac_ok = 0;
                        uint64_t iv = ipmx_pep_compute_iv_prime(
                            pep->base_iv, s2r_ssid);

                        uint64_t incoming_ctr = 0;
                        uint32_t incoming_kv = 0;
                        if (msg_len >= 16) {
                            incoming_ctr = ((uint64_t)frame[0] << 56) |
                                           ((uint64_t)frame[1] << 48) |
                                           ((uint64_t)frame[2] << 40) |
                                           ((uint64_t)frame[3] << 32) |
                                           ((uint64_t)frame[4] << 24) |
                                           ((uint64_t)frame[5] << 16) |
                                           ((uint64_t)frame[6] << 8)  |
                                            (uint64_t)frame[7];
                            incoming_kv  = ((uint32_t)frame[8] << 24) |
                                           ((uint32_t)frame[9] << 16) |
                                           ((uint32_t)frame[10] << 8) |
                                            (uint32_t)frame[11];
                        }

                        /* TR-10-13 §18: verify forward progress (SHOULD) */
                        if (fp && fp->initialized) {
                            if (!ctr_is_forward(incoming_ctr, fp->last_ctr)) {
                                av_log(log_ctx,
                                       fp->strict ? AV_LOG_ERROR
                                                  : AV_LOG_WARNING,
                                       "PEP: ctr not making forward progress "
                                       "(incoming=%llu last=%llu ssid=%u)\n",
                                       (unsigned long long)incoming_ctr,
                                       (unsigned long long)fp->last_ctr,
                                       s2r_ssid);
                                if (fp->strict)
                                    return AVERROR(EPROTO);
                            }
                            if (incoming_kv != 0 &&
                                !kv_is_forward(incoming_kv, fp->last_kv)) {
                                av_log(log_ctx,
                                       fp->strict ? AV_LOG_ERROR
                                                  : AV_LOG_WARNING,
                                       "PEP: key_version not making forward "
                                       "progress (incoming=0x%08x "
                                       "last=0x%08x ssid=%u)\n",
                                       incoming_kv, fp->last_kv, s2r_ssid);
                                if (fp->strict)
                                    return AVERROR(EPROTO);
                            }
                        }

                        if (incoming_kv != 0 &&
                            incoming_kv != pep->sender_kv) {
                            uint8_t kv_bytes[4];
                            int key_bits = pep->pep_s2r.key_len * 8;
                            av_log(log_ctx, AV_LOG_INFO,
                                   "Sender key_version changed %u -> %u, "
                                   "re-deriving S2R key\n",
                                   pep->sender_kv, incoming_kv);
                            pep->sender_kv = incoming_kv;
                            kv_bytes[0] = (incoming_kv >> 24) & 0xFF;
                            kv_bytes[1] = (incoming_kv >> 16) & 0xFF;
                            kv_bytes[2] = (incoming_kv >> 8)  & 0xFF;
                            kv_bytes[3] = incoming_kv & 0xFF;
                            pep->pep_s2r.key_version = incoming_kv;
                            if (ipmx_pep_derive_key(
                                    pep->psk, pep->psk_len,
                                    pep->key_generator, kv_bytes,
                                    pep->pep_s2r.key,
                                    key_bits) < 0) {
                                av_log(log_ctx, AV_LOG_ERROR,
                                       "S2R key re-derivation failed\n");
                                *rbuf_pos += msg_len;
                                continue;
                            }
                        }

                        ret = ipmx_pep_decrypt_usb_msg(&pep->pep_s2r, iv,
                                                       frame, msg_len,
                                                       &mac_ok);
                        if (ret < 0) {
                            av_log(log_ctx, AV_LOG_ERROR,
                                   "PEP decryption failed\n");
                            *rbuf_pos += msg_len;
                            continue;
                        }
                        if (!mac_ok) {
                            av_log(log_ctx, AV_LOG_WARNING,
                                   "PEP MAC verification failed "
                                   "(ssid=%u)\n", s2r_ssid);
                            /* Peek msg_type from the clear-text header so
                             * the caller knows what kind of message failed,
                             * but do NOT advance CTR, update forward-progress,
                             * or parse the payload. */
                            memset(msg, 0, sizeof(*msg));
                            if (msg_len > 12)
                                msg->hdr.msg_type = frame[12];
                            *rbuf_pos += msg_len;
                            return IPMX_USB_ERR_MAC_FAIL;
                        }
                        *ctr_s2r += ipmx_pep_usb_ctr_advance(msg_len);

                        if (fp) {
                            fp->last_ctr = incoming_ctr;
                            if (incoming_kv != 0)
                                fp->last_kv = incoming_kv;
                            fp->initialized = 1;
                        }

                        memset(frame, 0, 12);
                    }

                    ret = ipmx_usb_parse_one(frame, msg_len, msg);
                    *rbuf_pos += msg_len;
                    if (ret < 0) {
                        av_log(log_ctx, AV_LOG_WARNING,
                               "Failed to parse message, skipping\n");
                        continue;
                    }
                    return 0;
                }
            }
        }

        {
            int64_t now = av_gettime_relative();
            int remain_ms;
            if (now >= deadline)
                return AVERROR(EAGAIN);

            remain_ms = (int)((deadline - now) / 1000);
            if (remain_ms <= 0)
                return AVERROR(EAGAIN);

            ret = ipmx_usb_recv_fill(rbuf, rbuf_len, rbuf_pos,
                                     sock, remain_ms);
            if (ret == AVERROR_EOF)
                return AVERROR_EOF;
            if (ret < 0 && ret != AVERROR(EAGAIN))
                return ret;
        }
    }
}

/* ---------------------------------------------------------------------------
 * Send a built message with optional PEP encryption
 * --------------------------------------------------------------------------- */

int ipmx_usb_send_message(void *log_ctx, IpmxUsbPepState *pep,
                           int sock, uint16_t r2s_ssid,
                           uint64_t *ctr_r2s, uint8_t *buf, int len)
{
    if (pep && pep->encrypted) {
        uint64_t iv;
        int ret;

        buf[0] = (*ctr_r2s >> 56) & 0xFF;
        buf[1] = (*ctr_r2s >> 48) & 0xFF;
        buf[2] = (*ctr_r2s >> 40) & 0xFF;
        buf[3] = (*ctr_r2s >> 32) & 0xFF;
        buf[4] = (*ctr_r2s >> 24) & 0xFF;
        buf[5] = (*ctr_r2s >> 16) & 0xFF;
        buf[6] = (*ctr_r2s >> 8)  & 0xFF;
        buf[7] =  *ctr_r2s        & 0xFF;
        buf[8]  = (pep->receiver_kv >> 24) & 0xFF;
        buf[9]  = (pep->receiver_kv >> 16) & 0xFF;
        buf[10] = (pep->receiver_kv >> 8)  & 0xFF;
        buf[11] =  pep->receiver_kv        & 0xFF;

        iv = ipmx_pep_compute_iv_prime(pep->base_iv, r2s_ssid);
        ret = ipmx_pep_encrypt_usb_msg(&pep->pep_r2s, iv, buf, len);
        if (ret < 0) {
            av_log(log_ctx, AV_LOG_ERROR,
                   "PEP encryption failed (ssid=%u)\n", r2s_ssid);
            return AVERROR_EXIT;
        }
        *ctr_r2s += ipmx_pep_usb_ctr_advance(len);
    }

    if (ipmx_usb_send_all(sock, buf, len) < 0) {
        av_log(log_ctx, AV_LOG_ERROR, "Failed to send message\n");
        return AVERROR(EIO);
    }
    return 0;
}

/* ---------------------------------------------------------------------------
 * PEP initialization helpers
 * --------------------------------------------------------------------------- */

int ipmx_usb_hex_decode(const char *hex, uint8_t *out, int max_len)
{
    return av_ipmx_pep_hex_decode(hex, out, max_len);
}

enum IpmxPepMode ipmx_usb_parse_pep_mode_str(const char *val)
{
    return av_ipmx_pep_parse_mode_str(val);
}

int ipmx_usb_parse_sdp(void *log_ctx,
                        const char *path,
                        char *host_out, int host_size, int *port_out,
                        IpmxUsbPepState *pep)
{
    FILE *f;
    char line[1024];

    f = fopen(path, "r");
    if (!f) {
        av_log(log_ctx, AV_LOG_ERROR, "Cannot open SDP file: %s\n", path);
        return AVERROR(ENOENT);
    }

    host_out[0] = '\0';
    *port_out = 0;

    while (fgets(line, sizeof(line), f)) {
        char *p, *saveptr, *token, *eq, *key, *val, *end;

        if (line[0] == 'c' && line[1] == '=') {
            char nettype[8], addrtype[8], addr[256];
            if (sscanf(line + 2, "%7s %7s %255s", nettype, addrtype, addr) == 3) {
                char *slash = strchr(addr, '/');
                if (slash) *slash = '\0';
                av_strlcpy(host_out, addr, host_size);
            }
            continue;
        }

        if (line[0] == 'm' && line[1] == '=') {
            char mtype[32];
            int mport = 0;
            if (sscanf(line + 2, "%31s %d", mtype, &mport) == 2) {
                if (mport > 0 && mport <= 65535)
                    *port_out = mport;
            }
            continue;
        }

        p = strstr(line, "a=privacy:");
        if (!p)
            continue;
        p += 10;

        saveptr = NULL;
        for (token = strtok_r(p, ";", &saveptr); token;
             token = strtok_r(NULL, ";", &saveptr)) {
            while (*token == ' ') token++;
            eq = strchr(token, '=');
            if (!eq) continue;
            *eq = '\0';
            key = token;
            val = eq + 1;
            end = val + strlen(val) - 1;
            while (end > val && (*end == '\r' || *end == '\n' || *end == ' '))
                *end-- = '\0';

            if (!strcmp(key, "mode")) {
                pep->pep_mode = ipmx_usb_parse_pep_mode_str(val);
            } else if (!strcmp(key, "iv") || !strcmp(key, " iv")) {
                pep->base_iv = strtoull(val, NULL, 16);
            } else if (!strcmp(key, "key_generator") ||
                       !strcmp(key, " key_generator")) {
                ipmx_usb_hex_decode(val, pep->key_generator, 16);
            } else if (!strcmp(key, "key_version") ||
                       !strcmp(key, " key_version")) {
                pep->sender_kv = (uint32_t)strtoul(val, NULL, 16);
            }
        }
    }

    fclose(f);

    if (host_out[0])
        av_log(log_ctx, AV_LOG_INFO, "SDP: address %s:%d\n",
               host_out, *port_out);
    return 0;
}

int ipmx_usb_init_pep(void *log_ctx, IpmxUsbPepState *pep,
                       const char *psk_hex,
                       const char *pep_mode_str,
                       const char *pep_iv_str,
                       const char *pep_key_generator_str,
                       const char *pep_key_version_str,
                       int is_sender)
{
    uint8_t kv_bytes[4];
    int key_bits;

    if (psk_hex) {
        pep->psk_len = ipmx_usb_hex_decode(psk_hex, pep->psk, 64);
        if (pep->psk_len != 16 && pep->psk_len != 32 && pep->psk_len != 64) {
            av_log(log_ctx, AV_LOG_ERROR,
                   "PSK must be 16, 32, or 64 bytes (32/64/128 hex digits)\n");
            return AVERROR(EINVAL);
        }
    } else {
        memset(pep->psk, 0, 16);
        pep->psk_len = 16;
        av_log(log_ctx, AV_LOG_INFO, "Using default all-zero 128-bit PSK\n");
    }

    /* Defaults if no SDP was parsed beforehand */
    if (pep->pep_mode == 0 && !pep_mode_str)
        pep->pep_mode = IPMX_PEP_AES_128_CTR_CMAC_64_AAD;
    if (pep->sender_kv == 0 && !pep_key_version_str)
        pep->sender_kv = 1;

    if (pep_mode_str)
        pep->pep_mode = ipmx_usb_parse_pep_mode_str(pep_mode_str);
    if (pep_iv_str)
        pep->base_iv = strtoull(pep_iv_str, NULL, 16);
    if (pep_key_generator_str)
        ipmx_usb_hex_decode(pep_key_generator_str, pep->key_generator, 16);
    if (pep_key_version_str)
        pep->sender_kv = (uint32_t)strtoul(pep_key_version_str, NULL, 16);

    key_bits = (pep->pep_mode == IPMX_PEP_AES_128_CTR ||
                pep->pep_mode == IPMX_PEP_AES_128_CTR_CMAC_64 ||
                pep->pep_mode == IPMX_PEP_AES_128_CTR_CMAC_64_AAD) ? 128 : 256;

    /*
     * Key derivation: S2R key from sender_kv, R2S key from random kv.
     *
     * The common send/recv functions always use:
     *   send_message → pep_r2s + receiver_kv  (encrypt outgoing)
     *   recv_message → pep_s2r + sender_kv    (decrypt incoming)
     *
     * For Receiver: send=R2S, recv=S2R → pep_r2s=R2S key, pep_s2r=S2R key
     * For Sender:   send=S2R, recv=R2S → pep_r2s=S2R key, pep_s2r=R2S key
     */

    /* Derive the known S2R key (from the SDP/CLI sender_kv) */
    kv_bytes[0] = (pep->sender_kv >> 24) & 0xFF;
    kv_bytes[1] = (pep->sender_kv >> 16) & 0xFF;
    kv_bytes[2] = (pep->sender_kv >> 8) & 0xFF;
    kv_bytes[3] = pep->sender_kv & 0xFF;

    {
        IpmxPepContext s2r_ctx;
        s2r_ctx.mode = pep->pep_mode;
        s2r_ctx.iv = pep->base_iv;
        s2r_ctx.key_version = pep->sender_kv;
        s2r_ctx.key_len = key_bits / 8;
        if (ipmx_pep_derive_key(pep->psk, pep->psk_len,
                                pep->key_generator, kv_bytes,
                                s2r_ctx.key, key_bits) < 0) {
            av_log(log_ctx, AV_LOG_ERROR, "S2R key derivation failed\n");
            return AVERROR_EXIT;
        }

        /* Generate random key_version for the R2S direction */
        {
            uint8_t rand4[4];
            uint32_t local_kv;
            IpmxPepContext r2s_ctx;

            if (RAND_bytes(rand4, 4) != 1) {
                av_log(log_ctx, AV_LOG_ERROR,
                       "RAND_bytes failed for local_kv\n");
                return AVERROR_EXIT;
            }
            local_kv = ((uint32_t)rand4[0] << 24) |
                       ((uint32_t)rand4[1] << 16) |
                       ((uint32_t)rand4[2] << 8)  |
                        (uint32_t)rand4[3];
            if (local_kv == 0)
                local_kv = 1;

            kv_bytes[0] = (local_kv >> 24) & 0xFF;
            kv_bytes[1] = (local_kv >> 16) & 0xFF;
            kv_bytes[2] = (local_kv >> 8)  & 0xFF;
            kv_bytes[3] =  local_kv        & 0xFF;

            r2s_ctx.mode = pep->pep_mode;
            r2s_ctx.iv = pep->base_iv;
            r2s_ctx.key_version = local_kv;
            r2s_ctx.key_len = key_bits / 8;
            if (ipmx_pep_derive_key(pep->psk, pep->psk_len,
                                    pep->key_generator, kv_bytes,
                                    r2s_ctx.key, key_bits) < 0) {
                av_log(log_ctx, AV_LOG_ERROR, "R2S key derivation failed\n");
                return AVERROR_EXIT;
            }

            if (is_sender) {
                /* Sender: outgoing=S2R, incoming=R2S */
                pep->pep_r2s     = s2r_ctx;  /* encrypt outgoing with S2R key */
                pep->receiver_kv = pep->sender_kv; /* stamp our sender_kv */
                pep->pep_s2r     = r2s_ctx;  /* decrypt incoming with R2S key */
                pep->sender_kv   = local_kv; /* expect their kv in incoming */
            } else {
                /* Receiver: outgoing=R2S, incoming=S2R */
                pep->pep_s2r     = s2r_ctx;
                /* sender_kv stays as is */
                pep->pep_r2s     = r2s_ctx;
                pep->receiver_kv = local_kv;
            }
        }
    }

    pep->encrypted = 1;
    av_log(log_ctx, AV_LOG_INFO,
           "PEP initialized: mode=%d key_bits=%d sender_kv=0x%08X "
           "receiver_kv=0x%08X (is_sender=%d)\n",
           pep->pep_mode, key_bits, pep->sender_kv, pep->receiver_kv,
           is_sender);
    return 0;
}
