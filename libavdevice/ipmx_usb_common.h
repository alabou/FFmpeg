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

#ifndef AVDEVICE_IPMX_USB_COMMON_H
#define AVDEVICE_IPMX_USB_COMMON_H

#include <stdint.h>
#include "ipmx_usb_proto.h"
#include "ipmx_usb_pep.h"
#include "libavformat/avformat.h"

#define IPMX_USB_RECV_BUF_SIZE   (256 * 1024)
#define IPMX_USB_BUILD_BUF_SIZE  131072

/*
 * Substreamid (SSID) allocation for encrypted channels:
 *   0/1   — control channel (S2R / R2S)
 *   2/3   — data-channel handshake (USB_STREAM_INFO / USB_STREAM_STATUS)
 *   4+    — data-channel payload (allocated per device)
 * LSB encodes direction: 0 = S2R, 1 = R2S.
 */
#define IPMX_USB_DATA_HANDSHAKE_S2R_SSID   2
#define IPMX_USB_DATA_HANDSHAKE_R2S_SSID   3
#define IPMX_USB_DATA_PAYLOAD_SSID(ch_idx) ((uint16_t)(2 * ((ch_idx) + 2)))

/*
 * MAC failure sentinel returned by ipmx_usb_recv_message() when CMAC
 * verification fails.  The caller must not process the message payload
 * but may inspect msg->hdr.msg_type (peeked from the clear-text header).
 */
#define IPMX_USB_ERR_MAC_FAIL  AVERROR(EILSEQ)

/* USB RSTATUS for MAC-failed requests (Appendix A.1, Table 19) */
#define IPMX_USB_RSTATUS_REQUEST_FAILED  0x80000500U

/* PEP state carried by both input and output devices */
typedef struct IpmxUsbPepState {
    IpmxPepContext pep_s2r;
    IpmxPepContext pep_r2s;
    int            encrypted;
    uint64_t       base_iv;
    uint8_t        psk[64];
    int            psk_len;
    uint8_t        key_generator[16];
    uint32_t       sender_kv;
    uint32_t       receiver_kv;
    enum IpmxPepMode pep_mode;
} IpmxUsbPepState;

/*
 * PEP forward-progress tracker (TR-10-13 Section 18).
 *
 * One instance per receive direction per substreamid.  Tracks the last
 * received CTR and key_version so that ipmx_usb_recv_message() can
 * verify monotonic forward progress of the peer's encryption state.
 */
typedef struct IpmxPepForwardProgress {
    int      initialized;   /* 0 until first encrypted msg received */
    int      strict;        /* 1 = error + return EPROTO; 0 = warn + continue */
    uint64_t last_ctr;
    uint32_t last_kv;
} IpmxPepForwardProgress;

/* ---------------------------------------------------------------------------
 * TCP helpers
 * --------------------------------------------------------------------------- */

int ipmx_usb_tcp_connect(const char *host, int port);
int ipmx_usb_tcp_listen(int port);
int ipmx_usb_tcp_accept(int listen_fd, int timeout_ms);
int ipmx_usb_send_all(int sock, const uint8_t *buf, int len);

int ipmx_usb_recv_fill(uint8_t *rbuf, int *rbuf_len, int *rbuf_pos,
                        int sock, int timeout_ms);

/**
 * Receive one framed TR-10-14 message with optional PEP decryption.
 *
 * @param log_ctx  AVFormatContext for logging
 * @param pep      PEP state (may be NULL if unencrypted)
 * @param sock     TCP socket fd
 * @param rbuf     Persistent receive buffer (IPMX_USB_RECV_BUF_SIZE bytes)
 * @param rbuf_len Current fill level
 * @param rbuf_pos Current read cursor
 * @param s2r_ssid Substreamid for S2R IV computation
 * @param ctr_s2r  Pointer to S2R CTR counter (advanced on decrypt)
 * @param fp       Forward-progress tracker (may be NULL to skip checks)
 * @param msg      Output: parsed message
 * @param timeout_ms  Maximum wait time
 * @return 0 on success, IPMX_USB_ERR_MAC_FAIL on CMAC failure
 *         (msg->hdr.msg_type is valid, payload is not),
 *         other negative on error/timeout
 */
int ipmx_usb_recv_message(void *log_ctx, IpmxUsbPepState *pep,
                           int sock,
                           uint8_t *rbuf, int *rbuf_len, int *rbuf_pos,
                           uint16_t s2r_ssid, uint64_t *ctr_s2r,
                           IpmxPepForwardProgress *fp,
                           IpmxUsbMsg *msg, int timeout_ms);

/**
 * Send a built message with optional PEP encryption.
 *
 * @param log_ctx  AVFormatContext for logging
 * @param pep      PEP state (may be NULL if unencrypted)
 * @param sock     TCP socket fd
 * @param r2s_ssid Substreamid for R2S IV computation
 * @param ctr_r2s  Pointer to R2S CTR counter (advanced on encrypt)
 * @param buf      Message buffer (will be modified in-place for encryption)
 * @param len      Message length
 * @return 0 on success, negative on error
 */
int ipmx_usb_send_message(void *log_ctx, IpmxUsbPepState *pep,
                           int sock, uint16_t r2s_ssid,
                           uint64_t *ctr_r2s, uint8_t *buf, int len);

/* ---------------------------------------------------------------------------
 * PEP initialization helpers
 * --------------------------------------------------------------------------- */

int ipmx_usb_hex_decode(const char *hex, uint8_t *out, int max_len);
enum IpmxPepMode ipmx_usb_parse_pep_mode_str(const char *val);

/**
 * Parse SDP file for c=, m=, and a=privacy: attributes.
 * Populates host_out/port_out and pep fields.
 */
int ipmx_usb_parse_sdp(void *log_ctx,
                        const char *path,
                        char *host_out, int host_size, int *port_out,
                        IpmxUsbPepState *pep);

/**
 * Initialize PEP from PSK hex, SDP, and CLI overrides.
 * Derives both S2R and R2S keys.
 */
int ipmx_usb_init_pep(void *log_ctx, IpmxUsbPepState *pep,
                       const char *psk_hex,
                       const char *pep_mode_str,
                       const char *pep_iv_str,
                       const char *pep_key_generator_str,
                       const char *pep_key_version_str,
                       int is_sender);

#endif /* AVDEVICE_IPMX_USB_COMMON_H */
