/*
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

/**
 * @file
 * TR-10-14 IPMX USB TCP protocol message parsing and building.
 *
 * Direct C translation of ipmx_usb_message.py.
 * All field sizes and byte offsets follow VSF TR-10-14 (2024-09-24).
 * Big-endian byte order throughout.
 *
 * Message framing (Table 1):
 *   Bytes  0- 7  CTR         64-bit AES-CTR counter (0 when unencrypted)
 *   Bytes  8-11  KEYVERSION  32-bit key version     (0 when unencrypted)
 *   Byte  12     MSGTYPE     8-bit message type
 *   Bytes 13-15  Reserved(7 bits) || LENGTH(17 bits) — big-endian packed 3-byte word
 *   Bytes 16 … LENGTH-9  DATA  (LENGTH – 24) bytes
 *   Bytes LENGTH-8 … LENGTH-1  MAC  64-bit CMAC (0 when unencrypted)
 */

#include "ipmx_usb_proto.h"

#include <string.h>

#include "libavutil/mem.h"
#include "libavutil/avutil.h"
#include "libavutil/error.h"
#include "libavutil/intreadwrite.h"

/* ---------------------------------------------------------------------------
 * Internal: build_message helper
 * Writes CTR=0, KEYVERSION=0, MSGTYPE, packed LENGTH, DATA, MAC=0.
 * Returns total bytes written, or AVERROR(EINVAL) on error.
 * --------------------------------------------------------------------------- */
static int build_message(uint8_t *buf, int buf_size, uint8_t msg_type,
                         const uint8_t *data, int data_len)
{
    int total_len = IPMX_USB_HEADER_SIZE + data_len + IPMX_USB_MAC_SIZE;
    uint8_t *p;
    uint32_t rsvd_len;

    if (total_len < IPMX_USB_MIN_LENGTH || total_len > IPMX_USB_MAX_LENGTH)
        return AVERROR(EINVAL);
    if (buf_size < total_len)
        return AVERROR(EINVAL);

    p = buf;

    /* CTR = 0 (8 bytes) */
    AV_WB64(p, 0);
    p += 8;

    /* KEYVERSION = 0 (4 bytes) */
    AV_WB32(p, 0);
    p += 4;

    /* MSGTYPE (1 byte) */
    *p++ = msg_type & 0xFF;

    /* Reserved(7 bits) || LENGTH(17 bits) — 3 bytes big-endian */
    rsvd_len = total_len & 0x1FFFF;
    *p++ = (rsvd_len >> 16) & 0xFF;
    *p++ = (rsvd_len >> 8) & 0xFF;
    *p++ = rsvd_len & 0xFF;

    /* DATA */
    if (data_len > 0 && data != NULL) {
        memcpy(p, data, data_len);
        p += data_len;
    }

    /* MAC = 0 (8 bytes) */
    memset(p, 0, IPMX_USB_MAC_SIZE);

    return total_len;
}

/* ---------------------------------------------------------------------------
 * Peek length
 * --------------------------------------------------------------------------- */
int ipmx_usb_peek_length(const uint8_t *buf, int buf_len, uint32_t *out_length)
{
    uint32_t word24, length;

    if (buf_len < IPMX_USB_HEADER_SIZE)
        return -1;

    word24 = (buf[13] << 16) | (buf[14] << 8) | buf[15];
    length = word24 & 0x1FFFF;

    if (length < IPMX_USB_MIN_LENGTH || length > IPMX_USB_MAX_LENGTH)
        return -1;

    *out_length = length;
    return 0;
}

/* ---------------------------------------------------------------------------
 * Parse header (16 bytes). Returns 0 on success, -1 on error.
 * --------------------------------------------------------------------------- */
static int parse_header(const uint8_t *buf, int buf_len,
                        IpmxUsbMsgHeader *hdr)
{
    uint32_t word24, length;

    if (buf_len < IPMX_USB_HEADER_SIZE)
        return -1;

    word24 = (buf[13] << 16) | (buf[14] << 8) | buf[15];
    length = word24 & 0x1FFFF;

    if (length < IPMX_USB_MIN_LENGTH || length > IPMX_USB_MAX_LENGTH)
        return -1;

    hdr->ctr         = AV_RB64(buf + 0);
    hdr->key_version = AV_RB32(buf + 8);
    hdr->msg_type    = buf[12];
    hdr->length      = length;
    hdr->data_len    = length - IPMX_USB_HEADER_SIZE - IPMX_USB_MAC_SIZE;

    return 0;
}

/* ---------------------------------------------------------------------------
 * Type-specific payload decoders (for unencrypted messages only)
 * --------------------------------------------------------------------------- */

static int decode_sender_connection_info(const uint8_t *data, int data_len,
                                         IpmxUsbMsg *msg)
{
    int sn_len;

    if (data_len < 66)
        return -1;

    /* byte 0: MAVER(4) || MIVER(4), byte 1: Reserved, bytes 2-4: CID */
    memcpy(msg->payload.sender_connection_info.cid, data + 2, 3);

    /* bytes 5-65: serial number, null-terminated */
    sn_len = 61;
    if (sn_len >= (int)sizeof(msg->payload.sender_connection_info.serial_number))
        sn_len = sizeof(msg->payload.sender_connection_info.serial_number) - 1;
    memcpy(msg->payload.sender_connection_info.serial_number, data + 5, sn_len);
    msg->payload.sender_connection_info.serial_number[sn_len] = '\0';
    /* Trim trailing zeros */
    while (sn_len > 0 && msg->payload.sender_connection_info.serial_number[sn_len - 1] == '\0')
        sn_len--;
    msg->payload.sender_connection_info.serial_number[sn_len] = '\0';
    msg->payload.sender_connection_info.serial_len = sn_len;

    return 0;
}

static int decode_sender_connection_status(const uint8_t *data, int data_len,
                                           IpmxUsbMsg *msg)
{
    int sn_len;

    if (data_len < 68)
        return -1;

    /* byte 1: HBEAT(5 bits) */
    msg->payload.sender_connection_status.hbeat_index = data[1] & 0x1F;
    msg->payload.sender_connection_status.data_port   = AV_RB16(data + 2);
    memcpy(msg->payload.sender_connection_status.cid, data + 4, 3);

    sn_len = 61;
    if (sn_len >= (int)sizeof(msg->payload.sender_connection_status.serial_number))
        sn_len = sizeof(msg->payload.sender_connection_status.serial_number) - 1;
    memcpy(msg->payload.sender_connection_status.serial_number, data + 7, sn_len);
    msg->payload.sender_connection_status.serial_number[sn_len] = '\0';
    while (sn_len > 0 && msg->payload.sender_connection_status.serial_number[sn_len - 1] == '\0')
        sn_len--;
    msg->payload.sender_connection_status.serial_number[sn_len] = '\0';
    msg->payload.sender_connection_status.serial_len = sn_len;

    return 0;
}

static int decode_usb_stream_info(const uint8_t *data, int data_len,
                                  IpmxUsbMsg *msg)
{
    int busid_len;

    if (data_len < 66)
        return -1;

    /* Section 9.7, Table 7: SUBSTREAMID(1) + USBSPEED(1) + BUSID(64) */
    msg->payload.usb_stream_info.substreamid = data[0];
    msg->payload.usb_stream_info.usbspeed   = data[1];

    busid_len = 64;
    if (busid_len >= (int)sizeof(msg->payload.usb_stream_info.busid))
        busid_len = sizeof(msg->payload.usb_stream_info.busid) - 1;
    memcpy(msg->payload.usb_stream_info.busid, data + 2, busid_len);
    msg->payload.usb_stream_info.busid[busid_len] = '\0';
    while (busid_len > 0 && msg->payload.usb_stream_info.busid[busid_len - 1] == '\0')
        busid_len--;
    msg->payload.usb_stream_info.busid[busid_len] = '\0';
    msg->payload.usb_stream_info.busid_len = busid_len;

    return 0;
}

static int decode_usb_stream_status(const uint8_t *data, int data_len,
                                    IpmxUsbMsg *msg)
{
    if (data_len < 1)
        return -1;

    msg->payload.usb_stream_status.cstatus = data[0];
    return 0;
}

static int decode_control_submit(const uint8_t *data, int data_len,
                                  IpmxUsbMsg *msg)
{
    if (data_len < 20)
        return -1;

    msg->payload.control_submit.seqnum         = (data[0] << 16) | (data[1] << 8) | data[2];
    msg->payload.control_submit.endpoint        = (data[3] >> 4) & 0x0F;
    msg->payload.control_submit.direction      = data[3] & 0x01;
    msg->payload.control_submit.binterval      = AV_RB32(data + 4);
    msg->payload.control_submit.transferlength = AV_RB32(data + 8);
    memcpy(msg->payload.control_submit.usbdevreq, data + 12, 8);

    if (msg->payload.control_submit.direction == 0) {
        msg->payload.control_submit.transferdata     = data + 20;
        msg->payload.control_submit.transferdata_len = data_len - 20;
    } else {
        msg->payload.control_submit.transferdata     = NULL;
        msg->payload.control_submit.transferdata_len = 0;
    }
    return 0;
}

static int decode_control_submit_return(const uint8_t *data, int data_len,
                                        IpmxUsbMsg *msg)
{
    if (data_len < 12)
        return -1;

    msg->payload.control_submit_return.seqnum         = (data[0] << 16) | (data[1] << 8) | data[2];
    msg->payload.control_submit_return.endpoint       = (data[3] >> 4) & 0x0F;
    msg->payload.control_submit_return.direction      = data[3] & 0x01;
    msg->payload.control_submit_return.actuallength   = AV_RB32(data + 4);
    msg->payload.control_submit_return.rstatus       = AV_RB32(data + 8);

    if (msg->payload.control_submit_return.direction == 1 &&
        msg->payload.control_submit_return.actuallength > 0) {
        int al = msg->payload.control_submit_return.actuallength;
        if (data_len >= 12 + al) {
            msg->payload.control_submit_return.transferdata     = data + 12;
            msg->payload.control_submit_return.transferdata_len = al;
        } else {
            msg->payload.control_submit_return.transferdata     = data + 12;
            msg->payload.control_submit_return.transferdata_len = data_len - 12;
        }
    } else {
        msg->payload.control_submit_return.transferdata     = NULL;
        msg->payload.control_submit_return.transferdata_len = 0;
    }
    return 0;
}

static int decode_bulk_submit(const uint8_t *data, int data_len,
                              IpmxUsbMsg *msg)
{
    if (data_len < 12)
        return -1;

    msg->payload.bulk_submit.seqnum         = (data[0] << 16) | (data[1] << 8) | data[2];
    msg->payload.bulk_submit.endpoint       = (data[3] >> 4) & 0x0F;
    msg->payload.bulk_submit.direction      = data[3] & 0x01;
    msg->payload.bulk_submit.binterval      = AV_RB32(data + 4);
    msg->payload.bulk_submit.transferlength = AV_RB32(data + 8);

    if (msg->payload.bulk_submit.direction == 0) {
        msg->payload.bulk_submit.transferdata     = data + 12;
        msg->payload.bulk_submit.transferdata_len = data_len - 12;
    } else {
        msg->payload.bulk_submit.transferdata     = NULL;
        msg->payload.bulk_submit.transferdata_len = 0;
    }
    return 0;
}

static int decode_bulk_submit_return(const uint8_t *data, int data_len,
                                     IpmxUsbMsg *msg)
{
    return decode_control_submit_return(data, data_len, msg);
}

static int decode_interrupt_submit(const uint8_t *data, int data_len,
                                   IpmxUsbMsg *msg)
{
    return decode_bulk_submit(data, data_len, msg);
}

static int decode_interrupt_submit_return(const uint8_t *data, int data_len,
                                          IpmxUsbMsg *msg)
{
    return decode_control_submit_return(data, data_len, msg);
}

static int decode_iso_submit(const uint8_t *data, int data_len,
                             IpmxUsbMsg *msg)
{
    int iso_desc_end;

    if (data_len < 16)
        return -1;

    msg->payload.iso_submit.seqnum     = (data[0] << 16) | (data[1] << 8) | data[2];
    msg->payload.iso_submit.endpoint   = (data[3] >> 4) & 0x0F;
    msg->payload.iso_submit.direction  = data[3] & 0x01;
    msg->payload.iso_submit.binterval  = AV_RB32(data + 4);
    {
        uint32_t word32 = AV_RB32(data + 8);
        msg->payload.iso_submit.asap       = (word32 >> 31) & 0x01;
        msg->payload.iso_submit.startframe = word32 & 0x7FFFFFFF;
    }
    msg->payload.iso_submit.num_packets = AV_RB32(data + 12);

    if (msg->payload.iso_submit.num_packets > IPMX_USB_MAX_ISO_PACKETS)
        return -1;

    iso_desc_end = 16 + msg->payload.iso_submit.num_packets * 2;
    if (data_len < iso_desc_end)
        return -1;

    for (uint32_t i = 0; i < msg->payload.iso_submit.num_packets; i++)
        msg->payload.iso_submit.isolengths[i] = AV_RB16(data + 16 + i * 2);

    return 0;
}

static int decode_iso_submit_return(const uint8_t *data, int data_len,
                                   IpmxUsbMsg *msg)
{
    int desc_end, data_off;

    if (data_len < 16)
        return -1;

    msg->payload.iso_submit_return.seqnum      = (data[0] << 16) | (data[1] << 8) | data[2];
    msg->payload.iso_submit_return.endpoint    = (data[3] >> 4) & 0x0F;
    msg->payload.iso_submit_return.direction   = data[3] & 0x01;
    msg->payload.iso_submit_return.startframe  = AV_RB32(data + 4);
    msg->payload.iso_submit_return.errorcount  = AV_RB32(data + 8);
    msg->payload.iso_submit_return.num_packets = AV_RB32(data + 12);

    if (msg->payload.iso_submit_return.num_packets > IPMX_USB_MAX_ISO_PACKETS)
        return -1;

    desc_end = 16 + msg->payload.iso_submit_return.num_packets * 6;
    if (data_len < desc_end)
        return -1;

    for (uint32_t i = 0; i < msg->payload.iso_submit_return.num_packets; i++) {
        int off = 16 + i * 6;
        msg->payload.iso_submit_return.iso_packets[i].actuallength = AV_RB16(data + off);
        msg->payload.iso_submit_return.iso_packets[i].isostatus     = AV_RB32(data + off + 2);
    }

    data_off = desc_end;
    msg->payload.iso_submit_return.transferdata     = data + data_off;
    msg->payload.iso_submit_return.transferdata_len = data_len - data_off;

    return 0;
}

static int decode_cancel_submit(const uint8_t *data, int data_len,
                                IpmxUsbMsg *msg)
{
    if (data_len < 4)
        return -1;

    msg->payload.cancel_submit.seqnum    = (data[0] << 16) | (data[1] << 8) | data[2];
    msg->payload.cancel_submit.endpoint  = (data[3] >> 4) & 0x0F;
    msg->payload.cancel_submit.direction = data[3] & 0x01;
    return 0;
}

static int decode_cancel_submit_return(const uint8_t *data, int data_len,
                                       IpmxUsbMsg *msg)
{
    if (data_len < 8)
        return -1;

    msg->payload.cancel_submit_return.seqnum    = (data[0] << 16) | (data[1] << 8) | data[2];
    msg->payload.cancel_submit_return.endpoint  = (data[3] >> 4) & 0x0F;
    msg->payload.cancel_submit_return.direction = data[3] & 0x01;
    msg->payload.cancel_submit_return.rstatus   = AV_RB32(data + 4);
    return 0;
}

/* ---------------------------------------------------------------------------
 * Dispatch to type-specific decoder
 * --------------------------------------------------------------------------- */
static int decode_payload(IpmxUsbMsg *msg)
{
    const uint8_t *data = msg->raw + IPMX_USB_HEADER_SIZE;
    int data_len = msg->hdr.data_len;
    uint8_t mt = msg->hdr.msg_type;

    switch (mt) {
    case IPMX_USB_SENDER_CONNECTION_INFO:
        return decode_sender_connection_info(data, data_len, msg);
    case IPMX_USB_SENDER_CONNECTION_STATUS:
        return decode_sender_connection_status(data, data_len, msg);
    case IPMX_USB_HEARTBEAT:
        return 0;
    case IPMX_USB_STREAM_INFO:
        return decode_usb_stream_info(data, data_len, msg);
    case IPMX_USB_STREAM_STATUS:
        return decode_usb_stream_status(data, data_len, msg);
    case IPMX_USB_CONTROL_SUBMIT:
        return decode_control_submit(data, data_len, msg);
    case IPMX_USB_CONTROL_SUBMIT_RETURN:
        return decode_control_submit_return(data, data_len, msg);
    case IPMX_USB_BULK_SUBMIT:
        return decode_bulk_submit(data, data_len, msg);
    case IPMX_USB_BULK_SUBMIT_RETURN:
        return decode_bulk_submit_return(data, data_len, msg);
    case IPMX_USB_INTERRUPT_SUBMIT:
        return decode_interrupt_submit(data, data_len, msg);
    case IPMX_USB_INTERRUPT_SUBMIT_RETURN:
        return decode_interrupt_submit_return(data, data_len, msg);
    case IPMX_USB_ISOCHRONOUS_SUBMIT:
        return decode_iso_submit(data, data_len, msg);
    case IPMX_USB_ISOCHRONOUS_SUBMIT_RETURN:
        return decode_iso_submit_return(data, data_len, msg);
    case IPMX_USB_CANCEL_SUBMIT:
        return decode_cancel_submit(data, data_len, msg);
    case IPMX_USB_CANCEL_SUBMIT_RETURN:
        return decode_cancel_submit_return(data, data_len, msg);
    default:
        /* Unknown or vendor-specific: no payload decode */
        return 0;
    }
}

/* ---------------------------------------------------------------------------
 * Parse one message
 * --------------------------------------------------------------------------- */
int ipmx_usb_parse_one(const uint8_t *buf, int buf_len, IpmxUsbMsg *msg)
{
    uint32_t length;

    memset(msg, 0, sizeof(*msg));

    if (buf_len < IPMX_USB_HEADER_SIZE)
        return AVERROR(EAGAIN);

    if (parse_header(buf, buf_len, &msg->hdr) < 0)
        return AVERROR_INVALIDDATA;

    length = msg->hdr.length;
    if (buf_len < (int)length)
        return AVERROR(EAGAIN);

    memcpy(msg->mac, buf + length - IPMX_USB_MAC_SIZE, IPMX_USB_MAC_SIZE);
    msg->raw     = buf;
    msg->raw_len = length;

    /* Decode payload only when unencrypted */
    if (msg->hdr.ctr == 0 && msg->hdr.key_version == 0) {
        if (decode_payload(msg) < 0)
            return AVERROR_INVALIDDATA;
    }

    return (int)length;
}

/* ---------------------------------------------------------------------------
 * Message builders
 * --------------------------------------------------------------------------- */

int ipmx_usb_build_sender_connection_info(uint8_t *buf, int buf_size,
    const uint8_t *cid, const char *serial)
{
    uint8_t data[66];
    uint8_t *p = data;
    int sn_len = 0;

    /* ver_byte=0, reserved=0 */
    *p++ = 0;
    *p++ = 0;

    /* CID (3 bytes) */
    if (cid)
        memcpy(p, cid, 3);
    else
        memset(p, 0, 3);
    p += 3;

    /* Serial (61 bytes, null-padded) */
    if (serial) {
        sn_len = strlen(serial);
        if (sn_len > 61)
            sn_len = 61;
        memcpy(p, serial, sn_len);
    }
    memset(p + sn_len, 0, 61 - sn_len);

    return build_message(buf, buf_size, IPMX_USB_SENDER_CONNECTION_INFO, data, 66);
}

int ipmx_usb_build_sender_connection_status(uint8_t *buf, int buf_size,
    const uint8_t *cid, const char *serial, uint8_t hbeat_index, uint16_t data_port)
{
    uint8_t data[68];
    uint8_t *p = data;
    int sn_len = 0;

    /* ver_byte=0 */
    *p++ = 0;
    /* hbeat (5 bits) */
    *p++ = hbeat_index & 0x1F;

    AV_WB16(p, data_port);
    p += 2;

    if (cid)
        memcpy(p, cid, 3);
    else
        memset(p, 0, 3);
    p += 3;
    if (serial) {
        sn_len = strlen(serial);
        if (sn_len > 61)
            sn_len = 61;
        memcpy(p, serial, sn_len);
    }
    memset(p + sn_len, 0, 61 - sn_len);

    return build_message(buf, buf_size, IPMX_USB_SENDER_CONNECTION_STATUS, data, 68);
}

int ipmx_usb_build_heartbeat(uint8_t *buf, int buf_size)
{
    return build_message(buf, buf_size, IPMX_USB_HEARTBEAT, NULL, 0);
}

int ipmx_usb_build_usb_stream_info(uint8_t *buf, int buf_size,
    uint16_t substreamid, uint8_t usbspeed, const char *busid)
{
    uint8_t data[66];
    uint8_t *p = data;
    int busid_len = 0;

    /* Section 9.7, Table 7: SUBSTREAMID(1) + USBSPEED(1) + BUSID(64) */
    *p++ = (uint8_t)(substreamid & 0xFF);
    *p++ = usbspeed;
    if (busid) {
        busid_len = strlen(busid);
        if (busid_len > 64)
            busid_len = 64;
        memcpy(p, busid, busid_len);
    }
    memset(p + busid_len, 0, 64 - busid_len);

    return build_message(buf, buf_size, IPMX_USB_STREAM_INFO, data, 66);
}

int ipmx_usb_build_usb_stream_status(uint8_t *buf, int buf_size, uint8_t cstatus)
{
    uint8_t data[1] = { cstatus };
    return build_message(buf, buf_size, IPMX_USB_STREAM_STATUS, data, 1);
}

int ipmx_usb_build_control_submit(uint8_t *buf, int buf_size,
    uint32_t seqnum, uint8_t endpoint, uint8_t direction, uint32_t binterval,
    uint32_t transferlength, const uint8_t *usbdevreq,
    const uint8_t *transferdata, int transferdata_len)
{
    int data_len = 20 + (transferdata_len > 0 ? transferdata_len : 0);
    int total_len = IPMX_USB_HEADER_SIZE + data_len + IPMX_USB_MAC_SIZE;
    uint8_t *p;

    if (buf_size < total_len)
        return AVERROR(EINVAL);

    p = buf;

    /* Header: CTR=0, KEYVERSION=0, MSGTYPE */
    AV_WB64(p, 0);
    p += 8;
    AV_WB32(p, 0);
    p += 4;
    *p++ = IPMX_USB_CONTROL_SUBMIT;
    {
        uint32_t len17 = total_len & 0x1FFFF;
        *p++ = (len17 >> 16) & 0xFF;
        *p++ = (len17 >> 8) & 0xFF;
        *p++ = len17 & 0xFF;
    }

    /* DATA: SEQNUM(3) + EP_DIR(1) + BINTERVAL(4) + TRANSFERLENGTH(4) + USBDEVREQ(8) */
    *p++ = (seqnum >> 16) & 0xFF;
    *p++ = (seqnum >> 8) & 0xFF;
    *p++ = seqnum & 0xFF;
    *p++ = (endpoint << 4) | (direction & 0x01);
    AV_WB32(p, binterval);
    p += 4;
    AV_WB32(p, transferlength);
    p += 4;
    if (usbdevreq)
        memcpy(p, usbdevreq, 8);
    else
        memset(p, 0, 8);
    p += 8;

    if (transferdata_len > 0 && transferdata)
        memcpy(p, transferdata, transferdata_len);
    p += transferdata_len > 0 ? transferdata_len : 0;

    /* MAC = 0 */
    memset(p, 0, IPMX_USB_MAC_SIZE);

    return total_len;
}

int ipmx_usb_build_control_submit_return(uint8_t *buf, int buf_size,
    uint32_t seqnum, uint8_t endpoint, uint8_t direction,
    uint32_t actuallength, uint32_t rstatus,
    const uint8_t *transferdata, int transferdata_len)
{
    int data_len = 12 + (direction == 1 ? actuallength : 0);
    int total_len = IPMX_USB_HEADER_SIZE + data_len + IPMX_USB_MAC_SIZE;
    uint8_t *p;

    if (buf_size < total_len)
        return AVERROR(EINVAL);
    if (direction == 1 && transferdata_len > 0 && (uint32_t)transferdata_len != actuallength)
        return AVERROR(EINVAL);

    p = buf;

    AV_WB64(p, 0);
    p += 8;
    AV_WB32(p, 0);
    p += 4;
    *p++ = IPMX_USB_CONTROL_SUBMIT_RETURN;
    {
        uint32_t len17 = total_len & 0x1FFFF;
        *p++ = (len17 >> 16) & 0xFF;
        *p++ = (len17 >> 8) & 0xFF;
        *p++ = len17 & 0xFF;
    }

    *p++ = (seqnum >> 16) & 0xFF;
    *p++ = (seqnum >> 8) & 0xFF;
    *p++ = seqnum & 0xFF;
    *p++ = (endpoint << 4) | (direction & 0x01);
    AV_WB32(p, actuallength);
    p += 4;
    AV_WB32(p, rstatus);
    p += 4;

    if (direction == 1 && actuallength > 0 && transferdata)
        memcpy(p, transferdata, actuallength);
    p += direction == 1 ? actuallength : 0;

    memset(p, 0, IPMX_USB_MAC_SIZE);

    return total_len;
}

int ipmx_usb_build_bulk_submit(uint8_t *buf, int buf_size,
    uint32_t seqnum, uint8_t endpoint, uint8_t direction, uint32_t binterval,
    uint32_t transferlength, const uint8_t *transferdata, int transferdata_len)
{
    int data_len = 12 + (transferdata_len > 0 ? transferdata_len : 0);
    int total_len = IPMX_USB_HEADER_SIZE + data_len + IPMX_USB_MAC_SIZE;
    uint8_t *p;

    if (buf_size < total_len)
        return AVERROR(EINVAL);

    p = buf;

    AV_WB64(p, 0);
    p += 8;
    AV_WB32(p, 0);
    p += 4;
    *p++ = IPMX_USB_BULK_SUBMIT;
    {
        uint32_t len17 = total_len & 0x1FFFF;
        *p++ = (len17 >> 16) & 0xFF;
        *p++ = (len17 >> 8) & 0xFF;
        *p++ = len17 & 0xFF;
    }

    *p++ = (seqnum >> 16) & 0xFF;
    *p++ = (seqnum >> 8) & 0xFF;
    *p++ = seqnum & 0xFF;
    *p++ = (endpoint << 4) | (direction & 0x01);
    AV_WB32(p, binterval);
    p += 4;
    AV_WB32(p, transferlength);
    p += 4;

    if (transferdata_len > 0 && transferdata)
        memcpy(p, transferdata, transferdata_len);
    p += transferdata_len > 0 ? transferdata_len : 0;

    memset(p, 0, IPMX_USB_MAC_SIZE);

    return total_len;
}

int ipmx_usb_build_bulk_submit_return(uint8_t *buf, int buf_size,
    uint32_t seqnum, uint8_t endpoint, uint8_t direction,
    uint32_t actuallength, uint32_t rstatus,
    const uint8_t *transferdata, int transferdata_len)
{
    return ipmx_usb_build_control_submit_return(buf, buf_size,
        seqnum, endpoint, direction, actuallength, rstatus,
        transferdata, transferdata_len);
}

int ipmx_usb_build_interrupt_submit(uint8_t *buf, int buf_size,
    uint32_t seqnum, uint8_t endpoint, uint8_t direction, uint32_t binterval,
    uint32_t transferlength, const uint8_t *transferdata, int transferdata_len)
{
    return ipmx_usb_build_bulk_submit(buf, buf_size,
        seqnum, endpoint, direction, binterval, transferlength,
        transferdata, transferdata_len);
}

int ipmx_usb_build_interrupt_submit_return(uint8_t *buf, int buf_size,
    uint32_t seqnum, uint8_t endpoint, uint8_t direction,
    uint32_t actuallength, uint32_t rstatus,
    const uint8_t *transferdata, int transferdata_len)
{
    return ipmx_usb_build_control_submit_return(buf, buf_size,
        seqnum, endpoint, direction, actuallength, rstatus,
        transferdata, transferdata_len);
}

int ipmx_usb_build_iso_submit(uint8_t *buf, int buf_size,
    uint32_t seqnum, uint8_t endpoint, uint8_t direction, uint32_t binterval,
    uint8_t asap, uint32_t startframe, uint32_t num_packets,
    const uint16_t *isolengths)
{
    int data_len, total_len;
    uint8_t *p;
    uint32_t a_sf;

    if (num_packets > IPMX_USB_MAX_ISO_PACKETS)
        return AVERROR(EINVAL);

    data_len = 16 + num_packets * 2;
    total_len = IPMX_USB_HEADER_SIZE + data_len + IPMX_USB_MAC_SIZE;
    if (buf_size < total_len)
        return AVERROR(EINVAL);

    p = buf;

    AV_WB64(p, 0);
    p += 8;
    AV_WB32(p, 0);
    p += 4;
    *p++ = IPMX_USB_ISOCHRONOUS_SUBMIT;
    {
        uint32_t len17 = total_len & 0x1FFFF;
        *p++ = (len17 >> 16) & 0xFF;
        *p++ = (len17 >> 8) & 0xFF;
        *p++ = len17 & 0xFF;
    }

    *p++ = (seqnum >> 16) & 0xFF;
    *p++ = (seqnum >> 8) & 0xFF;
    *p++ = seqnum & 0xFF;
    *p++ = (endpoint << 4) | (direction & 0x01);
    AV_WB32(p, binterval);
    p += 4;

    a_sf = ((uint32_t)(asap & 0x01) << 31) | (startframe & 0x7FFFFFFF);
    AV_WB32(p, a_sf);
    p += 4;
    AV_WB32(p, num_packets);
    p += 4;

    for (uint32_t i = 0; i < num_packets; i++) {
        AV_WB16(p, isolengths ? isolengths[i] : 0);
        p += 2;
    }

    memset(p, 0, IPMX_USB_MAC_SIZE);

    return total_len;
}

int ipmx_usb_build_iso_submit_return(uint8_t *buf, int buf_size,
    uint32_t seqnum, uint8_t endpoint, uint8_t direction,
    uint32_t startframe, uint32_t errorcount, uint32_t num_packets,
    const IpmxUsbIsoPacket *iso_packets,
    const uint8_t *transferdata, int transferdata_len)
{
    int desc_len, data_len, total_len;
    uint8_t *p;

    if (num_packets > IPMX_USB_MAX_ISO_PACKETS)
        return AVERROR(EINVAL);

    desc_len = num_packets * 6;
    data_len = 16 + desc_len + (transferdata_len > 0 ? transferdata_len : 0);
    total_len = IPMX_USB_HEADER_SIZE + data_len + IPMX_USB_MAC_SIZE;
    if (buf_size < total_len)
        return AVERROR(EINVAL);

    p = buf;

    AV_WB64(p, 0);
    p += 8;
    AV_WB32(p, 0);
    p += 4;
    *p++ = IPMX_USB_ISOCHRONOUS_SUBMIT_RETURN;
    {
        uint32_t len17 = total_len & 0x1FFFF;
        *p++ = (len17 >> 16) & 0xFF;
        *p++ = (len17 >> 8) & 0xFF;
        *p++ = len17 & 0xFF;
    }

    *p++ = (seqnum >> 16) & 0xFF;
    *p++ = (seqnum >> 8) & 0xFF;
    *p++ = seqnum & 0xFF;
    *p++ = (endpoint << 4) | (direction & 0x01);
    AV_WB32(p, startframe);
    p += 4;
    AV_WB32(p, errorcount);
    p += 4;
    AV_WB32(p, num_packets);
    p += 4;

    for (uint32_t i = 0; i < num_packets; i++) {
        AV_WB16(p, iso_packets ? iso_packets[i].actuallength : 0);
        p += 2;
        AV_WB32(p, iso_packets ? iso_packets[i].isostatus : 0);
        p += 4;
    }

    if (transferdata_len > 0 && transferdata)
        memcpy(p, transferdata, transferdata_len);
    p += transferdata_len > 0 ? transferdata_len : 0;

    memset(p, 0, IPMX_USB_MAC_SIZE);

    return total_len;
}

int ipmx_usb_build_cancel_submit(uint8_t *buf, int buf_size,
    uint32_t seqnum, uint8_t endpoint, uint8_t direction)
{
    uint8_t data[4];
    data[0] = (seqnum >> 16) & 0xFF;
    data[1] = (seqnum >> 8) & 0xFF;
    data[2] = seqnum & 0xFF;
    data[3] = (endpoint << 4) | (direction & 0x01);
    return build_message(buf, buf_size, IPMX_USB_CANCEL_SUBMIT, data, 4);
}

int ipmx_usb_build_cancel_submit_return(uint8_t *buf, int buf_size,
    uint32_t seqnum, uint8_t endpoint, uint8_t direction, uint32_t rstatus)
{
    uint8_t data[8];
    uint8_t *p = data;
    *p++ = (seqnum >> 16) & 0xFF;
    *p++ = (seqnum >> 8) & 0xFF;
    *p++ = seqnum & 0xFF;
    *p++ = (endpoint << 4) | (direction & 0x01);
    AV_WB32(p, rstatus);
    return build_message(buf, buf_size, IPMX_USB_CANCEL_SUBMIT_RETURN, data, 8);
}
