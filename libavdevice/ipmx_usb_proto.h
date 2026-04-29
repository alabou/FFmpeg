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
 * All field sizes and byte offsets follow VSF TR-10-14 (2024-09-24).
 * Big-endian byte order throughout.
 */

#ifndef AVFORMAT_IPMX_USB_PROTO_H
#define AVFORMAT_IPMX_USB_PROTO_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------------------
 * Message type enum (Table 1, Section 9)
 * --------------------------------------------------------------------------- */

#define IPMX_USB_SENDER_CONNECTION_INFO        0x00
#define IPMX_USB_SENDER_CONNECTION_STATUS      0x01
#define IPMX_USB_HEARTBEAT                      0x02
#define IPMX_USB_VENDOR_SPECIFIC_INFO          0x04
#define IPMX_USB_VENDOR_SPECIFIC_QUERY         0x06
#define IPMX_USB_VENDOR_SPECIFIC_QUERY_RETURN   0x07
#define IPMX_USB_WAKEUP_CONTROL                 0x11
#define IPMX_USB_ENTER_SLEEP                    0x12
#define IPMX_USB_RESUME_OPERATION               0x14
#define IPMX_USB_STREAM_INFO                    0x80
#define IPMX_USB_STREAM_STATUS                  0x81
#define IPMX_USB_STREAM_RESET_RETURN            0x82
#define IPMX_USB_STREAM_RESET                   0x83
#define IPMX_USB_CONTROL_SUBMIT_RETURN          0x90
#define IPMX_USB_CONTROL_SUBMIT                 0x91
#define IPMX_USB_BULK_SUBMIT_RETURN             0x92
#define IPMX_USB_BULK_SUBMIT                    0x93
#define IPMX_USB_INTERRUPT_SUBMIT_RETURN        0x94
#define IPMX_USB_INTERRUPT_SUBMIT               0x95
#define IPMX_USB_ISOCHRONOUS_SUBMIT_RETURN      0x96
#define IPMX_USB_ISOCHRONOUS_SUBMIT             0x97
#define IPMX_USB_CANCEL_SUBMIT_RETURN           0x98
#define IPMX_USB_CANCEL_SUBMIT                  0x99

/* ---------------------------------------------------------------------------
 * USB speed enum (Section 9.7, Table 7)
 * --------------------------------------------------------------------------- */

#define IPMX_USB_SPEED_UNKNOWN  0
#define IPMX_USB_SPEED_LOW      1
#define IPMX_USB_SPEED_FULL     2
#define IPMX_USB_SPEED_HIGH     3

/* ---------------------------------------------------------------------------
 * Constants
 * --------------------------------------------------------------------------- */

#define IPMX_USB_HEADER_SIZE      16
#define IPMX_USB_MAC_SIZE          8
#define IPMX_USB_MIN_LENGTH        24
#define IPMX_USB_MAX_LENGTH        131047
#define IPMX_USB_MAX_ISO_PACKETS   256

/* ---------------------------------------------------------------------------
 * Message header
 * --------------------------------------------------------------------------- */

typedef struct IpmxUsbMsgHeader {
    uint64_t ctr;
    uint32_t key_version;
    uint8_t  msg_type;
    uint32_t length;        /* 17-bit LENGTH field */
    uint32_t data_len;      /* length - 24 */
} IpmxUsbMsgHeader;

/* ---------------------------------------------------------------------------
 * ISO packet descriptor (Table 15)
 * --------------------------------------------------------------------------- */

typedef struct IpmxUsbIsoPacket {
    uint16_t actuallength;
    uint32_t isostatus;
} IpmxUsbIsoPacket;

/* ---------------------------------------------------------------------------
 * Payload structs (one per message type)
 * --------------------------------------------------------------------------- */

typedef struct IpmxUsbSenderConnectionInfo {
    uint8_t  cid[3];
    char     serial_number[128];
    int      serial_len;
} IpmxUsbSenderConnectionInfo;

typedef struct IpmxUsbSenderConnectionStatus {
    uint8_t  cid[3];
    char     serial_number[128];
    int      serial_len;
    uint8_t  hbeat_index;
    uint16_t data_port;
} IpmxUsbSenderConnectionStatus;

typedef struct IpmxUsbStreamInfo {
    uint8_t  substreamid;
    uint8_t  usbspeed;
    char     busid[64];
    int      busid_len;
} IpmxUsbStreamInfo;

typedef struct IpmxUsbStreamStatus {
    uint8_t cstatus;
} IpmxUsbStreamStatus;

typedef struct IpmxUsbControlSubmit {
    uint32_t       seqnum;
    uint8_t        endpoint;
    uint8_t        direction;
    uint32_t       binterval;
    uint32_t       transferlength;
    uint8_t        usbdevreq[8];
    const uint8_t *transferdata;
    int            transferdata_len;
} IpmxUsbControlSubmit;

typedef struct IpmxUsbControlSubmitReturn {
    uint32_t       seqnum;
    uint8_t        endpoint;
    uint8_t        direction;
    uint32_t       actuallength;
    uint32_t       rstatus;
    const uint8_t *transferdata;
    int            transferdata_len;
} IpmxUsbControlSubmitReturn;

typedef struct IpmxUsbBulkSubmit {
    uint32_t       seqnum;
    uint8_t        endpoint;
    uint8_t        direction;
    uint32_t       binterval;
    uint32_t       transferlength;
    const uint8_t *transferdata;
    int            transferdata_len;
} IpmxUsbBulkSubmit;

typedef struct IpmxUsbBulkSubmitReturn {
    uint32_t       seqnum;
    uint8_t        endpoint;
    uint8_t        direction;
    uint32_t       actuallength;
    uint32_t       rstatus;
    const uint8_t *transferdata;
    int            transferdata_len;
} IpmxUsbBulkSubmitReturn;

typedef struct IpmxUsbInterruptSubmit {
    uint32_t       seqnum;
    uint8_t        endpoint;
    uint8_t        direction;
    uint32_t       binterval;
    uint32_t       transferlength;
    const uint8_t *transferdata;
    int            transferdata_len;
} IpmxUsbInterruptSubmit;

typedef struct IpmxUsbInterruptSubmitReturn {
    uint32_t       seqnum;
    uint8_t        endpoint;
    uint8_t        direction;
    uint32_t       actuallength;
    uint32_t       rstatus;
    const uint8_t *transferdata;
    int            transferdata_len;
} IpmxUsbInterruptSubmitReturn;

typedef struct IpmxUsbIsoSubmit {
    uint32_t seqnum;
    uint8_t  endpoint;
    uint8_t  direction;
    uint32_t binterval;
    uint8_t  asap;
    uint32_t startframe;
    uint32_t num_packets;
    uint16_t isolengths[IPMX_USB_MAX_ISO_PACKETS];
} IpmxUsbIsoSubmit;

typedef struct IpmxUsbIsoSubmitReturn {
    uint32_t            seqnum;
    uint8_t             endpoint;
    uint8_t             direction;
    uint32_t            startframe;
    uint32_t            errorcount;
    uint32_t            num_packets;
    IpmxUsbIsoPacket    iso_packets[IPMX_USB_MAX_ISO_PACKETS];
    const uint8_t      *transferdata;
    int                 transferdata_len;
} IpmxUsbIsoSubmitReturn;

typedef struct IpmxUsbCancelSubmit {
    uint32_t seqnum;
    uint8_t  endpoint;
    uint8_t  direction;
} IpmxUsbCancelSubmit;

typedef struct IpmxUsbCancelSubmitReturn {
    uint32_t seqnum;
    uint8_t  endpoint;
    uint8_t  direction;
    uint32_t rstatus;
} IpmxUsbCancelSubmitReturn;

/* ---------------------------------------------------------------------------
 * Parsed message with type-specific payload union
 * --------------------------------------------------------------------------- */

typedef struct IpmxUsbMsg {
    IpmxUsbMsgHeader hdr;
    uint8_t          mac[8];
    const uint8_t   *raw;
    int              raw_len;
    union {
        IpmxUsbSenderConnectionInfo   sender_connection_info;
        IpmxUsbSenderConnectionStatus sender_connection_status;
        IpmxUsbStreamInfo             usb_stream_info;
        IpmxUsbStreamStatus            usb_stream_status;
        IpmxUsbControlSubmit           control_submit;
        IpmxUsbControlSubmitReturn     control_submit_return;
        IpmxUsbBulkSubmit              bulk_submit;
        IpmxUsbBulkSubmitReturn        bulk_submit_return;
        IpmxUsbInterruptSubmit         interrupt_submit;
        IpmxUsbInterruptSubmitReturn   interrupt_submit_return;
        IpmxUsbIsoSubmit               iso_submit;
        IpmxUsbIsoSubmitReturn         iso_submit_return;
        IpmxUsbCancelSubmit            cancel_submit;
        IpmxUsbCancelSubmitReturn      cancel_submit_return;
    } payload;
} IpmxUsbMsg;

/* ---------------------------------------------------------------------------
 * Parsing
 * --------------------------------------------------------------------------- */

/**
 * Peek at the LENGTH field of the message at buf without full parsing.
 * Returns 0 if insufficient bytes; otherwise sets *out_length and returns 1.
 */
int ipmx_usb_peek_length(const uint8_t *buf, int buf_len, uint32_t *out_length);

/**
 * Parse one complete IPMX USB message from buf into msg.
 * Returns number of bytes consumed (>0) on success, or negative on error.
 * msg->raw points into buf; valid only while buf is valid.
 */
int ipmx_usb_parse_one(const uint8_t *buf, int buf_len, IpmxUsbMsg *msg);

/* ---------------------------------------------------------------------------
 * Message builders (return bytes written, or -1 on error)
 * --------------------------------------------------------------------------- */

int ipmx_usb_build_sender_connection_info(uint8_t *buf, int buf_size,
    const uint8_t *cid, const char *serial);

int ipmx_usb_build_sender_connection_status(uint8_t *buf, int buf_size,
    const uint8_t *cid, const char *serial, uint8_t hbeat_index, uint16_t data_port);

int ipmx_usb_build_heartbeat(uint8_t *buf, int buf_size);

int ipmx_usb_build_usb_stream_info(uint8_t *buf, int buf_size,
    uint16_t substreamid, uint8_t usbspeed, const char *busid);

int ipmx_usb_build_usb_stream_status(uint8_t *buf, int buf_size, uint8_t cstatus);

int ipmx_usb_build_control_submit(uint8_t *buf, int buf_size,
    uint32_t seqnum, uint8_t endpoint, uint8_t direction, uint32_t binterval,
    uint32_t transferlength, const uint8_t *usbdevreq,
    const uint8_t *transferdata, int transferdata_len);

int ipmx_usb_build_control_submit_return(uint8_t *buf, int buf_size,
    uint32_t seqnum, uint8_t endpoint, uint8_t direction,
    uint32_t actuallength, uint32_t rstatus,
    const uint8_t *transferdata, int transferdata_len);

int ipmx_usb_build_bulk_submit(uint8_t *buf, int buf_size,
    uint32_t seqnum, uint8_t endpoint, uint8_t direction, uint32_t binterval,
    uint32_t transferlength, const uint8_t *transferdata, int transferdata_len);

int ipmx_usb_build_bulk_submit_return(uint8_t *buf, int buf_size,
    uint32_t seqnum, uint8_t endpoint, uint8_t direction,
    uint32_t actuallength, uint32_t rstatus,
    const uint8_t *transferdata, int transferdata_len);

int ipmx_usb_build_interrupt_submit(uint8_t *buf, int buf_size,
    uint32_t seqnum, uint8_t endpoint, uint8_t direction, uint32_t binterval,
    uint32_t transferlength, const uint8_t *transferdata, int transferdata_len);

int ipmx_usb_build_interrupt_submit_return(uint8_t *buf, int buf_size,
    uint32_t seqnum, uint8_t endpoint, uint8_t direction,
    uint32_t actuallength, uint32_t rstatus,
    const uint8_t *transferdata, int transferdata_len);

int ipmx_usb_build_iso_submit(uint8_t *buf, int buf_size,
    uint32_t seqnum, uint8_t endpoint, uint8_t direction, uint32_t binterval,
    uint8_t asap, uint32_t startframe, uint32_t num_packets,
    const uint16_t *isolengths);

int ipmx_usb_build_iso_submit_return(uint8_t *buf, int buf_size,
    uint32_t seqnum, uint8_t endpoint, uint8_t direction,
    uint32_t startframe, uint32_t errorcount, uint32_t num_packets,
    const IpmxUsbIsoPacket *iso_packets,
    const uint8_t *transferdata, int transferdata_len);

int ipmx_usb_build_cancel_submit(uint8_t *buf, int buf_size,
    uint32_t seqnum, uint8_t endpoint, uint8_t direction);

int ipmx_usb_build_cancel_submit_return(uint8_t *buf, int buf_size,
    uint32_t seqnum, uint8_t endpoint, uint8_t direction, uint32_t rstatus);

#ifdef __cplusplus
}
#endif

#endif /* AVFORMAT_IPMX_USB_PROTO_H */
