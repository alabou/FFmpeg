/*
 * IPMX USB (TR-10-14) Sender output device for FFmpeg.
 *
 * Listens for an IPMX USB Receiver, negotiates control and data channels,
 * and transmits encoded video/audio frames as USB isochronous payloads.
 *
 * Supports two channel modes controlled by -split_device:
 *   0 (default) = single USB device with interleaved video+audio (one data channel)
 *   1           = separate video and audio USB devices (parallel data channels)
 *
 * Up to SENDER_MAX_DATA_CH (8) data channels can be allocated for future
 * multi-device scenarios (e.g. USB hub with independent devices).
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

#include "config_components.h"

#include <stdatomic.h>

#include <pthread.h>
#include "libavformat/avformat.h"
#include "libavformat/internal.h"
#include "libavformat/mux.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/time.h"
#include "libavutil/mathematics.h"
#include "libavutil/avstring.h"
#include "libavutil/log.h"
#include "libavutil/threadmessage.h"
#include "avdevice.h"

#include "ipmx_usb_proto.h"
#include "ipmx_usb_pep.h"
#include "ipmx_usb_common.h"
#include "ipmx_usb_virt_desc.h"
#include "ipmx_usb_uvc.h"
#include "ipmx_usb_uvc_seg.h"
#include "ipmx_usb_uac_pack.h"
#include "ipmx_usb_quirks.h"
#include "ipmx_timing.h"

#include <poll.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>

#if defined(__x86_64__) || defined(__i386__)
#include <immintrin.h>
#define spin_hint() _mm_pause()
#elif defined(__aarch64__)
#define spin_hint() __asm__ volatile("yield" ::: "memory")
#else
#define spin_hint() ((void)0)
#endif

#include <math.h>

#define SENDER_QUEUE_SIZE          120
#define SENDER_BUILD_BUF_SIZE      131072
#define SENDER_DEFAULT_HEARTBEAT_SEC 47
#define SENDER_CTRL_TIMEOUT_MS     15000
#define SENDER_DATA_TIMEOUT_MS     10000
#define SENDER_DEFAULT_PORT        27502
#define SENDER_ISO_MAX_PACKETS     64
#define SENDER_ISO_BUF_SIZE        (256 * 1024)
#define SENDER_MAX_DATA_CH         8
#define DATA_RECONNECT_DELAY_US    3000000  /* 3 seconds between reconnect attempts */
#define ISO_PACE_SPIN_US           500      /* CPU-spin last 0.5 ms for isochronous send */
#define ISO_PACE_MAX_EARLY_US      500000  /* ignore target if >0.5 s in the future (startup) */
#define DATA_RECONNECT_MAX_RETRIES 10

/* USB standard request types */
#define USB_DIR_IN              0x80
#define USB_REQ_GET_DESCRIPTOR  0x06
#define USB_DT_DEVICE           0x01
#define USB_DT_CONFIG           0x02
#define USB_DT_STRING           0x03

/* Forward declarations for data channel reconnect */
typedef struct SenderDataChannel SenderDataChannel;
static int connect_data_socket(AVFormatContext *s, SenderDataChannel *ch,
                               const char *peer_ip, uint16_t data_port,
                               int ch_idx);
static int handshake_data_channel(AVFormatContext *s, SenderDataChannel *ch,
                                  IpmxUsbPepState *pep, int ch_idx);

/* Queued packet wrapper */
typedef struct SenderQueueMsg {
    AVPacket *pkt;
} SenderQueueMsg;

/* Deferred ISO response — holds a fully-built ISO_SUBMIT_RETURN message
 * that should be sent at a specific wall-clock time.  This decouples
 * pacing from the TCP message loop, allowing independent video and audio
 * pacing on a single interleaved thread. */
typedef struct DeferredIsoResponse {
    uint8_t *buf;
    int      len;
    int64_t  target_us;
    char     media;       /* 'V' or 'A' — for TX logging */
    uint32_t startframe;  /* USB frame number — for TX logging */
    int      valid;
} DeferredIsoResponse;

#define DEFERRED_VIDEO  0
#define DEFERRED_AUDIO  1
#define DEFERRED_SLOTS  2

/* Per-data-channel state — self-contained for threading */
typedef struct SenderDataChannel {
    int      sock;
    uint16_t substreamid;
    uint32_t seqnum;

    uint8_t *recv_buf;
    int      recv_buf_len;
    int      recv_buf_pos;

    uint64_t ctr_s2r;
    uint64_t ctr_r2s;
    IpmxPepForwardProgress fp_r2s;

    pthread_t thread;
    int       thread_started;
    volatile int thread_running;

    /* Media identity for this channel */
    int is_video;
    int is_audio;
    AVThreadMessageQueue *video_q;
    AVThreadMessageQueue *audio_q;

    /* Per-channel descriptors */
    IpmxUsbVirtDescSet    desc_set;
    IpmxUsbVirtDescParams desc_params;

    /* Per-channel media processing */
    IpmxUvcSegmenter uvc_seg;
    IpmxUacPacker    uac_pack;
    AVPacket        *current_video_pkt;

    /* Audio residual buffer: holds leftover PCM that didn't fit in the
     * previous ISO response so it can be sent in the next one. */
    uint8_t *audio_residual;
    int      audio_residual_len;
    int      audio_residual_cap;

    /* Audio cumulative bytes sent (for pacing fallback) */
    int64_t  audio_bytes_sent;

    /* Audio PTS epoch for PTS-anchored pacing (mirrors pts_epoch_us for video) */
    int64_t  audio_pts_epoch_us;
    int      audio_pts_epoch_set;
    int64_t  audio_latest_pts_us;

    /* Shared synthetic SOF clock: emulates the single USB host controller
     * clock that stamps both video and audio in a real device.
     * frame_epoch_us is set once by whichever media arrives first.
     * video_sf_offset / audio_sf_offset align each media to this shared
     * timeline: startframe = offset + media_local_fn. */
    int64_t  frame_epoch_us;
    uint32_t video_sf_offset;
    int      video_sf_offset_set;
    uint32_t audio_sf_offset;
    int      audio_sf_offset_set;

    /* Capture-PTS epoch (microseconds) subtracted from UVC PTS so that
     * the first delivered frame starts near PTS=0 regardless of how
     * long the webcam has been running before a receiver connects. */
    int64_t  pts_epoch_us;
    int      pts_epoch_set;

    /* Isochronous pacing: wall-clock epoch anchored to the first media
     * sample.  For each subsequent ISO response the absolute target
     * send time is  epoch_us + media_offset_us.  The sender coarse-sleeps
     * to (target − SPIN), then CPU-spins the remainder.  Because the
     * target is tied to the media timeline it never accumulates drift;
     * the receiver PLL handles any residual jitter. */
    int64_t  video_epoch_us;
    int64_t  video_frame_count;
    int64_t  audio_epoch_us;

    /* Virtual device state for class-specific request responses */
    uint8_t  probe_commit_data[34];
    int      probe_commit_len;
    uint32_t active_sample_rate;
    uint8_t  uac_version;       /* 1 or 2 */
    uint8_t  clock_source_id;   /* UAC 2.0 Clock Source entity ID */
    uint8_t  uac_endpoint;      /* UAC endpoint address for UAC 1.0 matching */
    uint8_t  vs_interface;      /* UVC VS interface number */
    uint8_t  ac_interface;      /* UAC AC interface number for UAC 2.0 matching */

    /* Deferred ISO responses for interleaved-channel pacing.  On a dedicated
     * channel (only video or only audio) these are unused — pacing blocks
     * inline.  On an interleaved channel the thread builds the response
     * immediately but defers the TCP send until the pacing target so that
     * video and audio are independently paced without head-of-line blocking. */
    DeferredIsoResponse deferred[DEFERRED_SLOTS];

    /* Reconnect info (Sender is the connect() side for data channels) */
    char     peer_ip[64];
    uint16_t data_port;
    int      ch_idx;
} SenderDataChannel;

typedef struct DataChannelThreadArg {
    AVFormatContext    *s;
    SenderDataChannel  *ch;
} DataChannelThreadArg;

typedef struct IpmxUsbSenderContext {
    AVClass *class;

    /* CLI options */
    int   listen_port;
    char *psk_hex;
    char *sdp_path;
    char *serial;
    int   split_device;
    char *pep_mode_str;
    char *pep_iv_str;
    char *pep_key_generator_str;
    char *pep_key_version_str;
    int strict_pep;
    int audio_uac2;
    int video_clock_freq;  /* dwClockFrequency in Hz (0 = 1 MHz default) */

    /* PEP state */
    IpmxUsbPepState pep;

    /* Stream index mapping (-1 = absent) */
    int video_stream_idx;
    int audio_stream_idx;
    AVRational video_tb;
    AVRational audio_tb;

    /* Per-stream packet queues (producer: write_packet, consumer: data threads) */
    AVThreadMessageQueue *video_queue;
    AVThreadMessageQueue *audio_queue;

    pthread_t proto_thread;
    int       proto_thread_started;
    int       stop;

    /* Network */
    int ctrl_listen_sock;
    int ctrl_sock;
    SenderDataChannel data_ch[SENDER_MAX_DATA_CH];
    int               num_data_ch;

    /* Diagnostic counters */
    volatile int64_t diag_wp_video;
    volatile int64_t diag_wp_audio;
    volatile int64_t diag_wp_drop_video;
    volatile int64_t diag_wp_drop_audio;
    volatile int64_t diag_iso_vid_req;
    volatile int64_t diag_iso_vid_data;
    volatile int64_t diag_iso_vid_empty;
    volatile int64_t diag_iso_aud_req;
    volatile int64_t diag_iso_aud_data;
    volatile int64_t diag_iso_aud_empty;
    volatile int64_t diag_vid_deq;
    volatile int64_t diag_aud_deq;
    volatile int64_t diag_iso_send_err;
    FILE *diag_fp;

    /* Heartbeat */
    uint64_t ctrl_ctr_s2r;
    uint64_t ctrl_ctr_r2s;
    IpmxPepForwardProgress ctrl_fp_r2s;

    /* Shared synthetic SOF epoch: emulates the single USB bus clock
     * across all data channels.  Set once (atomic CAS) by whichever
     * channel thread processes the first media sample.  Both split-mode
     * channels read this to produce STARTFRAME values on the same
     * timeline — just like a real USB host controller's SOF counter. */
    atomic_int_fast64_t shared_frame_epoch_us;

    /* Set by the protocol thread once data channels are connected and
     * consuming.  write_packet skips (discards) all packets until this
     * flag is set, so no stale data accumulates in the queues while
     * waiting for a receiver. */
    volatile int data_ch_ready;

    /* Timing instrumentation (IPMX_TIMING=1) — appends to the sender log
     * opened by the local capture module (same process). */
    FILE *timing_fp;
    volatile int64_t timing_enq_vid;
    volatile int64_t timing_enq_aud;
    volatile int64_t timing_deq_vid;
    volatile int64_t timing_deq_aud;
    volatile int64_t timing_tx_vid;
    volatile int64_t timing_tx_aud;
} IpmxUsbSenderContext;

/* ---------------------------------------------------------------------------
 * Options
 * --------------------------------------------------------------------------- */

static const AVOption ipmx_usb_sender_options[] = {
    { "listen_port", "Control port to listen on",
      offsetof(IpmxUsbSenderContext, listen_port),
      AV_OPT_TYPE_INT, {.i64 = SENDER_DEFAULT_PORT}, 1, 65535,
      AV_OPT_FLAG_ENCODING_PARAM },
    { "psk", "Pre-shared key (hex)",
      offsetof(IpmxUsbSenderContext, psk_hex),
      AV_OPT_TYPE_STRING, {.str = NULL}, 0, 0, AV_OPT_FLAG_ENCODING_PARAM },
    { "sdp", "SDP file path",
      offsetof(IpmxUsbSenderContext, sdp_path),
      AV_OPT_TYPE_STRING, {.str = NULL}, 0, 0, AV_OPT_FLAG_ENCODING_PARAM },
    { "serial", "Device serial number",
      offsetof(IpmxUsbSenderContext, serial),
      AV_OPT_TYPE_STRING, {.str = "FFMPEG-SENDER-01"}, 0, 0,
      AV_OPT_FLAG_ENCODING_PARAM },
    { "split_device", "Split video and audio into separate USB devices/data channels",
      offsetof(IpmxUsbSenderContext, split_device),
      AV_OPT_TYPE_BOOL, {.i64 = 0}, 0, 1, AV_OPT_FLAG_ENCODING_PARAM },
    { "pep_mode", "PEP mode override",
      offsetof(IpmxUsbSenderContext, pep_mode_str),
      AV_OPT_TYPE_STRING, {.str = NULL}, 0, 0, AV_OPT_FLAG_ENCODING_PARAM },
    { "pep_iv", "PEP base IV override (hex)",
      offsetof(IpmxUsbSenderContext, pep_iv_str),
      AV_OPT_TYPE_STRING, {.str = NULL}, 0, 0, AV_OPT_FLAG_ENCODING_PARAM },
    { "pep_key_generator", "PEP key_generator override (hex)",
      offsetof(IpmxUsbSenderContext, pep_key_generator_str),
      AV_OPT_TYPE_STRING, {.str = NULL}, 0, 0, AV_OPT_FLAG_ENCODING_PARAM },
    { "pep_key_version", "PEP sender key_version override (hex)",
      offsetof(IpmxUsbSenderContext, pep_key_version_str),
      AV_OPT_TYPE_STRING, {.str = NULL}, 0, 0, AV_OPT_FLAG_ENCODING_PARAM },
    { "strict_pep", "Strict PEP forward-progress enforcement (close channel on violation)",
      offsetof(IpmxUsbSenderContext, strict_pep),
      AV_OPT_TYPE_BOOL, {.i64 = 0}, 0, 1, AV_OPT_FLAG_ENCODING_PARAM },
    { "audio_uac2", "Advertise UAC 2.0 descriptors (default: UAC 1.0)",
      offsetof(IpmxUsbSenderContext, audio_uac2),
      AV_OPT_TYPE_BOOL, {.i64 = 0}, 0, 1, AV_OPT_FLAG_ENCODING_PARAM },
    { "video_clock_freq", "UVC dwClockFrequency in Hz (0 = 1 MHz default)",
      offsetof(IpmxUsbSenderContext, video_clock_freq),
      AV_OPT_TYPE_INT, {.i64 = 0}, 0, INT_MAX, AV_OPT_FLAG_ENCODING_PARAM },
    { NULL },
};

static const AVClass ipmx_usb_sender_class = {
    .class_name = "ipmx_usb_sender outdev",
    .item_name  = av_default_item_name,
    .option     = ipmx_usb_sender_options,
    .version    = LIBAVUTIL_VERSION_INT,
    .category   = AV_CLASS_CATEGORY_DEVICE_VIDEO_OUTPUT,
};

/* ---------------------------------------------------------------------------
 * Queue free callback
 * --------------------------------------------------------------------------- */

static void queue_msg_free(void *msg)
{
    SenderQueueMsg *m = msg;
    if (m->pkt)
        av_packet_free(&m->pkt);
}

/* ---------------------------------------------------------------------------
 * Initialize virtual device state for class-specific request responses.
 * Called after ipmx_usb_virt_desc_build() populates desc_set/desc_params.
 * --------------------------------------------------------------------------- */

static void init_class_request_state(SenderDataChannel *ch)
{
    /* UVC: build default probe/commit data */
    ch->probe_commit_len = ipmx_uvc_build_probe_control(
        ch->probe_commit_data, sizeof(ch->probe_commit_data),
        1, 1,
        ch->desc_params.video_fps > 0
            ? 10000000U / ch->desc_params.video_fps : 333333);
    if (ch->probe_commit_len < 0)
        ch->probe_commit_len = 0;

    ch->vs_interface = ch->desc_set.video_vs_iface;

    /* UAC */
    ch->active_sample_rate = ch->desc_params.audio_sample_rate;
    ch->uac_version = ch->desc_params.audio_uac2 ? 2 : 1;
    ch->uac_endpoint = ch->desc_set.audio_ep & 0x0F;
    ch->ac_interface = 0xFF;
    ch->clock_source_id = 0;

    if (ch->desc_params.audio_uac2) {
        const uint8_t *d = ch->desc_set.config_desc;
        int dlen = ch->desc_set.config_desc_len;
        int pos = 0;
        ch->clock_source_id = 0x09;
        /* Scan for AC interface number in the generated config descriptor */
        while (pos + 2 <= dlen) {
            uint8_t bLen = d[pos];
            if (bLen < 2 || pos + bLen > dlen) break;
            if (d[pos + 1] == 0x04 && bLen >= 9 &&
                d[pos + 5] == 0x01 && d[pos + 6] == 0x01) {
                ch->ac_interface = d[pos + 2];
                break;
            }
            pos += bLen;
        }
    }
}

/* ---------------------------------------------------------------------------
 * Handle USB CONTROL_SUBMIT: serve virtual descriptors and respond to
 * class-specific UVC/UAC requests for cross-vendor interoperability.
 * --------------------------------------------------------------------------- */

static int handle_control_submit(AVFormatContext *s,
                                 SenderDataChannel *ch,
                                 const IpmxUsbControlSubmit *ctrl,
                                 IpmxUsbPepState *pep)
{
    uint8_t build_buf[SENDER_BUILD_BUF_SIZE];
    const uint8_t *req = ctrl->usbdevreq;
    uint8_t bmRequestType = req[0];
    uint8_t bRequest = req[1];
    uint16_t wValue  = req[2] | (req[3] << 8);
    uint16_t wIndex  = req[4] | (req[5] << 8);
    uint16_t wLength = req[6] | (req[7] << 8);
    uint8_t resp[512];
    int resp_len = 0;
    int len;

    uint8_t req_type = bmRequestType & 0x60;  /* bits 6:5 — type field */
    uint8_t req_dir  = bmRequestType & 0x80;  /* bit 7 — direction */

    if (req_dir && bRequest == USB_REQ_GET_DESCRIPTOR && req_type == 0x00) {
        /* --- Standard Device request: GET_DESCRIPTOR --- */
        uint8_t desc_type = (uint8_t)(wValue >> 8);
        uint8_t desc_idx  = (uint8_t)(wValue & 0xFF);

        switch (desc_type) {
        case USB_DT_DEVICE:
            resp_len = 18;
            if (resp_len > (int)wLength) resp_len = wLength;
            memcpy(resp, ch->desc_set.device_desc, resp_len);
            break;
        case USB_DT_CONFIG:
            resp_len = ch->desc_set.config_desc_len;
            if (resp_len > (int)wLength) resp_len = wLength;
            memcpy(resp, ch->desc_set.config_desc, resp_len);
            break;
        case USB_DT_STRING:
            resp_len = ipmx_usb_virt_desc_string(
                resp, sizeof(resp), desc_idx, &ch->desc_params);
            if (resp_len < 0) resp_len = 0;
            if (resp_len > (int)wLength) resp_len = wLength;
            break;
        default:
            resp_len = 0;
            break;
        }
    } else if (req_type == 0x00 && bRequest == 0x0B) {
        /* --- Standard Interface request: SET_INTERFACE --- */
        resp_len = 0;
    } else if (req_type == 0x20) {
        /* --- Class-specific request --- */
        uint8_t req_recip = bmRequestType & 0x1F;
        uint8_t cs = (uint8_t)(wValue >> 8);

        if (req_recip == 0x01 && (wIndex & 0xFF) == ch->vs_interface) {
            /* UVC class request targeting VS interface */
            if (cs == 0x01 || cs == 0x02) {
                /* PROBE_CONTROL (0x01) or COMMIT_CONTROL (0x02) */
                if (bRequest == 0x01) {
                    /* SET_CUR: accept the probe/commit data from host */
                    if (ctrl->transferlength > 0 && ctrl->transferlength <= 34 &&
                        ctrl->transferdata)
                        memcpy(ch->probe_commit_data, ctrl->transferdata,
                               ctrl->transferlength);
                    resp_len = 0;
                } else if (bRequest == 0x81) {
                    /* GET_CUR: return our probe/commit data */
                    resp_len = ch->probe_commit_len;
                    if (resp_len > (int)wLength) resp_len = wLength;
                    memcpy(resp, ch->probe_commit_data, resp_len);
                }
            }
        } else if (req_recip == 0x02 &&
                   ch->uac_version == 1 &&
                   (wIndex & 0xFF) == ch->uac_endpoint) {
            /* UAC 1.0 class request targeting audio endpoint */
            if (cs == 0x01) {
                /* SAMPLING_FREQ_CONTROL */
                if (bRequest == 0x01) {
                    /* SET_CUR: accept new rate */
                    if (ctrl->transferlength >= 3 && ctrl->transferdata) {
                        ch->active_sample_rate =
                            ctrl->transferdata[0] |
                            (ctrl->transferdata[1] << 8) |
                            (ctrl->transferdata[2] << 16);
                    }
                    resp_len = 0;
                } else if (bRequest == 0x81) {
                    /* GET_CUR: return 3-byte LE rate */
                    uint32_t r = ch->active_sample_rate;
                    resp[0] = (uint8_t)(r);
                    resp[1] = (uint8_t)(r >> 8);
                    resp[2] = (uint8_t)(r >> 16);
                    resp_len = 3;
                    if (resp_len > (int)wLength) resp_len = wLength;
                }
            }
        } else if (req_recip == 0x01 &&
                   ch->uac_version == 2 &&
                   (wIndex & 0xFF) == ch->clock_source_id &&
                   ((wIndex >> 8) & 0xFF) == ch->ac_interface) {
            /* UAC 2.0 class request targeting Clock Source entity */
            if (cs == 0x01) {
                /* SAM_FREQ_CONTROL */
                if (bRequest == 0x02) {
                    /* GET_RANGE: return single sub-range */
                    uint32_t r = ch->active_sample_rate;
                    /* wNumSubRanges(2) + {dMIN(4), dMAX(4), dRES(4)} = 14 bytes */
                    resp[0]  = 0x01; resp[1]  = 0x00;   /* wNumSubRanges = 1 */
                    resp[2]  = (uint8_t)(r);
                    resp[3]  = (uint8_t)(r >> 8);
                    resp[4]  = (uint8_t)(r >> 16);
                    resp[5]  = (uint8_t)(r >> 24);       /* dMIN */
                    resp[6]  = resp[2];
                    resp[7]  = resp[3];
                    resp[8]  = resp[4];
                    resp[9]  = resp[5];                  /* dMAX = dMIN */
                    resp[10] = 0; resp[11] = 0;
                    resp[12] = 0; resp[13] = 0;          /* dRES = 0 */
                    resp_len = 14;
                    if (resp_len > (int)wLength) resp_len = wLength;
                } else if (bRequest == 0x81 || bRequest == 0x01) {
                    if (req_dir) {
                        /* GET_CUR: return 4-byte LE rate */
                        uint32_t r = ch->active_sample_rate;
                        resp[0] = (uint8_t)(r);
                        resp[1] = (uint8_t)(r >> 8);
                        resp[2] = (uint8_t)(r >> 16);
                        resp[3] = (uint8_t)(r >> 24);
                        resp_len = 4;
                        if (resp_len > (int)wLength) resp_len = wLength;
                    } else {
                        /* SET_CUR: accept new rate */
                        if (ctrl->transferlength >= 4 && ctrl->transferdata) {
                            ch->active_sample_rate =
                                ctrl->transferdata[0] |
                                (ctrl->transferdata[1] << 8) |
                                (ctrl->transferdata[2] << 16) |
                                (ctrl->transferdata[3] << 24);
                        }
                        resp_len = 0;
                    }
                }
            }
        }
        /* Unknown class request: fall through with resp_len = 0 (ACK) */
    }
    /* Any other request type: ACK with zero-length data */

    len = ipmx_usb_build_control_submit_return(
        build_buf, sizeof(build_buf),
        ctrl->seqnum, ctrl->endpoint, ctrl->direction,
        resp_len, 0, resp, resp_len);

    if (len < 0)
        return AVERROR_EXIT;

    return ipmx_usb_send_message(s, pep, ch->sock,
                                  ch->substreamid,
                                  &ch->ctr_s2r, build_buf, len);
}

/* ---------------------------------------------------------------------------
 * Handle USB INTERRUPT_SUBMIT: return idle HID report
 * --------------------------------------------------------------------------- */

static int handle_interrupt_submit(AVFormatContext *s,
                                   SenderDataChannel *ch,
                                   const IpmxUsbInterruptSubmit *intr,
                                   IpmxUsbPepState *pep)
{
    uint8_t build_buf[256];
    uint8_t idle_report[8] = {0};
    int len;

    len = ipmx_usb_build_interrupt_submit_return(
        build_buf, sizeof(build_buf),
        intr->seqnum, intr->endpoint, intr->direction,
        sizeof(idle_report), 0, idle_report, sizeof(idle_report));
    if (len < 0)
        return AVERROR_EXIT;

    return ipmx_usb_send_message(s, pep, ch->sock,
                                  ch->substreamid,
                                  &ch->ctr_s2r, build_buf, len);
}

/* ---------------------------------------------------------------------------
 * Handle USB ISO_SUBMIT: dequeue AVPacket, segment/pack, send ISO_SUBMIT_RETURN
 *
 * When the channel owns a single media type (split_device=1), uses blocking
 * dequeue so the thread sleeps until data arrives — zero empty round-trips.
 * When interleaved (split_device=0), uses non-blocking dequeue to avoid
 * stalling the other media type.
 * --------------------------------------------------------------------------- */

/*
 * Derive the USB frame number for STARTFRAME.
 * Priority:
 *   1. Real frame_number from source (SOF-derived or wall-clock from local capture)
 *   2. Device PTS / 1000 when PTS is present but no SOF
 *   3. Wall-clock synthesis for file/pipe input
 */
static uint32_t get_startframe(SenderDataChannel *ch, const AVPacket *pkt,
                               AVRational tb)
{
    uint32_t fn = ipmx_timing_read_frame_number(pkt);
    if (fn)
        return fn;

    {
        uint32_t dev_pts;
        if (ipmx_timing_read_device_pts(pkt, &dev_pts))
            return dev_pts / 1000;
    }

    /* Synthetic source: derive from PTS, offset to the shared SOF
     * epoch so video and audio produce consistent raw_diff values
     * in the receiver's PLL — just like a real USB device where both
     * media share one SOF clock. */
    if (pkt->pts != AV_NOPTS_VALUE && ch->pts_epoch_set) {
        int64_t pts_us = av_rescale_q(pkt->pts, tb, (AVRational){1, 1000000});
        return ch->video_sf_offset +
               (uint32_t)((pts_us - ch->pts_epoch_us) / 1000);
    }

    {
        int64_t now = av_gettime_relative();
        if (!ch->frame_epoch_us)
            ch->frame_epoch_us = now;
        return (uint32_t)((now - ch->frame_epoch_us) / 1000);
    }
}

/*
 * Sleep until an absolute wall-clock target, coarse-sleeping all but the
 * last ISO_PACE_SPIN_US then busy-spinning for precision.  If target is
 * in the past or too far in the future, return immediately.
 */
static void iso_pace_wait(int64_t target_us)
{
    int64_t now = av_gettime_relative();
    int64_t wait = target_us - now;

    if (wait <= 0 || wait > ISO_PACE_MAX_EARLY_US)
        return;

    if (wait > ISO_PACE_SPIN_US)
        av_usleep((unsigned)(wait - ISO_PACE_SPIN_US));

    while (av_gettime_relative() < target_us)
        spin_hint();
}

/*
 * Send any deferred ISO responses whose target time has arrived.
 * Returns the number of responses sent.
 */
static int flush_deferred_iso(AVFormatContext *s, SenderDataChannel *ch,
                              IpmxUsbPepState *pep)
{
    IpmxUsbSenderContext *ctx = s->priv_data;
    int sent = 0;
    int i;

    for (i = 0; i < DEFERRED_SLOTS; i++) {
        DeferredIsoResponse *d = &ch->deferred[i];
        int64_t now;
        int ret;
        if (!d->valid)
            continue;
        now = av_gettime_relative();
        if (now < d->target_us && d->target_us - now <= ISO_PACE_MAX_EARLY_US)
            continue;
        /* Target reached (or far in future = startup) — send now */
        iso_pace_wait(d->target_us);
        {
            int64_t tx_us = av_gettime_relative();
            ret = ipmx_usb_send_message(s, pep, ch->sock,
                                        ch->substreamid,
                                        &ch->ctr_s2r, d->buf, d->len);
            if (ret < 0)
                ctx->diag_iso_send_err++;
            if (ctx->timing_fp) {
                    int64_t seq = (d->media == 'V') ? ctx->timing_tx_vid++
                                                     : ctx->timing_tx_aud++;
                    fprintf(ctx->timing_fp,
                            "%lld\tTX\t%c\t%lld\t1=%ld\tSF=%u\n",
                            (long long)tx_us, d->media,
                            (long long)seq, d->target_us, (unsigned)d->startframe);
                }
        }
        av_free(d->buf);
        d->buf   = NULL;
        d->valid = 0;
        sent++;
    }
    return sent;
}

/*
 * Compute the smallest positive wait (in µs) until the next deferred
 * response is due.  Returns 0 if nothing is deferred.
 */
static int64_t deferred_wait_us(const SenderDataChannel *ch)
{
    int64_t earliest = 0;
    int64_t wait = 0;
    int i;

    for (i = 0; i < DEFERRED_SLOTS; i++) {
        const DeferredIsoResponse *d = &ch->deferred[i];
        if (!d->valid)
            continue;
        if (earliest == 0 || d->target_us < earliest)
            earliest = d->target_us;
    }
    if (!earliest)
        return 0;
    wait = earliest - av_gettime_relative();
    return (wait > 0) ? wait : 0;
}

static int handle_iso_submit(AVFormatContext *s,
                             SenderDataChannel *ch,
                             const IpmxUsbIsoSubmit *iso,
                             IpmxUsbPepState *pep)
{
    IpmxUsbSenderContext *ctx = s->priv_data;
    uint8_t endpoint = iso->endpoint;
    uint8_t *build_buf = NULL;
    uint8_t *iso_data_buf = NULL;
    IpmxUsbIsoPacket iso_descs[SENDER_ISO_MAX_PACKETS];
    int total_transfer_len = 0;
    int num_pkts = 0;
    int len, ret;
    uint32_t max_pkts = iso->num_packets;
    int dequeue_flags;
    int is_video, is_audio;
    int ch_interleaved;
    uint32_t startframe = 0;
    int64_t pace_target = 0;
    int deferred_slot = -1;
    uint32_t last_audio_fn = 0;

    if (max_pkts > SENDER_ISO_MAX_PACKETS)
        max_pkts = SENDER_ISO_MAX_PACKETS;

    build_buf = av_malloc(SENDER_BUILD_BUF_SIZE);
    iso_data_buf = av_malloc(SENDER_ISO_BUF_SIZE);
    if (!build_buf || !iso_data_buf) {
        av_free(build_buf);
        av_free(iso_data_buf);
        return AVERROR(ENOMEM);
    }

    memset(iso_descs, 0, sizeof(iso_descs));

    /*
     * Determine which media type this request targets.
     * Dedicated channel (only video or only audio): use channel flags.
     * Interleaved channel (both video and audio): match by endpoint address.
     */
    if (!(ch->is_video && ch->is_audio)) {
        is_video = ch->is_video;
        is_audio = ch->is_audio;
    } else {
        is_video = (ch->desc_set.video_ep &&
                    (endpoint & 0x7F) == (ch->desc_set.video_ep & 0x7F));
        is_audio = (ch->desc_set.audio_ep &&
                    (endpoint & 0x7F) == (ch->desc_set.audio_ep & 0x7F));
    }

    ch_interleaved = (ch->is_video && ch->is_audio);
    dequeue_flags = ch_interleaved ? AV_THREAD_MESSAGE_NONBLOCK : 0;

    if (is_video && ch->video_q) {
        int have_frame = 0;
        ctx->diag_iso_vid_req++;

        if (!ch->uvc_seg.frame_done && ch->current_video_pkt) {
            have_frame = 1;
        } else {
            SenderQueueMsg qmsg;
            av_packet_free(&ch->current_video_pkt);
            ret = av_thread_message_queue_recv(ch->video_q, &qmsg,
                                               dequeue_flags);
            if (ret == 0) {
                uint32_t v_dev_pts = 0, v_scr_stc = 0, v_fn = 0;
                uint16_t v_scr_sof = 0;
                int64_t  v_wire_pts = -1, v_pace_us;
                int v_have_dev_pts = 0, v_have_scr = 0;

                ch->current_video_pkt = qmsg.pkt;
                ctx->diag_vid_deq++;

                if (!ch->video_epoch_us) {
                    ch->video_epoch_us    = av_gettime_relative();
                    ch->video_frame_count = 0;
                    {
                        int_fast64_t expected = 0;
                        atomic_compare_exchange_strong(
                            &ctx->shared_frame_epoch_us, &expected,
                            (int_fast64_t)ch->video_epoch_us);
                        ch->frame_epoch_us = atomic_load(
                            &ctx->shared_frame_epoch_us);
                    }
                    if (!ch->audio_epoch_us)
                        ch->audio_epoch_us = ch->video_epoch_us;
                    if (!ch->video_sf_offset_set) {
                        ch->video_sf_offset = (uint32_t)(
                            (ch->video_epoch_us - ch->frame_epoch_us) / 1000);
                        ch->video_sf_offset_set = 1;
                    }
                }
                if (!ch->pts_epoch_set && qmsg.pkt->pts != AV_NOPTS_VALUE) {
                    int64_t pts_us = (ctx->video_tb.den > 0)
                        ? av_rescale_q(qmsg.pkt->pts, ctx->video_tb,
                                       (AVRational){1, 1000000})
                        : qmsg.pkt->pts;
                    ch->pts_epoch_us  = pts_us;
                    ch->pts_epoch_set = 1;
                }

                v_pace_us = (qmsg.pkt->pts != AV_NOPTS_VALUE)
                    ? av_rescale_q(qmsg.pkt->pts, ctx->video_tb,
                                   (AVRational){1, 1000000})
                    : qmsg.pkt->pts;
                if (v_pace_us != AV_NOPTS_VALUE && ch->pts_epoch_set)
                    v_pace_us -= ch->pts_epoch_us;

                v_have_dev_pts = ipmx_timing_read_device_pts(qmsg.pkt, &v_dev_pts);
                v_have_scr     = ipmx_timing_read_scr(qmsg.pkt, &v_scr_stc, &v_scr_sof);
                v_fn           = ipmx_timing_read_frame_number(qmsg.pkt);

                if (v_have_dev_pts) {
                    v_wire_pts = (int64_t)v_dev_pts;
                    if (!v_have_scr) {
                        v_scr_stc  = v_dev_pts;
                        v_scr_sof  = (uint16_t)(v_fn & 0x7FF);
                        v_have_scr = 1;
                    }
                } else if (v_pace_us >= 0) {
                    v_wire_pts = v_pace_us;
                    v_scr_stc  = (uint32_t)(v_pace_us & 0xFFFFFFFF);
                    v_scr_sof  = v_fn ? (uint16_t)(v_fn & 0x7FF)
                                      : (uint16_t)((v_pace_us / 1000) & 0x7FF);
                    v_have_scr = 1;
                }

                ipmx_uvc_seg_begin_frame(&ch->uvc_seg,
                                         qmsg.pkt->data, qmsg.pkt->size,
                                         v_wire_pts,
                                         v_have_scr, v_scr_stc, v_scr_sof);

                if (ch->video_epoch_us && v_pace_us >= 0) {
                    pace_target = ch->video_epoch_us + v_pace_us;
                    deferred_slot = DEFERRED_VIDEO;
                }

                startframe = get_startframe(ch, qmsg.pkt, ctx->video_tb);

                if (ctx->timing_fp) {
                    fprintf(ctx->timing_fp,
                            "%lld\tDEQ\tV\t%lld\t%lld\t"
                            "pkt_pts=%lld\tSF=%u\tFN=%u\t"
                            "UVC_PTS=%u(%d)\tSCR=%u/%u(%d)\t"
                            "wire_pts=%lld\tpace_us=%lld\t"
                            "cap=%lld\n",
                            (long long)av_gettime_relative(),
                            (long long)ctx->timing_deq_vid,
                            (long long)ipmx_timing_read_pkt(qmsg.pkt),
                            (long long)qmsg.pkt->pts,
                            (unsigned)startframe,
                            (unsigned)v_fn,
                            (unsigned)v_dev_pts, v_have_dev_pts,
                            (unsigned)v_scr_stc,
                            (unsigned)v_scr_sof, v_have_scr,
                            (long long)v_wire_pts,
                            (long long)v_pace_us,
                            (long long)ipmx_timing_read_capture_wall(qmsg.pkt));
                    ctx->timing_deq_vid++;
                }

                ch->video_frame_count++;
                have_frame = 1;
            }
        }

        if (have_frame) {
            int pkt_pos = 0;
            int out_pos = 0;

            while (pkt_pos < (int)max_pkts) {
                int pkt_len = 0, is_last = 0;
                int sr = ipmx_uvc_seg_next_packet(
                    &ch->uvc_seg,
                    iso_data_buf + out_pos,
                    SENDER_ISO_BUF_SIZE - out_pos,
                    &pkt_len, &is_last);
                if (sr != 0)
                    break;
                iso_descs[pkt_pos].actuallength = (uint16_t)pkt_len;
                iso_descs[pkt_pos].isostatus    = 0;
                out_pos += pkt_len;
                pkt_pos++;
                if (is_last)
                    break;
            }
            num_pkts = pkt_pos;
            total_transfer_len = out_pos;
            ctx->diag_iso_vid_data++;

            if (ch->uvc_seg.frame_done)
                av_packet_free(&ch->current_video_pkt);
        } else {
            num_pkts = 0;
            total_transfer_len = 0;
            ctx->diag_iso_vid_empty++;
        }
    } else if (is_audio && ch->audio_q) {
        SenderQueueMsg qmsg;
        ctx->diag_iso_aud_req++;

        /* Fill the residual buffer from the queue until we have enough
         * data for the requested number of ISO packets, or the queue
         * is empty (non-blocking) / we have at least some data. */
        {
            int needed = ch->uac_pack.bytes_per_ms * (int)max_pkts;
            while (ch->audio_residual_len < needed) {
                int ret2 = av_thread_message_queue_recv(ch->audio_q, &qmsg,
                                                         dequeue_flags);
                if (ret2 != 0)
                    break;
                ctx->diag_aud_deq++;
                if (ctx->timing_fp) {
                    int64_t enq_us = ipmx_timing_read_pkt(qmsg.pkt);
                    uint32_t a_fn = ipmx_timing_read_frame_number(qmsg.pkt);
                    fprintf(ctx->timing_fp,
                            "%lld\tDEQ\tA\t%lld\t%lld\t"
                            "SF=%u\tcap=%lld\n",
                            (long long)av_gettime_relative(),
                            (long long)ctx->timing_deq_aud,
                            (long long)enq_us,
                            (unsigned)a_fn,
                            (long long)ipmx_timing_read_capture_wall(qmsg.pkt));
                    ctx->timing_deq_aud++;
                }

                if (!ch->audio_epoch_us) {
                    ch->audio_bytes_sent = 0;
                    ch->audio_epoch_us   = av_gettime_relative();
                    {
                        int_fast64_t expected = 0;
                        atomic_compare_exchange_strong(
                            &ctx->shared_frame_epoch_us, &expected,
                            (int_fast64_t)ch->audio_epoch_us);
                        ch->frame_epoch_us = atomic_load(
                            &ctx->shared_frame_epoch_us);
                    }
                    if (!ch->video_epoch_us)
                        ch->video_epoch_us = ch->audio_epoch_us;
                    if (!ch->audio_sf_offset_set) {
                        ch->audio_sf_offset = (uint32_t)(
                            (ch->audio_epoch_us - ch->frame_epoch_us) / 1000);
                        ch->audio_sf_offset_set = 1;
                    }
                }

                if (qmsg.pkt->pts != AV_NOPTS_VALUE) {
                    int64_t apts = (ctx->audio_tb.den > 0)
                        ? av_rescale_q(qmsg.pkt->pts, ctx->audio_tb,
                                       (AVRational){1, 1000000})
                        : qmsg.pkt->pts;
                    if (!ch->audio_pts_epoch_set) {
                        ch->audio_pts_epoch_us  = apts;
                        ch->audio_pts_epoch_set = 1;
                    }
                    ch->audio_latest_pts_us = apts;
                }

                {
                    uint32_t fn = ipmx_timing_read_frame_number(qmsg.pkt);
                    if (fn)
                        last_audio_fn = fn;
                }

                /* Grow residual buffer if needed */
                if (ch->audio_residual_len + qmsg.pkt->size > ch->audio_residual_cap) {
                    int new_cap = ch->audio_residual_len + qmsg.pkt->size + 8192;
                    uint8_t *tmp = av_realloc(ch->audio_residual, new_cap);
                    if (!tmp) {
                        av_packet_free(&qmsg.pkt);
                        break;
                    }
                    ch->audio_residual = tmp;
                    ch->audio_residual_cap = new_cap;
                }

                memcpy(ch->audio_residual + ch->audio_residual_len,
                       qmsg.pkt->data, qmsg.pkt->size);
                ch->audio_residual_len += qmsg.pkt->size;
                av_packet_free(&qmsg.pkt);
            }

            if (last_audio_fn)
                startframe = last_audio_fn;
            else if (ch->uac_pack.bytes_per_ms > 0) {
                /* Synthetic source: fn from current consumption position,
                 * offset to the shared SOF epoch. */
                last_audio_fn = ch->audio_sf_offset +
                                (uint32_t)(ch->audio_bytes_sent /
                                           ch->uac_pack.bytes_per_ms);
                startframe = last_audio_fn;
            }
        }

        if (ch->audio_residual_len > 0) {
            int consumed;

            if (last_audio_fn > 0) {
                /* Use frame_epoch (shared SOF clock) as base since
                 * last_audio_fn now includes the audio_sf_offset. */
                pace_target = ch->frame_epoch_us +
                              (int64_t)last_audio_fn * 1000;
                deferred_slot = DEFERRED_AUDIO;
            }

            if (!startframe) {
                int64_t now = av_gettime_relative();
                {
                    int_fast64_t expected = 0;
                    atomic_compare_exchange_strong(
                        &ctx->shared_frame_epoch_us, &expected,
                        (int_fast64_t)now);
                }
                if (!ch->frame_epoch_us)
                    ch->frame_epoch_us = atomic_load(
                        &ctx->shared_frame_epoch_us);
                startframe = (uint32_t)((now - ch->frame_epoch_us) / 1000);
            }

            ipmx_uac_pack_packets(&ch->uac_pack,
                                  ch->audio_residual, ch->audio_residual_len,
                                  iso_data_buf, SENDER_ISO_BUF_SIZE,
                                  iso_descs, (int)max_pkts,
                                  &num_pkts, &total_transfer_len);
            consumed = total_transfer_len;
            ch->audio_bytes_sent += consumed;
            if (consumed > 0 && consumed < ch->audio_residual_len) {
                memmove(ch->audio_residual,
                        ch->audio_residual + consumed,
                        ch->audio_residual_len - consumed);
            }
            ch->audio_residual_len -= consumed;
            ctx->diag_iso_aud_data++;
        } else {
            num_pkts = 0;
            total_transfer_len = 0;
            ctx->diag_iso_aud_empty++;
        }
    } else {
        num_pkts = 0;
        total_transfer_len = 0;
    }

    len = ipmx_usb_build_iso_submit_return(
        build_buf, SENDER_BUILD_BUF_SIZE,
        iso->seqnum, endpoint, iso->direction,
        startframe, 0, (uint32_t)num_pkts,
        iso_descs, iso_data_buf, total_transfer_len);

    av_free(iso_data_buf);

    if (len < 0) {
        av_free(build_buf);
        return AVERROR_EXIT;
    }

    /*
     * Pacing strategy:
     *   Dedicated channel (one media type): block inline until the pacing
     *     target — no head-of-line issue since only one stream is served.
     *   Interleaved channel (video + audio): blocking for a 33 ms video
     *     target would starve the 8 ms audio cadence.  Instead, store the
     *     built response in a deferred slot and let the main loop send it
     *     at the right time.
     */
    if (pace_target > 0 && !ch_interleaved) {
        iso_pace_wait(pace_target);
    }

    if (pace_target > 0 && ch_interleaved && deferred_slot >= 0) {
        int64_t wait = pace_target - av_gettime_relative();
        if (wait > 0 && wait <= ISO_PACE_MAX_EARLY_US) {
            DeferredIsoResponse *d = &ch->deferred[deferred_slot];
            if (d->valid) {
                /* Previous deferred response still pending — send it now
                 * before storing the new one. */
                int64_t tx_us = av_gettime_relative();
                ret = ipmx_usb_send_message(s, pep, ch->sock,
                                            ch->substreamid,
                                            &ch->ctr_s2r, d->buf, d->len);
                if (ret < 0)
                    ctx->diag_iso_send_err++;
                if (ctx->timing_fp) {
                    int64_t seq = (d->media == 'V') ? ctx->timing_tx_vid++
                                                     : ctx->timing_tx_aud++;
                    fprintf(ctx->timing_fp,
                            "%lld\tTX\t%c\t%lld\t0=%ld\tSF=%u\n",
                            (long long)tx_us, d->media,
                            (long long)seq, pace_target, (unsigned)d->startframe);
                }
                av_free(d->buf);
                d->buf   = NULL;
                d->valid = 0;
            }
            d->buf        = build_buf;
            d->len        = len;
            d->target_us  = pace_target;
            d->media      = is_video ? 'V' : 'A';
            d->startframe = startframe;
            d->valid      = 1;
            return 0;  /* build_buf ownership transferred to deferred slot */
        }
    }

    /* Target in the past, no pacing, or split mode (already waited) — send now */
    {
        int64_t tx_us = av_gettime_relative();
        ret = ipmx_usb_send_message(s, pep, ch->sock,
                                     ch->substreamid,
                                     &ch->ctr_s2r, build_buf, len);
        if (ret < 0)
            ctx->diag_iso_send_err++;
        if (ctx->timing_fp && total_transfer_len > 0) {
            char media = is_video ? 'V' : (is_audio ? 'A' : '?');
            int64_t seq = is_video ? ctx->timing_tx_vid++
                                   : ctx->timing_tx_aud++;
            fprintf(ctx->timing_fp,
                    "%lld\tTX\t%c\t%lld\t2=%ld\tSF=%u\n",
                    (long long)tx_us, media,
                    (long long)seq, pace_target, (unsigned)startframe);
        }
    }
    av_free(build_buf);
    return ret;
}

/* ---------------------------------------------------------------------------
 * Drain stale packets from the sender queues so the first ISO response
 * after (re)connection carries fresh data — minimises startup latency.
 * --------------------------------------------------------------------------- */

static void drain_stale_packets(AVFormatContext *s, SenderDataChannel *ch)
{
    SenderQueueMsg qmsg;
    int n_video = 0, n_audio = 0;

    if (ch->video_q) {
        while (av_thread_message_queue_recv(ch->video_q, &qmsg,
                                            AV_THREAD_MESSAGE_NONBLOCK) >= 0) {
            av_packet_free(&qmsg.pkt);
            n_video++;
        }
    }
    if (ch->audio_q) {
        while (av_thread_message_queue_recv(ch->audio_q, &qmsg,
                                            AV_THREAD_MESSAGE_NONBLOCK) >= 0) {
            av_packet_free(&qmsg.pkt);
            n_audio++;
        }
    }

    /* Reset pacing epochs so timing restarts from the first fresh packet */
    ch->frame_epoch_us     = 0;
    ch->video_epoch_us     = 0;
    ch->video_frame_count  = 0;
    ch->audio_epoch_us     = 0;
    ch->pts_epoch_us       = 0;
    ch->pts_epoch_set      = 0;
    ch->audio_pts_epoch_us  = 0;
    ch->audio_pts_epoch_set = 0;
    ch->audio_latest_pts_us = 0;
    ch->audio_bytes_sent   = 0;
    ch->audio_residual_len = 0;
    av_packet_free(&ch->current_video_pkt);
    ipmx_uvc_seg_init(&ch->uvc_seg,
                       ch->desc_params.video_max_pkt_size, 0);

    if (n_video || n_audio)
        av_log(s, AV_LOG_INFO,
               "Drained %d video + %d audio stale packets (ssid=%u)\n",
               n_video, n_audio, ch->substreamid);
}

/* ---------------------------------------------------------------------------
 * Data channel thread: dedicated per-channel USB message dispatch
 * --------------------------------------------------------------------------- */

static void *data_channel_thread(void *arg)
{
    DataChannelThreadArg *ta = arg;
    AVFormatContext *s = ta->s;
    IpmxUsbSenderContext *ctx = s->priv_data;
    SenderDataChannel *ch = ta->ch;
    IpmxUsbPepState *pep = ctx->pep.encrypted ? &ctx->pep : NULL;
    IpmxUsbMsg msg;
    int ret, reconnect_attempts = 0;
    int64_t diag_next = 0;
    const char *tag = ch->is_video ? "video" : (ch->is_audio ? "audio" : "mixed");

    av_free(ta);
    ch->thread_running = 1;

    if (!ctx->diag_fp && ctx->timing_fp) {
        ctx->diag_fp = fopen("/tmp/ipmx_usb_sender_diag.log", "w");
        if (ctx->diag_fp)
            setvbuf(ctx->diag_fp, NULL, _IOLBF, 0);
    }

    av_log(s, AV_LOG_INFO,
           "Data channel thread started: ssid=%u type=%s\n",
           ch->substreamid, tag);

    drain_stale_packets(s, ch);

    /* Outer reconnect loop: on data channel loss, close/sleep/reconnect */
    while (!ctx->stop) {

    /* Inner message loop */
    while (!ctx->stop) {
        int64_t now = av_gettime_relative();
        int recv_timeout_ms;

        if (ctx->diag_fp && now >= diag_next) {
            fprintf(ctx->diag_fp,
                    "[%s] t=%lld wp_v=%lld wp_a=%lld "
                    "iso_v=%lld/%lld/%lld iso_a=%lld/%lld/%lld "
                    "deq_v=%lld deq_a=%lld err=%lld\n",
                    tag,
                    (long long)(now / 1000000),
                    (long long)ctx->diag_wp_video,
                    (long long)ctx->diag_wp_audio,
                    (long long)ctx->diag_iso_vid_req,
                    (long long)ctx->diag_iso_vid_data,
                    (long long)ctx->diag_iso_vid_empty,
                    (long long)ctx->diag_iso_aud_req,
                    (long long)ctx->diag_iso_aud_data,
                    (long long)ctx->diag_iso_aud_empty,
                    (long long)ctx->diag_vid_deq,
                    (long long)ctx->diag_aud_deq,
                    (long long)ctx->diag_iso_send_err);
            diag_next = now + 2000000;
        }

        /* Flush any deferred ISO responses whose pacing target has arrived */
        flush_deferred_iso(s, ch, pep);

        /* In interleaved mode with pending deferred responses, use a short
         * recv timeout so we wake up promptly to send them.  Otherwise use
         * the default 200 ms timeout.  The deferred_wait_us helper returns
         * the time until the earliest deferred target (0 = nothing pending). */
        {
            int64_t dwait = deferred_wait_us(ch);
            if (dwait > 0) {
                int ms = (int)(dwait > ISO_PACE_SPIN_US
                               ? (dwait - ISO_PACE_SPIN_US + 999) / 1000 : 0);
                recv_timeout_ms = ms > 0 ? FFMIN(ms, 200) : 0;
            } else {
                recv_timeout_ms = 8;
            }
        }

        ret = ipmx_usb_recv_message(s, pep, ch->sock,
                                     ch->recv_buf, &ch->recv_buf_len,
                                     &ch->recv_buf_pos,
                                     ch->substreamid | 1,
                                     &ch->ctr_r2s,
                                     &ch->fp_r2s,
                                     &msg, recv_timeout_ms);
        if (ret == AVERROR(EAGAIN))
            continue;
        if (ret == IPMX_USB_ERR_MAC_FAIL) {
            if (ch->fp_r2s.strict) {
                av_log(s, AV_LOG_ERROR,
                       "Data channel %s ssid=%u: MAC failed (strict) — "
                       "closing channel\n", tag, ch->substreamid);
                break;
            }
            av_log(s, AV_LOG_WARNING,
                   "Data channel %s ssid=%u: MAC failed on msg_type=0x%02x"
                   " — discarding\n", tag, ch->substreamid,
                   msg.hdr.msg_type);
            continue;
        }
        if (ret == AVERROR_EOF) {
            av_log(s, AV_LOG_INFO,
                   "Data channel %s ssid=%u: Receiver disconnected\n",
                   tag, ch->substreamid);
            break;
        }
        if (ret < 0) {
            av_log(s, AV_LOG_ERROR,
                   "Data channel %s ssid=%u: recv error %d\n",
                   tag, ch->substreamid, ret);
            break;
        }

        reconnect_attempts = 0;

        switch (msg.hdr.msg_type) {
        case IPMX_USB_CONTROL_SUBMIT:
            handle_control_submit(s, ch, &msg.payload.control_submit, pep);
            break;
        case IPMX_USB_INTERRUPT_SUBMIT:
            handle_interrupt_submit(s, ch,
                                    &msg.payload.interrupt_submit, pep);
            break;
        case IPMX_USB_ISOCHRONOUS_SUBMIT:
            handle_iso_submit(s, ch, &msg.payload.iso_submit, pep);
            break;
        case IPMX_USB_BULK_SUBMIT: {
            uint8_t bbuf[128];
            int blen = ipmx_usb_build_bulk_submit_return(
                bbuf, sizeof(bbuf),
                msg.payload.bulk_submit.seqnum,
                msg.payload.bulk_submit.endpoint,
                msg.payload.bulk_submit.direction,
                0, 0, NULL, 0);
            if (blen > 0)
                ipmx_usb_send_message(s, pep, ch->sock,
                                       ch->substreamid,
                                       &ch->ctr_s2r, bbuf, blen);
            break;
        }
        case IPMX_USB_CANCEL_SUBMIT: {
            uint8_t cbuf[64];
            int clen = ipmx_usb_build_cancel_submit_return(
                cbuf, sizeof(cbuf),
                msg.payload.cancel_submit.seqnum,
                msg.payload.cancel_submit.endpoint,
                msg.payload.cancel_submit.direction, 0);
            if (clen > 0)
                ipmx_usb_send_message(s, pep, ch->sock,
                                       ch->substreamid,
                                       &ch->ctr_s2r, cbuf, clen);
            break;
        }
        default:
            av_log(s, AV_LOG_DEBUG,
                   "Data channel %s ssid=%u: ignoring msg_type=0x%02x\n",
                   tag, ch->substreamid, msg.hdr.msg_type);
            break;
        }
    } /* end inner message loop */

    if (ctx->stop)
        break;

    /* Data channel lost — attempt reconnect */
    if (ch->sock >= 0) {
        close(ch->sock);
        ch->sock = -1;
    }

    reconnect_attempts++;
    if (reconnect_attempts > DATA_RECONNECT_MAX_RETRIES) {
        av_log(s, AV_LOG_ERROR,
               "Data channel %s ch_%d: %d reconnect attempts failed — "
               "giving up\n", tag, ch->ch_idx, reconnect_attempts);
        break;
    }

    av_log(s, AV_LOG_INFO,
           "Data channel %s ch_%d: reconnecting to %s:%u "
           "(attempt %d/%d)...\n",
           tag, ch->ch_idx, ch->peer_ip, ch->data_port,
           reconnect_attempts, DATA_RECONNECT_MAX_RETRIES);

    av_usleep(DATA_RECONNECT_DELAY_US);
    if (ctx->stop) break;

    ret = connect_data_socket(s, ch, ch->peer_ip, ch->data_port, ch->ch_idx);
    if (ret < 0) {
        av_log(s, AV_LOG_WARNING,
               "Data channel %s ch_%d: reconnect failed (%d)\n",
               tag, ch->ch_idx, ret);
        continue;
    }

    ret = handshake_data_channel(s, ch, pep, ch->ch_idx);
    if (ret < 0) {
        av_log(s, AV_LOG_WARNING,
               "Data channel %s ch_%d: re-handshake failed (%d)\n",
               tag, ch->ch_idx, ret);
        if (ch->sock >= 0) {
            close(ch->sock);
            ch->sock = -1;
        }
        continue;
    }

    av_log(s, AV_LOG_INFO,
           "Data channel %s ch_%d: reconnected — session resumed\n",
           tag, ch->ch_idx);

    drain_stale_packets(s, ch);

    } /* end outer reconnect loop */

    if (ctx->diag_fp) {
        fprintf(ctx->diag_fp, "DATA_THREAD_EXIT [%s]\n", tag);
        fclose(ctx->diag_fp);
        ctx->diag_fp = NULL;
    }
    ch->thread_running = 0;
    return NULL;
}

/* ---------------------------------------------------------------------------
 * Helper: TCP-connect one data channel, allocate recv buffer (no handshake)
 * --------------------------------------------------------------------------- */

static int connect_data_socket(AVFormatContext *s, SenderDataChannel *ch,
                               const char *peer_ip, uint16_t data_port,
                               int ch_idx)
{
    int data_fd = ipmx_usb_tcp_connect(peer_ip, data_port);
    if (data_fd < 0) {
        av_log(s, AV_LOG_ERROR,
               "Failed to connect data channel %d to %s:%u\n",
               ch_idx, peer_ip, data_port);
        return AVERROR(EIO);
    }

    ch->sock = data_fd;
    ch->substreamid = IPMX_USB_DATA_PAYLOAD_SSID(ch_idx);
    ch->seqnum = 1;
    ch->recv_buf_len = 0;
    ch->recv_buf_pos = 0;
    ch->ctr_s2r = 0;
    ch->ctr_r2s = 0;
    memset(&ch->fp_r2s, 0, sizeof(ch->fp_r2s));
    ch->fp_r2s.strict = ((IpmxUsbSenderContext *)s->priv_data)->strict_pep;
    if (!ch->recv_buf) {
        ch->recv_buf = av_mallocz(IPMX_USB_RECV_BUF_SIZE);
        if (!ch->recv_buf)
            return AVERROR(ENOMEM);
    }

    av_strlcpy(ch->peer_ip, peer_ip, sizeof(ch->peer_ip));
    ch->data_port = data_port;
    ch->ch_idx    = ch_idx;

    av_log(s, AV_LOG_INFO,
           "Data channel %d: TCP connected to %s:%u\n",
           ch_idx, peer_ip, data_port);
    return 0;
}

/* ---------------------------------------------------------------------------
 * Helper: handshake one data channel (USBStreamInfo/USBStreamStatus exchange)
 * --------------------------------------------------------------------------- */

static int handshake_data_channel(AVFormatContext *s, SenderDataChannel *ch,
                                  IpmxUsbPepState *pep, int ch_idx)
{
    uint8_t build_buf[SENDER_BUILD_BUF_SIZE];
    IpmxUsbMsg msg;
    int ret, len;
    uint16_t hs_s2r_ssid, hs_r2s_ssid;
    char busid[8];

    hs_s2r_ssid = pep ? IPMX_USB_DATA_HANDSHAKE_S2R_SSID : 0;
    hs_r2s_ssid = pep ? IPMX_USB_DATA_HANDSHAKE_R2S_SSID : 0;

    snprintf(busid, sizeof(busid), "1-%d", ch_idx + 1);

    len = ipmx_usb_build_usb_stream_info(
        build_buf, sizeof(build_buf),
        ch->substreamid, IPMX_USB_SPEED_HIGH, busid);
    if (len < 0)
        return AVERROR_EXIT;

    ret = ipmx_usb_send_message(s, pep, ch->sock,
                                 hs_s2r_ssid, &ch->ctr_s2r,
                                 build_buf, len);
    if (ret < 0)
        return ret;
    av_log(s, AV_LOG_INFO,
           "Data channel %d: sent USBStreamInfo (ssid=%u)\n",
           ch_idx, ch->substreamid);

    ret = ipmx_usb_recv_message(s, pep, ch->sock,
                                 ch->recv_buf, &ch->recv_buf_len,
                                 &ch->recv_buf_pos,
                                 hs_r2s_ssid, &ch->ctr_r2s,
                                 &ch->fp_r2s,
                                 &msg, SENDER_DATA_TIMEOUT_MS);
    if (ret == IPMX_USB_ERR_MAC_FAIL) {
        av_log(s, AV_LOG_ERROR,
               "Data channel %d: USBStreamStatus MAC failed — "
               "cannot complete handshake\n", ch_idx);
        return AVERROR(EPROTO);
    }
    if (ret < 0) {
        av_log(s, AV_LOG_ERROR,
               "Data channel %d: failed to receive USBStreamStatus\n", ch_idx);
        return ret;
    }
    if (msg.hdr.msg_type != IPMX_USB_STREAM_STATUS) {
        av_log(s, AV_LOG_ERROR,
               "Data channel %d: expected USB_STREAM_STATUS, got 0x%02x\n",
               ch_idx, msg.hdr.msg_type);
        return AVERROR(EPROTO);
    }

    av_log(s, AV_LOG_INFO,
           "Data channel %d: USBStreamStatus received — active\n", ch_idx);
    return 0;
}

/* ---------------------------------------------------------------------------
 * Control thread: listen, handshake, spawn data threads, heartbeat
 * --------------------------------------------------------------------------- */

static void *sender_proto_thread(void *arg)
{
    AVFormatContext *s = arg;
    IpmxUsbSenderContext *ctx = s->priv_data;
    uint8_t build_buf[SENDER_BUILD_BUF_SIZE];
    uint8_t ctrl_recv_buf[IPMX_USB_RECV_BUF_SIZE];
    int ctrl_recv_buf_len = 0, ctrl_recv_buf_pos = 0;
    IpmxUsbMsg msg;
    int ret, len, i;
    IpmxUsbPepState *pep = ctx->pep.encrypted ? &ctx->pep : NULL;

    while (!ctx->stop) {
        int ctrl_fd;
        uint16_t data_port;
        int64_t last_hbeat;
        int64_t hbeat_interval_us;
        char peer_ip[64] = "127.0.0.1";
        int all_stopped;

        av_log(s, AV_LOG_INFO,
               "Waiting for Receiver on port %d...\n", ctx->listen_port);

        ctrl_fd = ipmx_usb_tcp_accept(ctx->ctrl_listen_sock, 1000);
        if (ctrl_fd < 0) {
            if (ctx->stop) break;
            continue;
        }
        ctx->ctrl_sock = ctrl_fd;
        av_log(s, AV_LOG_INFO, "Receiver connected (control)\n");

        ctrl_recv_buf_len = 0;
        ctrl_recv_buf_pos = 0;
        ctx->ctrl_ctr_s2r = 0;
        ctx->ctrl_ctr_r2s = 0;
        memset(&ctx->ctrl_fp_r2s, 0, sizeof(ctx->ctrl_fp_r2s));
        ctx->ctrl_fp_r2s.strict = ctx->strict_pep;

        /* Step 1: Send SenderConnectionInfo */
        {
            uint8_t cid[3] = {0x00, 0x50, 0xC2};
            len = ipmx_usb_build_sender_connection_info(
                build_buf, sizeof(build_buf), cid,
                ctx->serial ? ctx->serial : "FFMPEG-SENDER-01");
            if (len < 0) goto session_end;

            ret = ipmx_usb_send_message(s, pep, ctrl_fd,
                                         0, &ctx->ctrl_ctr_s2r,
                                         build_buf, len);
            if (ret < 0) goto session_end;
            av_log(s, AV_LOG_INFO, "Sent SenderConnectionInfo\n");
        }

        /* Step 2: Receive SenderConnectionStatus */
        ret = ipmx_usb_recv_message(s, pep, ctrl_fd,
                                     ctrl_recv_buf, &ctrl_recv_buf_len,
                                     &ctrl_recv_buf_pos,
                                     1, &ctx->ctrl_ctr_r2s,
                                     &ctx->ctrl_fp_r2s,
                                     &msg, SENDER_CTRL_TIMEOUT_MS);
        if (ret == IPMX_USB_ERR_MAC_FAIL) {
            av_log(s, AV_LOG_ERROR,
                   "SenderConnectionStatus MAC failed — "
                   "cannot establish control channel\n");
            goto session_end;
        }
        if (ret < 0) {
            av_log(s, AV_LOG_ERROR,
                   "Failed to receive SenderConnectionStatus\n");
            goto session_end;
        }
        if (msg.hdr.msg_type != IPMX_USB_SENDER_CONNECTION_STATUS) {
            av_log(s, AV_LOG_ERROR,
                   "Expected SENDER_CONNECTION_STATUS, got 0x%02x\n",
                   msg.hdr.msg_type);
            goto session_end;
        }

        data_port = msg.payload.sender_connection_status.data_port;
        {
            uint8_t hbi = msg.payload.sender_connection_status.hbeat_index;
            if (hbi < 5 || hbi > 30) {
                av_log(s, AV_LOG_WARNING,
                       "Receiver sent HBEAT index %u (out of [5,30]); "
                       "using default\n", hbi);
                hbeat_interval_us = (int64_t)(SENDER_DEFAULT_HEARTBEAT_SEC) * 1000000;
            } else {
                double period = 5.0 * pow(1.25, hbi);
                hbeat_interval_us = (int64_t)(period * 1e6);
                av_log(s, AV_LOG_INFO,
                       "Heartbeat period: %.1fs (index %u)\n", period, hbi);
            }
        }
        av_log(s, AV_LOG_INFO,
               "Received SenderConnectionStatus: data_port=%u\n", data_port);

        /* Resolve Receiver IP */
        {
            struct sockaddr_in peer;
            socklen_t plen = sizeof(peer);
            if (getpeername(ctrl_fd, (struct sockaddr *)&peer, &plen) == 0)
                inet_ntop(AF_INET, &peer.sin_addr, peer_ip, sizeof(peer_ip));
        }

        av_usleep(50000);

        /* Step 3a: TCP-connect all data channels (before any handshakes,
         * so the receiver's accept loop sees all connections) */
        for (i = 0; i < ctx->num_data_ch; i++) {
            ret = connect_data_socket(s, &ctx->data_ch[i],
                                       peer_ip, data_port, i);
            if (ret < 0)
                goto session_end;
            if (i < ctx->num_data_ch - 1)
                av_usleep(50000);
        }

        /* Step 3b: Handshake each channel and spawn its data thread immediately
         * (so the receiver's enumeration has a responder ready) */
        for (i = 0; i < ctx->num_data_ch; i++) {
            DataChannelThreadArg *ta;
            ret = handshake_data_channel(s, &ctx->data_ch[i], pep, i);
            if (ret < 0)
                goto session_end;

            ta = av_malloc(sizeof(*ta));
            if (!ta) goto session_end;
            ta->s  = s;
            ta->ch = &ctx->data_ch[i];
            ret = pthread_create(&ctx->data_ch[i].thread, NULL,
                                 data_channel_thread, ta);
            if (ret != 0) {
                av_free(ta);
                av_log(s, AV_LOG_ERROR,
                       "Failed to create data thread %d\n", i);
                goto session_end;
            }
            ctx->data_ch[i].thread_started = 1;
        }

        /* Tell write_packet to start enqueueing instead of discarding. */
        ctx->data_ch_ready = 1;
        av_log(s, AV_LOG_INFO,
               "Data channels ready — write_packet will start enqueueing\n");

        /* Step 5: Control channel loop — heartbeats + control messages */
        last_hbeat = av_gettime_relative();

        while (!ctx->stop) {
            int64_t now = av_gettime_relative();

            if (now - last_hbeat >= hbeat_interval_us) {
                int hb_len = ipmx_usb_build_heartbeat(build_buf,
                                                       sizeof(build_buf));
                if (hb_len > 0) {
                    ret = ipmx_usb_send_message(s, pep, ctrl_fd,
                                                 0, &ctx->ctrl_ctr_s2r,
                                                 build_buf, hb_len);
                    if (ret < 0)
                        av_log(s, AV_LOG_WARNING, "Heartbeat send failed\n");
                    else
                        av_log(s, AV_LOG_DEBUG, "Heartbeat sent\n");
                }
                last_hbeat = now;
            }

            {
                IpmxUsbMsg ctrl_msg;
                int cr = ipmx_usb_recv_message(s, pep, ctrl_fd,
                                                ctrl_recv_buf,
                                                &ctrl_recv_buf_len,
                                                &ctrl_recv_buf_pos,
                                                1, &ctx->ctrl_ctr_r2s,
                                                &ctx->ctrl_fp_r2s,
                                                &ctrl_msg, 1000);
                if (cr == 0) {
                    av_log(s, AV_LOG_DEBUG,
                           "Control msg_type=0x%02x\n",
                           ctrl_msg.hdr.msg_type);
                } else if (cr == IPMX_USB_ERR_MAC_FAIL) {
                    if (ctx->ctrl_fp_r2s.strict) {
                        av_log(s, AV_LOG_ERROR,
                               "Control channel MAC failed (strict) — "
                               "closing\n");
                        break;
                    }
                    av_log(s, AV_LOG_WARNING,
                           "Control channel MAC failed on "
                           "msg_type=0x%02x — discarding\n",
                           ctrl_msg.hdr.msg_type);
                } else if (cr == AVERROR_EOF) {
                    av_log(s, AV_LOG_INFO,
                           "Receiver control channel disconnected\n");
                    break;
                }
            }

            all_stopped = 1;
            for (i = 0; i < ctx->num_data_ch; i++) {
                if (ctx->data_ch[i].thread_running)
                    all_stopped = 0;
            }
            if (all_stopped) {
                av_log(s, AV_LOG_INFO,
                       "All data channel threads ended (exhausted retries) "
                       "— ending session\n");
                break;
            }
        }

        /* Join data threads */
        for (i = 0; i < ctx->num_data_ch; i++) {
            if (ctx->data_ch[i].thread_started) {
                pthread_join(ctx->data_ch[i].thread, NULL);
                ctx->data_ch[i].thread_started = 0;
            }
        }

session_end:
        /* Stop enqueueing immediately so write_packet discards packets
         * while we wait for the next receiver connection. */
        ctx->data_ch_ready = 0;

        for (i = 0; i < ctx->num_data_ch; i++) {
            if (ctx->data_ch[i].sock >= 0) {
                close(ctx->data_ch[i].sock);
                ctx->data_ch[i].sock = -1;
            }
        }
        if (ctx->ctrl_sock >= 0) {
            close(ctx->ctrl_sock);
            ctx->ctrl_sock = -1;
        }

        if (ctx->stop)
            break;

        av_log(s, AV_LOG_INFO, "Session ended, waiting for reconnection...\n");
    }

    return NULL;
}

/* ---------------------------------------------------------------------------
 * Helper: populate common descriptor params from stream codec parameters
 * --------------------------------------------------------------------------- */

static void fill_video_desc_params(IpmxUsbVirtDescParams *p,
                                   AVFormatContext *s, int stream_idx,
                                   const char *serial)
{
    AVCodecParameters *vp = s->streams[stream_idx]->codecpar;
    memset(p, 0, sizeof(*p));
    p->manufacturer = "FFmpeg";
    p->product      = "IPMX USB Sender (Video)";
    p->serial       = serial;
    p->pid          = 0x0001;
    p->video_width  = vp->width > 0  ? vp->width  : 1920;
    p->video_height = vp->height > 0 ? vp->height : 1080;
    p->video_fps    = 30;
    if (vp->codec_id == AV_CODEC_ID_H264)
        p->video_codec = IPMX_USB_VCODEC_H264;
    else if (vp->codec_id == AV_CODEC_ID_HEVC)
        p->video_codec = IPMX_USB_VCODEC_HEVC;
    else
        p->video_codec = IPMX_USB_VCODEC_MJPEG;
    p->video_max_pkt_size = 3072;
    if (s->streams[stream_idx]->avg_frame_rate.den > 0)
        p->video_fps = s->streams[stream_idx]->avg_frame_rate.num /
                       s->streams[stream_idx]->avg_frame_rate.den;
}

static void fill_audio_desc_params(IpmxUsbVirtDescParams *p,
                                   AVFormatContext *s, int stream_idx,
                                   const char *serial)
{
    AVCodecParameters *ap = s->streams[stream_idx]->codecpar;
    memset(p, 0, sizeof(*p));
    p->manufacturer = "FFmpeg";
    p->product      = "IPMX USB Sender (Audio)";
    p->serial       = serial;
    p->pid          = 0x0002;
    p->audio_sample_rate = ap->sample_rate > 0 ? ap->sample_rate : 48000;
    p->audio_channels = ap->ch_layout.nb_channels > 0
                        ? ap->ch_layout.nb_channels : 2;
    p->audio_bit_depth = ap->bits_per_coded_sample > 0
                         ? ap->bits_per_coded_sample : 16;
}

static void fill_combined_desc_params(IpmxUsbVirtDescParams *p,
                                      AVFormatContext *s,
                                      int video_idx, int audio_idx,
                                      const char *serial)
{
    memset(p, 0, sizeof(*p));
    p->manufacturer = "FFmpeg";
    p->product      = "IPMX USB Sender";
    p->serial       = serial;
    p->pid          = 0x0001;

    if (video_idx >= 0) {
        AVCodecParameters *vp = s->streams[video_idx]->codecpar;
        p->video_width  = vp->width > 0  ? vp->width  : 1920;
        p->video_height = vp->height > 0 ? vp->height : 1080;
        p->video_fps    = 30;
        if (vp->codec_id == AV_CODEC_ID_H264)
            p->video_codec = IPMX_USB_VCODEC_H264;
        else if (vp->codec_id == AV_CODEC_ID_HEVC)
            p->video_codec = IPMX_USB_VCODEC_HEVC;
        else
            p->video_codec = IPMX_USB_VCODEC_MJPEG;
        p->video_max_pkt_size = 3072;
        if (s->streams[video_idx]->avg_frame_rate.den > 0)
            p->video_fps = s->streams[video_idx]->avg_frame_rate.num /
                           s->streams[video_idx]->avg_frame_rate.den;
    }
    if (audio_idx >= 0) {
        AVCodecParameters *ap = s->streams[audio_idx]->codecpar;
        p->audio_sample_rate = ap->sample_rate > 0 ? ap->sample_rate : 48000;
        p->audio_channels = ap->ch_layout.nb_channels > 0
                            ? ap->ch_layout.nb_channels : 2;
        p->audio_bit_depth = ap->bits_per_coded_sample > 0
                             ? ap->bits_per_coded_sample : 16;
    }
}

/* ---------------------------------------------------------------------------
 * init: validate streams, build descriptors, configure channels
 * --------------------------------------------------------------------------- */

static int ipmx_usb_sender_init(AVFormatContext *s)
{
    IpmxUsbSenderContext *ctx = s->priv_data;
    const char *serial = ctx->serial ? ctx->serial : "0000000000";
    int i;

    ctx->video_stream_idx = -1;
    ctx->audio_stream_idx = -1;
    ctx->ctrl_listen_sock = -1;
    ctx->ctrl_sock        = -1;
    ctx->num_data_ch      = 0;

    for (i = 0; i < SENDER_MAX_DATA_CH; i++)
        ctx->data_ch[i].sock = -1;

    memset(&ctx->pep, 0, sizeof(ctx->pep));
    ctx->data_ch_ready = 0;

    for (i = 0; i < s->nb_streams; i++) {
        AVStream *st = s->streams[i];
        if (st->codecpar->codec_type == AVMEDIA_TYPE_VIDEO &&
            ctx->video_stream_idx < 0) {
            ctx->video_stream_idx = i;
            ctx->video_tb = st->time_base;
        } else if (st->codecpar->codec_type == AVMEDIA_TYPE_AUDIO &&
                   ctx->audio_stream_idx < 0) {
            ctx->audio_stream_idx = i;
            ctx->audio_tb = st->time_base;
        }
    }

    if (ctx->video_stream_idx < 0 && ctx->audio_stream_idx < 0) {
        av_log(s, AV_LOG_ERROR,
               "At least one video or audio stream is required\n");
        return AVERROR(EINVAL);
    }

    if (ctx->video_clock_freq == 0 && ctx->video_stream_idx >= 0) {
        const AVDictionaryEntry *e = av_dict_get(
            s->streams[ctx->video_stream_idx]->metadata,
            "uvc_clock_frequency", NULL, 0);
        if (e && e->value) {
            ctx->video_clock_freq = (int)strtoul(e->value, NULL, 10);
            av_log(s, AV_LOG_INFO,
                   "Auto-detected dwClockFrequency=%d Hz from source metadata\n",
                   ctx->video_clock_freq);
        }
    }

    if (ctx->split_device &&
        ctx->video_stream_idx >= 0 && ctx->audio_stream_idx >= 0) {
        /* Split mode: two separate USB devices (one video, one audio) */
        SenderDataChannel *vch = &ctx->data_ch[0];
        SenderDataChannel *ach = &ctx->data_ch[1];

        vch->is_video = 1;
        vch->is_audio = 0;
        fill_video_desc_params(&vch->desc_params, s,
                               ctx->video_stream_idx, serial);
        vch->desc_params.audio_uac2 = ctx->audio_uac2;
        vch->desc_params.video_clock_freq = ctx->video_clock_freq;
        if (ipmx_usb_virt_desc_build(&vch->desc_set, &vch->desc_params) < 0) {
            av_log(s, AV_LOG_ERROR,
                   "Failed to build video USB descriptors\n");
            return AVERROR_EXIT;
        }
        init_class_request_state(vch);
        ipmx_uvc_seg_init(&vch->uvc_seg,
                            vch->desc_params.video_max_pkt_size, 0);

        ach->is_video = 0;
        ach->is_audio = 1;
        fill_audio_desc_params(&ach->desc_params, s,
                               ctx->audio_stream_idx, serial);
        ach->desc_params.audio_uac2 = ctx->audio_uac2;
        if (ipmx_usb_virt_desc_build(&ach->desc_set, &ach->desc_params) < 0) {
            av_log(s, AV_LOG_ERROR,
                   "Failed to build audio USB descriptors\n");
            return AVERROR_EXIT;
        }
        init_class_request_state(ach);
        {
            uint8_t bps = ach->desc_params.audio_bit_depth / 8;
            if (bps == 0) bps = 2;
            ipmx_uac_pack_init(&ach->uac_pack,
                                ach->desc_params.audio_max_pkt_size
                                    ? ach->desc_params.audio_max_pkt_size : 192,
                                ach->desc_params.audio_sample_rate,
                                ach->desc_params.audio_channels, bps);
        }

        ctx->num_data_ch = 2;
        av_log(s, AV_LOG_INFO,
               "Split mode: 2 data channels (video + audio)\n");
    } else {
        /* Interleaved mode: single USB device */
        SenderDataChannel *ch = &ctx->data_ch[0];

        ch->is_video = (ctx->video_stream_idx >= 0);
        ch->is_audio = (ctx->audio_stream_idx >= 0);
        fill_combined_desc_params(&ch->desc_params, s,
                                  ctx->video_stream_idx,
                                  ctx->audio_stream_idx, serial);
        ch->desc_params.audio_uac2 = ctx->audio_uac2;
        ch->desc_params.video_clock_freq = ctx->video_clock_freq;
        if (ipmx_usb_virt_desc_build(&ch->desc_set, &ch->desc_params) < 0) {
            av_log(s, AV_LOG_ERROR,
                   "Failed to build virtual USB descriptors\n");
            return AVERROR_EXIT;
        }
        init_class_request_state(ch);

        if (ctx->video_stream_idx >= 0)
            ipmx_uvc_seg_init(&ch->uvc_seg,
                               ch->desc_params.video_max_pkt_size, 0);
        if (ctx->audio_stream_idx >= 0) {
            uint8_t bps = ch->desc_params.audio_bit_depth / 8;
            if (bps == 0) bps = 2;
            ipmx_uac_pack_init(&ch->uac_pack,
                                ch->desc_params.audio_max_pkt_size
                                    ? ch->desc_params.audio_max_pkt_size : 192,
                                ch->desc_params.audio_sample_rate,
                                ch->desc_params.audio_channels, bps);
        }

        ctx->num_data_ch = 1;
        av_log(s, AV_LOG_INFO,
               "Interleaved mode: 1 data channel\n");
    }

    return 0;
}

/* ---------------------------------------------------------------------------
 * write_header: PEP init, TCP listen, assign queues, spawn protocol thread
 * --------------------------------------------------------------------------- */

static int ipmx_usb_sender_write_header(AVFormatContext *s)
{
    IpmxUsbSenderContext *ctx = s->priv_data;
    int ret;

    /* Initialize PEP if requested */
    if (ctx->psk_hex || ctx->sdp_path ||
        ctx->pep_mode_str || ctx->pep_iv_str ||
        ctx->pep_key_generator_str || ctx->pep_key_version_str) {

        if (ctx->sdp_path) {
            char host[256] = {0};
            int port = 0;
            ipmx_usb_parse_sdp(s, ctx->sdp_path,
                                host, sizeof(host), &port, &ctx->pep);
        }

        {
            const char *pep_mode_src = ctx->pep_mode_str ? ctx->pep_mode_str : s->pep_mode;
            const char *pep_iv_src   = ctx->pep_iv_str ? ctx->pep_iv_str : s->pep_iv;
            const char *pep_kg_src   = ctx->pep_key_generator_str ? ctx->pep_key_generator_str : s->pep_key_generator;
            const char *pep_kv_src   = ctx->pep_key_version_str ? ctx->pep_key_version_str : s->pep_key_version;
            ret = ipmx_usb_init_pep(s, &ctx->pep,
                                     ctx->psk_hex,
                                     pep_mode_src,
                                     pep_iv_src,
                                     pep_kg_src,
                                     pep_kv_src,
                                     1);
        }
        if (ret < 0)
            return ret;
    }

    /* TCP listen */
    ctx->ctrl_listen_sock = ipmx_usb_tcp_listen(ctx->listen_port);
    if (ctx->ctrl_listen_sock < 0) {
        av_log(s, AV_LOG_ERROR, "Failed to listen on port %d\n",
               ctx->listen_port);
        return AVERROR(EIO);
    }
    av_log(s, AV_LOG_INFO, "Listening on port %d\n", ctx->listen_port);

    /* Allocate per-stream packet queues */
    if (ctx->video_stream_idx >= 0) {
        ret = av_thread_message_queue_alloc(&ctx->video_queue,
                                            SENDER_QUEUE_SIZE,
                                            sizeof(SenderQueueMsg));
        if (ret < 0)
            return ret;
        av_thread_message_queue_set_free_func(ctx->video_queue, queue_msg_free);
    }
    if (ctx->audio_stream_idx >= 0) {
        ret = av_thread_message_queue_alloc(&ctx->audio_queue,
                                            SENDER_QUEUE_SIZE,
                                            sizeof(SenderQueueMsg));
        if (ret < 0)
            return ret;
        av_thread_message_queue_set_free_func(ctx->audio_queue, queue_msg_free);
    }

    /* Assign per-media queue pointers to channels based on their media identity.
     * Each channel receives only the queues for the media types it serves. */
    for (int ci = 0; ci < ctx->num_data_ch; ci++) {
        SenderDataChannel *dch = &ctx->data_ch[ci];
        dch->video_q = dch->is_video ? ctx->video_queue : NULL;
        dch->audio_q = dch->is_audio ? ctx->audio_queue : NULL;
    }

    /* Spawn protocol thread */
    ctx->stop = 0;
    ret = pthread_create(&ctx->proto_thread, NULL, sender_proto_thread, s);
    if (ret != 0) {
        av_log(s, AV_LOG_ERROR, "Failed to create protocol thread\n");
        return AVERROR(ret);
    }
    ctx->proto_thread_started = 1;

    ctx->timing_fp = ipmx_timing_open("/tmp/ipmx_timing_sender.log");
    if (ctx->timing_fp)
        fprintf(ctx->timing_fp,
                "# wall_us\tevent\tmedia\tcounter\tref_us\textra\n"
                "# DEQ V extra: pkt_pts=<capture>\tSF=<startframe>\tFN=<frame_number>\t"
                "UVC_PTS=<ticks>(<valid>)\tSCR=<stc>/<sof>(<valid>)\t"
                "wire_pts=<ticks>\tpace_us=<us>\n"
                "# TX V/A extra: SF=<startframe>\n");

    return 0;
}

/* ---------------------------------------------------------------------------
 * write_packet: enqueue AVPacket for data channel threads
 * --------------------------------------------------------------------------- */

static int ipmx_usb_sender_write_packet(AVFormatContext *s, AVPacket *pkt)
{
    IpmxUsbSenderContext *ctx = s->priv_data;
    AVThreadMessageQueue *q;
    SenderQueueMsg qmsg;
    int ret;

    if (ctx->stop)
        return AVERROR_EOF;

    /* No receiver connected yet — discard the packet to keep the pipeline
     * running in real-time without accumulating stale data. */
    if (!ctx->data_ch_ready)
        return 0;

    if (pkt->stream_index == ctx->video_stream_idx) {
        q = ctx->video_queue;
        ctx->diag_wp_video++;
    } else if (pkt->stream_index == ctx->audio_stream_idx) {
        q = ctx->audio_queue;
        ctx->diag_wp_audio++;
    } else {
        return 0;
    }

    if (!q)
        return 0;

    qmsg.pkt = av_packet_clone(pkt);
    if (!qmsg.pkt)
        return AVERROR(ENOMEM);

    ipmx_timing_stamp_pkt(qmsg.pkt);

    if (ctx->timing_fp) {
        int64_t enq_us = av_gettime_relative();
        int is_v = (pkt->stream_index == ctx->video_stream_idx);
        fprintf(ctx->timing_fp,
                "%lld\tENQ\t%c\t%lld\t%lld\t%lld\n",
                (long long)enq_us,
                is_v ? 'V' : 'A',
                (long long)(is_v ? ctx->timing_enq_vid : ctx->timing_enq_aud),
                (long long)enq_us,
                (long long)pkt->pts);
        if (is_v) ctx->timing_enq_vid++;
        else      ctx->timing_enq_aud++;
    }

    ret = av_thread_message_queue_send(q, &qmsg, 0);
    if (ret < 0) {
        av_packet_free(&qmsg.pkt);
        return (ret == AVERROR_EOF) ? ret : 0;
    }

    return 0;
}

/* ---------------------------------------------------------------------------
 * write_trailer: signal shutdown and join
 * --------------------------------------------------------------------------- */

static int ipmx_usb_sender_write_trailer(AVFormatContext *s)
{
    IpmxUsbSenderContext *ctx = s->priv_data;

    ctx->stop = 1;

    if (ctx->video_queue)
        av_thread_message_queue_set_err_recv(ctx->video_queue, AVERROR_EOF);
    if (ctx->audio_queue)
        av_thread_message_queue_set_err_recv(ctx->audio_queue, AVERROR_EOF);

    if (ctx->proto_thread_started) {
        pthread_join(ctx->proto_thread, NULL);
        ctx->proto_thread_started = 0;
    }

    return 0;
}

/* ---------------------------------------------------------------------------
 * deinit: cleanup all resources
 * --------------------------------------------------------------------------- */

static void ipmx_usb_sender_deinit(AVFormatContext *s)
{
    IpmxUsbSenderContext *ctx = s->priv_data;
    int i;

    ctx->stop = 1;

    if (ctx->video_queue) {
        av_thread_message_queue_set_err_recv(ctx->video_queue, AVERROR_EOF);
        av_thread_message_queue_set_err_send(ctx->video_queue, AVERROR_EOF);
    }
    if (ctx->audio_queue) {
        av_thread_message_queue_set_err_recv(ctx->audio_queue, AVERROR_EOF);
        av_thread_message_queue_set_err_send(ctx->audio_queue, AVERROR_EOF);
    }

    if (ctx->proto_thread_started) {
        pthread_join(ctx->proto_thread, NULL);
        ctx->proto_thread_started = 0;
    }

    for (i = 0; i < SENDER_MAX_DATA_CH; i++) {
        SenderDataChannel *ch = &ctx->data_ch[i];
        av_packet_free(&ch->current_video_pkt);
        av_freep(&ch->audio_residual);
        ch->audio_residual_len = 0;
        ch->audio_residual_cap = 0;
        for (int d = 0; d < DEFERRED_SLOTS; d++) {
            av_freep(&ch->deferred[d].buf);
            ch->deferred[d].valid = 0;
        }
        if (ch->sock >= 0) {
            close(ch->sock);
            ch->sock = -1;
        }
        av_freep(&ch->recv_buf);
    }

    if (ctx->ctrl_sock >= 0) {
        close(ctx->ctrl_sock);
        ctx->ctrl_sock = -1;
    }
    if (ctx->ctrl_listen_sock >= 0) {
        close(ctx->ctrl_listen_sock);
        ctx->ctrl_listen_sock = -1;
    }

    if (ctx->video_queue) {
        av_thread_message_flush(ctx->video_queue);
        av_thread_message_queue_free(&ctx->video_queue);
    }
    if (ctx->audio_queue) {
        av_thread_message_flush(ctx->audio_queue);
        av_thread_message_queue_free(&ctx->audio_queue);
    }

    if (ctx->timing_fp) {
        fclose(ctx->timing_fp);
        ctx->timing_fp = NULL;
    }
}

/* ---------------------------------------------------------------------------
 * Bypass FFmpeg's default DTS-based interleave buffering: packets from the
 * local capture are already in capture order and the sender has its own
 * per-stream queues, so the muxer interleave queue only adds latency.
 * --------------------------------------------------------------------------- */

static int ipmx_usb_sender_interleave_passthrough(AVFormatContext *s,
                                                    AVPacket *pkt,
                                                    int flush, int has_packet)
{
    return has_packet;
}

/* ---------------------------------------------------------------------------
 * AVOutputFormat registration
 * --------------------------------------------------------------------------- */

const FFOutputFormat ff_ipmx_usb_sender_muxer = {
    .p.name         = "ipmx_usb_sender",
    .p.long_name    = NULL_IF_CONFIG_SMALL("IPMX USB (TR-10-14) Sender device"),
    .priv_data_size = sizeof(IpmxUsbSenderContext),
    .p.audio_codec  = AV_CODEC_ID_PCM_S16LE,
    .p.video_codec  = AV_CODEC_ID_MJPEG,
    .p.flags        = AVFMT_NOFILE,
    .init           = ipmx_usb_sender_init,
    .write_header   = ipmx_usb_sender_write_header,
    .write_packet   = ipmx_usb_sender_write_packet,
    .write_trailer  = ipmx_usb_sender_write_trailer,
    .deinit         = ipmx_usb_sender_deinit,
    .interleave_packet = ipmx_usb_sender_interleave_passthrough,
    .p.priv_class   = &ipmx_usb_sender_class,
};
