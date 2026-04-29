/*
 * IPMX USB (TR-10-14) input device for FFmpeg.
 *
 * Connects to an IPMX USB Sender over TCP, performs USB enumeration,
 * negotiates UVC/UAC streams, and delivers video/audio AVPackets.
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

#include <pthread.h>
#include "libavformat/avformat.h"
#include "libavformat/demux.h"
#include "libavformat/internal.h"
#include "libavutil/mem.h"
#include "ipmx_usb_proto.h"
#include "ipmx_usb_pep.h"
#include "ipmx_usb_common.h"
#include "libavutil/opt.h"
#include "libavutil/time.h"
#include "libavutil/mathematics.h"
#include "libavutil/avstring.h"
#include "libavutil/log.h"
#include "libavutil/pixdesc.h"
#include "libavutil/threadmessage.h"
#include "avdevice.h"
#include "ipmx_usb_uvc.h"
#include "ipmx_usb_uac.h"
#include "ipmx_usb_quirks.h"
#include "ipmx_timing.h"
#include "ipmx_usb_clockrec.h"

#include <openssl/rand.h>

#include <math.h>
#include <poll.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>

/* USB standard descriptor types */
#define USB_DT_DEVICE        0x01
#define USB_DT_CONFIG        0x02
#define USB_DT_STRING        0x03
#define USB_DT_INTERFACE     0x04
#define USB_DT_ENDPOINT      0x05

/* USB standard request types */
#define USB_REQ_GET_DESCRIPTOR   0x06
#define USB_REQ_SET_ADDRESS      0x05
#define USB_REQ_SET_CONFIG       0x09
#define USB_REQ_SET_INTERFACE    0x0B

/* USB request type fields */
#define USB_DIR_IN               0x80
#define USB_DIR_OUT              0x00
#define USB_TYPE_STANDARD        0x00
#define USB_TYPE_CLASS           0x20
#define USB_RECIP_DEVICE         0x00
#define USB_RECIP_INTERFACE      0x01

#define DEFAULT_HBEAT_INDEX      10    /* Section 11: valid range [5, 30] */
#define RECONNECT_DELAY_US       3000000  /* 3 seconds between reconnect attempts */
#define CTRL_MONITOR_POLL_MS     1000
#define CTRL_TIMEOUT_MS          5000
#define DATA_ACCEPT_TIMEOUT_MS   10000
#define ISO_TIMEOUT_MS           1000
#define DATA_IDLE_TIMEOUT_US     10000000  /* 10 seconds without data → EOF */
#define DEFAULT_DATA_PORT        49152

#define RECV_BUF_SIZE            (256 * 1024)
#define BUILD_BUF_SIZE           131072
#define CONFIG_DESC_MAX          4096
#define PCM_BUF_SIZE             (48000 * 8 * 4)  /* ~1s at 48kHz, 8ch, 32-bit */

/* ISO packet scheduling parameters */
#define ISO_VIDEO_NUM_PACKETS    32
#define ISO_AUDIO_NUM_PACKETS    8
#define ISO_MAX_PACKET_SIZE      3072

/* Forward-declare the per-channel struct used by function prototypes below */
typedef struct IpmxUsbDataChannel IpmxUsbDataChannel;

/* ---------------------------------------------------------------------------
 * Forward declarations
 * --------------------------------------------------------------------------- */

static int ctrl_transfer(AVFormatContext *s, IpmxUsbDataChannel *ch,
                         int direction, uint8_t endpoint,
                         const uint8_t *setup, int setup_len,
                         const uint8_t *data_out, int data_out_len,
                         uint8_t *data_in, int *data_in_len);
static int usb_enumerate(AVFormatContext *s, IpmxUsbDataChannel *ch);
static void *ctrl_monitor_thread(void *arg);
static void *channel_io_thread(void *arg);
static int reconnect_session(AVFormatContext *s);
static int poll_channel(AVFormatContext *s, IpmxUsbDataChannel *ch,
                        AVPacket *pkt);
static int ipmx_usb_read_header(AVFormatContext *s);
static int ipmx_usb_read_packet(AVFormatContext *s, AVPacket *pkt);
static int ipmx_usb_read_close(AVFormatContext *s);

/* ---------------------------------------------------------------------------
 * Private context
 * --------------------------------------------------------------------------- */

#define MAX_DATA_CHANNELS 8

struct IpmxUsbDataChannel {
    int sock;
    uint16_t substreamid;
    uint32_t seqnum;
    int handshake_done;

    /* Per-channel recv buffer (heap-allocated in read_header) */
    uint8_t *recv_buf;
    int      recv_buf_len;
    int      recv_buf_pos;

    /* Per-channel PEP CTR counters */
    uint64_t ctr_s2r;
    uint64_t ctr_r2s;
    IpmxPepForwardProgress fp_s2r;

    /* USB device state */
    IpmxUvcContext uvc;
    IpmxUacContext uac;
    int has_video;
    int has_audio;
    int video_stream_idx;
    int audio_stream_idx;

    /* USB frame number from last ISO_SUBMIT_RETURN (in ms, converted to μs) */
    int64_t  last_startframe_abs_us;

    /* Per-stream STARTFRAME epoch: first non-zero STARTFRAME seen,
     * subtracted from all subsequent values so pkt->pts starts near 0. */
    int64_t  video_sf_epoch_us;
    int      video_sf_epoch_set;
    int64_t  audio_sf_epoch_us;
    int      audio_sf_epoch_set;

    /* Last UVC SCR extracted from completed video frames */
    uint32_t last_scr_stc;
    uint16_t last_scr_sof;
    int      last_has_scr;

    /* STARTFRAME of the first URB contributing to the current video frame.
     * Per TR-10-14, all isochronous transfers for a single video frame
     * should share the same STARTFRAME for PTS computation.  Since USBIP
     * assigns a unique STARTFRAME per URB, we latch the first URB's value
     * when UVC reassembly begins a new frame and use it at completion. */
    uint32_t video_frame_first_startframe;
    int      video_frame_first_startframe_set;

    /* Wire-level values from the last completed video frame (for timing log) */
    uint32_t last_wire_uvc_pts;
    int      last_wire_has_uvc_pts;
    uint32_t last_wire_startframe;

    /* Device descriptor fields */
    uint16_t dev_vid;
    uint16_t dev_pid;

    /* Configuration descriptor blob */
    uint8_t  config_desc[CONFIG_DESC_MAX];
    int      config_desc_len;

    /* Pending ISO state */
    int video_iso_pending;
    int audio_iso_pending;

    /* PTS counters */
    int64_t video_frame_count;
    int64_t audio_sample_count;

    /* EOF detection: time of last non-empty ISO return (microseconds) */
    int64_t last_data_time;

    /* Per-channel I/O thread (producer for pkt_queue) */
    pthread_t    io_thread;
    int          io_thread_started;
    volatile int io_thread_running;
};

typedef enum {
    STARTUP_FILL,
    STARTUP_LATCHED,
    STARTUP_STEADY,
} StartupState;

/* Pacer slot window.  The scheduler maintains 2 * nb_streams generic
 * slots as a sliding window into the input queue.  Packets are dequeued
 * only when an empty slot exists — no packet is ever lost.  A 500 us
 * poll loop delivers whichever slot has the earliest PTS. */
#define PACER_SLOT_FACTOR 2           /* slots = factor * nb_streams */
#define PACER_MAX_SLOTS   8           /* hard ceiling (2 * 4 streams) */
#define PACER_POLL_US     500

typedef struct PacerSlot {
    AVPacket *pkt;      /* NULL = slot empty */
} PacerSlot;

typedef struct IpmxUsbContext {
    AVClass *class;

    /* CLI options */
    char *sender;
    char *psk_hex;
    char *sdp_path;
    char *serial;
    char *video_size_str;
    char *framerate_str;
    char *stream_type;
    int sample_rate;
    char *pep_mode_str;
    char *pep_iv_str;
    char *pep_key_generator_str;
    char *pep_key_version_str;
    int strict_pep;
    int hbeat_index;
    int low_latency;

    /* Network state */
    int ctrl_sock;
    int data_listen_sock;
    int data_port;

    /* Control channel recv buffer */
    uint8_t  ctrl_recv_buf[RECV_BUF_SIZE];
    int      ctrl_recv_buf_len;
    int      ctrl_recv_buf_pos;

    /* Resolved Sender address (used for reconnect) */
    char sender_host[256];
    int  sender_port;

    /* SDP-derived sender address (fallback when -sender not given) */
    char sdp_host[256];
    int  sdp_port;

    /* PEP (shared across channels) */
    IpmxUsbPepState pep;
    uint64_t ctrl_ctr_s2r;
    uint64_t ctrl_ctr_r2s;
    IpmxPepForwardProgress ctrl_fp_s2r;

    /* Data channels */
    IpmxUsbDataChannel channels[MAX_DATA_CHANNELS];
    int num_channels;

    /* Control monitor thread (replaces heartbeat_thread) */
    pthread_t ctrl_monitor;
    int ctrl_monitor_running;

    /* Heartbeat tracking (Section 11) */
    int64_t last_hbeat_us;
    double  hbeat_period_s;
    int     reconnect_attempt;

    /* Shared packet queue: channel threads produce, read_packet consumes */
    AVThreadMessageQueue *pkt_queue;
    pthread_mutex_t audio_rate_mutex;
    int64_t         audio_rx_samples;
    int64_t         audio_tx_samples;

    /* Active I/O thread count; when it reaches 0, queue signals EOF */
    volatile int io_threads_active;
    pthread_mutex_t io_threads_mutex;

    /* Shutdown */
    int stop;

    /* Diagnostics */
    int64_t diag_vid_frames;
    int64_t diag_aud_frames;
    int64_t diag_vid_iso_sent;
    int64_t diag_aud_iso_sent;
    int64_t diag_empty_returns;
    int64_t diag_next_log;
    FILE   *diag_fp;

    /* Timing instrumentation (IPMX_TIMING=1) */
    FILE   *timing_fp;
    int64_t timing_rcv_vid;
    int64_t timing_rcv_aud;
    int64_t timing_deq_cnt;

    /* Clock recovery / latency measurement */
    IpmxClockRecovery clockrec;
    int latency_target_ms;

    /* PTS-based pacer: with the affine model, PTS is an absolute
     * receiver wall-clock delivery time.  The pacer sleeps until PTS,
     * then converts PTS back to relative for ffplay (which expects
     * PTS starting near 0 for A/V sync). */
    int64_t video_latency_target_us;
    int64_t audio_latency_target_us;
    int64_t pts_epoch;             /* first calibrated absolute PTS seen */
    int     pts_epoch_set;         /* 1 once pts_epoch is captured */

    /* Legacy startup state (kept for video repeat guard) */
    StartupState startup_state;
    int64_t startup_target_us;

    /* Last video packet stash for frame repeat */
    AVPacket *last_video_pkt;

    /* Pacer slot window */
    PacerSlot pacer_slots[PACER_MAX_SLOTS];
    int       pacer_n_slots;   /* actual count: min(2*nb_streams, MAX) */
} IpmxUsbContext;

typedef struct ChannelThreadArg {
    AVFormatContext    *s;
    IpmxUsbDataChannel *ch;
} ChannelThreadArg;

/* ---------------------------------------------------------------------------
 * Options
 * --------------------------------------------------------------------------- */

static const AVOption ipmx_usb_options[] = {
    { "sender",     "Sender IP:port",             offsetof(IpmxUsbContext, sender),
      AV_OPT_TYPE_STRING, {.str = NULL}, 0, 0, AV_OPT_FLAG_DECODING_PARAM },
    { "psk",        "Pre-shared key (hex)",       offsetof(IpmxUsbContext, psk_hex),
      AV_OPT_TYPE_STRING, {.str = NULL}, 0, 0, AV_OPT_FLAG_DECODING_PARAM },
    { "sdp",        "SDP file path",              offsetof(IpmxUsbContext, sdp_path),
      AV_OPT_TYPE_STRING, {.str = NULL}, 0, 0, AV_OPT_FLAG_DECODING_PARAM },
    { "serial",     "Device serial number",       offsetof(IpmxUsbContext, serial),
      AV_OPT_TYPE_STRING, {.str = NULL}, 0, 0, AV_OPT_FLAG_DECODING_PARAM },
    { "video_size", "Preferred resolution",       offsetof(IpmxUsbContext, video_size_str),
      AV_OPT_TYPE_STRING, {.str = NULL}, 0, 0, AV_OPT_FLAG_DECODING_PARAM },
    { "framerate",  "Preferred frame rate",       offsetof(IpmxUsbContext, framerate_str),
      AV_OPT_TYPE_STRING, {.str = NULL}, 0, 0, AV_OPT_FLAG_DECODING_PARAM },
    { "stream_type","Stream type: video, audio, both",
      offsetof(IpmxUsbContext, stream_type),
      AV_OPT_TYPE_STRING, {.str = "both"}, 0, 0, AV_OPT_FLAG_DECODING_PARAM },
    { "sample_rate","Preferred audio sample rate",
      offsetof(IpmxUsbContext, sample_rate),
      AV_OPT_TYPE_INT, {.i64 = 48000}, 8000, 384000, AV_OPT_FLAG_DECODING_PARAM },
    { "pep_mode",   "PEP mode override (e.g. AES-128-CTR_CMAC-64-AAD)",
      offsetof(IpmxUsbContext, pep_mode_str),
      AV_OPT_TYPE_STRING, {.str = NULL}, 0, 0, AV_OPT_FLAG_DECODING_PARAM },
    { "pep_iv",     "PEP base IV override (hex, 16 chars)",
      offsetof(IpmxUsbContext, pep_iv_str),
      AV_OPT_TYPE_STRING, {.str = NULL}, 0, 0, AV_OPT_FLAG_DECODING_PARAM },
    { "pep_key_generator", "PEP key_generator override (hex, 32 chars)",
      offsetof(IpmxUsbContext, pep_key_generator_str),
      AV_OPT_TYPE_STRING, {.str = NULL}, 0, 0, AV_OPT_FLAG_DECODING_PARAM },
    { "pep_key_version",   "PEP sender key_version override (hex, 8 chars)",
      offsetof(IpmxUsbContext, pep_key_version_str),
      AV_OPT_TYPE_STRING, {.str = NULL}, 0, 0, AV_OPT_FLAG_DECODING_PARAM },
    { "strict_pep", "Strict PEP forward-progress enforcement (close channel on violation)",
      offsetof(IpmxUsbContext, strict_pep),
      AV_OPT_TYPE_BOOL, {.i64 = 0}, 0, 1, AV_OPT_FLAG_DECODING_PARAM },
    { "hbeat_index", "Heartbeat rate index per Section 11 (period = 5 * 1.25^index seconds)",
      offsetof(IpmxUsbContext, hbeat_index),
      AV_OPT_TYPE_INT, {.i64 = DEFAULT_HBEAT_INDEX}, 5, 30, AV_OPT_FLAG_DECODING_PARAM },
    { "clock_recovery", "Enable adaptive clock recovery (audio resampling for drift)",
      offsetof(IpmxUsbContext, clockrec.clock_recovery),
      AV_OPT_TYPE_BOOL, {.i64 = 0}, 0, 1, AV_OPT_FLAG_DECODING_PARAM },
    { "latency_report", "Enable direct pipeline latency measurement logging",
      offsetof(IpmxUsbContext, clockrec.latency_report),
      AV_OPT_TYPE_BOOL, {.i64 = 0}, 0, 1, AV_OPT_FLAG_DECODING_PARAM },
    { "latency_target_ms", "Target latency in ms (0=auto: 100ms default, 33ms low-latency)",
      offsetof(IpmxUsbContext, latency_target_ms),
      AV_OPT_TYPE_INT, {.i64 = 0}, 0, 5000, AV_OPT_FLAG_DECODING_PARAM },
    { "low_latency", "Enable low-latency mode (33ms startup target)",
      offsetof(IpmxUsbContext, low_latency),
      AV_OPT_TYPE_BOOL, {.i64 = 0}, 0, 1, AV_OPT_FLAG_DECODING_PARAM },
    { "clock_sim_drift_ppm", "Simulated receiver clock drift in PPM (for testing)",
      offsetof(IpmxUsbContext, clockrec.clock_sim_drift_ppm),
      AV_OPT_TYPE_DOUBLE, {.dbl = 0.0}, -1000.0, 1000.0, AV_OPT_FLAG_DECODING_PARAM },
    { "clock_sim_offset_ms", "Simulated receiver clock phase offset in ms (for testing)",
      offsetof(IpmxUsbContext, clockrec.clock_sim_offset_ms),
      AV_OPT_TYPE_INT, {.i64 = 0}, -60000, 60000, AV_OPT_FLAG_DECODING_PARAM },
    { NULL },
};

static const AVClass ipmx_usb_class = {
    .class_name = "ipmx_usb indev",
    .item_name  = av_default_item_name,
    .option     = ipmx_usb_options,
    .version    = LIBAVUTIL_VERSION_INT,
    .category   = AV_CLASS_CATEGORY_DEVICE_VIDEO_INPUT,
};

/* ---------------------------------------------------------------------------
 * USB control transfer over TR-10-14 data channel
 * --------------------------------------------------------------------------- */

static int ctrl_transfer(AVFormatContext *s, IpmxUsbDataChannel *ch,
                         int direction, uint8_t endpoint,
                         const uint8_t *setup, int setup_len,
                         const uint8_t *data_out, int data_out_len,
                         uint8_t *data_in, int *data_in_len)
{
    IpmxUsbContext *ctx = s->priv_data;
    IpmxUsbPepState *pep = ctx->pep.encrypted ? &ctx->pep : NULL;
    uint8_t *build_buf;
    IpmxUsbMsg reply;
    int ret, len;
    uint32_t seq = ch->seqnum++;
    uint32_t transfer_len = 0;

    build_buf = av_malloc(BUILD_BUF_SIZE);
    if (!build_buf)
        return AVERROR(ENOMEM);

    if (direction == 0 && data_out)
        transfer_len = data_out_len;
    else if (direction == 1 && data_in_len)
        transfer_len = *data_in_len;

    len = ipmx_usb_build_control_submit(build_buf, BUILD_BUF_SIZE,
                                        seq, endpoint,
                                        direction ? 1 : 0,
                                        0, transfer_len,
                                        setup,
                                        data_out, data_out_len);
    if (len < 0) {
        av_log(s, AV_LOG_ERROR, "Failed to build CONTROL_SUBMIT\n");
        av_free(build_buf);
        return AVERROR_EXIT;
    }

    ret = ipmx_usb_send_message(s, pep, ch->sock, ch->substreamid | 1,
                                &ch->ctr_r2s, build_buf, len);
    av_free(build_buf);
    if (ret < 0)
        return ret;

    for (;;) {
        ret = ipmx_usb_recv_message(s, pep, ch->sock,
                                    ch->recv_buf, &ch->recv_buf_len,
                                    &ch->recv_buf_pos,
                                    ch->substreamid, &ch->ctr_s2r,
                                    &ch->fp_s2r,
                                    &reply, CTRL_TIMEOUT_MS);
        if (ret == IPMX_USB_ERR_MAC_FAIL) {
            if (ch->fp_s2r.strict)
                return AVERROR(EPROTO);
            av_log(s, AV_LOG_WARNING,
                   "CONTROL_SUBMIT_RETURN MAC failed (seq=%u) — "
                   "treating as REQUEST_FAILED\n", seq);
            return AVERROR(EIO);
        }
        if (ret < 0) {
            av_log(s, AV_LOG_ERROR,
                   "Timeout waiting for CONTROL_SUBMIT_RETURN (seq=%u)\n",
                   seq);
            return ret;
        }

        if (reply.hdr.msg_type == IPMX_USB_CONTROL_SUBMIT_RETURN &&
            reply.payload.control_submit_return.seqnum == seq)
            break;

        av_log(s, AV_LOG_DEBUG,
               "Ignoring msg_type=0x%02x while waiting for ctrl return\n",
               reply.hdr.msg_type);
    }

    if (reply.payload.control_submit_return.rstatus != 0) {
        int is_class_req = (setup_len >= 1 && (setup[0] & 0x60) == 0x20);
        if (is_class_req && (ch->uac.quirks & IPMX_UAC_QUIRK_IGNORE_CTL_ERROR)) {
            av_log(s, AV_LOG_WARNING,
                   "CONTROL_SUBMIT_RETURN status=0x%08x for seq=%u (ignored, quirk)\n",
                   reply.payload.control_submit_return.rstatus, seq);
            if (direction == 1 && data_in_len)
                *data_in_len = 0;
            return 0;
        }
        av_log(s, AV_LOG_WARNING,
               "CONTROL_SUBMIT_RETURN status=0x%08x for seq=%u\n",
               reply.payload.control_submit_return.rstatus, seq);
        return AVERROR(EIO);
    }

    if (direction == 1 && data_in && data_in_len) {
        int copy_len = FFMIN(reply.payload.control_submit_return.transferdata_len,
                             *data_in_len);
        av_log(s, AV_LOG_DEBUG,
               "ctrl_xfer: actuallength=%u xfr_data_len=%d req_len=%d copy_len=%d raw_len=%d\n",
               reply.payload.control_submit_return.actuallength,
               reply.payload.control_submit_return.transferdata_len,
               *data_in_len, copy_len, reply.raw_len);
        if (copy_len > 0 && reply.payload.control_submit_return.transferdata)
            memcpy(data_in, reply.payload.control_submit_return.transferdata,
                   copy_len);
        *data_in_len = copy_len;
    }

    return 0;
}

/* ---------------------------------------------------------------------------
 * USB setup packet helpers
 * --------------------------------------------------------------------------- */

static void build_get_descriptor_setup(uint8_t *setup, uint8_t desc_type,
                                       uint8_t desc_index, uint16_t lang_id,
                                       uint16_t length)
{
    setup[0] = USB_DIR_IN | USB_TYPE_STANDARD | USB_RECIP_DEVICE;
    setup[1] = USB_REQ_GET_DESCRIPTOR;
    setup[2] = desc_index;
    setup[3] = desc_type;
    setup[4] = lang_id & 0xFF;
    setup[5] = (lang_id >> 8) & 0xFF;
    setup[6] = length & 0xFF;
    setup[7] = (length >> 8) & 0xFF;
}

static void build_set_address_setup(uint8_t *setup, uint16_t address)
{
    setup[0] = USB_DIR_OUT | USB_TYPE_STANDARD | USB_RECIP_DEVICE;
    setup[1] = USB_REQ_SET_ADDRESS;
    setup[2] = address & 0xFF;
    setup[3] = (address >> 8) & 0xFF;
    setup[4] = 0;
    setup[5] = 0;
    setup[6] = 0;
    setup[7] = 0;
}

static void build_set_configuration_setup(uint8_t *setup, uint16_t config)
{
    setup[0] = USB_DIR_OUT | USB_TYPE_STANDARD | USB_RECIP_DEVICE;
    setup[1] = USB_REQ_SET_CONFIG;
    setup[2] = config & 0xFF;
    setup[3] = (config >> 8) & 0xFF;
    setup[4] = 0;
    setup[5] = 0;
    setup[6] = 0;
    setup[7] = 0;
}

static void build_set_interface_setup(uint8_t *setup, uint16_t interface,
                                      uint16_t alt_setting)
{
    setup[0] = USB_DIR_OUT | USB_TYPE_STANDARD | USB_RECIP_INTERFACE;
    setup[1] = USB_REQ_SET_INTERFACE;
    setup[2] = alt_setting & 0xFF;
    setup[3] = (alt_setting >> 8) & 0xFF;
    setup[4] = interface & 0xFF;
    setup[5] = (interface >> 8) & 0xFF;
    setup[6] = 0;
    setup[7] = 0;
}

/* ---------------------------------------------------------------------------
 * USB GET_DESCRIPTOR helper (wraps ctrl_transfer)
 * --------------------------------------------------------------------------- */

static int usb_get_descriptor(AVFormatContext *s, IpmxUsbDataChannel *ch,
                              uint8_t desc_type, uint8_t desc_index,
                              uint16_t lang_id,
                              uint8_t *buf, int buf_size, int *out_len)
{
    uint8_t setup[8];
    int xfer_len = buf_size;
    int ret;

    build_get_descriptor_setup(setup, desc_type, desc_index, lang_id,
                               (uint16_t)buf_size);

    ret = ctrl_transfer(s, ch, 1, 0, setup, 8, NULL, 0, buf, &xfer_len);
    if (ret < 0)
        return ret;

    *out_len = xfer_len;
    return 0;
}

/* ---------------------------------------------------------------------------
 * USB enumeration
 * --------------------------------------------------------------------------- */

static int usb_enumerate(AVFormatContext *s, IpmxUsbDataChannel *ch)
{
    uint8_t desc_buf[512];
    uint8_t setup[8];
    int desc_len, ret;
    uint16_t total_len, lang_id;
    int i;

    av_log(s, AV_LOG_INFO, "Starting USB enumeration (ch ssid=%u)\n",
           ch->substreamid);

    ret = usb_get_descriptor(s, ch, USB_DT_DEVICE, 0, 0,
                             desc_buf, 8, &desc_len);
    if (ret < 0) {
        av_log(s, AV_LOG_ERROR, "Failed partial device descriptor read\n");
        return ret;
    }
    av_log(s, AV_LOG_DEBUG, "Partial device descriptor: %d bytes\n", desc_len);

    build_set_address_setup(setup, 5);
    ret = ctrl_transfer(s, ch, 0, 0, setup, 8, NULL, 0, NULL, NULL);
    if (ret < 0) {
        av_log(s, AV_LOG_ERROR, "SET_ADDRESS failed\n");
        return ret;
    }
    av_log(s, AV_LOG_DEBUG, "SET_ADDRESS(5) done\n");

    ret = usb_get_descriptor(s, ch, USB_DT_DEVICE, 0, 0,
                             desc_buf, 18, &desc_len);
    if (ret < 0) {
        av_log(s, AV_LOG_ERROR, "Failed full device descriptor read\n");
        return ret;
    }
    if (desc_len >= 10) {
        ch->dev_vid = desc_buf[8] | (desc_buf[9] << 8);
        ch->dev_pid = desc_buf[10] | (desc_buf[11] << 8);
        av_log(s, AV_LOG_INFO, "USB device VID:PID = %04x:%04x\n",
               ch->dev_vid, ch->dev_pid);
    }

    total_len = 0;
    ret = usb_get_descriptor(s, ch, USB_DT_CONFIG, 0, 0,
                             desc_buf, 9, &desc_len);
    if (ret < 0) {
        av_log(s, AV_LOG_ERROR, "Failed partial config descriptor read\n");
        return ret;
    }
    if (desc_len >= 4)
        total_len = desc_buf[2] | (desc_buf[3] << 8);
    if (total_len < 9 || total_len > CONFIG_DESC_MAX) {
        av_log(s, AV_LOG_ERROR, "Invalid config descriptor wTotalLength=%u\n",
               total_len);
        return AVERROR(EIO);
    }
    av_log(s, AV_LOG_DEBUG, "Config descriptor wTotalLength=%u\n", total_len);

    ret = usb_get_descriptor(s, ch, USB_DT_CONFIG, 0, 0,
                             ch->config_desc, total_len,
                             &ch->config_desc_len);
    if (ret < 0) {
        av_log(s, AV_LOG_ERROR, "Failed full config descriptor read\n");
        return ret;
    }
    av_log(s, AV_LOG_WARNING, "Full config descriptor: %d bytes (requested %u)\n",
           ch->config_desc_len, total_len);

    ret = usb_get_descriptor(s, ch, USB_DT_STRING, 0, 0,
                             desc_buf, 255, &desc_len);
    if (ret < 0)
        av_log(s, AV_LOG_DEBUG, "String descriptor 0 not available\n");

    lang_id = 0x0409;
    if (desc_len >= 4)
        lang_id = desc_buf[2] | (desc_buf[3] << 8);

    for (i = 1; i <= 3; i++) {
        ret = usb_get_descriptor(s, ch, USB_DT_STRING, i, lang_id,
                                 desc_buf, 255, &desc_len);
        if (ret < 0) {
            av_log(s, AV_LOG_DEBUG, "String descriptor %d not available\n", i);
            continue;
        }
        if (desc_len > 2) {
            char str_utf8[128];
            int str_pos = 0, j;
            for (j = 2; j < desc_len - 1 && str_pos < (int)sizeof(str_utf8) - 1; j += 2) {
                uint16_t wchar = desc_buf[j] | (desc_buf[j + 1] << 8);
                if (wchar < 0x80)
                    str_utf8[str_pos++] = (char)wchar;
            }
            str_utf8[str_pos] = '\0';
            av_log(s, AV_LOG_INFO, "String[%d]: %s\n", i, str_utf8);
        }
    }

    build_set_configuration_setup(setup, 1);
    ret = ctrl_transfer(s, ch, 0, 0, setup, 8, NULL, 0, NULL, NULL);
    if (ret < 0) {
        av_log(s, AV_LOG_ERROR, "SET_CONFIGURATION(1) failed\n");
        return ret;
    }
    av_log(s, AV_LOG_DEBUG, "SET_CONFIGURATION(1) done\n");

    ret = ipmx_uvc_parse_config(&ch->uvc, ch->config_desc,
                                ch->config_desc_len);
    if (ret < 0)
        av_log(s, AV_LOG_INFO, "No UVC interface found in descriptors\n");
    else
        av_log(s, AV_LOG_INFO,
               "UVC: vc_iface=%d vs_iface=%d ep=0x%02x formats=%d "
               "dwClockFrequency=%u Hz\n",
               ch->uvc.vc_interface, ch->uvc.vs_interface,
               ch->uvc.vs_endpoint, ch->uvc.num_formats,
               ch->uvc.clock_frequency);

    ret = ipmx_uac_parse_config(&ch->uac, ch->config_desc,
                                ch->config_desc_len);
    if (ret < 0)
        av_log(s, AV_LOG_INFO, "No UAC interface found in descriptors\n");
    else
        av_log(s, AV_LOG_INFO,
               "UAC: ac_iface=%d as_iface=%d ep=0x%02x ch=%d rate=%u\n",
               ch->uac.ac_interface, ch->uac.as_interface,
               ch->uac.as_endpoint, ch->uac.channels,
               ch->uac.sample_rates[0]);

    ch->uvc.quirks = ipmx_uvc_quirks_lookup(ch->dev_vid, ch->dev_pid);
    ch->uac.quirks = ipmx_uac_quirks_lookup(ch->dev_vid, ch->dev_pid);
    if (ch->uvc.quirks)
        av_log(s, AV_LOG_INFO, "UVC quirks: 0x%04x\n", ch->uvc.quirks);
    if (ch->uac.quirks)
        av_log(s, AV_LOG_INFO, "UAC quirks: 0x%04x\n", ch->uac.quirks);

    return 0;
}

/* ---------------------------------------------------------------------------
 * UVC probe/commit negotiation
 * --------------------------------------------------------------------------- */

static int uvc_negotiate(AVFormatContext *s, IpmxUsbDataChannel *ch)
{
    uint8_t probe_data[26];
    uint8_t setup[8];
    int probe_len, ret;

    if (ch->uvc.quirks & IPMX_UVC_QUIRK_PROBE_DEF) {
        ipmx_uvc_build_get_def_req(setup, UVC_VS_PROBE_CONTROL,
                                    ch->uvc.vs_interface, 26);
        probe_len = 26;
        ret = ctrl_transfer(s, ch, 1, 0, setup, 8, NULL, 0,
                            probe_data, &probe_len);
        if (ret >= 0 && probe_len == 26) {
            av_log(s, AV_LOG_DEBUG, "GET_DEF(PROBE) returned %d bytes (quirk)\n",
                   probe_len);
        } else {
            av_log(s, AV_LOG_DEBUG, "GET_DEF(PROBE) failed, using built probe\n");
            ret = ipmx_uvc_build_probe_control(probe_data, sizeof(probe_data),
                                               ch->uvc.active_format,
                                               ch->uvc.active_frame,
                                               ch->uvc.active_interval);
            if (ret < 0) {
                av_log(s, AV_LOG_ERROR, "Failed to build probe control\n");
                return AVERROR_EXIT;
            }
        }
    } else if (ch->uvc.quirks & IPMX_UVC_QUIRK_PROBE_MINMAX) {
        uint8_t min_data[26], max_data[26];
        uint8_t fmt_clamp, frame_clamp;
        uint32_t interval_min, interval_max, interval_clamp;

        ipmx_uvc_build_get_min_req(setup, UVC_VS_PROBE_CONTROL,
                                    ch->uvc.vs_interface, 26);
        probe_len = 26;
        ret = ctrl_transfer(s, ch, 1, 0, setup, 8, NULL, 0, min_data, &probe_len);
        if (ret < 0 || probe_len != 26) {
            av_log(s, AV_LOG_DEBUG, "GET_MIN(PROBE) failed, using built probe\n");
            ret = ipmx_uvc_build_probe_control(probe_data, sizeof(probe_data),
                                               ch->uvc.active_format,
                                               ch->uvc.active_frame,
                                               ch->uvc.active_interval);
            if (ret < 0) {
                av_log(s, AV_LOG_ERROR, "Failed to build probe control\n");
                return AVERROR_EXIT;
            }
        } else {
            ipmx_uvc_build_get_max_req(setup, UVC_VS_PROBE_CONTROL,
                                        ch->uvc.vs_interface, 26);
            probe_len = 26;
            ret = ctrl_transfer(s, ch, 1, 0, setup, 8, NULL, 0, max_data, &probe_len);
            if (ret < 0 || probe_len != 26) {
                av_log(s, AV_LOG_DEBUG, "GET_MAX(PROBE) failed, using built probe\n");
                ret = ipmx_uvc_build_probe_control(probe_data, sizeof(probe_data),
                                                   ch->uvc.active_format,
                                                   ch->uvc.active_frame,
                                                   ch->uvc.active_interval);
                if (ret < 0) {
                    av_log(s, AV_LOG_ERROR, "Failed to build probe control\n");
                    return AVERROR_EXIT;
                }
            } else {
                interval_min = min_data[4] | ((uint32_t)min_data[5] << 8) |
                              ((uint32_t)min_data[6] << 16) | ((uint32_t)min_data[7] << 24);
                interval_max = max_data[4] | ((uint32_t)max_data[5] << 8) |
                              ((uint32_t)max_data[6] << 16) | ((uint32_t)max_data[7] << 24);
                fmt_clamp = (uint8_t)av_clip(ch->uvc.active_format, min_data[2], max_data[2]);
                frame_clamp = (uint8_t)av_clip(ch->uvc.active_frame, min_data[3], max_data[3]);
                interval_clamp = (uint32_t)av_clip((int64_t)ch->uvc.active_interval,
                                                   (int64_t)interval_min,
                                                   (int64_t)interval_max);
                ret = ipmx_uvc_build_probe_control(probe_data, sizeof(probe_data),
                                                   fmt_clamp, frame_clamp,
                                                   (uint32_t)interval_clamp);
                if (ret < 0) {
                    av_log(s, AV_LOG_ERROR, "Failed to build probe control\n");
                    return AVERROR_EXIT;
                }
            }
        }
    } else {
        ret = ipmx_uvc_build_probe_control(probe_data, sizeof(probe_data),
                                           ch->uvc.active_format,
                                           ch->uvc.active_frame,
                                           ch->uvc.active_interval);
        if (ret < 0) {
            av_log(s, AV_LOG_ERROR, "Failed to build probe control\n");
            return AVERROR_EXIT;
        }
    }

    ipmx_uvc_build_set_cur_req(setup, UVC_VS_PROBE_CONTROL,
                               ch->uvc.vs_interface, 26);
    ret = ctrl_transfer(s, ch, 0, 0, setup, 8, probe_data, 26, NULL, NULL);
    if (ret < 0) {
        av_log(s, AV_LOG_ERROR, "SET_CUR(PROBE) failed\n");
        return ret;
    }
    av_log(s, AV_LOG_DEBUG, "SET_CUR(PROBE) done\n");

    ipmx_uvc_build_get_cur_req(setup, UVC_VS_PROBE_CONTROL,
                               ch->uvc.vs_interface, 26);
    probe_len = 26;
    ret = ctrl_transfer(s, ch, 1, 0, setup, 8, NULL, 0,
                        probe_data, &probe_len);
    if (ret < 0) {
        av_log(s, AV_LOG_ERROR, "GET_CUR(PROBE) failed\n");
        return ret;
    }
    av_log(s, AV_LOG_DEBUG, "GET_CUR(PROBE) returned %d bytes\n", probe_len);

    ipmx_uvc_build_set_cur_req(setup, UVC_VS_COMMIT_CONTROL,
                               ch->uvc.vs_interface, 26);
    ret = ctrl_transfer(s, ch, 0, 0, setup, 8, probe_data, 26, NULL, NULL);
    if (ret < 0) {
        av_log(s, AV_LOG_ERROR, "SET_CUR(COMMIT) failed\n");
        return ret;
    }
    av_log(s, AV_LOG_INFO, "UVC probe/commit negotiation complete\n");

    build_set_interface_setup(setup, ch->uvc.vs_interface, 1);
    ret = ctrl_transfer(s, ch, 0, 0, setup, 8, NULL, 0, NULL, NULL);
    if (ret < 0) {
        av_log(s, AV_LOG_ERROR, "SET_INTERFACE(%d, 1) failed\n",
               ch->uvc.vs_interface);
        return ret;
    }
    av_log(s, AV_LOG_DEBUG, "SET_INTERFACE(%d, 1) done\n",
           ch->uvc.vs_interface);

    return 0;
}

/* ---------------------------------------------------------------------------
 * UAC interface activation
 * --------------------------------------------------------------------------- */

static int uac_activate(AVFormatContext *s, IpmxUsbDataChannel *ch)
{
    uint8_t setup[8];
    uint32_t rate;
    int ret;

    build_set_interface_setup(setup, ch->uac.as_interface, 1);
    ret = ctrl_transfer(s, ch, 0, 0, setup, 8, NULL, 0, NULL, NULL);
    if (ret < 0) {
        av_log(s, AV_LOG_ERROR, "SET_INTERFACE(%d, 1) for audio failed\n",
               ch->uac.as_interface);
        return ret;
    }
    av_log(s, AV_LOG_INFO, "UAC streaming interface activated\n");

    if (ch->uac.uac_version == 2 && ch->uac.clock_source_id != 0 &&
        !(ch->uac.quirks & IPMX_UAC_QUIRK_SKIP_CLOCK_SELECTOR)) {
        /* UAC 2.0: negotiate sample rate via Clock Source entity */
        uint8_t range_buf[2 + 12];  /* wNumSubRanges(2) + one {dMIN,dMAX,dRES} triplet */
        int range_len = 0;

        ipmx_uac2_build_clock_get_range_req(setup, ch->uac.clock_source_id,
                                             ch->uac.ac_interface);
        ret = ctrl_transfer(s, ch, 0, 0, setup, 8, NULL, 0,
                            range_buf, &range_len);
        if (ret < 0 || range_len < 14) {
            av_log(s, AV_LOG_WARNING,
                   "UAC 2.0 GET_RANGE on Clock Source failed or short (%d bytes)\n",
                   range_len);
        } else {
            ch->uac.clock_freq_min = range_buf[2] | (range_buf[3] << 8) |
                                     (range_buf[4] << 16) | (range_buf[5] << 24);
            ch->uac.clock_freq_max = range_buf[6] | (range_buf[7] << 8) |
                                     (range_buf[8] << 16) | (range_buf[9] << 24);
            av_log(s, AV_LOG_INFO,
                   "UAC 2.0 Clock Source range: %u–%u Hz\n",
                   ch->uac.clock_freq_min, ch->uac.clock_freq_max);
        }

        rate = ch->uac.active_sample_rate;
        if (rate > 0) {
            uint8_t rate_data[4];
            rate_data[0] = (uint8_t)(rate);
            rate_data[1] = (uint8_t)(rate >> 8);
            rate_data[2] = (uint8_t)(rate >> 16);
            rate_data[3] = (uint8_t)(rate >> 24);

            ipmx_uac2_build_clock_set_cur_req(setup, ch->uac.clock_source_id,
                                               ch->uac.ac_interface);
            ret = ctrl_transfer(s, ch, 0, 0, setup, 8, rate_data, 4, NULL, NULL);
            if (ret < 0) {
                av_log(s, AV_LOG_WARNING,
                       "UAC 2.0 SET_CUR on Clock Source failed (non-fatal)\n");
            } else {
                av_log(s, AV_LOG_INFO,
                       "UAC 2.0 sample rate set to %u Hz\n", rate);
            }
        }
    } else if (ch->uac.num_sample_rates > 1 && ch->uac.active_sample_rate > 0) {
        /* UAC 1.0: SET_CUR Sampling Frequency on endpoint */
        uint8_t rate_data[3];
        rate = ch->uac.active_sample_rate;
        rate_data[0] = (uint8_t)(rate);
        rate_data[1] = (uint8_t)(rate >> 8);
        rate_data[2] = (uint8_t)(rate >> 16);

        setup[0] = 0x22;  /* Class, Endpoint, Host-to-device */
        setup[1] = 0x01;  /* SET_CUR */
        setup[2] = 0x00;
        setup[3] = 0x01;  /* SAMPLING_FREQ_CONTROL */
        setup[4] = ch->uac.as_endpoint;
        setup[5] = 0x00;
        setup[6] = 0x03;  /* wLength = 3 */
        setup[7] = 0x00;

        ret = ctrl_transfer(s, ch, 0, 0, setup, 8, rate_data, 3, NULL, NULL);
        if (ret < 0) {
            av_log(s, AV_LOG_WARNING,
                   "SET_CUR Sampling Frequency failed (non-fatal)\n");
        } else {
            av_log(s, AV_LOG_INFO,
                   "UAC sample rate set to %u Hz\n", rate);
        }
    }

    return 0;
}

/* ---------------------------------------------------------------------------
 * Heartbeat thread
 * --------------------------------------------------------------------------- */

static double hbeat_index_to_period(int index)
{
    return 5.0 * pow(1.25, index);
}

/* Tear down I/O threads and sockets, preserving pkt_queue and AVStreams. */
static void teardown_session(AVFormatContext *s)
{
    IpmxUsbContext *ctx = s->priv_data;
    int i;

    for (i = 0; i < ctx->num_channels; i++) {
        IpmxUsbDataChannel *ch = &ctx->channels[i];
        if (ch->sock >= 0) {
            close(ch->sock);
            ch->sock = -1;
        }
    }

    for (i = 0; i < ctx->num_channels; i++) {
        IpmxUsbDataChannel *ch = &ctx->channels[i];
        if (ch->io_thread_started) {
            pthread_join(ch->io_thread, NULL);
            ch->io_thread_started = 0;
        }
    }

    if (ctx->ctrl_sock >= 0) {
        close(ctx->ctrl_sock);
        ctx->ctrl_sock = -1;
    }
    if (ctx->data_listen_sock >= 0) {
        close(ctx->data_listen_sock);
        ctx->data_listen_sock = -1;
    }
}

/*
 * Re-handshake a data channel after reconnect.  Receives USBStreamInfo,
 * sends USBStreamStatus, switches to payload SSIDs.  Does NOT re-enumerate
 * (cached descriptors remain valid).
 */
static int rehandshake_data_channel(AVFormatContext *s,
                                    IpmxUsbDataChannel *ch, int ch_idx)
{
    IpmxUsbContext *ctx = s->priv_data;
    IpmxUsbPepState *pep = ctx->pep.encrypted ? &ctx->pep : NULL;
    IpmxUsbMsg msg;
    uint8_t build_buf[256];
    int len, ret;
    uint16_t hs_s2r_ssid, hs_r2s_ssid;

    ch->recv_buf_len = 0;
    ch->recv_buf_pos = 0;
    ch->seqnum       = 1;
    ch->ctr_s2r      = 0;
    ch->ctr_r2s      = 0;
    memset(&ch->fp_s2r, 0, sizeof(ch->fp_s2r));
    ch->fp_s2r.strict = ctx->strict_pep;
    ch->video_iso_pending = 0;
    ch->audio_iso_pending = 0;
    ch->video_frame_count = 0;
    ch->audio_sample_count = 0;
    ch->last_data_time = av_gettime_relative();
    ch->handshake_done = 0;

    hs_s2r_ssid = pep ? IPMX_USB_DATA_HANDSHAKE_S2R_SSID : 0;
    hs_r2s_ssid = pep ? IPMX_USB_DATA_HANDSHAKE_R2S_SSID : 0;

    ret = ipmx_usb_recv_message(s, pep, ch->sock,
                                ch->recv_buf, &ch->recv_buf_len,
                                &ch->recv_buf_pos,
                                hs_s2r_ssid, &ch->ctr_s2r,
                                &ch->fp_s2r, &msg, CTRL_TIMEOUT_MS);
    if (ret < 0) {
        av_log(s, AV_LOG_ERROR,
               "Reconnect: failed to receive USB_STREAM_INFO ch %d\n", ch_idx);
        return ret;
    }
    if (msg.hdr.msg_type != IPMX_USB_STREAM_INFO) {
        av_log(s, AV_LOG_ERROR,
               "Reconnect: expected USB_STREAM_INFO ch %d, got 0x%02x\n",
               ch_idx, msg.hdr.msg_type);
        return AVERROR(EPROTO);
    }

    len = ipmx_usb_build_usb_stream_status(build_buf, sizeof(build_buf), 0);
    if (len < 0)
        return AVERROR_EXIT;
    ret = ipmx_usb_send_message(s, pep, ch->sock, hs_r2s_ssid,
                                &ch->ctr_r2s, build_buf, len);
    if (ret < 0)
        return ret;

    ch->substreamid    = msg.payload.usb_stream_info.substreamid;
    ch->handshake_done = 1;
    av_log(s, AV_LOG_INFO,
           "Reconnect: channel %d re-handshaked (ssid=%u)\n",
           ch_idx, ch->substreamid);
    return 0;
}

/*
 * Reconnect to the Sender: control handshake, data channel accept,
 * data handshakes, restart I/O threads.  Skips USB enumeration and
 * AVStream creation (cached from first connection).
 */
static int reconnect_session(AVFormatContext *s)
{
    IpmxUsbContext *ctx = s->priv_data;
    IpmxUsbPepState *pep;
    IpmxUsbMsg msg;
    uint8_t *build_buf;
    int len, ret, ch_idx, data_fd, p;

    ctx->reconnect_attempt++;
    av_log(s, AV_LOG_INFO,
           "Reconnecting to Sender at %s:%d (attempt %d)...\n",
           ctx->sender_host, ctx->sender_port, ctx->reconnect_attempt);

    ctx->ctrl_sock = ipmx_usb_tcp_connect(ctx->sender_host, ctx->sender_port);
    if (ctx->ctrl_sock < 0) {
        av_log(s, AV_LOG_WARNING,
               "Reconnect: failed to connect to %s:%d\n",
               ctx->sender_host, ctx->sender_port);
        return AVERROR(EIO);
    }
    av_log(s, AV_LOG_INFO, "Control channel reconnected\n");

    ctx->ctrl_recv_buf_len = 0;
    ctx->ctrl_recv_buf_pos = 0;
    ctx->ctrl_ctr_s2r = 0;
    ctx->ctrl_ctr_r2s = 0;
    memset(&ctx->ctrl_fp_s2r, 0, sizeof(ctx->ctrl_fp_s2r));
    ctx->ctrl_fp_s2r.strict = ctx->strict_pep;

    if (ctx->pep.encrypted) {
        const char *pep_mode_src = ctx->pep_mode_str ? ctx->pep_mode_str : s->pep_mode;
        const char *pep_iv_src   = ctx->pep_iv_str ? ctx->pep_iv_str : s->pep_iv;
        const char *pep_kg_src   = ctx->pep_key_generator_str ? ctx->pep_key_generator_str : s->pep_key_generator;
        const char *pep_kv_src   = ctx->pep_key_version_str ? ctx->pep_key_version_str : s->pep_key_version;
        ret = ipmx_usb_init_pep(s, &ctx->pep,
                                 ctx->psk_hex, pep_mode_src,
                                 pep_iv_src, pep_kg_src,
                                 pep_kv_src, 0);
        if (ret < 0) goto reconn_fail;
    }
    pep = ctx->pep.encrypted ? &ctx->pep : NULL;

    build_buf = av_malloc(BUILD_BUF_SIZE);
    if (!build_buf)
        return AVERROR(ENOMEM);

    /* Receive SenderConnectionInfo */
    ret = ipmx_usb_recv_message(s, pep, ctx->ctrl_sock,
                                ctx->ctrl_recv_buf,
                                &ctx->ctrl_recv_buf_len,
                                &ctx->ctrl_recv_buf_pos,
                                0, &ctx->ctrl_ctr_s2r,
                                &ctx->ctrl_fp_s2r, &msg, CTRL_TIMEOUT_MS);
    if (ret < 0) {
        av_log(s, AV_LOG_ERROR, "Reconnect: failed to receive SCI\n");
        av_free(build_buf);
        goto reconn_fail;
    }
    if (msg.hdr.msg_type != IPMX_USB_SENDER_CONNECTION_INFO) {
        av_log(s, AV_LOG_ERROR, "Reconnect: expected SCI, got 0x%02x\n",
               msg.hdr.msg_type);
        av_free(build_buf);
        ret = AVERROR(EPROTO);
        goto reconn_fail;
    }

    /* Send SenderConnectionStatus */
    len = ipmx_usb_build_sender_connection_status(
        build_buf, BUILD_BUF_SIZE,
        msg.payload.sender_connection_info.cid,
        ctx->serial ? ctx->serial : "",
        ctx->hbeat_index, ctx->data_port);
    if (len < 0) {
        av_free(build_buf);
        ret = AVERROR_EXIT;
        goto reconn_fail;
    }
    ret = ipmx_usb_send_message(s, pep, ctx->ctrl_sock, 1,
                                &ctx->ctrl_ctr_r2s, build_buf, len);
    av_free(build_buf);
    if (ret < 0) goto reconn_fail;

    /* Listen for data channels */
    ctx->data_listen_sock = ipmx_usb_tcp_listen(ctx->data_port);
    if (ctx->data_listen_sock < 0) {
        for (p = DEFAULT_DATA_PORT + 1; p < DEFAULT_DATA_PORT + 100; p++) {
            ctx->data_listen_sock = ipmx_usb_tcp_listen(p);
            if (ctx->data_listen_sock >= 0) {
                ctx->data_port = p;
                break;
            }
        }
    }
    if (ctx->data_listen_sock < 0) {
        ret = AVERROR(EIO);
        goto reconn_fail;
    }

    /* Accept data channels */
    for (ch_idx = 0; ch_idx < ctx->num_channels; ch_idx++) {
        data_fd = ipmx_usb_tcp_accept(ctx->data_listen_sock,
                                       DATA_ACCEPT_TIMEOUT_MS);
        if (data_fd < 0) {
            av_log(s, AV_LOG_ERROR,
                   "Reconnect: timeout waiting for data channel %d\n", ch_idx);
            ret = AVERROR(ETIMEDOUT);
            goto reconn_fail;
        }
        ctx->channels[ch_idx].sock = data_fd;
    }

    /* Handshake each data channel (no re-enumeration) */
    for (ch_idx = 0; ch_idx < ctx->num_channels; ch_idx++) {
        ret = rehandshake_data_channel(s, &ctx->channels[ch_idx], ch_idx);
        if (ret < 0) goto reconn_fail;
    }

    /* Reset I/O thread tracking and clear queue EOF so read_packet resumes */
    pthread_mutex_lock(&ctx->io_threads_mutex);
    ctx->io_threads_active = 0;
    pthread_mutex_unlock(&ctx->io_threads_mutex);
    if (ctx->pkt_queue) {
        av_thread_message_queue_set_err_recv(ctx->pkt_queue, 0);
        av_thread_message_queue_set_err_send(ctx->pkt_queue, 0);
    }

    /* Restart I/O threads */
    for (ch_idx = 0; ch_idx < ctx->num_channels; ch_idx++) {
        ChannelThreadArg *ta = av_malloc(sizeof(*ta));
        if (!ta) {
            ret = AVERROR(ENOMEM);
            goto reconn_fail;
        }
        ta->s  = s;
        ta->ch = &ctx->channels[ch_idx];
        ret = pthread_create(&ctx->channels[ch_idx].io_thread, NULL,
                             channel_io_thread, ta);
        if (ret != 0) {
            av_free(ta);
            ret = AVERROR(ret);
            goto reconn_fail;
        }
        ctx->channels[ch_idx].io_thread_started = 1;
    }

    ctx->last_hbeat_us = av_gettime_relative();
    ctx->reconnect_attempt = 0;
    av_log(s, AV_LOG_INFO, "Reconnected — session resumed\n");
    return 0;

reconn_fail:
    if (ctx->ctrl_sock >= 0) {
        close(ctx->ctrl_sock);
        ctx->ctrl_sock = -1;
    }
    if (ctx->data_listen_sock >= 0) {
        close(ctx->data_listen_sock);
        ctx->data_listen_sock = -1;
    }
    for (ch_idx = 0; ch_idx < ctx->num_channels; ch_idx++) {
        if (ctx->channels[ch_idx].sock >= 0) {
            close(ctx->channels[ch_idx].sock);
            ctx->channels[ch_idx].sock = -1;
        }
    }
    return ret;
}

/*
 * Control monitor thread: reads Sender control messages (heartbeats,
 * VendorSpecific, EnterSleep), monitors heartbeat timeout per Section 11,
 * and triggers reconnection when the control channel is lost.
 */
static void *ctrl_monitor_thread(void *arg)
{
    AVFormatContext *s = arg;
    IpmxUsbContext *ctx = s->priv_data;
    IpmxUsbMsg msg;
    IpmxUsbPepState *pep;
    int ret;

    while (!ctx->stop) {
        pep = ctx->pep.encrypted ? &ctx->pep : NULL;

        ret = ipmx_usb_recv_message(s, pep, ctx->ctrl_sock,
                                     ctx->ctrl_recv_buf,
                                     &ctx->ctrl_recv_buf_len,
                                     &ctx->ctrl_recv_buf_pos,
                                     0, &ctx->ctrl_ctr_s2r,
                                     &ctx->ctrl_fp_s2r,
                                     &msg, CTRL_MONITOR_POLL_MS);
        if (ret == AVERROR(EAGAIN)) {
            /* Check heartbeat timeout */
            int64_t now = av_gettime_relative();
            double elapsed = (now - ctx->last_hbeat_us) / 1e6;
            if (ctx->hbeat_period_s > 0 &&
                elapsed > ctx->hbeat_period_s * 2.0) {
                av_log(s, AV_LOG_WARNING,
                       "Heartbeat timeout — Sender non-responsive "
                       "(no heartbeat for %.1fs, expected every %.1fs). "
                       "Disconnecting per Section 11.\n",
                       elapsed, ctx->hbeat_period_s);
                goto do_reconnect;
            }
            continue;
        }
        if (ret == AVERROR_EOF) {
            av_log(s, AV_LOG_INFO,
                   "Sender closed control channel. Reconnecting...\n");
            goto do_reconnect;
        }
        if (ret == IPMX_USB_ERR_MAC_FAIL) {
            if (ctx->ctrl_fp_s2r.strict) {
                av_log(s, AV_LOG_ERROR,
                       "Control channel MAC failed (strict) — closing\n");
                goto do_reconnect;
            }
            av_log(s, AV_LOG_WARNING,
                   "Control channel MAC failed on msg_type=0x%02x "
                   "— discarding\n", msg.hdr.msg_type);
            continue;
        }
        if (ret < 0) {
            av_log(s, AV_LOG_ERROR,
                   "Control channel recv error %d. Reconnecting...\n", ret);
            goto do_reconnect;
        }

        switch (msg.hdr.msg_type) {
        case IPMX_USB_HEARTBEAT:
            ctx->last_hbeat_us = av_gettime_relative();
            av_log(s, AV_LOG_DEBUG, "Heartbeat received\n");
            break;
        case IPMX_USB_ENTER_SLEEP:
            av_log(s, AV_LOG_INFO,
                   "Sender entering sleep — closing session "
                   "(no reconnect per Section 9.12)\n");
            ctx->stop = 1;
            goto out;
        case IPMX_USB_VENDOR_SPECIFIC_INFO:
            av_log(s, AV_LOG_INFO, "Received VendorSpecificInfo\n");
            break;
        case IPMX_USB_VENDOR_SPECIFIC_QUERY:
            av_log(s, AV_LOG_DEBUG, "Received VendorSpecificQuery\n");
            break;
        default:
            av_log(s, AV_LOG_DEBUG,
                   "Control msg_type=0x%02x (ignored)\n", msg.hdr.msg_type);
            break;
        }
        continue;

do_reconnect:
        if (ctx->stop) break;
        teardown_session(s);

        av_usleep(RECONNECT_DELAY_US);
        if (ctx->stop) break;

        ret = reconnect_session(s);
        if (ret < 0) {
            av_log(s, AV_LOG_WARNING,
                   "Reconnect failed (%d), retrying in %ds...\n",
                   ret, (int)(RECONNECT_DELAY_US / 1000000));
            av_usleep(RECONNECT_DELAY_US);
        }
    }

out:
    return NULL;
}

/* ---------------------------------------------------------------------------
 * Parse sender option "host:port"
 * --------------------------------------------------------------------------- */

static int parse_sender(AVFormatContext *s, char *sender,
                        char *host_out, int host_size, int *port_out)
{
    const char *start;
    const char *colon;
    const char *scheme_end;
    int hlen;

    if (!sender || !sender[0]) {
        av_log(s, AV_LOG_ERROR,
               "No sender specified; use -sender host:port\n");
        return AVERROR(EINVAL);
    }

    /* Strip scheme prefix (e.g. "tcp://") */
    scheme_end = strstr(sender, "://");
    start = scheme_end ? scheme_end + 3 : sender;

    colon = strrchr(start, ':');
    if (!colon || colon == start) {
        av_log(s, AV_LOG_ERROR,
               "Invalid sender format '%s'; expected host:port\n", sender);
        return AVERROR(EINVAL);
    }

    hlen = (int)(colon - start);
    if (hlen >= host_size)
        hlen = host_size - 1;
    memcpy(host_out, start, hlen);
    host_out[hlen] = '\0';

    *port_out = atoi(colon + 1);
    if (*port_out <= 0 || *port_out > 65535) {
        av_log(s, AV_LOG_ERROR, "Invalid port number in '%s'\n", sender);
        return AVERROR(EINVAL);
    }

    return 0;
}

/* ---------------------------------------------------------------------------
 * Parse preferred video size
 * --------------------------------------------------------------------------- */

static void parse_video_size(const char *str, int *width, int *height)
{
    *width  = 1920;
    *height = 1080;

    if (!str || !str[0])
        return;

    if (sscanf(str, "%dx%d", width, height) != 2) {
        *width  = 1920;
        *height = 1080;
    }
}

/* ---------------------------------------------------------------------------
 * Parse preferred framerate
 * --------------------------------------------------------------------------- */

static int parse_framerate(const char *str)
{
    int fps = 30;
    if (str && str[0])
        fps = atoi(str);
    return (fps > 0) ? fps : 30;
}

/* ---------------------------------------------------------------------------
 * ISO submit helpers
 * --------------------------------------------------------------------------- */

static int send_iso_submit(AVFormatContext *s, IpmxUsbDataChannel *ch,
                           uint8_t endpoint,
                           int num_packets, uint16_t max_pkt_size)
{
    IpmxUsbContext *ctx = s->priv_data;
    IpmxUsbPepState *pep = ctx->pep.encrypted ? &ctx->pep : NULL;
    uint8_t *build_buf;
    uint16_t iso_lengths[IPMX_USB_MAX_ISO_PACKETS];
    uint32_t seq = ch->seqnum++;
    int len, i, ret;

    build_buf = av_malloc(BUILD_BUF_SIZE);
    if (!build_buf)
        return AVERROR(ENOMEM);

    for (i = 0; i < num_packets; i++)
        iso_lengths[i] = max_pkt_size;

    len = ipmx_usb_build_iso_submit(build_buf, BUILD_BUF_SIZE,
                                    seq, endpoint, 1, 0,
                                    1, 0, num_packets, iso_lengths);
    if (len < 0) {
        av_log(s, AV_LOG_ERROR, "Failed to build ISO_SUBMIT for ep=0x%02x\n",
               endpoint);
        av_free(build_buf);
        return AVERROR_EXIT;
    }

    ret = ipmx_usb_send_message(s, pep, ch->sock, ch->substreamid | 1,
                                &ch->ctr_r2s, build_buf, len);
    av_free(build_buf);
    return ret;
}

/* ---------------------------------------------------------------------------
 * Process an ISO_SUBMIT_RETURN for video
 * --------------------------------------------------------------------------- */

static int process_video_iso_return(AVFormatContext *s,
                                    IpmxUsbDataChannel *ch,
                                    const IpmxUsbIsoSubmitReturn *iso,
                                    AVPacket *pkt)
{
    IpmxUsbContext *ctx = s->priv_data;
    const uint8_t *data_ptr = iso->transferdata;
    int offset = 0;
    int got_frame = 0;

    /* Latch the STARTFRAME from the first URB that contributes to a new
     * video frame.  When a frame spans multiple URBs, USBIP assigns each
     * URB its own STARTFRAME (the USB frame number at transmission time).
     * Per TR-10-14, the frame's PTS should reflect the capture time, which
     * corresponds to the first URB — not the one that happens to complete
     * the UVC reassembly. */
    if (iso->startframe && !ch->video_frame_first_startframe_set &&
        !ch->uvc.frame_complete) {
        ch->video_frame_first_startframe     = iso->startframe;
        ch->video_frame_first_startframe_set = 1;
    }

    for (uint32_t i = 0; i < iso->num_packets; i++) {
        int pkt_len = iso->iso_packets[i].actuallength;
        if (pkt_len <= 0 || iso->iso_packets[i].isostatus != 0) {
            offset += pkt_len;
            continue;
        }

        if (data_ptr && offset + pkt_len <= iso->transferdata_len) {
            int ret = ipmx_uvc_process_payload(&ch->uvc,
                                               data_ptr + offset, pkt_len);
            if (ret < 0)
                av_log(s, AV_LOG_DEBUG,
                       "UVC payload processing error at pkt %u\n", i);
        }
        offset += pkt_len;

        if (!got_frame && ipmx_uvc_frame_ready(&ch->uvc)) {
            uint8_t *frame_data;
            int frame_size;
            if (ipmx_uvc_get_frame(&ch->uvc, &frame_data, &frame_size) == 0 &&
                frame_size > 0) {
                uint32_t uvc_pts;
                uint32_t frame_sf = ch->video_frame_first_startframe_set
                                  ? ch->video_frame_first_startframe
                                  : iso->startframe;
                int ret = av_new_packet(pkt, frame_size);
                if (ret < 0)
                    return ret;
                memcpy(pkt->data, frame_data, frame_size);
                pkt->stream_index = ch->video_stream_idx;

                ch->last_startframe_abs_us =
                    (int64_t)frame_sf * 1000;
                ch->last_wire_startframe = frame_sf;
                ch->last_wire_has_uvc_pts =
                    ipmx_uvc_get_frame_pts(&ch->uvc, &uvc_pts);
                ch->last_wire_uvc_pts = uvc_pts;

                {
                    uint32_t stc;
                    uint16_t sof;
                    ch->last_has_scr = ipmx_uvc_get_frame_scr(
                        &ch->uvc, &stc, &sof);
                    if (ch->last_has_scr) {
                        ch->last_scr_stc = stc;
                        ch->last_scr_sof = sof;
                    }
                }

                if (frame_sf) {
                    int64_t current_abs_us = (int64_t)frame_sf * 1000;

                    if (ctx->clockrec.clock_recovery &&
                        ctx->clockrec.mid_calibrated) {
                        /* Affine model: PTS = wall-clock estimate + latency.
                         * This is an absolute delivery time (no epoch). */
                        pkt->pts = ipmx_clockrec_reconstruct_ts(
                                       &ctx->clockrec, current_abs_us, 0)
                                   + ctx->video_latency_target_us;
                    } else {
                        /* Pre-calibration or no clock recovery: relative PTS */
                        if (!ch->video_sf_epoch_set) {
                            ch->video_sf_epoch_us  = current_abs_us;
                            ch->video_sf_epoch_set = 1;
                        }
                        pkt->pts = current_abs_us - ch->video_sf_epoch_us;
                    }
                } else
                    pkt->pts = AV_NOPTS_VALUE;
                pkt->dts = pkt->pts;

                /* Reset latch for the next video frame */
                ch->video_frame_first_startframe_set = 0;

                ch->video_frame_count++;
                pkt->flags |= AV_PKT_FLAG_KEY;
                got_frame = 1;
            }
        }
    }

    return got_frame ? 0 : AVERROR(EAGAIN);
}

/* ---------------------------------------------------------------------------
 * Process an ISO_SUBMIT_RETURN for audio
 * --------------------------------------------------------------------------- */

static int process_audio_iso_return(AVFormatContext *s,
                                    IpmxUsbDataChannel *ch,
                                    const IpmxUsbIsoSubmitReturn *iso,
                                    AVPacket *pkt)
{
    IpmxUsbContext *ctx = s->priv_data;
    uint8_t pcm_buf[PCM_BUF_SIZE];
    int pcm_total = 0;
    const uint8_t *data_ptr = iso->transferdata;
    int offset = 0;

    for (uint32_t i = 0; i < iso->num_packets; i++) {
        int pkt_len = iso->iso_packets[i].actuallength;
        if (pkt_len <= 0 || iso->iso_packets[i].isostatus != 0) {
            offset += pkt_len;
            continue;
        }

        if (data_ptr && offset + pkt_len <= iso->transferdata_len) {
            int n = ipmx_uac_extract_audio(&ch->uac, data_ptr + offset,
                                           pkt_len,
                                           pcm_buf + pcm_total,
                                           PCM_BUF_SIZE - pcm_total);
            if (n > 0)
                pcm_total += n;
        }
        offset += pkt_len;
    }

    if (pcm_total > 0) {
        int ret = av_new_packet(pkt, pcm_total);
        if (ret < 0)
            return ret;
        memcpy(pkt->data, pcm_buf, pcm_total);
        pkt->stream_index = ch->audio_stream_idx;

        if (iso->startframe) {
            int64_t current_abs_us = (int64_t)iso->startframe * 1000;

            if (ctx->clockrec.clock_recovery &&
                ctx->clockrec.mid_calibrated) {
                /* Affine model: PTS = wall-clock estimate + latency. */
                pkt->pts = ipmx_clockrec_reconstruct_ts(
                               &ctx->clockrec, current_abs_us, 1)
                           + ctx->audio_latency_target_us;
            } else {
                if (!ch->audio_sf_epoch_set) {
                    ch->audio_sf_epoch_us  = current_abs_us;
                    ch->audio_sf_epoch_set = 1;
                }
                pkt->pts = current_abs_us - ch->audio_sf_epoch_us;
            }
        } else
            pkt->pts = AV_NOPTS_VALUE;
        pkt->dts = pkt->pts;
        ch->last_startframe_abs_us =
            (int64_t)iso->startframe * 1000;

        {
            int bytes_per_sample = ch->uac.channels
                                   * (ch->uac.bit_resolution / 8);
            int num_samples = bytes_per_sample > 0
                              ? pcm_total / bytes_per_sample : 0;
            ch->audio_sample_count += num_samples;
        }
        return 0;
    }

    return AVERROR(EAGAIN);
}

/* ---------------------------------------------------------------------------
 * Determine which stream type to request based on context options
 * --------------------------------------------------------------------------- */

static void resolve_stream_type(IpmxUsbContext *ctx, IpmxUsbDataChannel *ch)
{
    int want_video = 1, want_audio = 1;

    if (ctx->stream_type) {
        if (!strcmp(ctx->stream_type, "video"))
            want_audio = 0;
        else if (!strcmp(ctx->stream_type, "audio"))
            want_video = 0;
    }

    ch->has_video = want_video && (ch->uvc.num_formats > 0);
    ch->has_audio = want_audio && (ch->uac.channels > 0);

    if (!ch->has_video && !ch->has_audio) {
        ch->has_video = (ch->uvc.num_formats > 0);
        ch->has_audio = (ch->uac.channels > 0);
    }
}

/* ---------------------------------------------------------------------------
 * Packet queue free callback (for AVThreadMessageQueue)
 * --------------------------------------------------------------------------- */

static void pkt_queue_free(void *msg)
{
    AVPacket **ppkt = msg;
    av_packet_free(ppkt);
}

static int64_t ipmx_usb_audio_packet_samples(AVFormatContext *s,
                                             const AVPacket *pkt)
{
    AVStream *st;
    AVCodecParameters *par;

    if (!pkt || pkt->stream_index < 0 || pkt->stream_index >= (int)s->nb_streams)
        return 0;

    st = s->streams[pkt->stream_index];
    par = st ? st->codecpar : NULL;
    if (!par || par->codec_type != AVMEDIA_TYPE_AUDIO)
        return 0;

    if (par->block_align > 0 && pkt->size > 0)
        return pkt->size / par->block_align;

    if (pkt->duration > 0 && par->sample_rate > 0)
        return av_rescale_q(pkt->duration, st->time_base,
                            (AVRational){ par->sample_rate, 1 });

    return 0;
}

static void ipmx_usb_audio_rate_note_receive(IpmxUsbContext *ctx,
                                             int64_t samples)
{
    if (samples <= 0)
        return;

    pthread_mutex_lock(&ctx->audio_rate_mutex);
    ctx->audio_rx_samples += samples;
    pthread_mutex_unlock(&ctx->audio_rate_mutex);
}

static void ipmx_usb_audio_rate_note_handoff(IpmxUsbContext *ctx,
                                             int64_t samples)
{
    if (samples <= 0)
        return;

    pthread_mutex_lock(&ctx->audio_rate_mutex);
    ctx->audio_tx_samples += samples;
    pthread_mutex_unlock(&ctx->audio_rate_mutex);
}

static void ipmx_usb_audio_rate_snapshot(IpmxUsbContext *ctx,
                                         int64_t *rx_samples,
                                         int64_t *tx_samples)
{
    pthread_mutex_lock(&ctx->audio_rate_mutex);
    *rx_samples = ctx->audio_rx_samples;
    *tx_samples = ctx->audio_tx_samples;
    pthread_mutex_unlock(&ctx->audio_rate_mutex);
}

static void ipmx_usb_audio_rate_reset(IpmxUsbContext *ctx)
{
    pthread_mutex_lock(&ctx->audio_rate_mutex);
    ctx->audio_rx_samples = 0;
    ctx->audio_tx_samples = 0;
    pthread_mutex_unlock(&ctx->audio_rate_mutex);
}

/* ---------------------------------------------------------------------------
 * Per-channel I/O thread: runs poll_channel in a loop, enqueues packets
 * --------------------------------------------------------------------------- */

static void *channel_io_thread(void *arg)
{
    ChannelThreadArg *ta = arg;
    AVFormatContext *s = ta->s;
    IpmxUsbContext *ctx = s->priv_data;
    IpmxUsbDataChannel *ch = ta->ch;
    int remaining;

    av_free(ta);
    ch->io_thread_running = 1;

    pthread_mutex_lock(&ctx->io_threads_mutex);
    ctx->io_threads_active++;
    pthread_mutex_unlock(&ctx->io_threads_mutex);

    av_log(s, AV_LOG_INFO,
           "Channel I/O thread started (ssid=%u video=%d audio=%d)\n",
           ch->substreamid, ch->has_video, ch->has_audio);

    while (!ctx->stop) {
        AVPacket *pkt;
        int ret;
        int64_t now = av_gettime_relative();
        int64_t idle = now - ch->last_data_time;

        if (ch->last_data_time > 0 && idle > DATA_IDLE_TIMEOUT_US) {
            av_log(s, AV_LOG_INFO,
                   "Channel ssid=%u idle for >%ds, thread exiting\n",
                   ch->substreamid,
                   (int)(DATA_IDLE_TIMEOUT_US / 1000000));
            break;
        }

        pkt = av_packet_alloc();
        if (!pkt)
            break;

        ret = poll_channel(s, ch, pkt);
        if (ret == 0) {
            if (ctx->clockrec.clock_recovery || ctx->clockrec.latency_report) {
                ipmx_timing_force_stamp_pkt(pkt);
                if (pkt->opaque_ref) {
                    IpmxTimingOpaque *t =
                        (IpmxTimingOpaque *)pkt->opaque_ref->data;
                    int is_audio = (pkt->stream_index == ch->audio_stream_idx);

                    t->sender_raw_us = ch->last_startframe_abs_us;

                    if (ctx->clockrec.clock_recovery && ctx->clockrec.calibrated) {
                        t->sender_us = ipmx_clockrec_reconstruct_ts(&ctx->clockrec,
                                                                    ch->last_startframe_abs_us, is_audio);
                    } else {
                        t->sender_us = ch->last_startframe_abs_us;
                    }

                    if (ch->last_has_scr) {
                        t->scr_stc = ch->last_scr_stc;
                        t->scr_sof = ch->last_scr_sof;
                        t->has_scr = 1;
                        ch->last_has_scr = 0;
                    }
                }
            } else
                ipmx_timing_stamp_pkt(pkt);
            if (ctx->timing_fp) {
                int64_t now = av_gettime_relative();
                int is_v = (pkt->stream_index != ch->audio_stream_idx);
                if (is_v) {
                    fprintf(ctx->timing_fp,
                            "%lld\tNET_RCV\tV\t%lld\t%lld\t"
                            "pts=%lld\tSF=%u\tcal=%d\n",
                            (long long)now,
                            (long long)ctx->timing_rcv_vid++,
                            (long long)now,
                            (long long)pkt->pts,
                            (unsigned)ch->last_wire_startframe,
                            ctx->clockrec.mid_calibrated);
                } else {
                    fprintf(ctx->timing_fp,
                            "%lld\tNET_RCV\tA\t%lld\t%lld\t"
                            "pts=%lld\tSF=%lld\tcal=%d\n",
                            (long long)now,
                            (long long)ctx->timing_rcv_aud++,
                            (long long)now,
                            (long long)pkt->pts,
                            (long long)(ch->last_startframe_abs_us / 1000),
                            ctx->clockrec.mid_calibrated);
                }
            }
            {
            AVPacket *qpkt = pkt;
            int64_t audio_samples = 0;
            ret = av_thread_message_queue_send(ctx->pkt_queue, &qpkt, 0);
            if (ret < 0) {
                av_packet_free(&pkt);
                break;
            }
            if (ctx->clockrec.clock_recovery && ctx->clockrec.is_pcm)
                audio_samples = ipmx_usb_audio_packet_samples(s, pkt);
            ipmx_usb_audio_rate_note_receive(ctx, audio_samples);
            }
        } else {
            av_packet_free(&pkt);
            if (ret != AVERROR(EAGAIN))
                break;
        }
    }

    ch->io_thread_running = 0;

    pthread_mutex_lock(&ctx->io_threads_mutex);
    remaining = --ctx->io_threads_active;
    pthread_mutex_unlock(&ctx->io_threads_mutex);

    if (remaining <= 0 && ctx->pkt_queue)
        av_thread_message_queue_set_err_recv(ctx->pkt_queue, AVERROR_EOF);

    av_log(s, AV_LOG_INFO, "Channel I/O thread exiting (ssid=%u, %d remaining)\n",
           ch->substreamid, remaining);
    return NULL;
}

/* ---------------------------------------------------------------------------
 * read_header — connect, handshake, enumerate, configure, create streams
 * --------------------------------------------------------------------------- */

/* ---------------------------------------------------------------------------
 * Per-channel setup: handshake, enumerate, negotiate, create AVStreams
 * --------------------------------------------------------------------------- */

static int setup_data_channel(AVFormatContext *s, IpmxUsbDataChannel *ch,
                              int ch_idx)
{
    IpmxUsbContext *ctx = s->priv_data;
    IpmxUsbPepState *pep = ctx->pep.encrypted ? &ctx->pep : NULL;
    IpmxUsbMsg msg;
    uint8_t *build_buf;
    int len, ret;
    uint16_t hs_s2r_ssid, hs_r2s_ssid;

    build_buf = av_malloc(BUILD_BUF_SIZE);
    if (!build_buf)
        return AVERROR(ENOMEM);

    /* Allocate per-channel recv buffer */
    ch->recv_buf = av_mallocz(RECV_BUF_SIZE);
    if (!ch->recv_buf) {
        av_free(build_buf);
        return AVERROR(ENOMEM);
    }
    ch->recv_buf_len = 0;
    ch->recv_buf_pos = 0;
    ch->seqnum = 1;
    ch->ctr_s2r = 0;
    ch->ctr_r2s = 0;
    memset(&ch->fp_s2r, 0, sizeof(ch->fp_s2r));
    ch->fp_s2r.strict = ctx->strict_pep;
    ch->video_stream_idx = -1;
    ch->audio_stream_idx = -1;

    hs_s2r_ssid = ctx->pep.encrypted ? IPMX_USB_DATA_HANDSHAKE_S2R_SSID : 0;
    hs_r2s_ssid = ctx->pep.encrypted ? IPMX_USB_DATA_HANDSHAKE_R2S_SSID : 0;

    /* Receive USB_STREAM_INFO */
    ret = ipmx_usb_recv_message(s, pep, ch->sock,
                                ch->recv_buf, &ch->recv_buf_len,
                                &ch->recv_buf_pos,
                                hs_s2r_ssid, &ch->ctr_s2r,
                                &ch->fp_s2r,
                                &msg, CTRL_TIMEOUT_MS);
    if (ret == IPMX_USB_ERR_MAC_FAIL) {
        av_log(s, AV_LOG_ERROR,
               "USB_STREAM_INFO MAC failed on channel %d — "
               "cannot complete handshake\n", ch_idx);
        av_free(build_buf);
        return AVERROR(EPROTO);
    }
    if (ret < 0) {
        av_log(s, AV_LOG_ERROR,
               "Failed to receive USB_STREAM_INFO for channel %d\n", ch_idx);
        av_free(build_buf);
        return ret;
    }
    if (msg.hdr.msg_type != IPMX_USB_STREAM_INFO) {
        av_log(s, AV_LOG_ERROR,
               "Expected USB_STREAM_INFO on channel %d, got 0x%02x\n",
               ch_idx, msg.hdr.msg_type);
        av_free(build_buf);
        return AVERROR(EPROTO);
    }
    av_log(s, AV_LOG_INFO,
           "Channel %d: USB_STREAM_INFO (substreamid=%u speed=%u)\n",
           ch_idx, msg.payload.usb_stream_info.substreamid,
           msg.payload.usb_stream_info.usbspeed);

    /* Send USB_STREAM_STATUS */
    len = ipmx_usb_build_usb_stream_status(build_buf, BUILD_BUF_SIZE, 0);
    if (len < 0) {
        av_free(build_buf);
        return AVERROR_EXIT;
    }
    ret = ipmx_usb_send_message(s, pep, ch->sock, hs_r2s_ssid,
                                &ch->ctr_r2s, build_buf, len);
    if (ret < 0) {
        av_free(build_buf);
        return ret;
    }

    /* Switch to payload SSIDs */
    ch->substreamid = msg.payload.usb_stream_info.substreamid;
    ch->handshake_done = 1;
    ch->last_data_time = av_gettime_relative();
    av_log(s, AV_LOG_INFO, "Channel %d: payload SSID %u\n",
           ch_idx, ch->substreamid);

    /* USB enumeration */
    ret = usb_enumerate(s, ch);
    if (ret < 0) {
        av_log(s, AV_LOG_ERROR,
               "USB enumeration failed on channel %d\n", ch_idx);
        av_free(build_buf);
        return ret;
    }

    resolve_stream_type(ctx, ch);

    if (!ch->has_video && !ch->has_audio) {
        av_log(s, AV_LOG_ERROR,
               "No video or audio on channel %d\n", ch_idx);
        av_free(build_buf);
        return AVERROR(ENODEV);
    }

    /* UVC negotiation */
    if (ch->has_video) {
        int pref_w, pref_h, pref_fps;
        parse_video_size(ctx->video_size_str, &pref_w, &pref_h);
        pref_fps = parse_framerate(ctx->framerate_str);

        ret = ipmx_uvc_select_format(&ch->uvc, pref_w, pref_h, pref_fps);
        if (ret < 0) {
            av_log(s, AV_LOG_WARNING,
                   "Ch %d: could not match %dx%d@%d, using first\n",
                   ch_idx, pref_w, pref_h, pref_fps);
            ch->uvc.active_format = ch->uvc.formats[0].format_index;
            ch->uvc.active_frame  = 1;
            ch->uvc.active_interval =
                ch->uvc.formats[0].frames[0].default_interval;
        }
        if (ch->uvc.quirks & IPMX_UVC_QUIRK_RESTRICT_FRAME_RATE) {
            for (int f = 0; f < ch->uvc.num_formats; f++) {
                if (ch->uvc.formats[f].format_index == ch->uvc.active_format &&
                    ch->uvc.formats[f].num_frames > 0) {
                    ch->uvc.active_interval =
                        ch->uvc.formats[f].frames[0].default_interval;
                    av_log(s, AV_LOG_DEBUG,
                           "RESTRICT_FRAME_RATE quirk: interval=%u\n",
                           (unsigned)ch->uvc.active_interval);
                    break;
                }
            }
        }

        ret = uvc_negotiate(s, ch);
        if (ret < 0) {
            av_log(s, AV_LOG_ERROR,
                   "UVC negotiation failed on channel %d\n", ch_idx);
            av_free(build_buf);
            return ret;
        }

        ret = ipmx_uvc_init_reassembler(&ch->uvc);
        if (ret < 0) {
            av_log(s, AV_LOG_ERROR,
                   "Failed to init UVC reassembler on channel %d\n", ch_idx);
            av_free(build_buf);
            return ret;
        }
    }

    /* UAC activation */
    if (ch->has_audio) {
        ipmx_uac_select_rate(&ch->uac, ctx->sample_rate);
        if (ch->uac.quirks & IPMX_UAC_QUIRK_RATE_FIX && ch->uac.active_sample_rate == 0)
            ch->uac.active_sample_rate = 48000;

        ret = uac_activate(s, ch);
        if (ret < 0) {
            av_log(s, AV_LOG_ERROR,
                   "UAC activation failed on channel %d\n", ch_idx);
            av_free(build_buf);
            return ret;
        }
    }

    /* Create video AVStream */
    if (ch->has_video) {
        AVStream *vst;
        int sel_fmt = -1, sel_frm = -1, f, r;
        const IpmxUvcFormatInfo *active_fmt_info = NULL;
        const char *fmt_name;

        vst = avformat_new_stream(s, NULL);
        if (!vst) {
            av_free(build_buf);
            return AVERROR(ENOMEM);
        }
        ch->video_stream_idx = vst->index;
        vst->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;

        for (f = 0; f < ch->uvc.num_formats; f++) {
            if (ch->uvc.formats[f].format_index == ch->uvc.active_format) {
                sel_fmt = f;
                break;
            }
        }
        if (sel_fmt >= 0) {
            active_fmt_info = &ch->uvc.formats[sel_fmt];
            for (r = 0; r < ch->uvc.formats[sel_fmt].num_frames; r++) {
                if (r + 1 == ch->uvc.active_frame) {
                    sel_frm = r;
                    break;
                }
            }
        }

        if (active_fmt_info && ipmx_uvc_is_h264(active_fmt_info)) {
            vst->codecpar->codec_id = AV_CODEC_ID_H264;
        } else if (active_fmt_info && ipmx_uvc_is_hevc(active_fmt_info)) {
            vst->codecpar->codec_id = AV_CODEC_ID_HEVC;
        } else if (active_fmt_info &&
                   active_fmt_info->format_type == UVC_VS_FORMAT_UNCOMPRESSED) {
            vst->codecpar->codec_id = AV_CODEC_ID_RAWVIDEO;
            if (memcmp(active_fmt_info->guid, UVC_GUID_FORMAT_YUY2, 16) == 0)
                vst->codecpar->format = AV_PIX_FMT_YUYV422;
            else if (memcmp(active_fmt_info->guid, UVC_GUID_FORMAT_NV12, 16) == 0)
                vst->codecpar->format = AV_PIX_FMT_NV12;
            else
                vst->codecpar->format = AV_PIX_FMT_YUYV422;
        } else {
            vst->codecpar->codec_id = AV_CODEC_ID_MJPEG;
        }

        if (sel_fmt >= 0 && sel_frm >= 0) {
            vst->codecpar->width =
                ch->uvc.formats[sel_fmt].frames[sel_frm].width;
            vst->codecpar->height =
                ch->uvc.formats[sel_fmt].frames[sel_frm].height;
        } else {
            vst->codecpar->width  = 1920;
            vst->codecpar->height = 1080;
        }

        avpriv_set_pts_info(vst, 64, 1, 1000000);

        if (sel_fmt >= 0 && sel_frm >= 0 &&
            ch->uvc.formats[sel_fmt].frames[sel_frm].default_interval > 0) {
            uint32_t interval = ch->uvc.formats[sel_fmt]
                                    .frames[sel_frm].default_interval;
            vst->avg_frame_rate = (AVRational){10000000, (int)interval};
            av_reduce(&vst->avg_frame_rate.num, &vst->avg_frame_rate.den,
                      vst->avg_frame_rate.num, vst->avg_frame_rate.den,
                      INT_MAX);
        }

        fmt_name = active_fmt_info ? ipmx_uvc_format_name(active_fmt_info)
                                   : "MJPEG";
        av_log(s, AV_LOG_INFO, "Channel %d video stream[%d]: %dx%d %s @ %d/%d fps\n",
               ch_idx, vst->index,
               vst->codecpar->width, vst->codecpar->height, fmt_name,
               vst->avg_frame_rate.num, vst->avg_frame_rate.den);
    }

    /* Create audio AVStream */
    if (ch->has_audio) {
        AVStream *ast;
        int bps;
        const char *pcm_name;

        ast = avformat_new_stream(s, NULL);
        if (!ast) {
            av_free(build_buf);
            return AVERROR(ENOMEM);
        }
        ch->audio_stream_idx = ast->index;
        ast->codecpar->codec_type  = AVMEDIA_TYPE_AUDIO;

        bps = ch->uac.subframe_size ? ch->uac.subframe_size : 2;
        if (ch->uac.format_tag == UAC_FORMAT_IEEE_FLOAT && bps == 4) {
            ast->codecpar->codec_id = AV_CODEC_ID_PCM_F32LE;
            pcm_name = "F32LE";
        } else if (bps == 4) {
            ast->codecpar->codec_id = AV_CODEC_ID_PCM_S32LE;
            pcm_name = "S32LE";
        } else if (bps == 3) {
            ast->codecpar->codec_id = AV_CODEC_ID_PCM_S24LE;
            pcm_name = "S24LE";
        } else {
            ast->codecpar->codec_id = AV_CODEC_ID_PCM_S16LE;
            pcm_name = "S16LE";
            bps = 2;
        }

        ast->codecpar->sample_rate = ch->uac.active_sample_rate
                                     ? ch->uac.active_sample_rate : 48000;
        ast->codecpar->ch_layout.nb_channels = ch->uac.channels
                                               ? ch->uac.channels : 2;
        ast->codecpar->ch_layout.order = AV_CHANNEL_ORDER_UNSPEC;
        ast->codecpar->bits_per_coded_sample = ch->uac.bit_resolution
                                               ? ch->uac.bit_resolution : 16;
        ast->codecpar->block_align =
            ast->codecpar->ch_layout.nb_channels * bps;
        ast->codecpar->bit_rate =
            ast->codecpar->sample_rate *
            ast->codecpar->block_align * 8;

        avpriv_set_pts_info(ast, 64, 1, 1000000);

        av_log(s, AV_LOG_INFO,
               "Channel %d audio stream[%d]: %d Hz, %d ch, %s\n",
               ch_idx, ast->index, ast->codecpar->sample_rate,
               ast->codecpar->ch_layout.nb_channels, pcm_name);
    }

    av_free(build_buf);
    return 0;
}

/* ---------------------------------------------------------------------------
 * read_header — connect, handshake, enumerate, configure, create streams
 * --------------------------------------------------------------------------- */

static int ipmx_usb_read_header(AVFormatContext *s)
{
    IpmxUsbContext *ctx = s->priv_data;
    IpmxUsbPepState *pep = NULL;
    char host[256];
    int port, ret;
    IpmxUsbMsg msg;
    uint8_t *build_buf;
    int len, ch_idx;

    build_buf = av_malloc(BUILD_BUF_SIZE);
    if (!build_buf)
        return AVERROR(ENOMEM);

    ctx->ctrl_sock        = -1;
    ctx->data_listen_sock = -1;
    ctx->data_port        = DEFAULT_DATA_PORT;
    ctx->num_channels     = 0;
    ctx->io_threads_active = 0;
    pthread_mutex_init(&ctx->io_threads_mutex, NULL);
    pthread_mutex_init(&ctx->audio_rate_mutex, NULL);

    for (ch_idx = 0; ch_idx < MAX_DATA_CHANNELS; ch_idx++) {
        ctx->channels[ch_idx].sock = -1;
        ctx->channels[ch_idx].recv_buf = NULL;
        ctx->channels[ch_idx].video_sf_epoch_set = 0;
        ctx->channels[ch_idx].audio_sf_epoch_set = 0;
    }

    /* Parse SDP early so its c=/m= lines are available for sender resolution
     * and its a=privacy: attributes are ready for PEP initialization. */
    if (ctx->sdp_path) {
        ret = ipmx_usb_parse_sdp(s, ctx->sdp_path,
                                  ctx->sdp_host, sizeof(ctx->sdp_host),
                                  &ctx->sdp_port, &ctx->pep);
        if (ret < 0) {
            av_free(build_buf);
            return ret;
        }
    }

    /* Sender resolution: -sender > SDP c=/m= > s->url */
    if (ctx->sender) {
        ret = parse_sender(s, ctx->sender, host, sizeof(host), &port);
    } else if (ctx->sdp_host[0] && ctx->sdp_port > 0) {
        av_strlcpy(host, ctx->sdp_host, sizeof(host));
        port = ctx->sdp_port;
        ret = 0;
    } else {
        ret = parse_sender(s, s->url, host, sizeof(host), &port);
    }
    if (ret < 0) {
        av_free(build_buf);
        return ret;
    }

    av_strlcpy(ctx->sender_host, host, sizeof(ctx->sender_host));
    ctx->sender_port = port;

    av_log(s, AV_LOG_INFO, "Connecting to IPMX USB Sender at %s:%d\n",
           host, port);

    ctx->ctrl_sock = ipmx_usb_tcp_connect(host, port);
    if (ctx->ctrl_sock < 0) {
        av_log(s, AV_LOG_ERROR,
               "Failed to connect to Sender at %s:%d\n", host, port);
        av_free(build_buf);
        return AVERROR(EIO);
    }
    av_log(s, AV_LOG_INFO, "Control channel connected\n");

    {
        const char *pep_mode_src = ctx->pep_mode_str ? ctx->pep_mode_str : s->pep_mode;
        const char *pep_iv_src   = ctx->pep_iv_str ? ctx->pep_iv_str : s->pep_iv;
        const char *pep_kg_src   = ctx->pep_key_generator_str ? ctx->pep_key_generator_str : s->pep_key_generator;
        const char *pep_kv_src   = ctx->pep_key_version_str ? ctx->pep_key_version_str : s->pep_key_version;

    if (ctx->psk_hex || ctx->sdp_path ||
        pep_mode_src || pep_iv_src || pep_kg_src || pep_kv_src) {
        ret = ipmx_usb_init_pep(s, &ctx->pep,
                                 ctx->psk_hex, pep_mode_src,
                                 pep_iv_src, pep_kg_src,
                                 pep_kv_src, 0);
        if (ret < 0)
            goto fail;
        ctx->ctrl_ctr_s2r = 0;
        ctx->ctrl_ctr_r2s = 0;
        memset(&ctx->ctrl_fp_s2r, 0, sizeof(ctx->ctrl_fp_s2r));
        ctx->ctrl_fp_s2r.strict = ctx->strict_pep;
    }
    }
    pep = ctx->pep.encrypted ? &ctx->pep : NULL;

    /* Receive SENDER_CONNECTION_INFO (SSID 0) */
    ret = ipmx_usb_recv_message(s, pep, ctx->ctrl_sock,
                                ctx->ctrl_recv_buf,
                                &ctx->ctrl_recv_buf_len,
                                &ctx->ctrl_recv_buf_pos,
                                0, &ctx->ctrl_ctr_s2r,
                                &ctx->ctrl_fp_s2r,
                                &msg, CTRL_TIMEOUT_MS);
    if (ret == IPMX_USB_ERR_MAC_FAIL) {
        av_log(s, AV_LOG_ERROR,
               "SENDER_CONNECTION_INFO MAC failed — "
               "cannot establish control channel\n");
        ret = AVERROR(EPROTO);
        goto fail;
    }
    if (ret < 0) {
        av_log(s, AV_LOG_ERROR,
               "Failed to receive SENDER_CONNECTION_INFO\n");
        goto fail;
    }
    if (msg.hdr.msg_type != IPMX_USB_SENDER_CONNECTION_INFO) {
        av_log(s, AV_LOG_ERROR,
               "Expected SENDER_CONNECTION_INFO, got 0x%02x\n",
               msg.hdr.msg_type);
        ret = AVERROR(EPROTO);
        goto fail;
    }
    av_log(s, AV_LOG_INFO, "Received SENDER_CONNECTION_INFO (serial=%.*s)\n",
           msg.payload.sender_connection_info.serial_len,
           msg.payload.sender_connection_info.serial_number);

    /* Start listening for data channels */
    {
    int p;
    ctx->data_listen_sock = ipmx_usb_tcp_listen(ctx->data_port);
    if (ctx->data_listen_sock < 0) {
        for (p = DEFAULT_DATA_PORT + 1; p < DEFAULT_DATA_PORT + 100; p++) {
            ctx->data_listen_sock = ipmx_usb_tcp_listen(p);
            if (ctx->data_listen_sock >= 0) {
                ctx->data_port = p;
                break;
            }
        }
        if (ctx->data_listen_sock < 0) {
            av_log(s, AV_LOG_ERROR, "Failed to bind data listen socket\n");
            ret = AVERROR(EIO);
            goto fail;
        }
    }
    }
    av_log(s, AV_LOG_INFO, "Data channel listening on port %d\n",
           ctx->data_port);

    /* Send SENDER_CONNECTION_STATUS (SSID 1) */
    len = ipmx_usb_build_sender_connection_status(
        build_buf, BUILD_BUF_SIZE,
        msg.payload.sender_connection_info.cid,
        ctx->serial ? ctx->serial : "",
        ctx->hbeat_index, ctx->data_port);
    if (len < 0) {
        av_log(s, AV_LOG_ERROR,
               "Failed to build SENDER_CONNECTION_STATUS\n");
        ret = AVERROR_EXIT;
        goto fail;
    }
    ret = ipmx_usb_send_message(s, pep, ctx->ctrl_sock, 1,
                                &ctx->ctrl_ctr_r2s, build_buf, len);
    if (ret < 0)
        goto fail;
    av_log(s, AV_LOG_DEBUG, "Sent SENDER_CONNECTION_STATUS\n");

    /* Compute heartbeat period for timeout monitoring */
    ctx->hbeat_period_s = hbeat_index_to_period(ctx->hbeat_index);
    av_log(s, AV_LOG_INFO,
           "Heartbeat period: %.1fs (index %d, timeout %.1fs)\n",
           ctx->hbeat_period_s, ctx->hbeat_index, ctx->hbeat_period_s * 2.0);

    /* Accept data channel connections from Sender.
     * Accept the first one (mandatory), then try accepting more with a
     * shorter timeout until no more arrive. */
    av_log(s, AV_LOG_INFO, "Waiting for Sender data connection(s)...\n");

    {
    int data_fd;
    data_fd = ipmx_usb_tcp_accept(ctx->data_listen_sock, DATA_ACCEPT_TIMEOUT_MS);
    if (data_fd < 0) {
        av_log(s, AV_LOG_ERROR,
               "Timeout waiting for first Sender data connection\n");
        ret = AVERROR(ETIMEDOUT);
        goto fail;
    }
    ctx->channels[0].sock = data_fd;
    ctx->num_channels = 1;
    av_log(s, AV_LOG_INFO, "Data channel 0 connected\n");

    while (ctx->num_channels < MAX_DATA_CHANNELS) {
        data_fd = ipmx_usb_tcp_accept(ctx->data_listen_sock, 500);
        if (data_fd < 0)
            break;
        ctx->channels[ctx->num_channels].sock = data_fd;
        av_log(s, AV_LOG_INFO, "Data channel %d connected\n",
               ctx->num_channels);
        ctx->num_channels++;
    }
    }
    av_log(s, AV_LOG_INFO, "Accepted %d data channel(s)\n", ctx->num_channels);

    /* Per-channel handshake, enumeration, stream creation */
    for (ch_idx = 0; ch_idx < ctx->num_channels; ch_idx++) {
        ret = setup_data_channel(s, &ctx->channels[ch_idx], ch_idx);
        if (ret < 0) {
            av_log(s, AV_LOG_ERROR,
                   "Data channel %d setup failed\n", ch_idx);
            goto fail;
        }
    }

    /* Resolve startup latency target early — needed for queue sizing. */
    if (ctx->latency_target_ms > 0)
        ctx->startup_target_us = (int64_t)ctx->latency_target_ms * 1000;
    else if (ctx->low_latency)
        ctx->startup_target_us = 33333;
    else
        ctx->startup_target_us = 100000;

    /* Allocate shared packet queue and spawn per-channel I/O threads.
     * Size the queue to hold at least the startup latency target worth
     * of packets (~8 ms per element) plus headroom for steady-state. */
    {
        int64_t target_us = ctx->startup_target_us;
        int target_elems = (int)(target_us / 8000) + 4;
        int q_size;
        /* In interleaved mode (1 channel), the queue holds both video
         * and audio packets.  At 60 video + 125 audio = 185 pkt/s, the
         * queue needs ~2.3× more capacity than a single-media estimate.
         * Add headroom for bursty arrivals. */
        if (ctx->num_channels == 1)
            target_elems = target_elems * 3;
        q_size = ctx->num_channels *
                 (target_elems > 4 ? target_elems : 4);
        ret = av_thread_message_queue_alloc(&ctx->pkt_queue, q_size,
                                            sizeof(AVPacket *));
        if (ret < 0) {
            av_log(s, AV_LOG_ERROR, "Failed to allocate pkt_queue\n");
            goto fail;
        }
        av_thread_message_queue_set_free_func(ctx->pkt_queue, pkt_queue_free);
    }

    for (ch_idx = 0; ch_idx < ctx->num_channels; ch_idx++) {
        ChannelThreadArg *ta = av_malloc(sizeof(*ta));
        if (!ta) {
            ret = AVERROR(ENOMEM);
            goto fail;
        }
        ta->s  = s;
        ta->ch = &ctx->channels[ch_idx];
        ret = pthread_create(&ctx->channels[ch_idx].io_thread, NULL,
                             channel_io_thread, ta);
        if (ret != 0) {
            av_free(ta);
            av_log(s, AV_LOG_ERROR,
                   "Failed to create I/O thread for channel %d\n", ch_idx);
            ret = AVERROR(ret);
            goto fail;
        }
        ctx->channels[ch_idx].io_thread_started = 1;
    }

    /* Start control monitor thread (heartbeat monitoring + reconnect) */
    ctx->last_hbeat_us = av_gettime_relative();
    ctx->ctrl_monitor_running = 1;
    ret = pthread_create(&ctx->ctrl_monitor, NULL, ctrl_monitor_thread, s);
    if (ret != 0) {
        av_log(s, AV_LOG_ERROR, "Failed to start ctrl_monitor thread\n");
        ctx->ctrl_monitor_running = 0;
        ret = AVERROR(ret);
        goto fail;
    }

    ctx->timing_fp = ipmx_timing_open("/tmp/ipmx_timing_receiver.log");
    if (ctx->timing_fp)
        fprintf(ctx->timing_fp,
                "# wall_us\tevent\tmedia\tcounter\treceive_us\textra\n"
                "# NET_RCV V extra: pts=<out>\tSF=<startframe>\tsfepoch=<us>\tcal=<0|1>\n"
                "# NET_RCV A extra: pts=<out>\tSF=<startframe_ms>\tsfepoch=<us>\tcal=<0|1>\n"
                "# PKT_DEQ extra: cap=<capture_wall_us>\n"
                "# PKT_DELIVER extra: cap=<capture_wall_us>\n"
                "# mode: channels=%d clock_recovery=%d low_latency=%d\n",
                ctx->num_channels,
                ctx->clockrec.clock_recovery,
                ctx->low_latency);

    /* Initialise clock recovery / latency measurement if requested. */
    if (ctx->clockrec.clock_recovery || ctx->clockrec.latency_report) {
        for (ch_idx = 0; ch_idx < ctx->num_channels; ch_idx++) {
            IpmxUsbDataChannel *ch = &ctx->channels[ch_idx];
            if (ch->has_audio && ch->audio_stream_idx >= 0) {
                AVStream *ast = s->streams[ch->audio_stream_idx];
                int bps = ch->uac.subframe_size ? ch->uac.subframe_size : 2;
                ctx->clockrec.sample_rate     = ast->codecpar->sample_rate;
                ctx->clockrec.channels        = ast->codecpar->ch_layout.nb_channels;
                ctx->clockrec.bytes_per_sample = bps;
                if (ch->uac.format_tag == UAC_FORMAT_IEEE_FLOAT && bps == 4)
                    ctx->clockrec.sample_fmt = AV_SAMPLE_FMT_FLT;
                else if (bps == 4)
                    ctx->clockrec.sample_fmt = AV_SAMPLE_FMT_S32;
                else if (bps == 3)
                    ctx->clockrec.sample_fmt = AV_SAMPLE_FMT_S32;
                else
                    ctx->clockrec.sample_fmt = AV_SAMPLE_FMT_S16;
                break;
            }
        }

        for (ch_idx = 0; ch_idx < ctx->num_channels; ch_idx++) {
            IpmxUsbDataChannel *ch = &ctx->channels[ch_idx];
            if (ch->has_video && ch->video_stream_idx >= 0) {
                AVStream *vst = s->streams[ch->video_stream_idx];
                if (vst->avg_frame_rate.den > 0 && vst->avg_frame_rate.num > 0)
                    ctx->clockrec.video_repeat_pts_inc =
                        (int64_t)(1000000.0 * vst->avg_frame_rate.den /
                                  vst->avg_frame_rate.num);
                else
                    ctx->clockrec.video_repeat_pts_inc = 33333;
                ctx->clockrec.scr_clock_freq = ch->uvc.clock_frequency;
                break;
            }
        }

        ret = ipmx_clockrec_init(&ctx->clockrec, s);
        if (ret < 0)
            av_log(s, AV_LOG_WARNING,
                   "Clock recovery init failed: %s (continuing without)\n",
                   av_err2str(ret));
    }

    /* Per-media latency targets for PTS-based pacer. */
    if (ctx->low_latency) {
        ctx->video_latency_target_us = 33333;  /* ~1 frame @ 30fps */
        ctx->audio_latency_target_us = 33333;
    } else {
        ctx->video_latency_target_us = 100000;
        ctx->audio_latency_target_us = 100000;
    }

    ctx->clockrec.latency_target_us = ctx->startup_target_us;
    ctx->startup_state = STARTUP_STEADY;  /* pacer replaces latch */

    av_log(s, AV_LOG_INFO,
           "IPMX USB device opened: %d channel(s), %d stream(s), "
           "%d I/O thread(s), latency video=%lld ms audio=%lld ms%s\n",
           ctx->num_channels, s->nb_streams, ctx->num_channels,
           (long long)(ctx->video_latency_target_us / 1000),
           (long long)(ctx->audio_latency_target_us / 1000),
           ctx->clockrec.clock_recovery ? " (paced)" : "");
    av_free(build_buf);
    return 0;

fail:
    av_free(build_buf);
    ipmx_usb_read_close(s);
    return ret;
}

/* ---------------------------------------------------------------------------
 * read_packet — send ISO_SUBMIT, receive and decode payloads
 * --------------------------------------------------------------------------- */

/* Process one round on a single data channel; returns 0 if pkt filled */
static int poll_channel(AVFormatContext *s, IpmxUsbDataChannel *ch,
                        AVPacket *pkt)
{
    IpmxUsbContext *ctx = s->priv_data;
    IpmxUsbPepState *pep = ctx->pep.encrypted ? &ctx->pep : NULL;
    IpmxUsbMsg msg;
    int ret;

    /* Feed ISO submits for both endpoints.
     *
     * TODO: Currently we request all video packets (ISO_VIDEO_NUM_PACKETS)
     * then all audio packets (ISO_AUDIO_NUM_PACKETS) in two large messages.
     * In interleaved mode (single data channel for A/V), a large video
     * response serialises ahead of audio on the shared TCP connection,
     * introducing head-of-line blocking.  For 1080p/4K on GbE the delay is
     * sub-millisecond and harmless.  If future use-cases (8K, multi-stream,
     * slower links) push the measured A-V serialisation delay above ~50 ms
     * (ITU-R BT.1359 lip-sync threshold), switch to finer-grained "sliced"
     * interleaving: request small video slices (e.g. 4 packets) alternated
     * with audio slices (1 packet) to emulate host-controller-level
     * scheduling.  This is a pure receiver-side change — the sender and the
     * TR-10-14 protocol require no modifications. */
    if (ch->has_video && !ch->video_iso_pending) {
        ret = send_iso_submit(s, ch, ch->uvc.vs_endpoint,
                              ISO_VIDEO_NUM_PACKETS,
                              ch->uvc.max_packet_size
                                  ? ch->uvc.max_packet_size
                                  : ISO_MAX_PACKET_SIZE);
        if (ret < 0)
            return ret;
        ch->video_iso_pending = 1;
        ctx->diag_vid_iso_sent++;
    }

    if (ch->has_audio && !ch->audio_iso_pending) {
        ret = send_iso_submit(s, ch, ch->uac.as_endpoint,
                              ISO_AUDIO_NUM_PACKETS,
                              ch->uac.max_packet_size
                                  ? ch->uac.max_packet_size
                                  : 192);
        if (ret < 0)
            return ret;
        ch->audio_iso_pending = 1;
    }

    ret = ipmx_usb_recv_message(s, pep, ch->sock,
                                ch->recv_buf, &ch->recv_buf_len,
                                &ch->recv_buf_pos,
                                ch->substreamid, &ch->ctr_s2r,
                                &ch->fp_s2r,
                                &msg, ISO_TIMEOUT_MS);
    if (ret == IPMX_USB_ERR_MAC_FAIL) {
        if (ch->fp_s2r.strict)
            return AVERROR(EPROTO);
        av_log(s, AV_LOG_WARNING,
               "Data I/O MAC failed (ssid=%u) — discarding message\n",
               ch->substreamid);
        /* Treat as if the pending submit received an error:
         * mark pending slots as available so new submits can be issued. */
        if (msg.hdr.msg_type == IPMX_USB_ISOCHRONOUS_SUBMIT_RETURN) {
            ch->video_iso_pending = 0;
            ch->audio_iso_pending = 0;
        }
        return AVERROR(EAGAIN);
    }
    if (ret < 0)
        return ret;

    if (msg.hdr.msg_type == IPMX_USB_ISOCHRONOUS_SUBMIT_RETURN) {
        const IpmxUsbIsoSubmitReturn *iso = &msg.payload.iso_submit_return;
        uint8_t ep = iso->endpoint;
        int got_data = 0;

        if (ch->has_video && ep == ch->uvc.vs_endpoint) {
            ch->video_iso_pending = 0;
            ret = process_video_iso_return(s, ch, iso, pkt);
            if (ret == 0) {
                ch->last_data_time = av_gettime_relative();
                ctx->diag_vid_frames++;
                return 0;
            }
            got_data = (iso->num_packets > 0 && iso->transferdata_len > 0);
        } else if (ch->has_audio && ep == ch->uac.as_endpoint) {
            ch->audio_iso_pending = 0;
            ret = process_audio_iso_return(s, ch, iso, pkt);
            if (ret == 0) {
                ch->last_data_time = av_gettime_relative();
                ctx->diag_aud_frames++;
                return 0;
            }
            got_data = (iso->num_packets > 0 && iso->transferdata_len > 0);
        } else {
            av_log(s, AV_LOG_DEBUG,
                   "ISO_SUBMIT_RETURN for unknown ep=0x%02x\n", ep);
        }

        if (got_data) {
            ch->last_data_time = av_gettime_relative();
        } else {
            ctx->diag_empty_returns++;

            /* Drain: when we got an empty response but the other media
             * is still pending, its response is likely next in the TCP
             * buffer.  Read it now (non-blocking) to prevent head-of-line
             * starvation where one media monopolises the recv loop. */
            if (ch->video_iso_pending || ch->audio_iso_pending) {
                IpmxUsbMsg msg2;
                int ret2 = ipmx_usb_recv_message(s, pep, ch->sock,
                        ch->recv_buf, &ch->recv_buf_len,
                        &ch->recv_buf_pos,
                        ch->substreamid, &ch->ctr_s2r,
                        &ch->fp_s2r,
                        &msg2, 0);  /* non-blocking */
                if (ret2 >= 0 &&
                    msg2.hdr.msg_type == IPMX_USB_ISOCHRONOUS_SUBMIT_RETURN) {
                    const IpmxUsbIsoSubmitReturn *iso2 =
                        &msg2.payload.iso_submit_return;
                    uint8_t ep2 = iso2->endpoint;
                    if (ch->has_video && ep2 == ch->uvc.vs_endpoint) {
                        ch->video_iso_pending = 0;
                        ret = process_video_iso_return(s, ch, iso2, pkt);
                        if (ret == 0) {
                            ch->last_data_time = av_gettime_relative();
                            ctx->diag_vid_frames++;
                            return 0;
                        }
                    } else if (ch->has_audio && ep2 == ch->uac.as_endpoint) {
                        ch->audio_iso_pending = 0;
                        ret = process_audio_iso_return(s, ch, iso2, pkt);
                        if (ret == 0) {
                            ch->last_data_time = av_gettime_relative();
                            ctx->diag_aud_frames++;
                            return 0;
                        }
                    }
                }
            }

            av_usleep(100);
        }
    } else if (msg.hdr.msg_type == IPMX_USB_STREAM_RESET) {
        av_log(s, AV_LOG_WARNING, "STREAM_RESET on channel ssid=%u\n",
               ch->substreamid);
        ch->video_iso_pending = 0;
        ch->audio_iso_pending = 0;
    }

    return AVERROR(EAGAIN);
}

/* -----------------------------------------------------------------------
 * Pacer helpers: at-dequeue processing and slot management
 * ----------------------------------------------------------------------- */

/**
 * Process a freshly dequeued packet: PLL feed, SCR, resample, audio rate.
 * Called once per packet when it enters a pacer slot (not at delivery).
 */
static void pacer_process_dequeued(AVFormatContext *s, AVPacket *queued)
{
    IpmxUsbContext *ctx = s->priv_data;

    if (ctx->timing_fp) {
        int64_t deq_wall = av_gettime_relative();
        int64_t rcv_us = ipmx_timing_read_pkt(queued);
        int64_t cap_us = ipmx_timing_read_capture(queued);
        char is_v = (queued->stream_index == 0) ? 'V' : 'A';
        fprintf(ctx->timing_fp,
                "%lld\tPKT_DEQ\t%c\t%lld\t%lld\tcap=%lld\n",
                (long long)deq_wall,
                is_v,
                (long long)ctx->timing_deq_cnt++,
                (long long)rcv_us,
                (long long)cap_us);
    }

    /* Clock recovery / latency measurement */
    if (ctx->clockrec.clock_recovery || ctx->clockrec.latency_report) {
        int64_t sender_abs_us = ipmx_timing_read_sender(queued);
        int64_t arrival_us    = ipmx_timing_read_pkt(queued);
        int pkt_is_audio = (queued->stream_index >= 0 &&
                            queued->stream_index < (int)s->nb_streams &&
                            s->streams[queued->stream_index]->codecpar
                                ->codec_type == AVMEDIA_TYPE_AUDIO);

        if (sender_abs_us > 0 && arrival_us > 0)
            ipmx_clockrec_add_timestamp(&ctx->clockrec,
                                        sender_abs_us, arrival_us,
                                        pkt_is_audio, s);

        if (!pkt_is_audio && queued->opaque_ref &&
            queued->opaque_ref->size >= (int)sizeof(IpmxTimingOpaque)) {
            const IpmxTimingOpaque *t =
                (const IpmxTimingOpaque *)queued->opaque_ref->data;
            if (t->has_scr)
                ipmx_clockrec_add_scr(&ctx->clockrec,
                                      t->scr_stc, t->scr_sof, s);
        }

        if (pkt_is_audio)
            ipmx_clockrec_resample_audio(&ctx->clockrec, queued);

        if (!pkt_is_audio)
            ctx->clockrec.video_rx_frames++;
    }

    if (ctx->clockrec.clock_recovery && ctx->clockrec.is_pcm &&
        queued->stream_index >= 0 && queued->stream_index < (int)s->nb_streams &&
        s->streams[queued->stream_index]->codecpar->codec_type ==
            AVMEDIA_TYPE_AUDIO) {
        int64_t rx_samples;
        int64_t tx_samples;
        int64_t out_samples = ipmx_usb_audio_packet_samples(s, queued);

        ipmx_usb_audio_rate_note_handoff(ctx, out_samples);
        ipmx_usb_audio_rate_snapshot(ctx, &rx_samples, &tx_samples);
        ipmx_clockrec_update_audio_rate(&ctx->clockrec, rx_samples, tx_samples);
    }
}

/**
 * Try to fill empty pacer slots from pkt_queue (non-blocking).
 * Only dequeues when an empty slot exists — never loses a packet.
 * Returns the number of slots filled this call.
 */
static int pacer_fill_slots(AVFormatContext *s)
{
    IpmxUsbContext *ctx = s->priv_data;
    int filled = 0;
    int i;

    for (;;) {
        AVPacket *pkt = NULL;
        int empty_slot = -1;
        int ret;

        /* Find an empty slot */
        for (i = 0; i < ctx->pacer_n_slots; i++) {
            if (!ctx->pacer_slots[i].pkt) {
                empty_slot = i;
                break;
            }
        }
        if (empty_slot < 0)
            break;  /* all slots full — stop dequeuing */

        ret = av_thread_message_queue_recv(ctx->pkt_queue, &pkt,
                                           AV_THREAD_MESSAGE_NONBLOCK);
        if (ret < 0)
            break;  /* queue empty or error */

        pacer_process_dequeued(s, pkt);
        ctx->pacer_slots[empty_slot].pkt = pkt;
        filled++;
    }

    return filled;
}

/**
 * Find the slot with the earliest PTS.  Returns slot index, or -1
 * if no slot is filled.
 */
static int pacer_earliest_slot(IpmxUsbContext *ctx)
{
    int best = -1;
    int64_t best_pts = INT64_MAX;
    int i;

    for (i = 0; i < ctx->pacer_n_slots; i++) {
        if (ctx->pacer_slots[i].pkt &&
            ctx->pacer_slots[i].pkt->pts < best_pts) {
            best_pts = ctx->pacer_slots[i].pkt->pts;
            best = i;
        }
    }
    return best;
}

/* ----------------------------------------------------------------------- */

static int ipmx_usb_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    IpmxUsbContext *ctx = s->priv_data;
    AVPacket *queued = NULL;
    int ret;

    if (ctx->stop)
        return AVERROR_EOF;

    /* Open diag log on first call (only with --log / IPMX_TIMING=1) */
    if (!ctx->diag_fp && ctx->timing_fp) {
        ctx->diag_fp = fopen("/tmp/ipmx_usb_receiver_diag.log", "w");
        if (ctx->diag_fp)
            setvbuf(ctx->diag_fp, NULL, _IOLBF, 0);
    }

    if (ctx->diag_fp) {
        int64_t now = av_gettime_relative();
        if (now >= ctx->diag_next_log) {
            fprintf(ctx->diag_fp,
                    "t=%lld vid_frames=%lld aud_frames=%lld "
                    "vid_iso_sent=%lld empty_ret=%lld\n",
                    (long long)(now / 1000000),
                    (long long)ctx->diag_vid_frames,
                    (long long)ctx->diag_aud_frames,
                    (long long)ctx->diag_vid_iso_sent,
                    (long long)ctx->diag_empty_returns);
            ctx->diag_next_log = now + 2000000;
        }
    }

    /* N-slot pacer scheduler.
     *
     * Maintains one packet per stream.  On each call:
     * 1. Fill empty slots from pkt_queue (non-blocking)
     * 2. Find the slot with the earliest PTS
     * 3. If its PTS is due (now >= pts): deliver it
     * 4. Otherwise sleep 500 us and go to 1
     *
     * This decouples audio and video pacing: sleeping for an audio
     * packet never delays a video packet that's due, and vice versa.
     *
     * Pre-calibration packets (mid_calibrated=0) bypass the scheduler
     * and are delivered immediately (no pacing). */

    if (!ctx->clockrec.clock_recovery || !ctx->clockrec.mid_calibrated) {
        /* No pacing — pass through directly */
        ret = av_thread_message_queue_recv(ctx->pkt_queue, &queued, 0);
        if (ret < 0)
            return ret;
        pacer_process_dequeued(s, queued);
        av_packet_move_ref(pkt, queued);
        av_packet_free(&queued);
        return 0;
    }

    /* --- Calibrated: slot-window scheduler --- */
    {
        int best;
        int64_t now;

        /* Initialize slot count on first calibrated call */
        if (!ctx->pacer_n_slots) {
            int ns = (int)s->nb_streams * PACER_SLOT_FACTOR;
            ctx->pacer_n_slots = ns < PACER_MAX_SLOTS ? ns : PACER_MAX_SLOTS;
            if (ctx->pacer_n_slots < 2)
                ctx->pacer_n_slots = 2;
        }

        for (;;) {
            if (ctx->stop)
                return AVERROR_EOF;

            /* 1. Fill empty slots from the input queue (non-blocking) */
            pacer_fill_slots(s);

            /* 2. Find the slot with the earliest PTS */
            best = pacer_earliest_slot(ctx);

            if (best < 0) {
                /* All slots empty — blocking dequeue into slot 0 */
                AVPacket *bpkt = NULL;

                ret = av_thread_message_queue_recv(ctx->pkt_queue, &bpkt, 0);
                if (ret < 0)
                    return ret;

                pacer_process_dequeued(s, bpkt);
                ctx->pacer_slots[0].pkt = bpkt;
                continue;  /* re-enter to fill more + find earliest */
            }

            queued = ctx->pacer_slots[best].pkt;

            /* Capture PTS epoch from first calibrated packet */
            if (!ctx->pts_epoch_set) {
                ctx->pts_epoch = queued->pts;
                ctx->pts_epoch_set = 1;
            }

            /* 3. Is it due? */
            now = av_gettime_relative();

            if (queued->pts != AV_NOPTS_VALUE &&
                queued->pts - now > PACER_POLL_US) {
                /* Not yet — sleep 500 us and re-check */
                av_usleep(PACER_POLL_US);
                continue;
            }

            /* --- Packet is due: deliver --- */

            /* Video pace decision (skip/repeat) */
            if (queued->stream_index >= 0 &&
                queued->stream_index < (int)s->nb_streams &&
                s->streams[queued->stream_index]->codecpar->codec_type ==
                    AVMEDIA_TYPE_VIDEO) {
                int decision = ipmx_clockrec_video_pace_decision(
                                   &ctx->clockrec, queued->pts, now);

                if (decision > 0) {
                    /* Skip: frame is stale — discard, clear slot */
                    ctx->clockrec.video_frames_skipped++;
                    ctx->clockrec.video_consecutive_skips++;
                    av_packet_free(&ctx->pacer_slots[best].pkt);
                    ctx->pacer_slots[best].pkt = NULL;
                    continue;
                }

                if (decision < 0 && ctx->last_video_pkt) {
                    /* Repeat: too early — deliver previous frame, keep
                     * the current packet in its slot for next time. */
                    AVPacket *repeat = av_packet_clone(ctx->last_video_pkt);
                    if (repeat) {
                        if (ctx->clockrec.video_repeat_pts_inc > 0)
                            repeat->pts += ctx->clockrec.video_repeat_pts_inc;
                        av_packet_free(&ctx->last_video_pkt);
                        ctx->last_video_pkt = av_packet_clone(repeat);
                        ctx->clockrec.video_frames_repeated++;

                        repeat->pts -= ctx->pts_epoch;
                        repeat->dts = repeat->pts;

                        av_packet_move_ref(pkt, repeat);
                        av_packet_free(&repeat);
                        return 0;
                    }
                    /* Clone failed — fall through to normal delivery */
                }

                /* Normal: reset consecutive skip counter */
                ctx->clockrec.video_consecutive_skips = 0;
            }

            /* Take packet from slot */
            ctx->pacer_slots[best].pkt = NULL;
            ctx->clockrec.last_dequeue_us = now;

            /* Log delivery timing (post-pacer) */
            if (ctx->timing_fp) {
                int64_t del_wall = av_gettime_relative();
                int64_t rcv_us = ipmx_timing_read_pkt(queued);
                int64_t cap_us = ipmx_timing_read_capture(queued);
                char is_v = (queued->stream_index == 0) ? 'V' : 'A';
                fprintf(ctx->timing_fp,
                        "%lld\tPKT_DELIVER\t%c\t%lld\t%lld\tcap=%lld\n",
                        (long long)del_wall,
                        is_v,
                        (long long)ctx->timing_deq_cnt,
                        (long long)rcv_us,
                        (long long)cap_us);
            }

            /* ASRC feedback: update cumulative sample balance correction.
             * Uses total_samples_in/out from the ASRC (already updated
             * in pacer_process_dequeued -> resample_audio). */
#if CLOCKREC_ASRC_FEEDBACK
            if (queued->stream_index >= 0 &&
                queued->stream_index < (int)s->nb_streams &&
                s->streams[queued->stream_index]->codecpar->codec_type ==
                    AVMEDIA_TYPE_AUDIO)
                ipmx_clockrec_fb_update(&ctx->clockrec);
#endif

            /* Convert absolute PTS to relative for ffplay */
            queued->pts -= ctx->pts_epoch;
            queued->dts = queued->pts;

            /* Stash last video frame for repeat + count delivered */
            if (queued->stream_index >= 0 &&
                queued->stream_index < (int)s->nb_streams &&
                s->streams[queued->stream_index]->codecpar->codec_type ==
                    AVMEDIA_TYPE_VIDEO) {
                av_packet_free(&ctx->last_video_pkt);
                ctx->last_video_pkt = av_packet_clone(queued);
                ctx->clockrec.video_tx_frames++;
            }

            av_packet_move_ref(pkt, queued);
            av_packet_free(&queued);
            return 0;
        }
    }
}

/* ---------------------------------------------------------------------------
 * read_close — shutdown everything
 * --------------------------------------------------------------------------- */

static int ipmx_usb_read_close(AVFormatContext *s)
{
    IpmxUsbContext *ctx = s->priv_data;
    int i;

    ipmx_clockrec_free(&ctx->clockrec, s);
    av_packet_free(&ctx->last_video_pkt);

    /* Free any packets remaining in pacer slots */
    for (i = 0; i < PACER_MAX_SLOTS; i++) {
        if (ctx->pacer_slots[i].pkt)
            av_packet_free(&ctx->pacer_slots[i].pkt);
    }

    /* timing_fp and diag_fp are closed after thread joins below —
     * the I/O threads use fprintf(ctx->timing_fp) and closing here
     * while they're still running causes a use-after-free crash. */

    if (ctx->diag_fp) {
        fprintf(ctx->diag_fp, "READ_CLOSE vid=%lld aud=%lld\n",
                (long long)ctx->diag_vid_frames,
                (long long)ctx->diag_aud_frames);
        fclose(ctx->diag_fp);
        ctx->diag_fp = NULL;
    }

    ctx->stop = 1;

    /* Unblock read_packet (consumer) and channel threads (producers) */
    if (ctx->pkt_queue) {
        av_thread_message_queue_set_err_recv(ctx->pkt_queue, AVERROR_EOF);
        av_thread_message_queue_set_err_send(ctx->pkt_queue, AVERROR_EOF);
    }

    /* Join channel I/O threads */
    for (i = 0; i < ctx->num_channels; i++) {
        IpmxUsbDataChannel *ch = &ctx->channels[i];
        if (ch->io_thread_started) {
            pthread_join(ch->io_thread, NULL);
            ch->io_thread_started = 0;
        }
    }

    if (ctx->ctrl_monitor_running) {
        pthread_join(ctx->ctrl_monitor, NULL);
        ctx->ctrl_monitor_running = 0;
    }

    /* Now safe to close log files — all threads have exited. */
    if (ctx->timing_fp) {
        fclose(ctx->timing_fp);
        ctx->timing_fp = NULL;
    }

    for (i = 0; i < ctx->num_channels; i++) {
        IpmxUsbDataChannel *ch = &ctx->channels[i];
        if (ch->sock >= 0) {
            close(ch->sock);
            ch->sock = -1;
        }
        av_freep(&ch->recv_buf);
        ipmx_uvc_free_reassembler(&ch->uvc);
    }
    ctx->num_channels = 0;

    if (ctx->pkt_queue) {
        av_thread_message_flush(ctx->pkt_queue);
        av_thread_message_queue_free(&ctx->pkt_queue);
    }
    ipmx_usb_audio_rate_reset(ctx);

    if (ctx->data_listen_sock >= 0) {
        close(ctx->data_listen_sock);
        ctx->data_listen_sock = -1;
    }
    if (ctx->ctrl_sock >= 0) {
        close(ctx->ctrl_sock);
        ctx->ctrl_sock = -1;
    }

    pthread_mutex_destroy(&ctx->io_threads_mutex);
    pthread_mutex_destroy(&ctx->audio_rate_mutex);

    av_log(s, AV_LOG_INFO, "IPMX USB device closed\n");
    return 0;
}

/* ---------------------------------------------------------------------------
 * FFInputFormat registration
 * --------------------------------------------------------------------------- */

const FFInputFormat ff_ipmx_usb_demuxer = {
    .p.name         = "ipmx_usb",
    .p.long_name    = NULL_IF_CONFIG_SMALL("IPMX USB (TR-10-14) input device"),
    .p.flags        = AVFMT_NOFILE,
    .p.priv_class   = &ipmx_usb_class,
    .priv_data_size = sizeof(IpmxUsbContext),
    .read_header    = ipmx_usb_read_header,
    .read_packet    = ipmx_usb_read_packet,
    .read_close     = ipmx_usb_read_close,
};
