/*
 * RTP output format
 * Copyright (c) 2002 Fabrice Bellard
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include "avformat.h"
#include "mpegts.h"
#include "internal.h"
#include "mux.h"
#include "libavutil/mathematics.h"
#include "libavutil/mem.h"
#include "libavutil/pixdesc.h"
#include "libavutil/random_seed.h"
#include "libavutil/opt.h"

#include "rtp.h"
#include "rtpenc.h"
#include "rtp_cipher.h"
#include "rtp_pep.h"

static const AVOption options[] = {
    FF_RTP_FLAG_OPTS(RTPMuxContext, flags),
    { "payload_type", "Specify RTP payload type", offsetof(RTPMuxContext, payload_type), AV_OPT_TYPE_INT, {.i64 = -1 }, -1, 127, AV_OPT_FLAG_ENCODING_PARAM },
    { "ssrc", "Stream identifier", offsetof(RTPMuxContext, ssrc), AV_OPT_TYPE_UINT, { .i64 = 0 }, 0, UINT32_MAX, AV_OPT_FLAG_ENCODING_PARAM },
    { "cname", "CNAME to include in RTCP SR packets", offsetof(RTPMuxContext, cname), AV_OPT_TYPE_STRING, { .str = NULL }, 0, 0, AV_OPT_FLAG_ENCODING_PARAM },
    { "seq", "Starting sequence number", offsetof(RTPMuxContext, seq), AV_OPT_TYPE_INT, { .i64 = -1 }, -1, 65535, AV_OPT_FLAG_ENCODING_PARAM },
    { NULL },
};

static const AVClass rtp_muxer_class = {
    .class_name = "RTP muxer",
    .item_name  = av_default_item_name,
    .option     = options,
    .version    = LIBAVUTIL_VERSION_INT,
};

#define RTCP_SR_SIZE 28

static int is_supported(enum AVCodecID id)
{
    switch(id) {
    case AV_CODEC_ID_DIRAC:
    case AV_CODEC_ID_H261:
    case AV_CODEC_ID_H263:
    case AV_CODEC_ID_H263P:
    case AV_CODEC_ID_H264:
    case AV_CODEC_ID_HEVC:
    case AV_CODEC_ID_MPEG1VIDEO:
    case AV_CODEC_ID_MPEG2VIDEO:
    case AV_CODEC_ID_MPEG4:
    case AV_CODEC_ID_AAC:
    case AV_CODEC_ID_MP2:
    case AV_CODEC_ID_MP3:
    case AV_CODEC_ID_PCM_ALAW:
    case AV_CODEC_ID_PCM_MULAW:
    case AV_CODEC_ID_PCM_S8:
    case AV_CODEC_ID_PCM_S16BE:
    case AV_CODEC_ID_PCM_S16LE:
    case AV_CODEC_ID_PCM_S24BE:
    case AV_CODEC_ID_PCM_U16BE:
    case AV_CODEC_ID_PCM_U16LE:
    case AV_CODEC_ID_PCM_U8:
    case AV_CODEC_ID_MPEG2TS:
    case AV_CODEC_ID_AMR_NB:
    case AV_CODEC_ID_AMR_WB:
    case AV_CODEC_ID_VORBIS:
    case AV_CODEC_ID_THEORA:
    case AV_CODEC_ID_VP8:
    case AV_CODEC_ID_VP9:
    case AV_CODEC_ID_AV1:
    case AV_CODEC_ID_ADPCM_G722:
    case AV_CODEC_ID_ADPCM_G726:
    case AV_CODEC_ID_ADPCM_G726LE:
    case AV_CODEC_ID_ILBC:
    case AV_CODEC_ID_MJPEG:
    case AV_CODEC_ID_SPEEX:
    case AV_CODEC_ID_OPUS:
    case AV_CODEC_ID_RAWVIDEO:
    case AV_CODEC_ID_BITPACKED:
    case AV_CODEC_ID_G728:
    case AV_CODEC_ID_AM824:
        return 1;
    default:
        return 0;
    }
}

static int rtp_write_header(AVFormatContext *s1)
{
    RTPMuxContext *s = s1->priv_data;
    int n, ret = AVERROR(EINVAL);
    AVStream *st;

    if (s1->nb_streams != 1) {
        av_log(s1, AV_LOG_ERROR, "Only one stream supported in the RTP muxer\n");
        return AVERROR(EINVAL);
    }
    st = s1->streams[0];
    if (!is_supported(st->codecpar->codec_id)) {
        av_log(s1, AV_LOG_ERROR, "Unsupported codec %s\n", avcodec_get_name(st->codecpar->codec_id));

        return -1;
    }

    if (s->payload_type < 0) {
        /* Re-validate non-dynamic payload types */
        if (st->id < RTP_PT_PRIVATE)
            st->id = ff_rtp_get_payload_type(s1, st->codecpar, -1);

        s->payload_type = st->id;
    } else {
        /* private option takes priority */
        st->id = s->payload_type;
    }

    if (s1->hdcp_scramble) {
        if (!s1->video_hdcp_cipher.callback) {
            s1->video_hdcp_cipher.callback = dummy_xor_hdcp_cipher;
            s1->video_hdcp_cipher.param = 1;
        }
        if (!s1->audio_hdcp_cipher.callback) {
            s1->audio_hdcp_cipher.callback = dummy_xor_hdcp_cipher;
            s1->audio_hdcp_cipher.param = 1;
        }
        if (!s1->data_hdcp_cipher.callback) {
            s1->data_hdcp_cipher.callback = dummy_xor_hdcp_cipher;
            s1->data_hdcp_cipher.param = 1;
        }
        s1->full_ctr = 1;
    }

    if (s1->privacy_encrypt) {
        if (s1->privacy_scramble) {
            av_log(s1, AV_LOG_ERROR, "privacy_encrypt and privacy_scramble are mutually exclusive\n");
            return AVERROR(EINVAL);
        }
        {
            RtpPepContext *pep_ctx = av_mallocz(sizeof(*pep_ctx));
            if (!pep_ctx)
                return AVERROR(ENOMEM);
            if (rtp_pep_init(pep_ctx, s1, s1->pep_psk, s1->pep_mode,
                             s1->pep_iv, s1->pep_key_generator,
                             s1->pep_key_version, s1->pep_rtp_kv) < 0) {
                av_free(pep_ctx);
                return AVERROR(EINVAL);
            }
            /* Wire kv_rotate: only valid with RTP_KV */
            if (s1->pep_kv_rotate > 0) {
                if (!s1->pep_rtp_kv) {
                    av_log(s1, AV_LOG_ERROR, "pep_kv_rotate requires pep_rtp_kv=1\n");
                    av_free(pep_ctx);
                    return AVERROR(EINVAL);
                }
                pep_ctx->kv_rotate_slices = s1->pep_kv_rotate;
            }
            s1->video_privacy_cipher.callback = rtp_pep_cipher_callback;
            s1->video_privacy_cipher.opaque = pep_ctx;
            s1->video_privacy_cipher.param = pep_ctx->key_version;
            s1->audio_privacy_cipher.callback = rtp_pep_cipher_callback;
            s1->audio_privacy_cipher.opaque = pep_ctx;
            s1->audio_privacy_cipher.param = pep_ctx->key_version;
            s1->data_privacy_cipher.callback = rtp_pep_cipher_callback;
            s1->data_privacy_cipher.opaque = pep_ctx;
            s1->data_privacy_cipher.param = pep_ctx->key_version;
            /* Set owner_cb so rotation can update .param for the active stream type */
            pep_ctx->owner_cb = &s1->video_privacy_cipher; /* default; codec_type selection below overrides */
            s1->full_ctr = 1;
            s1->privacy_mac_size = av_ipmx_pep_mode_has_cmac(pep_ctx->mode)
                                   ? AV_IPMX_PEP_CMAC_TAG_SIZE : 0;
        }
    } else if (s1->privacy_scramble) {
        if (!s1->video_privacy_cipher.callback) {
            s1->video_privacy_cipher.callback = dummy_xor_privacy_cipher;
            s1->video_privacy_cipher.param = 1;
        }
        if (!s1->audio_privacy_cipher.callback) {
            s1->audio_privacy_cipher.callback = dummy_xor_privacy_cipher;
            s1->audio_privacy_cipher.param = 1;
        }
        if (!s1->data_privacy_cipher.callback) {
            s1->data_privacy_cipher.callback = dummy_xor_privacy_cipher;
            s1->data_privacy_cipher.param = 1;
        }
        s1->full_ctr = 1;
    }

    {
        struct AVCipherCB * hdcp_cipher;
        struct AVCipherCB * privacy_cipher;

        if (st->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
            hdcp_cipher = &s1->video_hdcp_cipher;
            privacy_cipher = &s1->video_privacy_cipher;
        } else if (st->codecpar->codec_type == AVMEDIA_TYPE_AUDIO) {
            hdcp_cipher = &s1->audio_hdcp_cipher;
            privacy_cipher = &s1->audio_privacy_cipher;
        } else {
            hdcp_cipher = &s1->data_hdcp_cipher;
            privacy_cipher = &s1->data_privacy_cipher;
        }

        if (privacy_cipher && privacy_cipher->callback) {
            s->privacy_cipher = privacy_cipher;
            s->encryption_input_ctr = 0;
            s->encryption_stream_ctr = privacy_cipher->param;
            /* Update owner_cb to point to the actual stream-type cipher */
            if (privacy_cipher->opaque) {
                RtpPepContext *pep = (RtpPepContext *)privacy_cipher->opaque;
                if (pep->kv_rotate_slices > 0)
                    pep->owner_cb = privacy_cipher;
            }
        }

        if (hdcp_cipher && hdcp_cipher->callback) {
            s->hdcp_cipher = hdcp_cipher;
            s->encryption_input_ctr = 0;
            s->encryption_stream_ctr = hdcp_cipher->param;
        }
    }

    s->base_timestamp = av_get_random_seed();
    s->timestamp = s->base_timestamp;
    s->cur_timestamp = 0;
    if (!s->ssrc)
        s->ssrc = av_get_random_seed();
    s->first_packet = 1;
    s->first_rtcp_ntp_time = ff_ntp_time();
    if (s1->start_time_realtime != 0  &&  s1->start_time_realtime != AV_NOPTS_VALUE)
        /* Round the NTP time to whole milliseconds. */
        s->first_rtcp_ntp_time = (s1->start_time_realtime / 1000) * 1000 +
                                 NTP_OFFSET_US;
    // Pick a random sequence start number, but in the lower end of the
    // available range, so that any wraparound doesn't happen immediately.
    // (Immediate wraparound would be an issue for SRTP.)
    if (s->seq < 0) {
        if (s1->flags & AVFMT_FLAG_BITEXACT) {
            s->seq = 0;
        } else
            s->seq = av_get_random_seed() & 0x0fff;
    } else
        s->seq &= 0xffff; // Use the given parameter, wrapped to the right interval

    if (s1->packet_size) {
        if (s1->pb->max_packet_size)
            s1->packet_size = FFMIN(s1->packet_size,
                                    s1->pb->max_packet_size);
    } else
        s1->packet_size = s1->pb->max_packet_size;
    if (s1->packet_size <= 12) {
        av_log(s1, AV_LOG_ERROR, "Max packet size %u too low\n", s1->packet_size);
        return AVERROR(EIO);
    }
    s->buf = av_malloc(s1->packet_size);
    if (!s->buf) {
        return AVERROR(ENOMEM);
    }
    s->max_payload_size = s1->packet_size - 12;

    // If there is encryption then we must reserve space for the extended header. We reserve
    // space the the full header because some formats like MPT2-TS and other audio ones require
    // to write an integral number of frames or packets into the payload such that we cannot
    // dynamically reserve for either the full or short header. The inneficiency is mainly
    // caused by the ffmpeg architecture.
    if (s->hdcp_cipher || s->privacy_cipher) {
        s->max_payload_size = s->max_payload_size - sizeof(struct RTP_ExtHdrFull);
        /* Reserve space for CMAC-64 MAC tag (8 bytes) if CMAC mode is active */
        s->max_payload_size -= s1->privacy_mac_size;
    }

    if (st->codecpar->codec_type == AVMEDIA_TYPE_AUDIO) {
        avpriv_set_pts_info(st, 32, 1, st->codecpar->sample_rate);
    } else {
        avpriv_set_pts_info(st, 32, 1, 90000);
    }
    s->buf_ptr = s->buf;
    switch(st->codecpar->codec_id) {
    case AV_CODEC_ID_MP2:
    case AV_CODEC_ID_MP3:
        s->buf_ptr = s->buf + 4;
        avpriv_set_pts_info(st, 32, 1, 90000);
        break;
    case AV_CODEC_ID_MPEG1VIDEO:
    case AV_CODEC_ID_MPEG2VIDEO:
        break;
    case AV_CODEC_ID_MPEG2TS:
        if (s->max_payload_size < TS_PACKET_SIZE) {
            av_log(s1, AV_LOG_ERROR,
                   "RTP payload size %u too small for MPEG-TS "
                   "(minimum %d bytes required)\n",
                   s->max_payload_size, TS_PACKET_SIZE);
            ret = AVERROR(EINVAL);
            goto fail;
        }

        n = s->max_payload_size / TS_PACKET_SIZE;
        s->max_payload_size = n * TS_PACKET_SIZE;
        break;
    case AV_CODEC_ID_DIRAC:
        if (s1->strict_std_compliance > FF_COMPLIANCE_EXPERIMENTAL) {
            av_log(s, AV_LOG_ERROR,
                   "Packetizing VC-2 is experimental and does not use all values "
                   "of the specification "
                   "(even though most receivers may handle it just fine). "
                   "Please set -strict experimental in order to enable it.\n");
            ret = AVERROR_EXPERIMENTAL;
            goto fail;
        }
        break;
    case AV_CODEC_ID_H261:
        if (s1->strict_std_compliance > FF_COMPLIANCE_EXPERIMENTAL) {
            av_log(s, AV_LOG_ERROR,
                   "Packetizing H.261 is experimental and produces incorrect "
                   "packetization for cases where GOBs don't fit into packets "
                   "(even though most receivers may handle it just fine). "
                   "Please set -f_strict experimental in order to enable it.\n");
            ret = AVERROR_EXPERIMENTAL;
            goto fail;
        }
        break;
    case AV_CODEC_ID_H264:
        /* check for H.264 MP4 syntax */
        if (st->codecpar->extradata_size > 4 && st->codecpar->extradata[0] == 1) {
            s->nal_length_size = (st->codecpar->extradata[4] & 0x03) + 1;
        }
        break;
    case AV_CODEC_ID_HEVC:
        /* Only check for the standardized hvcC version of extradata, keeping
         * things simple and similar to the avcC/H.264 case above, instead
         * of trying to handle the pre-standardization versions (as in
         * libavcodec/hevc.c). */
        if (st->codecpar->extradata_size > 21 && st->codecpar->extradata[0] == 1) {
            s->nal_length_size = (st->codecpar->extradata[21] & 0x03) + 1;
        }
        break;
    case AV_CODEC_ID_MJPEG:
        if (st->codecpar->width <= 0 || st->codecpar->height <= 0) {
            av_log(s1, AV_LOG_ERROR, "dimensions not set\n");
            return AVERROR(EINVAL);
        }
        break;
    case AV_CODEC_ID_BITPACKED:
    case AV_CODEC_ID_RAWVIDEO: {
        const FFRFC4175Format *rfc4175;
        if (st->codecpar->width <= 0 || st->codecpar->height <= 0) {
            av_log(s1, AV_LOG_ERROR, "dimensions not set\n");
            return AVERROR(EINVAL);
        }
        rfc4175 = ff_rfc4175_get_format(st->codecpar->codec_id,
                                        st->codecpar->format);
        if (!rfc4175) {
            av_log(s1, AV_LOG_ERROR,
                   "RFC 4175 RTP packetization does not support pixel "
                   "format %s under codec %s. The rfc4175 packetizer "
                   "expects pkt->data to already be in RFC 4175 wire "
                   "format; choose a supported (codec, pix_fmt) pair "
                   "from rtpenc_rfc4175.c (e.g. -c:v rawvideo "
                   "-pix_fmt uyvy422 / rgb24, or -c:v bitpacked "
                   "-pix_fmt yuv420p / yuv422p / yuv422p10 / yuv444p).\n",
                   av_get_pix_fmt_name(st->codecpar->format),
                   avcodec_get_name(st->codecpar->codec_id));
            return AVERROR(EINVAL);
        }
        /* ST 2110-20 §6.2.5: 4:2:0 shall be progressive only. The yinc=2
         * formats in the table are exactly the 4:2:0 family. */
        if (rfc4175->yinc > 1 &&
            st->codecpar->field_order != AV_FIELD_PROGRESSIVE) {
            av_log(s1, AV_LOG_ERROR,
                   "RFC 4175 / ST 2110-20: 4:2:0 sampling (%s) shall be "
                   "transmitted progressive-only; interlaced field_order "
                   "is not allowed (ST 2110-20 §6.2.5).\n",
                   av_get_pix_fmt_name(st->codecpar->format));
            return AVERROR(EINVAL);
        }
        break;
        }
    case AV_CODEC_ID_VP9:
        if (s1->strict_std_compliance > FF_COMPLIANCE_EXPERIMENTAL) {
            av_log(s, AV_LOG_ERROR,
                   "Packetizing VP9 is experimental and its specification is "
                   "still in draft state. "
                   "Please set -strict experimental in order to enable it.\n");
            ret = AVERROR_EXPERIMENTAL;
            goto fail;
        }
        break;
    case AV_CODEC_ID_AV1:
        if (s1->strict_std_compliance > FF_COMPLIANCE_EXPERIMENTAL) {
            av_log(s, AV_LOG_ERROR,
                   "Packetizing AV1 is experimental and its specification is "
                   "still in draft state. "
                   "Please set -strict experimental in order to enable it.\n");
            ret = AVERROR_EXPERIMENTAL;
            goto fail;
        }
        break;
    case AV_CODEC_ID_VORBIS:
    case AV_CODEC_ID_THEORA:
        s->max_frames_per_packet = 15;
        break;
    case AV_CODEC_ID_ADPCM_G722:
        /* Due to a historical error, the clock rate for G722 in RTP is
         * 8000, even if the sample rate is 16000. See RFC 3551. */
        avpriv_set_pts_info(st, 32, 1, 8000);
        break;
    case AV_CODEC_ID_OPUS:
        if (st->codecpar->ch_layout.nb_channels > 2) {
            av_log(s1, AV_LOG_ERROR, "Multistream opus not supported in RTP\n");
            goto fail;
        }
        /* The opus RTP RFC says that all opus streams should use 48000 Hz
         * as clock rate, since all opus sample rates can be expressed in
         * this clock rate, and sample rate changes on the fly are supported. */
        avpriv_set_pts_info(st, 32, 1, 48000);
        break;
    case AV_CODEC_ID_ILBC:
        if (st->codecpar->block_align != 38 && st->codecpar->block_align != 50) {
            av_log(s1, AV_LOG_ERROR, "Incorrect iLBC block size specified\n");
            goto fail;
        }
        s->max_frames_per_packet = s->max_payload_size / st->codecpar->block_align;
        break;
    case AV_CODEC_ID_AMR_NB:
    case AV_CODEC_ID_AMR_WB:
        s->max_frames_per_packet = 50;
        if (st->codecpar->codec_id == AV_CODEC_ID_AMR_NB)
            n = 31;
        else
            n = 61;
        /* max_header_toc_size + the largest AMR payload must fit */
        if (1 + s->max_frames_per_packet + n > s->max_payload_size) {
            av_log(s1, AV_LOG_ERROR, "RTP max payload size too small for AMR\n");
            goto fail;
        }
        if (st->codecpar->ch_layout.nb_channels != 1) {
            av_log(s1, AV_LOG_ERROR, "Only mono is supported\n");
            goto fail;
        }
        break;
    case AV_CODEC_ID_AAC:
        s->max_frames_per_packet = 50;
        break;
    default:
        break;
    }

    return 0;

fail:
    av_freep(&s->buf);
    return ret;
}

/* send an rtcp sender report packet */
static void rtcp_send_sr(AVFormatContext *s1, int64_t ntp_time, int bye)
{
    RTPMuxContext *s = s1->priv_data;
    uint32_t rtp_ts;

    av_log(s1, AV_LOG_TRACE, "RTCP: %02x %"PRIx64" %"PRIx32"\n", s->payload_type, ntp_time, s->timestamp);

    s->last_rtcp_ntp_time = ntp_time;
    rtp_ts = av_rescale_q(ntp_time - s->first_rtcp_ntp_time, (AVRational){1, 1000000},
                          s1->streams[0]->time_base) + s->base_timestamp;
    avio_w8(s1->pb, RTP_VERSION << 6);
    avio_w8(s1->pb, RTCP_SR);
    avio_wb16(s1->pb, 6); /* length in words - 1 */
    avio_wb32(s1->pb, s->ssrc);
    avio_wb32(s1->pb, ntp_time / 1000000);
    avio_wb32(s1->pb, ((ntp_time % 1000000) << 32) / 1000000);
    avio_wb32(s1->pb, rtp_ts);
    avio_wb32(s1->pb, s->packet_count);
    avio_wb32(s1->pb, s->octet_count);

    if (s->cname) {
        int len = FFMIN(strlen(s->cname), 255);
        avio_w8(s1->pb, (RTP_VERSION << 6) + 1);
        avio_w8(s1->pb, RTCP_SDES);
        avio_wb16(s1->pb, (7 + len + 3) / 4); /* length in words - 1 */

        avio_wb32(s1->pb, s->ssrc);
        avio_w8(s1->pb, 0x01); /* CNAME */
        avio_w8(s1->pb, len);
        avio_write(s1->pb, s->cname, len);
        avio_w8(s1->pb, 0); /* END */
        for (len = (7 + len) % 4; len % 4; len++)
            avio_w8(s1->pb, 0);
    }

    if (bye) {
        avio_w8(s1->pb, (RTP_VERSION << 6) | 1);
        avio_w8(s1->pb, RTCP_BYE);
        avio_wb16(s1->pb, 1); /* length in words - 1 */
        avio_wb32(s1->pb, s->ssrc);
    }

    avio_flush(s1->pb);
}

/* send an rtp packet. sequence number is incremented, but the caller
   must update the timestamp itself */
void ff_rtp_send_data(AVFormatContext *s1, const uint8_t *buf1, int len, int m)
{
    int extHdr = 0;
    int ext = 0;
    RTPMuxContext *s = s1->priv_data;
    int cipher_flags = 0;

    av_log(s1, AV_LOG_TRACE, "rtp_send_data size=%d\n", len);

    if (s->hdcp_cipher || s->privacy_cipher) {
        extHdr = 1;
    }

    /* build the RTP header */
    avio_w8(s1->pb, (RTP_VERSION << 6) | (extHdr << 4));
    avio_w8(s1->pb, (s->payload_type & 0x7f) | ((m & 0x01) << 7));
    avio_wb16(s1->pb, s->seq);
    avio_wb32(s1->pb, s->timestamp);
    avio_wb32(s1->pb, s->ssrc);

    if (s->hdcp_cipher || s->privacy_cipher) {

        if (s1->full_ctr) {

            struct RTP_ExtHdrFull hdr = {0};
            hdr.be = 0xBE;
            hdr.de = 0xDE;
            hdr.length_0 = 4;
            hdr.l = 0x0E;
            hdr.id = 1;

            hdr.input_ctr_7 = (s->encryption_input_ctr >> (7*8)) & 255;
            hdr.input_ctr_6 = (s->encryption_input_ctr >> (6*8)) & 255;
            hdr.input_ctr_5 = (s->encryption_input_ctr >> (5*8)) & 255;
            hdr.input_ctr_4 = (s->encryption_input_ctr >> (4*8)) & 255;
            hdr.input_ctr_3 = (s->encryption_input_ctr >> (3*8)) & 255;
            hdr.input_ctr_2 = (s->encryption_input_ctr >> (2*8)) & 255;
            hdr.input_ctr_1 = (s->encryption_input_ctr >> (1*8)) & 255;
            hdr.input_ctr_0 = (s->encryption_input_ctr >> (0*8)) & 255;

            // refresh the stream_ctr from external cipher
            if (s->hdcp_cipher) {
                if (s->hdcp_cipher->param != s->encryption_stream_ctr) {
                    s->encryption_stream_ctr = s->hdcp_cipher->param;
                }
            } else if (s->privacy_cipher) {
                if (s->privacy_cipher->param != s->encryption_stream_ctr) {
                    s->encryption_stream_ctr = s->privacy_cipher->param;
                }
            }

            // Used for both HDCP and PEP .. if both are active it is the HDCP stream-ctr
            hdr.inband_ctr_3 = (s->encryption_stream_ctr >> (3*8)) & 255;
            hdr.inband_ctr_2 = (s->encryption_stream_ctr >> (2*8)) & 255;
            hdr.inband_ctr_1 = (s->encryption_stream_ctr >> (1*8)) & 255;
            hdr.inband_ctr_0 = (s->encryption_stream_ctr >> (0*8)) & 255;

            cipher_flags |= AVCIPHERCB_VALIDPARAM | AVCIPHERCB_VALIDINPUTCTR;

            avio_write(s1->pb, (const uint8_t*)&hdr, sizeof(hdr));

            ext = sizeof(hdr);

            av_log(s1, AV_LOG_TRACE, "rtp_send_data HDCP/PEP header FULL streamCtr=%d, inputCtr=%ld\n", s->encryption_stream_ctr, s->encryption_input_ctr);

        } else {

            struct RTP_ExtHdrShort hdr = {0};

            hdr.be = 0xBE;
            hdr.de = 0xDE;
            hdr.length_0 = 1;
            hdr.l = 0x02;
            hdr.id = 2;

            hdr.input_ctr_2 = (s->encryption_input_ctr >> (2*8)) & 255;
            hdr.input_ctr_1 = (s->encryption_input_ctr >> (1*8)) & 255;
            hdr.input_ctr_0 = (s->encryption_input_ctr >> (0*8)) & 255;

            avio_write(s1->pb, (const uint8_t*)&hdr, sizeof(hdr));

            ext = sizeof(hdr);

            av_log(s1, AV_LOG_TRACE, "rtp_send_data HDCP/PEP header SHORT streamCtr=%d, inputCtr=%ld\n", s->encryption_stream_ctr, s->encryption_input_ctr);
        }

        av_log(s, AV_LOG_TRACE, "rtp_send_data EXTHDR size=%d, payload=%0x%0x%0x%0x\n", ext, buf1[0], buf1[1], buf1[2], buf1[3]);

    } else {

        av_log(s, AV_LOG_TRACE, "rtp_send_data EXTHDR size=%d, payload=%0x%0x%0x%0x\n", ext, buf1[0], buf1[1], buf1[2], buf1[3]);
    }

    #pragma GCC diagnostic ignored "-Wdiscarded-qualifiers"

    if (s->hdcp_cipher) {
        // !!! encrypting over the incoming data (TODO: check if required to do otherwise)
        s->hdcp_cipher->callback(s->hdcp_cipher->opaque, s->encryption_stream_ctr, s->encryption_input_ctr, buf1 + s1->payload_header_size, len - s1->payload_header_size, cipher_flags);
    }

    if (s->privacy_cipher) {
        int pep_flags = cipher_flags | AVCIPHERCB_ENCRYPT;
        s->privacy_cipher->callback(s->privacy_cipher->opaque, s->encryption_stream_ctr, s->encryption_input_ctr, buf1 + s1->payload_header_size, len - s1->payload_header_size, pep_flags);
    }

    {
        int encrypted_len = len - s1->payload_header_size + s1->privacy_mac_size;
        s->encryption_input_ctr += (encrypted_len & 15) ? (encrypted_len >> 4) + 1 : (encrypted_len >> 4); // once after both encryptions
    }

    avio_write(s1->pb, buf1, len + s1->privacy_mac_size);
    avio_flush(s1->pb);

    s->seq = (s->seq + 1) & 0xffff;
    s->octet_count += len;
    s->packet_count++;
}

/* send an integer number of samples and compute time stamp and fill
   the rtp send buffer before sending. */
static int rtp_send_samples(AVFormatContext *s1,
                            const uint8_t *buf1, int size, int sample_size_bits)
{
    RTPMuxContext *s = s1->priv_data;
    int len, max_packet_size, n;
    /* Calculate the number of bytes to get samples aligned on a byte border */
    int aligned_samples_size = sample_size_bits/av_gcd(sample_size_bits, 8);

    max_packet_size = (s->max_payload_size / aligned_samples_size) * aligned_samples_size;
    /* Not needed, but who knows. Don't check if samples aren't an even number of bytes. */
    if ((sample_size_bits % 8) == 0 && ((8 * size) % sample_size_bits) != 0)
        return AVERROR(EINVAL);
    n = 0;
    while (size > 0) {
        s->buf_ptr = s->buf;
        len = FFMIN(max_packet_size, size);

        /* copy data */
        memcpy(s->buf_ptr, buf1, len);
        s->buf_ptr += len;
        buf1 += len;
        size -= len;
        s->timestamp = s->cur_timestamp + n * 8 / sample_size_bits;
        s1->payload_header_size = 0;
        s1->full_ctr = (n == 0);
        ff_rtp_send_data(s1, s->buf, s->buf_ptr - s->buf, 0);
        n += (s->buf_ptr - s->buf);
    }
    return 0;
}

static void rtp_send_mpegaudio(AVFormatContext *s1,
                               const uint8_t *buf1, int size)
{
    RTPMuxContext *s = s1->priv_data;
    int len, count, max_packet_size;

    max_packet_size = s->max_payload_size;

    /* test if we must flush because not enough space */
    len = (s->buf_ptr - s->buf);
    if ((len + size) > max_packet_size) {
        if (len > 4) {
            ff_rtp_send_data(s1, s->buf, s->buf_ptr - s->buf, 0);
            s->buf_ptr = s->buf + 4;
        }
    }
    if (s->buf_ptr == s->buf + 4) {
        s->timestamp = s->cur_timestamp;
    }

    /* add the packet */
    if (size > max_packet_size) {
        /* big packet: fragment */
        count = 0;
        while (size > 0) {
            len = max_packet_size - 4;
            if (len > size)
                len = size;
            /* build fragmented packet */
            s->buf[0] = 0;
            s->buf[1] = 0;
            s->buf[2] = count >> 8;
            s->buf[3] = count;
            memcpy(s->buf + 4, buf1, len);
            ff_rtp_send_data(s1, s->buf, len + 4, 0);
            size -= len;
            buf1 += len;
            count += len;
        }
    } else {
        if (s->buf_ptr == s->buf + 4) {
            /* no fragmentation possible */
            s->buf[0] = 0;
            s->buf[1] = 0;
            s->buf[2] = 0;
            s->buf[3] = 0;
        }
        memcpy(s->buf_ptr, buf1, size);
        s->buf_ptr += size;
    }
}

static void rtp_send_raw(AVFormatContext *s1,
                         const uint8_t *buf1, int size)
{
    RTPMuxContext *s = s1->priv_data;
    int len, max_packet_size;

    max_packet_size = s->max_payload_size;

    while (size > 0) {
        len = FFMIN(max_packet_size, size);

        /* Copy into s->buf so the CMAC callback has headroom for the
         * MAC tag without corrupting the next chunk in the source buffer. */
        memcpy(s->buf, buf1, len);

        s->timestamp = s->cur_timestamp;
        s1->payload_header_size = 0;
        s1->full_ctr = 1;
        ff_rtp_send_data(s1, s->buf, len, (len == size));

        buf1 += len;
        size -= len;
    }
}

/* NOTE: size is assumed to be an integer multiple of TS_PACKET_SIZE */
static void rtp_send_mpegts_raw(AVFormatContext *s1,
                                const uint8_t *buf1, int size)
{
    RTPMuxContext *s = s1->priv_data;
    int len, out_len;

    s->timestamp = s->cur_timestamp;
    while (size >= TS_PACKET_SIZE) {
        len = s->max_payload_size - (s->buf_ptr - s->buf);
        if (len > size)
            len = size;
        memcpy(s->buf_ptr, buf1, len);
        buf1 += len;
        size -= len;
        s->buf_ptr += len;

        out_len = s->buf_ptr - s->buf;
        if (out_len >= s->max_payload_size) {
            s1->payload_header_size = 0;
            s1->full_ctr = 1;
            ff_rtp_send_data(s1, s->buf, out_len, 0);
            s->buf_ptr = s->buf;
        }
    }
}

static int rtp_send_ilbc(AVFormatContext *s1, const uint8_t *buf, int size)
{
    RTPMuxContext *s = s1->priv_data;
    AVStream *st = s1->streams[0];
    int frame_duration = av_get_audio_frame_duration2(st->codecpar, 0);
    int frame_size = st->codecpar->block_align;
    int frames = size / frame_size;

    while (frames > 0) {
        if (s->num_frames > 0 &&
            av_compare_ts(s->cur_timestamp - s->timestamp, st->time_base,
                          s1->max_delay, AV_TIME_BASE_Q) >= 0) {
            ff_rtp_send_data(s1, s->buf, s->buf_ptr - s->buf, 1);
            s->num_frames = 0;
        }

        if (!s->num_frames) {
            s->buf_ptr = s->buf;
            s->timestamp = s->cur_timestamp;
        }
        memcpy(s->buf_ptr, buf, frame_size);
        frames--;
        s->num_frames++;
        s->buf_ptr       += frame_size;
        buf              += frame_size;
        s->cur_timestamp += frame_duration;

        if (s->num_frames == s->max_frames_per_packet) {
            ff_rtp_send_data(s1, s->buf, s->buf_ptr - s->buf, 1);
            s->num_frames = 0;
        }
    }
    return 0;
}

static int rtp_write_packet(AVFormatContext *s1, AVPacket *pkt)
{
    RTPMuxContext *s = s1->priv_data;
    AVStream *st = s1->streams[0];
    int rtcp_bytes;
    int size= pkt->size;

    av_log(s1, AV_LOG_TRACE, "%d: write len=%d\n", pkt->stream_index, size);

    rtcp_bytes = ((s->octet_count - s->last_octet_count) * RTCP_TX_RATIO_NUM) /
        RTCP_TX_RATIO_DEN;
    if ((s->first_packet || ((rtcp_bytes >= RTCP_SR_SIZE) &&
                            (ff_ntp_time() - s->last_rtcp_ntp_time > 5000000))) &&
        !(s->flags & FF_RTP_FLAG_SKIP_RTCP)) {
        rtcp_send_sr(s1, ff_ntp_time(), 0);
        s->last_octet_count = s->octet_count;
        s->first_packet = 0;
    }
    s->cur_timestamp = s->base_timestamp + pkt->pts;

    switch(st->codecpar->codec_id) {
    case AV_CODEC_ID_PCM_MULAW:
    case AV_CODEC_ID_PCM_ALAW:
    case AV_CODEC_ID_PCM_U8:
    case AV_CODEC_ID_PCM_S8:
        return rtp_send_samples(s1, pkt->data, size, 8 * st->codecpar->ch_layout.nb_channels);
    case AV_CODEC_ID_PCM_U16BE:
    case AV_CODEC_ID_PCM_U16LE:
    case AV_CODEC_ID_PCM_S16BE:
    case AV_CODEC_ID_PCM_S16LE:
        return rtp_send_samples(s1, pkt->data, size, 16 * st->codecpar->ch_layout.nb_channels);
    case AV_CODEC_ID_PCM_S24BE:
        return rtp_send_samples(s1, pkt->data, size, 24 * st->codecpar->ch_layout.nb_channels);
    case AV_CODEC_ID_AM824:
        ff_rtp_send_am824(s1, pkt->data, size);
        break;
    case AV_CODEC_ID_ADPCM_G722:
        /* The actual sample size is half a byte per sample, but since the
         * stream clock rate is 8000 Hz while the sample rate is 16000 Hz,
         * the correct parameter for send_samples_bits is 8 bits per stream
         * clock. */
        return rtp_send_samples(s1, pkt->data, size, 8 * st->codecpar->ch_layout.nb_channels);
    case AV_CODEC_ID_ADPCM_G726:
    case AV_CODEC_ID_ADPCM_G726LE:
        return rtp_send_samples(s1, pkt->data, size,
                                st->codecpar->bits_per_coded_sample * st->codecpar->ch_layout.nb_channels);
    case AV_CODEC_ID_MP2:
    case AV_CODEC_ID_MP3:
        rtp_send_mpegaudio(s1, pkt->data, size);
        break;
    case AV_CODEC_ID_MPEG1VIDEO:
    case AV_CODEC_ID_MPEG2VIDEO:
        ff_rtp_send_mpegvideo(s1, pkt->data, size);
        break;
    case AV_CODEC_ID_AAC:
        if (s->flags & FF_RTP_FLAG_MP4A_LATM)
            ff_rtp_send_latm(s1, pkt->data, size);
        else
            ff_rtp_send_aac(s1, pkt->data, size);
        break;
    case AV_CODEC_ID_AMR_NB:
    case AV_CODEC_ID_AMR_WB:
        ff_rtp_send_amr(s1, pkt->data, size);
        break;
    case AV_CODEC_ID_AV1:
        ff_rtp_send_av1(s1, pkt->data, size, (pkt->flags & AV_PKT_FLAG_KEY) ? 1 : 0);
        break;
    case AV_CODEC_ID_MPEG2TS:
        rtp_send_mpegts_raw(s1, pkt->data, size);
        break;
    case AV_CODEC_ID_DIRAC:
        ff_rtp_send_vc2hq(s1, pkt->data, size, st->codecpar->field_order != AV_FIELD_PROGRESSIVE ? 1 : 0);
        break;
    case AV_CODEC_ID_H264:
        ff_rtp_send_h264_hevc(s1, pkt->data, size);
        break;
    case AV_CODEC_ID_H261:
        ff_rtp_send_h261(s1, pkt->data, size);
        break;
    case AV_CODEC_ID_H263:
        if (s->flags & FF_RTP_FLAG_RFC2190) {
            size_t mb_info_size;
            const uint8_t *mb_info =
                av_packet_get_side_data(pkt, AV_PKT_DATA_H263_MB_INFO,
                                        &mb_info_size);
            ff_rtp_send_h263_rfc2190(s1, pkt->data, size, mb_info, mb_info_size);
            break;
        }
        /* Fallthrough */
    case AV_CODEC_ID_H263P:
        ff_rtp_send_h263(s1, pkt->data, size);
        break;
    case AV_CODEC_ID_HEVC:
        ff_rtp_send_h264_hevc(s1, pkt->data, size);
        break;
    case AV_CODEC_ID_VORBIS:
    case AV_CODEC_ID_THEORA:
        ff_rtp_send_xiph(s1, pkt->data, size);
        break;
    case AV_CODEC_ID_VP8:
        ff_rtp_send_vp8(s1, pkt->data, size);
        break;
    case AV_CODEC_ID_VP9:
        ff_rtp_send_vp9(s1, pkt->data, size);
        break;
    case AV_CODEC_ID_ILBC:
        rtp_send_ilbc(s1, pkt->data, size);
        break;
    case AV_CODEC_ID_MJPEG:
        ff_rtp_send_jpeg(s1, pkt->data, size);
        break;
    case AV_CODEC_ID_BITPACKED:
    case AV_CODEC_ID_RAWVIDEO: {
        int interlaced = st->codecpar->field_order != AV_FIELD_PROGRESSIVE;

        ff_rtp_send_raw_rfc4175(s1, pkt->data, size, interlaced, 0);
        if (interlaced)
            ff_rtp_send_raw_rfc4175(s1, pkt->data, size, interlaced, 1);
        break;
        }
    case AV_CODEC_ID_OPUS:
        if (size > s->max_payload_size) {
            av_log(s1, AV_LOG_ERROR,
                   "Packet size %d too large for max RTP payload size %d\n",
                   size, s->max_payload_size);
            return AVERROR(EINVAL);
        }
        /* Intentional fallthrough */
    default:
        /* better than nothing : send the codec raw data */
        rtp_send_raw(s1, pkt->data, size);
        break;
    }
    return 0;
}

static int rtp_write_trailer(AVFormatContext *s1)
{
    RTPMuxContext *s = s1->priv_data;

    /* If the caller closes and recreates ->pb, this might actually
     * be NULL here even if it was successfully allocated at the start. */
    if (s1->pb && (s->flags & FF_RTP_FLAG_SEND_BYE))
        rtcp_send_sr(s1, ff_ntp_time(), 1);

    return 0;
}

static void rtp_deinit(AVFormatContext *s1)
{
    RTPMuxContext *s = s1->priv_data;

    av_freep(&s->buf);
}

const FFOutputFormat ff_rtp_muxer = {
    .p.name            = "rtp",
    .p.long_name       = NULL_IF_CONFIG_SMALL("RTP output"),
    .priv_data_size    = sizeof(RTPMuxContext),
    .p.audio_codec     = AV_CODEC_ID_PCM_MULAW,
    .p.video_codec     = AV_CODEC_ID_MPEG4,
    .write_header      = rtp_write_header,
    .write_packet      = rtp_write_packet,
    .write_trailer     = rtp_write_trailer,
    .deinit            = rtp_deinit,
    .p.priv_class      = &rtp_muxer_class,
    .p.flags           = AVFMT_NODIMENSIONS | AVFMT_TS_NONSTRICT,
};

