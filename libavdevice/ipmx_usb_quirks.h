/*
 * IPMX USB device quirk tables
 * Factual VID:PID → quirk-flag mappings for UVC webcams and UAC audio devices.
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

#ifndef AVDEVICE_IPMX_USB_QUIRKS_H
#define AVDEVICE_IPMX_USB_QUIRKS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Quirks marked as "deferred" or "no-op" are in the table for lookup but have
 * no behavioral implementation in receiver/local (capture-only scope or
 * require additional VC/VS logic). */
enum IpmxUvcQuirk {
    IPMX_UVC_QUIRK_NONE                  = 0,
    IPMX_UVC_QUIRK_STATUS_INTERVAL       = 0x0001,  /* deferred */
    IPMX_UVC_QUIRK_STREAM_NO_FID         = 0x0002,
    IPMX_UVC_QUIRK_IGNORE_SELECTOR_UNIT  = 0x0004,  /* deferred */
    IPMX_UVC_QUIRK_FIX_BANDWIDTH         = 0x0008,
    IPMX_UVC_QUIRK_PROBE_MINMAX          = 0x0010,
    IPMX_UVC_QUIRK_PROBE_DEF             = 0x0020,
    IPMX_UVC_QUIRK_RESTRICT_FRAME_RATE   = 0x0040,
    IPMX_UVC_QUIRK_RESTORE_CTRLS_ON_INIT = 0x0080,  /* deferred */
    IPMX_UVC_QUIRK_FORCE_Y8              = 0x0100,  /* deferred */
    IPMX_UVC_QUIRK_MJPEG_NO_EOF          = 0x0200,
    IPMX_UVC_QUIRK_WAKE_AUTOSUSPEND      = 0x0400,  /* deferred */
};

/* PLAYBACK_FIRST is not implemented: capture-only (webcam/mic) scope. */
enum IpmxUacQuirk {
    IPMX_UAC_QUIRK_NONE                = 0,
    IPMX_UAC_QUIRK_RATE_FIX            = 0x0001,
    IPMX_UAC_QUIRK_ALIGN_TRANSFER      = 0x0002,
    IPMX_UAC_QUIRK_IGNORE_CTL_ERROR    = 0x0004,
    IPMX_UAC_QUIRK_PLAYBACK_FIRST      = 0x0008,  /* no-op: capture-only */
    IPMX_UAC_QUIRK_SKIP_CLOCK_SELECTOR = 0x0010,
};

/**
 * Look up UVC quirk flags for a given VID:PID.
 * Returns 0 (IPMX_UVC_QUIRK_NONE) if not in the table.
 */
uint32_t ipmx_uvc_quirks_lookup(uint16_t vid, uint16_t pid);

/**
 * Look up UAC quirk flags for a given VID:PID.
 * Returns 0 (IPMX_UAC_QUIRK_NONE) if not in the table.
 */
uint32_t ipmx_uac_quirks_lookup(uint16_t vid, uint16_t pid);

#ifdef __cplusplus
}
#endif

#endif /* AVDEVICE_IPMX_USB_QUIRKS_H */
