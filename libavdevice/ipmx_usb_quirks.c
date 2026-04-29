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

#include "ipmx_usb_quirks.h"

typedef struct QuirkEntry {
    uint16_t vid;
    uint16_t pid;
    uint32_t quirks;
} QuirkEntry;

/* Binary search on sorted (vid, pid) table */
static uint32_t quirk_lookup(const QuirkEntry *table, int count,
                             uint16_t vid, uint16_t pid)
{
    int lo = 0, hi = count - 1;
    uint32_t key = ((uint32_t)vid << 16) | pid;

    while (lo <= hi) {
        int mid = (lo + hi) / 2;
        uint32_t mk = ((uint32_t)table[mid].vid << 16) | table[mid].pid;
        if (mk < key)
            lo = mid + 1;
        else if (mk > key)
            hi = mid - 1;
        else
            return table[mid].quirks;
    }
    return 0;
}

/* ---------------------------------------------------------------------------
 * UVC quirk table — sorted by (vid, pid)
 * --------------------------------------------------------------------------- */

static const QuirkEntry uvc_quirks[] = {
    { 0x0408, 0x3060, 0x0010 },  /* Quanta HP Webcam */
    { 0x041E, 0x4057, 0x0002 },  /* Creative Live! Cam Optia */
    { 0x041E, 0x405F, 0x0002 },  /* Creative Live! Cam Notebook Pro */
    { 0x045E, 0x00F8, 0x0010 },  /* Microsoft LifeCam NX-6000 */
    { 0x045E, 0x0723, 0x0010 },  /* Microsoft LifeCam VX-7000 */
    { 0x045E, 0x074A, 0x0010 },  /* Microsoft LifeCam Cinema (HD) */
    { 0x046D, 0x082D, 0x0080 },  /* Logitech HD Pro Webcam C920 */
    { 0x046D, 0x0892, 0x0080 },  /* Logitech OrbiCam */
    { 0x046D, 0x08C1, 0x0080 },  /* Logitech QuickCam Fusion */
    { 0x046D, 0x08C2, 0x0080 },  /* Logitech QuickCam Orbit MP */
    { 0x046D, 0x08C3, 0x0080 },  /* Logitech QuickCam Pro Notebooks */
    { 0x046D, 0x08C5, 0x0080 },  /* Logitech QuickCam Pro 5000 */
    { 0x046D, 0x08C6, 0x0080 },  /* Logitech QuickCam OEM Dell */
    { 0x046D, 0x08C7, 0x0080 },  /* Logitech QuickCam OEM Cisco */
    { 0x046D, 0x0994, 0x0080 },  /* Logitech QuickCam Orbit/Sphere AF */
    { 0x04F2, 0xB071, 0x0040 },  /* Chicony CNF7129 */
    { 0x04F2, 0xB1BB, 0x0040 },  /* Chicony Integrated Camera */
    { 0x04F2, 0xB5AB, 0x0040 },  /* Lenovo EasyCamera */
    { 0x058F, 0x3820, 0x0010 },  /* Alcor AU3820 USB camera */
    { 0x05A9, 0x2640, 0x0010 },  /* OmniVision OV2640 */
    { 0x05A9, 0x7670, 0x0010 },  /* OmniVision OV7670 */
    { 0x05AC, 0x8501, 0x0030 },  /* Apple Built-in iSight */
    { 0x05CA, 0x1810, 0x0008 },  /* Ricoh USB2.0 Camera */
    { 0x05E3, 0x0505, 0x0002 },  /* Genesys Logic USB2.0 Camera */
    { 0x064E, 0xA219, 0x0010 },  /* Suyin HP TrueVision HD */
    { 0x0AC8, 0x3420, 0x0008 },  /* Z-Star Venus USB2.0 Camera */
    { 0x0BDA, 0x5846, 0x0010 },  /* Realtek RTS5846 */
    { 0x0C45, 0x62C0, 0x0018 },  /* Microdia Sonix USB 2.0 */
    { 0x0C45, 0x6340, 0x0010 },  /* Sonix USB 2.0 Camera */
    { 0x0E8D, 0x0004, 0x0010 },  /* MediaTek Ven_AVerMedia */
    { 0x13D3, 0x5103, 0x0002 },  /* IMC USB2.0 UVC HD Webcam */
    { 0x174F, 0x5212, 0x0002 },  /* Syntek STK1135 */
    { 0x174F, 0x5931, 0x0002 },  /* Syntek STK1160 */
    { 0x18EC, 0x3288, 0x0010 },  /* Arkmicro USB2.0 PC CAMERA */
    { 0x18EC, 0x3290, 0x0010 },  /* Arkmicro USB2.0 PC CAMERA */
    { 0x1BCF, 0x0C01, 0x0018 },  /* SiGma NB USB2.0 PC Camera */
    { 0x1BCF, 0x2883, 0x0002 },  /* Sunplus Integrated Camera */
    { 0x413C, 0x2003, 0x0010 },  /* Dell Integrated Webcam */
    { 0x5986, 0x0100, 0x0010 },  /* Bison BisonCam NB Pro */
    { 0x5986, 0x0101, 0x0010 },  /* Bison BisonCam NB Pro */
    { 0x5986, 0x0241, 0x0010 },  /* Acer Integrated Webcam */
    { 0x5986, 0x0652, 0x0010 },  /* Acer Integrated Webcam HD */
    { 0x8086, 0x0AD3, 0x0080 },  /* Intel RealSense D435 */
    { 0x8086, 0x0B07, 0x0080 },  /* Intel RealSense D435i */
    { 0xEB1A, 0x2710, 0x0202 },  /* eMPIA EM2710 */
};

#define UVC_QUIRKS_COUNT (sizeof(uvc_quirks) / sizeof(uvc_quirks[0]))

/* ---------------------------------------------------------------------------
 * UAC quirk table — sorted by (vid, pid)
 * --------------------------------------------------------------------------- */

static const QuirkEntry uac_quirks[] = {
    { 0x046D, 0x082D, 0x0004 },  /* Logitech C920 built-in mic */
    { 0x046D, 0x0A0E, 0x0004 },  /* Logitech USB Headset H340 */
    { 0x046D, 0x0A45, 0x0001 },  /* Logitech USB Headset H390 */
    { 0x047F, 0x0CA1, 0x0004 },  /* Plantronics GameCom 780 */
    { 0x047F, 0xC010, 0x0004 },  /* Plantronics .Audio 628 */
    { 0x08BB, 0x2902, 0x0002 },  /* TI PCM2902 Audio Codec */
    { 0x0951, 0x16A4, 0x0005 },  /* Kingston HyperX Cloud II */
    { 0x0B0E, 0x0305, 0x0004 },  /* Jabra SPEAK 410 */
    { 0x0B0E, 0x0412, 0x0004 },  /* Jabra PRO 930 */
    { 0x0BDA, 0x4014, 0x0001 },  /* Realtek USB Audio */
    { 0x0D8C, 0x0102, 0x0002 },  /* C-Media CM106 Like Sound */
    { 0x0D8C, 0x013C, 0x0002 },  /* C-Media CM108 Audio */
    { 0x1038, 0x12AD, 0x0004 },  /* SteelSeries Arctis 7 */
    { 0x1235, 0x8016, 0x0008 },  /* Focusrite Scarlett 2i2 */
    { 0x1235, 0x8200, 0x0008 },  /* Focusrite Scarlett 2i2 3rd Gen */
    { 0x1395, 0x0025, 0x0004 },  /* Sennheiser SC 60 */
    { 0x1397, 0x0508, 0x0003 },  /* Behringer UMC404HD */
    { 0x19F7, 0x0003, 0x0001 },  /* RODE NT-USB */
    { 0xB58E, 0x9E82, 0x0004 },  /* Blue Snowball */
    { 0xB58E, 0x9E84, 0x0004 },  /* Blue Yeti */
};

#define UAC_QUIRKS_COUNT (sizeof(uac_quirks) / sizeof(uac_quirks[0]))

uint32_t ipmx_uvc_quirks_lookup(uint16_t vid, uint16_t pid)
{
    return quirk_lookup(uvc_quirks, UVC_QUIRKS_COUNT, vid, pid);
}

uint32_t ipmx_uac_quirks_lookup(uint16_t vid, uint16_t pid)
{
    return quirk_lookup(uac_quirks, UAC_QUIRKS_COUNT, vid, pid);
}
