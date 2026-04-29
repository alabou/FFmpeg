/*
 * IPMX USB local device access via libusb (optional debug transport).
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

#ifdef CONFIG_LIBUSB

#ifndef AVDEVICE_IPMX_USB_LOCAL_H
#define AVDEVICE_IPMX_USB_LOCAL_H

#include <stdint.h>
#include <stddef.h>

typedef struct IpmxUsbLocalDevice IpmxUsbLocalDevice;

/* Open a USB device by VID:PID. Returns NULL on failure. */
IpmxUsbLocalDevice *ipmx_usb_local_open(uint16_t vid, uint16_t pid);

/* Close the device and free resources. */
void ipmx_usb_local_close(IpmxUsbLocalDevice *dev);

/* Get the full configuration descriptor. Returns bytes written or -1. */
int ipmx_usb_local_get_config_desc(IpmxUsbLocalDevice *dev,
                                    uint8_t *buf, int buf_size);

/* Get a string descriptor by index. Returns bytes written or -1. */
int ipmx_usb_local_get_string_desc(IpmxUsbLocalDevice *dev,
                                    uint8_t index, uint16_t langid,
                                    uint8_t *buf, int buf_size);

/* Perform a control transfer. Returns bytes transferred or -1. */
int ipmx_usb_local_control_transfer(IpmxUsbLocalDevice *dev,
                                     uint8_t bmRequestType, uint8_t bRequest,
                                     uint16_t wValue, uint16_t wIndex,
                                     uint8_t *data, uint16_t wLength,
                                     unsigned int timeout_ms);

/* Claim an interface. Returns 0 on success. */
int ipmx_usb_local_claim_interface(IpmxUsbLocalDevice *dev, int interface);

/* Release an interface. Returns 0 on success. */
int ipmx_usb_local_release_interface(IpmxUsbLocalDevice *dev, int interface);

/* Set alternate setting. Returns 0 on success. */
int ipmx_usb_local_set_alt_setting(IpmxUsbLocalDevice *dev,
                                    int interface, int alt_setting);

/* Submit an isochronous transfer and wait for completion.
   Returns total bytes received across all packets, or -1. */
int ipmx_usb_local_iso_transfer(IpmxUsbLocalDevice *dev,
                                 uint8_t endpoint, int num_packets,
                                 int packet_size, uint8_t *buf, int buf_size,
                                 unsigned int timeout_ms);

#endif /* AVDEVICE_IPMX_USB_LOCAL_H */

#endif /* CONFIG_LIBUSB */
