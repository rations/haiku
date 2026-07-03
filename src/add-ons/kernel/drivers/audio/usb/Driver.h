/*
 *	Driver for USB Audio Device Class devices.
 *	Copyright (c) 2009-13 S.Zharski <imker@gmx.li>
 *	Distributed under the terms of the MIT license.
 *
 */
#ifndef _USB_AUDIO_DRIVER_H_
#define _USB_AUDIO_DRIVER_H_


#include <Drivers.h>
#include <USB3.h>


#define DRIVER_NAME	"usb_audio"
#define MAX_DEVICES	8

const char* const kVersion = "ver.0.0.5";

// initial buffer size in samples
const uint32 kSamplesBufferSize = 2048;
// Number of sub-buffers cycled between the driver and the media server. Unlike a
// PCI DMA card -- whose hardware loops over its buffers and hands each one back at
// a buffer boundary -- this driver emulates the ring in software: an isochronous
// completion re-queues its buffer immediately, so with only two buffers the media
// server has no headroom and must refill a buffer that is already back in the USB
// ring, racing the controller's DMA. That window shrinks with the sample rate and
// causes rate-dependent dropouts. Extra buffers give the media server a buffer to
// fill ahead while the others play. (The media kit accepts 2..several here.)
const uint32 kSamplesBufferCount = 4;


extern usb_module_info* gUSBModule;

extern "C" status_t usb_audio_device_added(usb_device device, void** cookie);
extern "C" status_t usb_audio_device_removed(void* cookie);

extern "C" status_t init_hardware();
extern "C" void uninit_driver();

extern "C" const char** publish_devices();
extern "C" device_hooks *find_device(const char* name);


#endif // _USB_AUDIO_DRIVER_H_

