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

// default buffer size in frames, used when the consumer does not request a
// size of its own via B_MULTI_GET_BUFFERS
const uint32 kSamplesBufferSize = 2048;
// Bounds for a consumer-requested buffer geometry (the request arrives from
// userland and is untrusted). The packet bound stays below the host
// controller's per-transfer packet limit (512 on XHCI) with room to spare,
// since rate feedback re-splits a buffer into more, smaller packets when the
// device runs slow.
const uint32 kMaxRequestFrames = 16384;
const uint32 kMaxPacketsPerBuffer = 384;
// Upper bound for the buffer counts a B_MULTI_GET_BUFFERS request may carry;
// they size on-stack arrays in the ioctl handler (the multi_audio add-on
// requests 32).
const int32 kMaxRequestBuffers = 32;
// Default and maximum number of sub-buffers cycled between the driver and the
// media server (B_MULTI_GET_BUFFERS may request fewer). Unlike a
// PCI DMA card -- whose hardware loops over its buffers and hands each one back at
// a buffer boundary -- this driver emulates the ring in software: an isochronous
// completion re-queues its buffer immediately, so with only two buffers the media
// server has no headroom and must refill a buffer that is already back in the USB
// ring, racing the controller's DMA. That window shrinks with the sample rate and
// causes rate-dependent dropouts. Extra buffers give the media server a buffer to
// fill ahead while the others play. (The media kit accepts 2..several here.)
// Eight buffers also ride out multi-buffer stalls of the host controller's
// completion thread under system load, at the cost of added latency. Together
// with the one-transfer startup warmup this must fit the host controller's
// queued-transfers budget per endpoint (XHCI_MAX_TRANSFERS - 1).
const uint32 kSamplesBufferCount = 8;


extern usb_module_info* gUSBModule;

extern "C" status_t usb_audio_device_added(usb_device device, void** cookie);
extern "C" status_t usb_audio_device_removed(void* cookie);

extern "C" status_t init_hardware();
extern "C" void uninit_driver();

extern "C" const char** publish_devices();
extern "C" device_hooks *find_device(const char* name);


#endif // _USB_AUDIO_DRIVER_H_

