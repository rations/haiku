/*
 *	Driver for USB Audio Device Class devices.
 *	Copyright (c) 2009-13 S.Zharski <imker@gmx.li>
 *	Distributed under the terms of the MIT license.
 *
 */
#ifndef _USB_AUDIO_STREAM_H_
#define _USB_AUDIO_STREAM_H_


#include "AudioStreamingInterface.h"


class Device;

class Stream : public AudioStreamingInterface {
	friend	class			Device;
public:
							Stream(Device* device, size_t interface,
								usb_interface_list* List);
							~Stream();

			status_t		Init();
			status_t		InitCheck() { return fStatus; }

			status_t		Start();
			status_t		Stop();
			bool			IsRunning() { return fIsRunning; }
			void			OnRemove();

			status_t		GetBuffers(multi_buffer_list* List);

			status_t		OnSetConfiguration(usb_device device,
							const usb_configuration_info* config);

			bool			ExchangeBuffer(multi_buffer_info* Info);
			status_t		GetEnabledChannels(uint32& offset,
								multi_channel_enable* Enable);
			status_t		SetEnabledChannels(uint32& offset,
								multi_channel_enable* Enable);
			status_t		GetGlobalFormat(multi_format_info* Format);
			status_t		SetGlobalFormat(multi_format_info* Format);

protected:
			Device*			fDevice;
			status_t		fStatus;

			usb_pipe		fStreamEndpoint;

			bool			fIsRunning;
			uint16			fMaxPacketSize;
			area_id			fArea, fKernelArea;
			size_t			fAreaSize;
			usb_iso_packet_descriptor* fDescriptors;
			size_t			fDescriptorsCount;
			uint8*			fBuffers;
			uint8*			fKernelBuffers;
			size_t			fCurrentBuffer;
			size_t			fSamplesCount;

			bigtime_t		fRealTime;
			uint32			fStartingFrame;
			int32			fProcessedBuffers;
			int32			fInsideNotify;

			// Asynchronous feedback for a UAC2 async playback endpoint. The
			// device reports its true sampling rate so we can resize the
			// outgoing packets to match and keep its FIFO from under/overrunning.
			// Two sources are supported: an explicit isochronous feedback IN
			// endpoint (16.16 samples per microframe), or -- when the device
			// tags its capture endpoint with the "implicit feedback" usage type
			// -- the capture stream's own measured delivery rate. Playback and
			// capture share one clock, so the latter is exact and is preferred
			// (many devices, e.g. the Behringer UMC2xxHD, advertise an explicit
			// feedback endpoint their firmware never actually services).
			static const uint32 kFeedbackPackets = 8;

			bool			fDataEndpointIsAsync;
			bool			fIsFeedbackSource;
			bool			fUseImplicitFeedback;
			bool			fUseExplicitFeedback;

			// Running totals used by a capture feedback source to derive the
			// device's true rate. Per-buffer frame counts are integers and so
			// quantize the rate too coarsely to see crystal drift; accumulating
			// across buffers recovers the fractional part. Halved periodically
			// so the average stays responsive and the counters stay bounded.
			uint64			fCaptureFramesTotal;
			uint64			fCapturePacketsTotal;

			usb_pipe		fFeedbackEndpoint;
			area_id			fFeedbackArea;
			uint8*			fFeedbackBuffer;
			usb_iso_packet_descriptor fFeedbackDescriptors[kFeedbackPackets];
			uint16			fFeedbackPacketSize;
			uint32			fFeedbackFrame;
			size_t			fPacketsPerBuffer;
			uint32			fNominalFreq;
			uint32			fMaxFreq;
			uint32			fMaxFrameSize;
			uint8			fDataInterval;
			int32			fCurrentFreq;
			uint32			fFeedbackPhase;
			int32			fFreqShift;

private:
			status_t		_ChooseAlternate();
			status_t		_SetupUAC2Rates();
			status_t		_SetupBuffers();
			status_t		_QueueNextTransfer(size_t buffer, bool start);
	static	void			_TransferCallback(void* cookie, status_t status,
								void* data, size_t actualLength);
			void			_InitFeedbackParams(uint32 rate);
			size_t			_FillPlaybackPackets(
								usb_iso_packet_descriptor* descriptors,
								size_t frames, uint32 stride);
			status_t		_QueueFeedback();
	static	void			_FeedbackCallback(void* cookie, status_t status,
								void* data, size_t actualLength);
			void			_ProcessFeedback(const uint8* data, size_t length);
			void			_PublishImplicitFeedback(size_t actualLength);
			void			_DumpDescriptors();
};


#endif // _USB_AUDIO_STREAM_H_

