/*
 *	Driver for USB Audio Device Class devices.
 *	Copyright (c) 2009-13 S.Zharski <imker@gmx.li>
 *	Distributed under the terms of the MIT license.
 */


#include "Stream.h"

#include <new>

#include <kernel.h>
#include <usb/USB_audio.h>

#include "Device.h"
#include "Driver.h"
#include "Settings.h"


// Sentinel for fFreqShift: the feedback format has not been auto-detected yet.
static const int32 kFreqShiftUnset = -2147483647 - 1;


Stream::Stream(Device* device, size_t interface, usb_interface_list* List)
	:
	AudioStreamingInterface(&device->AudioControl(), interface, List),
	fDevice(device),
	fStatus(B_NO_INIT),
	fStreamEndpoint(0),
	fIsRunning(false),
	fMaxPacketSize(0),
	fArea(-1),
	fKernelArea(-1),
	fAreaSize(0),
	fDescriptors(NULL),
	fDescriptorsCount(0),
	fCurrentBuffer(0),
	fSamplesCount(0),
	fRealTime(0),
	fStartingFrame(0),
	fProcessedBuffers(0),
	fInsideNotify(0),
	fDataEndpointIsAsync(false),
	fIsFeedbackSource(false),
	fUseImplicitFeedback(false),
	fUseExplicitFeedback(false),
	fCaptureFramesTotal(0),
	fCapturePacketsTotal(0),
	fFeedbackEndpoint(0),
	fFeedbackArea(-1),
	fFeedbackBuffer(NULL),
	fFeedbackPacketSize(0),
	fFeedbackFrame(0),
	fPacketsPerBuffer(0),
	fNominalFreq(0),
	fMaxFreq(0),
	fMaxFrameSize(0),
	fDataInterval(0),
	fCurrentFreq(0),
	fFeedbackPhase(0),
	fFreqShift(kFreqShiftUnset)
{
}


Stream::~Stream()
{
	delete_area(fArea);
	delete_area(fKernelArea);
	delete_area(fFeedbackArea);
	delete fDescriptors;
}


status_t
Stream::_ChooseAlternate()
{
	// lookup alternate with maximal (ch * 100 + resolution)
	uint16 maxChxRes = 0;
	for (int i = 0; i < fAlternates.Count(); i++) {
		if (fAlternates[i]->Interface() == 0) {
			TRACE(INF, "Ignore alternate %d - zero interface description.\n", i);
			continue;
		}

		if (fAlternates[i]->Format() == 0) {
			TRACE(INF, "Ignore alternate %d - zero format description.\n", i);
			continue;
		}

		if (fAlternates[i]->Format()->fFormatType
				!= USB_AUDIO_FORMAT_TYPE_I) {
			TRACE(ERR, "Ignore alternate %d - format type %#02x "
				"is not supported.\n", i, fAlternates[i]->Format()->fFormatType);
			continue;
		}

		ASInterfaceDescriptor* asInterface = fAlternates[i]->Interface();
		TypeIFormatDescriptor* format
			= static_cast<TypeIFormatDescriptor*>(fAlternates[i]->Format());

		if (asInterface->fIsR2) {
			// R2: supported formats are a bitmap; accept PCM / PCM8 / FLOAT.
			uint32 supported = USB_AUDIO_R2_FORMAT_PCM
				| USB_AUDIO_R2_FORMAT_PCM8 | USB_AUDIO_R2_FORMAT_IEEE_FLOAT;
			if ((asInterface->fBmFormats & supported) == 0) {
				TRACE(ERR, "Ignore alternate %d - formats %#08x are not "
					"supported.\n", i, asInterface->fBmFormats);
				continue;
			}

			if ((asInterface->fBmFormats & USB_AUDIO_R2_FORMAT_PCM) != 0) {
				switch (format->fBitResolution) {
					default:
					TRACE(ERR, "Ignore alternate %d - bit resolution %d "
						"is not supported.\n", i, format->fBitResolution);
						continue;
					case 8: case 16: case 18: case 20: case 24: case 32:
						break;
				}
			}
		} else {
			switch (asInterface->fFormatTag) {
				case USB_AUDIO_FORMAT_PCM:
				case USB_AUDIO_FORMAT_PCM8:
				case USB_AUDIO_FORMAT_IEEE_FLOAT:
			//	case USB_AUDIO_FORMAT_ALAW:
			//	case USB_AUDIO_FORMAT_MULAW:
					break;
				default:
					TRACE(ERR, "Ignore alternate %d - format %#04x is not "
						"supported.\n", i, asInterface->fFormatTag);
				continue;
			}

			if (asInterface->fFormatTag == USB_AUDIO_FORMAT_PCM) {
				switch(format->fBitResolution) {
					default:
					TRACE(ERR, "Ignore alternate %d - bit resolution %d "
						"is not supported.\n", i, format->fBitResolution);
						continue;
					case 8: case 16: case 18: case 20: case 24: case 32:
						break;
				}
			}
		}

		// R1 was only ever exercised with mono/stereo; R2 targets such as the
		// 4-in/4-out UMC204HD need more, bounded by the hmulti channel maximum.
		uint8 maxChannels = asInterface->fIsR2
			? AudioControlInterface::kChannels : 2;
		if (format->fNumChannels > maxChannels) {
			TRACE(ERR, "Ignore alternate %d - channel count %d "
				"is not supported.\n", i, format->fNumChannels);
			continue;
		}

		uint16 chxRes = format->fNumChannels * 100 + format->fBitResolution;
		if (chxRes > maxChxRes) {
			maxChxRes = chxRes;
			fActiveAlternate = i;
		}
	}

	if (maxChxRes <= 0) {
		TRACE(ERR, "No compatible alternate found. "
			"Stream initialization failed.\n");
		return B_NO_INIT;
	}

	const ASEndpointDescriptor* endpoint
		= fAlternates[fActiveAlternate]->Endpoint();
	fIsInput = (endpoint->fEndpointAddress & USB_ENDPOINT_ADDR_DIR_IN)
		== USB_ENDPOINT_ADDR_DIR_IN;

	if (fIsInput)
		fCurrentBuffer = (size_t)-1;

	TRACE(INF, "Alternate %d EP:%x selected for %s!\n",
		fActiveAlternate, endpoint->fEndpointAddress,
		fIsInput ? "recording" : "playback");

	return B_OK;
}


status_t
Stream::Init()
{
	fStatus = _ChooseAlternate();
	if (fStatus != B_OK)
		return fStatus;

	// R2 devices do not carry the sample-rate list in the format descriptor;
	// query it from the Clock Source that feeds this stream's terminal.
	if (fControlInterface->SpecReleaseNumber() >= 0x200)
		fStatus = _SetupUAC2Rates();

	return fStatus;
}


status_t
Stream::_SetupUAC2Rates()
{
	AudioStreamAlternate* alternate = fAlternates[fActiveAlternate];
	TypeIFormatDescriptor* format
		= static_cast<TypeIFormatDescriptor*>(alternate->Format());
	if (format == NULL)
		return B_NO_INIT;

	uint8 clockId = fControlInterface->ClockSourceIdForTerminal(TerminalLink());
	if (clockId == 0) {
		TRACE(ERR, "No clock source for terminal %d.\n", TerminalLink());
		return B_ERROR;
	}

	Vector<uint32> rates;
	status_t status = fControlInterface->GetSamplingRates(clockId, rates);
	if (status != B_OK)
		return status;

	if (rates.Count() <= 0) {
		TRACE(ERR, "Clock %d reported no sampling rates.\n", clockId);
		return B_ERROR;
	}

	// Cache the discrete rates on the active alternate so the existing rate
	// reporting / selection paths (which read the format descriptor) work
	// unchanged. A non-zero frequency type marks a discrete list.
	format->fSampleFrequencies.MakeEmpty();
	for (int i = 0; i < rates.Count(); i++)
		format->fSampleFrequencies.PushBack(rates[i]);
	format->fSampleFrequencyType = rates.Count();

	// Pick a sensible default (highest available) until the media kit sets one.
	alternate->SetSamplingRate(0);

	return B_OK;
}


void
Stream::OnRemove()
{
	// the transfer callback schedule traffic - so we must ensure that we are
	// not inside the callback anymore before returning, as we would otherwise
	// violate the promise not to use any of the pipes after returning from the
	// removed callback
	while (atomic_get(&fInsideNotify) != 0)
		snooze(100);

	if (fFeedbackEndpoint != 0)
		gUSBModule->cancel_queued_transfers(fFeedbackEndpoint);
	gUSBModule->cancel_queued_transfers(fStreamEndpoint);
}


status_t
Stream::_SetupBuffers()
{
	// allocate buffer for worst (maximal size) case
	TypeIFormatDescriptor* format = static_cast<TypeIFormatDescriptor*>(
		fAlternates[fActiveAlternate]->Format());

	uint32 samplingRate = fAlternates[fActiveAlternate]->GetSamplingRate();
	uint32 sampleSize = format->fNumChannels * format->fSubframeSize;

	// data size pro 1 ms USB 1 frame or 1/8 ms USB 2 microframe
	size_t packetSize = samplingRate * sampleSize
		/ (fDevice->fUSBVersion < 0x0200 ? 1000 : 8000);
	TRACE(INF, "packetSize:%ld\n", packetSize);

	if (packetSize == 0) {
		TRACE(ERR, "computed packet size is 0!");
		return B_BAD_VALUE;
	}

	// Restart the rate estimate whenever the format/rate changes.
	fCaptureFramesTotal = 0;
	fCapturePacketsTotal = 0;

	if (fArea != -1) {
		Stop();
		delete_area(fArea);
		delete_area(fKernelArea);
		delete fDescriptors;
		fDescriptors = NULL;
	}

	fAreaSize = sampleSize * kSamplesBufferSize * kSamplesBufferCount;
	TRACE(INF, "estimate fAreaSize:%d\n", fAreaSize);

	// round up to B_PAGE_SIZE and create area
	fAreaSize = (fAreaSize + (B_PAGE_SIZE - 1)) &~ (B_PAGE_SIZE - 1);
	TRACE(INF, "rounded up fAreaSize:%d\n", fAreaSize);

	fArea = create_area(fIsInput ? DRIVER_NAME "_record_area"
		: DRIVER_NAME "_playback_area", (void**)&fBuffers,
		B_ANY_ADDRESS, fAreaSize, B_NO_LOCK,
		B_READ_AREA | B_WRITE_AREA);
	if (fArea < 0) {
		TRACE(ERR, "Error of creating %#x - "
			"bytes size buffer area:%#010x\n", fAreaSize, fArea);
		fStatus = fArea;
		return fStatus;
	}

	// The kernel is not allowed to touch userspace areas, so we clone our
	// area into kernel space.
	fKernelArea = clone_area("usb_audio cloned area", (void**)&fKernelBuffers,
		B_ANY_ADDRESS, B_KERNEL_READ_AREA | B_KERNEL_WRITE_AREA, fArea);
	if (fKernelArea < 0) {
		fStatus = fKernelArea;
		return fStatus;
	}

	TRACE(INF, "Created area id:%d at addr:%#010x size:%#010lx\n",
		fArea, fDescriptors, fAreaSize);

	fDescriptorsCount = fAreaSize / packetSize;
	// we need same size sub-buffers. round it
	fDescriptorsCount = ROUNDDOWN(fDescriptorsCount, kSamplesBufferCount);

	// samples count - based on the nominal packet size; feedback only changes
	// how the samples are split into packets, not how many a buffer holds.
	fSamplesCount = fDescriptorsCount * packetSize / sampleSize;
	TRACE(INF, "samplesCount:%d\n", fSamplesCount);

	fPacketsPerBuffer = fDescriptorsCount / kSamplesBufferCount;

	// Implicit feedback (pacing playback from the capture stream's measured
	// rate) is disabled for now: on this USB stack the isochronous-IN
	// actual_length does not reliably reflect the device's true per-microframe
	// delivery, so the rate cannot be measured accurately yet. The detection
	// plumbing is kept for when it can be revisited.
	fUseImplicitFeedback = false;

	// Prefer the device's implicit feedback source (its capture stream, which
	// shares the sampling clock) over an explicit feedback endpoint, as USB
	// Audio 2.0 (section 5.12.4.2) intends. Some devices advertise an explicit
	// feedback endpoint their firmware never actually services (e.g. the
	// Behringer UMCxxHD): queuing it every service interval only floods the
	// host controller with transaction errors, which starves the shared
	// transfer-completion path and audibly glitches playback. Since implicit
	// rate measurement is not reliable here yet, when an implicit source is
	// present we run the playback endpoint at its nominal rate (no drift
	// correction) instead of polling a dead explicit endpoint.
	// TODO: recover drift correction from the implicit (capture) source once
	// per-microframe iso-IN lengths are trustworthy on this stack.
	fUseExplicitFeedback = fFeedbackEndpoint != 0
		&& !fDevice->HasImplicitFeedbackSource();

	bool useFeedback = fUseImplicitFeedback || fUseExplicitFeedback;
	if (useFeedback) {
		_InitFeedbackParams(samplingRate);

		// With feedback the packet size varies per microframe: when the
		// device asks for a lower rate more (smaller) packets are needed to
		// carry the same number of samples. freqm is clamped to >= 7/8 of
		// nominal, so twice the nominal packet count is always sufficient.
		fPacketsPerBuffer *= 2;

		// Only the explicit feedback endpoint needs a DMA buffer to receive
		// into; implicit feedback reads the rate straight from the device.
		if (fUseExplicitFeedback && fFeedbackArea < 0) {
			fFeedbackArea = create_area(DRIVER_NAME "_feedback_area",
				(void**)&fFeedbackBuffer, B_ANY_ADDRESS, B_PAGE_SIZE,
				B_CONTIGUOUS, B_KERNEL_READ_AREA | B_KERNEL_WRITE_AREA);
			if (fFeedbackArea < 0) {
				TRACE(ERR, "Error creating feedback area:%#010x\n",
					fFeedbackArea);
				fStatus = fFeedbackArea;
				return fStatus;
			}
		}
	}

	fDescriptorsCount = fPacketsPerBuffer * kSamplesBufferCount;
	fDescriptors = new(std::nothrow)
		usb_iso_packet_descriptor[fDescriptorsCount];
	if (fDescriptors == NULL) {
		TRACE(ERR, "Cannot allocate iso packet descriptors.\n");
		fStatus = B_NO_MEMORY;
		return fStatus;
	}
	TRACE(INF, "descriptorsCount:%d\n", fDescriptorsCount);

	// initialize descriptors array; feedback fills request_length per queue.
	for (size_t i = 0; i < fDescriptorsCount; i++) {
		fDescriptors[i].request_length = useFeedback ? 0 : packetSize;
		fDescriptors[i].actual_length = 0;
		fDescriptors[i].status = B_OK;
	}

	return fStatus;
}


status_t
Stream::OnSetConfiguration(usb_device device,
		const usb_configuration_info* config)
{
	if (config == NULL) {
		TRACE(ERR, "NULL configuration. Not set.\n");
		return B_ERROR;
	}

	usb_interface_info* interface
		= &config->interface[fInterface].alt[fActiveAlternate];
	if (interface == NULL) {
		TRACE(ERR, "NULL interface. Not set.\n");
		return B_ERROR;
	}

	status_t status = gUSBModule->set_alt_interface(device, interface);
	uint8 address = fAlternates[fActiveAlternate]->Endpoint()->fEndpointAddress;

	TRACE(INF, "set_alt_interface %x\n", status);

	fStreamEndpoint = 0;
	fFeedbackEndpoint = 0;

	fDataEndpointIsAsync = false;
	fIsFeedbackSource = false;

	for (size_t i = 0; i < interface->endpoint_count; i++) {
		const usb_endpoint_descriptor* descriptor
			= interface->endpoint[i].descr;

		if (address == descriptor->endpoint_address) {
			fStreamEndpoint = interface->endpoint[i].handle;
			fMaxPacketSize = descriptor->max_packet_size;
			// bInterval encodes the service interval as 2^(bInterval-1)
			// (micro)frames; the feedback math needs the exponent.
			fDataInterval = descriptor->interval > 0
				? descriptor->interval - 1 : 0;
			fDataEndpointIsAsync = (descriptor->attributes
				& USB_ENDPOINT_ATTR_SYNCHRONIZE_MASK)
					== USB_ENDPOINT_ATTR_ASYNCRONOUS;
			// A capture endpoint tagged with the implicit-feedback usage type
			// is the device's designated clock reference for its asynchronous
			// playback endpoint (USB Audio 2.0). Advertise it on the device so
			// the playback stream can pick it up.
			if (fIsInput && (descriptor->attributes
					& USB_ENDPOINT_ATTR_USAGE_MASK)
						== USB_ENDPOINT_ATTR_IMPLICIT_USAGE) {
				fIsFeedbackSource = true;
				fDevice->SetImplicitFeedbackSource();
			}
			TRACE(INF, "%s Stream Endpoint [address %#04x] handle is: %#010x.\n",
				fIsInput ? "Input" : "Output", address, fStreamEndpoint);
			continue;
		}

		// A separate isochronous IN endpoint carrying the feedback usage
		// type is the explicit feedback endpoint of an asynchronous playback
		// stream. Only high-speed devices use the 16.16 format handled here;
		// capture streams never have one.
		if (!fIsInput && fDevice->fUSBVersion >= 0x0200
			&& (descriptor->attributes & USB_ENDPOINT_ATTR_MASK)
				== USB_ENDPOINT_ATTR_ISOCHRONOUS
			&& (descriptor->attributes & USB_ENDPOINT_ATTR_USAGE_MASK)
				== USB_ENDPOINT_ATTR_FEEDBACK_USAGE
			&& (descriptor->endpoint_address & USB_ENDPOINT_ADDR_DIR_IN) != 0) {
			fFeedbackEndpoint = interface->endpoint[i].handle;
			fFeedbackPacketSize = descriptor->max_packet_size;
			if (fFeedbackPacketSize > B_PAGE_SIZE)
				fFeedbackPacketSize = B_PAGE_SIZE;
			TRACE(INF, "Feedback Endpoint [address %#04x] handle is: %#010x.\n",
				descriptor->endpoint_address, fFeedbackEndpoint);
		}
	}

	if (fStreamEndpoint == 0) {
		TRACE(INF, "%s Stream Endpoint [address %#04x] was not found.\n",
			fIsInput ? "Input" : "Output", address);
		return B_ERROR;
	}

	return B_OK;
}


status_t
Stream::Start()
{
	status_t result = B_BUSY;
	if (!fIsRunning) {
		// Nothing has been played or recorded yet for this run; clear any count
		// left over from a previous run (e.g. a sample-rate change stops and
		// restarts the stream) so it does not corrupt the media server's timing.
		atomic_set(&fProcessedBuffers, 0);

		// Mark running before queuing so completion callbacks re-queue.
		fIsRunning = true;

		// Start reading the feedback endpoint first so a rate estimate is
		// available as the data packets begin flowing.
		if (fUseExplicitFeedback && fFeedbackBuffer != NULL) {
			result = _QueueFeedback();
			if (result != B_OK) {
				fIsRunning = false;
				return result;
			}
		}

		for (size_t i = 0; i < kSamplesBufferCount; i++)
			result = _QueueNextTransfer(i, i == 0);
		fIsRunning = result == B_OK;
	}
	return result;
}


status_t
Stream::Stop()
{
	if (fIsRunning) {
		// wait until possible notification handling finished...
		while (atomic_get(&fInsideNotify) != 0)
			snooze(100);
		fIsRunning = false;
	}
	if (fFeedbackEndpoint != 0)
		gUSBModule->cancel_queued_transfers(fFeedbackEndpoint);
	gUSBModule->cancel_queued_transfers(fStreamEndpoint);

	return B_OK;
}


status_t
Stream::_QueueNextTransfer(size_t queuedBuffer, bool start)
{
	TypeIFormatDescriptor* format = static_cast<TypeIFormatDescriptor*>(
		fAlternates[fActiveAlternate]->Format());

	uint32 stride = format->fNumChannels * format->fSubframeSize;
	size_t frames = fSamplesCount / kSamplesBufferCount;
	size_t bufferSize = frames * stride;

	usb_iso_packet_descriptor* descriptors
		= fDescriptors + queuedBuffer * fPacketsPerBuffer;
	uint8* buffer = fKernelBuffers + bufferSize * queuedBuffer;

	// With feedback the per-packet sizes vary and are computed here; without
	// it the descriptors already carry the fixed nominal packet size.
	size_t packetsCount = (fUseImplicitFeedback || fUseExplicitFeedback)
		? _FillPlaybackPackets(descriptors, frames, stride)
		: fPacketsPerBuffer;

	TRACE(DTA, "buffers:%#010x[%#x]\ndescrs:%#010x[%#x]\n",
		buffer, bufferSize, descriptors, packetsCount);

	status_t status = gUSBModule->queue_isochronous(fStreamEndpoint,
		buffer, bufferSize, descriptors, packetsCount,
		&fStartingFrame, USB_ISO_ASAP,
		Stream::_TransferCallback, this);

	TRACE(DTA, "frame:%#010x\n", fStartingFrame);
	return status;
}


void
Stream::_InitFeedbackParams(uint32 rate)
{
	// Nominal feedback value for a high-speed endpoint: audio frames per
	// microframe (fs / 8000) in 16.16 fixed point, rounded to nearest
	// (USB 2.0 spec 5.12.4.2). freqm is later clamped around this value.
	fNominalFreq = (uint32)((((uint64)rate << 10) + 62) / 125);
	fMaxFreq = fNominalFreq + fNominalFreq / 2;
	fMaxFrameSize = ((fMaxFreq << fDataInterval) + 0xffff) >> 16;

	atomic_set(&fCurrentFreq, (int32)fNominalFreq);
	fFeedbackPhase = 0;
	fFreqShift = kFreqShiftUnset;
}


size_t
Stream::_FillPlaybackPackets(usb_iso_packet_descriptor* descriptors,
	size_t frames, uint32 stride)
{
	uint32 freqm;
	if (fUseImplicitFeedback) {
		// The capture stream publishes its measured rate; use the nominal rate
		// until the first buffer has been measured, and clamp to the same
		// plausible window the explicit feedback path accepts.
		int32 measured = fDevice->Feedback();
		freqm = measured > 0 ? (uint32)measured : fNominalFreq;
		if (freqm < fNominalFreq - fNominalFreq / 8)
			freqm = fNominalFreq - fNominalFreq / 8;
		if (freqm > fMaxFreq)
			freqm = fMaxFreq;
	} else
		freqm = (uint32)atomic_get(&fCurrentFreq);
	size_t emitted = 0;
	size_t count = 0;

	// Distribute exactly "frames" audio frames across whole microframe
	// packets, sized from the phase accumulator so the average matches the
	// rate the device requested via feedback. The fractional remainder is
	// carried in fFeedbackPhase between packets. fPacketsPerBuffer is sized
	// for the worst (slowest) case, so this can never overrun the array.
	while (emitted < frames && count < fPacketsPerBuffer) {
		fFeedbackPhase = (fFeedbackPhase & 0xffff) + (freqm << fDataInterval);
		uint32 packetFrames = fFeedbackPhase >> 16;
		if (packetFrames > fMaxFrameSize)
			packetFrames = fMaxFrameSize;
		if (packetFrames == 0)
			packetFrames = 1;
		if (packetFrames > frames - emitted)
			packetFrames = frames - emitted;

		descriptors[count].request_length = packetFrames * stride;
		descriptors[count].actual_length = 0;
		descriptors[count].status = B_OK;

		emitted += packetFrames;
		count++;
	}

	return count;
}


status_t
Stream::_QueueFeedback()
{
	for (uint32 i = 0; i < kFeedbackPackets; i++) {
		fFeedbackDescriptors[i].request_length = fFeedbackPacketSize;
		fFeedbackDescriptors[i].actual_length = 0;
		fFeedbackDescriptors[i].status = B_OK;
	}

	// Queue several packets at once so the isochronous IN ring stays
	// continuously scheduled at the endpoint's service interval, matching how
	// the data endpoints are driven.
	return gUSBModule->queue_isochronous(fFeedbackEndpoint,
		fFeedbackBuffer, fFeedbackPacketSize * kFeedbackPackets,
		fFeedbackDescriptors, kFeedbackPackets,
		&fFeedbackFrame, USB_ISO_ASAP, Stream::_FeedbackCallback, this);
}


void
Stream::_FeedbackCallback(void* cookie, status_t status, void* data,
	size_t actualLength)
{
	Stream* stream = (Stream*)cookie;

	atomic_add(&stream->fInsideNotify, 1);
	if (status == B_CANCELED || stream->fDevice->fRemoved
			|| !stream->fIsRunning) {
		atomic_add(&stream->fInsideNotify, -1);
		return;
	}

	// Apply the most recent valid feedback packet in this transfer.
	for (uint32 i = 0; i < kFeedbackPackets; i++) {
		if (stream->fFeedbackDescriptors[i].status == B_OK
				&& stream->fFeedbackDescriptors[i].actual_length >= 3) {
			stream->_ProcessFeedback(
				stream->fFeedbackBuffer + i * stream->fFeedbackPacketSize,
				stream->fFeedbackDescriptors[i].actual_length);
		}
	}

	stream->_QueueFeedback();

	atomic_add(&stream->fInsideNotify, -1);
}


void
Stream::_ProcessFeedback(const uint8* buffer, size_t length)
{
	uint32 value = buffer[0] | (buffer[1] << 8)
		| (buffer[2] << 16) | (buffer[3] << 24);

	// High-speed devices report a 4-byte 16.16 value, full-speed a 3-byte
	// 10.14 value; the remaining upper bits are reserved and masked off.
	if (length == 3)
		value &= 0x00ffffff;
	else
		value &= 0x0fffffff;

	if (value == 0)
		return;

	// Some devices place the value at the wrong bit position. Detect the
	// shift once by aligning the first sample near the nominal frequency,
	// assuming a deviation of no more than +50% / -25%.
	if (fFreqShift == kFreqShiftUnset) {
		int32 shift = 0;
		while (value < fNominalFreq - fNominalFreq / 4) {
			value <<= 1;
			shift++;
		}
		while (value > fNominalFreq + fNominalFreq / 2) {
			value >>= 1;
			shift--;
		}
		fFreqShift = shift;
	} else if (fFreqShift >= 0)
		value <<= fFreqShift;
	else
		value >>= -fFreqShift;

	// Only accept plausible values; otherwise re-detect the format next time.
	bool accepted = value >= fNominalFreq - fNominalFreq / 8
		&& value <= fMaxFreq;
	if (accepted)
		atomic_set(&fCurrentFreq, (int32)value);
	else
		fFreqShift = kFreqShiftUnset;
}


void
Stream::_PublishImplicitFeedback(size_t actualLength)
{
	// Convert the audio frames the device delivered over this buffer into an
	// average frames-per-(micro)frame value in 16.16 fixed point and hand it to
	// the playback stream. Both streams run off the device's single crystal, so
	// this is its true sampling rate. The buffer spanned fPacketsPerBuffer
	// service intervals (one packet each, capture uses fixed nominal packets).
	TypeIFormatDescriptor* format = static_cast<TypeIFormatDescriptor*>(
		fAlternates[fActiveAlternate]->Format());
	uint32 stride = format->fNumChannels * format->fSubframeSize;
	if (stride == 0 || fPacketsPerBuffer == 0)
		return;

	// Accumulate the frames delivered against the (micro)frames they spanned.
	// The long-run ratio is the device's true samples-per-(micro)frame; a
	// single buffer only resolves it to whole frames. Halve both totals once
	// they grow large so recent drift keeps ~equal weight and they never
	// overflow.
	fCaptureFramesTotal += actualLength / stride;
	fCapturePacketsTotal += fPacketsPerBuffer;
	if (fCapturePacketsTotal >= (1ULL << 22)) {
		fCaptureFramesTotal >>= 1;
		fCapturePacketsTotal >>= 1;
	}

	int32 framesPerPacket
		= (int32)((fCaptureFramesTotal << 16) / fCapturePacketsTotal);
	fDevice->PublishFeedback(framesPerPacket);
}


void
Stream::_TransferCallback(void* cookie, status_t status, void* data,
	size_t actualLength)
{
	Stream* stream = (Stream*)cookie;

	TRACE(status == B_OK ? DTA : ERR,
		"stream:%010x: status:%#010x, data:%#010x, len:%d\n",
		stream->fStreamEndpoint, status, data, actualLength);

	atomic_add(&stream->fInsideNotify, 1);
	if (status == B_CANCELED || stream->fDevice->fRemoved || !stream->fIsRunning) {
		TRACE(ERR, "Cancelled: c:%p st:%#010x, data:%#010x, len:%d\n",
			cookie, status, data, actualLength);
		atomic_add(&stream->fInsideNotify, -1);
		return;
	}

#if 0
	stream->_DumpDescriptors();
#endif

	// A capture stream tagged as the implicit-feedback source paces the
	// asynchronous playback stream from its own delivery rate.
	if (stream->fIsFeedbackSource)
		stream->_PublishImplicitFeedback(actualLength);

	if (atomic_add(&stream->fProcessedBuffers, 1) > (int32)kSamplesBufferCount)
		TRACE(ERR, "Processed buffers overflow:%d\n", stream->fProcessedBuffers);
	stream->fRealTime = system_time();

	release_sem_etc(stream->fDevice->fBuffersReadySem, 1, B_DO_NOT_RESCHEDULE);

	stream->fCurrentBuffer = (stream->fCurrentBuffer + 1) % kSamplesBufferCount;
	status = stream->_QueueNextTransfer(stream->fCurrentBuffer, false);

	atomic_add(&stream->fInsideNotify, -1);
}


void
Stream::_DumpDescriptors()
{
	for (size_t i = 0; i < fDescriptorsCount; i++)
		TRACE(ISO, "%d:req_len:%d; act_len:%d; stat:%#010x\n", i,
			fDescriptors[i].request_length,	fDescriptors[i].actual_length,
			fDescriptors[i].status);
}


status_t
Stream::GetEnabledChannels(uint32& offset, multi_channel_enable* Enable)
{
	AudioChannelCluster* cluster = ChannelCluster();
	if (cluster == 0)
		return B_ERROR;

	for (size_t i = 0; i < cluster->ChannelsCount(); i++) {
		B_SET_CHANNEL(Enable->enable_bits, offset++, true);
		TRACE(INF, "Report channel %d as enabled.\n", offset);
	}

	return B_OK;
}


status_t
Stream::SetEnabledChannels(uint32& offset, multi_channel_enable* Enable)
{
	AudioChannelCluster* cluster = ChannelCluster();
	if (cluster == 0)
		return B_ERROR;

	for (size_t i = 0; i < cluster->ChannelsCount(); i++, offset++) {
		TRACE(INF, "%s channel %d.\n",
			(B_TEST_CHANNEL(Enable->enable_bits, offset)
			? "Enable" : "Disable"), offset + 1);
	}

	return B_OK;
}


status_t
Stream::GetGlobalFormat(multi_format_info* Format)
{
	_multi_format* format = fIsInput ? &Format->input : &Format->output;
	format->cvsr = fAlternates[fActiveAlternate]->GetSamplingRate();
	format->rate = fAlternates[fActiveAlternate]->GetSamplingRateId(0);
	format->format = fAlternates[fActiveAlternate]->GetFormatId();
	TRACE(INF, "%s.rate:%d cvsr:%f format:%#08x\n",
		fIsInput ? "input" : "ouput",
		format->rate, format->cvsr, format->format);
	return B_OK;
}


status_t
Stream::SetGlobalFormat(multi_format_info* Format)
{
	_multi_format* format = fIsInput ? &Format->input : &Format->output;
	AudioStreamAlternate* alternate = fAlternates[fActiveAlternate];
	// Skip reconfiguration only when nothing changed AND the buffers have
	// already been allocated. On the first call the requested format usually
	// already matches the alternate's default, but the sample buffers still
	// need to be set up (and the device's sampling rate programmed) before any
	// streaming can happen, so fall through while no area exists yet.
	if (fArea >= 0 && format->rate == alternate->GetSamplingRateId(0)
			&& format->format == alternate->GetFormatId()) {
		TRACE(INF, "No changes required\n");
		return B_OK;
	}

	alternate->SetSamplingRateById(format->rate);
	alternate->SetFormatId(format->format);
	TRACE(INF, "%s.rate:%d cvsr:%f format:%#08x\n",
		fIsInput ? "input" : "ouput",
		format->rate, format->cvsr, format->format);

	// cancel data flow - it will be rewaked at next buffer exchange call
	Stop();

	// TODO: wait for cancelling?

	// layout of buffers should be adjusted after changing sampling rate/format
	status_t status = _SetupBuffers();

	if (status != B_OK)
		return status;

	// set the sampling rate
	uint32 samplingRate = fAlternates[fActiveAlternate]->GetSamplingRate();

	if (fControlInterface->SpecReleaseNumber() >= 0x200) {
		// R2: the rate is set with a Clock Source class request on the
		// AudioControl interface, not an endpoint request as in R1.
		uint8 clockId
			= fControlInterface->ClockSourceIdForTerminal(TerminalLink());
		if (clockId == 0) {
			TRACE(ERR, "No clock source for terminal %d.\n", TerminalLink());
			return B_ERROR;
		}

		status = fControlInterface->SetSamplingRate(clockId, samplingRate);
		TRACE(ERR, "set_speed %d for clock %d: %s\n",
			samplingRate, clockId, strerror(status));
		return status;
	}

	// R1: set endpoint speed
	size_t actualLength = 0;
	usb_audio_sampling_freq freq = _ASFormatDescriptor::GetSamFreq(samplingRate);
	uint8 address = fAlternates[fActiveAlternate]->Endpoint()->fEndpointAddress;

	status = gUSBModule->send_request(fDevice->fDevice,
		USB_REQTYPE_CLASS | USB_REQTYPE_ENDPOINT_OUT,
		USB_AUDIO_SET_CUR, USB_AUDIO_SAMPLING_FREQ_CONTROL << 8,
		address, sizeof(freq), &freq, &actualLength);

	TRACE(ERR, "set_speed %02x%02x%02x for ep %#x %d: %s\n",
		freq.bytes[0], freq.bytes[1], freq.bytes[2],
		address, actualLength, strerror(status));
	return status;
}


status_t
Stream::GetBuffers(multi_buffer_list* List)
{
// TODO: check the available buffers count!
	if (fAreaSize == 0)
		return B_NO_INIT;

	int32 startChannel = List->return_playback_channels;
	buffer_desc** Buffers = List->playback_buffers;

	if (fIsInput) {
		List->flags |= B_MULTI_BUFFER_RECORD;
		List->return_record_buffer_size = fSamplesCount / kSamplesBufferCount;
		List->return_record_buffers = kSamplesBufferCount;
		startChannel = List->return_record_channels;
		Buffers = List->record_buffers;

		TRACE(DTA, "flags:%#10x\nreturn_record_buffer_size:%#010x\n"
			"return_record_buffers:%#010x\n", List->flags,
			List->return_record_buffer_size, List->return_record_buffers);
	} else {
		List->flags |= B_MULTI_BUFFER_PLAYBACK;
		List->return_playback_buffer_size = fSamplesCount / kSamplesBufferCount;
		List->return_playback_buffers = kSamplesBufferCount;

		TRACE(DTA, "flags:%#10x\nreturn_playback_buffer_size:%#010x\n"
			"return_playback_buffers:%#010x\n", List->flags,
			List->return_playback_buffer_size, List->return_playback_buffers);
	}

	TypeIFormatDescriptor* format = static_cast<TypeIFormatDescriptor*>(
		fAlternates[fActiveAlternate]->Format());

	// [buffer][channel] init buffers
	for (size_t buffer = 0; buffer < kSamplesBufferCount; buffer++) {
		TRACE(DTA, "%s buffer #%d:\n", fIsInput ? "input" : "output", buffer + 1);

		struct buffer_desc descs[format->fNumChannels];
		for (size_t channel = startChannel;
				channel < format->fNumChannels; channel++) {
			// init stride to the same for all buffers
			uint32 stride = format->fSubframeSize * format->fNumChannels;
			descs[channel].stride = stride;

			size_t bufferSize = (fSamplesCount / kSamplesBufferCount) * stride;
			descs[channel].base = (char*)fBuffers;
			descs[channel].base += buffer * bufferSize;
			descs[channel].base += channel * format->fSubframeSize;

			TRACE(DTA, "%d:%d: base:%#010x; stride:%#010x\n", buffer, channel,
				descs[channel].base, descs[channel].stride);
		}
		if (!IS_USER_ADDRESS(Buffers[buffer])
				|| user_memcpy(Buffers[buffer], descs, sizeof(descs)) < B_OK) {
			return B_BAD_ADDRESS;
		}
	}

	if (fIsInput) {
		List->return_record_channels += format->fNumChannels;
		TRACE(MIX, "return_record_channels:%#010x\n",
			List->return_record_channels);
	} else {
		List->return_playback_channels += format->fNumChannels;
		TRACE(MIX, "return_playback_channels:%#010x\n",
			List->return_playback_channels);
	}

	return B_OK;
}


bool
Stream::ExchangeBuffer(multi_buffer_info* Info)
{
	if (atomic_get(&fProcessedBuffers) <= 0)
		return false;

	if (fIsInput) {
		Info->recorded_real_time = fRealTime;
		Info->recorded_frames_count += fSamplesCount / kSamplesBufferCount;
		Info->record_buffer_cycle = fCurrentBuffer;
	} else {
		Info->played_real_time = fRealTime;
		Info->played_frames_count += fSamplesCount / kSamplesBufferCount;
		Info->playback_buffer_cycle = fCurrentBuffer;
	}

	atomic_add(&fProcessedBuffers, -1);

	return true;
}
