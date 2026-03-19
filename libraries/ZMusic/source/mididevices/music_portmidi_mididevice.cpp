/*
** music_coremidi_mididevice.mm
** Provides access to CoreMIDI on macOS for hardware MIDI playback
**
**---------------------------------------------------------------------------
** Copyright 2025 GZDoom Maintainers and Contributors
** All rights reserved.
**
** Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions
** are met:
**
** 1. Redistributions of source code must retain the above copyright
**    notice, this list of conditions and the following disclaimer.
** 2. Redistributions in binary form must reproduce the above copyright
**    notice, this list of conditions and the following disclaimer in the
**    documentation and/or other materials provided with the distribution.
** 3. The name of the author may not be used to endorse or promote products
**    derived from this software without specific prior written permission.
**
** THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
** IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
** OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
** IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
** INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
** NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
** THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**---------------------------------------------------------------------------
**
*/

#ifdef USE_PORTMIDI

#include <cmath>
#include <condition_variable>
#include <mutex>
#include <thread>

#include <portmidi.h>
#include <porttime.h>

#include "mididevice.h"
#include "zmusic/mididefs.h"
#include "zmusic/mus2midi.h"

//==========================================================================
//
// CoreMIDIDevice - CoreMIDI implementation for macOS
//
// Based on WinMIDIDevice (Windows) and AlsaMIDIDevice (Linux) patterns
//
//==========================================================================

class PortMIDIDevice : public MIDIDevice
{
public:
	PortMIDIDevice(int deviceID, bool precache);
	~PortMIDIDevice();

	int Open() override;
	void Close() override;
	bool IsOpen() const override;
	int GetTechnology() const override;
	int SetTempo(int tempo) override;
	int SetTimeDiv(int timediv) override;
	int StreamOut(MidiHeader* data) override;
	int StreamOutSync(MidiHeader* data) override;
	int Resume() override;
	void Stop() override;
	bool Pause(bool paused) override;
	bool FakeVolume() override;
	void InitPlayback() override;
	void PrecacheInstruments(const uint16_t* instruments, int count);

protected:
	void CalcTickRate();
	bool PlayTick();

	// PortMidi internals
	PortMidiStream* Stream;
	int BufferSize;
	int Latency;
	PmDeviceID DeviceID;

	enum EventType { TempoEv, LongMsgEv, ShortMsgEv };
	EventType EventType;
	bool Precache;

	// Threading
	std::thread PlayerThread;
	bool ExitRequested;
	std::condition_variable EventCV; // Still needed for pause/resume
	std::mutex EventMutex; // Still needed for pause/resume

	// Timing
	int Tempo;
	int InitialTempo;
	int Division;
	PmTimestamp CurrentEvTimeStamp; // This will track the host time of the current event being processed.
	PmTimestamp NextEvTimeStamp;
	double MilliSecsBetweenTicks; // Conversion factor: Host Time Units per MIDI Tick.
	MidiHeader* Events; // Linked list of MIDI headers
	uint32_t Position; // Current position in the MidiHeader buffer
	uint32_t PositionOffset;

	// Thread functions
	static void PlayerThreadProc(PortMIDIDevice* device);
	void PlayerLoop();

	void PrepareTempo(uint32_t tempo);
	void PrepareLongMsg(uint8_t* long_msg);
	void PrepareShortMsg(uint32_t short_msg);
	union EventMsg
	{
		uint32_t Tempo;
		uint8_t* Long;
		uint32_t Short;
	};
	EventMsg EventMsg;
	void HandleCurrentEvent();
};

//==========================================================================
//
// CoreMIDIDevice :: Constructor
//
//==========================================================================

PortMIDIDevice::PortMIDIDevice(int deviceID, bool precache)
	: DeviceID(deviceID)
	, Stream(nullptr)
	, BufferSize(1024)
	, Latency(1)
	, ExitRequested(false)
	, InitialTempo(500000)      // Default: 120 BPM (500,000 µs per quarter note)
	, Division(96)       // Default PPQN
	, CurrentEvTimeStamp(0)
	, Events(nullptr)
	, Position(0)
	, Precache(precache)
{
	// Initialize PM
	PmError error = Pm_Initialize();
	if (error)
	{
		ZMusic_Printf(ZMUSIC_MSG_ERROR,"Couldn't initialize PortMIDI: %s", Pm_GetErrorText(error));
	}
}

//==========================================================================
//
// CoreMIDIDevice :: Destructor
//
//==========================================================================

PortMIDIDevice::~PortMIDIDevice()
{
	Close();
	Pm_Terminate();
}

//==========================================================================
//
// CoreMIDIDevice :: Open
//
// Opens the MIDI device and connects to the specified endpoint
//
//==========================================================================

int PortMIDIDevice::Open()
{
	if (Stream) { return 0; }

	PmError error;

	// Create PM output
	int outputDevice = DeviceID;
	if (DeviceID > Pm_CountDevices() - 1)
	{
		outputDevice = Pm_GetDefaultOutputDeviceID();
		ZMusic_Printf(ZMUSIC_MSG_ERROR,"Device index \"%d\" is invalid, using default \"%d\" \"%s\" instead.\n"
			, DeviceID, outputDevice, Pm_GetDeviceInfo(outputDevice)->name);
	}

	error = Pm_OpenOutput(&Stream, outputDevice, NULL, BufferSize, NULL, NULL, Latency);
	if (error)
	{
		ZMusic_Printf(ZMUSIC_MSG_ERROR,"Couldn't create PortMIDI output device: %s\n", Pm_GetErrorText(error));
	}

	return error;
}

//==========================================================================
//
// CoreMIDIDevice :: Close
//
//==========================================================================

void PortMIDIDevice::Close()
{
	if (!Stream) { return; }

	// Stop player thread
	Stop();

	PmError error = Pm_Close(Stream);
	if (error)
	{
		ZMusic_Printf(ZMUSIC_MSG_ERROR,"Couldn't close PortMIDI stream: %s\n", Pm_GetErrorText(error));
	}
	Stream = nullptr;
}

//==========================================================================
//
// CoreMIDIDevice :: IsOpen
//
//==========================================================================

bool PortMIDIDevice::IsOpen() const
{
	return Stream;
}

//==========================================================================
//
// CoreMIDIDevice :: GetTechnology
//
//==========================================================================

int PortMIDIDevice::GetTechnology() const
{
	// Query if device is offline/virtual
	if (DeviceID == -1)
	{
		return MIDIDEV_SWSYNTH;
	}
	return MIDIDEV_MIDIPORT;
}

//==========================================================================
//
// CoreMIDIDevice :: CalcTickRate
//
//==========================================================================

void PortMIDIDevice::CalcTickRate()
{
	// Tempo is in microseconds per quarter note. Division is PPQN.
	// Host time units per second = AudioGetHostClockFrequency()
	// Microseconds per second = 1,000,000
	// Microseconds per tick = Tempo / Division
	// Host time units per tick = (Microseconds per tick) * (Host time units per second / Microseconds per second)
	MilliSecsBetweenTicks = (double)Tempo / (double)Division / double(1000);
}

//==========================================================================
//
// CoreMIDIDevice :: SetTempo
//
// Sets the playback tempo (microseconds per quarter note)
//
//==========================================================================

int PortMIDIDevice::SetTempo(int tempo)
{
	InitialTempo = tempo;
	return 0;
}

//==========================================================================
//
// CoreMIDIDevice :: SetTimeDiv
//
// Sets the time division (PPQN - pulses per quarter note)
//
//==========================================================================

int PortMIDIDevice::SetTimeDiv(int timediv)
{
	Division = timediv > 0 ? timediv : 96;
	return 0;
}

//==========================================================================
//
// CoreMIDIDevice :: StreamOut
//
// Queue MIDI data for asynchronous playback
//
//==========================================================================

int PortMIDIDevice::StreamOut(MidiHeader* data)
{
	if (!Stream) { return -1; }

	data->lpNext = nullptr;
	if (Events == nullptr)
	{
		Events = data;
		Position = 0;
	}
	else
	{
		MidiHeader** p;
		for (p = &Events; *p != nullptr; p = &(*p)->lpNext)
		{
		}
		*p = data;
	}
	return 0;
}

//==========================================================================
//
// CoreMIDIDevice :: StreamOutSync
//
// Queue MIDI data for synchronous playback
//
//==========================================================================

int PortMIDIDevice::StreamOutSync(MidiHeader* data)
{
	return StreamOut(data);
}

//==========================================================================
//
// CoreMIDIDevice :: Resume
//
// Start or resume playback
//
//==========================================================================

int PortMIDIDevice::Resume()
{
	if (!Stream) { return -1; }

	if (!PlayerThread.joinable())
	{
		ExitRequested = false;
		PlayerThread = std::thread(PlayerThreadProc, this);
	}
	return 0;
}

//==========================================================================
//
// CoreMIDIDevice :: Stop
//
// Stop playback
//
//==========================================================================

void PortMIDIDevice::Stop()
{
	if (!Stream) { return; }

	if (PlayerThread.joinable())
	{
		ExitRequested = true;
		EventCV.notify_all();
		PlayerThread.join();
	}

	std::this_thread::sleep_for(std::chrono::milliseconds(48)); // 3x time limit
#define SYSEX_RESET
#ifdef SYSEX_RESET
	Pm_WriteSysEx(Stream, 0, (uint8_t[]){0xf0, 0x7e, 0x7f, 0x09, 0x01, 0xf7}); // Universal General Midi reset message
#else
	for (int i = 0; i < 16; ++i)
	{
		Pm_WriteShort(Stream, 0, Pm_Message(0xB0 | i, 0x7B, 0x00)); // Notes off
		Pm_WriteShort(Stream, 0, Pm_Message(0xB0 | i, 0x78, 0x00)); // Sound off
		Pm_WriteShort(Stream, 0, Pm_Message(0xB0 | i, 0x79, 0x00)); // Reset all controllers
		Pm_WriteShort(Stream, 0, Pm_Message(0xB0 | i, 0x07, 0x64)); // Channel volume
		Pm_WriteShort(Stream, 0, Pm_Message(0xB0 | i, 0x0A, 0x40)); // Pan
		Pm_WriteShort(Stream, 0, Pm_Message(0xB0 | i, 0x00, 0x00)); // Bank select msb
		Pm_WriteShort(Stream, 0, Pm_Message(0xB0 | i, 0x20, 0x00)); // Bank select lsb
		Pm_WriteShort(Stream, 0, Pm_Message(0xC0 | i, 0x00, 0x00)); // Program change
		Pm_WriteShort(Stream, 0, Pm_Message(0xB0 | i, 0x64, 0x00)); // Pitch bend sens RPN LSB
		Pm_WriteShort(Stream, 0, Pm_Message(0xB0 | i, 0x65, 0x00)); // Pitch bend sens RPN MSB
		Pm_WriteShort(Stream, 0, Pm_Message(0xB0 | i, 0x06, 0x02)); // Data entry MSB
		Pm_WriteShort(Stream, 0, Pm_Message(0xB0 | i, 0x26, 0x00)); // Data entry LSB
		Pm_WriteShort(Stream, 0, Pm_Message(0xB0 | i, 0x64, 0x7F)); // Null RPN LSB
		Pm_WriteShort(Stream, 0, Pm_Message(0xB0 | i, 0x65, 0x7F)); // Null RPN MSB
		Pm_WriteShort(Stream, 0, Pm_Message(0xB0 | i, 0x5B, 40)); // Reverb
		Pm_WriteShort(Stream, 0, Pm_Message(0xB0 | i, 0x5D, 0)); // Chorus
	}
#endif
	std::this_thread::sleep_for(std::chrono::milliseconds(16));

	// Clear event queue
	Events = nullptr;
}

//==========================================================================
//
// CoreMIDIDevice :: Pause
//
// Pause/resume playback
//
//==========================================================================

bool PortMIDIDevice::Pause(bool paused)
{
	return false;
}

//==========================================================================
//
// CoreMIDIDevice :: FakeVolume
//
// CoreMIDI doesn't support volume control directly
//
//==========================================================================

bool PortMIDIDevice::FakeVolume()
{
	return true;  // No true volume control support, so fake volume
}

//==========================================================================
//
// CoreMIDIDevice :: InitPlayback
//
// Initialize playback state
//
//==========================================================================

void PortMIDIDevice::InitPlayback()
{
	// Initialize with current time (absolute, starts ticking when Pm_OpenOutput is called)
	CurrentEvTimeStamp = Pt_Time();
	Position = 0;
	Events = nullptr;
	Tempo = InitialTempo;
	CalcTickRate();
}

// WinMIDIDevice :: PrecacheInstruments
//
// Each entry is packed as follows:
//   Bits 0- 6: Instrument number
//   Bits 7-13: Bank number
//   Bit    14: Select drum set if 1, tone bank if 0
//
// My old GUS PnP needed the instruments to be preloaded, or it would miss
// some notes the first time through a song. I doubt any modern
// hardware has this problem, but since I'd already written the code for
// ZDoom 1.22 and below, I'm resurrecting it now for completeness, since I'm
// using preloading for the internal Timidity.
//
// NOTETOSELF: Why did I never notice the midiOutCache(Drum)Patches calls
// before now? Should I switch to them? This code worked on my GUS, but
// using the APIs intended for caching might be better.
//
//==========================================================================

void PortMIDIDevice::PrecacheInstruments(const uint16_t* instruments, int count)
{
	// Setting snd_midiprecache to false disables this precaching, since it
	// does involve sleeping for more than a miniscule amount of time.
	if (!Precache)
	{
		return;
	}
	uint8_t bank[16] = {0};
	int i, chan;

	for (i = 0, chan = 0; i < count; ++i)
	{
		int instr = instruments[i] & 127;
		int banknum = (instruments[i] >> 7) & 127;
		int percussion = instruments[i] >> 14;

		if (percussion)
		{
			if (bank[9] != banknum)
			{
				Pm_WriteShort(Stream, 0, Pm_Message(MIDI_CTRLCHANGE | 9, 0, banknum)); //(MIDI_CTRLCHANGE | 9 | (0 << 8) | (banknum << 16));
				bank[9] = banknum;
			}
			Pm_WriteShort(Stream, 0, Pm_Message(MIDI_NOTEON | 9, instruments[i] & 0x7f, 1)); // (MIDI_NOTEON | 9 | ((instruments[i] & 0x7f) << 8) | (1 << 16));
		}
		else
		{ // Melodic
			if (bank[chan] != banknum)
			{
				Pm_WriteShort(Stream, 0, Pm_Message(MIDI_CTRLCHANGE | 9, 0, banknum)); //(MIDI_CTRLCHANGE | 9 | (0 << 8) | (banknum << 16));
				bank[chan] = banknum;
			}
			Pm_WriteShort(Stream, 0, Pm_Message(MIDI_PRGMCHANGE | chan, instruments[i], 0)); //(MIDI_PRGMCHANGE | chan | (instruments[i] << 8));
			Pm_WriteShort(Stream, 0, Pm_Message(MIDI_NOTEON | chan, 60, 1)); //(MIDI_NOTEON | chan | (60 << 8) | (1 << 16));
			if (++chan == 9)
			{ // Skip the percussion channel
				chan = 10;
			}
		}
		// Once we've got an instrument playing on each melodic channel, sleep to give
		// the driver time to load the instruments. Also do this for the final batch
		// of instruments.
		if (chan == 16 || i == count - 1)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(250));
			for (chan = 15; chan-- != 0; )
			{
				// Turn all notes off
				Pm_WriteShort(Stream, 0, Pm_Message(MIDI_CTRLCHANGE | chan, 123, 0)); //(MIDI_CTRLCHANGE | chan | (123 << 8));
			}
			// And now chan is back at 0, ready to start the cycle over.
		}
	}
	// Make sure all channels are set back to bank 0.
	for (i = 0; i < 16; ++i)
	{
		if (bank[i] != 0)
		{
			Pm_WriteShort(Stream, 0, Pm_Message(MIDI_CTRLCHANGE | 9, 0, 0)); //(MIDI_CTRLCHANGE | 9 | (0 << 8) | (0 << 16));
		}
	}
}

//==========================================================================
//
// CoreMIDIDevice :: PlayTick
//
// Plays all events up to the current tick.
//
//==========================================================================

bool PortMIDIDevice::PlayTick()
{
	if (!Events && Callback)
	{	// No events in the current MidiHeader, request next buffer
		Callback(CallbackData);
	}

	if (!Events)
	{	// No events available to process.
		return false;
	}

	if (Position >= Events->dwBytesRecorded)
	{	// All events in the "Events" buffer were used, point to next buffer
		Events = Events->lpNext;
		Position = 0;
		if (Callback)
		{	// This ensures that we always have 2 unused buffers after 1 is used up.
			// omit this nested "if" block if you want to use up the 2 buffers before requesting new buffers
			Callback(CallbackData);
		}
	}

	if (!Events)
	{	// No events in the new buffer
		return false;
	}

	// Read the delta time (first 4 bytes of the event)
	uint32_t* event_ptr = (uint32_t*)(Events->lpData + Position);
	uint32_t tick_delta = event_ptr[0]; // Assuming delta time is the first uint32_t

	// Advance CurrentEventHostTime based on delta ticks.
	// This timestamp will be used for the current event, accurate to the 0.5 millisecond.
	NextEvTimeStamp = CurrentEvTimeStamp + round((double)tick_delta * MilliSecsBetweenTicks);

	uint32_t midi_event_type_param = event_ptr[2]; // This is the actual MIDI event or meta-event info

	if (midi_event_type_param < 0x80000000) // Short message (midi_event_type_param is the combined status/data bytes)
	{
		PositionOffset = 12; // 4 bytes delta time, 4 bytes reserved, 4 bytes MIDI message (up to 3 bytes + padding)
	}
	else // Long message or meta-event (midi_event_type_param holds type and parameter length)
	{
		PositionOffset = 12 + ((MEVENT_EVENTPARM(midi_event_type_param) + 3) & ~3);
	}

	switch (MEVENT_EVENTTYPE(midi_event_type_param))
	{
	case MEVENT_TEMPO:
		// Tempo change event, update our internal calculation for future events
		PrepareTempo(MEVENT_EVENTPARM(midi_event_type_param));
		break;
	case MEVENT_LONGMSG:
	{	// Long MIDI message (SysEx, etc.), data starts after event_ptr[3]
		int long_msg_len = MEVENT_EVENTPARM(midi_event_type_param);
		uint8_t* long_msg_data = (uint8_t*)&event_ptr[3];
		// Ensure valid sysex message
		if (long_msg_len > 2 && long_msg_data[0] == 0xF0 && long_msg_data[long_msg_len - 1] == 0xF7)
		{
			PrepareLongMsg(long_msg_data);
			break;
		}
	}
		break;
	case 0: // Short MIDI message (note on/off, control change, etc.)
	{
		uint8_t status = midi_event_type_param & 0xFF;
		uint8_t param1 = (midi_event_type_param >> 8) & 0x7f;
		uint8_t param2 = (midi_event_type_param >> 16) & 0x7f;
		uint32_t msg = Pm_Message(status, param1, param2);
		PrepareShortMsg(msg);
		break;
	}
	default:
		uint32_t msg = 0;
		PrepareShortMsg(msg);
	}
	// Other MEVENT_EVENTTYPE values (e.g., MEVENT_NOTEON, MEVENT_NOTEOFF etc. from WinMIDI)
	// are not directly used here; the raw MIDI message is parsed from event_ptr[2]

	return true;
	// Indicate that an event was processed and potentially more are available in the current tick.
	// The PlayerLoop will decide when to call PlayTick again.
}

//==========================================================================
//
// CoreMIDIDevice :: PlayerThreadProc
//
// Static thread entry point
//
//==========================================================================

void PortMIDIDevice::PlayerThreadProc(PortMIDIDevice* device)
{
	device->PlayerLoop();
}

//==========================================================================
//
// CoreMIDIDevice :: PlayerLoop
//
// Main player thread loop - processes MIDI events from queue
//
//==========================================================================

void PortMIDIDevice::PlayerLoop()
{
	std::unique_lock<std::mutex> lock(EventMutex);
	const int buffer_time_limit = 40;
	// Process all available events and schedule them with CoreMIDI
	while (!ExitRequested) //while (Events != nullptr && !Paused && !ExitRequested)
	{
		if (!PlayTick())
		{
			EventCV.wait_for(lock, std::chrono::milliseconds(buffer_time_limit));
			continue;
		}

		int next_ev_time_delta = NextEvTimeStamp - Pt_Time();
		int schedule_time = next_ev_time_delta - buffer_time_limit;
		if (schedule_time >= buffer_time_limit)
		{
			// Try to keep events under 2x time limit
			EventCV.wait_for(lock, std::chrono::milliseconds(schedule_time));
			continue;
		}
		CurrentEvTimeStamp = NextEvTimeStamp;
		Position += PositionOffset;
		HandleCurrentEvent();
	}
}

void PortMIDIDevice::PrepareTempo(const uint32_t tempo)
{
	EventType = TempoEv;
	EventMsg.Tempo = tempo;
}
void PortMIDIDevice::PrepareLongMsg(uint8_t* long_msg)
{
	EventType = LongMsgEv;
	EventMsg.Long = long_msg;
}
void PortMIDIDevice::PrepareShortMsg(uint32_t short_msg)
{
	EventType = ShortMsgEv;
	EventMsg.Short = short_msg;
}

void PortMIDIDevice::HandleCurrentEvent()
{
	switch (EventType)
	{
	case TempoEv:
		Tempo = EventMsg.Tempo;
		CalcTickRate();
		break;
	case LongMsgEv:
		Pm_WriteSysEx(Stream, CurrentEvTimeStamp, EventMsg.Long);
		break;
	case ShortMsgEv:
		Pm_WriteShort(Stream, CurrentEvTimeStamp, EventMsg.Short);
	}
}


//==========================================================================
//
// CreateCoreMIDIDevice
//
// Factory function to create a CoreMIDI device instance
//
//==========================================================================

MIDIDevice* CreatePortMIDIDevice(int mididevice)
{
	//return new PortMIDIDevice(mididevice);
	return new PortMIDIDevice(mididevice, miscConfig.snd_midiprecache);
}

#endif
