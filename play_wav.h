/* Audio Library for Teensy
 * Copyright (c) 2014, Paul Stoffregen, paul@pjrc.com
 *
 * Development of this audio library was funded by PJRC.COM, LLC by sales of
 * Teensy and Audio Adaptor boards.  Please support PJRC's efforts to develop
 * open source software by purchasing Teensy or other PJRC products.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

// (c) Frank Bösing, 07/2021

#pragma once

#include <Arduino.h>
#include <AudioStream.h>
#include <SD.h>


#define APW_ERR_OK              0 // no Error
#define APW_ERR_FORMAT          1 // not supported Format
#define APW_ERR_FILE   			2 // File not readable (does ist exist?)
#define APW_ERR_OUT_OF_MEMORY   3 // Not enough dynamic memory available
#define APW_ERR_NO_AUDIOBLOCKS  4


typedef int16_t _wpsample_t;

#if defined(KINETISL)
    const int _AudioPlayWav_MaxChannels = 2;
#else
	const int _AudioPlayWav_MaxChannels = 16;
#endif


class AudioPlayWav : public AudioStream
{
public:
	AudioPlayWav(void) : AudioStream(_AudioPlayWav_MaxChannels, queue) { begin(); }
	bool play(File file);
	bool play(File file, bool paused);
	bool play(const char *filename);
	bool play(const char *filename, bool paused); //start in paused state?
#if !defined(KINETISL)
	bool addMemoryForRead(size_t bytes); //add memory
#endif
	void togglePlayPause(void);
	void pause(bool pause);
	void stop(void);
	bool isPlaying(void);
	bool isPaused(void);
	bool isStopped(void);
	uint32_t positionMillis(void);
	uint32_t lengthMillis(void);
	uint32_t numBits(void);
	uint32_t numChannels(void);
	uint32_t sampleRate(void);
	uint32_t channelMask(void);
	uint8_t lastErr(void);              // returns last error
	size_t memUsed(void);
	uint8_t instanceID(void);
	virtual void update(void);
private:
  void begin(void);
	File wavfile;
	bool readHeader(int newState);
	inline void startUsingSPI(void);
	inline void stopUsingSPI(void);
    bool stopInt();
    void startInt(bool enabled);
	audio_block_t *queue[_AudioPlayWav_MaxChannels];
	_wpsample_t *buffer = nullptr;	    // buffer data
	size_t sz_mem = 0;					// Size of allocated memory
	size_t sz_frame;				    // Size of a sample frame in bytes
	size_t data_length;		  	        // number of frames remaining in file
	size_t buffer_rd;	                // where we're at consuming "buffer"	 Lesezeiger
	size_t total_length = 0;			// number of audio data bytes in file
	unsigned int sample_rate = 0;
	unsigned int channels = 0;			// #of channels in the wave file
	uint32_t channelmask = 0;           // dwChannelMask
	uint8_t my_instance;                // instance id
	uint8_t bytes = 0;  				// 1 or 2 bytes?
	uint8_t state;					    // play status (stop, pause, playing)
	uint8_t last_err = APW_ERR_OK;
};

