/* 
   Wavefile player 
   Copyright (c) 2021, Frank Bösing, f.boesing @ gmx (dot) de
   
   for 
   
   Audio Library for Teensy
   Copyright (c) 2014, Paul Stoffregen, paul@pjrc.com

   
   Development of this audio library was funded by PJRC.COM, LLC by sales of
   Teensy and Audio Adaptor boards.  Please support PJRC's efforts to develop
   open source software by purchasing Teensy or other PJRC products.

   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:

   The above copyright notice, development funding notice, and this permission
   notice shall be included in all copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
   THE SOFTWARE.


   Addition by Frank Bösing:
   Use of this code (wavefile player) permitted for use with hardware developed by PJRC only.
*/

#pragma once

#include <Arduino.h>
#include <AudioStream.h>
#include <SD.h>
#include <EventResponder.h>


#define APW_ERR_OK              0 // no Error
#define APW_ERR_FORMAT          1 // not supported Format
#define APW_ERR_FILE   			2 // File not readable (does it exist?)
#define APW_ERR_OUT_OF_MEMORY   3 // Not enough dynamic memory available
#define APW_ERR_NO_AUDIOBLOCKS  4 // insufficient available audo blocks

#define xDEBUG_PRINT_PLAYWAV
#if defined(DEBUG_PRINT_PLAYWAV)
#define SPLN(...) Serial.println(__VA_ARGS__)
#define SPRT(...) Serial.print(__VA_ARGS__)
#define SPTF(...) Serial.printf(__VA_ARGS__)
#define SFSH(...) Serial.flush()
#else
#define SPLN(...) 
#define SPRT(...) 
#define SPTF(...) 
#define SFSH(...) 
#endif  // defined(DEBUG_PRINT_PLAYWAV)

#if defined(KINETISL)
    const int _AudioPlayWav_MaxChannels = 2;
#else
	const int _AudioPlayWav_MaxChannels = 16;
#endif

class AudioPlayWav;

/**
 * Class to move WAV data from (and eventually to?) files using
 * EventResponder to decouple interrupt demand from foreground
 * data access. Of most use with SD cards.
 */
class WavMover
{
public:
	WavMover() : buffer{nullptr} {SPTF("Constructing WavMover at %X\r\n",this);}
	~WavMover() { close(); }
	bool play(File file); //!< prepare to play a file that's already open
	
	// Simple functions we can define immediately:
	void readLater(void) //!< from interrupt: request re-fill the buffer
		{ 
			if (eventReadingEnabled)
				evResp.triggerEvent(0,this); // do the read in the foreground: must delay() or yield()
			else
				read(buffer,sz_mem);		 // read immediately
				
		}
		
	int8_t* getBuffer() //!< return pointer to buffer holding WAV data
		{ return buffer; }
		
	size_t getBufferSize() //!< return size of buffer
		{ return sz_mem; }
		
	int8_t* createBuffer(size_t len) //!< allocate the buffer
		{ 
			sz_mem = len; 
			buffer = (int8_t*) malloc(sz_mem); 
			SPTF("Allocated %d bytes at %X - %X\r\n",sz_mem, buffer, buffer+sz_mem-1);
			for (size_t i=0;i<len/2;i++) *((int16_t*) buffer+i) = i * 30000 / len;
			return buffer;
		}
		
	size_t read(void* buf,size_t len) //!< read len bytes immediately into buffer provided
		{ 
			uint32_t tmp = ARM_DWT_CYCCNT;
			size_t result = wavfile.read(buf,len);
			
			if ( result < len ) 
				memset((int8_t*) buf+result, padding , len - result);				
			SPTF("Read %d bytes to %x: fifth int16 is %d\r\n",len,buf,*(((int16_t*) buf)+4));
			
			// % CPU load per track per update cycle
			lastReadLoad = ((ARM_DWT_CYCCNT - tmp) * AUDIO_BLOCK_SAMPLES / len)>>6;
			
			return result;
		}
		
	void seek(size_t pos) //!< seek to new file position
		{ wavfile.seek(pos); }
		
	size_t position() //!< return file position
		{ return wavfile.position(); }
		
	void close() //!< close file, free up the buffer, detach responder
		{ 
			bool irq = stopInt();
			
			if (wavfile)  
				wavfile.close();  
			
			if (nullptr != buffer) 
			{ 
				SPTF("\r\Freed %d bytes at %X - %X\r\n",sz_mem, buffer, buffer+sz_mem-1);
				free(buffer); 
				buffer = nullptr; 
			}
			
			//evResp.detach();	
			evResp.clearEvent(); // not intuitive, but works SO much better...
			
			startInt(irq);
		}
		
	void setPadding(uint8_t b) { padding = b; }
	
	static void enableEventReading(bool enable) { eventReadingEnabled = enable; }
	
	operator bool() {return wavfile;}
	
	uint32_t lastReadLoad;		//!< CPU load for last SD card read, spread over the number of audio blocks loaded
	File wavfile;					//!< file if streaming to/from SD card
	
private:
	bool stopInt()
	{
		if ( NVIC_IS_ENABLED(IRQ_SOFTWARE) )
		{
			NVIC_DISABLE_IRQ(IRQ_SOFTWARE);
			return true;
		}
		return false;
	}

	void startInt(bool enabled)
	{
    if (enabled)
        NVIC_ENABLE_IRQ(IRQ_SOFTWARE);
	}	
	
	static void evFunc(EventResponderRef ref) //!< foreground: respond to request to load WAV data
	{
		WavMover& thisWM = *(WavMover*) ref.getData();
		
		SPRT("*** ");
		thisWM.read(thisWM.buffer,thisWM.sz_mem);
	}
	EventResponder evResp; 			//!< executes data transfer in foreground
	int8_t* buffer;					//!< buffer to store pre-loaded WAV data
	size_t sz_mem;					//!< size of buffer	
	uint8_t padding;				//!< value to pad buffer at EOF
	static bool eventReadingEnabled;//!< true to read filesystem via EventResponder; otherwise inside update() as usual
};


class AudioPlayWav : public AudioStream
{
public:
	AudioPlayWav(void) : AudioStream(0, NULL) { begin(); }
	~AudioPlayWav(void) { end(); } // no need to free audio blocks, never permanently allocates any
	bool play(File file);
	bool play(File file, bool paused);
	bool play(const char *filename);
	bool play(const char *filename, bool paused); // optional start in paused state
	static bool addMemoryForRead(size_t mult); // add memory
	void togglePlayPause(void);
	void pause(bool pause);
	void stop(void);
	bool isPlaying(void);
	bool isPaused(void);
	bool isStopped(void);
	uint32_t positionMillis(void);
	uint32_t filePos(void);
	uint32_t lengthMillis(void);
	uint32_t numBits(void);
	uint32_t numChannels(void);
	uint32_t sampleRate(void);
	uint32_t channelMask(void);
	uint8_t lastErr(void);              // returns last error
	size_t memUsed(void);
	size_t memRead(void);
	uint8_t instanceID(void);
	File file(void);
	virtual void update(void);
	static void enableEventReading(bool enable) { WavMover::enableEventReading(enable); }
	float getCPUload() { return CYCLE_COUNTER_APPROX_PERCENT(wavMovr.lastReadLoad); }
private:
    void begin(void);
	void end(void);
	bool readHeader(int newState);
	void startUsingSPI(void);
	void stopUsingSPI(void);
    bool stopInt(void);
    void startInt(bool enabled);
    void (*decoder)(int8_t buffer[], size_t *buffer_rd, audio_block_t *queue[], unsigned int channels);
	WavMover wavMovr;
	//File wavfile;
	//int8_t *buffer = nullptr;	        // buffer data
	//size_t sz_mem = 0;				// Size of allocated memory
	int data_length;		  	        // number of frames remaining in file
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

