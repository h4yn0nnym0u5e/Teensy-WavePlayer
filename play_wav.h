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
#define APW_ERR_NO_AUDIOBLOCKS  4 // insufficient available audio blocks

#define APW_STATE_STOP    0
#define APW_STATE_PAUSED  1
#define APW_STATE_PLAY    2
#define APW_STATE_RECORD  APW_STATE_PLAY

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

//#define DEBUG_PIN_PLAYWAV 0 //enable to view the timing on a scope

#define CPULOAD_PLAYWAV // Can we remove this before first release?

#if defined(KINETISL)
    const int _AudioPlayWav_MaxChannels = 2;
#else
	const int _AudioPlayWav_MaxChannels = 16;
#endif




/*********************************************************************************************************/
/**
 * Class to move WAV data to and from files, optionally using
 * EventResponder to decouple interrupt demand from foreground
 * data access. Of most use with SD cards.
 *
 * Also keeps track of the required memory buffer, holds the
 * file object etc.
 */
class AudioBaseWav
{
public:
    AudioBaseWav(void)
	{
        SPTF("Constructing AudioBaseWav at %X\r\n",this);
        buf_unaligned = buffer = nullptr;
        #if defined (DEBUG_PIN_PLAYWAV)
            pinMode(DEBUG_PIN_PLAYWAV, OUTPUT);
        #endif

	}
    ~AudioBaseWav(void){ close(); }
	void pause(bool pause);
	bool isPaused(void) {return (state == APW_STATE_PAUSED);};
	bool isStopped(void) {return (state == APW_STATE_STOP);};

    uint8_t lastErr(void) {return last_err;};
	size_t memUsed(void) {return getBufferSize();};
	uint32_t filePos(void);
	uint32_t lengthMillis(void) {return total_length * (1000.0f / AUDIO_SAMPLE_RATE_EXACT);};
	uint32_t numBits(void) {return bytes * 8;}
	uint32_t numChannels(void) {return channels;};
	uint32_t sampleRate(void) {return sample_rate;};
    uint8_t instanceID(void) {return my_instance;};
    File file(void) {return wavfile;};
    float getCPUload() { return CYCLE_COUNTER_APPROX_PERCENT(lastFileCPUload * bytes * channels);}
	inline size_t getBufferSize() { return sz_mem; } //!< return size of buffer
	inline size_t position() { return wavfile.position(); }//!< return file position
	static void enableEventReading(bool enable) { eventReadingEnabled = enable; }

	operator bool() {return wavfile;}

	uint32_t lastFileCPUload;	//!< CPU load for last SD card transaction, spread over the number of audio blocks loaded
	File wavfile;				//!< file if streaming to/from SD card
	//--------------------------------------------------------------------------------------------------

protected:
	//--------------------------------------------------------------------------------------------------
    inline void setPadding(uint8_t b) { padding = b; }
    inline void seek(size_t pos) { wavfile.seek(pos); }//!< seek to new file position

	// Simple functions we can define immediately:
	inline void readLater(void) //!< from interrupt: request to re-fill the buffer
		{
			if (eventReadingEnabled)
				evResp.triggerEvent(0,this); // do the read in the foreground: must delay() or yield()
			else
				read(buffer,sz_mem);		 // read immediately

		}

	inline void writeLater(void) //!< from interrupt: request to write the buffer to filesystem
		{
			if (eventReadingEnabled)
				evResp.triggerEvent(0,this); // do the read in the foreground: must delay() or yield()
			else
				write(buffer,sz_mem);		 // write immediately

		}

	inline int8_t* getBuffer() { return buffer; } //!< return pointer to buffer holding WAV data

	int8_t* createBuffer(size_t len) //!< allocate the buffer
		{
			sz_mem = len;
			buf_unaligned = malloc(sz_mem + 31);
            buffer = (int8_t*)(((uintptr_t)(buf_unaligned) + 31) & ~31);
			SPTF("Allocated %d aligned bytes at %X - %X\r\n",sz_mem, buffer, buffer+sz_mem-1);
			//for (size_t i=0;i<len/2;i++) *((int16_t*) buffer+i) = i * 30000 / len;
			return buffer;
		}

	inline size_t read(void* buf,size_t len) //!< read len bytes immediately into buffer provided
		{
            #if defined (DEBUG_PIN_PLAYWAV)
                digitalWriteFast(DEBUG_PIN_PLAYWAV, HIGH);
            #endif
            #if defined (CPULOAD_PLAYWAV)
			uint32_t tmp = ARM_DWT_CYCCNT;
            #endif
			size_t result = wavfile.read(buf,len);

			if ( result < len )
				memset((int8_t*) buf+result, padding , len - result);
			SPTF("Read %d bytes to %x: fifth int16 is %d\r\n",len,buf,*(((int16_t*) buf)+4));

            #if defined (CPULOAD_PLAYWAV)
			// % CPU load per track per update cycle: assumes 8-bit samples
			lastFileCPUload = ((ARM_DWT_CYCCNT - tmp) * AUDIO_BLOCK_SAMPLES / len)>>6;
            #endif
            #if defined (DEBUG_PIN_PLAYWAV)
                digitalWriteFast(DEBUG_PIN_PLAYWAV, LOW);
            #endif

			return result;
		}

	inline size_t write(void* buf,size_t len) //!< write len bytes immediately from buffer to filesystem
		{
            #if defined (DEBUG_PIN_PLAYWAV)
                digitalWriteFast(DEBUG_PIN_PLAYWAV, HIGH);
            #endif
            #if defined (CPULOAD_PLAYWAV)
			uint32_t tmp = ARM_DWT_CYCCNT;
            #endif
			size_t result = wavfile.write(buf,len);

			SPTF("Wrote %d bytes to %x: fifth int16 is %d\r\n",len,buf,*(((int16_t*) buf)+4));

			#if defined (CPULOAD_PLAYWAV)
            // % CPU load per track per update cycle: assumes 8-bit samples
			lastFileCPUload = ((ARM_DWT_CYCCNT - tmp) * AUDIO_BLOCK_SAMPLES / len)>>6;
            #endif
            #if defined (DEBUG_PIN_PLAYWAV)
                digitalWriteFast(DEBUG_PIN_PLAYWAV, LOW);
            #endif

			return result;
		}

    void close() //!< close file, free up the buffer, detach responder
		{
			bool irq = stopInt();

			if (wavfile)
				wavfile.close();

			if (nullptr != buf_unaligned)
			{
				SPTF("\r\Freed %d aligned bytes at %X - %X\r\n",sz_mem, buffer, buffer+sz_mem-1);
				free(buf_unaligned);
				buf_unaligned = buffer = nullptr;
			}

			evResp.clearEvent(); // not intuitive, but works SO much better...

			startInt(irq);
		}

	static void evFuncRead(EventResponderRef ref) //!< foreground: respond to request to load WAV data
	{
		AudioBaseWav& thisWM = *(AudioBaseWav*) ref.getData();

		SPRT("*** ");
		thisWM.read(thisWM.buffer,thisWM.sz_mem);
	}

	static void evFuncWrite(EventResponderRef ref) //!< foreground: respond to request to save WAV data
	{
		AudioBaseWav& thisWM = *(AudioBaseWav*) ref.getData();

		SPRT("*** ");
		thisWM.write(thisWM.buffer,thisWM.sz_mem);
	}

	//--------------------------------------------------------------------------------------------------

	bool initRead(File file);
	bool initWrite(File file);
    bool isRunning(void);
    void togglePause(void);
	void startUsingSPI(void);
	void stopUsingSPI(void);
    bool stopInt(void);
    void startInt(bool enabled);
	unsigned int sample_rate = 0;
	unsigned int channels = 0;			// #of channels in the wave file
    size_t total_length = 0;			// number of audio data bytes in file
    uint8_t fileFmt = 0;                // file format (0 = *.wav, 3= *.aif, more to come)
    uint8_t dataFmt = 0;                // data format (0 = std, 1 = 8 bit signed, 2 = ulaw, 3=16bit big endian)
	uint8_t my_instance;                // instance id
	uint8_t bytes = 0;  				// 1 or 2 bytes?
	uint8_t state = APW_STATE_STOP;	    // play status (stop, pause, playing)
    uint8_t last_err = APW_ERR_OK;

private:
	EventResponder evResp; 			//!< executes data transfer in foreground
    void* buf_unaligned;            // the malloc'd buffer
    int8_t* buffer;					//!< buffer to store pre-loaded WAV data going to or from SD card
	size_t sz_mem;					//!< size of buffer

	uint8_t padding;				//!< value to pad buffer at EOF
	static bool eventReadingEnabled;//!< true to read filesystem via EventResponder; otherwise inside update() as usual
};

/*********************************************************************************************************/

class AudioPlayWav : public AudioBaseWav, public AudioStream
{
public:
	AudioPlayWav(void) : AudioStream(0, NULL) { begin(); }
	~AudioPlayWav(void) { end(); } // no need to free audio blocks, never permanently allocates any
    void stop(void);
	bool play(File file, bool paused = false);
	bool play(const char *filename, bool paused = false); // optional start in paused state
    // Todo:
    // - playRaw
    // - playAiff (!! code added, untested, test needed!!)
	static bool addMemoryForRead(size_t mult); // add memory
	void togglePlayPause(void) {togglePause();};
    bool isPlaying(void) {return isRunning();};
	uint32_t positionMillis(void);
	uint32_t channelMask(void) {return channelmask;};
	virtual void update(void);
	static void enableEventReading(bool enable) { AudioBaseWav::enableEventReading(enable); }
	float getCPUload()
	{ 	// correct for bytes/sample and number of channels in file
		return CYCLE_COUNTER_APPROX_PERCENT(lastFileCPUload * bytes * channels);
	}
private:
    void begin(void);
	void end(void);
	bool readHeader(int newState);
    size_t (*decoder)(int8_t buffer[], size_t buffer_rd, audio_block_t *queue[], const unsigned int channels);    
	int data_length;		  	        // number of frames remaining in file
	size_t buffer_rd;	                // where we're at consuming "buffer"	 Lesezeiger
	uint32_t channelmask = 0;           // dwChannelMask
};

#if 0 && !defined(KINETISL)
class AudioRecordWav : public AudioBaseWav, public AudioStream
{
};
#endif // defined(KINETISL)