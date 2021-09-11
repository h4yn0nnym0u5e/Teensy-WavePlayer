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

#define ENABLE_EVENTRESPONDER_PLAYWAV //comment to reduce codesize, TensyLC: always disabled.

enum APW_FORMAT { APW_8BIT_UNSIGNED = 0, APW_8BIT_SIGNED, APW_ULAW,
                  APW_16BIT_SIGNED, APW_16BIT_SIGNED_BIGENDIAN,
                  APW_NONE};

enum APW_ERR { ERR_OK = 0,              // no Error
               ERR_FORMAT = 1,          // File not usable (does it exist?)
               ERR_FILE = 2,            // not supported Format
               ERR_OUT_OF_MEMORY = 3,   // Not enough dynamic memory available
               ERR_NO_AUDIOBLOCKS = 4}; // insufficient # of available audio blocks

/*********************************************************************************************************/

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


#if defined(KINETISL)
    const int _AudioPlayWav_MaxChannels = 2;
#define USE_EVENTRESPONDER_PLAYWAV 0
#else
	const int _AudioPlayWav_MaxChannels = 16;
    const int _AudioRecordWav_MaxChannels = 4;
#ifdef ENABLE_EVENTRESPONDER_PLAYWAV
#define USE_EVENTRESPONDER_PLAYWAV 1
#endif

#endif//defined(KINETISL)


#ifdef USE_EVENTRESPONDER_PLAYWAV
#include <EventResponder.h>
#define CPULOAD_PLAYWAV // TODO: Can we remove this before next release?
#endif


enum APW_STATE {STATE_STOP, STATE_PAUSED, STATE_RUNNING};

class AudioPlayWav;
class AudioRecordWav;

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
	void pause(bool pause);
    void togglePause(void);
	bool isPaused(void) {return (state == STATE_PAUSED);};
	bool isStopped(void) {return (state == STATE_STOP);};
	inline size_t getBufferSize() { return sz_mem; } //!< return size of buffer
	inline size_t position() { return wavfile.position(); }//!< return file position
    operator bool() {return wavfile;}

    #ifdef CPULOAD_PLAYWAV
    float getCPUload() { return CYCLE_COUNTER_APPROX_PERCENT(lastFileCPUload * bytes * channels);}
    uint32_t lastFileCPUload;	//!< CPU load for last SD card transaction, spread over the number of audio blocks loaded
    #endif

    #if USE_EVENTRESPONDER_PLAYWAV
	static void enableEventReading(bool enable) { eventReadingEnabled = enable; }
    #endif

    bool addMemory(size_t mult);
	size_t memUsed(void) {return getBufferSize();};
	uint32_t filePos(void);
	uint32_t numBits(void) {return bytes * 8;}
	uint32_t numChannels(void) {return channels;};
	uint32_t sampleRate(void) {return sample_rate;};
    uint8_t instanceID(void) {return my_instance;};
    uint8_t lastErr(void) {return (int)last_err;};
    File file(void) {return wavfile;};
	//--------------------------------------------------------------------------------------------------

private:
    friend class AudioPlayWav;
    friend class AudioRecordWav;

    AudioBaseWav(void);
    ~AudioBaseWav(void){ close(); }

	int8_t* createBuffer(size_t len); //!< allocate the buffer
    inline int8_t* getBuffer() { return buffer; } //!< return pointer to buffer holding WAV data

    inline bool seek(size_t pos) { return wavfile.seek(pos); }//!< seek to new file position
    inline void flush(void) { wavfile.flush(); }
    inline size_t size() { return wavfile.size(); }//!< return file position

	inline void readLater(void); //!< from interrupt: request to re-fill the buffer
	inline void writeLater(void); //!< from interrupt: request to write the buffer to filesystem

	inline size_t read(void* buf,size_t len); //!< read len bytes immediately into buffer provided
	inline size_t write(void* buf,size_t len); //!< write len bytes immediately from buffer to filesystem

    void close(bool closeFile = true); //!< close file, free up the buffer, detach responder

    #if USE_EVENTRESPONDER_PLAYWAV
	static void evFuncRead(EventResponderRef ref); //!< foreground: respond to request to load WAV data
	static void evFuncWrite(EventResponderRef ref); //!< foreground: respond to request to save WAV data
    #endif

	//--------------------------------------------------------------------------------------------------
	bool initRead(File file);
	bool initWrite(File file);
    bool isRunning(void);
	void startUsingSPI(void);
	void stopUsingSPI(void);
    bool stopInt(void);
    void startInt(bool enabled);

    File wavfile;
    size_t sz_mem;					    //!< size of buffer
    void* buf_unaligned;                // the malloc'd buffer
    int8_t* buffer;					    //!< buffer to store pre-loaded WAV data going to or from SD card
    #if USE_EVENTRESPONDER_PLAYWAV
	EventResponder evResp; 			    //!< executes data transfer in foreground
    static bool eventReadingEnabled;    //!< true to read filesystem via EventResponder; otherwise inside update() as usual
    #endif
	unsigned int sample_rate = 0;
	unsigned int channels = 0;			// #of channels in the wave file
    int data_length;		  	        // number of frames remaining in file /# of recorded frames
    APW_FORMAT dataFmt;
	uint8_t my_instance;                // instance id
    bool usingSPI = false;
	uint8_t bytes = 0;  				// 1 or 2 bytes?
	APW_STATE state = STATE_STOP;	    // play status (stop, pause, running)
    APW_ERR last_err = ERR_OK;
    uint8_t padding = 0;                //!< value to pad buffer at EOF
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

    bool playRaw(File file, APW_FORMAT fmt, uint32_t sampleRate, uint8_t number_of_channels, bool paused = false);
    bool playRaw(const char *filename, APW_FORMAT fmt, uint32_t sampleRate, uint8_t number_of_channels, bool paused = false);

    bool addMemoryForRead(size_t mult){return addMemory(mult);}; // add memory
	void togglePlayPause(void) {togglePause();};
    bool isPlaying(void) {return isRunning();};
	uint32_t positionMillis(void);
	uint32_t channelMask(void) {return channelmask;};
    uint32_t lengthMillis(void) {return total_length * (1000.0f / AUDIO_SAMPLE_RATE_EXACT);};
    #if USE_EVENTRESPONDER_PLAYWAV
	static void enableEventReading(bool enable) { AudioBaseWav::enableEventReading(enable); }
    #endif
private:
    virtual void update(void);
    void begin(void);
	void end(void);
    bool readHeader(APW_FORMAT fmt, uint32_t sampleRate, uint8_t number_of_channels, APW_STATE newState );
    inline void setPadding(uint8_t b) { padding = b; }
    size_t (*decoder)(int8_t buffer[], size_t buffer_rd, audio_block_t *queue[], const unsigned int channels);
    size_t total_length = 0;			// number of audio data bytes in file
	size_t buffer_rd;	                // where we're at consuming "buffer"	 Lesezeiger
	uint32_t channelmask = 0;           // dwChannelMask
};
/*********************************************************************************************************/

#if !defined(KINETISL)
class AudioRecordWav : public AudioBaseWav, public AudioStream
{
public:
    AudioRecordWav(void): AudioStream(_AudioRecordWav_MaxChannels, queue) { begin(); }
    ~AudioRecordWav(void) { end(); }
    void stop(bool closeFile = true);
	void pause(const bool pause);
    bool record(File file, APW_FORMAT fmt, unsigned int channels, bool paused = false);
    bool record(const char *filename, APW_FORMAT fmt, unsigned int channels, bool paused = false);

    bool writeHeader(File file); //updates header of a file
    bool writeHeader(const char *filename);
    bool writeHeader(void) {return writeHeader(wavfile);}; //updates header of current file

    bool isRecording(void) {return isRunning();};
    void toggleRecordPause(void) {togglePause();};
    bool addMemoryForWrite(size_t mult){return addMemory(mult);}; // add memory

    #if USE_EVENTRESPONDER_PLAYWAV
	static void enableEventReading(bool enable) { AudioBaseWav::enableEventReading(enable); }
    #endif

private:
    virtual void update(void);
    void begin(void);
    void end(void);
    size_t (*encoder)(int8_t buffer[], size_t buffer_rd, audio_block_t *queue[], const unsigned int channels);
    audio_block_t *queue[_AudioRecordWav_MaxChannels];
    size_t sz_frame;
    size_t buffer_wr;
    int data_length_old;
};
#endif // defined(KINETISL)