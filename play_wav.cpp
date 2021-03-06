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

#include "play_wav.h"
#include <spi_interrupt.h>


//#define HANDLE_SPI    1 //TODO...


#if defined(KINETISL)
static const uint8_t _instances = 1;
static const uint8_t _sz_mem_additional = 1;
static uint8_t _lastInstance;
#if AUDIO_BLOCK_SAMPLES < 128
//#warning WavePlay: AUDIO_BLOCK_SAMPLES is less than 128. Expect noise.
#endif // AUDIO_BLOCK_SAMPLES < 128
#else
static uint8_t _instances = 0;
static uint8_t _sz_mem_additional = 1;
static uint8_t _lastInstance;
static const audio_block_t zeroblock = {0}; // required to deal gracefully with NULL block, no matter which encoder we use
#endif // defined(KINETISL)

#define PACKED __attribute__((packed))

static const uint8_t bytesPerSample[APW_NONE] = {1, 1, 1, 2, 2};

//----------------------------------------------------------------------------------------------------
#if !defined(KINETISL)
//Reverse Byte order:
__attribute__( ( always_inline ) ) static inline uint32_t __rev(uint32_t value)
{
  uint32_t result;
  asm volatile ("rev %0, %1" : "=r" (result) : "r" (value) );
  return(result);
}

__attribute__( ( always_inline ) ) static inline uint32_t __rev16(uint32_t value)
{
  uint32_t result;
  asm volatile ("rev16 %0, %1" : "=r" (result) : "r" (value) );
  return(result);
}

__attribute__(( always_inline )) static inline uint32_t __ldrexw(volatile uint32_t *addr)
{
   uint32_t result;
   asm volatile ("ldrex %0, [%1]" : "=r" (result) : "r" (addr) );
   return(result);
}

__attribute__(( always_inline )) static inline uint32_t __strexw(uint32_t value, volatile uint32_t *addr)
{
   uint32_t result;
   asm volatile ("strex %0, %2, [%1]" : "=&r" (result) : "r" (addr), "r" (value) );
   return(result);
}

#endif

//----------------------------------------------------------------------------------------------------
AudioBaseWav::AudioBaseWav(void)
	{
        SPTF("Constructing AudioBaseWav at %X\r\n",this);
        buf_unaligned = buffer = nullptr;
        #if defined (DEBUG_PIN_PLAYWAV)
            pinMode(DEBUG_PIN_PLAYWAV, OUTPUT);
        #endif
	}

bool AudioBaseWav::stopInt()
{
    if ( NVIC_IS_ENABLED(IRQ_SOFTWARE) )
    {
        NVIC_DISABLE_IRQ(IRQ_SOFTWARE);
        return true;
    }
    return false;
}

void AudioBaseWav::startInt(bool enabled)
{
    if (enabled)
        NVIC_ENABLE_IRQ(IRQ_SOFTWARE);
}

bool AudioBaseWav::isRunning(void)
{
    //yield();
    return state == STATE_RUNNING;
}

void AudioBaseWav::pause(const bool pause)
{
    if (state == STATE_STOP) return;
    bool irq = stopInt();
    if (pause)
    {
        state = STATE_PAUSED;
        stopUsingSPI();
    } else {
        startUsingSPI();
        state = STATE_RUNNING;
    }
    startInt(irq);
}

void AudioBaseWav::togglePause(void)
{
    bool irq = stopInt();
    pause(state == STATE_RUNNING);
    startInt(irq);
}

uint32_t AudioBaseWav::filePos(void)
{
	uint32_t result = 123456L;

	if (wavfile)
		result = wavfile.position();

	return result;
}

bool AudioBaseWav::addMemory(__attribute__ ((unused)) size_t mult)
{
#if !defined(KINETISL)
    if (mult < 1) mult = 1;
	_sz_mem_additional = mult;
#endif // !defined(KINETISL)
	return true;
}

void AudioBaseWav::startUsingSPI(void)
{
//TODO... https://forum.pjrc.com/threads/67989-Teensyduino-1-55-Beta-1?p=287023&viewfull=1#post287023
//this must be smarter.
#if defined(HANDLE_SPI)
#if defined(HAS_KINETIS_SDHC)
    if (!usingSPI && !(SIM_SCGC3 & SIM_SCGC3_SDHC))
    {
        AudioStartUsingSPI();
        usingSPI = true;
    }
#else
    if (!usingSPI)
    {
        AudioStartUsingSPI();
        usingSPI = true;
    }
#endif // defined(HAS_KINETIS_SDHC)
#endif //defined(HANDLE_SPI)
}

void AudioBaseWav::stopUsingSPI(void)
{ //TODO...
#if defined(HANDLE_SPI)
#if defined(HAS_KINETIS_SDHC)
    if (usingSPI && !(SIM_SCGC3 & SIM_SCGC3_SDHC)
    {
        AudioStopUsingSPI();
        usingSPI = false;
    }
#else
    if (usingSPI)
    {
        AudioStopUsingSPI();
        usingSPI = false;
    }
#endif // defined(HAS_KINETIS_SDHC)
#endif //defined(HANDLE_SPI)
}

//----------------------------------------------------------------------------------------------------
int8_t* AudioBaseWav::createBuffer(size_t len) //!< allocate the buffer
{

    #if defined(__IMXRT1062__)
    buf_unaligned = malloc(len + 31);
    buffer = (int8_t*)(((uintptr_t)(buf_unaligned) + 31) & ~31);
    #else
    buf_unaligned = malloc(len);
    buffer = (int8_t*) buf_unaligned;
    #endif

	if (buf_unaligned != nullptr)
		sz_mem  = len;
	else
		sz_mem = 0;
	
	firstHalf = true; // start by using first half, if we're double-buffering
	count = 0; // zero count of times we've attempted to write from this buffer

    //Serial.printf("Allocated %d aligned bytes at %X - %X\r\n",sz_mem, buffer, buffer+sz_mem-1);
    for (size_t i=0;i<len/2;i++) *((int16_t*) buffer+i) = i * 30000 / len;
    return buffer;
}

#if USE_EVENTRESPONDER_PLAYWAV
void AudioBaseWav::evFuncRead(EventResponderRef ref) //!< foreground: respond to request to load WAV data
{
    AudioBaseWav& thisWM = *(AudioBaseWav*) ref.getData();

    SPRT("*** ");
    thisWM.read(thisWM.getOtherBuffer(),thisWM.sz_mem / 2);
}

void AudioBaseWav::evFuncWrite(EventResponderRef ref) //!< foreground: respond to request to save WAV data
{
    AudioBaseWav& thisWM = *(AudioBaseWav*) ref.getData();
	size_t wr,expectedWR = thisWM.sz_mem / 2;

    SPRT("*** ");
    wr = thisWM.write(thisWM.getOtherBuffer(),expectedWR);
	if (wr < expectedWR) 
	{
		thisWM.state = STATE_ABORT; // needs to stop on next update()
		thisWM.last_err = ERR_FILE;	//Disk full, max filesize reached, SD Card removed etc.
	}
}
#endif

void AudioBaseWav::readLater(void) //!< from interrupt: request to re-fill the buffer
{
    #if USE_EVENTRESPONDER_PLAYWAV
    if (eventReadingEnabled)
        evResp.triggerEvent(0,this); // do the read in the foreground: must delay() or yield()
    else
    #endif
        read(getBuffer(),sz_mem);		 // read immediately

	firstHalf = !firstHalf; // switch interrupts to using the other half of the buffer memory
}

size_t AudioBaseWav::read(void* buf,size_t len) //!< read len bytes immediately into buffer provided
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

size_t AudioBaseWav::write(void* buf,size_t len) //!< write len bytes immediately from buffer to filesystem
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

void AudioBaseWav::writeLater(void) //!< from interrupt: request to write the buffer to filesystem
{
	int16_t* buf = (int16_t*) getBuffer();
	count++;
	//*buf = count + (firstHalf?30000:31000);
	
    #if USE_EVENTRESPONDER_PLAYWAV
    if (eventReadingEnabled)
        evResp.triggerEvent(0,this);   // do the read in the foreground: must delay() or yield()
    else
    #endif
		write(getBuffer(),sz_mem / 2); // write immediately
	
	firstHalf = !firstHalf; // switch interrupts to using the other half of the buffer memory
}

#if USE_EVENTRESPONDER_PLAYWAV
bool AudioBaseWav::eventReadingEnabled = false; //!< true to access filesystem in EventResponder, otherwise accesses happen under interrupt
#endif
/*
 * Initialise ready to move data from SD card to memory
 */
bool AudioBaseWav::initRead(File file)
{
	wavfile = file;
    startUsingSPI();
    #if USE_EVENTRESPONDER_PLAYWAV
	evResp.attach(evFuncRead);
    #endif
	return true;
}

/*
 * Initialise ready to move data from memory to SD card
 */
bool AudioBaseWav::initWrite(File file)
{
	wavfile = file;
    startUsingSPI();
    #if USE_EVENTRESPONDER_PLAYWAV
	evResp.attach(evFuncWrite);
    #endif
	return true;
}


void AudioBaseWav::close(bool closeFile) //!< close file, free up the buffer, detach responder
{
    bool irq = stopInt();

    if (wavfile)
    {
        if (closeFile) wavfile.close();
        stopUsingSPI();
    }

    if (nullptr != buf_unaligned)
    {
        SPTF("\r\Freed %d aligned bytes at %X - %X\r\n",sz_mem, buffer, buffer+sz_mem-1);
        free(buf_unaligned);
        buf_unaligned = buffer = nullptr;
		sz_mem = 0;
    }

    #if USE_EVENTRESPONDER_PLAYWAV
    evResp.clearEvent(); // not intuitive, but works SO much better...
    #endif

    startInt(irq);
}

int AudioBaseWav::calcBufPreload(int count, int last, int my)
{
  int c = -1;
  do {
    ++c;
    if (++last >= count) last = 0;
  } while (last != my);
  return c;
}

//----------------------------------------------------------------------------------------------------

// 8 bit unsigned:
__attribute__((hot)) static
size_t decode_8bit(int8_t buffer[], size_t buffer_rd, audio_block_t *queue[], const unsigned int channels)
{
    int8_t *p = &buffer[buffer_rd];

    size_t i = 0;
	switch (channels) {
		case 1:
			//todo: 32 bit reads
    do {
				queue[0]->data[i  ] = ( p[i  ] - 128 ) << 8; //8 bit fmt is unsigned
				queue[0]->data[i+1] = ( p[i+1] - 128 ) << 8;
				i += 2;
			} while (i < AUDIO_BLOCK_SAMPLES);
			break;

		case 2:
			//todo: 32 bit reads
			do {
				queue[0]->data[i] = ( *p++ - 128 ) << 8;
				queue[1]->data[i] = ( *p++ - 128 ) << 8;
			} while (++i < AUDIO_BLOCK_SAMPLES);
			break;

		default:
			do {
        unsigned int chan = 0;
        do {
					queue[chan]->data[i] = ( *p++ - 128 ) << 8;
        } while (++chan < channels);
    } while (++i < AUDIO_BLOCK_SAMPLES);
			break;
}
    return AUDIO_BLOCK_SAMPLES;
}


// 8 bit signed:
__attribute__((hot)) static
size_t decode_8bit_signed(int8_t buffer[], size_t buffer_rd, audio_block_t *queue[], const unsigned int channels)
{
    int8_t *p = &buffer[buffer_rd];

    size_t i = 0;
    do {
        unsigned int chan = 0;
        do {
            queue[chan]->data[i] = (*p++) << 8;
        } while (++chan < channels);
    } while (++i < AUDIO_BLOCK_SAMPLES);
    return AUDIO_BLOCK_SAMPLES;

}

//https://github.com/dpwe/dpwelib
const static short ulaw_decode[256] = {
    -32124, -31100, -30076, -29052, -28028, -27004, -25980, -24956,
    -23932, -22908, -21884, -20860, -19836, -18812, -17788, -16764,
    -15996, -15484, -14972, -14460, -13948, -13436, -12924, -12412,
    -11900, -11388, -10876, -10364,  -9852,  -9340,  -8828,  -8316,
     -7932,  -7676,  -7420,  -7164,  -6908,  -6652,  -6396,  -6140,
     -5884,  -5628,  -5372,  -5116,  -4860,  -4604,  -4348,  -4092,
     -3900,  -3772,  -3644,  -3516,  -3388,  -3260,  -3132,  -3004,
     -2876,  -2748,  -2620,  -2492,  -2364,  -2236,  -2108,  -1980,
     -1884,  -1820,  -1756,  -1692,  -1628,  -1564,  -1500,  -1436,
     -1372,  -1308,  -1244,  -1180,  -1116,  -1052,   -988,   -924,
      -876,   -844,   -812,   -780,   -748,   -716,   -684,   -652,
      -620,   -588,   -556,   -524,   -492,   -460,   -428,   -396,
      -372,   -356,   -340,   -324,   -308,   -292,   -276,   -260,
      -244,   -228,   -212,   -196,   -180,   -164,   -148,   -132,
      -120,   -112,   -104,    -96,    -88,    -80,    -72,    -64,
       -56,    -48,    -40,    -32,    -24,    -16,     -8,      0,
     32124,  31100,  30076,  29052,  28028,  27004,  25980,  24956,
     23932,  22908,  21884,  20860,  19836,  18812,  17788,  16764,
     15996,  15484,  14972,  14460,  13948,  13436,  12924,  12412,
     11900,  11388,  10876,  10364,   9852,   9340,   8828,   8316,
      7932,   7676,   7420,   7164,   6908,   6652,   6396,   6140,
      5884,   5628,   5372,   5116,   4860,   4604,   4348,   4092,
      3900,   3772,   3644,   3516,   3388,   3260,   3132,   3004,
      2876,   2748,   2620,   2492,   2364,   2236,   2108,   1980,
      1884,   1820,   1756,   1692,   1628,   1564,   1500,   1436,
      1372,   1308,   1244,   1180,   1116,   1052,    988,    924,
       876,    844,    812,    780,    748,    716,    684,    652,
       620,    588,    556,    524,    492,    460,    428,    396,
       372,    356,    340,    324,    308,    292,    276,    260,
       244,    228,    212,    196,    180,    164,    148,    132,
       120,    112,    104,     96,     88,     80,     72,     64,
	    56,     48,     40,     32,     24,     16,      8,      0 };

// 8 bit ulaw:
__attribute__((hot)) static
size_t decode_8bit_ulaw(int8_t buffer[], size_t buffer_rd, audio_block_t *queue[], const unsigned int channels)
{
    uint8_t *p = (uint8_t*)&buffer[buffer_rd];

    size_t i = 0;
	switch (channels) {
		case 1:
    do {
				queue[0]->data[i    ] = ulaw_decode[p[i    ]];
				queue[0]->data[i + 1] = ulaw_decode[p[i + 1]];
				i += 2;
			} while (i < AUDIO_BLOCK_SAMPLES);
			break;

		case 2:
			do {
				queue[0]->data[i] = ulaw_decode[*p++];
				queue[1]->data[i] = ulaw_decode[*p++];
			} while (++i < AUDIO_BLOCK_SAMPLES);
			break;

		default:
			do {
        unsigned int chan = 0;
        do {
            queue[chan]->data[i] = ulaw_decode[*p++];
        } while (++chan < channels);
    } while (++i < AUDIO_BLOCK_SAMPLES);
			break;
}
    return AUDIO_BLOCK_SAMPLES;
}

// 16 bit:
__attribute__((hot)) static
size_t decode_16bit(int8_t buffer[], size_t buffer_rd, audio_block_t *queue[], const unsigned int channels)
{

	switch(channels) {
		case 1:
			memcpy(&queue[0]->data[0], &buffer[buffer_rd], AUDIO_BLOCK_SAMPLES*2); //benchmak this
			break;
		case 2: {
			uint32_t sample;
    size_t i = 0;
			int32_t *p32 = (int32_t*) &buffer[buffer_rd];
    do {
				sample = *p32++;
				queue[0]->data[i] = sample;
				queue[1]->data[i] = sample >> 16;
			} while (++i < AUDIO_BLOCK_SAMPLES);
			break;
		}
		default: {
			size_t i = 0;
			int16_t *p = (int16_t*) &buffer[buffer_rd];
			do {
        unsigned int chan = 0;
        do {
            queue[chan]->data[i] = *p++;
        } while (++chan < channels);
    } while (++i < AUDIO_BLOCK_SAMPLES);
			break;
}
	}
    return AUDIO_BLOCK_SAMPLES * 2;
}

// 16 bit big endian:
#if !defined(KINETISL)
__attribute__((hot)) static
size_t decode_16bit_bigendian(int8_t buffer[], size_t buffer_rd, audio_block_t *queue[], const unsigned int channels)
{

    int16_t *p = (int16_t*) &buffer[buffer_rd];
    size_t i = 0;
    do {
        unsigned int chan = 0;
        do {
            queue[chan]->data[i] = __rev16(*p++);
        } while (++chan < channels);
    } while (++i < AUDIO_BLOCK_SAMPLES);
    return AUDIO_BLOCK_SAMPLES * 2;
}
#endif

static const _tEncoderDecoder decoders[APW_NONE] = {
	decode_8bit, decode_8bit_signed, decode_8bit_ulaw,
	decode_16bit, decode_16bit_bigendian};

// Todo:
//- upsampling (CMSIS?)
//- downsampling (CMSIS?) (is that really needed? pretty inefficient to load way more data than necessary and downsample then..
//- ...but may be useful for sample rate conversion or playback speed variation
//- play float formats?

//----------------------------------------------------------------------------------------------------
void AudioPlayWav::begin(void)
{
    state = STATE_STOP;
#if !defined(KINETISL)
    my_instance = _instances;
    ++_instances;
#endif // !defined(KINETISL)
}

void AudioPlayWav::end(void)
{
	stop();
#if !defined(KINETISL)
    --_instances;
#endif // !defined(KINETISL)
}

bool AudioPlayWav::play(File file, const bool paused)
{
    stop();
    if (!file) return false;

    initRead(file);

    if (!readHeader(APW_NONE, 0, 0,  paused ? STATE_PAUSED : STATE_RUNNING ))
    {
        stop();
        return false;
    }
    return true;
}

bool AudioPlayWav::play(const char *filename, const bool paused)
{
    stop();

    bool irq = stopInt();
    File file = SD.open(filename);
    startInt(irq);
    if (!file) return false;
    return play(file, paused);
}

bool AudioPlayWav::playRaw(File file, APW_FORMAT fmt, uint32_t sampleRate, uint8_t number_of_channels, bool paused)
{
    stop();

    initRead(file);

    if (!readHeader(fmt, sampleRate, number_of_channels, paused ? STATE_PAUSED : STATE_RUNNING ))
    {
        stop();
        return false;
    }
    return true;
}

bool AudioPlayWav::playRaw(const char *filename, APW_FORMAT fmt, uint32_t sampleRate, uint8_t number_of_channels, bool paused)
{
    stop();

    bool irq = stopInt();
    File file = SD.open(filename);
    startInt(irq);
    if (!file) return false;
    return playRaw(file, fmt, sampleRate, number_of_channels, paused);
}

void AudioPlayWav::stop(void)
{

    state = STATE_STOP;
	SPLN("\r\nSTOP!");
    bool irq = stopInt();
    close();
    startInt(irq);

}

//WAV:
/*
  00000000  52494646 66EA6903 57415645 666D7420  RIFFf.i.WAVEfmt
  00000010  10000000 01000200 44AC0000 10B10200  ........D.......
  00000020  04001000 4C495354 3A000000 494E464F  ....LIST:...INFO
  00000030  494E414D 14000000 49205761 6E742054  INAM....I Want T
  00000040  6F20436F 6D65204F 76657200 49415254  o Come Over.IART
  00000050  12000000 4D656C69 73736120 45746865  ....Melissa Ethe
  00000060  72696467 65006461 746100EA 69030100  ridge. a..i...
  00000070  FEFF0300 FCFF0400 FDFF0200 0000FEFF  ................
  00000080  0300FDFF 0200FFFF 00000100 FEFF0300  ................
  00000090  FDFF0300 FDFF0200 FFFF0100 0000FFFF  ................
*/

static const uint32_t cRIFF = 0x46464952; //'RIFF'
static const uint32_t cWAVE = 0x45564157; //'WAVE'
static const uint32_t cFMT  = 0x20746D66; //'fmt '
static const uint32_t cFACT = 0x74636166; //'fact'
static const uint32_t cDATA = 0x61746164; //'data'
static const char cGUID[14] = {'\x00','\x00','\x00','\x00','\x10','\x00',
                    '\x80','\x00','\x00','\xAA','\x00','\x38','\x9B','\x71'};


typedef struct {
  unsigned long id;
  unsigned long len;
  unsigned long riffType;
} PACKED tFileHeader;

// https://docs.microsoft.com/de-de/windows/win32/api/mmreg/ns-mmreg-waveformat
typedef struct
{
  //unsigned long  chunkID;
  //unsigned long  chunkSize;
  unsigned short  wFormatTag;
  unsigned short  nChannels;
  unsigned long   nSamplesPerSec;
  unsigned long   nAvgBytesPerSec;
  unsigned short  nBlockAlign;
} PACKED tFmtHeader;

// https://docs.microsoft.com/en-us/previous-versions/dd757713(v=vs.85)
typedef struct
{
  //unsigned long  chunkID;
  //unsigned long  chunkSize;
  unsigned short wFormatTag;
  unsigned short wChannels;
  unsigned long  dwSamplesPerSec;
  unsigned long  dwAvgBytesPerSec;
  unsigned short wBlockAlign;
  unsigned short wBitsPerSample;
  //unsigned short cbSize;
} PACKED tFmtHeaderEx;

// https://docs.microsoft.com/de-de/windows/win32/api/mmreg/ns-mmreg-waveformatextensible
typedef struct
{
	//unsigned long  chunkID;
  //unsigned long  chunkSize;
	//tFmtHeaderex fmtHeader;
  union {
    unsigned short wValidBitsPerSample;
    unsigned short wSamplesPerBlock;
    unsigned short wReserved;
  } PACKED;
  unsigned long dwChannelMask;
  struct {
    unsigned short  format;
    char guid[14];
  } PACKED;
} PACKED tFmtHeaderExtensible;

typedef struct
{
  unsigned long chunkID;
  unsigned long chunkSize;
} PACKED tDataHeader;


//AIFF, AIFC:
// http://www-mmsp.ece.mcgill.ca/Documents/AudioFormats/AIFF/Docs/AIFF-1.3.pdf
// http://www-mmsp.ece.mcgill.ca/Documents/AudioFormats/AIFF/Docs/AIFF-C.9.26.91.pdf

/*
 00000000  46 4F 52 4D 00 01 6F 94 41 49 46 46 43 4F 4D 4D  FORM..o”AIFFCOMM
 00000010  00 00 00 12 00 02 00 00 5B C5 00 10 40 0B FA 00  ........[Å..@.ú.
 00000020  00 00 00 00 00 00 41 4E 4E 4F 00 00 00 49 41 46  ......ANNO...IAF
 00000030  73 70 64 61 74 65 3A 20 32 30 30 33 2D 30 31 2D  spdate: 2003-01-
 00000040  33 30 20 30 33 3A 32 38 3A 33 36 20 55 54 43 00  30 03:28:36 UTC.
 00000050  75 73 65 72 3A 20 6B 61 62 61 6C 40 43 41 50 45  user: kabal@CAPE
 00000060  4C 4C 41 00 70 72 6F 67 72 61 6D 3A 20 43 6F 70  LLA.program: Cop
 00000070  79 41 75 64 69 6F 00 00 53 53 4E 44 00 01 6F 1C  yAudio..SSND..o.
*/

static const uint32_t cFORM = 0x4D524F46; //'FORM'
static const uint32_t cAIFF = 0x46464941; //'AIFF'
static const uint32_t cAIFC = 0x43464941; //'AIFC'
static const uint32_t cCOMM = 0x4D4D4F43; //'COMM'
static const uint32_t cSSND = 0x444e5353; //'SSND'
//static const uint32_t cULAW = 0x57414C55; //'ULAW' // not supported
static const uint32_t culaw = 0x77616c75; //'ulaw'
static const uint32_t craw =  0x20776172; //'raw '

typedef struct {
    short numChannels;
    unsigned long numSampleFrames;
    short sampleSize;
    uint8_t sampleRate[10]; //80  bit  IEEE  Standard  754  floating  point... ignore that!
} PACKED taiffCommonChunk;

typedef struct {
    short numChannels;
    unsigned long numSampleFrames;
    short sampleSize;
    uint8_t sampleRate[10]; //80  bit  IEEE  Standard  754  floating  point... ignore that!
    unsigned long compressionType;
} PACKED taifcCommonChunk;


bool AudioPlayWav::readHeader(APW_FORMAT fmt, uint32_t sampleRate, uint8_t number_of_channels, APW_STATE newState)
{
    size_t sz_frame, rd;
    tFileHeader fileHeader;
    tDataHeader dataHeader;
    bool irq;

    sample_rate = sampleRate;
    channels = number_of_channels;
    dataFmt = fmt;

    buffer_rd = total_length = data_length = 0;
    channelmask = bytes = 0;

    last_err = ERR_FILE;
    if (!wavfile) return false;

    irq = stopInt();
    seek(0);

    if ( dataFmt == APW_NONE) {
        rd = read(&fileHeader, sizeof(fileHeader));
        if (rd < sizeof(fileHeader)) { startInt(irq); return false; }
    } else {
         //just check if the file is readable:
        rd = read(&fileHeader, 1);
        if (rd < 1) { startInt(irq); return false; }
        seek(0);
    }
    startInt(irq);

    last_err = ERR_FORMAT;
    if ( dataFmt != APW_NONE) {
        // ---------- RAW ----------------------------
        //Serial.println("Format: RAW");
        total_length = size();
        if (total_length == 0) return false;
        }
    else
#if !defined(KINETISL)
    if (fileHeader.id == cFORM &&
       (fileHeader.riffType == cAIFF || fileHeader.riffType == cAIFC))
    {
        // ---------- AIFF ----------------------------
        //unlike wav, the samples chunk (here "SSND") can be everywhere in the file!
        //unfortunately, it is big endian :-(
        size_t position = sizeof(fileHeader);

        bool isAIFC = fileHeader.riffType == cAIFC;
        //if ( isAIFC ) Serial.println("AIFC"); else Serial.println("AIFF");
        bool COMMread = false;
        bool SSNDread = false;
		uint8_t bytes;

        do {
            irq = stopInt();
            if (!seek(position)) return false;
            rd = read(&dataHeader, sizeof(dataHeader));
            startInt(irq);
            dataHeader.chunkSize = __rev(dataHeader.chunkSize);
            if (rd < sizeof(dataHeader)) return false;
            //Serial.printf("Chunk:%c%c%c%c", (char)dataHeader.chunkID & 0xff, (char)(dataHeader.chunkID >> 8 & 0xff), (char)(dataHeader.chunkID  >> 16 & 0xff), (char)(dataHeader.chunkID >> 24 &0xff));
            //Serial.printf(" 0x%x size: %d\n",dataHeader.chunkID, dataHeader.chunkSize);
            if (!COMMread && dataHeader.chunkID == cCOMM) {
                //Serial.print(":COMM ");
                taifcCommonChunk commonChunk;
                rd = read(&commonChunk, sizeof(commonChunk));
                if (rd < sizeof(commonChunk)) return false;

                channels = __rev16(commonChunk.numChannels);
                commonChunk.sampleSize = __rev16 (commonChunk.sampleSize);
                bytes = commonChunk.sampleSize / 8;
                total_length = __rev(commonChunk.numSampleFrames) * channels * bytes;

                //Serial.printf("Channels:%d Length:%d Bytes:%d\n", (int)channels, (int)total_length, (int)bytes);
                if (total_length == 0) return false;
                if (commonChunk.sampleSize != 8 && commonChunk.sampleSize != 16) return false;

                //if (isAIFC) Serial.printf("Compression:%c%c%c%c 0x%x\n", (char)commonChunk.compressionType & 0xff, (char)(commonChunk.compressionType >> 8 & 0xff), (char)(commonChunk.compressionType  >> 16 & 0xff), (char)(commonChunk.compressionType >> 24 &0xff), commonChunk.compressionType);

                if (bytes == 2) {
                    if (isAIFC) return false;
                    dataFmt = APW_16BIT_SIGNED_BIGENDIAN;
                } else
                if (bytes == 1){
                    if (isAIFC) {
                        switch(commonChunk.compressionType)
                        {
                            case culaw: dataFmt = APW_ULAW;
                                        break;
                            case craw:  dataFmt = APW_8BIT_UNSIGNED;
                                        break;
                            default:    return false;
                        }
                    } else
                    dataFmt = APW_8BIT_SIGNED;
                } else return false;

                COMMread = true;
                if (SSNDread) break;

            } else if (dataHeader.chunkID == cSSND) {
                //todo: offset etc...

                //Serial.println(":SSND");
                SSNDread = true;
                if (COMMread) break;
            } ;

            position += sizeof(dataHeader) + dataHeader.chunkSize ;
            if (position & 1) position++; //make position even
        } while(true);

        if (!SSNDread || !COMMread) return false;

    }  // AIFF
	else
#endif

    if ( fileHeader.id == cRIFF &&
         fileHeader.riffType == cWAVE )
    {
        // ---------- WAV ----------------------------
        //Serial.println("Format: WAV");
        size_t position = sizeof(fileHeader);
        bool fmtok = false;
		uint8_t bytes;

        do {
            irq = stopInt();
            seek(position);
            rd = read(&dataHeader, sizeof(dataHeader));
            startInt(irq);

            if (rd < sizeof(dataHeader)) return false;

            if (dataHeader.chunkID == cFMT) {
                tFmtHeaderEx fmtHeader;
                memset((void*)&fmtHeader, 0, sizeof(tFmtHeaderEx));

                //Serial.println(dataHeader.chunkSize);
                irq = stopInt();
                if (dataHeader.chunkSize < 16) {
                    read(&fmtHeader, sizeof(tFmtHeader));
                    bytes = 1;
                } else if (dataHeader.chunkSize == 16) {
                    read(&fmtHeader, sizeof(tFmtHeaderEx));
                    bytes = fmtHeader.wBitsPerSample / 8;
                } else {
                    tFmtHeaderExtensible fmtHeaderExtensible;
                    read(&fmtHeader, sizeof(tFmtHeaderEx));
                    bytes = fmtHeader.wBitsPerSample / 8;
                    memset((void*)&fmtHeaderExtensible, 0, sizeof(fmtHeaderExtensible));
                    read(&fmtHeaderExtensible, sizeof(fmtHeaderExtensible));
                    channelmask = fmtHeaderExtensible.dwChannelMask;
                    //Serial.printf("channel mask: 0x%x\n", channelmask);
                }
                startInt(irq);

                //Serial.printf("Format:%d Bits:%d\n", fmtHeader.wFormatTag, fmtHeader.wBitsPerSample);
                if (fmtHeader.dwSamplesPerSec == 0) return false;
                sample_rate = fmtHeader.dwSamplesPerSec;
                channels = fmtHeader.wChannels;
				if (bytes == 1) dataFmt = APW_8BIT_UNSIGNED;
				else if (bytes == 2) dataFmt = APW_16BIT_SIGNED;
				else return false;
                if (fmtHeader.wFormatTag != 1 &&
                    fmtHeader.wFormatTag != 7 && //ulaw
                    fmtHeader.wFormatTag != 65534) return false;
                if (fmtHeader.wFormatTag == 7) {
                    if (bytes != 1) return false;
                    dataFmt = APW_ULAW; //ulaw
                    //Serial.println("ULAW!");
                }
                fmtok = true;
            }
            else
            if (dataHeader.chunkID == cDATA) {
                    total_length = dataHeader.chunkSize;
                    break;
            }

            position += sizeof(dataHeader) + dataHeader.chunkSize;
        } while(true);

        if (fmtok != true) return false;

    }  //wav
    else
        return false; //unknown format

    if (channels == 0 || channels > _AudioPlayWav_MaxChannels) return false;
    if (sample_rate == 0) sample_rate = AUDIO_SAMPLE_RATE_EXACT;
	if (dataFmt == APW_NONE) return false;

	bytes = bytesPerSample[dataFmt];
    sz_frame = AUDIO_BLOCK_SAMPLES * channels;
    data_length = total_length / (sz_frame * bytes);

    //calculate the needed buffer memory:
    size_t len;
	len = _instances * sz_frame * bytes;
    len *= _sz_mem_additional;

    //allocate: note this buffer pointer is temporary
    int8_t* buffer =  createBuffer( len * 2 );
	if (buffer == nullptr) {
		last_err = ERR_OUT_OF_MEMORY;
		return false;
	}

    last_err = ERR_OK;

	decoder = decoders[dataFmt];

	if (dataFmt == APW_8BIT_UNSIGNED)
		setPadding(0x80);
	else
		setPadding(0);

#if !defined(KINETISL)
    if (_instances > 1) {

        irq = stopInt();
		int load = calcBufPreload(_instances, _lastInstance, my_instance);
		buffer_rd = load * sz_frame * bytes; // pre-load according to instance number
		if (load > 0)
			read(&buffer[buffer_rd], getBufferSize() - buffer_rd);

        state = newState;
        startInt(irq);
    } else
        state = newState;
#else
    state = newState; //this *must* be the last instruction.
#endif // !defined(KINETISL)
    return true;
}


__attribute__((hot))
void  AudioPlayWav::update(void)
{
	_lastInstance = my_instance;
    if (state != STATE_RUNNING) return;

    unsigned int chan;

	// allocate the audio blocks to transmit
    audio_block_t *queue[channels];
    chan = 0;
    do {
		queue[chan] = AudioStream::allocate();
		if ( (queue[chan] == nullptr) )
		{
			for (unsigned int i = 0; i != chan; ++i)
				AudioStream::release(queue[i]);
			last_err = ERR_NO_AUDIOBLOCKS;
			SPLN("Waveplayer stopped: Not enough AudioMemory().");
			stop();
            return;
		}
	} while (++chan < channels);

	int8_t* buffer = getBuffer(); // buffer pointer: don't cache, could change in the future

	// copy the samples to the audio blocks:
    buffer_rd += decoder(buffer, buffer_rd, queue, channels) * channels;

    size_t sz_mem = getBufferSize() / 2; // double-buffered
    if (buffer_rd >= sz_mem ) {
        readLater();    // trigger buffer fill
        buffer_rd = 0;
    }

	// transmit them:
    chan = 0;
    do
    {
		AudioStream::transmit(queue[chan], chan);
		AudioStream::release(queue[chan]);
	} while (++chan < channels);


    --data_length;
	if (data_length <= 0) {
        SPLN("Stop: No Data anymore."); // if you care (why would you?), #define DEBUG_PRINT_PLAYWAV in play_wav.h
        stop(); // proper tidy up when playing is done
    }

}

#define _positionMillis() ((AUDIO_BLOCK_SAMPLES * 1000.0f / AUDIO_SAMPLE_RATE_EXACT) * (total_length / (bytes * channels * AUDIO_BLOCK_SAMPLES) - data_length))
#if !defined(KINETISL)
uint32_t AudioPlayWav::positionMillis(void)
{
    uint32_t safe_read, ret;
    //use an interrupt detector to make sure all vars are consistent.
    //strex will fail if an interrupt occured. An other way would be to block audio interrupts.
    do
    {
        __ldrexw(&safe_read);
        ret = _positionMillis();
	} while ( __strexw(1, &safe_read));

    return ret;
}


#else
uint32_t AudioPlayWav::positionMillis(void)
{
    bool irq;
    uint32_t ret;
    irq = stopInt();
    ret = _positionMillis();
    startInt(irq);
    return ret;
}

#endif // !defined(KINETISL)

//----------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------
#if !defined(KINETISL)

typedef struct
{
    tFileHeader fileHeader;
    struct {
        tDataHeader dataHeader;
        tFmtHeaderEx fmtHeader;
    } PACKED file;
} PACKED tWaveFileHeader;

typedef struct {
        tDataHeader dataHeader;
        uint32_t dwSampleLength;
} PACKED tfactChunk;


//----------------------------------------------------------------------------------------------------
// 8 bit unsigned:
__attribute__((hot)) static
size_t encode_8bit(int8_t buffer[], size_t buffer_rd, audio_block_t *queue[], const unsigned int channels)
{
    int8_t *p = &buffer[buffer_rd];
	size_t i = 0;

	switch (channels)
	{
		case 1:
			//todo: 32 bit
    do {
				*p++ = (queue[0]->data[i] >> 8) + 128; //8 bit fmt is unsigned
			} while (++i < AUDIO_BLOCK_SAMPLES);
			break;

		case 2:
			//todo: 32 bit
			do {
				*p++ = (queue[0]->data[i] >> 8) + 128;
				*p++ = (queue[1]->data[i] >> 8) + 128;
			} while (++i < AUDIO_BLOCK_SAMPLES);
			break;

		default:
			do {
        unsigned int chan = 0;
        do {
					*p++ = (queue[chan]->data[i] >> 8) + 128;
        } while (++chan < channels);
    } while (++i < AUDIO_BLOCK_SAMPLES);
			break;
	}
    return AUDIO_BLOCK_SAMPLES;

}
// 16 bit:
__attribute__((hot)) static
size_t encode_16bit(int8_t buffer[], size_t buffer_rd, audio_block_t *queue[], const unsigned int channels)
{
    int16_t *p = (int16_t*) &buffer[buffer_rd];
    size_t i = 0;
	switch (channels)
	{
		case 1:
			memcpy(p, &queue[0]->data[0], AUDIO_BLOCK_SAMPLES * 2);
			break;
		case 2:
			//todo: 32 bit
    do {
				*p++ = queue[0]->data[i];
				*p++ = queue[1]->data[i];
			} while (++i < AUDIO_BLOCK_SAMPLES);
			break;
		default:
			do {
        unsigned int chan = 0;
        do {
			*p++ = queue[chan]->data[i];
        } while (++chan < channels);
    } while (++i < AUDIO_BLOCK_SAMPLES);
			break;
}
    return AUDIO_BLOCK_SAMPLES * 2;
}
//----------------------------------------------------------------------------------------------------

void AudioRecordWav::begin(void)
{
    state = STATE_STOP;
    my_instance = _instances;
    ++_instances;
}

void AudioRecordWav::end(void)
{
	stop();
    --_instances;
}

void AudioRecordWav::stop(bool closeFile)
{
    state = STATE_STOP;
    SPLN("\r\nRecording STOP!");

    if (wavfile) writeHeader(wavfile);

    close(closeFile);

}

void AudioRecordWav::pause(const bool pause)
{
    bool irq = stopInt();
    if (pause && state != STATE_PAUSED) {
        if (wavfile) writeHeader(wavfile);
    }

    AudioBaseWav::pause(pause);
    startInt(irq);
}

bool AudioRecordWav::record(File file, APW_FORMAT fmt, unsigned int numchannels, bool paused)
{
    bool irq, ok;
    size_t wr;

    stop(false);

    if (!file) {
        last_err = ERR_FILE;
        return false;
    }

    last_err = ERR_FORMAT;
    if (numchannels == 0 || numchannels > _AudioRecordWav_MaxChannels) return false;

    if (fmt != APW_8BIT_UNSIGNED && fmt != APW_16BIT_SIGNED) return false;

    sample_rate = 0;
    sz_mem = sz_frame = buffer_wr = 0;
    data_length = data_length_old = 0;

    dataFmt = fmt;
	bytes = bytesPerSample[dataFmt];
    channels = numchannels;

    #if !defined(__IMXRT1062__)
    sample_rate = ((int)(AUDIO_SAMPLE_RATE_EXACT) / 20) * 20; //round (for Teensy 3.x)
    #else
    sample_rate = AUDIO_SAMPLE_RATE_EXACT;
    #endif


    switch(dataFmt)
    {
        case APW_8BIT_UNSIGNED:
            encoder = &encode_8bit;
            break;
        case APW_16BIT_SIGNED:
            encoder = &encode_16bit;
            break;
        default:
            return false;
    }

    initWrite(file);

    //write a dummy fileheader and check if it was successful:

	char tmp[512];
    memset(&tmp, 0xff, sizeof tmp);
    irq = stopInt();

    ok = seek(0);
    wr = write(&tmp, sizeof tmp );
    flush();
    startInt(irq);

    if (!ok || wr < sizeof tmp) {
        last_err = ERR_FILE;
        return false;
    }

    sz_frame = AUDIO_BLOCK_SAMPLES * channels;

    //calculate the needed buffer memory:

    size_t len;
	len = _instances * sz_frame * bytes;
    len *= _sz_mem_additional;

    //allocate: note this buffer pointer is temporary
    int8_t* buffer =  createBuffer( len * 2 ); // recording uses double-buffering
	if (buffer == nullptr) {
		last_err = ERR_OUT_OF_MEMORY;
		return false;
	}
	//Serial.printf("recbuf:%x len:%x\n",(intptr_t) buffer, sz_mem);
    buffer_wr = my_instance * sz_frame * bytes;
    //if (buffer_wr >= sz_mem) buffer_wr = 0;

    last_err = ERR_OK;
    state = paused? STATE_PAUSED : STATE_RUNNING;

    return true;
}

bool AudioRecordWav::record(const char *filename, APW_FORMAT fmt, unsigned int channels, bool paused)
{
    stop(true);

    bool irq = stopInt();
    File file = SD.open(filename, FILE_WRITE_BEGIN);
    startInt(irq);

    return record(file, fmt, channels, paused);

}

bool AudioRecordWav::writeHeader(File file)
{

    if (state == STATE_RUNNING) return false;
    if (data_length == data_length_old) return false;
    data_length_old = data_length;

    bool ok, irq;
    size_t pos, sz, wr, wrpos;
	char data[512];

    last_err = ERR_FILE;

    irq = stopInt();
    pos = position();

    if (pos > 0)
		ok = seek(0);
	else
		ok = true;
    startInt(irq);
    if (!ok)
		return false;


    const bool extended = false; //may be needed later.
    tWaveFileHeader header;
    tFmtHeaderExtensible headerextensible;
    tDataHeader dataHeader;

	wrpos = 0;

    size_t szh = sizeof header;
    if (extended) szh += sizeof headerextensible;

    sz = size();
    if (sz == 0) sz = szh;

    header.fileHeader.id = cRIFF;
    header.fileHeader.len = sz - 8;
    header.fileHeader.riffType = cWAVE;
    header.file.dataHeader.chunkID = cFMT;
    if (!extended)
    {
        header.file.dataHeader.chunkSize = sizeof header.file.fmtHeader;
        header.file.fmtHeader.wFormatTag = 1;
    }
    else
    {
         header.file.dataHeader.chunkSize = sizeof header.file.fmtHeader + sizeof headerextensible;
        header.file.fmtHeader.wFormatTag = 65534;
    }
    header.file.fmtHeader.wChannels = channels;
    header.file.fmtHeader.dwSamplesPerSec = sample_rate;
    header.file.fmtHeader.dwAvgBytesPerSec = sample_rate * bytes * channels;
    header.file.fmtHeader.wBlockAlign = bytes * channels;
    header.file.fmtHeader.wBitsPerSample = bytes * 8;
    //header.file.fmtHeader.cbSize = 0;

    irq = stopInt();
	memcpy(&data[wrpos], &header, sizeof header);
	wrpos += sizeof header;

    if (extended)
    {
        headerextensible.wValidBitsPerSample = bytes * 8;
        headerextensible.dwChannelMask = (1 << channels) - 1;
        headerextensible.format = 0x01;
        memcpy(headerextensible.guid, cGUID, sizeof headerextensible.guid);
		memcpy(&data[wrpos], &headerextensible, sizeof headerextensible);
		wrpos += sizeof headerextensible;
    }

	//fill in a dummy chunk to make sure the samples start @ position 512
	size_t toFill = 512 - szh - 2 * sizeof dataHeader;
	dataHeader.chunkID = 0x796d7564; //'dumy' chunk
	dataHeader.chunkSize = toFill;

	memcpy(&data[wrpos], &dataHeader, sizeof dataHeader);
	wrpos += sizeof dataHeader;
	memset(&data[wrpos], 0xff, toFill);
	wrpos += toFill;

	szh += sizeof dataHeader + toFill;


    dataHeader.chunkID = cDATA;
    dataHeader.chunkSize = sz - szh - sizeof dataHeader;
	memcpy(&data[wrpos], &dataHeader, sizeof dataHeader);
	wrpos += sizeof dataHeader;

	wr = write(&data, sizeof data);
    flush();
    if (pos > 0)
		ok = seek(pos);
	else
		ok = true;
    startInt(irq);
    if (!ok || wr < sizeof dataHeader)
		return false;

    last_err = ERR_OK;

    return true;
}

__attribute__((hot))
void  AudioRecordWav::update(void)
{
    if (STATE_ABORT == state) // event has caused us to abort
		stop();
	
    if (state != STATE_RUNNING) return;

	unsigned int chan;

    chan = 0;
    do
    {
        audio_block_t *q;
		q = AudioStream::receiveReadOnly(chan);
        if (!q) // we've been passed NULL, this means...
			q = (audio_block_t*)&zeroblock; // ... data of all zeros: we NEED this!
        queue[chan] = q;
	} while (++chan < channels);

    buffer_wr += encoder(getBuffer(), buffer_wr, queue, channels) * channels;
	size_t sz_mem = getBufferSize() / 2; // recording uses double-buffering
    if (buffer_wr >= sz_mem)
    {
        buffer_wr = 0;
		writeLater(); // may be immediate if not using EventResponder
    }

    ++data_length;

	// release queues
    chan = 0;
    do
    {
        if (queue[chan] != nullptr
		 &&	queue[chan] != (audio_block_t*)&zeroblock) // don't release our block of zeroes!
            AudioStream::release(queue[chan]);
        queue[chan] = nullptr; // < why is this needed?
	} while (++chan < channels);

}

#endif // !defined(KINETISL)