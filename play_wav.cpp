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
static const uint8_t _AudioPlayWavInstances = 1;
static const uint8_t _sz_mem_additional = 1;
#if AUDIO_BLOCK_SAMPLES < 128
//#warning WavePlay: AUDIO_BLOCK_SAMPLES is less than 128. Expect noise.
#endif // AUDIO_BLOCK_SAMPLES < 128
#else
static uint8_t _AudioPlayWavInstances = 0;
static uint8_t _AudioRecordWavInstances = 0;
static uint8_t _sz_mem_additional = 1;
static const audio_block_t zeroblock = {0};
#endif // defined(KINETISL)


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
    yield();
    bool irq = stopInt();
    bool s = state == STATE_RUNNING;
    startInt(irq);
    return s;
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
    sz_mem = len;

    #if defined(__IMXRT1062__)
    buf_unaligned = malloc(sz_mem + 31);
    buffer = (int8_t*)(((uintptr_t)(buf_unaligned) + 31) & ~31);
    #else
    buf_unaligned = malloc(sz_mem);
    buffer = (int8_t*) buf_unaligned;
    #endif
    SPTF("Allocated %d aligned bytes at %X - %X\r\n",sz_mem, buffer, buffer+sz_mem-1);
    //for (size_t i=0;i<len/2;i++) *((int16_t*) buffer+i) = i * 30000 / len;
    return buffer;
}

#if USE_EVENTRESPONDER_PLAYWAV
void AudioBaseWav::evFuncRead(EventResponderRef ref) //!< foreground: respond to request to load WAV data
{
    AudioBaseWav& thisWM = *(AudioBaseWav*) ref.getData();

    SPRT("*** ");
    thisWM.read(thisWM.buffer,thisWM.sz_mem);
}

void AudioBaseWav::evFuncWrite(EventResponderRef ref) //!< foreground: respond to request to save WAV data
{
    AudioBaseWav& thisWM = *(AudioBaseWav*) ref.getData();

    SPRT("*** ");
    thisWM.write(thisWM.buffer,thisWM.sz_mem);
}
#endif

void AudioBaseWav::readLater(void) //!< from interrupt: request to re-fill the buffer
{
    #if USE_EVENTRESPONDER_PLAYWAV
    if (eventReadingEnabled)
        evResp.triggerEvent(0,this); // do the read in the foreground: must delay() or yield()
    else
    #endif
        read(buffer,sz_mem);		 // read immediately

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
    #if USE_EVENTRESPONDER_PLAYWAV
    if (eventReadingEnabled)
        evResp.triggerEvent(0,this); // do the read in the foreground: must delay() or yield()
    else
    #endif
        write(buffer,sz_mem);		 // write immediately

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

void AudioBaseWav::close() //!< close file, free up the buffer, detach responder
{
    bool irq = stopInt();

    if (wavfile)
    {
        wavfile.close();
        stopUsingSPI();
    }

    if (nullptr != buf_unaligned)
    {
        SPTF("\r\Freed %d aligned bytes at %X - %X\r\n",sz_mem, buffer, buffer+sz_mem-1);
        free(buf_unaligned);
        buf_unaligned = buffer = nullptr;
    }

    #if USE_EVENTRESPONDER_PLAYWAV
    evResp.clearEvent(); // not intuitive, but works SO much better...
    #endif

    startInt(irq);
}
//----------------------------------------------------------------------------------------------------

// 8 bit unsigned:
__attribute__((hot)) static
size_t decode_8bit(int8_t buffer[], size_t buffer_rd, audio_block_t *queue[], const unsigned int channels)
{
    int8_t *p = &buffer[buffer_rd];

    size_t i = 0;
    do {
        unsigned int chan = 0;
        do {
            queue[chan]->data[i] = ( *p++ - 128 ) << 8; //8 bit fmt is unsigned
        } while (++chan < channels);
    } while (++i < AUDIO_BLOCK_SAMPLES);
    return p - buffer;

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
    return p - buffer;

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
    do {
        unsigned int chan = 0;
        do {
            queue[chan]->data[i] = ulaw_decode[*p++];
        } while (++chan < channels);
    } while (++i < AUDIO_BLOCK_SAMPLES);
    return (int8_t*)p - buffer;
}

// 16 bit:
__attribute__((hot)) static
size_t decode_16bit(int8_t buffer[], size_t buffer_rd, audio_block_t *queue[], const unsigned int channels)
{

    int16_t *p = (int16_t*) &buffer[buffer_rd];
    size_t i = 0;
    do {
        unsigned int chan = 0;
        do {
            queue[chan]->data[i] = *p++;
        } while (++chan < channels);
    } while (++i < AUDIO_BLOCK_SAMPLES);
    return (int8_t*)p - buffer;
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
    return (int8_t*)p - buffer;

}
#endif

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
    my_instance = _AudioPlayWavInstances;
    ++_AudioPlayWavInstances;
#endif // !defined(KINETISL)
}

void AudioPlayWav::end(void)
{
	stop();
#if !defined(KINETISL)
    --_AudioPlayWavInstances;
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
} tFileHeader;

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
} __attribute__ ((__packed__)) tFmtHeader;

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
} __attribute__ ((__packed__)) tFmtHeaderEx;

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
  };
  unsigned long dwChannelMask;
  struct {
    unsigned short  format;
    char guid[14];
  } __attribute__ ((__packed__));
} __attribute__ ((__packed__)) tFmtHeaderExtensible;

typedef struct {
  unsigned long chunkID;
  unsigned long chunkSize;
} __attribute__ ((__packed__)) tDataHeader;


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
}  __attribute__ ((__packed__)) taiffCommonChunk;

typedef struct {
    short numChannels;
    unsigned long numSampleFrames;
    short sampleSize;
    uint8_t sampleRate[10]; //80  bit  IEEE  Standard  754  floating  point... ignore that!
    unsigned long compressionType;
}  __attribute__ ((__packed__)) taifcCommonChunk;


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
        switch (dataFmt)
        {
            case APW_NONE: //to prevent a GCC warning..
                break;
            case APW_8BIT_UNSIGNED:
            case APW_8BIT_SIGNED:
            case APW_ULAW:
                bytes = 1;
                break;
            case APW_16BIT_SIGNED:
            case APW_16BIT_SIGNED_BIGENDIAN:
                bytes = 2;
                break;
        }
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
                    dataFmt = APW_16BIT_SIGNED; //16 Bit signed
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
                if (bytes == 0 || bytes > 2) return false;
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

    sz_frame = AUDIO_BLOCK_SAMPLES * channels;
    data_length = total_length / (sz_frame * bytes);

    //calculate the needed buffer memory:
    size_t sz_mem = _AudioPlayWavInstances * sz_frame * bytes;
    sz_mem *= _sz_mem_additional;

    //allocate: note this buffer pointer is temporary
    int8_t* buffer =  createBuffer( sz_mem );
	if (buffer == nullptr) {
        sz_mem = 0;
		last_err = ERR_OUT_OF_MEMORY;
		return false;
	}

    last_err = ERR_OK;

    setPadding(0);

    switch(bytes) {
        default:
        case 1: switch (dataFmt) {
                    default:
                    case APW_8BIT_UNSIGNED:
                            decoder = &decode_8bit;
                            setPadding(128);
                            break;
                    case APW_8BIT_SIGNED:
                            decoder = &decode_8bit_signed;
                            break;
                    case APW_ULAW:
                            decoder = &decode_8bit_ulaw;
                            break;
                }
                break;

        case 2: switch (dataFmt) {
                    default:
                    case APW_16BIT_SIGNED:
                            decoder = &decode_16bit;
                            break;
                    #if !defined(KINETISL)
                    case APW_16BIT_SIGNED_BIGENDIAN:
                            decoder = &decode_16bit_bigendian;
                            break;
                    #endif
                }
                break;
    }

#if !defined(KINETISL)
    if (_AudioPlayWavInstances > 1) {
        //For sync start, and to start immediately:

        irq = stopInt();

		// Simplified calculation of how much buffer to pre-load, from all down
		// to 1/Nth depending on the instance number. This sort of works, but
		// (a) can be sub-optimal if started in paused mode then un-paused at EXACTLY the
		// wrong times, (b) isn't great if additional AudioPlayWav objects are created or
		// are deleted (which will happen with dynamic audio objects)

		buffer_rd = my_instance*(sz_frame * bytes); // pre-load according to instance number
        read(&buffer[buffer_rd], sz_mem - buffer_rd);
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
    if (state != STATE_RUNNING) return;

    unsigned int chan;

	// allocate the audio blocks to transmit
    audio_block_t *queue[channels];
    chan = 0;
    do {
		queue[chan] = AudioStream::allocate();
		if ( (queue[chan] == nullptr) ) {
			for (unsigned int i = 0; i != chan; ++i) AudioStream::release(queue[i]);
			last_err = ERR_NO_AUDIOBLOCKS;
			SPLN("Waveplayer stopped: Not enough AudioMemory().");
			stop();
            return;
		}
	} while (++chan < channels);

	int8_t* buffer = getBuffer(); // buffer pointer: don't cache, could change in the future
    //if (nullptr == currentPos) return; //This WILL NOT happen. IF it happens, let it crash for easier debug.

	// copy the samples to the audio blocks:
    buffer_rd = decoder(buffer, buffer_rd, queue, channels);

    size_t sz_mem = getBufferSize();
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
        SPLN("Stop: No Data anymore.");
        stop();
    }
}

#define _positionMillis() ((AUDIO_BLOCK_SAMPLES * 1000.0f / AUDIO_SAMPLE_RATE_EXACT) * (total_length / (bytes * channels * AUDIO_BLOCK_SAMPLES) - data_length))

#if !defined(KINETISL)
__attribute__( ( always_inline ) ) static inline uint32_t __ldrexw(volatile uint32_t *addr)
{
   uint32_t result;
   asm volatile ("ldrex %0, [%1]" : "=r" (result) : "r" (addr) );
   return(result);
}


__attribute__( ( always_inline ) ) static inline uint32_t __strexw(uint32_t value, volatile uint32_t *addr)
{
   uint32_t result;
   asm volatile ("strex %0, %2, [%1]" : "=&r" (result) : "r" (addr), "r" (value) );
   return(result);
}


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
        //tFmtHeader fmtHeader;
        tFmtHeaderEx fmtHeader;
    } __attribute__ ((__packed__)) file;
} __attribute__ ((__packed__)) tWaveFileHeader;

typedef struct {
        tDataHeader dataHeader;
        uint32_t dwSampleLength;
} __attribute__ ((__packed__)) tfactChunk;


//----------------------------------------------------------------------------------------------------
// 8 bit unsigned:
__attribute__((hot)) static
size_t encode_8bit(int8_t buffer[], size_t buffer_rd, audio_block_t *queue[], const unsigned int channels)
{
    int8_t *p = &buffer[buffer_rd];

    size_t i = 0;
    do {
        unsigned int chan = 0;
        do {
            *p++ = (queue[chan]->data[i] >> 8) + 128; //8 bit fmt is unsigned
        } while (++chan < channels);
    } while (++i < AUDIO_BLOCK_SAMPLES);
    return p - buffer;

}
// 16 bit:
__attribute__((hot)) static
size_t encode_16bit(int8_t buffer[], size_t buffer_rd, audio_block_t *queue[], const unsigned int channels)
{

    int16_t *p = (int16_t*) &buffer[buffer_rd];
    size_t i = 0;
    do {
        unsigned int chan = 0;
        do {
            *p++ = queue[chan]->data[i];
        } while (++chan < channels);
    } while (++i < AUDIO_BLOCK_SAMPLES);
    return (int8_t*)p - buffer;
}
//----------------------------------------------------------------------------------------------------

void AudioRecordWav::begin(void)
{
    state = STATE_STOP;
    my_instance = _AudioRecordWavInstances;
    ++_AudioRecordWavInstances;
}

void AudioRecordWav::end(void)
{
	stop();
    --_AudioRecordWavInstances;
}

void AudioRecordWav::stop(bool closeFile)
{
    state = STATE_STOP;
    SPLN("\r\nRecording STOP!");

    if (wavfile) writeHeader(wavfile);

    bool irq = stopInt();
    if (closeFile) close();
    startInt(irq);

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
    tWaveFileHeader fileHeader;
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

    sz_mem = sz_frame = bytes = sample_rate = 0;
    data_length = data_length_old = 0;

    dataFmt = fmt;
    channels = numchannels;

    #if !defined(__IMXRT1062__)
    sample_rate = ((int)(AUDIO_SAMPLE_RATE_EXACT) / 20) * 20; //round (for Teensy 3.x)
    #else
    sample_rate = AUDIO_SAMPLE_RATE_EXACT;
    #endif

    switch(dataFmt)
    {
        case APW_8BIT_UNSIGNED:
            bytes = 1;
            encoder = &encode_8bit;
            break;
        case APW_16BIT_SIGNED:
            bytes = 2;
            encoder = &encode_16bit;
            break;
        default:
            return false;
    }

    initWrite(file);

    //write a dummy fileheader and check if it was successful:
    memset(&fileHeader, 0xff, sizeof(fileHeader));
    irq = stopInt();

    ok = seek(0);
    wr = write(&fileHeader, sizeof(fileHeader));
    flush();
    startInt(irq);

    if (!ok || wr < sizeof(fileHeader)) {
        last_err = ERR_FILE;
        return false;
    }

    sz_frame = AUDIO_BLOCK_SAMPLES * channels;

    //calculate the needed buffer memory:
    sz_mem = _AudioRecordWavInstances * sz_frame * bytes;
    sz_mem *= _sz_mem_additional;

    //allocate: note this buffer pointer is temporary
    /* todo: use that later
    int8_t* buffer =  createBuffer( sz_mem );
	if (buffer == nullptr) {
        sz_mem = 0;
		last_err = ERR_OUT_OF_MEMORY;
		return false;
	}
    */
    buffer_wr = 0;
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
    size_t pos, sz, wr;

    last_err = ERR_FILE;

    irq = stopInt();
    pos = position();
//    flush(); //TODO: is a flush needed?
    if (pos > 0) ok = seek(0); else ok = true;
    startInt(irq);
    if (!ok) return false;

    
    bool extended = false;

    size_t szh = sizeof(tWaveFileHeader);
    if (extended) szh += sizeof(tFmtHeaderExtensible);
    
    
    sz = size();
    if (sz == 0) sz = szh;

    tWaveFileHeader header;
    tFmtHeaderExtensible headerextensible;
    tDataHeader dataHeader;

    header.fileHeader.id = cRIFF;
    header.fileHeader.len = sz - 8;
    header.fileHeader.riffType = cWAVE;
    header.file.dataHeader.chunkID = cFMT;
    if (!extended) 
    {
        header.file.dataHeader.chunkSize = sizeof(header.file.fmtHeader);
        header.file.fmtHeader.wFormatTag = 1;
    }
    else
    {
         header.file.dataHeader.chunkSize = sizeof(header.file.fmtHeader) + sizeof(headerextensible);
        header.file.fmtHeader.wFormatTag = 65534;
    }
    header.file.fmtHeader.wChannels = channels;
    header.file.fmtHeader.dwSamplesPerSec = sample_rate;
    header.file.fmtHeader.dwAvgBytesPerSec = sample_rate * bytes * channels;
    header.file.fmtHeader.wBlockAlign = bytes * channels;
    header.file.fmtHeader.wBitsPerSample = bytes * 8;
    //header.file.fmtHeader.cbSize = 0;

    irq = stopInt();
    write(&header, sizeof(header));
#if 0
    if (extended)
    {
        headerextensible.wValidBitsPerSample = bytes * 8;
        headerextensible.dwChannelMask = (1 << channels) - 1;
        headerextensible.format = 0x01;
        memcpy(headerextensible.guid, cGUID, sizeof(headerextensible.guid));
        write(&headerextensible, sizeof(headerextensible));
    }
#endif
    dataHeader.chunkID = cDATA;
    dataHeader.chunkSize = sz - szh;
    wr = write(&dataHeader, sizeof(dataHeader));
    flush();
    if (pos > 0) ok = seek(pos); else ok = true;
    startInt(irq);
    if (!ok || wr < sizeof(dataHeader)) return false;

    last_err = ERR_OK;

    return true;
}

__attribute__((hot))
void  AudioRecordWav::update(void)
{

    unsigned int chan;
    size_t wr;
    size_t sz = sz_frame * bytes;
    int8_t buf[sz]; //todo: remove that later and use malloc'd mem

    chan = 0;
    do
    {
        audio_block_t *q;
		q = AudioStream::receiveReadOnly(chan);
        if (!q) q = (audio_block_t*)&zeroblock;
        queue[chan] = q;
	} while (++chan < _AudioRecordWav_MaxChannels);

    if (state != STATE_RUNNING) goto noRecording;

    buffer_wr = encoder(buf, buffer_wr, queue, channels);

    buffer_wr = 0; //todo

    if (data_length == 1 || buffer_wr == 0) {
        wr = write(&buf[buffer_wr], sz - buffer_wr);
        buffer_wr = 0;
        if (wr < sz) {
            stop(false); //Disk full, max filesize reached, SD Card removed etc.
            last_err = ERR_FILE;
        }
    }

    ++data_length;

noRecording:
	// release queues
    chan = 0;
    do
    {
        if (queue[chan] != (audio_block_t*)&zeroblock)
            AudioStream::release(queue[chan]);
        queue[chan] = nullptr; // < why is this needed?
	} while (++chan < _AudioRecordWav_MaxChannels);

}

#endif // !defined(KINETISL)