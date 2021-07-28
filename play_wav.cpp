/* Audio Library for Teensy
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
*/

// (c) Frank Bösing, 07/2021

#include <Arduino.h>
#include "play_wav.h"
#include "spi_interrupt.h"

#define STATE_STOP    0
#define STATE_PAUSED  1
#define STATE_PLAY    2


static short _AudioPlayWavInstances = 0;
static short _AudioPlayWavInstance = -1;


FLASHMEM
void AudioPlayWav::begin(void)
{
    my_instance = _AudioPlayWavInstances;
    ++_AudioPlayWavInstances;
}

bool AudioPlayWav::play(File file)
{
    return play(file, false);
}

bool AudioPlayWav::play(File file, const bool paused)
{

    stop();

    bool irq = false;
    if (NVIC_IS_ENABLED(IRQ_SOFTWARE)) {
        NVIC_DISABLE_IRQ(IRQ_SOFTWARE);
        irq = true;
    }

    startUsingSPI();

    bool ret;
    wavfile = file;
    if (!readHeader())
    {
        stop();
        ret = false;
    } else {
        state = paused ? STATE_PAUSED : STATE_PLAY;
        ret = true;
    }

    if (irq) NVIC_ENABLE_IRQ(IRQ_SOFTWARE);
    last_err = APW_ERR_OK;
    return ret;

}

bool AudioPlayWav::play(const char *filename)
{
    return play(filename, false);
}

bool AudioPlayWav::play(const char *filename, const bool paused)
{
    stop();

    bool irq = false;
    if (NVIC_IS_ENABLED(IRQ_SOFTWARE)) {
        NVIC_DISABLE_IRQ(IRQ_SOFTWARE);
        irq = true;
    }

    startUsingSPI();

    wavfile = SD.open(filename);

    bool ret;
    if (!readHeader())
    {
        stop();
        ret = false;
    } else {
        state = paused ? STATE_PAUSED : STATE_PLAY;
        ret = true;
    }
    if (irq) NVIC_ENABLE_IRQ(IRQ_SOFTWARE);
    last_err = APW_ERR_OK;
    return ret;
}

void AudioPlayWav::stop(void)
{
    bool irq = false;
    if (NVIC_IS_ENABLED(IRQ_SOFTWARE)) {
        NVIC_DISABLE_IRQ(IRQ_SOFTWARE);
        irq = true;
    }

    if (wavfile) wavfile.close();

    stopUsingSPI();

    if (buffer) {
        free(buffer);
        buffer = nullptr;
    }
    data_length = 0;
    state = STATE_STOP;

    if (irq) NVIC_ENABLE_IRQ(IRQ_SOFTWARE);
}

__attribute__((hot))
void  AudioPlayWav::update(void)
{

	if (++_AudioPlayWavInstance >= _AudioPlayWavInstances)
        _AudioPlayWavInstance = 0;

    if ( state != STATE_PLAY ) return;


	if (buffer_rd == sz_mem - sz_frame && _AudioPlayWavInstance == my_instance)
	{
        size_t rd = wavfile.read( buffer, sz_mem);
        
        //at EOF, fill remaining space:
        if (rd < sz_mem) {                
            int len = sz_frame - rd;            
            if (len > 0) {
                //prevent "plopp" for 8 bit (8 bit fmt is unsigned)
                memset( &buffer[rd], (bytes == 1) ? 128:0 , len);
            }
            data_length = 1;
        }
	}


	++blocks_played;


	// allocate the audio blocks to transmit
	for (unsigned chan = 0; chan < channels; chan++) {
		queue[chan] = AudioStream::allocate();
		if (queue[chan] == nullptr) {
			for (unsigned i = 0; i < chan; i++) AudioStream::release(queue[i]);
			last_err = APW_ERR_NO_AUDIOBLOCKS;
			//Serial.println("Waveplayer stopped: not enough AudioMemory().");
			goto end;
		}
	}


	// copy the samples to the audio blocks:
	if (bytes == 2) {

		// 16 bits:

        _wpsample_t *p = &buffer[buffer_rd];
        buffer_rd += sz_frame;
        if (buffer_rd * sizeof(_wpsample_t) > sz_mem) buffer_rd = 0;

        unsigned i = 0;
        do {
            unsigned chan = 0;
            do {
                queue[chan]->data[i] = *p++;                
            } while (++chan < channels);
                        
            chan = 0;
            do {
                queue[chan]->data[i + 1] = *p++;                
            } while (++chan < channels);
            
            i+=2;            
            
        } while (i < AUDIO_BLOCK_SAMPLES);


	} else {

		// 8 bits
		int8_t *p = (int8_t*) &buffer[buffer_rd];
		buffer_rd += sz_frame;
		if (buffer_rd >= sz_mem) buffer_rd = 0;

		unsigned i = 0;
		do {

			unsigned chan = 0;
			do {
				queue[chan]->data[i] = ( *p++ - 128 ) << 8; //8 bit fmt is unsigned
			} while (++chan < channels);

			chan = 0;
			do {
				queue[chan]->data[i + 1] = ( *p++ - 128 ) << 8;
			} while (++chan < channels);

			i += 2;
		} while (i < AUDIO_BLOCK_SAMPLES);

	}


	// transmit them:
	for (unsigned chan = 0; chan < channels; chan++) {
		AudioStream::transmit(queue[chan], chan);
		AudioStream::release(queue[chan]);
	}

	if (--data_length > 0) return;

end:  // end of file reached or other reason to stop
    stop();
    return;
}

inline void AudioPlayWav::startUsingSPI(void)
{

#if defined(HAS_KINETIS_SDHC)
    if (!(SIM_SCGC3 & SIM_SCGC3_SDHC)) AudioStartUsingSPI();
#else
    AudioStartUsingSPI();
#endif
}

inline void AudioPlayWav::stopUsingSPI(void)
{
#if defined(HAS_KINETIS_SDHC)
    if (!(SIM_SCGC3 & SIM_SCGC3_SDHC)) AudioStopUsingSPI();
#else
    AudioStopUsingSPI();
#endif
}

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
static const uint32_t cDATA = 0x61746164; //'data'

typedef struct {
  unsigned long id;
  unsigned long len;
  unsigned long riffType;
} tFileHeader;

// https://docs.microsoft.com/de-de/windows/win32/api/mmreg/ns-mmreg-waveformat
typedef struct __attribute__ ((__packed__))
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
	unsigned short cbSize;
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
  } Samples;
  unsigned long        dwChannelMask;
  //GUID         SubFormat;
} __attribute__ ((__packed__)) tFmtHeaderExtensible;

typedef struct {
  unsigned long chunkID;
  unsigned long chunkSize;
} tDataHeader;


bool AudioPlayWav::readHeader()
{
    size_t position, rd;
    tFileHeader fileHeader;
    tFmtHeaderEx fmtHeader;
    tFmtHeaderExtensible fmtHeaderExtensible;
    tDataHeader dataHeader;
    bool fmtok;

    if (buffer) {
        free(buffer);
        buffer = nullptr;
    }

    buffer_rd = total_length = data_length = 0;
    channelmask = sample_rate = channels = bytes = blocks_played = 0;
    
    last_err = APW_ERR_FILE;
    if (!wavfile) return false;

    memset((void*)&fileHeader, 0, sizeof(fileHeader));

    rd = wavfile.read(&fileHeader, sizeof(fileHeader));
    if (rd < sizeof(fileHeader)) return false;
    
    last_err = APW_ERR_FORMAT;
	if ( fileHeader.id != cRIFF || fileHeader.riffType != cWAVE ) return false;

    memset((void*)&fmtHeader, 0, sizeof(fmtHeader));
    memset((void*)&fmtHeaderExtensible, 0, sizeof(fmtHeaderExtensible));
    memset((void*)&dataHeader, 0, sizeof(dataHeader));

	position = sizeof(fileHeader);
    fmtok = false;

    while(true) {

        rd = wavfile.read(&dataHeader, sizeof(dataHeader));
        if (rd < sizeof(dataHeader)) return false;

        if (dataHeader.chunkID == cFMT) {
                //Serial.println(dataHeader.chunkSize);

            if (dataHeader.chunkSize < 16) {
                wavfile.read(&fmtHeader, sizeof(tFmtHeader));
                bytes = 1;
            } else if (dataHeader.chunkSize == 16) {
                wavfile.read(&fmtHeader, sizeof(tFmtHeaderEx));
                bytes = fmtHeader.wBitsPerSample / 8;
            } else {
                wavfile.read(&fmtHeader, sizeof(tFmtHeaderEx));
                bytes = fmtHeader.wBitsPerSample / 8;
                rd = wavfile.read(&fmtHeaderExtensible, sizeof(fmtHeaderExtensible));
                channelmask = fmtHeaderExtensible.dwChannelMask;
                //Serial.printf("channel mask: 0x%x\n", channelmask);
            }

            //Serial.printf("Format:%d Bits:%d\n", fmtHeader.wFormatTag, fmtHeader.wBitsPerSample);            
            sample_rate = fmtHeader.dwSamplesPerSec;
            channels = fmtHeader.wChannels;
            sz_frame = AUDIO_BLOCK_SAMPLES * channels * bytes;
            if (sz_frame == 0) return false;
            if (bytes > 2) return false;
            if (channels > _AudioPlayWav_MaxChannels) return false;
            if (fmtHeader.wFormatTag != 1 && fmtHeader.wFormatTag != 65534) return false;
            fmtok = true;
        }
        else if (dataHeader.chunkID == cDATA) break;

        position += sizeof(dataHeader) + dataHeader.chunkSize;
        if (!wavfile.seek(position)) {
            last_err = APW_ERR_FILE;
            return false;
        }

    };

    if (fmtok != true) return false;
    last_err = APW_ERR_OK;

    //Calculate the needed memory:
    // #of instances * #channels * #bytes per sample * Audio block size, 512 Bytes minimum
    size_t sz = _AudioPlayWavInstances * sz_frame;
    sz_mem = sz;
    sz_mem += sz_mem_additional & ~(sz - 1);
    while (sz_mem < 512) sz_mem += sz;

    buffer = (_wpsample_t*) malloc(sz_mem);
    if (buffer == nullptr) {
		last_err = APW_ERR_OUT_OF_MEMORY;
		return false;
	}

    total_length = dataHeader.chunkSize;
	data_length = dataHeader.chunkSize / sz_frame;

    return true;
}

bool AudioPlayWav::addMemoryForRead(size_t bytes)
{
	sz_mem_additional = bytes;
	return true;
}

bool AudioPlayWav::isPlaying(void)
{
    return state == STATE_PLAY;
}

void AudioPlayWav::togglePlayPause(void)
{
    if (state == STATE_STOP) return;
    state = (state == STATE_PLAY) ? STATE_PAUSED : STATE_PLAY;
}

void AudioPlayWav::pause(const bool pause)
{
    if (state == STATE_STOP) return;
    state = (pause) ? STATE_PAUSED : STATE_PLAY;
}

bool AudioPlayWav::isPaused(void)
{
    return (state == STATE_PAUSED);
}


bool AudioPlayWav::isStopped(void)
{
    return (state == STATE_STOP);
}


uint32_t AudioPlayWav::positionMillis(void)
{
    return (AUDIO_BLOCK_SAMPLES * 1000.0f / AUDIO_SAMPLE_RATE_EXACT) * blocks_played;
}


uint32_t AudioPlayWav::lengthMillis(void)
{
    return total_length * (1000.0f / AUDIO_SAMPLE_RATE_EXACT);
}

uint32_t AudioPlayWav::numBits(void)
{
    return bytes * 8;
};

uint32_t AudioPlayWav::numChannels(void)
{
    return channels;
}

uint32_t AudioPlayWav::sampleRate(void)
{
    return sample_rate;
}

uint32_t AudioPlayWav::channelMask(void)
{
	return channelmask;
}

uint8_t AudioPlayWav::lastErr(void)
{
	return last_err;
}

size_t AudioPlayWav::memUsed(void)
{
	return sz_mem;
}

uint8_t AudioPlayWav::instance(void)
{
	return my_instance;
}