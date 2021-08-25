# Teensy-WavePlayer

Extended version

- Sample rate agnostic
- (up to) 8 Channels / 8 or 16Bit
- delay() after start not needed anymore
- every audio block size
- interleaved reads: only one file access on each audio-cycle
- lastErr(void) returns last error
- addMemoryForRead() adds memory
- syncronized start


Example wave files taken from here:
https://www.jensign.com/bdp95/7dot1voiced/index.html

Converted to 16 bit using SoX (http://sox.sourceforge.net/):

sox  Nums_5dot1_24_48000.wav -b 16 -r 44100 Nums_5dot1_16_44100.wav

Sinewave files from here https://forum.pjrc.com/threads/67754-WaveplayerEx?p=286914&viewfull=1#post286914 (thanks to Jonathan )
