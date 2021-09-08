# Teensy-WavePlayer

This is an extended file player audio object, combining the functions of AudioPlaySdWav and AudioPlaySdRaw
with additional stability and functionality.

## Features

- Sample rate agnostic
- (up to) 8 Channels / 8- or 16-bit
- delay() after start no longer needed
- all audio block sizes
- interleaved reads: only one file access on each audio-cycle
- lastErr() returns last error
- addMemoryForRead() uses extra buffer memory to reduce read
- synchronized start
- optional: can use EventResponder - no reads during interrupt

## Working Formats:
- n channel 8 bit unsigned *.wav
- n channel u-law *.wav
- n channel 16 bit signed *.wav
- n channel 8 bit signed *.aiff
- n channel 8 bit unsigned *.aifc
- n channel u-law *.aifc (Apple, non ccitt)
- n channel 16 bit signed *.aiff
- n channel 8 bit signed, unsigned or u-law RAW
- n channel 16 bit signed, unsigned or 16 bit big-endian signed RAW

---
## Updated GUI

An updated `index.html` file is supplied which documents the key API calls provided by AudioPlayWav. The object placed on the design area shows 8 outputs, of which 1, 2, 4, 6 or 8 will output audio data, depending on the number of channels provided in the file [can a RAW file have 3, 5 or 7 channels?]

---
## Main functions
#### play(File | filename [,paused])
Plays the `File` object or named file; setting the optional `paused` parameter to `true` allocates and pre-loads the buffer, but does not start playing. Buffer memory is allocated when this function is called, with the amount dependent on the number of audio channels provided in the file.
#### pause(bool)
Starts a paused object if the parameter is `true`, or pauses it if `false`
#### stop()
Stops playing, whether or not it is paused. The file is closed and buffer memory returned to the heap.
#### isPlaying()
Return true (non-zero) if playing, or false (zero) when stopped or paused.  
#### isStopped()
Return true (non-zero) if stopped, or false (zero) when playing or paused.  
#### isPaused()
Return true (non-zero) if paused, or false (zero) when playing or stopped.  
#### positionMillis()
While playing, return the current time offset, in milliseconds.  When not playing, the return from this function is undefined.
#### lengthMillis()
Return the total length of the current sound clip,in milliseconds. When not playing, the return from this function is undefined.
#### addMemoryForRead(mult) [static]
To make SD reads less frequent, additional buffer memory may be allocated: this should be done prior to calling `play()`. The value of `nult` actually scales the allocated memory, so `2` will double it (and thus halve the read frequency), `1` will allocate the normal minimum. The `mult` value given is used for all objects.
#### enableEventReading(bool) [static]
By default SD card reads occur within the (interrupt driven) audio update cycle. By calling this function with a `true` parameter, SD card reads instead occur within the "foreground" code, using the EventResponder mechanism. Switching between mechanisms may be done at any time, even while playing is in progress, though there *may* be some loss of synchronisation.

*N.B. see the note below about the requirement to call `yield()` if using this option.*
#### lasterr()
If an internal error occurs, for example during a call to `play()` or within the audio update loop interrupt, this function will return the most recent error code. Possible values are:
- 0: no error
- 1: file format unsupported
- 2: file unredable (may not exist?)
- 3: insufficient memory to allocate buffer etc.
- 4: insufficient available audio blocks
---
## Detailed description
For the purposes of this documentation, it is assumed that the audio engine is compiled without changing the default audio block size (128 samples) or sample rate (44.1kHz), resulting in an update cycle of approximately 2.9ms. However, the AudioPlayWav object is intended to work with other block sizes and samples rates - please raise an issue if you find a combination that doesn't work, and you think that it should...
### Interleaved reads
By careful choice of buffer sizes and pre-load amount, the AudioPlayWav objects attempt to ensure that only one SD card read is attempted on each audio engine update. The buffer size required in order to do this is proportional to the total number of AudioPlayWav objects, the number of channels the WAV file contains, the sample size, and the audio block size. So for 3x 4-channel 16-bit files, 3 buffers of 4 x 2 x 128 bytes are needed, for a total allocation of 3kBytes of RAM. The SD card reads will occur every 2.9ms, and read in 1kByte of data.

Note that it is possible to defeat the interleave by starting a number of objects in the paused state, and then un-pausing at exactly the wrong times!
### EventResponder SD card reading
While this option seems likely to reduce efficiency, in fact it appears to improve stability: the authors have comfortably played 16 simultaneous 16-bit mono WAV files without obvious "glitching", where the exact same files / SD card had evident problems using interrupt reads. This mechanism requires the `yield()` function to be called, which could be explicit, or implied by a call to `delay()` or allowing `loop()` to exit.
### Synchronising playback start
In order to play multiple files in exact synchronisation, it is preferable to proceed as follows:
- call `play(<file>,true)` for all files, to pre-load their buffers without outputting any sound
- call `AudioNoInterrupts()` to prevent the audio interrupt firing
- call `pause(false)` for all the objects
- call `AudioInterrupts()` to allow the audio interrupt to fire: playback will start at the next interrupt

---
Example wave files taken from here:
https://www.jensign.com/bdp95/7dot1voiced/index.html
Converted to 16 bit using SoX (http://sox.sourceforge.net/):
`sox  Nums_5dot1_24_48000.wav -b 16 -r 44100 Nums_5dot1_16_44100.wav`

Sinewave files from here https://forum.pjrc.com/threads/67754-WaveplayerEx?p=286914&viewfull=1#post286914 (thanks to Jonathan )
