// Simple WAV file recorder example
//
//"C:\Program Files (x86)\Arduino\hardware\tools\arm\bin\arm-none-eabi-addr2line" -e C:\Users\Jonathan\AppData\Local\Temp\arduino_build_348415/MultiWavFileRecorderTest.ino.elf 0x2328

#include <math.h>

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

#define AudioRecordQueue AudioRecordWav

// GUItool: begin automatically generated code
AudioPlayWav             playWav1;       //xy=366,1267
AudioPlayWav             playWav3; //xy=365,1541
AudioPlayWav             playWav2;       //xy=367,1400
AudioPlayWav             playWav4; //xy=366,1679
AudioSynthWaveform       waveform1;      //xy=731,1226.015869140625
AudioMixer4              mixerR; //xy=757,1483
AudioMixer4              mixerL;         //xy=759,1364
AudioMixer4              mixerDummy; //xy=900,1302
AudioRecordQueue         recordWav1;         //xy=907,1226
AudioOutputI2S           i2s1;           //xy=962,1424.015869140625
AudioConnection          patchCord1(playWav1, 0, mixerL, 0);
AudioConnection          patchCord2(playWav1, 0, mixerR, 0);
AudioConnection          patchCord3(playWav3, 0, mixerL, 2);
AudioConnection          patchCord4(playWav3, 0, mixerR, 2);
AudioConnection          patchCord5(playWav2, 0, mixerL, 1);
AudioConnection          patchCord6(playWav2, 0, mixerR, 1);
AudioConnection          patchCord7(playWav4, 0, mixerL, 3);
AudioConnection          patchCord8(playWav4, 0, mixerR, 3);
AudioConnection          patchCord9(waveform1, recordWav1);
AudioConnection          patchCord10(waveform1, 0, mixerDummy, 0);
AudioConnection          patchCord11(mixerR, 0, i2s1, 1);
AudioConnection          patchCord12(mixerL, 0, mixerDummy, 3);
AudioConnection          patchCord13(mixerDummy, 0, i2s1, 0);
AudioControlSGTL5000     sgtl5000_1;     //xy=952,1481.015869140625
// GUItool: end automatically generated code

#define SDCARD_CS_PIN    BUILTIN_SDCARD

/*********************************************************************************/
void setStereo(AudioMixer4& left,AudioMixer4& right,int channel,float level,float pan)
{
   left.gain(channel,level*(pan-1)/-2.0f);
  right.gain(channel,level*(pan+1)/ 2.0f);
}
/*********************************************************************************/
extern unsigned long _heap_end;
uint32_t FreeMem(){ 
  char* p = (char*) malloc(10000); // size should be quite big, to avoid allocating fragment!
  free(p);
  return (char *)&_heap_end - p; 
}
/*********************************************************************************/
bool useEventReading = false;
void toggleEventReading(void)
{
  useEventReading = !useEventReading;
  AudioPlayWav::enableEventReading(useEventReading);
  Serial.print(useEventReading?"*** event ***":"*** interrupt ***");
}
/*********************************************************************************/

const char* waves[]={
  "testRec1.wav",
  "testRec2.wav",
  "testRec3.wav",
  "testRec4.wav",
};

AudioPlayWav* tracks[]={&playWav1,&playWav2,&playWav3,&playWav4};
#define SEMI (pow(2.0f,1.0/12))
float freqs[]={220.0f,220.0f*pow(SEMI,4),220.0f*pow(SEMI,7),220.0f*pow(SEMI,10)};

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;
  
  if (CrashReport) {
    Serial.println(CrashReport);
    CrashReport.clear();
  }
  Serial.println("Started!");

  AudioMemory(50);

  while (!(SD.begin(SDCARD_CS_PIN))) {
      Serial.println("Unable to access the SD card");
      delay(500);
  }

  setStereo(mixerL,mixerR,0,0.5,-0.3);
  setStereo(mixerL,mixerR,1,0.5,+0.3);
  setStereo(mixerL,mixerR,2,0.5,-0.7);
  setStereo(mixerL,mixerR,3,0.5,+0.7);

  mixerDummy.gain(0,0.0f);
  mixerDummy.gain(3,1.0f);
  
  sgtl5000_1.enable();
  sgtl5000_1.volume(0.5);

  playWav1.addMemoryForRead(16);
  useEventReading = true;
  AudioPlayWav::enableEventReading(useEventReading);
}


uint32_t next,count,nextDot;
enum state_e {delWavs,rec1,rec2,rec3,rec4,playWavs,silence} state = delWavs;
void loop() 
{  
  // prove we're looping
  if (millis() > nextDot)
  {
    char c = '.';

    nextDot = millis()+250;
    if (playWav1.isPlaying())
      c = '#';
    Serial.print(c); 
  }

  // see if it's time to change state
  if (millis() > next)
  {
    int rec=-1;

    if (recordWav1.isRecording())
    {
      waveform1.amplitude(0.0f);
      delay(4); // a bit of silence at the end
      recordWav1.stop();
      Serial.print("stopped!");
    }
    delay(250);
    next = millis()+3000;
    
    switch (state)
    {
      case silence: // 0.5s silence between playback states
        Serial.println();
        next -= 2500;
        state = playWavs;
        rec = 999;
        playWav1.enableEventReading(useEventReading);
        break;
        
      case delWavs: // delete any existing wave files
        Serial.println("Deleting!");
        for (int i=0;i<4;i++)
          SD.remove(waves[i]);
        state = rec1;
        rec = 999;
        next = millis(); // no need for the delay here
        break;  

      // play back all lower-numbered files while recording 
      // the next, or just all the files
      case playWavs:
        rec++;
        playWav4.play(waves[3],true);
      case rec4:
        rec++;
        playWav3.play(waves[2],true);
      case rec3:
        rec++;
        playWav2.play(waves[1],true);
      case rec2:
        rec++;
        playWav1.play(waves[0],true);
      case rec1:
        rec++;
        state = (state_e)(((int) state)+1);
        break;
    }

    // if we need to record another file, do that
    if (rec<4)
    {
      Serial.printf("\nRecording %s: sine wave at ",waves[rec]);
      Serial.print(freqs[rec]);
      Serial.print("Hz");

      AudioNoInterrupts();
      recordWav1.record(waves[rec],APW_16BIT_SIGNED,1,true);
      waveform1.begin(0.2f,freqs[rec],WAVEFORM_SINE);
    }

    // start playing and recording in sync
    AudioNoInterrupts();
    recordWav1.pause(false);
    playWav1.pause(false);
    playWav2.pause(false);
    playWav3.pause(false);
    playWav4.pause(false);
    AudioInterrupts();
  }
}
