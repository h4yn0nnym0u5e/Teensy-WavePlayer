
#include <Audio.h>
#include <MemoryHexDump.h>

AudioSynthToneSweep myEffect;
AudioOutputPT8211   audioOutput;
AudioRecordWav      record;
AudioPlayWav        play;

//AudioConnection c1(myEffect, 0, audioOutput, 0);
//AudioConnection c2(myEffect, 0, audioOutput, 1);
AudioConnection c1(play, 0, audioOutput, 0);
AudioConnection c2(play, 0, audioOutput, 1);

AudioConnection c3(myEffect, 0, record, 0);
AudioConnection c4(myEffect, 0, record, 1);

const char filename[] = "test5.wav";

char buf[8192];

float t_ampx = 0.8;
int t_lox = 500;
int t_hix = 5000;
float t_timex = 0.5;// Length of time for the sweep in seconds

File file;

void dump(void)
{
  file = SD.open(filename,FILE_READ);
  Serial.printf("File size:%d", file.size());  
  file.read(&buf, sizeof(buf));
  MemoryHexDump(Serial, &buf, sizeof(buf), true, "\n*** Buffer dups removed ***\n");
  file.close();
}

void setup(void)
{
  AudioMemory(20);
  Serial.begin(9600);
  memset(&buf, 0, sizeof(buf));
  delay(1000);
  if (CrashReport) {
    Serial.println(CrashReport);
    CrashReport.clear();  
  }

  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("SD: initialization failed!");
    return;
  }
  
  SD.remove(filename);
  
  // open the file. 
  file = SD.open(filename, FILE_WRITE_BEGIN);
    
#if 1  
  Serial.println("Record effect (silent):");
  AudioNoInterrupts();
  record.record(file, APW_16BIT_SIGNED, 2);  
  if(!myEffect.play(t_ampx,t_lox,t_hix,t_timex)) {
    Serial.println("AudioSynthToneSweep - begin failed");
    while(1);
  }
  AudioInterrupts();
  // wait for the sweep to end
  while(myEffect.isPlaying());

  // and now reverse the sweep
  if(!myEffect.play(t_ampx,t_hix,t_lox,t_timex)) {
    Serial.println("AudioSynthToneSweep - begin failed");
    while(1);
  }
  // wait for the sweep to end
  while(myEffect.isPlaying());
  record.stop();
  file.close();
  
  Serial.println("Done");
#endif
 // dump();
  Serial.println("Play:");
  play.play(filename);
}

void loop(void)
{
}
