#include <FUTABA_SBUS.h>
#include <SerialPort.h>
#include <Streaming.h>


FUTABA_SBUS sBus;

SerialPort<0,64,64> debug;

void setup(){
  sBus.Begin();
  debug.begin(57600);
}

void loop(){
  sBus.FeedLine();
  if (sBus.toChannels == 1){
    sBus.UpdateServos();
    sBus.UpdateChannels();
    sBus.toChannels = 0;
    debug<<sBus.channels[0]<<","<<sBus.channels[1]<<","<<sBus.channels[2]<<"\r\n";
  }
}
