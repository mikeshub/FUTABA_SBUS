#include <FUTABA_SBUS.h>
#include <Streaming.h>


FUTABA_SBUS sBus;


void setup(){
  sBus.begin();
  Serial.begin(115200);
}

void loop(){
  sBus.FeedLine();
  if (sBus.toChannels == 1){
    sBus.UpdateServos();
    sBus.UpdateChannels();
    sBus.toChannels = 0;
    Serial<<sBus.channels[0]<<","<<sBus.channels[1]<<","<<sBus.channels[2]<<"\r\n";
  }
}
