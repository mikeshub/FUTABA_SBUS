#include "FUTABA_SBUS.h"
//#include <SerialPort.h>

void FUTABA_SBUS::Begin(){
	//int i;
	uint8_t loc_sbusData[25] = {
	  0x0f,0x01,0x04,0x20,0x00,0xff,0x07,0x40,0x00,0x02,0x10,0x80,0x2c,0x64,0x21,0x0b,0x59,0x08,0x40,0x00,0x02,0x10,0x80,0x00,0x00};
	int16_t loc_channels[18]  = {
	  		1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,0,0};
	int16_t loc_servos[18]    = {
  			1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,0,0};
	port0.begin(BAUDRATE,  SP_2_STOP_BIT | SP_EVEN_PARITY | SP_8_BIT_CHAR);
	//sbusData[] = {0x0f,0x01,0x04,0x20,0x00,0xff,0x07,0x40,0x00,0x02,0x10,0x80,0x2c,0x64,0x21,0x0b,0x59,0x08,0x40,0x00,0x02,0x10,0x80,0x00,0x00};
	//channels[]  = {1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,0,0};
	//servos    = [1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,1023,0,0];
	memcpy(sbusData,loc_sbusData,25);
	memcpy(channels,loc_channels,18);
	memcpy(servos,loc_servos,18);
	failsafe_status = SBUS_SIGNAL_OK;
	sbus_passthrough = 1;
	toChannels = 0;
	bufferIndex=0;
	feedState = 0;
}

int16_t FUTABA_SBUS::Channel(uint8_t ch) {
  // Read channel data
  if ((ch>0)&&(ch<=16)){
    return channels[ch-1];
  }
  else{
    return 1023;
  }
}
uint8_t FUTABA_SBUS::DigiChannel(uint8_t ch) {
  // Read digital channel data
  if ((ch>0) && (ch<=2)) {
    return channels[15+ch];
  }
  else{
    return 0;
  }
}
void FUTABA_SBUS::Servo(uint8_t ch, int16_t position) {
  // Set servo position
  if ((ch>0)&&(ch<=16)) {
    if (position>2048) {
      position=2048;
    }
    servos[ch-1] = position;
  }
}
void FUTABA_SBUS::DigiServo(uint8_t ch, uint8_t position) {
  // Set digital servo position
  if ((ch>0) && (ch<=2)) {
    if (position>1) {
      position=1;
    }
    servos[15+ch] = position;
  }
}
uint8_t FUTABA_SBUS::Failsafe(void) {
  return failsafe_status;
}

void FUTABA_SBUS::PassthroughSet(int mode) {
  // Set passtrough mode, if true, received channel data is send to servos
  sbus_passthrough = mode;
}

int FUTABA_SBUS::PassthroughRet(void) {
  // Return current passthrough mode
  return sbus_passthrough;
}
void FUTABA_SBUS::UpdateServos(void) {
  // Send data to servos
  // Passtrough mode = false >> send own servo data
  // Passtrough mode = true >> send received channel data
  uint8_t i;
  if (sbus_passthrough==0) {
    // clear received channel data
    for (i=1; i<24; i++) {
      sbusData[i] = 0;
    }

    // reset counters
    ch = 0;
    bit_in_servo = 0;
    byte_in_sbus = 1;
    bit_in_sbus = 0;

    // store servo data
    for (i=0; i<176; i++) {
      if (servos[ch] & (1<<bit_in_servo)) {
        sbusData[byte_in_sbus] |= (1<<bit_in_sbus);
      }
      bit_in_sbus++;
      bit_in_servo++;

      if (bit_in_sbus == 8) {
        bit_in_sbus =0;
        byte_in_sbus++;
      }
      if (bit_in_servo == 11) {
        bit_in_servo =0;
        ch++;
      }
    }

    // DigiChannel 1
    if (channels[16] == 1) {
      sbusData[23] |= (1<<0);
    }
    // DigiChannel 2
    if (channels[17] == 1) {
      sbusData[23] |= (1<<1);
    }

    // Failsafe
    if (failsafe_status == SBUS_SIGNAL_LOST) {
      sbusData[23] |= (1<<2);
    }

    if (failsafe_status == SBUS_SIGNAL_FAILSAFE) {
      sbusData[23] |= (1<<2);
      sbusData[23] |= (1<<3);
    }
  }
  // send data out
  //serialPort.write(sbusData,25);
  for (i=0;i<25;i++) {
    port0.write(sbusData[i]);
  }
}
void FUTABA_SBUS::UpdateChannels(void) {
  uint8_t i;
  uint8_t sbus_pointer = 0;
  // clear channels[]
  for (i=0; i<16; i++) {
    channels[i] = 0;
  }

  // reset counters
  byte_in_sbus = 1;
  bit_in_sbus = 0;
  ch = 0;
  bit_in_channel = 0;

  // process actual sbus data
  for (i=0; i<176; i++) {
    if (sbusData[byte_in_sbus] & (1<<bit_in_sbus)) {
      channels[ch] |= (1<<bit_in_channel);
    }
    bit_in_sbus++;
    bit_in_channel++;

    if (bit_in_sbus == 8) {
      bit_in_sbus =0;
      byte_in_sbus++;
    }
    if (bit_in_channel == 11) {
      bit_in_channel =0;
      ch++;
    }
  }
  // DigiChannel 1
  if (sbusData[23] & (1<<0)) {
    channels[16] = 1;
  }
  else{
    channels[16] = 0;
  }
  // DigiChannel 2
  if (sbusData[23] & (1<<1)) {
    channels[17] = 1;
  }
  else{
    channels[17] = 0;
  }
  // Failsafe
  failsafe_status = SBUS_SIGNAL_OK;
  if (sbusData[23] & (1<<2)) {
    failsafe_status = SBUS_SIGNAL_LOST;
  }
  if (sbusData[23] & (1<<3)) {
    failsafe_status = SBUS_SIGNAL_FAILSAFE;
  }

}
void FUTABA_SBUS::FeedLine(void){
  if (port0.available() > 24){
    while(port0.available() > 0){
      inData = port0.read();
      switch (feedState){
      case 0:
        if (inData != 0x0f){
          while(port0.available() > 0){//read the contents of in buffer this should resync the transmission
            inData = port0.read();
          }
          return;
        }
        else{
          bufferIndex = 0;
          inBuffer[bufferIndex] = inData;
          inBuffer[24] = 0xff;
          feedState = 1;
        }
        break;
      case 1:
        bufferIndex ++;
        inBuffer[bufferIndex] = inData;
        if (bufferIndex < 24 && port0.available() == 0){
          feedState = 0;
        }
        if (bufferIndex == 24){
          feedState = 0;
          if (inBuffer[0]==0x0f && inBuffer[24] == 0x00){
            memcpy(sbusData,inBuffer,25);
            toChannels = 1;
          }
        }
        break;
      }
    }
  }
}

