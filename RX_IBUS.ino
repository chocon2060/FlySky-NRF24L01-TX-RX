

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "ibus.h"

#define UPDATE_INTERVAL 7.7
#define NUM_DATA_INPUTS 14
uint16_t dat[] = {0,1, 2, 3,4,5,6,7,8,9,10,11,12,13}; 
#define NUM_CHANNELS 14
#define Led 4
IBus ibus(NUM_CHANNELS);

const uint64_t pipeIn =  0343347641; 
RF24 radio(9, 10);
uint16_t radio_channel[10];
void setup(){
  pinMode(Led,OUTPUT);
  for (int i=0; i<=16 ;i++){
     dat[i] = 1500; 
  }
  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setCRCLength(RF24_CRC_16);
  radio.openReadingPipe(1,pipeIn);
  radio.startListening();
  Serial.begin(115200);   
  
}
unsigned long lastRecvTime = 0;
void recvData()
{  
  while ( radio.available() ) {        
    radio.read(&radio_channel, sizeof(radio_channel));
    lastRecvTime = millis();
  }
}

/**************************************************/

void loop(){
  recvData();

  unsigned long now = millis();
  unsigned long time = millis();
  for(int i = 0; i<=9 ; i++){
    dat[i] = radio_channel[i];
  }
  ibus.begin();
  
  if ( now - lastRecvTime > 500 ) {
    digitalWrite(4,0);
  }
  else
  {
  for(int i=0; i<=13 ; i++){
    ibus.write(dat[i]);
  }
    digitalWrite(4,1);
  }
 
  ibus.end();

  time = millis() - time; 
  if(time < UPDATE_INTERVAL)
  delay(UPDATE_INTERVAL  - time);
}
