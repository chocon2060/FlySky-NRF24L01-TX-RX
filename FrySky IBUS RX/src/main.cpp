#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "ibus.h"

#define RSSI_Channel 12
#define UPDATE_INTERVAL 7.7
#define NUM_CHANNELS 14
#define Radio_Channel 12
#define Led 4

uint16_t ibusData[NUM_CHANNELS];
IBus ibus(NUM_CHANNELS);

const uint64_t pipeIn =  0343347641; 
RF24 radio(9, 10);

uint16_t radioData[Radio_Channel];
void setup(){
  pinMode(Led,OUTPUT);
  for (int i=0; i<NUM_CHANNELS ;i++){
     ibusData[i] = 1500; 
  }

  radio.begin(); 
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW,true);
  radio.setChannel(104);
  radio.setCRCLength(RF24_CRC_16);
  radio.openReadingPipe(1,pipeIn);
  radio.startListening();
  radio.powerUp();

  Serial.begin(115200);   
  
}
unsigned long lastRecvTime = 0;
void recvData()
{  
  while (radio.available()) {        
    radio.read(&radioData, sizeof(radioData));
    lastRecvTime = millis();
  }
}

/**************************************************/

void loop(){
  recvData();
  
  unsigned long now = millis();
  unsigned long time = millis();
  for(int i = 0; i < Radio_Channel ; i++){
    ibusData[i] = radioData[i];
  }

  
  
  if ( now - lastRecvTime > 500 ) {
    digitalWrite(4,0);
  }
  else
  {
    if(radio.testRPD()){
      if(ibusData[RSSI_Channel] <2000){
        ibusData[RSSI_Channel]++;
      } 
    }else{
     if(ibusData[RSSI_Channel] >= 2){
      ibusData[RSSI_Channel]--;
      }  
    }
    ibus.begin();
    for(int i=0; i < NUM_CHANNELS ; i++){
      ibus.write(ibusData[i]);
    }
    digitalWrite(4,1);
  }

  ibus.end();
  
  time = millis() - time; 
  if(time < UPDATE_INTERVAL)
  delay(UPDATE_INTERVAL  - time);
}