
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

struct MyData {
  uint8_t ch_16bit[7][2];
  uint8_t swa;
  uint8_t swb;
  uint8_t swc;
  uint8_t swd;
};

MyData data;

   
  
void setup()
{

  
  pinMode(Led,OUTPUT);
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.setAutoAck(false);

  radio.openReadingPipe(1,pipeIn);
  radio.startListening();
  Serial.begin(115200);    

}


unsigned long lastRecvTime = 0;

void recvData()
{  
  while ( radio.available() ) {        
    radio.read(&data, sizeof(MyData));
    lastRecvTime = millis();
  }
}

/**************************************************/



void loop(){
  recvData();
//  for(int i =0;i<=6;i++){
//    Serial.print("CH[");
//    Serial.print(i);
//    Serial.print("]:");
//    Serial.print((data.ch_16bit[i][1] <<8) | data.ch_16bit[i][0]);
//    Serial.print("  ");
//  }
//  Serial.println("");
  unsigned long now = millis();
  unsigned long time = millis();
  dat[0] = (data.ch_16bit[1][1] <<8) | data.ch_16bit[1][0];
  dat[1] = (data.ch_16bit[2][1] <<8) | data.ch_16bit[2][0];
  dat[2] = (data.ch_16bit[3][1] <<8) | data.ch_16bit[3][0];
  dat[3] = (data.ch_16bit[4][1] <<8) | data.ch_16bit[4][0];
  dat[4] = map(data.swa,0,255,1000,2000);  
  dat[5] = map(data.swb,0,255,1000,2000);
  dat[6] = map(data.swc,0,255,1000,2000);
  dat[7] = map(data.swd,0,255,1000,2000);
  dat[8] = (data.ch_16bit[5][1] <<8) | data.ch_16bit[5][0];
  dat[9] = (data.ch_16bit[6][1] <<8) | data.ch_16bit[6][0];
  ibus.begin();
  if ( now - lastRecvTime > 500 ) {
    digitalWrite(4,0);
  }
  else
  {
  for(int i=0; i< 10;i++){
  ibus.write(dat[i]);
  }
  for (int i=10;i<=13;i++)
  {
     ibus.write (dat[i] = 0); 
  }
    digitalWrite(4,1);
  }
 
  ibus.end();

  time = millis() - time; 
  if(time < UPDATE_INTERVAL)
  delay(UPDATE_INTERVAL  - time);
}
