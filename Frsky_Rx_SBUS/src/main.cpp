#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// #define RSSI_Channel 12
#define UPDATE_INTERVAL 7.7
#define NUM_CHANNELS 14
#define Radio_Channel 12
#define Led 4

const  uint8_t HEADER_ = 0x0F;
const  uint8_t FOOTER_ = 0x00;
const  uint8_t FOOTER2_ = 0x04;
const  uint8_t CH17_MASK_ = 0x01;
const  uint8_t CH18_MASK_ = 0x02;
const  uint8_t LOST_FRAME_MASK_ = 0x04;
const  uint8_t FAILSAFE_MASK_ = 0x08;
bool failsafe_ = false;
bool lost_frame_ = false;
bool dataReceiveDone = false;

const uint64_t pipeIn =  0343347641; 
RF24 radio(9, 10);
uint8_t radioRead[17];
uint16_t radioData[Radio_Channel];

uint8_t sbusData[25];
uint16_t sbusChannel[16];

uint64_t now;
uint64_t sbusLasttime;
void sbusConvert();
void setup(){
  pinMode(Led,OUTPUT);
  for(int i = 0; i < 16 ; i++){
        sbusChannel[i] = 172;
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

  Serial.begin(100000,SERIAL_8E2);
  // Serial.begin(115200);
}
unsigned long lastRecvTime = 0;
void recvData()
{  
  while (radio.available()) {        
    radio.read(&radioRead, sizeof(radioRead));
    lastRecvTime = millis();
    dataReceiveDone = true;
  }
}

/**************************************************/

void loop(){
  recvData();
  now = millis();
  if ( now - lastRecvTime > 500 ) {
    failsafe_ = true;
    for(int i = 0; i < 25 ; i++){
        sbusConvert();
        Serial.write(sbusData[i]);
    }
    digitalWrite(4,0);
  }
  else
  {
    failsafe_ = false;
    if(radio.testRPD()){
      if(sbusChannel[12] < 1811){
        sbusChannel[12]++;
      } 
    }else{
     if(sbusChannel[12] > 172){
      sbusChannel[12]--;  
      }  
    }
    digitalWrite(4,1);
    if(dataReceiveDone){
      radio_decompress();
      for(int i = 0; i < Radio_Channel ; i++){
        sbusChannel[i] = map(radioData[i],1000,2000,172,1811);
      }
      sbusConvert();
      for(int i = 0; i < 25 ; i++){
        Serial.write(sbusData[i]);
      }
      dataReceiveDone = false;
      // sbusLasttime = now; 
    }
  }

}
void radio_decompress(){
  radioData[0]  = uint16_t(radioRead[0]       | radioRead[1]  << 8 & 0x07FF);
  radioData[1]  = uint16_t(radioRead[1]  >> 3 | radioRead[2]  << 5 & 0x07FF);
  radioData[2]  = uint16_t(radioRead[2]  >> 6 | radioRead[3]  << 2  |
                    radioRead[4] << 10 & 0x07FF);
  radioData[3]  = uint16_t(radioRead[4]  >> 1 | radioRead[5]  << 7 & 0x07FF);
  radioData[4]  = uint16_t(radioRead[5]  >> 4 | radioRead[6]  << 4 & 0x07FF);
  radioData[5]  = uint16_t(radioRead[6]  >> 7 | radioRead[7]  << 1  |
                    radioRead[8] << 9 & 0x07FF);
  radioData[6]  = uint16_t(radioRead[8]  >> 2 | radioRead[9] << 6 & 0x07FF);
  radioData[7]  = uint16_t(radioRead[9] >> 5 | radioRead[10] << 3 & 0x07FF);
  radioData[8]  = uint16_t(radioRead[11]      | radioRead[12] << 8 & 0x07FF);
  radioData[9]  = uint16_t(radioRead[12] >> 3 | radioRead[13] << 5 & 0x07FF);
  radioData[10] = uint16_t(radioRead[13] >> 6 | radioRead[14] << 2  |
                    radioRead[15] << 10 & 0x07FF);
  radioData[11] = uint16_t(radioRead[15] >> 1 | radioRead[16] << 7 & 0x07FF);
}
void sbusConvert(){
  sbusData[0] = HEADER_;
  sbusData[1] =   (uint8_t) ((sbusChannel[0]   & 0x07FF));
  sbusData[2] =   (uint8_t) ((sbusChannel[0]   & 0x07FF) >> 8  | (sbusChannel[1]  & 0x07FF) << 3);
  sbusData[3] =   (uint8_t) ((sbusChannel[1]   & 0x07FF) >> 5  | (sbusChannel[2]  & 0x07FF) << 6);
  sbusData[4] =   (uint8_t) ((sbusChannel[2]   & 0x07FF) >> 2);
  sbusData[5] =   (uint8_t) ((sbusChannel[2]   & 0x07FF) >> 10 | (sbusChannel[3]  & 0x07FF) << 1);
  sbusData[6] =   (uint8_t) ((sbusChannel[3]   & 0x07FF) >> 7  | (sbusChannel[4]  & 0x07FF) << 4);
  sbusData[7] =   (uint8_t) ((sbusChannel[4]   & 0x07FF) >> 4  | (sbusChannel[5]  & 0x07FF) << 7);
  sbusData[8] =   (uint8_t) ((sbusChannel[5]   & 0x07FF) >> 1);
  sbusData[9] =   (uint8_t) ((sbusChannel[5]   & 0x07FF) >> 9  | (sbusChannel[6]  & 0x07FF) << 2);
  sbusData[10] =  (uint8_t) ((sbusChannel[6]   & 0x07FF) >> 6  | (sbusChannel[7]  & 0x07FF) << 5);
  sbusData[11] =  (uint8_t) ((sbusChannel[7]   & 0x07FF) >> 3);
  sbusData[12] =  (uint8_t) ((sbusChannel[8]   & 0x07FF));
  sbusData[13] =  (uint8_t) ((sbusChannel[8]   & 0x07FF) >> 8  | (sbusChannel[9]  & 0x07FF) << 3);
  sbusData[14] =  (uint8_t) ((sbusChannel[9]   & 0x07FF) >> 5  | (sbusChannel[10] & 0x07FF) << 6);
  sbusData[15] =  (uint8_t) ((sbusChannel[10]  & 0x07FF) >> 2);
  sbusData[16] =  (uint8_t) ((sbusChannel[10]  & 0x07FF) >> 10 | (sbusChannel[11] & 0x07FF) << 1);
  sbusData[17] =  (uint8_t) ((sbusChannel[11]  & 0x07FF) >> 7  | (sbusChannel[12] & 0x07FF) << 4);
  sbusData[18] =  (uint8_t) ((sbusChannel[12]  & 0x07FF) >> 4  | (sbusChannel[13] & 0x07FF) << 7);
  sbusData[19] =  (uint8_t) ((sbusChannel[13]  & 0x07FF) >> 1);
  sbusData[20] =  (uint8_t) ((sbusChannel[13]  & 0x07FF) >> 9  | (sbusChannel[14] & 0x07FF) << 2);
  sbusData[21] =  (uint8_t) ((sbusChannel[14]  & 0x07FF) >> 6  | (sbusChannel[15] & 0x07FF) << 5);
  sbusData[22] =  (uint8_t) ((sbusChannel[15]  & 0x07FF) >> 3);
  sbusData[23] = 0x00 | (false * CH17_MASK_) | (false * CH18_MASK_) |
             (failsafe_ * FAILSAFE_MASK_) | (lost_frame_ * LOST_FRAME_MASK_);
  sbusData[24] = FOOTER_;
}