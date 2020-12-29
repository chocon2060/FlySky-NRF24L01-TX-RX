#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define RC_CHANNEL_MIN 990
#define RC_CHANNEL_MAX 2010

#define SBUS_MIN_OFFSET 173
#define SBUS_MID_OFFSET 992
#define SBUS_MAX_OFFSET 1811
#define SBUS_CHANNEL_NUMBER 16
#define SBUS_PACKET_LENGTH 25
#define SBUS_FRAME_HEADER 0x0f
#define SBUS_FRAME_FOOTER 0x00
#define SBUS_FRAME_FOOTER_V2 0x04
#define SBUS_STATE_FAILSAFE 0x08
#define SBUS_STATE_SIGNALLOSS 0x04
#define SBUS_UPDATE_RATE 15 //ms

const uint64_t pipeIn =  0343347641; 
RF24 radio(9, 10);

void sbusPreparePacket(uint8_t packet[], int channels[], bool isSignalLoss, bool isFailsafe){

    static int output[SBUS_CHANNEL_NUMBER] = {0};

    /*
     * Map 1000-2000 with middle at 1500 chanel values to
     * 173-1811 with middle at 992 S.BUS protocol requires
     */
    for (uint8_t i = 0; i < SBUS_CHANNEL_NUMBER; i++) {
        output[i] = map(channels[i], RC_CHANNEL_MIN, RC_CHANNEL_MAX, SBUS_MIN_OFFSET, SBUS_MAX_OFFSET);
    }

    uint8_t stateByte = 0x00;
    if (isSignalLoss) {
        stateByte |= SBUS_STATE_SIGNALLOSS;
    }
    if (isFailsafe) {
        stateByte |= SBUS_STATE_FAILSAFE;
    }
    packet[0] = SBUS_FRAME_HEADER; //Header

    packet[1] = (uint8_t) (output[0] & 0x07FF);
    packet[2] = (uint8_t) ((output[0] & 0x07FF)>>8 | (output[1] & 0x07FF)<<3);
    packet[3] = (uint8_t) ((output[1] & 0x07FF)>>5 | (output[2] & 0x07FF)<<6);
    packet[4] = (uint8_t) ((output[2] & 0x07FF)>>2);
    packet[5] = (uint8_t) ((output[2] & 0x07FF)>>10 | (output[3] & 0x07FF)<<1);
    packet[6] = (uint8_t) ((output[3] & 0x07FF)>>7 | (output[4] & 0x07FF)<<4);
    packet[7] = (uint8_t) ((output[4] & 0x07FF)>>4 | (output[5] & 0x07FF)<<7);
    packet[8] = (uint8_t) ((output[5] & 0x07FF)>>1);
    packet[9] = (uint8_t) ((output[5] & 0x07FF)>>9 | (output[6] & 0x07FF)<<2);
    packet[10] = (uint8_t) ((output[6] & 0x07FF)>>6 | (output[7] & 0x07FF)<<5);
    packet[11] = (uint8_t) ((output[7] & 0x07FF)>>3);
    packet[12] = (uint8_t) ((output[8] & 0x07FF));
    packet[13] = (uint8_t) ((output[8] & 0x07FF)>>8 | (output[9] & 0x07FF)<<3);
    packet[14] = (uint8_t) ((output[9] & 0x07FF)>>5 | (output[10] & 0x07FF)<<6);  
    packet[15] = (uint8_t) ((output[10] & 0x07FF)>>2);
    packet[16] = (uint8_t) ((output[10] & 0x07FF)>>10 | (output[11] & 0x07FF)<<1);
    packet[17] = (uint8_t) ((output[11] & 0x07FF)>>7 | (output[12] & 0x07FF)<<4);
    packet[18] = (uint8_t) ((output[12] & 0x07FF)>>4 | (output[13] & 0x07FF)<<7);
    packet[19] = (uint8_t) ((output[13] & 0x07FF)>>1);
    packet[20] = (uint8_t) ((output[13] & 0x07FF)>>9 | (output[14] & 0x07FF)<<2);
    packet[21] = (uint8_t) ((output[14] & 0x07FF)>>6 | (output[15] & 0x07FF)<<5);
    packet[22] = (uint8_t) ((output[15] & 0x07FF)>>3);

    packet[23] = stateByte; //Flags byte
    packet[24] = SBUS_FRAME_FOOTER; //Footer
}

uint8_t sbusPacket[SBUS_PACKET_LENGTH];
int rcChannels[SBUS_CHANNEL_NUMBER];
uint32_t sbusTime = 0;
struct MyData {
  uint8_t ch_16bit[7][2];
  uint8_t swa;
  uint8_t swb;
  uint8_t swc;
  uint8_t swd;
};

MyData data;
bool SignalLoss = false;
bool Failsafe = false;
void setup() {
    for (uint8_t i = 0; i < SBUS_CHANNEL_NUMBER; i++) {
        rcChannels[i] = 0;
    }
    radio.begin();
    radio.setDataRate(RF24_250KBPS);
    radio.setAutoAck(false);
    radio.openReadingPipe(1,pipeIn);
    radio.startListening();
    
    Serial.begin(100000, SERIAL_8E2);
}

unsigned long lastRecvTime = 0;

void recvData()
{  
  while ( radio.available() ) {        
    radio.read(&data, sizeof(MyData));
    lastRecvTime = millis();
  }
}
void loop() {
      recvData();
    uint32_t currentMillis = millis();
    unsigned long now = millis();
    if ( now - lastRecvTime > 500 ) {
        SignalLoss = true;
        Failsafe = true;
    }
    else{ 
      SignalLoss = false;
      Failsafe = false;
      rcChannels[0] = (data.ch_16bit[1][1] <<8) | data.ch_16bit[1][0];
      rcChannels[1] = (data.ch_16bit[2][1] <<8) | data.ch_16bit[2][0];
      rcChannels[2] = (data.ch_16bit[3][1] <<8) | data.ch_16bit[3][0];
      rcChannels[3] = (data.ch_16bit[4][1] <<8) | data.ch_16bit[4][0];
      rcChannels[4] = map(data.swa,0,255,1000,2000);  
      rcChannels[5] = map(data.swb,0,255,1000,2000);
      rcChannels[6] = map(data.swc,0,255,1000,2000);
      rcChannels[7] = map(data.swd,0,255,1000,2000);
      rcChannels[8] = (data.ch_16bit[5][1] <<8) | data.ch_16bit[5][0];
      rcChannels[9] = (data.ch_16bit[6][1] <<8) | data.ch_16bit[6][0];
    }
    if (currentMillis > sbusTime) {
        
        sbusPreparePacket(sbusPacket, rcChannels, SignalLoss, Failsafe);
        Serial.write(sbusPacket, SBUS_PACKET_LENGTH);

        sbusTime = currentMillis + SBUS_UPDATE_RATE;
    }
}
