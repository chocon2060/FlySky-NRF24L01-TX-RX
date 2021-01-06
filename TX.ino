#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#define diswc 5 
#define diswd 4
#define avra A6
#define avrb A7
#define ppm_pin 2
uint16_t ch[7];
uint8_t ch_i = 0;
unsigned int PPM_Signal_now;
unsigned int PPM_Signal_last = 0;
  
const uint64_t pipeOut = 0343347641;

RF24 radio(9, 10); 

uint16_t radio_channel[10];

void setup()
{  
  pinMode(ppm_pin,INPUT);
  pinMode(diswc,INPUT);
  pinMode(diswd,INPUT);
  pinMode(avra,INPUT);
  pinMode(avrb,INPUT);
  for(int i = 0; i<= 9; i++){
    radio_channel[i] = 1500;
  }
  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setCRCLength(RF24_CRC_16);
  radio.openWritingPipe(pipeOut);
  
  attachInterrupt(digitalPinToInterrupt(ppm_pin),ppm_read,RISING);
}

void loop()
{
  radio_channel[6] = map(digitalRead(diswc),0,1,2000,1000);
  radio_channel[7]= map(digitalRead(diswd),0,1,2000,1000);
  radio_channel[8] = map(analogRead(avra),0,673,2000,1000); 
  radio_channel[9] = map(analogRead(avrb),0,673,2000,1000);
  radio.write(&radio_channel, sizeof(radio_channel));
}
void ppm_read(){
PPM_Signal_now = micros();
  ch[ch_i] = PPM_Signal_now - PPM_Signal_last;
  PPM_Signal_last = PPM_Signal_now;
  if(ch[ch_i] > 2500 ){
    ch_i = 0;
  }else if(ch[ch_i] >= 2000 &&  ch[ch_i] < 2500){
    ch[ch_i] = 2000;
  } else if(ch[ch_i] <= 1000){
    ch[ch_i] = 1000;
  }
  if(ch_i >=1 && ch_i <=6 ){
  radio_channel[ch_i-1] = ch[ch_i];
  }
  ch_i++;
}
