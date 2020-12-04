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
uint8_t bit_ppm[6][2];
uint16_t data_avra,data_avrb;
unsigned int PPM_Signal_now;
unsigned int PPM_Signal_last = 0;
  
const uint64_t pipeOut = 0343347641;

RF24 radio(9, 10); 

struct MyData {
  uint8_t ch_16bit[6][2];
  uint8_t swa;
  uint8_t swb;
  uint8_t swc;
  uint8_t swd;
};

MyData data;

void resetData() 
{
  data.ch_16bit[0][0] = 220;
  data.ch_16bit[0][1] = 5;
  data.ch_16bit[1][0] = 220;
  data.ch_16bit[1][1] = 5;
  data.ch_16bit[2][0] = 232;
  data.ch_16bit[2][1] = 3;
  data.ch_16bit[3][0] = 220;
  data.ch_16bit[3][1] = 5;
  data.ch_16bit[4][0] = 220;
  data.ch_16bit[4][1] = 5;
  data.ch_16bit[5][0] = 220;
  data.ch_16bit[5][1] = 5;
  data.swa = 255;
  data.swb = 255;
  data.swc = 255;
  data.swd = 255;
  
}

void setup()
{  
  pinMode(ppm_pin,INPUT);
  pinMode(diswc,INPUT);
  pinMode(diswd,INPUT);
  pinMode(avra,INPUT);
  pinMode(avrb,INPUT);
  attachInterrupt(digitalPinToInterrupt(ppm_pin),ppm_read,RISING);
 
  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(pipeOut);
  resetData();

}


void loop()
{
  data_avra = map(analogRead(avra),0,673,2000,1000);
  bit_ppm[4][0] = data_avra &  0x00ff;
  bit_ppm[4][1] = (data_avra &  0xff00)>>8;
  
  data_avrb = map(analogRead(avrb),0,673,2000,1000);
  bit_ppm[5][0] = data_avrb &  0x00ff;
  bit_ppm[5][1] = (data_avrb &  0xff00)>>8;
  
  for(int i = 0;i<6;i++){
    data.ch_16bit[i][0] = bit_ppm[i][0];
    data.ch_16bit[i][1] = bit_ppm[i][1];
  }
  data.swa      = map(ch[5],1000,2000,0,255);
  data.swb      = map(ch[6],1000,2000,0,255);
  data.swc      = map(digitalRead(diswc),0,1,255,0);
  data.swd      = map(digitalRead(diswd),0,1,255,0);
  radio.write(&data, sizeof(MyData));
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
  if(ch_i>=1 && ch_i <= 4){
    bit_ppm[ch_i-1][0] = ch[ch_i] &  0x00ff;
    bit_ppm[ch_i-1][1] = (ch[ch_i] &  0xff00)>>8;
  }
  ch_i++;
}
