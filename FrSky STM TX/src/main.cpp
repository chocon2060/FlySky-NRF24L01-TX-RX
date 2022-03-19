#include <Arduino.h>
#include <ibus.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define numOfSwitch 4
#define numOfVariable 4

#define PinSw1 PA1
#define PinSw2 PA2
#define PinSw3 PA3
#define PinSw4 PA4

#define PinVar1 PA5
#define PinVar2 PA6
#define PinVar3 PA7
#define PinVar4 PB0

#define PinCE PA8
#define PinCSN PB12
#define PinMISO PB14
#define PinMOSI PB15
#define PinSCLK PB13

#define Nrfswtich PB10
#define R9mswtich PB11
#define ppmInput PA0

#define nrfPower PB4
#define bluetoothPower PB3
#define ppmOutput PB6

#define ppmInChannel 4
HardwareTimer *ppmTimer2 = new HardwareTimer(TIM2);
int32_t channel_start, channel_stop, channel;
uint16_t ppmData[ppmInChannel];
uint8_t channel_count = 0;

#define PinIbus PA9
#define UPDATE_INTERVAL 8
#define NUM_CHANNELS  (ppmInChannel + numOfSwitch + numOfVariable)
uint16_t ibusDat[NUM_CHANNELS];
uint32_t ibusTime;
IBus ibus(NUM_CHANNELS);

#define radio_Rawchannel  (ppmInChannel + numOfSwitch + numOfVariable)
RF24 radio(PinCE, PinCSN);
const uint64_t pipeOut = 0343347641;
uint16_t radio_RawData[radio_Rawchannel];
uint8_t radio_data[17];
#define cppmlength (ppmInChannel + numOfSwitch + numOfVariable)
HardwareTimer *cppm = new HardwareTimer(TIM4);
uint16_t cppmOut[cppmlength];
uint8_t cppmCount = 0;

bool NrfMode = false;
bool BluetoothMode = false;
bool R9mMode = false;

uint16_t swData[numOfSwitch];
uint16_t varData[numOfVariable];
void switch_var_read();

void modeChange();
void ppmIn_intterrupt(void);
void cppmInterrupt();

void radioSetting();
void timer2Setting();
void timer4Setting();

void disableIbus();
void disableSpi();
void dataCompression();
void setup() {
    pinMode(PinSw1,INPUT);
    pinMode(PinSw2,INPUT);
    pinMode(PinSw3,INPUT);
    pinMode(PinSw4,INPUT);

    pinMode(PinVar1,INPUT);
    pinMode(PinVar2,INPUT);
    pinMode(PinVar3,INPUT);
    pinMode(PinVar4,INPUT);

    pinMode(ppmInput,INPUT);
    pinMode(Nrfswtich,INPUT);
    pinMode(R9mswtich,INPUT);
    
    pinMode(nrfPower,OUTPUT);
    pinMode(bluetoothPower,OUTPUT);

    digitalWrite(nrfPower,LOW);
    digitalWrite(bluetoothPower,LOW);

    attachInterrupt(Nrfswtich,modeChange,CHANGE);
    attachInterrupt(R9mswtich,modeChange,CHANGE);
    //NRF24L01 On
    if(digitalRead(Nrfswtich)){
        NrfMode = true;
        digitalWrite(nrfPower,HIGH);
        delay(200);
        disableIbus();
        radioSetting();
       
    }
    //R9m On
    else if(digitalRead(R9mswtich)){
        R9mMode = true;
        delay(200);
        disableSpi();
        disableIbus();
        for(int i = 0 ; i < cppmlength ; i++){
            cppmOut[i] = 1500;
        }
        timer4Setting();
    }
    //Bluetooth On
    else if(!digitalRead(Nrfswtich) && !digitalRead(R9mswtich)){
        BluetoothMode = true;
        digitalWrite(bluetoothPower,HIGH);
        delay(200);
        disableSpi();
        for(int i = 0; i < NUM_CHANNELS ; i++){
            ibusDat[i] = 1500;
        }
        Serial.begin(115200);
    }
    timer2Setting();
}

void loop() {
    switch_var_read();
    //NRF Mode Runinng
    if(NrfMode){
        for(int i = 0 ; i < ppmInChannel ; i++){
            radio_RawData[i] = ppmData[i];
        }
       for(int i = 0 ; i < numOfSwitch ; i++){
            radio_RawData[i+ppmInChannel] = swData[i];
        }
        for(int i = 0 ; i < numOfVariable ; i++){
            radio_RawData[i + ppmInChannel + numOfSwitch] = varData[i];
        }
        dataCompression();
        radio.write(&radio_data,sizeof(radio_data));
    }
    //R9m Mode Runinng
    else if(R9mMode){
        for(int i = 0;i < ppmInChannel ; i++){
            cppmOut[i] = ppmData[i];
        }
        for(int i = 0 ; i < numOfSwitch ; i++){
            cppmOut[i+ppmInChannel] = swData[i];
        }
        for(int i = 0 ; i < numOfVariable ; i++){
            cppmOut[i + ppmInChannel + numOfSwitch] = varData[i];
        }
    }
    //Bluetooth Mode Runinng
    else if(BluetoothMode){
        
        for(int i = 0;i < ppmInChannel ; i++){
            ibusDat[i] = ppmData[i];
        }
        for(int i = 0 ; i < numOfSwitch ; i++){
            ibusDat[i+ppmInChannel] = swData[i];
        }
        for(int i = 0 ; i < numOfVariable ; i++){
            ibusDat[i + ppmInChannel + numOfSwitch] = varData[i];
        }
        ibusTime = millis();
        ibus.begin();
        for(int i = 0; i < NUM_CHANNELS ; i++){
            ibus.write(ibusDat[i]);
        }
        ibus.end();

        ibusTime = millis() - ibusTime; 
        if(ibusTime < UPDATE_INTERVAL)
        delay(UPDATE_INTERVAL  - ibusTime);
    }
}
void switch_var_read(){
    swData[0] = (digitalRead(PinSw1)*1000) + 1000;
    swData[1] = (digitalRead(PinSw2)*1000) + 1000;
    swData[2] = (digitalRead(PinSw3)*1000) + 1000;
    swData[3] = (digitalRead(PinSw4)*1000) + 1000;
    varData[0] = map(analogRead(PinVar1),0,1023,1000,2000);
    varData[1] = map(analogRead(PinVar2),0,1023,1000,2000);
    varData[2] = map(analogRead(PinVar3),0,1023,1000,2000);
    varData[3] = map(analogRead(PinVar4),0,1023,1000,2000);
}
void modeChange(){
    detachInterrupt(Nrfswtich);
    detachInterrupt(R9mswtich);
    delay(100);
    HAL_NVIC_SystemReset();
}
void ppmIn_intterrupt(void){
  channel_start = TIM2->CCR1;
  channel = channel_start - channel_stop;
  channel_stop = channel_start;
  if(channel < 0){
      channel += 0xffff;
  }
  if(channel > 2500){
    channel_count = 0;
  }else{
    if(channel >= 2000){
      channel = 2000;
    }else if (channel <=1000){
      channel = 1000;
    }
    ppmData[channel_count] = channel;
    channel_count++;
  }
}
void cppmInterrupt(){
    cppmCount++;
    if(cppmCount > cppmlength){
        cppmCount = 0;
    }
    if(cppmCount < cppmlength){
        if(cppmOut[cppmCount] <= 0){
            cppmOut[cppmCount] = 1000;
        }
        TIM4->ARR = cppmOut[cppmCount];
    }else if(cppmCount == cppmlength){
        TIM4->ARR = 8000;
    }
}
void radioSetting(){
    radio.begin(); 
    radio.setAutoAck(false);
    radio.setDataRate(RF24_250KBPS);
    radio.setPALevel(RF24_PA_HIGH,false);
    radio.setChannel(104);
    radio.setCRCLength(RF24_CRC_16);
    radio.openWritingPipe(pipeOut);
    radio.powerUp();
}
void timer2Setting(){
    ppmTimer2->attachInterrupt(1,ppmIn_intterrupt);
    TIM2->CR1 = TIM_CR1_CEN;
    TIM2->CR2 = 0;
    TIM2->SMCR = 0;
    TIM2->DIER = TIM_DIER_CC1IE;
    TIM2->EGR = 0;
    TIM2->CCMR1 = TIM_CCMR1_CC1S_0;
    TIM2->CCMR2 = 0;
    TIM2->CCER = TIM_CCER_CC1E;
    TIM2->PSC = 71;
    TIM2->ARR = 0xffff;
    TIM2->DCR = 0;
}
void timer4Setting(){
    cppm->attachInterrupt(cppmInterrupt);
    cppm->setMode(1,TIMER_OUTPUT_COMPARE_PWM1,ppmOutput);
    cppm->setPreloadEnable(true);
    TIM4->CCER = TIM_CCER_CC1E;
    TIM4->PSC = 71;
    TIM4->ARR = cppmOut[0];
    TIM4->DCR = 0;
    TIM4->CCR1 = 300;
    TIM4->CR1 = TIM_CR1_CEN;
}
void disableIbus(){
    pinMode(PinIbus,OUTPUT);
    digitalWrite(PinIbus,LOW);
}
void disableSpi(){
    pinMode(PinCE,OUTPUT);
    pinMode(PinCSN,OUTPUT);
    pinMode(PinMISO,OUTPUT);
    pinMode(PinMOSI,OUTPUT);
    pinMode(PinSCLK,OUTPUT);

    digitalWrite(PinCE,LOW);
    digitalWrite(PinCSN,LOW);
    digitalWrite(PinMISO,LOW);
    digitalWrite(PinMOSI,LOW);
    digitalWrite(PinSCLK,LOW);
}
void dataCompression(){
    radio_data[0] =   (uint8_t) ((radio_RawData[0]   & 0x07FF));
    radio_data[1] =   (uint8_t) ((radio_RawData[0]   & 0x07FF) >> 8  | (radio_RawData[1]  & 0x07FF) << 3);
    radio_data[2] =   (uint8_t) ((radio_RawData[1]   & 0x07FF) >> 5  | (radio_RawData[2]  & 0x07FF) << 6);
    radio_data[3] =   (uint8_t) ((radio_RawData[2]   & 0x07FF) >> 2);
    radio_data[4] =   (uint8_t) ((radio_RawData[2]   & 0x07FF) >> 10 | (radio_RawData[3]  & 0x07FF) << 1);
    radio_data[5] =   (uint8_t) ((radio_RawData[3]   & 0x07FF) >> 7  | (radio_RawData[4]  & 0x07FF) << 4);
    radio_data[6] =   (uint8_t) ((radio_RawData[4]   & 0x07FF) >> 4  | (radio_RawData[5]  & 0x07FF) << 7);
    radio_data[7] =   (uint8_t) ((radio_RawData[5]   & 0x07FF) >> 1);
    radio_data[8] =   (uint8_t) ((radio_RawData[5]   & 0x07FF) >> 9  | (radio_RawData[6]  & 0x07FF) << 2);
    radio_data[9] =  (uint8_t) ((radio_RawData[6]   & 0x07FF) >> 6  | (radio_RawData[7]  & 0x07FF) << 5);
    radio_data[10] =  (uint8_t) ((radio_RawData[7]   & 0x07FF) >> 3);
    radio_data[11] =  (uint8_t) ((radio_RawData[8]   & 0x07FF));
    radio_data[12] =  (uint8_t) ((radio_RawData[8]   & 0x07FF) >> 8  | (radio_RawData[9]  & 0x07FF) << 3);
    radio_data[13] =  (uint8_t) ((radio_RawData[9]   & 0x07FF) >> 5  | (radio_RawData[10] & 0x07FF) << 6);
    radio_data[14] =  (uint8_t) ((radio_RawData[10]  & 0x07FF) >> 2);
    radio_data[15] =  (uint8_t) ((radio_RawData[10]  & 0x07FF) >> 10 | (radio_RawData[11] & 0x07FF) << 1);
    radio_data[16] =  (uint8_t) ((radio_RawData[11]  & 0x07FF) >> 7);
}