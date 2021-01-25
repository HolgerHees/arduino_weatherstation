// w132.cpp
#include "Arduino.h"
#include "../log.h"
#include "../helper.h"
#include "w132.h"

unsigned long W132Sensor::lastTrigger;

byte W132Sensor::bitPos;
byte W132Sensor::messageNum;
// there are always sent 6 messages. For temperature/humidity, the message is repeated 6 times. For wind, 2 messages are repeated 3 times each
bool W132Sensor::message1Bits[36];
bool W132Sensor::message2Bits[36];

volatile float W132Sensor::windSpeed;
volatile float W132Sensor::windGust;
volatile int16_t W132Sensor::windDirection;
//volatile float W132Sensor::temperature;
//volatile int8_t W132Sensor::humidity;

void (*W132Sensor::interruptCallback)(bool);

void W132Sensor::init(int8_t pin,void(*interruptCallback)(bool))
{
  lastTrigger = 0;
  
  bitPos=-1;
  messageNum=-1;

  W132Sensor::interruptCallback = interruptCallback;
  
  windSpeed = 0;
  windGust = 0;
  windDirection = 0;
//  temperature = 0;
//  humidity = 0;
  
  attachInterrupt(digitalPinToInterrupt(pin), dataHandler, FALLING);
}

W132Values W132Sensor::getValues()
{
  W132Values values = (struct W132Values) {Helper::roundFloat(windSpeed),Helper::roundFloat(windGust),windDirection};//,temperature,humidity};

  windSpeed = 0;

  return values;
}

bool W132Sensor::verifyChecksum(bool bits[]) {
  int checksum=0xf;
  for (int i=0;i<8;i++) {
    checksum-=bits[i*4]|bits[i*4+1]<<1|bits[i*4+2]<<2|bits[i*4+3]<<3;
  }
  checksum&=0xf;
  int expectedChecksum=bits[32]|bits[33]<<1|bits[34]<<2|bits[35]<<3;
  return checksum==expectedChecksum;
}

void W132Sensor::decodeMessages() {
  if (!verifyChecksum(message1Bits)) {
    return;
  }
  
  if (message1Bits[9]==1 && message1Bits[10]==1) { // wind data (2 messages)

    if (!verifyChecksum(message2Bits)) {
      DEBUG_PRINTLN(F("W132 CS ERR"));
      return;
    }
    
    DEBUG_PRINTLN(F("W132 WD"));

    float _windSpeed= ((message1Bits[24]    | message1Bits[25]<<1 | message1Bits[26]<<2 | message1Bits[27]<<3 |
                       message1Bits[28]<<4 | message1Bits[29]<<5 | message1Bits[30]<<6 | message1Bits[31]<<7)*0.2f);
    if( _windSpeed > windSpeed ) windSpeed = _windSpeed;
  
    windGust= ((message2Bits[24]    | message2Bits[25]<<1 | message2Bits[26]<<2 | message2Bits[27]<<3 |
                      message2Bits[28]<<4 | message2Bits[29]<<5 | message2Bits[30]<<6 | message2Bits[31]<<7)*0.2f);

    windDirection=(message2Bits[15]    | message2Bits[16]<<1 | message2Bits[17]<<2 | message2Bits[18]<<3 |
                       message2Bits[19]<<4 | message2Bits[20]<<5 | message2Bits[21]<<6 | message2Bits[22]<<7 |
                       message2Bits[23]<<8);
  }
  /*else{  // temperature/humidity in both messages
    DEBUG_PRINTLN(F("W132 TH"));

    int temperatureRaw=(message1Bits[12]    | message1Bits[13]<<1 | message1Bits[14]<<2 | message1Bits[15]<<3 |
                        message1Bits[16]<<4 | message1Bits[17]<<5 | message1Bits[18]<<6 | message1Bits[19]<<7 | 
                        message1Bits[20]<<8 | message1Bits[21]<<9 | message1Bits[22]<<10| message1Bits[23]<<11);
    if (temperatureRaw& 0x800) temperatureRaw+=0xF000; // negative number, 12 to 16 bit
    temperature= (temperatureRaw*0.1f);

    humidity=(message1Bits[24] | message1Bits[25]<<1 | message1Bits[26]<<2 | message1Bits[27]<<3 )+
                 (message1Bits[28] | message1Bits[29]<<1 | message1Bits[30]<<2 | message1Bits[31]<<3 )*10;
  }*/
}

void W132Sensor::dataHandler() {
  interruptCallback(true);
  
  sei();//allow interrupts
  
  unsigned long now=micros();
  unsigned long duration=now-lastTrigger;
  lastTrigger=now;

  if (duration>30000) { // a news block of messages begins
    messageNum=0;
  }

  if (duration>7000) { // ~9 ms = sync signal
    if (bitPos==36) { // we got a full message
      if (messageNum==0) { // 1st message completed
        messageNum=1;
      } else if (messageNum==1) { // 2nd message completed
        decodeMessages();
        messageNum=-1;
        interruptCallback(false);
      }
    }
    bitPos=0; // Nachricht begonnen
    return;
  }

  if (messageNum<0) return; // ignore repeated messages
  if (bitPos<0) return; // invalid message, ignored

  if (messageNum==0) {
    message1Bits[bitPos]=(duration>=3300); // 2.2ms=LOW, 4.4ms = HIGH bits
  } else {
    message2Bits[bitPos]=(duration>=3300); // 2.2ms=LOW, 4.4ms = HIGH bits
  }
  bitPos++;
  if (bitPos>36) bitPos=-1; // message too long -> invalid
}
