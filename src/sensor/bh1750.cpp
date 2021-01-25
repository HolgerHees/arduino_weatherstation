// bh1750.cpp
#include "Arduino.h"
#include "../log.h"
#include "bh1750.h"

#include <Wire.h>

#define BH_1750 0x23

//#include <BH1750.h>

//BH1750 lightMeter;

bool BH1750Sensor::connected;
int8_t BH1750Sensor::measuringTimeFactor;
BH1750Sensor::BH1750Mode BH1750Sensor::mode;

// https://github.com/claws/BH1750
void BH1750Sensor::init(int8_t sclPin,int8_t sdaPin)
{  
  Wire.begin();
  
  connected = setResolutionMode(BH1750_ONE_TIME_HIGH_RES_MODE);//(bool) lightMeter.begin(BH1750::ONE_TIME_HIGH_RES_MODE,0x23, &Wire);
  if (!connected) 
  {
    DEBUG_PRINTLN(F("BH1750 ERR"));
    return;
  }
}

uint32_t BH1750Sensor::measure()
{
  if( !connected )
  {
    return;
  }
  
  //powerUp();

  // https://github.com/hexenmeister/AS_BH1750
  // https://cdn-reichelt.de/documents/datenblatt/A300/SENSOR_BH1750.pdf

  // 2 step measuring to have a wider range from 0.11lux to 100000lux
  setMeasuringSensity(BH1750_MTREG_DEFAULT); 
  setResolutionMode(BH1750_CONTINUOUS_LOW_RES_MODE);
  delay(24); // readtime continues LowResMode
  uint16_t rawLux = readBH1750();
  
  if(rawLux<10) {
    // very dark
    // measure again with maximum sensity
    setMeasuringSensity(BH1750_MTREG_MAX); 
    setResolutionMode(BH1750_ONE_TIME_HIGH_RES_MODE_2);
    //delay(120*3.68);
    //delay(120*2.68); // additional delay needed ???
  }
  else if(rawLux<32767) {
    // a little bit lighter
    // until here the 0,5 lx mode with normal sensity is enough.
    setMeasuringSensity(BH1750_MTREG_DEFAULT); 
    setResolutionMode(BH1750_ONE_TIME_HIGH_RES_MODE_2);
  }
  else if(rawLux<60000) {
    // high range
    // here we should use the 1 lx mode with normal sensity
    setMeasuringSensity(BH1750_MTREG_DEFAULT); 
    setResolutionMode(BH1750_ONE_TIME_HIGH_RES_MODE);
  }
  else
  {
    // very bright
    setMeasuringSensity(BH1750_MTREG_MIN+1); // min not possible. A bug produces then wrong results
    setResolutionMode(BH1750_ONE_TIME_HIGH_RES_MODE);
  }
  
  delay(180);
  rawLux = readBH1750();

  float level = (rawLux/1.2)/measuringTimeFactor;
  
  if( mode == BH1750_ONE_TIME_HIGH_RES_MODE_2 || mode == BH1750_CONTINUOUS_HIGH_RES_MODE_2 )
  {
    level = level / 2.0;
  }

  //powerDown();
  
  return (uint32_t) round(level);
}

//void BH1750Sensor::powerDown(){
//  writeBH1750(POWER_DOWN);
//}

//void BH1750Sensor::powerUp(){
//  writeBH1750(POWER_ON);
//  setMode();
//}

//void BH1750Sensor::dataRegReset(){
//  writeBH1750(DATA_REG_RESET);
//}

bool BH1750Sensor::setResolutionMode(BH1750Sensor::BH1750Mode mode){
  bool result = writeBH1750(mode);
  delay(5); // without a delay the ode is not correctly activated
  return result;
}

bool BH1750Sensor::setMeasuringSensity(int8_t sensity){
  measuringTimeFactor = (float)BH1750_MTREG_DEFAULT / sensity;
  
  byte mt = sensity;
  byte highByteMT = ((mt>>5) | 0b01000000);
  byte lowByteMT = (mt & 0b01111111);
  lowByteMT |= 0b01100000;
  writeBH1750(highByteMT);
  return writeBH1750(lowByteMT);
}

uint16_t BH1750Sensor::readBH1750(){
  uint8_t MSbyte, LSbyte;
  Wire.requestFrom(BH_1750, 2);
  if(Wire.available()){
    MSbyte=Wire.read();
    LSbyte=Wire.read(); 
  }
  return ((MSbyte<<8) + LSbyte);
}

bool BH1750Sensor::writeBH1750(byte val){
  Wire.beginTransmission(BH_1750);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}
