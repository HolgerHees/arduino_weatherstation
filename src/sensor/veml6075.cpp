// veml6075.cpp
#include "Arduino.h"
#include "../log.h"
#include "../helper.h"
#include "veml6075.h"

//https://github.com/schizobovine/VEML6075.git
#include <VEML6075.h>

VEML6075 veml6075 = VEML6075();

bool VEML6075Sensor::connected;

void VEML6075Sensor::init(int8_t sclPin,int8_t sdaPin)
{  
  Wire.begin();
  
  connected = (bool) veml6075.begin(VEML6075_IT_100MS,false,true);//0x23, &Wire);
  if (!connected) 
  {
    DEBUG_PRINTLN(F("VEML6075 ERR"));
    return;
  }
}

VEML6075Values VEML6075Sensor::measure()
{
  if( !connected )
  {
    return;
  }

  veml6075.powerUp();
  
  veml6075.poll();

  float uvA = veml6075.getUVA();
  float uvB = veml6075.getUVB();

  veml6075.powerDown();

  if( uvA < 0.0 ) uvA = 0.0;
  else uvA= Helper::roundFloat(uvA);
  
  if( uvB < 0.0 ) uvB = 0.0;
  else uvB= Helper::roundFloat(uvB);
  
  return (struct VEML6075Values) {uvA,uvB};
}
