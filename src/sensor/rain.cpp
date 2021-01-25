// rain.cpp
#include "Arduino.h"
#include "../log.h"
#include "../helper.h"
#include "rain.h"

int8_t RainSensor::measurePin;
int8_t RainSensor::chargePin;

#define BALANCE_RESISTOR 100000.0F

void RainSensor::init(int8_t measurePin, int8_t chargePin)
{
  RainSensor::measurePin = measurePin;
  RainSensor::chargePin = chargePin;

  pinMode (chargePin, OUTPUT);
}

int32_t RainSensor::measure() 
{
  long measuredResistor = Helper::measureResistor(measurePin,chargePin,BALANCE_RESISTOR);

  //DEBUG_PRINT("Rain messuredResistor: ");
  //DEBUG_PRINTLN(messuredResistor);
  
  // maximum is 102.300.096 but to reduce noice we set the upper limit to 10.000.000. It is still enough to detect rain and wet air
  if( measuredResistor > 10000000 ) measuredResistor = 10000000;

  return measuredResistor;
}
