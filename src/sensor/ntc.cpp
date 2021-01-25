// ntc.cpp
#include "Arduino.h"
#include "../log.h"
#include "../helper.h"
#include "ntc.h"

#define BALANCE_RESISTOR 47000.0F
#define BETA_CONST 3950.0F // => (b)eta const => Materialconst value
#define T_0 273.15F // 0 °C in K
#define T_25 298.15F // 25 °C in K
#define R_25 10000.0F // NTC Resistor value at 25°C

int8_t NtcSensor::measurePin;
int8_t NtcSensor::chargePin;

void NtcSensor::init(int8_t measurePin, int8_t chargePin)
{
  NtcSensor::measurePin = measurePin;
  NtcSensor::chargePin = chargePin;

  pinMode (chargePin, OUTPUT);
}

float NtcSensor::measure() 
{
  long R = Helper::measureResistor(measurePin,chargePin,BALANCE_RESISTOR);

  //DEBUG_PRINT("Ntc R: ");
  //DEBUG_PRINTLN(R);

  float temperature;
  
  // Make sure we won't divide by 0 when calculating T: this happens if R_ntc is too small.
  if (R <= (R_25 * exp(-BETA_CONST / T_25)))
  {
    temperature = -20.0;
  }
  else
  {
    //https://www.giangrandi.org/electronics/ntc/ntc.shtml
    //T = 1 / ( (Math.log(R / R_25) / beta) + (1 / T_25) ) - T_0;
    temperature = ( 1.0 / ( ( log(R / R_25) / BETA_CONST) + ( 1.0 / T_25 ) ) ) - T_0;
  }
  
  temperature = Helper::roundFloat(temperature);
  
  return temperature;
}
