#include "Arduino.h"
#include "log.h"
#include "helper.h"

#define SAMPLE_NUMBER 10
#define MAX_ADC 1024.0F

float Helper::roundFloat(float value)
{
    return round( value * 10.0) / 10.0;  
}

long Helper::readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1125300L / result; // Back-calculate AVcc in mV
  return result;
}

float Helper::measureVoltage(int8_t measurePin, float maxVoltage)
{
  /*float adcAverage  = 0;
  //DEBUG_PRINTLN("-----");
  for (int i = 0; i < SAMPLE_NUMBER; i++  ) 
  {
    int tmp = analogRead(messurePin);
    adcAverage += tmp;
    delay(10);
    //DEBUG_PRINTLN(tmp);
  }
  adcAverage /= SAMPLE_NUMBER;
  
  //Vout = (Vmax x R2) / (R1 + R2)
  //R2 = (Vout * R1) / (Vmax - Vout);
  
  //float Vout = ( adcAverage * maxVoltage ) / MAX_ADC;
  //float R = ( Vout * balanceRegistor ) / (maxVoltage - Vout);
  
  //DEBUG_PRINT("adcAverage: ");
  //DEBUG_PRINTLN(adcAverage);

  float messuredVoltage = ( adcAverage * maxVoltage ) / MAX_ADC;*/

  analogRead(measurePin);
  delay(10);
  analogRead(measurePin);
  delay(10);
  float adcValue = analogRead(measurePin);
  delay(10);

  float measuredVoltage = ( adcValue * maxVoltage ) / MAX_ADC;

  return measuredVoltage;
}

long Helper::measureResistor(int8_t measurePin, int8_t chargePin, float balanceRegistor )
{
  digitalWrite(chargePin, HIGH);
  
  float maxVoltage = Helper::readVcc() / 1000.0F;
  
  //DEBUG_PRINTLN(maxVoltage);
  
  float measuredResistorVoltage = Helper::measureVoltage(measurePin,maxVoltage);
  
  digitalWrite(chargePin, LOW);
  
  //R2= R1 * buffer;
  //R1= R2 / buffer;
  long measuredResistor = (long) ( balanceRegistor / ((maxVoltage/measuredResistorVoltage) - 1.0F) );
  
  /*DEBUG_PRINT("MVol: ");
  DEBUG_PRINT(messuredResistorVoltage);
  DEBUG_PRINT(", MRes: ");
  DEBUG_PRINT(messuredResistor);
  DEBUG_PRINTLN("");*/

  return measuredResistor;
}

