// reed.cpp
#include "Arduino.h"
#include "../log.h"
#include "reed.h"

int8_t ReedSensor::pin = 0;
volatile uint8_t ReedSensor::impulse_count = 0;

ISR(PCINT2_vect)
{
  ReedSensor::dataHandler();
}

void ReedSensor::init(int8_t pin)
{
  ReedSensor::pin = pin;
  
  impulse_count = 0;
  
  if( pin == 4 )
  {
    cli();
    //PCMSK0 |= bit (PCINT1);   // Pin D9
    //PCIFR  |= bit (PCIF0);    // Clear any outstanding interrupts
    //PCICR  |= bit (PCIE0);    // Enable pin change interrupts for D8 to D13
    PCMSK2 |= bit (PCINT20);  // Pin D4
    PCIFR  |= bit (PCIF2);    // Clear any outstanding interrupts
    PCICR  |= bit (PCIE2);    // Enable pin change interrupts for D0 to D7
    sei();
  }
  else
  {
    DEBUG_PRINTLN(F("REED ERR"));
    return;
  }
}

void ReedSensor::dataHandler() {
  sei();//allow interrupts
  //DEBUG_PRINTLN(F("reed trigger"));
  
  int8_t value = digitalRead(ReedSensor::pin);
  if( value == HIGH )
  {
    ReedSensor::impulse_count ++;
  }
}

uint8_t ReedSensor::getValue()
{
  uint8_t result = impulse_count;
  impulse_count = 0;
  return result;
}
