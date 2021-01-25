// rain_heater.cpp
#include "Arduino.h"
#include "log.h"
#include "rain_heater.h"

int8_t RainHeater::pin;
bool RainHeater::active;
bool RainHeater::suspended;
int RainHeater::level;

void RainHeater::init(int8_t pin)
{
  RainHeater::pin = pin;

  pinMode (pin, OUTPUT);
}

bool RainHeater::getValue()
{
  return RainHeater::active;
}

void RainHeater::setValue(bool isActive)
{
  RainHeater::active = isActive;
}

void RainHeater::setSuspended(bool suspended)
{
  if( suspended != RainHeater::suspended )
  {
      int level = ( !suspended && RainHeater::active ) ? HIGH : LOW;
      if( level != RainHeater::level )
      {
          RainHeater::level = level;
          digitalWrite(pin, level);
          
          //delaymicroseconds( level == HIGH ? 250 : 100 );
      }
      RainHeater::suspended = suspended;
  }
}

/*
 * HIGH 1
 * LOW 2
 * HIGH 3
 * LOW 4
 * HIGH 5
 * LOW 6
 * HIGH 7
 * LOW 8
 * HIGH 9
 * 
 * LOW -9
 * LOW -8
 * LOW -7
 * LOW -6
 * LOW -5
 * LOW -4
 * LOW -3
 * LOW -2
 * LOW -1
 * LOW 0
 */
