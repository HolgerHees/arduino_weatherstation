// reed.h
#ifndef REED_SENSOR_h
#define REED_SENSOR_h

#include "core/MyMessage.h"

class ReedSensor 
{
  private:
    static int8_t pin;
    static volatile uint8_t impulse_count;

  public:
    static void init(int8_t pin);
    static uint8_t getValue();
    
    static void dataHandler();
}; 
#endif
