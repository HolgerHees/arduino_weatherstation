// rain_heater.h
#ifndef RAIN_HEATER_h
#define RAIN_HEATER_h

class RainHeater
{
  private:
    static int8_t pin;
    static bool active;
    static bool suspended;
    static int level;

  public:
    static void init(int8_t pin);
    static bool getValue();
    static void setValue(bool state);
    static void setSuspended(bool suspended);
}; 

#endif
