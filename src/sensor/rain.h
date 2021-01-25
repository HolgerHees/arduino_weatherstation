// rain.h
#ifndef RAIN_SENSOR_h
#define RAIN_SENSOR_h

class RainSensor
{
  private:
    static int8_t measurePin;
    static int8_t chargePin;

  public:
    static void init(int8_t measurePin, int8_t chargePin);
    static int32_t measure();
}; 
#endif
