// ntc.h
#ifndef NTC_SENSOR_h
#define NTC_SENSOR_h

class NtcSensor
{
  private:
    static int8_t measurePin;
    static int8_t chargePin;

  public:
    static void init(int8_t measurePin, int8_t chargePin);
    static float measure();
}; 
#endif
