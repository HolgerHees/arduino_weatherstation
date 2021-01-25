// battery.h
#ifndef BATTERY_SENSOR_h
#define BATTERY_SENSOR_h

struct BatteryValues { 
  uint16_t voltage;
  int16_t current;
};

class BatterySensor
{
  private:
    static bool connected;

  public:
    static void init(int8_t sclPin,int8_t sdaPin);
    static BatteryValues measure();
}; 
#endif
