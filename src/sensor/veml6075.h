// veml6075.h
#ifndef VEML6075_SENSOR_h
#define VEML6075_SENSOR_h

struct VEML6075Values { 
  float uvA;
  float uvB; 
};

class VEML6075Sensor
{
  private:
    static bool connected;

  public:
    static void init(int8_t sdaPin,int8_t sclPin);
    static VEML6075Values measure();
}; 
#endif
