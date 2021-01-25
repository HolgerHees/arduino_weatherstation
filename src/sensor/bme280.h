// bme280.h
#ifndef BME280_SENSOR_h
#define BME280_SENSOR_h

struct BME280Values { 
  float temperature; // °C
  uint8_t humidity; // %
  float pressure; // hPa
};

class BME280Sensor 
{
  private:
    static bool connected;

  public:
    static void init(int8_t sdaPin,int8_t sclPin);
    static BME280Values measure();
}; 
#endif
