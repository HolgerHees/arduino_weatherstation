// bme280.cpp
#include "Arduino.h"
#include "../log.h"
#include "../helper.h"
#include "bme280.h"

#include <Adafruit_BME280.h>

Adafruit_BME280 bme; // I2C
//Adafruit_BME280 bme(BME_CS); // hardware SPI
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

bool BME280Sensor::connected;

void BME280Sensor::init(int8_t sdaPin,int8_t sclPin)
{  
  //unsigned status = bme.begin();
  connected = (bool) bme.begin(0x76, &Wire);
  
  if (!connected) 
  {
    DEBUG_PRINTLN(F("BME280 ERR"));
    //DEBUG_PRINT("SensorID was: 0x"); 
    //DEBUG_PRINTLN(bme.sensorID());
    return;
  }
 
  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X1, // temperature
                  Adafruit_BME280::SAMPLING_X1, // pressure
                  Adafruit_BME280::SAMPLING_X1, // humidity
                  Adafruit_BME280::FILTER_OFF   );

  /*bme.setSampling(Adafruit_BME280::MODE_FORCED,
                Adafruit_BME280::SAMPLING_X2, // temperature
                Adafruit_BME280::SAMPLING_X16, // pressure
                Adafruit_BME280::SAMPLING_X1, // humidity
                Adafruit_BME280::FILTER_X16,
                Adafruit_BME280::STANDBY_MS_0_5);*/
}

BME280Values BME280Sensor::measure()
{
  //unsigned long start = micros();
  
  if( !connected )
  {
    return;
  }

  bme.takeForcedMeasurement();

  float temperature = bme.readTemperature();
  uint8_t humidity = (uint8_t) round( bme.readHumidity() );
  //bme.readAltitude(SEALEVELPRESSURE_HPA);
  float pressure = bme.readPressure() / 100.0; // Pa => hPa;
  
  //unsigned long end = micros();

  //DEBUG_PRINT(end-start);
  //DEBUG_PRINTLN(" micro seconds");
  
  //DEBUG_PRINT(pressure);
  //DEBUG_PRINTLN(" hPa");
  
  temperature = Helper::roundFloat(temperature);
  pressure = Helper::roundFloat(pressure);
  
  return (struct BME280Values) {temperature,humidity,pressure};
}
