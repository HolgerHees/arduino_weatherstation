// battery.cpp
#include "Arduino.h"
#include "../log.h"
#include "battery.h"

#include <Wire.h>
#include <INA219_WE.h>
#define I2C_ADDRESS 0x40

// Voltage Meter => INA219
// 12,0mA        => 15,2mA          // Running
// 7,6mA         =>                 // Sleep
#define DEVICE_OFFSET 4.4F          // Offset between Running and Sleep
#define CORRECTION_FACTOR 0.7895F   // Factor between value from voltage meter and value from ina219 sensor 

INA219_WE ina219(I2C_ADDRESS);

bool BatterySensor::connected;

void BatterySensor::init(int8_t sclPin,int8_t sdaPin)
{
  Wire.begin();
  
  connected = ina219.init();
  
  if (!connected) 
  {
    DEBUG_PRINTLN(F("BATTERY ERR"));
    return;
  }
  
  ina219.setADCMode(BIT_MODE_12);
  
  ina219.setPGain(PG_160);
  
  ina219.setBusRange(BRNG_16);

  ina219.setCorrectionFactor(CORRECTION_FACTOR);
}

BatteryValues BatterySensor::measure() 
{
  if( !connected )
  {
      return;
  }
  
  //bool ina219_overflow = false;
  
  ina219.powerUp();
  
  ina219.setMeasureMode(TRIGGERED);
  //delay(50);
  
  ina219.startSingleMeasurement(); // triggers single-shot measurement and waits until completed
  
  //float shuntVoltage_mV = ina219.getShuntVoltage_mV();
  uint16_t voltage  = ina219.getBusVoltage_V() * 1000.0;
  int16_t current = round(ina219.getCurrent_mA() + DEVICE_OFFSET);
  
  //Serial.print(F("current: ")); Serial.println(ina219.getCurrent_mA());
  //Serial.print("shuntVoltage_mV: "); Serial.println(shuntVoltage_mV);
  //Serial.print("ina219_overflow: "); Serial.println(ina219.getOverflow());
  //Serial.print("busVoltage_V: "); Serial.println(busVoltage_V);
  //Serial.print("power_mW: "); Serial.println(ina219.getBusPower());
  
  ina219.powerDown();

  //power_mW = ina219.getBusPower();
  //ina219_overflow = ina219.getOverflow();
  
  /*if(!ina219_overflow){
    Serial.println("Values OK - no overflow");
  }
  else{
    Serial.println("Overflow! Choose higher PGAIN");
  }
  Serial.println();*/
  
  return (struct BatteryValues) {voltage,current};
}
