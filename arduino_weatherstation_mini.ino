#include <Arduino.h>
#include "src/config.h"

//#define MY_DEBUG

//#define NO_RADIO
//#define NO_CUSTOM_SLEEP
//#define NO_BATTERY_SENSOR
//#define NO_REED_SENSOR
//#define NO_RAIN_SENSOR
//#define NO_W132_SENSOR
//#define NO_BME280_SENSOR 
//#define NO_NTC_SENSOR
//#define NO_BH1750_SENSOR
//#define NO_VEML6075_SENSOR
//#define NO_RAIN_HEATER

#ifndef NO_RADIO
//#define MY_TRANSPORT_WAIT_READY_MS 10000
#define MY_SOFTSPI
#define MY_SOFT_SPI_SCK_PIN (14)
#define MY_SOFT_SPI_MISO_PIN (12)
#define MY_SOFT_SPI_MOSI_PIN (11)
#define MY_RADIO_RFM69
#define MY_RFM69_NEW_DRIVER
#define MY_RFM69_ENABLE_ENCRYPTION 
//#define MY_DEBUG_VERBOSE_RFM69
#define MY_RFM69_IRQ_PIN 2
#define MY_RFM69_IRQ_NUM 0
#define MY_RFM69_CS_PIN 10
#define MY_RFM69_FREQUENCY (RFM69_868MHZ) // RFM69_868MHZ is default
#define MY_RFM69_NETWORKID 100
//#define MY_SIGNAL_REPORT_ENABLED
#define MY_RFM69_TX_POWER_DBM (5) // 1mW - 25mW, 5 is default
//#define MY_RFM69_ATC_TARGET_RSSI_DBM (-80)  // target RSSI -70dBm
#define MY_RFM69_MAX_POWER_LEVEL_DBM (10)

#define MY_NODE_ID 1
#define MY_PARENT_NODE_ID 0
#define MY_PARENT_NODE_IS_STATIC
#else
#define MY_GATEWAY_SERIAL
#endif

//#define MY_DISABLED_SERIAL
#define MY_SPLASH_SCREEN_DISABLED
#define MY_CUSTOM_WDT_VECT

#include <MySensors.h> 

#include "src/log.h"
//#include "src/message.h"
#include "src/powersave.h"
#include "src/rain_heater.h"

#include "src/sensor/w132.h"
#include "src/sensor/reed.h"
#include "src/sensor/rain.h"
#include "src/sensor/bme280.h"
#include "src/sensor/bh1750.h"
#include "src/sensor/veml6075.h"
#include "src/sensor/ntc.h"
#include "src/sensor/battery.h"

#define PIN_LED 13

#define PIN_W132_DATA 3

#define PIN_RAIN_REED_IMPULE 4

#define PIN_RAIN_HEATER 7

#define PIN_RAIN_DETECTOR_CHARGE 8
#define PIN_RAIN_DETECTOR_MESSUREMENT A1

#define PIN_SOLAR_POWER_CHARGE 9
#define PIN_SOLAR_POWER_MESSUREMENT A2

//#define PIN_RFM69_IRQ 3
//#define PIN_RFM69_NSS 10
//#define PIN_RFM69_MOSI 11
//#define PIN_RFM69_MISO 12
//#define PIN_RFM69_SCK 13

#define PIN_I2C_SDA A4
#define PIN_I2C_SCL A5

#ifndef NO_BATTERY_SENSOR
#define SENSOR_ID_BATTERY_VOLTAGE 0
MyMessage msgBatteryVoltage(SENSOR_ID_BATTERY_VOLTAGE,V_VOLTAGE);
#define SENSOR_ID_BATTERY_CURRENT 1
MyMessage msgBatteryCurrent(SENSOR_ID_BATTERY_CURRENT,V_CURRENT);
#endif
#ifndef NO_REED_SENSOR
#define SENSOR_ID_RAIN_AMOUNT 2
MyMessage msgRainAmount(SENSOR_ID_RAIN_AMOUNT,V_RAINRATE);
#endif
#ifndef NO_RAIN_SENSOR
#define SENSOR_ID_RAIN_RATE 3
MyMessage msgRainRate(SENSOR_ID_RAIN_RATE,V_RAIN);
#endif
#ifndef NO_RAIN_HEATER
#define SENSOR_ID_RAIN_HEATING 4
MyMessage msgRainHeating(SENSOR_ID_RAIN_HEATING,V_STATUS);
#endif
#ifndef NO_W132_SENSOR
#define SENSOR_ID_WIND_SPEED 5
MyMessage msgWindSpeed(SENSOR_ID_WIND_SPEED,V_WIND);
#define SENSOR_ID_WIND_GUST 6
MyMessage msgWindGust(SENSOR_ID_WIND_GUST,V_GUST);
#define SENSOR_ID_WIND_DIRECTION 7
MyMessage msgWindDirection(SENSOR_ID_WIND_DIRECTION,V_DIRECTION);
#endif
#ifndef NO_BME280_SENSOR
#define SENSOR_ID_TEMPERATURE 8
MyMessage msgTemperature(SENSOR_ID_TEMPERATURE,V_TEMP);
#define SENSOR_ID_HUMIDITY 9
MyMessage msgHumidity(SENSOR_ID_HUMIDITY,V_HUM);
#define SENSOR_ID_PRESSURE 10
MyMessage msgPressure(SENSOR_ID_PRESSURE,V_PRESSURE);
#endif
#ifndef NO_NTC_SENSOR
#define SENSOR_ID_SOLAR_POWER 11
MyMessage msgSolarPower(SENSOR_ID_SOLAR_POWER,V_TEMP);
#endif
#ifndef NO_BH1750_SENSOR
#define SENSOR_ID_LIGHT 12
MyMessage msgLight(SENSOR_ID_LIGHT,V_LEVEL);
#endif
#ifndef NO_VEML6075_SENSOR
#define SENSOR_ID_UVA 13
MyMessage msgUvA(SENSOR_ID_UVA,V_UV);
#define SENSOR_ID_UVB 14
MyMessage msgUvB(SENSOR_ID_UVB,V_UV);
#endif

#define SENSOR_COUNT 15

#define UPDATE_INTERVAL_HEATING_FLAG 15  // Loop Iterations/Minutes
#define FORCE_SEND_MESSAGE_INTERVAL 4    // Loop Iterations/Minutes (0-4) => 5 iterations

int8_t skipCount[SENSOR_COUNT];

void setup()
{
  Serial.begin(115200);
  //UCSR0C = UCSR0C | B00100000; // Even Parity

  pinMode(PIN_LED, OUTPUT);

  // Send the sketch version information to the gateway and Controller
  sendSketchInfo(F("WS"),F("1.0"));

  Powersave::init();

  RainHeater::init(PIN_RAIN_HEATER);

#ifndef NO_BATTERY_SENSOR
  present(SENSOR_ID_BATTERY_VOLTAGE, S_MULTIMETER, F("B-V"));
  present(SENSOR_ID_BATTERY_CURRENT, S_MULTIMETER, F("B-C"));
  BatterySensor::init(PIN_I2C_SDA,PIN_I2C_SCL);
#endif

#ifndef NO_REED_SENSOR
  // Register all sensors to gateway (they will be created as child devices)
  present(SENSOR_ID_RAIN_AMOUNT, S_RAIN, F("R-A"));
  ReedSensor::init(PIN_RAIN_REED_IMPULE);
#endif

#ifndef NO_RAIN_SENSOR
  present(SENSOR_ID_RAIN_RATE, S_CUSTOM, F("R-S"));
  RainSensor::init(PIN_RAIN_DETECTOR_MESSUREMENT,PIN_RAIN_DETECTOR_CHARGE);
#endif

#ifndef NO_RAIN_HEATER
  present(SENSOR_ID_RAIN_HEATING, S_BINARY, F("R-H"));
#endif

#ifndef NO_W132_SENSOR
  present(SENSOR_ID_WIND_SPEED, S_WIND, F("W-C"));
  present(SENSOR_ID_WIND_GUST, S_WIND, F("W-M"));
  present(SENSOR_ID_WIND_DIRECTION, S_WIND, F("W-D"));
  W132Sensor::init(PIN_W132_DATA,Powersave::interruptTriggered);
#endif

#ifndef NO_BME280_SENSOR
  present(SENSOR_ID_TEMPERATURE, S_TEMP, F("T"));
  present(SENSOR_ID_HUMIDITY, S_HUM, F("H"));
  present(SENSOR_ID_PRESSURE, S_BARO, F("P"));
  BME280Sensor::init(PIN_I2C_SDA,PIN_I2C_SCL);
#endif
  
#ifndef NO_NTC_SENSOR
  present(SENSOR_ID_SOLAR_POWER, S_TEMP, F("T-S"));
  NtcSensor::init(PIN_SOLAR_POWER_MESSUREMENT,PIN_SOLAR_POWER_CHARGE);
#endif

#ifndef NO_BH1750_SENSOR
  present(SENSOR_ID_LIGHT, S_LIGHT_LEVEL, F("L"));
  BH1750Sensor::init(PIN_I2C_SDA,PIN_I2C_SCL);
#endif

#ifndef NO_VEML6075_SENSOR
  present(SENSOR_ID_UVA, S_UV, F("U-A"));
  present(SENSOR_ID_UVB, S_UV, F("U-B"));
  VEML6075Sensor::init(PIN_I2C_SDA,PIN_I2C_SCL);
#endif

  DEBUG_PROGRESS(200);
  delay(200);
  DEBUG_PROGRESS(200);
}

// the loop function runs over and over again forever
void loop() {
  DEBUG_PRINTLN(F("LOOP"));

  // Suspend transport again, because it was reavtivated from MySensors during every loop iteration
  Powersave::suspendTransport();

  int8_t messageCount = 0;
  MyMessage* messages[SENSOR_COUNT];

#ifndef NO_BATTERY_SENSOR
  BatteryValues batteryValues = BatterySensor::measure();
#endif

#ifndef NO_REED_SENSOR
  uint8_t impulseCount = ReedSensor::getValue();
  // each impluse count must be send to gateway, this is why we check for "impulseCount > 0"
  if( skipCount[msgRainAmount.getSensor()] == 0 || msgRainAmount.getByte() != impulseCount || impulseCount > 0 ) messages[messageCount++] = &msgRainAmount.set(impulseCount);
  else skipCount[msgRainAmount.getSensor()]--;
#endif

#ifndef NO_RAIN_SENSOR
  int32_t rainRate = RainSensor::measure();
  if( skipCount[msgRainRate.getSensor()] == 0 || msgRainRate.getLong() != rainRate ) messages[messageCount++] = &msgRainRate.set(rainRate);
  else skipCount[msgRainRate.getSensor()]--;
#endif

#ifndef NO_W132_SENSOR
  W132Values w132Values = W132Sensor::getValues();
  // force on >0.1 difference
  if( skipCount[msgWindSpeed.getSensor()] == 0 || abs(msgWindSpeed.getFloat() - w132Values.windSpeed) > 0.1 ) messages[messageCount++] = &msgWindSpeed.set(w132Values.windSpeed,1);
  else skipCount[msgWindSpeed.getSensor()]--;
  // force on >0.1 difference
  if( skipCount[msgWindGust.getSensor()] == 0 || abs(msgWindGust.getFloat() - w132Values.windGust) > 0.1 ) messages[messageCount++] = &msgWindGust.set(w132Values.windGust,1);
  else skipCount[msgWindGust.getSensor()]--;

  // force on difference and if direction is needed
  if( skipCount[msgWindDirection.getSensor()] == 0 || ( (w132Values.windSpeed > 0 || w132Values.windGust > 0) && msgWindDirection.getInt() != w132Values.windDirection ) ) messages[messageCount++] = &msgWindDirection.set(w132Values.windDirection);
  else skipCount[msgWindDirection.getSensor()]--;
#endif

#ifndef NO_BME280_SENSOR
  BME280Values bme280Values = BME280Sensor::measure();
  // force on >0.1 difference
  if( skipCount[msgTemperature.getSensor()] == 0 || abs(msgTemperature.getFloat() - bme280Values.temperature) > 0.1 ) messages[messageCount++] = &msgTemperature.set(bme280Values.temperature,1);
  else skipCount[msgTemperature.getSensor()]--;
  // force on >1 difference
  if( skipCount[msgHumidity.getSensor()] == 0 || abs(msgHumidity.getByte() - bme280Values.humidity) > 1 ) messages[messageCount++] = &msgHumidity.set(bme280Values.humidity);
  else skipCount[msgHumidity.getSensor()]--;
  // force on >0.1 difference
  if( skipCount[msgPressure.getSensor()] == 0 || abs(msgPressure.getFloat() - bme280Values.pressure) > 0.1 ) messages[messageCount++] = &msgPressure.set(bme280Values.pressure,1);
  else skipCount[msgPressure.getSensor()]--;
#endif

#ifndef NO_NTC_SENSOR
  float ntcTemperature = NtcSensor::measure();
  // force on >0.1 difference
  if( skipCount[msgSolarPower.getSensor()] == 0 || abs(msgSolarPower.getFloat() - ntcTemperature) > 0.1 ) messages[messageCount++] = &msgSolarPower.set(ntcTemperature,1);
  else skipCount[msgSolarPower.getSensor()]--;
#endif
  
#ifndef NO_BH1750_SENSOR
  uint32_t lux = BH1750Sensor::measure();
  // force on >5% difference
  if( skipCount[msgLight.getSensor()] == 0 || abs(msgLight.getULong() - lux) > (max(msgLight.getULong(),lux) * 0.05) ) messages[messageCount++] = &msgLight.set(lux);
  else skipCount[msgLight.getSensor()]--;
#endif

#ifndef NO_VEML6075_SENSOR
  VEML6075Values veml6075Values = VEML6075Sensor::measure();
  // force on >5% difference
  if( skipCount[msgUvA.getSensor()] == 0 || abs(msgUvA.getFloat() - veml6075Values.uvA) > (max(msgUvA.getFloat(),veml6075Values.uvA) * 0.05) ) messages[messageCount++] = &msgUvA.set(veml6075Values.uvA,1);
  else skipCount[msgUvA.getSensor()]--;
  // force on >5% difference
  if( skipCount[msgUvB.getSensor()] == 0 || abs(msgUvB.getFloat() != veml6075Values.uvB) > (max(msgUvB.getFloat(),veml6075Values.uvB) * 0.05) ) messages[messageCount++] = &msgUvB.set(veml6075Values.uvB,1);
  else skipCount[msgUvB.getSensor()]--;
#endif

bool transportActive = false;

#ifndef NO_RAIN_HEATER
  bool heatingState = RainHeater::getValue();
  if( heatingState || skipCount[msgRainHeating.getSensor()] == 0 || msgRainHeating.getBool() != heatingState )
  {
    RainHeater::setSuspended(true); // must be disabled. Otherwise wireless communication can be disturbed (sending)

#ifndef NO_CUSTOM_SLEEP
    Powersave::wakeupTransport();
    transportActive = true;
#endif  
    bool successful = requestHeatingDemand(heatingState);

    bool _heatingState = RainHeater::getValue();

    if( heatingState != _heatingState )
    {
#ifndef NO_CUSTOM_SLEEP
      Powersave::suspendTransport();
      transportActive = false;
#endif  

#ifndef NO_BATTERY_SENSOR
      RainHeater::setSuspended(false); // temporary enable for current messurement
      batteryValues = BatterySensor::measure();
      RainHeater::setSuspended(true); // disable again. Otherwise wireless communication can be disturbed (receiving)
#endif
    }

    if( !successful || msgRainHeating.getBool() != _heatingState ) messages[messageCount++] = &msgRainHeating.set(_heatingState);
  }
  else skipCount[msgRainHeating.getSensor()]--;
#endif

#ifndef NO_BATTERY_SENSOR
  // force on >10mV difference
  if( skipCount[msgBatteryVoltage.getSensor()] == 0 || abs(msgBatteryVoltage.getUInt() - batteryValues.voltage) > 10 ) messages[messageCount++] = &msgBatteryVoltage.set(batteryValues.voltage);
  else skipCount[msgBatteryVoltage.getSensor()]--;
  // force on >1mA difference
  if( skipCount[msgBatteryCurrent.getSensor()] == 0 || abs(msgBatteryCurrent.getInt() - batteryValues.current) > 1 ) messages[messageCount++] = &msgBatteryCurrent.set(batteryValues.current);
  else skipCount[msgBatteryCurrent.getSensor()]--;
#endif

  if( messageCount > 0 )
  {
#ifndef NO_CUSTOM_SLEEP
    if( !transportActive ) 
    {
      Powersave::wakeupTransport();
    }
#endif
    
    DEBUG_PRINT(F("MsgCnt: "));
    DEBUG_PRINTLN(messageCount);

    for( int i = 0; i < messageCount; i++ )
    {
      send(*messages[i]);
      skipCount[(*messages[i]).getSensor()] = FORCE_SEND_MESSAGE_INTERVAL;
    }
    
#ifdef MY_SIGNAL_REPORT_ENABLED
    dumpSignalReport();
#endif
  }

#ifndef NO_RAIN_HEATER
  RainHeater::setSuspended(false);
#endif

DEBUG_PROGRESS(10);

#ifndef NO_CUSTOM_SLEEP
  Powersave::suspendTransport();
  
  Powersave::sleep( random( 56, 60 ) );
  //Powersave::sleep( 10 );
  //delay(5000);
#else
  DEBUG_PRINTLN(F("WAIT 10 sec"));
  sleep(10000);
#endif
}

bool requestHeatingDemand(bool heatingState)
{
  // retry 5 times
  for(int i = 0; i < 10;i++)
  {
    if( requestHeatingDemandFromGateway(i) ) return true;
  }

  // request was not sucessful 
  if( heatingState ) RainHeater::setValue(false);

  return false;
}

bool requestHeatingDemandFromGateway(int requestCount)
{
  // set force message count to 0 to force a message if the request is not successful
  skipCount[msgRainHeating.getSensor()] = 0;

  //DEBUG_PRINT(F("REQUEST: "));
  //DEBUG_PRINTLN(requestCount);
  
  request(SENSOR_ID_RAIN_HEATING, V_STATUS,GATEWAY_ADDRESS); // request value
  for(int i = 0; i < 25;i++)                                 // wait for value, max 250ms
  {
    wait(10);                                                // wait 10 ms

    // a successful request will reset the skipCount to FORCE_SEND_MESSAGE_INTERVAL which avoid forced messages
    if( skipCount[msgRainHeating.getSensor()] == FORCE_SEND_MESSAGE_INTERVAL )
    {
      //DEBUG_PRINT(F("ITEM REQUEST SUCCESSFULL AFTER: "));
      //DEBUG_PRINTLN(i);
      return true;
    }
  }

  //DEBUG_PRINTLN(F("ITEM REQUEST NOT SUCCESSFULL"));
  return false;
}

void receive(const MyMessage &message)
{
  //DEBUG_PRINT(F("MESSAGE: sensorId: "));
  //DEBUG_PRINT(message.getSensor());
  //DEBUG_PRINT(F(", type: "));
  //DEBUG_PRINTLN(message.getType());
  
  if( message.getSensor() == SENSOR_ID_RAIN_HEATING && message.getType() == V_STATUS )
  {
    //DEBUG_PRINTLN(F("ITEM RESPONSE"));
    skipCount[msgRainHeating.getSensor()] = FORCE_SEND_MESSAGE_INTERVAL; // no forced message
  
    if( RainHeater::getValue() != message.getBool() )
    {
      // set message to skip an additional message
      msgRainHeating.set(message.getBool());
      
      RainHeater::setValue(message.getBool());
    }
  }
}

void showProgress(int milliseconds)
{
  digitalWrite(PIN_LED, HIGH); // Enable LED

  delay( milliseconds );

  digitalWrite(PIN_LED, LOW);  // Disable LED
}

void dumpSignalReport()
{
    DEBUG_PRINTLN(transportGetSignalReport(SR_UPLINK_QUALITY));
    DEBUG_PRINTLN(transportGetSignalReport(SR_TX_POWER_LEVEL));
    DEBUG_PRINTLN(transportGetSignalReport(SR_TX_POWER_PERCENT));
    // retrieve RSSI / SNR reports from incoming ACK
    DEBUG_PRINTLN(transportGetSignalReport(SR_TX_RSSI));
    DEBUG_PRINTLN(transportGetSignalReport(SR_RX_RSSI));
    DEBUG_PRINTLN(transportGetSignalReport(SR_TX_SNR));
    DEBUG_PRINTLN(transportGetSignalReport(SR_RX_SNR));  
}
