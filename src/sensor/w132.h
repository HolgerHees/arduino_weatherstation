// w132.h
#ifndef W132_SENSOR_h
#define W132_SENSOR_h

struct W132Values { 
  float windSpeed;
  float windGust;
  int16_t windDirection;
//  float temperature;
//  int8_t humidity;
};

class W132Sensor
{
  private:
    static unsigned long lastTrigger;
    
    static byte bitPos;
    static byte messageNum;
    // there are always sent 6 messages. For temperature/humidity, the message is repeatet 6 times. For wind, 2 messages are repeatet 3 times each
    static bool message1Bits[36];
    static bool message2Bits[36];
    
    static volatile float windSpeed; // m/s
    static volatile float windGust; // m/s
    static volatile int16_t windDirection; // °
    //static volatile float temperature; // °C
    //static volatile int8_t humidity; // %

    static void (*interruptCallback)(bool);
    
    static bool verifyChecksum(bool bits[]);
    static void decodeMessages();

  public:
    static void init(int8_t pin,void(*interruptCallback)(bool));

    static void dataHandler();
    
    static W132Values getValues();
//    int _state;
//    int _last;
//    unsigned long _click;
}; 
#endif
