// bh1750.h
#ifndef BH1750_SENSOR_h
#define BH1750_SENSOR_h

// MTreg Values
// Default
#define BH1750_MTREG_DEFAULT 69
// Sensitivity : default = 0.45
#define BH1750_MTREG_MIN 31
// Sensitivity : default = 3.68
#define BH1750_MTREG_MAX 254

#define DATA_REG_RESET 0b00000111
#define POWER_DOWN 0b00000000
#define POWER_ON 0b00000001

class BH1750Sensor
{
  private:
    static bool connected;
    static int8_t measuringTimeFactor;
    static enum BH1750Mode {
      BH1750_CONTINUOUS_HIGH_RES_MODE = 0b00010000,
      BH1750_CONTINUOUS_HIGH_RES_MODE_2 = 0b00010001,
      BH1750_CONTINUOUS_LOW_RES_MODE = 0b00010011,
      BH1750_ONE_TIME_HIGH_RES_MODE = 0b00100000,
      BH1750_ONE_TIME_HIGH_RES_MODE_2 = 0b00100001,
      BH1750_ONE_TIME_LOW_RES_MODE = 0b00100011
    } mode;

  public:
    static void init(int8_t sdaPin,int8_t sclPin);
    static uint32_t measure();
    
    //static void powerDown();
    //static void powerUp();
    //static void dataRegReset();
    static bool setResolutionMode(BH1750Mode mode);
    static bool setMeasuringSensity(int8_t sensity);
    static uint16_t readBH1750();
    static bool writeBH1750(byte val);
}; 
#endif
