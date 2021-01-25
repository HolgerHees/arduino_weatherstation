#ifndef HELPER_h
#define HELPER_h

class Helper
{
  public:
    static float Helper::roundFloat(float value);
    static long readVcc();
    static float measureVoltage(int8_t measurePin, float maxVoltage);
    static long measureResistor(int8_t measurePin, int8_t chargePin, float balanceRegistors);
}; 
#endif
