// rain.h
#ifndef POWERSAVE_h
#define POWERSAVE_h

class Powersave
{
  public:
    static void init();
    static void sleep(int seconds);
    static void wakeupTransport();
    static void suspendTransport();
    static void interruptTriggered(bool stayIdle);

    static volatile bool wdtTriggered;
  private:
    static volatile int sleepMode;
    static volatile unsigned long triggeredMillis;
}; 

#endif
