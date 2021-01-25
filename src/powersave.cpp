// rain.cpp
#include "Arduino.h"
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include "log.h"
#include "powersave.h"

//#include "hal/transport/MyTransportHAL.h"
#include "core/MySensorsCore.h"
#include "core/MyMessage.h"
#include "core/MyTransport.h"
#include "core/MyIndication.h"

// http://www.netzmafia.de/skripten/hardware/Arduino/Sleep/index.html

volatile int Powersave::sleepMode;
volatile unsigned long Powersave::triggeredMillis;

volatile bool Powersave::wdtTriggered;

void Powersave::init()
{
  wdtTriggered = false;

  // The following saves some extra power by disabling some peripherals I am not using.
  //ADCSRA = ADCSRA & B01111111; // Disable ADC, ADEN bit7 to 0
  //ACSR = B10000000; // Disable analog comparator, ACD bit7 to 1
  //DIDR0 = DIDR0 | B00111111; // Disable digitale inputbuffer, set analoge input pins 0-5 to 1
}

void Powersave::interruptTriggered(bool stayIdle)
{
  if (stayIdle) sleepMode = SLEEP_MODE_IDLE;
  triggeredMillis = millis();
}

void Powersave::wakeupTransport()
{
#ifndef NO_RADIO
  transportReInitialise();
#endif
}

void Powersave::suspendTransport()
{
#ifndef NO_RADIO
  // Do not sleep if transport not ready
  if (!isTransportReady()) {
    DEBUG_PRINTLN(F("TSP: Wait"));
    const uint32_t sleepEnterMS = millis();
    uint32_t sleepDeltaMS = 0;
    while (!isTransportReady() && (sleepDeltaMS < MY_SLEEP_TRANSPORT_RECONNECT_TIMEOUT_MS)) {
      _process();
      sleepDeltaMS = millis() - sleepEnterMS;
    }

    if( !isTransportReady() )
    {
      DEBUG_PRINTLN(F("TSP: Not ready"));
    }
  }
  // OTA FW feature: do not sleep if FW update ongoing
//#if defined(MY_OTA_FIRMWARE_FEATURE)
//  while (isFirmwareUpdateOngoing() && sleepingTimeMS) {
//    DEBUG_PRINTLN("Wait to finish OTA update");
//    wait(1000ul);
//    sleepingTimeMS = sleepingTimeMS >= 1000ul ? sleepingTimeMS - 1000ul : 1000ul;
//  }
//#endif
  transportDisable();
#endif
}

void Powersave::sleep(int seconds)
{
  setIndication(INDICATION_SLEEP);

  //DEBUG_PRINTLN(" prepare");
  //delay(5000);

  DEBUG_PRINT(F("Sleep "));
  DEBUG_PRINT(seconds);
  DEBUG_PRINTLN(F("s"));
  DEBUG_FLUSH();

  // save WDT settings
  const uint8_t WDTsave = WDTCSR;

  // force initial sleep mode reset
  wdtTriggered = true;

  // ADCSRA = 0;  // disable ADC by setting ADCSRA register to 0
  ADCSRA &= ~(1 << ADEN);     // Ensure AD control register is disable before power disable
  //power_all_disable();
  power_adc_disable();        // Disable the Analog to Digital Converter module
  power_twi_disable();
  //power_usart0_disable();
  //power_spi_disable(); 
  
  for ( int i = 0; i < seconds; i++)
  {
    if( sleepMode != SLEEP_MODE_PWR_DOWN && millis() - triggeredMillis > 100 )
    {
      sleepMode = SLEEP_MODE_PWR_DOWN;
    }

    set_sleep_mode(sleepMode); // Set deepest sleep mode PWR_DOWN

    // https://arduino-projekte.webnode.at/registerprogrammierung/watchdog-interrupt/
    cli(); // disable interrupts for changing the registers

    if ( wdtTriggered )
    {
      wdtTriggered = false;
      
      MCUSR &= ~(1 << WDRF); // Clear the reset flag on the MCUSR, the WDRF bit (bit 3).
      WDTCSR |= (1 << WDCE) | (1 << WDE); // Watchdog change enabled for the next 4 cpu cycles
      WDTCSR  = (0 << WDP3) | (1 << WDP2) | (1 << WDP1) | (0 << WDP0); // Set watchdog prescaler to 128K > is round about 1 second
      WDTCSR |= 1 << WDIE; // Enable the WD interrupt (note: no reset).
      wdt_reset();
    }
    else
    {
      i--;
    }

    sleep_enable();             // Set the SE (sleep enable) bit.

    sleep_bod_disable();        // must be call directly before sleep_cpu

    sei();                      // enable interrupts

    sleep_cpu();                // sleep ***
    
    sleep_disable();            // Clear the SE (sleep enable) bit.
  }

  //power_adc_enable();
  //power_twi_enable();
  power_all_enable();
  
  cli();
  WDTCSR |= (1 << WDCE) | (1 << WDE); // Watchdog change enabled for the next 4 cpu cycles
  WDTCSR = WDTsave; // restore saved WDT settings
  wdt_reset();
  sei();
  
  // enable ADC
  ADCSRA |= (1 << ADEN);

  setIndication(INDICATION_WAKEUP);

  //DEBUG_PRINTLN(F("done"));
}

ISR(WDT_vect)
{
  Powersave::wdtTriggered = true;
  wdt_disable();
}
