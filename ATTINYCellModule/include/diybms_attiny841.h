#ifndef DIYBMS_ATTINY841_H // include guard
#define DIYBMS_ATTINY841_H

//Show error is not targeting ATTINY841
#if !(defined(__AVR_ATtiny841__))
#error Written for ATTINY841 chip
#endif

#include <Arduino.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

/*
This class wraps the hardware pins of DIYBMS away from the core logic/code
if you are porting to another chipset, clone this class and modify it.
*/
class DiyBMSATTiny841
{
public:
   DiyBMSATTiny841() {}
   ~DiyBMSATTiny841() {}
   static void ConfigurePorts();

   static void EnableTOCPMCOE()
   {
      // TOCPMSA1 and TOCPMSA0 – Timer/Counter Output Compare Pin Mux Selection Registers
      // Timer/Counter Output Compare Pin Mux Channel Output Enable
      // Enable TOCC2 to be output
      TOCPMCOE = (1 << TOCC2OE);
   }
   static void DisableTOCPMCOE()
   {
      TOCPMCOE = 0;
   }

   static void GreenLedOn();
   static void GreenLedOff();

   static void DumpLoadOn();
   static void DumpLoadOff();

#if defined(DIYBMSMODULEVERSION) && DIYBMSMODULEVERSION < 430
   static void SparePinOn();
   static void SparePinOff();
#endif

#if defined(DIYBMSMODULEVERSION) && DIYBMSMODULEVERSION < 430
   static void BlueLedOn();
   static void BlueLedOff();
#endif

   static void ReferenceVoltageOn();
   static void ReferenceVoltageOff();

   static void FlushSerial0();

   static void DisableSerial0();
   static void EnableSerial0();

   static void DisableSerial0TX();
   static void EnableSerial0TX();

   static void DisableSerial1();
   static void EnableSerial1();
   /*
   void EnablePinChangeInterrupt();
   void DisablePinChangeInterrupt();
*/
   static void SetWatchdog8sec();

   static uint16_t ReadADC();

   static void BeginADCReading();

   static void Sleep();

   static void SelectCellVoltageChannel();
   static void SelectInternalTemperatureChannel();
   static void SelectExternalTemperatureChannel();
   static void EnableStartFrameDetection();

   static void StopTimer2();
   static void StartTimer2();
   static void SetTimer2Value(uint16_t value);

   static void double_tap_green_led();
#if defined(DIYBMSMODULEVERSION) && DIYBMSMODULEVERSION < 430
   static void double_tap_blue_led();
#endif
};

#endif
