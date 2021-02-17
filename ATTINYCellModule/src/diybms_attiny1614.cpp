/*
  ____  ____  _  _  ____  __  __  ___    _  _  __
 (  _ \(_  _)( \/ )(  _ \(  \/  )/ __)  ( \/ )/. |
  )(_) )_)(_  \  /  ) _ < )    ( \__ \   \  /(_  _)
 (____/(____) (__) (____/(_/\/\_)(___/    \/   (_)

DIYBMS V4.0
CELL MODULE FOR ATTINY1614

(c)2019-2021 Stuart Pittaway

COMPILE THIS CODE USING PLATFORM.IO AND VSCODE

LICENSE
Attribution-NonCommercial-ShareAlike 2.0 UK: England & Wales (CC BY-NC-SA 2.0 UK)
https://creativecommons.org/licenses/by-nc-sa/2.0/uk/

* Non-Commercial — You may not use the material for commercial purposes.
* Attribution — You must give appropriate credit, provide a link to the license, and indicate if changes were made.
  You may do so in any reasonable manner, but not in any way that suggests the licensor endorses you or your use.
* ShareAlike — If you remix, transform, or build upon the material, you must distribute your
  contributions under the same license as the original.
* No additional restrictions — You may not apply legal terms or technological measures
  that legally restrict others from doing anything the license permits.
*/
/*

HARDWARE ABSTRACTION CODE FOR ATTINY1614

*/
#if defined(__AVR_ATtiny1614__)

#include "diybms_attiny1614.h"

void diyBMSHAL::double_tap_Notification_led()
{
  NotificationLedOn();
  delay(50);
  NotificationLedOff();
  delay(50);
  NotificationLedOn();
  delay(50);
  NotificationLedOff();
}


void diyBMSHAL::ConfigurePorts()
{
  //PUEA – Port A Pull-Up Enable Control Register (All disabled)
  //PUEA = 0;
  //PUEB – Port B Pull-Up Enable Control Register (All disabled)
  //PUEB = 0;

//DDRA – Port A Data Direction Register
  //4.4 boards don't have blue led
  //PB2 = DUMP LOAD ENABLE
  //PA6 = Notification LED (BLUE)
  //PA7 = ENABLE
  //DDRA = _BV(DDA6) | _BV(DDA7);
  //DDRB = _BV(DDB2);

  //Digital Input Disable Register 0
  //PA3 (ADC3), PA4 (ADC4) and PA5 (ADC5) are analog inputs, so disable digital pins to save power
  //DIDR0 = _BV(ADC3D) | _BV(ADC4D) |_BV(ADC5D);


  //Set the extra high sink capability of pin PA7 is enabled.
  //PHDE |= _BV(PHDEA1);

  //Set pins to initial state
  DumpLoadOff();
  ReferenceVoltageOff();
  NotificationLedOff();
}

void diyBMSHAL::SetWatchdog8sec()
{
  //Setup a watchdog timer for 8 seconds
  //MCUSR = 0;
  //Enable watchdog (to reset)
  //WDTCSR |= bit(WDE);

  //CCP = 0xD8;
  // WDTCSR – Watchdog Timer Control and Status Register
  // We INTERRUPT the chip after 8 seconds of sleeping (not reboot!)
  // WDE: Watchdog Enable
  // Bits 5, 2:0 – WDP[3:0]: Watchdog Timer Prescaler 3 - 0
  //WDTCSR = bit(WDIE) | bit(WDP3) | bit(WDP0);
  //| bit(WDE)

  wdt_reset();
}

uint16_t diyBMSHAL::ReadADC()
{

  return 0;
}

void diyBMSHAL::BeginADCReading()
{
  
}

void diyBMSHAL::Sleep()
{
  
}
#endif