/*
  ____  ____  _  _  ____  __  __  ___    _  _  __
 (  _ \(_  _)( \/ )(  _ \(  \/  )/ __)  ( \/ )/. |
  )(_) )_)(_  \  /  ) _ < )    ( \__ \   \  /(_  _)
 (____/(____) (__) (____/(_/\/\_)(___/    \/   (_)

DIYBMS V4.5 +
CELL MODULE FOR tinyAVR2

(c)2019-2021 Stuart Pittaway

COMPILE THIS CODE USING PLATFORM.IO AND VSCODE

LICENSE
Attribution-NonCommercial-ShareAlike 2.0 UK: England & Wales (CC BY-NC-SA 2.0
UK)
https://creativecommons.org/licenses/by-nc-sa/2.0/uk/

* Non-Commercial — You may not use the material for commercial purposes.
* Attribution — You must give appropriate credit, provide a link to the license,
and indicate if changes were made.
  You may do so in any reasonable manner, but not in any way that suggests the
licensor endorses you or your use.
* ShareAlike — If you remix, transform, or build upon the material, you must
distribute your
  contributions under the same license as the original.
* No additional restrictions — You may not apply legal terms or technological
measures
  that legally restrict others from doing anything the license permits.
*/
/*

HARDWARE ABSTRACTION CODE FOR tinyAVR2

*/

#include <Arduino.h>

#if defined(__AVR_ATtinyx24__)

#include "diybms_tinyAVR2.h"


#ifdef MILLIS_USE_TIMERA0
#error "This sketch takes over TCA0 - please use a different timer for millis"
#endif
#ifndef MILLIS_USE_TIMERB0
  #error "This sketch is written for use with TCB0 as the millis timing source"
#endif


void diyBMSHAL::FlashNotificationLed(size_t times, uint32_t milliseconds)
{
  for (size_t i = 0; i < times; i++)
  {
    NotificationLedOn();
    delay(milliseconds);
    NotificationLedOff();
    delay(milliseconds);
  }
}

void diyBMSHAL::PowerOn_Notification_led()
{
  FlashNotificationLed(4, 150);
}

void diyBMSHAL::double_tap_Notification_led()
{
  FlashNotificationLed(2, 50);
}

void diyBMSHAL::ConfigurePorts()
{
  takeOverTCA0();

  // pin out
  // https://github.com/SpenceKonde/megaTinyCore/blob/master/megaavr/extras/ATtiny_x24.md

  // Datasheet
  // *********
  // AVR PIN / ARDUINO PIN MAPPING
  // PB0 /7= ENABLE
  // PB1 /6= DUMP LOAD ENABLE
  // PB2 /5= TXD
  // PB3 /4= RXD
  // PA0 /11= RESET
  // PA1 /8= REF_ENABLE
  // PA2 /9= NOT CONNECTED
  // PA3 /10= EXTERNAL TEMP SENSOR (ADC) (ADC0=AIN3)
  // PA4 /0= VOLTAGE INPUT (ADC) (ADC0=AIN4)  **NO LONGER USED**
  // PA5 /1= VREFERENCE (ADC) (VREFA/ ADC0=AIN5)
  // PA6 /2= NOTIFICATION LED
  // PA7 /3= INTERNAL TEMP SENSOR (ADC)(ADC0=AIN7)

  // PA3 and PA7 use the VCC voltage as a reference - not the PCB volt reference!!

  // For lowest power consumption, disable the digital input buffer of unused
  // pins and pins that are used as analog inputs or outputs.

  // Set Port A digital outputs

  // PA1 /8= REF_ENABLE
  // PA6 /2= NOTIFICATION LED
  // PA2 /9 = NOT CONNECTED (SPARE)
  PORTA.DIRSET = PIN1_bm | PIN6_bm | PIN2_bm;

  // Set Port B digital outputs
  // PB0 /7= ENABLE
  // PB1 /6= DUMP LOAD ENABLE
  // PB2 /TX as output
  PORTB.DIRSET = PIN0_bm | PIN1_bm | PIN2_bm;
  // Set RX as input
  PORTB.DIRCLR = PIN3_bm;

  // Set Port A analogue inputs
  // PA3 /10= EXTERNAL TEMP SENSOR (ADC)
  // PA4 /0= VOLTAGE INPUT (ADC)
  // PA7 /3= INTERNAL TEMP SENSOR (ADC)
  PORTA.DIRCLR = PIN3_bm | PIN7_bm;

  /* The digital input buffer for pin n can be disabled by writing the
  INPUT_DISABLE setting to ISC. This can reduce power consumption and may reduce
  noise if the pin is used as analog input. While configured to INPUT_DISABLE,
  bit n in PORTx.IN will not change since the input synchronizer is disabled.
  */
  PORTA.PIN0CTRL = PORT_ISC_enum::PORT_ISC_INPUT_DISABLE_gc;
  PORTA.PIN1CTRL = PORT_ISC_enum::PORT_ISC_INPUT_DISABLE_gc;
  PORTA.PIN2CTRL = PORT_ISC_enum::PORT_ISC_INPUT_DISABLE_gc;
  PORTA.PIN3CTRL = PORT_ISC_enum::PORT_ISC_INPUT_DISABLE_gc;
  PORTA.PIN4CTRL = PORT_ISC_enum::PORT_ISC_INPUT_DISABLE_gc;
  PORTA.PIN5CTRL = PORT_ISC_enum::PORT_ISC_INPUT_DISABLE_gc;
  PORTA.PIN6CTRL = PORT_ISC_enum::PORT_ISC_INPUT_DISABLE_gc;
  PORTA.PIN7CTRL = PORT_ISC_enum::PORT_ISC_INPUT_DISABLE_gc;

  PORTB.PIN0CTRL = PORT_ISC_enum::PORT_ISC_INPUT_DISABLE_gc;
  PORTB.PIN1CTRL = PORT_ISC_enum::PORT_ISC_INPUT_DISABLE_gc;

  // step 1: Enable ADC
  ADC0.CTRLA = ADC_ENABLE_bm;
  // PRESC[3:0]
  // DIV16 = 5Mhz/2 = 2500000hz
  // Max. conversion rate up to 375 ksps at 12-bit resolution
  ADC0.CTRLB = ADC_PRESC_enum::ADC_PRESC_DIV2_gc;
  // SAMPDUR[7:0]
  ADC0.CTRLE = 128;
  // WINSRC / WINCM[2:0]
  ADC0.CTRLD = 0;
  ADC0.PGACTRL = 0;

  // Set pins to initial state
  DumpLoadOff();
  ReferenceVoltageOff();
  TemperatureVoltageOff();
  NotificationLedOff();

  // More power saving changes
  // EnableSerial0();
}

uint16_t diyBMSHAL::BeginADCReading(uint8_t mode)
{
  uint16_t value = 0;

  // RUNSTDBY / LOWLAT / ENABLE
  // Enable ADC
  ADC0.CTRLA = ADC_ENABLE_bm;

  // TIMEBASE[4:0] / REFSEL[2:0]
  ADC0.CTRLC = TIMEBASE_1US | ADC_REFSEL_enum::ADC_REFSEL_VDD_gc; // FOR READING VREF

  // Take multiple samples (over sample)
  ADC0.COMMAND = ADC_MODE_BURST_SCALING_gc | ADC_START_IMMEDIATE_gc;
  while (!(ADC0.INTFLAGS & ADC_RESRDY_bm))
    ;
  value = (uint16_t)ADC0.RESULT;

  // Switch off ADC
  ADC0.CTRLA &= ~ADC_ENABLE_bm;

  return value;
}

#endif