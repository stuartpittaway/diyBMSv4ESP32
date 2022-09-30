/*
  ____  ____  _  _  ____  __  __  ___    _  _  __
 (  _ \(_  _)( \/ )(  _ \(  \/  )/ __)  ( \/ )/. |
  )(_) )_)(_  \  /  ) _ < )    ( \__ \   \  /(_  _)
 (____/(____) (__) (____/(_/\/\_)(___/    \/   (_)

DIYBMS V4.0
CELL MODULE FOR ATTINY841

(c)2019/2020 Stuart Pittaway

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

HARDWARE ABSTRACTION CODE FOR ATTINY841

  PIN MAPPINGS
  Diagram
  https://github.com/SpenceKonde/ATTinyCore/blob/master/avr/extras/ATtiny_x41.md

  PA1 = PIN 12 SERIAL TRANSMIT (TXD0)
  PA2 = PIN 11 SERIAL RECEIVE (RXD0)

FOR MODULE VERSION 400,410,420,421....
  PA3 = DUMP LOAD ENABLE / PIN 10 /  ARDUINO PIN 7/A3 / TOCC2
  PA4 = ADC4 PIN 9 ARDUINO PIN 6/A4 = ON BOARD TEMP sensor
  PA5 = SERIAL PORT 1 TXD1 - NOT USED (BLUE LED ON <V430 BOARDS AND EXT TEMP SENSOR ON >=430)
  PA6 = Notification LED / PIN 7 / ARDUINO PIN 4/A6
  PA7 = ADC7 = PIN 6 = ARDUINO PIN 3/A7 = 2.048V REFERENCE ENABLE
  PB2 = ADC8 PIN 5 ARDUINO PIN 2/A8 = VOLTAGE reading
  PB0 = ADC11 PIN 2 ARDUINO PIN 0/A11 = REMOTE TEMP sensor = XTAL
  PB1 = ADC10 PIN 3 ARDUINO PIN 1/A10 = SPARE INPUT/OUTPUT = XTAL

  ATTiny841 data sheet
  http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-8495-8-bit-AVR-Microcontrollers-ATtiny441-ATtiny841_Datasheet.pdf

V440 pin mappings
PA0 = EXT REFERENCE
PA6 = Notification LED
PA4 = INT THERMISTOR ADC
PA7 = ENABLE

PB2 = DUMP LOAD ENABLE
PA3 = VOLTAGE ADC
PA5 = EXT THERMISTOR ADC

BLUE LED DOES NOT EXIST ON V440 (Well it does, but the green has been replaced with blue!)
*/
#include <Arduino.h>

#if defined(__AVR_ATtiny841__)

#include "diybms_attiny841.h"

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

#if defined(DIYBMSMODULEVERSION) && DIYBMSMODULEVERSION < 440
void diyBMSHAL::double_tap_blue_led()
{
  BlueLedOn();
  delay(50);
  BlueLedOff();
  delay(50);
  BlueLedOn();
  delay(50);
  BlueLedOff();
}
#endif

void diyBMSHAL::ConfigurePorts()
{
  // PUEA – Port A Pull-Up Enable Control Register (All disabled)
  PUEA = 0;
  // PUEB – Port B Pull-Up Enable Control Register (All disabled)
  PUEB = 0;

// DDRA – Port A Data Direction Register
// When DDAn is set, the pin PAn is configured as an output. When DDAn is cleared, the pin is configured as an input
#if defined(DIYBMSMODULEVERSION) && DIYBMSMODULEVERSION < 440
  // PA3 = dump load enable
  // PA6 = Notification LED (green)
  // PA7 = enable
  // PA5 = BLUE
  DDRA = _BV(DDA3) | _BV(DDA6) | _BV(DDA7) | _BV(DDA5);
  // DDRB – Port B Data Direction Register
  // Spare pin is output
  DDRB = _BV(DDB1);

  // Digital Input Disable Register 0
  // PA4 (ADC4), PB2 (ADC8) and PB0 (ADC11) analog inputs, so disable digital pins to save power
  DIDR0 = _BV(ADC4D);
  DIDR1 = _BV(ADC8D) | _BV(ADC11D);

#else
  // 4.4 boards don't have blue led
  // PB2 = DUMP LOAD ENABLE
  // PA6 = Notification LED (BLUE)
  // PA7 = ENABLE
  DDRA = _BV(DDA6) | _BV(DDA7);
  DDRB = _BV(DDB2);

  // Digital Input Disable Register 0
  // PA3 (ADC3), PA4 (ADC4) and PA5 (ADC5) are analog inputs, so disable digital pins to save power
  DIDR0 = _BV(ADC3D) | _BV(ADC4D) | _BV(ADC5D);
#endif

  // Set the extra high sink capability of pin PA7 is enabled.
  PHDE |= _BV(PHDEA1);

  // Set pins to initial state
  DumpLoadOff();
  ReferenceVoltageOff();

#if defined(DIYBMSMODULEVERSION) && DIYBMSMODULEVERSION < 440
  BlueLedOff();
#endif

  NotificationLedOff();

  // More power saving changes
  EnableSerial0();

  DisableSerial1();
}

void diyBMSHAL::SetWatchdog8sec()
{
  noInterrupts();

  // Setup a watchdog timer for 8 seconds
  MCUSR = 0;
  // Enable watchdog (to reset)
  WDTCSR |= bit(WDE);

  CCP = 0xD8;
  // WDTCSR – Watchdog Timer Control and Status Register
  // We INTERRUPT the chip after 8 seconds of sleeping (not reboot!)
  // WDE: Watchdog Enable
  // Bits 5, 2:0 – WDP[3:0]: Watchdog Timer Prescaler 3 - 0
  WDTCSR = bit(WDIE) | bit(WDP3) | bit(WDP0);
  //| bit(WDE)
  interrupts();
  wdt_reset();
}

uint16_t diyBMSHAL::ReadADC()
{
  // must read ADCL first
  uint8_t low = ADCL;
  return (ADCH << 8) | low;
}

uint16_t diyBMSHAL::BeginADCReading(uint8_t mode)
{
  // ADMUXB – ADC Multiplexer Selection Register
  // Select external AREF pin (internal reference turned off)
  ADMUXB = _BV(REFS2);

  // ADCSRA – ADC Control and Status Register A
  // Consider ADC sleep conversion mode?

  /*
#if !(F_CPU == 8000000)
  //prescaler of 64 = 8MHz/64 = 125KHz.
  ADCSRA |= _BV(ADPS2) | _BV(ADPS1); // | _BV(ADPS0);
#endif

#if !(F_CPU == 2000000)
*/

  // prescaler of 16 = 2MHz/16 = 125000.
  ADCSRA |= _BV(ADPS2);

  //#endif

  // Bit 4 – ADIF: ADC Interrupt Flag
  // Bit 7 – ADEN: ADC Enable
  ADCSRA |= _BV(ADEN) | _BV(ADIF); // enable ADC, turn off any pending interrupt

  // wait for ADC to settle
  // The ADC must be enabled during the settling time.
  // ADC requires a settling time of 1ms before measurements are stable
  delay(1);

  // noInterrupts();
  set_sleep_mode(SLEEP_MODE_IDLE); // IDLE sleep during ADC sample, allowing counters and timers to work
  sleep_enable();

  // start the conversion
  ADCSRA |= _BV(ADSC) | _BV(ADIE);
  // interrupts();
  sleep_cpu();
  sleep_disable();

  // awake again, reading should be done, better make sure maybe the timer interrupt fired
  while (bit_is_set(ADCSRA, ADSC))
  {
  }

  // adc_disable
  ADCSRA &= (~(1 << ADEN));

  return 0;
}

void diyBMSHAL::Sleep()
{

  // Switch of TX - save power
  diyBMSHAL::DisableSerial0TX();

  // Wake up on Serial port RX
  diyBMSHAL::EnableStartFrameDetection();

  // ATTINY841 sleep mode
  byte old_ADCSRA = ADCSRA;
  // For low power applications, before entering sleep, remember to turn off the ADC
  // ADCSRA&=(~(1<<ADEN));
  //  disable ADC
  ADCSRA = 0;

#if defined(DIYBMSMODULEVERSION) && DIYBMSMODULEVERSION < 440
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
#else
  // Using an external crystal so keep it awake - consumes more power (about 0.97mA vs 0.78mA) but module wakes quicker (6 clock cycles)
  set_sleep_mode(SLEEP_MODE_STANDBY);
#endif

  power_spi_disable();
  power_timer0_disable();
  power_timer1_disable();
  power_timer2_disable();
  power_twi_disable();
  power_adc_disable();

  power_usart1_disable();

  // Keep this alive
  // power_usart0_enable();

  // sei();
  interrupts();
  sleep_enable();
  sleep_cpu();

  // Snoring can be heard at this point....

  sleep_disable();

  power_adc_enable();
  power_timer0_enable();
  power_timer1_enable();
  // power_timer2_enable();

  // power_all_enable();

  ADCSRA = old_ADCSRA;
}
#endif
