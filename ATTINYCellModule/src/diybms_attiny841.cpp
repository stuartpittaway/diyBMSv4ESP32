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
  PA6 = GREEN_LED / PIN 7 / ARDUINO PIN 4/A6
  PA7 = ADC7 = PIN 6 = ARDUINO PIN 3/A7 = 2.048V REFERENCE ENABLE
  PB2 = ADC8 PIN 5 ARDUINO PIN 2/A8 = VOLTAGE reading
  PB0 = ADC11 PIN 2 ARDUINO PIN 0/A11 = REMOTE TEMP sensor = XTAL
  PB1 = ADC10 PIN 3 ARDUINO PIN 1/A10 = SPARE INPUT/OUTPUT = XTAL

  ATTiny841 data sheet
  http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-8495-8-bit-AVR-Microcontrollers-ATtiny441-ATtiny841_Datasheet.pdf
*/

#include "diybms_attiny841.h"

void DiyBMSATTiny841::double_tap_green_led()
{
  GreenLedOn();
  delay(50);
  GreenLedOff();
  delay(50);
  GreenLedOn();
  delay(50);
  GreenLedOff();
}

#if defined(DIYBMSMODULEVERSION) && DIYBMSMODULEVERSION < 430
void DiyBMSATTiny841::double_tap_blue_led()
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

void DiyBMSATTiny841::ConfigurePorts()
{
  //PUEA – Port A Pull-Up Enable Control Register (All disabled)
  PUEA = 0;
  //PUEB – Port B Pull-Up Enable Control Register (All disabled)
  PUEB = 0;

  //DDRA – Port A Data Direction Register
  //When DDAn is set, the pin PAn is configured as an output. When DDAn is cleared, the pin is configured as an input
#if defined(DIYBMSMODULEVERSION) && DIYBMSMODULEVERSION < 430
  DDRA |= _BV(DDA3) | _BV(DDA6) | _BV(DDA7) | _BV(DDA5);
#else
  //4.3 boards dont have blue led, so don't configure DA5
  DDRA |= _BV(DDA3) | _BV(DDA6) | _BV(DDA7);
#endif
  //DDRB – Port B Data Direction Register
  //Spare pin is output
  DDRB |= _BV(DDB1);

  //Set the extra high sink capability of pin PA7 is enabled.
  PHDE |= _BV(PHDEA1);

  //Set pins to initial state
  DumpLoadOff();
  ReferenceVoltageOff();

#if defined(DIYBMSMODULEVERSION) && DIYBMSMODULEVERSION < 430
  BlueLedOff();
#endif
  GreenLedOff();
}

void DiyBMSATTiny841::SetWatchdog8sec()
{
  //Setup a watchdog timer for 8 seconds
  MCUSR = 0;
  //Enable watchdog (to reset)
  WDTCSR |= bit(WDE);

  CCP = 0xD8;
  // WDTCSR – Watchdog Timer Control and Status Register
  // We INTERRUPT the chip after 8 seconds of sleeping (not reboot!)
  // WDE: Watchdog Enable
  // Bits 5, 2:0 – WDP[3:0]: Watchdog Timer Prescaler 3 - 0
  WDTCSR = bit(WDIE) | bit(WDP3) | bit(WDP0);
  //| bit(WDE)

  wdt_reset();
}

uint16_t DiyBMSATTiny841::ReadADC()
{
  // must read ADCL first
  uint8_t low = ADCL;
  return (ADCH << 8) | low;
}

void DiyBMSATTiny841::BeginADCReading()
{
  //ADMUXB – ADC Multiplexer Selection Register
  //Select external AREF pin (internal reference turned off)
  ADMUXB = _BV(REFS2);

  //ADCSRA – ADC Control and Status Register A
  //Consider ADC sleep conversion mode?
  //prescaler of 64 = 8MHz/64 = 125KHz.
  ADCSRA |= _BV(ADPS2) | _BV(ADPS1); // | _BV(ADPS0);

  //adc_enable();
  //Bit 4 – ADIF: ADC Interrupt Flag
  //Bit 7 – ADEN: ADC Enable
  ADCSRA |= _BV(ADEN) | _BV(ADIF); // enable ADC, turn off any pending interrupt

  // wait for ADC to settle
  // The ADC must be enabled during the settling time.
  // ADC requires a settling time of 1ms before measurements are stable
  delay(1);

  //noInterrupts();
  set_sleep_mode(SLEEP_MODE_IDLE); // IDLE sleep during ADC sample, allowing counters and timers to work
  sleep_enable();

  // start the conversion
  ADCSRA |= _BV(ADSC) | _BV(ADIE);
  //interrupts();
  sleep_cpu();
  sleep_disable();

  // awake again, reading should be done, better make sure maybe the timer interrupt fired
  while (bit_is_set(ADCSRA, ADSC))
  {
  }

  //adc_disable
  ADCSRA &= (~(1 << ADEN));
}

void DiyBMSATTiny841::Sleep()
{
  //ATTINY841 sleep mode
  byte old_ADCSRA = ADCSRA;
  //For low power applications, before entering sleep, remember to turn off the ADC
  //ADCSRA&=(~(1<<ADEN));
  // disable ADC
  ADCSRA = 0;

#if defined(DIYBMSMODULEVERSION) && DIYBMSMODULEVERSION < 430
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
#else
  //Using an external crystal so keep it awake - consumes more power (about 0.97mA vs 0.78mA) but module wakes quicker (6 clock cycles)
  set_sleep_mode(SLEEP_MODE_STANDBY);
#endif

  power_spi_disable();
  power_timer0_disable();
  power_timer1_disable();
  power_timer2_disable();
  power_twi_disable();
  power_adc_disable();

  power_usart1_disable();

  //Keep this alive
  //power_usart0_enable();

  sei();
  interrupts();
  sleep_enable();
  sleep_cpu();

  //Snoring can be heard at this point....

  sleep_disable();

  power_adc_enable();
  power_timer0_enable();
  power_timer1_enable();
  //power_timer2_enable();

  //power_all_enable();

  ADCSRA = old_ADCSRA;
}
