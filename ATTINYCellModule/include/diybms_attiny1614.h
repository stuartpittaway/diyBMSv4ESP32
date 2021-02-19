#if defined(__AVR_ATtiny1614__)

#ifndef DIYBMS_ATTINY1614_H

#define DIYBMS_ATTINY1614_H

#pragma once

#if defined(DIYBMSMODULEVERSION) && DIYBMSMODULEVERSION < 450
#error Incorrect value for DIYBMSMODULEVERSION should be 450 or higher for tiny1614
#endif

#if !(F_CPU == 4000000)
#error Processor speed should be 4Mhz
#endif

/*
#if !defined(ATTINY_CORE)
#error Expected ATTINYCORE
#endif
*/

#include <Arduino.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

/*
This class wraps the hardware pins of DIYBMS away from the core logic/code
if you are porting to another chipset, clone this class and modify it.
*/
class diyBMSHAL
{
public:
  static void ConfigurePorts();

  static void ResumePWM()
  {
    // Switch ON Bit 1 – OCIE1A: Timer/Counter, Output Compare A Match Interrupt
    // Enable
    // TIMSK1 |= _BV(OCIE1A);
    interrupts();
  }

  static void StopTimer1()
  {
    // Switch OFF Bit 1 – OCIE1A: Timer/Counter, Output Compare A Match
    // Interrupt Enable
    // TIMSK1 &= (~_BV(OCIE1A));
  }

  static void StartTimer1()
  {
    // Normal port operation, OC1A/OC1B disconnected
    // Bits 1:0 – WGMn[1:0]: Waveform Generation Mode
    // COM1A1 COM1A0 COM1B1 COM1B0 – – WGM11 WGM10
    //        CCCC--WW
    // TCCR1A = B00000000;

    // CTC (Clear Timer on Compare) = mode 4 = 0100
    // TOP = OCR1A
    // Bits 2:0 – CSn[2:0]: Clock Select = Prescale 64
    // ICNC1 ICES1 – WGM13 WGM12 CS12 CS11 CS10
    //        II-WWCCC
    // TCCR1B = B00001011;

    // Prescaler 8Mhz/64 = 125000 counts per second, call ISR 1000 times per
    // second
    // Prescaler 2Mhz/64 = 31250 counts per second, call ISR 1000 times per
    // second - roughly, as rounding error of 31.25
    // OCR1A = (F_CPU / 64) / 1000;

    ResumePWM();
  }

  static void PausePWM()
  {
    StopTimer1();
    DumpLoadOff();
  }

  static void SetPrescaler()
  {
    // This isn't needed for tiny1614, chip runs at 4Mhz internal clock (to
    // allow down to 1.8V operation)
  }

  static inline void DumpLoadOn() { PORTB.OUTSET = PIN1_bm; }

  static inline void DumpLoadOff() { PORTB.OUTCLR = PIN1_bm; }

  static void ReferenceVoltageOn()
  {
    // Switch REFERENCE VOLTAGE and ENABLE pins ON

    // Ref voltage ON (PA1)
    PORTA.OUTSET = PIN1_bm;
    // PB0 (ENABLE)
    PORTB.OUTSET = PIN0_bm;

    // allow reference voltage to stabilize
    delayMicroseconds(50);
  }

  static void ReferenceVoltageOff()
  {
    // Ref voltage ON (PA1)
    PORTA.OUTCLR = PIN1_bm;
    // PB0 (ENABLE)
    PORTB.OUTCLR = PIN0_bm;
  }

  static inline void NotificationLedOn() { PORTA.OUTSET = PIN6_bm; }

  static inline void NotificationLedOff() { PORTA.OUTCLR = PIN6_bm; }

  // static void SpareOn() { PORTA.OUTSET = PIN2_bm; }

  // static void SpareOff() { PORTA.OUTCLR = PIN2_bm; }

  static inline void FlushSerial0() { Serial.flush(); }

  static inline void DisableSerial0TX()
  {
    //On tiny1614 this saves about 10mA of current
    USART0.CTRLB &= ~(USART_TXEN_bm); /* Transmitter Enable bit mask. */
  }

  static inline void EnableSerial0TX()
  {
    //When the transmitter is disabled, it will no longer override the TXD pin, and the pin
    //direction is automatically set as input by hardware, even if it was configured as output by the user
    PORTB.DIRSET = PIN2_bm;
    USART0.CTRLB |= USART_TXEN_bm; /* Transmitter Enable bit mask. */
  }

  static void EnableSerial0()
  {
    //USART0.CTRLB |= USART_RXEN_bm | USART_TXEN_bm;
  }

  static void DisableSerial1()
  {
    // Not applicable on TINY1614 (only 1 UART)
  }

  static void EnableSerial1()
  {
    // Not applicable on TINY1614 (only 1 UART)
  }

  //The Start-of-Frame Detection feature enables the USART to wake up from Standby Sleep mode upon data reception.
  static inline void EnableStartFrameDetection()
  {
    USART0.CTRLB |= USART_SFDEN_bm;
  }

  static void SetWatchdog8sec();

  static uint16_t BeginADCReading(uint8_t adcmode);

  static void Sleep()
  {
    interrupts();

    // Switch of TX - save power
    diyBMSHAL::DisableSerial0TX();

    //RUNSTBY

    //Standby mode is needed to allow the "USART Start-of-Frame interrupts" to wake CPU

    // Wake up on Serial port RX
    diyBMSHAL::EnableStartFrameDetection();

    set_sleep_mode(SLEEP_MODE_STANDBY);
    sleep_enable();
    sleep_cpu();

    //Snoring can be heard at this point....

    sleep_disable();
  }

  static void SelectCellVoltageChannel()
  {
    // Nothing to do... ADC1 is fixed to read the cell voltage
  }

  static inline void SelectInternalTemperatureChannel()
  {
    // PA7
    ADC0.MUXPOS = ADC_MUXPOS_enum::ADC_MUXPOS_AIN7_gc;
  }

  static inline void SelectExternalTemperatureChannel()
  {
    // PA3
    ADC0.MUXPOS = ADC_MUXPOS_enum::ADC_MUXPOS_AIN3_gc;
  }

  static void double_tap_Notification_led();
};

#endif

#endif
