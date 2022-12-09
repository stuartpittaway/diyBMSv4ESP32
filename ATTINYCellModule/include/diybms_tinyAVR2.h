#if defined(__AVR_ATtinyx24__)


#ifndef DIYBMS_tinyAVR2_H

#define DIYBMS_tinyAVR2_H

#pragma once

#if defined(DIYBMSMODULEVERSION) && DIYBMSMODULEVERSION < 450
#error Incorrect value for DIYBMSMODULEVERSION should be 450 or higher for tinyAVR 2 series
#endif

//#if !(F_CPU == 4000000)
//#error Processor speed should be 4Mhz
//#endif

#include <Arduino.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

// Used for temperature readings only (13 bit ADC with oversample)
#define MAXIUMUM_ATTINY_ADC_SCALE 8191.0F

/*
This class wraps the hardware pins of DIYBMS away from the core logic/code
if you are porting to another chipset, clone this class and modify it.
*/
class diyBMSHAL
{
public:
  static void ConfigurePorts();

  inline static void ResumePWM() __attribute__((always_inline))
  {
    TCA0.SINGLE.CTRLA |= TCA_SINGLE_ENABLE_bm;
    interrupts();
  }

  inline static void StopTimer1() __attribute__((always_inline))
  {
    TCA0.SINGLE.CTRLA &= ~TCA_SINGLE_ENABLE_bm;
  }

  static void StartTimer1()
  {    
    // Top value...
    TCA0.SINGLE.PER = (F_CPU / 4) / 1000;   
    //5000000/4/1000 = 1250

    // CMP0, Compare Channel 0 interrupt = Match between the counter value and the Compare 0 register
    //TCA0.SINGLE.CMP0=0x1000;
    TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm;
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV4_gc;
    ResumePWM();
  }

  static void PausePWM()
  {
    StopTimer1();
    DumpLoadOff();
  }

  static void SetPrescaler()
  {
    // This isn't needed for tiny2, chip runs at 5Mhz internal clock (to allow down to 1.8V operation)
  }

  static inline void DumpLoadOn() { PORTB.OUTSET = PIN1_bm; }

  static inline void DumpLoadOff() { PORTB.OUTCLR = PIN1_bm; }

  inline static void ReferenceVoltageOn() __attribute__((always_inline))
  {
    // Switch REFERENCE VOLTAGE on
    // Ref voltage ON (PA1)
    PORTA.OUTSET = PIN1_bm;
  }

  inline static void ReferenceVoltageOff() __attribute__((always_inline))
  {
    // Ref voltage (PA1)
    PORTA.OUTCLR = PIN1_bm;
  }

  inline static void TemperatureVoltageOn() __attribute__((always_inline))
  {
    // PB0
    PORTB.OUTSET = PIN0_bm;
  }

  inline static void TemperatureVoltageOff() __attribute__((always_inline))
  {
    // PB0
    PORTB.OUTCLR = PIN0_bm;
  }

  static void FlashNotificationLed(size_t times, uint32_t milliseconds);
  static void PowerOn_Notification_led();

  static inline void NotificationLedOn() __attribute__((always_inline)) { PORTA.OUTSET = PIN6_bm; }

  static inline void NotificationLedOff() __attribute__((always_inline)) { PORTA.OUTCLR = PIN6_bm; }

  static inline void SpareToggle() __attribute__((always_inline)) { PORTA.OUTTGL = PIN2_bm; }

  static inline void SpareOn() __attribute__((always_inline)) { PORTA.OUTSET = PIN2_bm; }

  static inline void SpareOff() __attribute__((always_inline)) { PORTA.OUTCLR = PIN2_bm; }

  static inline void FlushSerial0() __attribute__((always_inline)) { Serial.flush(); }

  static inline void DisableSerial0TX() __attribute__((always_inline))
  {
    // On tiny1624 this saves about 7mA of current
    USART0.CTRLB &= ~(USART_TXEN_bm); /* Transmitter Enable bit mask. */
  }

  static inline void EnableSerial0TX() __attribute__((always_inline))
  {
    // When the transmitter is disabled, it will no longer override the TXD pin, and the pin
    // direction is automatically set as input by hardware, even if it was configured as output by the user
    USART0.CTRLB |= USART_TXEN_bm; /* Transmitter Enable bit mask. */
    PORTB.DIRSET = PIN2_bm;
  }

  // The Start-of-Frame Detection feature enables the USART to wake up from Standby Sleep mode upon data reception.
  static inline void EnableStartFrameDetection() __attribute__((always_inline))
  {
    USART0.CTRLB |= USART_SFDEN_bm;
  }

  static void SetWatchdog8sec()
  {
    // Setup a watchdog timer for 8 seconds
    CCP = 0xD8;
    // 8 seconds
    WDT.CTRLA = WDT_PERIOD_8KCLK_gc;
    wdt_reset();
  }

  static uint16_t BeginADCReading(uint8_t adcmode);

  static void Sleep()
  {
    interrupts();

    // Switch of TX - save power
    diyBMSHAL::DisableSerial0TX();

    // RUNSTBY

    // Standby mode is needed to allow the "USART Start-of-Frame interrupts" to wake CPU

    // Wake up on Serial port RX
    diyBMSHAL::EnableStartFrameDetection();

    set_sleep_mode(SLEEP_MODE_STANDBY);
    sleep_enable();
    sleep_cpu();

    // Snoring can be heard at this point....

    sleep_disable();
  }

  static void SelectCellVoltageChannel()
  {
    // FREERUN / LEFTADJ / SAMPNUM[3:0]
    ADC0.CTRLF = ADC_SAMPNUM_enum::ADC_SAMPNUM_ACC128_gc;
    // PA5 = VREF pin
    ADC0.MUXPOS = ADC_MUXPOS_enum::ADC_MUXPOS_AIN5_gc;
  }

  static inline void SelectInternalTemperatureChannel()
  {
    // PA7
    // FREERUN / LEFTADJ / SAMPNUM[3:0]
    ADC0.CTRLF = ADC_SAMPNUM_enum::ADC_SAMPNUM_ACC2_gc;
    // PA5 = VREF pin
    ADC0.MUXPOS = ADC_MUXPOS_enum::ADC_MUXPOS_AIN7_gc;
  }

  static inline void SelectExternalTemperatureChannel()
  {
    // PA3
    // FREERUN / LEFTADJ / SAMPNUM[3:0]
    ADC0.CTRLF = ADC_SAMPNUM_enum::ADC_SAMPNUM_ACC2_gc;
    // PA5 = VREF pin
    ADC0.MUXPOS = ADC_MUXPOS_enum::ADC_MUXPOS_AIN3_gc;
  }

  static void double_tap_Notification_led();
};

#endif

#endif
