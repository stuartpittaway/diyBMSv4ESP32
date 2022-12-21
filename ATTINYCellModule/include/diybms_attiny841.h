#if defined(__AVR_ATtiny841__)

#ifndef DIYBMS_ATTINY841_H // include guard
#define DIYBMS_ATTINY841_H

#include <Arduino.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

#define nop __asm__("nop\n\t");

#if defined(DIYBMSMODULEVERSION) && DIYBMSMODULEVERSION > 440
#error Incorrect value for DIYBMSMODULEVERSION
#endif

#if !(F_CPU == 2000000)
#error Processor speed should be 2Mhz
#endif

#if !defined(ATTINY_CORE)
#error Expected ATTINYCORE
#endif

#define MAXIUMUM_ATTINY_ADC_SCALE 1023.0F
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
      // Switch ON Bit 1 – OCIE1A: Timer/Counter, Output Compare A Match Interrupt Enable
      TIMSK1 |= _BV(OCIE1A);
      interrupts();
   }

   static void StopTimer1()
   {
      // Switch OFF Bit 1 – OCIE1A: Timer/Counter, Output Compare A Match Interrupt Enable
      TIMSK1 &= (~_BV(OCIE1A));
   }
   static void StartTimer1()
   {
      // Normal port operation, OC1A/OC1B disconnected
      // Bits 1:0 – WGMn[1:0]: Waveform Generation Mode
      // COM1A1 COM1A0 COM1B1 COM1B0 – – WGM11 WGM10
      //         CCCC--WW
      TCCR1A = B00000000;

      // CTC (Clear Timer on Compare) = mode 4 = 0100
      // TOP = OCR1A
      // Bits 2:0 – CSn[2:0]: Clock Select = Prescale 64
      // ICNC1 ICES1 – WGM13 WGM12 CS12 CS11 CS10
      //         II-WWCCC
      TCCR1B = B00001011;

      // Prescaler 8Mhz/64 = 125000 counts per second, call ISR 1000 times per second
      // Prescaler 2Mhz/64 = 31250 counts per second, call ISR 1000 times per second - roughly, as rounding error of 31.25
      OCR1A = (F_CPU / 64) / 1000;

      ResumePWM();
   }

   static void PausePWM()
   {
      StopTimer1();
      DumpLoadOff();
   }

   static void SetPrescaler()
   {
      // Boot up will be in 1Mhz CKDIV8 mode, swap to /4 to change speed to 2Mhz
      byte oldSREG = SREG;
      cli();
      /*atomic code, as its time sensitive*/;
      // CCP – Configuration Change Protection Register
      CCP = 0xD8;
      // CLKPR – Clock Prescale Register
      CLKPR = _BV(CLKPS1);
      SREG = oldSREG;
   }

   static void DumpLoadOn()
   {
#if defined(DIYBMSMODULEVERSION) && DIYBMSMODULEVERSION < 440
      // Pre 4.4 board
      PORTA |= _BV(PORTA3);
#else
      // 4.4 board
      PORTB |= _BV(PORTB2);
#endif
   }

   static void DumpLoadOff()
   {
#if defined(DIYBMSMODULEVERSION) && DIYBMSMODULEVERSION < 440
      // Pre 4.4 board
      PORTA &= (~_BV(PORTA3));
#else
      // 4.4 board
      PORTB &= (~_BV(PORTB2));
#endif
   }

   inline static void ReferenceVoltageOn() __attribute__((always_inline))
   {
      // When to switch external voltage reference on or off. Connected to Pin 6, PA7
      PORTA |= _BV(PORTA7);
   }

   inline static void ReferenceVoltageOff() __attribute__((always_inline))
   {
      // When to switch external voltage reference on or off. Connected to Pin 6, PA7
      PORTA &= (~_BV(PORTA7));
   }

   //On boards earlier than v4.5, temperature and reference are connected to the same pin (PA7)
   inline static void TemperatureVoltageOn() __attribute__((always_inline))
   {
      ReferenceVoltageOn();
   }

   inline static void TemperatureVoltageOff() __attribute__((always_inline))
   {
      ReferenceVoltageOff();
   }

   static void FlashNotificationLed(size_t times, uint32_t milliseconds);
   static void PowerOn_Notification_led();

   inline static void NotificationLedOn() __attribute__((always_inline))
   {
      PORTA |= _BV(PORTA6);
   }

   inline static void NotificationLedOff() __attribute__((always_inline))
   {
      PORTA &= (~_BV(PORTA6));
   }

#if defined(DIYBMSMODULEVERSION) && DIYBMSMODULEVERSION < 440
   inline static void SparePinOn() __attribute__((always_inline))
   {
      PORTB |= _BV(PORTB1);
   }

   inline static void SparePinOff() __attribute__((always_inline))
   {
      PORTB &= (~_BV(PORTB1));
   }
#endif

#if defined(DIYBMSMODULEVERSION) && DIYBMSMODULEVERSION < 440
   inline static void BlueLedOn() __attribute__((always_inline))
   {
      PORTA |= _BV(PORTA5);
   }

   inline static void BlueLedOff() __attribute__((always_inline))
   {
      PORTA &= (~_BV(PORTA5));
   }
#endif

   inline static void FlushSerial0() __attribute__((always_inline))
   {
      Serial.flush();

      // while (bit_is_set(UCSR0B, UDRIE0) || bit_is_clear(UCSR0A, TXC0))       {      }
   }

   inline static void DisableSerial0TX() __attribute__((always_inline))
   {
      UCSR0B &= ~_BV(TXEN0); // disable transmitter (saves 6mA)
   }

   inline static void EnableSerial0TX() __attribute__((always_inline))
   {
      UCSR0B |= (1 << TXEN0); // enable transmitter
   }

   inline static void EnableSerial0() __attribute__((always_inline))
   {
      UCSR0B |= (1 << RXEN0); // enable RX Serial0
      UCSR0B |= (1 << TXEN0); // enable TX Serial0
   }

   inline static void DisableSerial1() __attribute__((always_inline))
   {
      UCSR1B &= ~_BV(RXEN1); // disable receiver
      UCSR1B &= ~_BV(TXEN1); // disable transmitter
   }

   inline static void EnableSerial1() __attribute__((always_inline))
   {
      UCSR1B |= (1 << RXEN1); // enable RX Serial1
      UCSR1B |= (1 << TXEN1); // enable TX Serial1
   }

   static void EnableStartFrameDetection()
   {
      noInterrupts();

      // Enable Start Frame Detection
      UCSR0D = (1 << RXSIE0) | (1 << SFDE0);

      interrupts();
   }

   static void SetWatchdog8sec();

   static uint16_t ReadADC();

   static uint16_t BeginADCReading(uint8_t mode);

   static void Sleep();

   static void SelectCellVoltageChannel()
   {
#if defined(DIYBMSMODULEVERSION) && DIYBMSMODULEVERSION < 440
      // Pre 4.4 board
      // PB2 = ADC8 PIN 5 ARDUINO PIN 2/A8 = VOLTAGE reading
      // ADMUXA – ADC Multiplexer Selection Register A
      // ADC8 (single end) MUX[5:0] 00 1000
      // ADMUXA = (0 << MUX5) | (0 << MUX4) | (1 << MUX3) | (0 << MUX2) | (0 << MUX1) | (0 << MUX0);
      ADMUXA = B00001000;

#else
      // 4.4 board

      // PA3 = ADC3 PIN 10 = VOLTAGE READING
      // See Page 144 in the datasheet for information
      // ADMUXA = (0 << MUX5) | (0 << MUX4) | (0 << MUX3) | (0 << MUX2) | (1 << MUX1) | (1 << MUX0);
      ADMUXA = B00000011;

#endif
   }

   static void SelectInternalTemperatureChannel()
   {
      // PA4
      // ADMUXA – ADC Multiplexer Selection Register A
      // ADC4 (single end) MUX[5:0] 00 0100
      // ADMUXA = (0 << MUX5) | (0 << MUX4) | (0 << MUX3) | (1 << MUX2) | (0 << MUX1) | (0 << MUX0);
      ADMUXA = B00000100;
   }

   static void SelectExternalTemperatureChannel()
   {
      // External sensor
      // ADMUXA – ADC Multiplexer Selection Register A

#if defined(DIYBMSMODULEVERSION) && DIYBMSMODULEVERSION < 440
      // ADC11 (single end) MUX[5:0] 00 1011
      // ADMUXA = (0 << MUX5) | (0 << MUX4) | (1 << MUX3) | (0 << MUX2) | (1 << MUX1) | (1 << MUX0);
      ADMUXA = B00001011;
#else
      // V4.4 boards ADC5 (single end) MUX[5:0] 00 0101
      // ADMUXA = (0 << MUX5) | (0 << MUX4) | (0 << MUX3) | (1 << MUX2) | (0 << MUX1) | (1 << MUX0);
      ADMUXA = B00000101;
#endif
   }

   static void double_tap_Notification_led();
#if defined(DIYBMSMODULEVERSION) && DIYBMSMODULEVERSION < 440
   static void double_tap_blue_led();
#endif
};

#endif

#endif
