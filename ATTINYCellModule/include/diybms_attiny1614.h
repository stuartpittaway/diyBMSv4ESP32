#if defined(__AVR_ATtiny1614__)

#ifndef DIYBMS_ATTINY1614_H

#define DIYBMS_ATTINY1614_H

#pragma once


#if defined(DIYBMSMODULEVERSION) && DIYBMSMODULEVERSION < 450
#error Incorrect value for DIYBMSMODULEVERSION should be 450 or higher for tiny1614
#endif


/*
#if !(F_CPU == 2000000)
#error Processor speed should be 2Mhz
#endif

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
      //Switch ON Bit 1 – OCIE1A: Timer/Counter, Output Compare A Match Interrupt Enable
      //TIMSK1 |= _BV(OCIE1A);
      interrupts();
   }

   static void StopTimer1()
   {
      //Switch OFF Bit 1 – OCIE1A: Timer/Counter, Output Compare A Match Interrupt Enable
      //TIMSK1 &= (~_BV(OCIE1A));
   }
   static void StartTimer1()
   {
      //Normal port operation, OC1A/OC1B disconnected
      //Bits 1:0 – WGMn[1:0]: Waveform Generation Mode
      //COM1A1 COM1A0 COM1B1 COM1B0 – – WGM11 WGM10
      //        CCCC--WW
      //TCCR1A = B00000000;

      //CTC (Clear Timer on Compare) = mode 4 = 0100
      //TOP = OCR1A
      //Bits 2:0 – CSn[2:0]: Clock Select = Prescale 64
      //ICNC1 ICES1 – WGM13 WGM12 CS12 CS11 CS10
      //        II-WWCCC
      //TCCR1B = B00001011;

      // Prescaler 8Mhz/64 = 125000 counts per second, call ISR 1000 times per second
      // Prescaler 2Mhz/64 = 31250 counts per second, call ISR 1000 times per second - roughly, as rounding error of 31.25
      //OCR1A = (F_CPU / 64) / 1000;

      ResumePWM();
   }

   static void PausePWM()
   {
      StopTimer1();
      DumpLoadOff();
   }

   static void SetPrescaler()
   {
      //Boot up will be in 1Mhz CKDIV8 mode, swap to /4 to change speed to 2Mhz
      //byte oldSREG = SREG;
      //cli();
      /*atomic code, as its time sensitive*/;
      //CCP – Configuration Change Protection Register
      //CCP = 0xD8;
      //CLKPR – Clock Prescale Register
      //CLKPR = _BV(CLKPS1);
      //SREG = oldSREG;
   }

   static void DumpLoadOn()
   {
      //4.4 board
      //PORTB |= _BV(PORTB2);
   }

   static void DumpLoadOff()
   {
      //4.4 board
      //PORTB &= (~_BV(PORTB2));
   }

   static void ReferenceVoltageOn()
   {
      //When to switch external voltage reference on or off. Connected to Pin 6, PA7
      //PORTA |= _BV(PORTA7);
   }

   static void ReferenceVoltageOff()
   {
      //When to switch external voltage reference on or off. Connected to Pin 6, PA7
      //PORTA &= (~_BV(PORTA7));
   }

   static void NotificationLedOn()
   {
      //PORTA |= _BV(PORTA6);
   }

   static void NotificationLedOff()
   {
      //PORTA &= (~_BV(PORTA6));
   }

   static void FlushSerial0()
   {
      Serial.flush();
      //while (bit_is_set(UCSR0B, UDRIE0) || bit_is_clear(UCSR0A, TXC0))       {      }
   }

   static void DisableSerial0TX()
   {
      //UCSR0B &= ~_BV(TXEN0); //disable transmitter (saves 6mA)
   }

   static void EnableSerial0TX()
   {
      //UCSR0B |= (1 << TXEN0); // enable transmitter
   }

   static void DisableSerial0()
   {
      //Disable serial0
      //UCSR0B &= ~_BV(RXEN0); //disable receiver
      //UCSR0B &= ~_BV(TXEN0); //disable transmitter
   }

   static void EnableSerial0()
   {
      //UCSR0B |= (1 << RXEN0); // enable RX Serial0
      //UCSR0B |= (1 << TXEN0); // enable TX Serial0
   }

   static void DisableSerial1()
   {
      //UCSR1B &= ~_BV(RXEN1); //disable receiver
      //UCSR1B &= ~_BV(TXEN1); //disable transmitter
   }

   static void EnableSerial1()
   {
      //UCSR1B |= (1 << RXEN1); // enable RX Serial1
      //UCSR1B |= (1 << TXEN1); // enable TX Serial1
   }

   static void EnableStartFrameDetection()
   {
      //noInterrupts();

      // Enable Start Frame Detection
      //UCSR0D = (1 << RXSIE0) | (1 << SFDE0);

      //interrupts();
   }

   static void SetWatchdog8sec();

   static uint16_t ReadADC();

   static void BeginADCReading();

   static void Sleep();

   static void SelectCellVoltageChannel()
   {
      //ADMUXA = B00000011;
   }

   static void SelectInternalTemperatureChannel()
   {
      //PA4
      //ADMUXA – ADC Multiplexer Selection Register A
      //ADC4 (single end) MUX[5:0] 00 0100
      //ADMUXA = (0 << MUX5) | (0 << MUX4) | (0 << MUX3) | (1 << MUX2) | (0 << MUX1) | (0 << MUX0);
      //ADMUXA = B00000100;
   }

   static void SelectExternalTemperatureChannel()
   {
      //External sensor
      //ADMUXA – ADC Multiplexer Selection Register A

      //ADMUXA = B00000101;
   }

   static void double_tap_Notification_led();
};

#endif

#endif
