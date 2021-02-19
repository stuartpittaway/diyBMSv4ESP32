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
  // pin out
  // https://github.com/SpenceKonde/megaTinyCore/blob/master/megaavr/extras/ATtiny_x14.md

  // Datasheet
  // http://ww1.microchip.com/downloads/en/DeviceDoc/ATtiny1614-16-17-DataSheet-DS40002204A.pdf

  // AVR PIN / ARDUINO PIN MAPPING
  // PB0 /7= ENABLE
  // PB1 /6= DUMP LOAD ENABLE
  // PB2 /5= TXD
  // PB3 /4= RXD
  // PA0 /11= RESET
  // PA1 /8= REF_ENABLE
  // PA2 /9= NOT CONNECTED
  // PA3 /10= EXTERNAL TEMP SENSOR (ADC) (ADC0=AIN3)
  // PA4 /0= VOLTAGE INPUT (ADC) (ADC0=AIN4, ADC1=AIN0)
  // PA5 /1= VREFERENCE (ADC) (ADC0=AIN5, ADC1=AIN1)
  // PA6 /2= NOTIFICATION LED
  // PA7 /3= INTERNAL TEMP SENSOR (ADC)(ADC0=AIN7, ADC1=AIN3)

  // PA3 and PA7 use the VCC voltage as a reference - not the volt reference!!

  // For lowest power consumption, disable the digital input buffer of unused
  // pins and pins that are used as analog inputs or outputs.

  // Set Port A digital outputs

  // PA1 /8= REF_ENABLE
  // PA6 /2= NOTIFICATION LED

  PORTA.DIRSET = PIN1_bm | PIN6_bm;

  // Set Port B digital outputs
  // PB0 /7= ENABLE
  // PB1 /6= DUMP LOAD ENABLE
  // PB2 /TX as output
  PORTB.DIRSET = PIN0_bm | PIN1_bm | PIN2_bm;
  // Set RX as input
  PORTB.DIRCLR = PIN3_bm;

  // Set Port A analogue inputs
  // PA2 /9= NOT CONNECTED
  // PA3 /10= EXTERNAL TEMP SENSOR (ADC)
  // PA4 /0= VOLTAGE INPUT (ADC)
  // PA7 /3= INTERNAL TEMP SENSOR (ADC)
  PORTA.DIRCLR = PIN2_bm | PIN3_bm | PIN4_bm | PIN7_bm;

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

  // REFSEL must be above the voltage you are using as a REFERENCE
  /*
  When using the external reference voltage VREFA, configure ADCnREFSEL[0:2] in
  the corresponding VREF.CTRLn register to the value that is closest, but above
  the applied reference voltage. For external references higher than 4.3V, use
  ADCnREFSEL[0:2] = 0x3.
*/
  VREF.CTRLA =
      VREF_ADC0REFSEL_4V34_gc; /* Set the ADC0 Vref to 4.3V (using VCC)*/
  VREF.CTRLC =
      VREF_ADC1REFSEL_1V5_gc; /* Set the ADC1 Vref to 1.5V (using 1.25V ext
                                 ref)*/
  VREF.CTRLB = 0;             // Switch off "force" internal references

  // Over sample 16 times
  ADC1.CTRLB = ADC_SAMPNUM_enum::ADC_SAMPNUM_ACC16_gc;

  // Trigger interrupt when complete
  // ADC1.INTCTRL = ADC_RESRDY_bm;
  // use external VREF pin and clock freq=500000hz
  // (ADC freq = 4Mhz/8, needs to be 50khz to 1.5Mhz)
  ADC1.CTRLC = ADC_REFSEL_enum::ADC_REFSEL_VREFA_gc |
               ADC_PRESC_enum::ADC_PRESC_DIV8_gc | ADC_SAMPCAP_bm;

  ADC1.MUXPOS = ADC_MUXPOS_enum::ADC_MUXPOS_AIN0_gc;

  // Over sample 16 times
  ADC0.CTRLB = ADC_SAMPNUM_enum::ADC_SAMPNUM_ACC16_gc;

  // Trigger interrupt when complete
  // ADC0.INTCTRL = ADC_RESRDY_bm;
  // use VCC and clock freq=500000hz (4Mhz/8, needs to be 50khz to 1.5Mhz)
  ADC0.CTRLC = ADC_REFSEL_enum::ADC_REFSEL_VDDREF_gc |
               ADC_PRESC_enum::ADC_PRESC_DIV8_gc | ADC_SAMPCAP_bm;

  // Set pins to initial state
  DumpLoadOff();
  ReferenceVoltageOff();
  NotificationLedOff();
}

void diyBMSHAL::SetWatchdog8sec()
{
  // Setup a watchdog timer for 8 seconds
  // MCUSR = 0;
  // Enable watchdog (to reset)
  // WDTCSR |= bit(WDE);

  // CCP = 0xD8;
  // WDTCSR – Watchdog Timer Control and Status Register
  // We INTERRUPT the chip after 8 seconds of sleeping (not reboot!)
  // WDE: Watchdog Enable
  // Bits 5, 2:0 – WDP[3:0]: Watchdog Timer Prescaler 3 - 0
  // WDTCSR = bit(WDIE) | bit(WDP3) | bit(WDP0);
  //| bit(WDE)

  wdt_reset();
}

uint16_t diyBMSHAL::BeginADCReading(uint8_t mode)
{
  // sei(); /* Enable Global Interrupts */
  uint16_t value = 0;

  if (mode == 0)
  {
    // Cell voltage
    // ADC1

    // Enable ADC
    ADC1.CTRLA = ADC_RESSEL_enum::ADC_RESSEL_10BIT_gc | ADC_ENABLE_bm;
    // | ADC_RUNSTBY_bm;

    // Start the conversion (interrupt is then triggered)
    ADC1.COMMAND = ADC_STCONV_bm;

    // Wait for completion
    while ((ADC1.INTFLAGS & ADC_RESRDY_bm) == false)
    {
    }

    //8 times over sample
    value = ADC1.RES >> 4;
    ADC1.CTRLA &= ~ADC_ENABLE_bm;
  }
  else
  {
    // Temperature
    // Enable ADC
    ADC0.CTRLA = ADC_RESSEL_enum::ADC_RESSEL_10BIT_gc | ADC_ENABLE_bm;
    // | ADC_RUNSTBY_bm;

    // Start the conversion (interrupt is then triggered)
    ADC0.COMMAND = ADC_STCONV_bm;

    // Wait for completion
    while ((ADC0.INTFLAGS & ADC_RESRDY_bm) == false)
    {
    }
    //8 times over sample
    value = ADC0.RES >> 4;
    ADC0.CTRLA &= ~ADC_ENABLE_bm;
  }

  return value;
}


#endif