#include <Arduino.h>

void DumpLoadOn() { PORTB.OUTSET = PIN1_bm; }

void DumpLoadOff() { PORTB.OUTCLR = PIN1_bm; }

void ReferenceVoltageOn()
{
  // Switch REFERENCE VOLTAGE and ENABLE pins ON

  // Ref voltage ON (PA1)
  PORTA.OUTSET = PIN1_bm;
  // PB0 (ENABLE)
  PORTB.OUTSET = PIN0_bm;

  delayMicroseconds(50);
}

void ReferenceVoltageOff()
{
  // Ref voltage ON (PA1)
  PORTA.OUTCLR = PIN1_bm;
  // PB0 (ENABLE)
  PORTB.OUTCLR = PIN0_bm;
}

uint16_t ReadADC() { return ADC0.RES; }

uint16_t ReadADC1() { return ADC1.RES; }

void SelectCellVoltageChannel()
{
  // Nothing to do...
}

void SelectInternalTemperatureChannel()
{
  // PA7
  ADC0.MUXPOS = ADC_MUXPOS_enum::ADC_MUXPOS_AIN7_gc;
}

void SelectExternalTemperatureChannel()
{
  // PA3
  ADC0.MUXPOS = ADC_MUXPOS_enum::ADC_MUXPOS_AIN3_gc;
}

void NotificationLedOn() { PORTA.OUTSET = PIN6_bm; }

void NotificationLedOff() { PORTA.OUTCLR = PIN6_bm; }

// static void SpareOn() { PORTA.OUTSET = PIN2_bm; }

// static void SpareOff() { PORTA.OUTCLR = PIN2_bm; }

uint16_t BeginADCReading(uint8_t adcmode)
{
  // sei(); /* Enable Global Interrupts */
  uint16_t value = 0;

  if (adcmode == 0)
  {
    // Cell voltage
    // ADC1
    // Over sample 16 times
    ADC1.CTRLB = ADC_SAMPNUM_enum::ADC_SAMPNUM_ACC16_gc;

    // Trigger interrupt when complete
    // ADC1.INTCTRL = ADC_RESRDY_bm;
    // use external VREF pin and clock freq=500000hz
    // (ADC freq = 4Mhz/8, needs to be 50khz to 1.5Mhz)
    ADC1.CTRLC = ADC_REFSEL_enum::ADC_REFSEL_VREFA_gc |
                 ADC_PRESC_enum::ADC_PRESC_DIV8_gc | ADC_SAMPCAP_bm;

    ADC1.MUXPOS = ADC_MUXPOS_enum::ADC_MUXPOS_AIN0_gc;

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
    // Over sample 16 times
    ADC0.CTRLB = ADC_SAMPNUM_enum::ADC_SAMPNUM_ACC16_gc;

    // Trigger interrupt when complete
    // ADC0.INTCTRL = ADC_RESRDY_bm;
    // use VCC and clock freq=500000hz (4Mhz/8, needs to be 50khz to 1.5Mhz)
    ADC0.CTRLC = ADC_REFSEL_enum::ADC_REFSEL_VDDREF_gc |
                 ADC_PRESC_enum::ADC_PRESC_DIV8_gc | ADC_SAMPCAP_bm;

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

  // set_sleep_mode(SLEEP_MODE_STANDBY); /* Set sleep mode to STANDBY mode */
  // sleep_enable();

  // Disable ADC

  // Let the conversion finish
  //delay(1);
  return value;
}

void ConfigurePorts()
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

  // Set pins to initial state
  DumpLoadOff();
  ReferenceVoltageOff();
  NotificationLedOff();
}

void setup()
{
  ConfigurePorts();

  Serial.begin(115200, SERIAL_8N1);

  Serial.println("\n\n\nI'm alive...");
}

void loop()
{

  //Each ADC step is 1.220mV of the 1.25V reference

  ReferenceVoltageOn();
  Serial.println("Loop");
  Serial.print("Voltage ADC=");
  uint16_t raw_adc_voltage = BeginADCReading(0);
  Serial.print(raw_adc_voltage);
  Serial.print(' ');
  //float voltage = (float)raw_adc_voltage * (float)4.3355;
  //Serial.print(voltage);
  //Serial.println('mV');

  //Do some integer maths, multiply by 2
  raw_adc_voltage=raw_adc_voltage<<1;
  //Fixed point math multiplication
  uint32_t x = ((uint32_t)raw_adc_voltage * 21748UL) / 100000UL;
  
  Serial.print(x);
  Serial.println('mV');

  /*
RAM:   [=         ]   8.1% (used 165 bytes from 2048 bytes)
Flash: [===       ]  25.1% (used 4107 bytes from 16384 bytes)
*/
  ReferenceVoltageOff();

  ReferenceVoltageOn();
  SelectInternalTemperatureChannel();
  Serial.print("Voltage ADC=");
  Serial.println(BeginADCReading(1));
  ReferenceVoltageOff();

  ReferenceVoltageOn();
  SelectExternalTemperatureChannel();
  Serial.print("Temp Ext ADC=");
  Serial.println(BeginADCReading(2));
  ReferenceVoltageOff();

  delay(4000);
}
