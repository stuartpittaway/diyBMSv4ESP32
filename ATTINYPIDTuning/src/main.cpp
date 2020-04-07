#include <Arduino.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <FastPID.h>

//Kp: Determines how aggressively the PID reacts to the current amount of error (Proportional)
//Ki: Determines how aggressively the PID reacts to error over time (Integral)
//Kd: Determines how aggressively the PID reacts to the change in error (Derivative)

int16_t setPoint = 70;

/*
The parameter P domain is [0.00390625 to 255] inclusive.
The parameter I domain is P / Hz
The parameter D domain is P * Hz
*/
//6Hz rate - number of times we call this code in Loop
//Kp, Ki, Kd, Hz, output_bits, output_signed);
//Settings for V4.00 boards with 2R2 resistors = (4.0, 0.5, 0.2, 6, 8, false);
FastPID myPID(4.0, 0.5, 0.2, 6, 8, false);


int16_t ThermistorToCelcius(uint16_t BCOEFFICIENT, uint16_t RawADC)
{

//We can calculate the  Steinhart-Hart Thermistor Equation based on the B Coefficient of the thermistor
// at 25 degrees C rating
#define NOMINAL_TEMPERATURE 25

  //If we get zero its likely the ADC is connected to ground
  if (RawADC > 0)
  {
    //47000 = resistor value
    //https://arduinodiy.wordpress.com/2015/11/10/measuring-temperature-with-ntc-the-steinhart-hart-formula/
    //float Resistance = 47000.0 * (1023.0F/(float)RawADC - 1.0);
    //float steinhart;
    //steinhart = Resistance / 47000.0; // (R/Ro)

    float steinhart = (1023.0F / (float)RawADC - 1.0);

    steinhart = log(steinhart);                        // ln(R/Ro)
    steinhart /= BCOEFFICIENT;                         // 1/B * ln(R/Ro)
    steinhart += 1.0 / (NOMINAL_TEMPERATURE + 273.15); // + (1/To)
    steinhart = 1.0 / steinhart;                       // Invert
    steinhart -= 273.15;                               // convert to oC

    return (int16_t)steinhart;
  }

  return (int16_t)-999;
}

void SetTimer2Value(uint16_t value)
{
  OCR2B = value;
}

void StopTimer2()
{
  TOCPMCOE = 0;
  TCCR2B = 0;
  OCR2B = 0;
  //TIMSK2 = 0;
}

void StartTimer2()
{
  //Dump resistor is on PA3 which maps to TOCC2
  //Before this is called, the DDR register has already been set

  //Enable OC2B for TOCC2
  TOCPMSA0 = (1 << TOCC2S1);

  // Timer/Counter Output Compare Pin Mux Channel Output Enable
  TOCPMCOE = (1 << TOCC2OE);

  // Fast PWM, mode 14, non inverting, presc 1:8
  //COM2b1= Clear OCnA/OCnB on Compare Match (Set output to low level)
  TCCR2A = (1 << COM2B1) | 1 << WGM21;

  //Clock div 64 prescaler
  TCCR2B = 1 << CS21 | 1 << CS20 | 1 << WGM23 | 1 << WGM22;

  //Maximum of 10000 and low of zero
  ICR2 = 10000 - 1;

  //OFF
  SetTimer2Value(0);
}

void ReferenceVoltageOn()
{
  //When to switch external voltage reference on or off. Connected to Pin 6, PA7
  PORTA |= _BV(PORTA7);
}

void GreenLedOn()
{
  PORTA |= _BV(PORTA6);
}

void GreenLedOff()
{
  PORTA &= (~_BV(PORTA6));
}

void DumpLoadOn() {
  PORTA |= _BV(PORTA3);
}

void DumpLoadOff() {
  PORTA &= (~_BV(PORTA3));
}



void SelectInternalTemperatureChannel()
{
  //PA4
  //ADMUXA – ADC Multiplexer Selection Register A
  //ADC4 (single end) MUX[5:0] 00 0100
  ADMUXA = (0 << MUX5) | (0 << MUX4) | (0 << MUX3) | (1 << MUX2) | (0 << MUX1) | (0 << MUX0);
}

void BeginADCReading()
{

  //BlueLedOn();

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
  delay(2);

  noInterrupts();
  set_sleep_mode(SLEEP_MODE_ADC); // sleep during ADC sample
  sleep_enable();

  // start the conversion
  ADCSRA |= _BV(ADSC) | _BV(ADIE);
  interrupts();
  sleep_cpu();
  sleep_disable();

  // awake again, reading should be done, better make sure maybe the timer interrupt fired
  while (bit_is_set(ADCSRA, ADSC))
  {
  }

  //adc_disable
  ADCSRA &= (~(1 << ADEN));
}

uint16_t ReadADC()
{
  // must read ADCL first
  uint8_t low = ADCL;
  return (ADCH << 8) | low;
}

uint16_t RawADC;

ISR(ADC_vect)
{
  // when ADC completed, take an interrupt and process result
  RawADC = ReadADC();
}

void setup()
{
  //PUEA – Port A Pull-Up Enable Control Register (All disabled)
  PUEA = 0;
  //PUEB – Port B Pull-Up Enable Control Register (All disabled)
  PUEB = 0;

  //DDRA – Port A Data Direction Register
  //When DDAn is set, the pin PAn is configured as an output. When DDAn is cleared, the pin is configured as an input
  DDRA |= _BV(DDA3) | _BV(DDA6) | _BV(DDA7) | _BV(DDA5);

  //DDRB – Port B Data Direction Register
  //Spare pin is output
  DDRB |= _BV(DDB1);

  //Set the extra high sink capability of pin PA7 is enabled.
  PHDE |= _BV(PHDEA1);

  ReferenceVoltageOn();

  Serial.begin(9600, SERIAL_8N1);

  //The TIMER2 can vary between 0 and 100
  myPID.setOutputRange(0, 100);
}

void loop()
{
  // put your main code here, to run repeatedly:
  GreenLedOn();

  SelectInternalTemperatureChannel();
  BeginADCReading();

  int16_t temp = ThermistorToCelcius(3955, RawADC);

  uint16_t PWMValue;
  if (temp > setPoint - 10)
  {
    DumpLoadOff();
    StartTimer2();
    //Set point 50, scale PWM output from 0-10000
    PWMValue = myPID.step(setPoint, temp);
    SetTimer2Value(PWMValue*100);
  }
  else
  {
    //Go full power
    StopTimer2();
    //myPID.clear();
    PWMValue = 0;
    DumpLoadOn();
  }

  Serial.print(temp);
  Serial.print('\t');
  Serial.print(setPoint);
  Serial.print('\t');
  Serial.print(PWMValue);
  Serial.println("");

  GreenLedOff();
  //We call the PID 6 times a second
  delay(160);
}