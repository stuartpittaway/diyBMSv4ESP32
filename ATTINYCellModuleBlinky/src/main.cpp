//USING CLOCKWISE PIN MAPPINGS
#define dump_enable 3
#define green_led 6
#define blue_led 5
#define enable_2048 7

#include <Arduino.h>

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
}

void loop()
{
    PORTA |= _BV(PORTA6);
    delay(500);
    PORTA |= _BV(PORTA5);
    delay(500);
    PORTA &= (~_BV(PORTA6));
    delay(500);
    PORTA &= (~_BV(PORTA5));
    delay(500);
}
