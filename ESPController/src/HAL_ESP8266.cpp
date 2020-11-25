#if defined(ESP8266)

#include "defines.h"
#include "HAL_ESP8266.h"

#ifndef _PCF8574_ESP_H
#include <pcf8574_esp.h>

//PCF8574P has an i2c address of 0x38 instead of the normal 0x20

//IF YOUR CONTROLLER DOESN'T RECOGNISE THE PCF CHIP, CHANGE 0x38 to 0x20 BELOW...

PCF857x pcf8574(0x38, &Wire);
#endif

uint8_t HAL_ESP8266::ReadInputRegisters()
{
    return InputsEnabled ? pcf8574.read8() : 0;
}

void HAL_ESP8266::SetOutputState(uint8_t outputId, RelayState state)
{
    // NOTE: When you write LOW to a pin on a PCF8574 it becomes an OUTPUT.
    // It wouldn't generate an interrupt if you were to connect a button to it that pulls it HIGH when you press the button.
    // Any pin you wish to use as input must be written HIGH and be pulled LOW to generate an interrupt.

    //If PCF is enabled change the pin otherwise silently ignore request
    if (OutputsEnabled)
    {
        //LOW is ON
        pcf8574.write(outputId, (state == RelayState::RELAY_ON) ? LOW : HIGH);
    }
}

void HAL_ESP8266::ConfigureI2C(void (*ExternalInputInterrupt)(void))
{
    //SDA / SCL
    //I'm sure this should be 4,5 !
    Wire.begin(5, 4);

    Wire.setClock(100000L);

    //D5 is interrupt pin from PCF8574
    pinMode(PFC_INTERRUPT_PIN, INPUT_PULLUP);

    //Make PINs 4-7 INPUTs - the interrupt fires when triggered
    pcf8574.begin();

    //We test to see if the i2c expander is actually fitted
    pcf8574.read8();

    if (pcf8574.lastError() == 0)
    {
        SERIAL_DEBUG.println(F("Found PCF8574"));
        pcf8574.write(4, HIGH);
        pcf8574.write(5, HIGH);
        pcf8574.write(6, HIGH);
        pcf8574.write(7, HIGH);

        InputsEnabled = true;
        OutputsEnabled = true;
    }
    else
    {
        //Not fitted
        SERIAL_DEBUG.println(F("PCF8574 not fitted"));
        InputsEnabled = false;
        OutputsEnabled = false;
    }

    //internal pullup-resistor on the interrupt line via ESP8266
    pcf8574.resetInterruptPin();

    //TODO: Fix this for ESP32 different PIN
    attachInterrupt(digitalPinToInterrupt(PFC_INTERRUPT_PIN), ExternalInputInterrupt, FALLING);
}

#endif