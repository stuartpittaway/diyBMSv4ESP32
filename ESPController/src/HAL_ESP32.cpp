#if defined(ESP32)
#include "defines.h"
#include "HAL_ESP32.h"

uint8_t HAL_ESP32::ReadInputRegisters()
{
    return 0;
}

void HAL_ESP32::SetOutputState(uint8_t outputId, RelayState state)
{
    // DO NOTE: When you write LOW to a pin on a PCF8574 it becomes an OUTPUT.
    // It wouldn't generate an interrupt if you were to connect a button to it that pulls it HIGH when you press the button.
    // Any pin you wish to use as input must be written HIGH and be pulled LOW to generate an interrupt.

    //If PCF is enabled change the pin otherwise silently ignore request
    if (OutputsEnabled)
    {
        //LOW is ON
        //pcf8574.write(outputId, (state == RelayState::RELAY_ON) ? LOW : HIGH);
    }
}

void HAL_ESP32::ConfigureI2C(void (*ExternalInputInterrupt)(void))
{
    //SDA / SCL
    //ESP32 = I2C0-SDA / I2C0-SCL
    //I2C Bus 1: uses GPIO 27 (SDA) and GPIO 26 (SCL);
    //I2C Bus 2: uses GPIO 33 (SDA) and GPIO 32 (SCL);
    Wire.begin(27, 26);

    Wire.setClock(100000L);
}

#endif