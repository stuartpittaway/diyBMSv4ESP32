/*
THIS IS THE HARDWARE ABSTRACT LAYER 
FOR DIYBMS ESP8265 CONTROLLER PCB - THIS IS THE SMALLER
PCB WITH INTERFACE FOR EXTERNAL RELAY BOARD USING PCF8574
I2C I/O EXPANDER
*/
#if defined(ESP8266)

#include <Arduino.h>
#include <Wire.h>

#ifndef HAL_ESP8266_H_
#define HAL_ESP8266_H_

// Derived classes
class HAL_ESP8266
{
public:
    void ConfigureI2C(void (*ExternalInputInterrupt)(void));
    void SetOutputState(uint8_t outputId, RelayState state);
    uint8_t ReadInputRegisters();
    bool OutputsEnabled = false;
    bool InputsEnabled = false;

private:
    //Private constructor (static class)
};

#endif

#endif