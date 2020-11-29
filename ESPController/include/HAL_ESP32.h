/*
THIS IS THE HARDWARE ABSTRACT LAYER 
FOR DIYBMS ESP32 CONTROLLER PCB - THIS IS THE LARGER
PCB WITH RS485/CANBUS/TFT DISPLAY
*/
#if defined(ESP32)

#include <Arduino.h>
#include <Wire.h>

//#define GREEN_LED 2

//0 is the BOOT button
#define RESET_WIFI_PIN 0
#define PFC_INTERRUPT_PIN 33

#ifndef HAL_ESP32_H_
#define HAL_ESP32_H_


 #define TCA9534APWR_ADDRESS                           0x38
 #define TCA9534APWR_OUTPUT                          0x01
 #define TCA9534APWR_POLARITY_INVERSION              0x02
 #define TCA9534APWR_CONFIGURATION                   0x03

 #define TCA6408_ADDRESS                           0x20
 #define TCA6408_INPUT                           0x00
 #define TCA6408_OUTPUT                          0x01
 #define TCA6408_POLARITY_INVERSION              0x02
 #define TCA6408_CONFIGURATION                   0x03

// Derived classes
class HAL_ESP32
{
public:
    void ConfigureI2C(void (*ExternalInputInterrupt)(void));
    void SetOutputState(uint8_t outputId, RelayState state);
    uint8_t ReadInputRegisters();
    bool OutputsEnabled = false;
    bool InputsEnabled = false;
    void GreenLedOn();
    void GreenLedOff();
    void ConfigurePins();
    

private:
    //Private constructor (static class)

    //Copy of pin state for TCA9534
    uint8_t TCA9534APWR_Value;
    //Copy of pin state for TCA6408
    uint8_t TCA6408_Value;
};

#endif
#endif