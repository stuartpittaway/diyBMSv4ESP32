/*
THIS IS THE HARDWARE ABSTRACT LAYER
FOR DIYBMS ESP32 CONTROLLER PCB - THIS IS THE LARGER
PCB WITH RS485/CANBUS/TFT DISPLAY
*/
#if defined(ESP32)

#include <Arduino.h>
#include "driver/i2c.h"
#include "esp32-hal-i2c.h"
#include <SPI.h>

//#define GREEN_LED 2

//0 is the BOOT button
#define RESET_WIFI_PIN 0
#define PFC_INTERRUPT_PIN 33

#ifndef HAL_ESP32_H_
#define HAL_ESP32_H_

//GPIO34 (input only pin)
#define TCA9534A_INTERRUPT_PIN GPIO_NUM_34
#define TCA9534APWR_ADDRESS 0x38
#define TCA9534APWR_INPUT 0x00
#define TCA9534APWR_OUTPUT 0x01
#define TCA9534APWR_POLARITY_INVERSION 0x02
#define TCA9534APWR_CONFIGURATION 0x03
#define TCA9534APWR_INPUTMASK B10000000

//GPIO39 (input only pin)
#define TCA6408_INTERRUPT_PIN GPIO_NUM_39
//#define TCA6408_ADDRESS 0x20
#define TCA6408_ADDRESS 0x38
#define TCA6408_INPUT 0x00
#define TCA6408_OUTPUT 0x01
#define TCA6408_POLARITY_INVERSION 0x02
#define TCA6408_CONFIGURATION 0x03
#define TCA6408_INPUTMASK B10001111

#define VSPI_SCK GPIO_NUM_18

#define TOUCH_IRQ GPIO_NUM_36
#define TOUCH_CHIPSELECT GPIO_NUM_4
#define SDCARD_CHIPSELECT GPIO_NUM_5

#define RS485_RX GPIO_NUM_21
#define RS485_TX GPIO_NUM_22
#define RS485_ENABLE GPIO_NUM_25

// Derived classes
class HAL_ESP32
{
public:
    SPIClass vspi;
    HAL_ESP32()
    {
        vspi = SPIClass(VSPI);
    }

    void ConfigureI2C(void (*TCA6408Interrupt)(void), void (*TCA9534AInterrupt)(void));
    void SetOutputState(uint8_t outputId, RelayState state);
    uint8_t ReadTCA6408InputRegisters();
    uint8_t ReadTCA9534InputRegisters();
    bool OutputsEnabled = true;
    bool InputsEnabled = false;
    void Led(uint8_t bits);
    void ConfigurePins(void (*WiFiPasswordResetInterrput)(void));
    void TFTScreenBacklight(bool Status);
    void SwapGPIO0ToOutput();
    void ConfigureVSPI()
    {
        vspi.endTransaction();
        vspi.end();
        //VSPI
        //GPIO23 (MOSI), GPIO19(MISO), GPIO18(CLK) and GPIO5 (CS)
        vspi.begin(18,19,23,-1);
        //Don't use hardware chip selects on VSPI
        vspi.setHwCs(false);
        vspi.setBitOrder(MSBFIRST);
        vspi.setDataMode(SPI_MODE0);       
        //10mhz
        vspi.setFrequency(10000000);
    }





private:

    //Copy of pin state for TCA9534
    uint8_t TCA9534APWR_Value;
    //Copy of pin state for TCA6408
    uint8_t TCA6408_Value;

    esp_err_t writeByte(i2c_port_t i2c_num, uint8_t dev, uint8_t reg, uint8_t data);
    uint8_t readByte(i2c_port_t i2c_num, uint8_t dev, uint8_t reg);
};

#endif
#endif
