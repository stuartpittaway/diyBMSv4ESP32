/*
THIS IS THE HARDWARE ABSTRACT LAYER
FOR DIYBMS ESP32 CONTROLLER PCB - THIS IS THE LARGER
PCB WITH RS485/CANBUS/TFT DISPLAY
*/

#define CONFIG_DISABLE_HAL_LOCKS 1

#include <Arduino.h>
#include "driver/i2c.h"
#include <driver/twai.h>
#include "esp32-hal-i2c.h"
#include <SPI.h>
#include "SD.h"

#define PFC_INTERRUPT_PIN 33

#ifndef HAL_ESP32_H_
#define HAL_ESP32_H_

#pragma once

// GPIO34 (input only pin)
#define TCA9534A_INTERRUPT_PIN GPIO_NUM_34
#define TCA9534APWR_ADDRESS 0x38
#define TCA9534APWR_INPUT 0x00
#define TCA9534APWR_OUTPUT 0x01
#define TCA9534APWR_POLARITY_INVERSION 0x02
#define TCA9534APWR_CONFIGURATION 0x03
// Two pins are inputs
#define TCA9534APWR_INPUTMASK B10010000

// GPIO39 (input only pin)
#define TCA6408_INTERRUPT_PIN GPIO_NUM_39
#define TCA6408_ADDRESS 0x20
#define TCA6408_INPUT 0x00
#define TCA6408_OUTPUT 0x01
#define TCA6408_POLARITY_INVERSION 0x02
#define TCA6408_CONFIGURATION 0x03
// All pins on TCA6408 are outputs - prevents interrupts triggering randomly due to static/ESD
#define TCA6408_INPUTMASK B00000000

#define TCA6416_INTERRUPT_PIN GPIO_NUM_39
#define TCA6416_ADDRESS 0x21
#define TCA6416_INPUT 0x00
#define TCA6416_OUTPUT 0x02
#define TCA6416_POLARITY_INVERSION 0x04
#define TCA6416_CONFIGURATION 0x06
// byte 1   * byte 2
// P7 to P0 * P17 to P10
// 00000000 * 11010000
//  P17, 16 and 14 are inputs
//  i2c output is written as low byte, high byte
#define TCA6416_INPUTMASK 0xD000

#define VSPI_SCK GPIO_NUM_18

#define TOUCH_IRQ GPIO_NUM_36
#define TOUCH_CHIPSELECT GPIO_NUM_4
#define SDCARD_CHIPSELECT GPIO_NUM_5

#define INA229_CHIPSELECT GPIO_NUM_33
#define INA229_INTERRUPT_PIN GPIO_NUM_35

#define RS485_RX GPIO_NUM_21
#define RS485_TX GPIO_NUM_22
#define RS485_ENABLE GPIO_NUM_25

struct TouchScreenValues
{
    bool touched;
    uint8_t pressure;
    uint16_t X;
    uint16_t Y;
};

// extern SPIClass vspi;

// Derived classes
class HAL_ESP32
{
public:
    bool TCA6416_Fitted;

    HAL_ESP32()
    {
        TCA6416_Fitted = false;
        xVSPIMutex = xSemaphoreCreateMutex();
        xDisplayMutex = xSemaphoreCreateMutex();
        xi2cMutex = xSemaphoreCreateMutex();
        RS485Mutex = xSemaphoreCreateMutex();
    }

    void ConfigureI2C(void (*TCA6408Interrupt)(void), void (*TCA9534AInterrupt)(void), void (*TCA6416Interrupt)(void));
    void SetOutputState(uint8_t outputId, RelayState state);
    uint8_t ReadTCA6408InputRegisters();
    uint8_t ReadTCA9534InputRegisters();
    uint16_t ReadTCA6416InputRegisters();

    // Returns pointer to VSPI object class
    SPIClass *VSPI_Ptr();

    void Led(uint8_t bits);
    void ConfigurePins();
    void TFTScreenBacklight(bool Status);

    void CANBUSEnable(bool value);
    void ConfigureCAN();

    bool IsVSPIMutexAvailable();
    bool GetVSPIMutex();
    bool ReleaseVSPIMutex();

    bool GetDisplayMutex();
    bool ReleaseDisplayMutex();

    bool Geti2cMutex();
    bool Releasei2cMutex();

    bool GetRS485Mutex();
    bool ReleaseRS485Mutex();

    // Infinite loop flashing the LED RED/WHITE
    void Halt(RGBLED colour);

    uint8_t LastTCA6408Value();
    uint8_t LastTCA9534APWRValue();
    bool MountSDCard();
    void UnmountSDCard();
    TouchScreenValues TouchScreenUpdate();
    bool IsScreenAttached();
    void ConfigureVSPI();

private:
    SemaphoreHandle_t xVSPIMutex = NULL;
    SemaphoreHandle_t xDisplayMutex = NULL;
    SemaphoreHandle_t xi2cMutex = NULL;
    SemaphoreHandle_t RS485Mutex = NULL;

    // Input pin state for TCA9534
    uint8_t TCA9534APWR_Input;
    // Input pin state for TCA6408
    uint8_t TCA6408_Input;
    // Input pin state for TCA6416
    uint16_t TCA6416_Input;

    // Output pin state for TCA9534
    uint8_t TCA9534APWR_Output_Pins;
    // Output pin state for TCA6408
    uint8_t TCA6408_Output_Pins;
    // Output pin state for TCA6416
    uint16_t TCA6416_Output_Pins;

    esp_err_t writeByte(i2c_port_t i2c_num, uint8_t dev, uint8_t reg, uint8_t data);
    uint8_t readByte(i2c_port_t i2c_num, uint8_t dev, uint8_t reg);

    esp_err_t write16bitWord(i2c_port_t i2c_num, uint8_t deviceAddress, uint8_t i2cregister, uint16_t data);
    uint16_t read16bitWord(i2c_port_t i2c_num, uint8_t dev, uint8_t reg);
    esp_err_t Testi2cAddress(i2c_port_t port, uint8_t address);

    void WriteTCA6408OutputState();
    void WriteTCA9534APWROutputState();
    void WriteTCA6416OutputState();
};

#endif
