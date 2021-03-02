/*
 ____  ____  _  _  ____  __  __  ___    _  _  __
(  _ \(_  _)( \/ )(  _ \(  \/  )/ __)  ( \/ )/. |
 )(_) )_)(_  \  /  ) _ < )    ( \__ \   \  /(_  _)
(____/(____) (__) (____/(_/\/\_)(___/    \/   (_)

  (c) 2017 to 2021 Stuart Pittaway

BOARD TEST CODE FOR USE AFTER BUILDING CONTROLLER PCB


TEST LED

RED/BLUE = FAILED SD CARD DETECTION OR WRITE
RED/PURPLE = TCA9534APWR error
RED/GREEN = TCA6408 Error
RED/WHITE = TX1/RX1 error (make sure loop cable is connected!)

Quickly flashing Green = All good
*/

#if defined(ESP8266)
#error ESP8266 is not supported by this code
#endif

#undef CONFIG_DISABLE_HAL_LOCKS

#include <Arduino.h>

#include "FS.h"
#include "SD.h"
#include <LITTLEFS.h>
#include <WiFi.h>
#include <SPI.h>
#include "driver/gpio.h"
#include "driver/can.h"
#include <XPT2046_Touchscreen.h>
#include "TFT_eSPI.h"
#include "driver/i2c.h"
#include "esp32-hal-i2c.h"

#define TOUCH_IRQ GPIO_NUM_36
#define TOUCH_CHIPSELECT GPIO_NUM_4
#define SDCARD_CHIPSELECT GPIO_NUM_5

#define SERIAL_DATA Serial2
#define SERIAL_DEBUG Serial
#define SERIAL_RS485 Serial1

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
#define TCA6408_ADDRESS 0x20
#define TCA6408_INPUT 0x00
#define TCA6408_OUTPUT 0x01
#define TCA6408_POLARITY_INVERSION 0x02
#define TCA6408_CONFIGURATION 0x03
#define TCA6408_INPUTMASK B00001111

#define VSPI_SCK GPIO_NUM_18

#define RS485_RX GPIO_NUM_21
#define RS485_TX GPIO_NUM_22
#define RS485_ENABLE GPIO_NUM_25

enum RGBLED : uint8_t
{
    OFF = 0,
    Blue = B00000001,
    Red = B00000010,
    Purple = B00000011,
    Green = B00000100,
    Cyan = B00000101,
    Yellow = B00000110,
    White = B00000111
};

SPIClass vspi;
//Copy of pin state for TCA9534
uint8_t TCA9534APWR_Value;
//Copy of pin state for TCA6408
uint8_t TCA6408_Value;

TFT_eSPI tft = TFT_eSPI();
//Don't enable IRQ on touchscreen class (we do that later)
XPT2046_Touchscreen touchscreen(TOUCH_CHIPSELECT, TOUCH_IRQ);

//Prototype functions
void Halt(RGBLED colour);

uint8_t readByte(i2c_port_t i2c_num, uint8_t dev, uint8_t reg)
{
    //We use the native i2c commands for ESP32 as the Arduino library
    //seems to have issues with corrupting i2c data if used from multiple threads

    uint8_t data;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    //Select the correct register on the i2c device
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    // Send repeated start, and read the register
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev << 1) | I2C_MASTER_READ, true);
    //Read single byte and expect NACK in reply
    i2c_master_read_byte(cmd, &data, i2c_ack_type_t::I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    //esp_err_t ret =
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(100)));

    //ESP_LOGD(TAG,"I2C reply %i",ret);

    i2c_cmd_link_delete(cmd);

    return data;
}

esp_err_t writeByte(i2c_port_t i2c_num, uint8_t deviceAddress, uint8_t i2cregister, uint8_t data)
{
    //We use the native i2c commands for ESP32 as the Arduino library
    //seems to have issues with corrupting i2c data if used from multiple threads
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (deviceAddress << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, i2cregister, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);

    esp_err_t ret = ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(100)));
    i2c_cmd_link_delete(cmd);
    return ret;
}

uint8_t ReadTCA6408InputRegisters()
{
    TCA6408_Value = readByte(I2C_NUM_0, TCA6408_ADDRESS, TCA6408_INPUT);
    return TCA6408_Value & TCA6408_INPUTMASK;
}

uint8_t ReadTCA9534InputRegisters()
{
    TCA9534APWR_Value = readByte(i2c_port_t::I2C_NUM_0, TCA9534APWR_ADDRESS, TCA9534APWR_INPUT);
    return TCA9534APWR_Value & TCA9534APWR_INPUTMASK;
}

void ConfigurePins()
{
    //GPIO39 is interrupt pin from TCA6408 (doesnt have pull up/down resistors)
    pinMode(TCA6408_INTERRUPT_PIN, INPUT);

    //GPIO34 is interrupt pin from TCA9534A (doesnt have pull up/down resistors)
    pinMode(TCA9534A_INTERRUPT_PIN, INPUT);

    //BOOT Button on ESP32 module is used for resetting wifi details
    pinMode(GPIO_NUM_0, INPUT_PULLUP);
    //attachInterrupt(GPIO_NUM_0, WiFiPasswordResetInterrupt, CHANGE);

    //For touch screen
    pinMode(TOUCH_IRQ, INPUT_PULLUP);
    //attachInterrupt(TOUCH_IRQ, TFTScreenTouch, FALLING);

    //Configure the CHIP SELECT pins as OUTPUT and set HIGH
    pinMode(TOUCH_CHIPSELECT, OUTPUT);
    digitalWrite(TOUCH_CHIPSELECT, HIGH);
    pinMode(SDCARD_CHIPSELECT, OUTPUT);
    digitalWrite(SDCARD_CHIPSELECT, HIGH);

    pinMode(RS485_ENABLE, OUTPUT);
    //Enable receive
    digitalWrite(RS485_ENABLE, LOW);
}

void ConfigureI2C()
{
    ESP_LOGI(TAG, "Configure I2C");

    //SDA / SCL
    //ESP32 = I2C0-SDA / I2C0-SCL
    //I2C Bus 1: uses GPIO 27 (SDA) and GPIO 26 (SCL);
    //I2C Bus 2: uses GPIO 33 (SDA) and GPIO 32 (SCL);

    // Initialize
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = gpio_num_t::GPIO_NUM_27;
    conf.scl_io_num = gpio_num_t::GPIO_NUM_26;
    //conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
    //conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 400000;
    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);

    //All off
    esp_err_t ret = writeByte(I2C_NUM_0, TCA9534APWR_ADDRESS, TCA9534APWR_OUTPUT, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "TCA9534APWR Error");
        Halt(RGBLED::Purple);
    }

    //0×03 Configuration, P7 as input, others outputs (0=OUTPUT)
    ret = writeByte(I2C_NUM_0, TCA9534APWR_ADDRESS, TCA9534APWR_CONFIGURATION, TCA9534APWR_INPUTMASK);

    //0×02 Polarity Inversion, zero = off
    //writeByte(TCA9534APWR_ADDRESS, TCA9534APWR_POLARITY_INVERSION, 0);
    TCA9534APWR_Value = readByte(I2C_NUM_0, TCA9534APWR_ADDRESS, TCA9534APWR_INPUT);
    //SERIAL_DEBUG.println("Found TCA9534APWR");

    //    attachInterrupt(TCA9534A_INTERRUPT_PIN, TCA9534AInterrupt, FALLING);

    ESP_LOGI(TAG, "Found TCA9534A");

    /*
Now for the TCA6408
*/

    //P0=EXT_IO_A
    //P1=EXT_IO_B
    //P2=EXT_IO_C
    //P3=EXT_IO_D
    //P4=RELAY 1
    //P5=RELAY 2
    //P6=RELAY 3 (SSR)
    //P7=EXT_IO_E

    //Set ports to off before we set configuration
    ret = writeByte(I2C_NUM_0, TCA6408_ADDRESS, TCA6408_OUTPUT, 0);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "TCA6408 Error");
        Halt(RGBLED::Green);
    }

    //Ports A/B/C/D inputs, RELAY1/2/3/4 outputs
    ret = writeByte(I2C_NUM_0, TCA6408_ADDRESS, TCA6408_CONFIGURATION, TCA6408_INPUTMASK);
    //ret =writeByte(i2c_port_t::I2C_NUM_0,TCA6408_ADDRESS, TCA6408_POLARITY_INVERSION, B00000000);
    TCA6408_Value = readByte(I2C_NUM_0, TCA6408_ADDRESS, TCA6408_INPUT);
    //TODO: Validate if there was a read error or not.

    ESP_LOGI(TAG, "Found TCA6408");

    //    attachInterrupt(TCA6408_INTERRUPT_PIN, TCA6408Interrupt, FALLING);
}

void Led(uint8_t bits)
{
    //Clear LED pins
    TCA9534APWR_Value = TCA9534APWR_Value & B11111000;
    //Set on
    TCA9534APWR_Value = TCA9534APWR_Value | (bits & B00000111);
    //esp_err_t ret =
    ESP_ERROR_CHECK_WITHOUT_ABORT(writeByte(I2C_NUM_0, TCA9534APWR_ADDRESS, TCA9534APWR_OUTPUT, TCA9534APWR_Value));
}

//Infinite loop flashing the LED RED/WHITE
void Halt(RGBLED colour)
{
    while (true)
    {
        Led(RGBLED::Red);
        delay(700);
        Led(colour);
        delay(300);
    }
}

void ConfigureVSPI()
{
    vspi.endTransaction();
    vspi.end();
    //VSPI
    //GPIO23 (MOSI), GPIO19(MISO), GPIO18(CLK) and GPIO5 (CS)
    vspi.begin(18, 19, 23, -1);
    //Don't use hardware chip selects on VSPI
    vspi.setHwCs(false);
    vspi.setBitOrder(MSBFIRST);
    vspi.setDataMode(SPI_MODE0);
    //10mhz
    vspi.setFrequency(10000000);
}

void TFTScreenBacklight(bool value)
{
    //Clear LED pins
    TCA9534APWR_Value = TCA9534APWR_Value & B11110111;

    if (value == true)
    {
        //Set on
        TCA9534APWR_Value = TCA9534APWR_Value | B00001000;
    }

    //esp_err_t ret =
    ESP_ERROR_CHECK_WITHOUT_ABORT(writeByte(I2C_NUM_0, TCA9534APWR_ADDRESS, TCA9534APWR_OUTPUT, TCA9534APWR_Value));
    //TODO: Check return value
    //ESP_LOGD(TAG,"TCA9534 reply %i",ret);
}

void init_tft_display()
{
    tft.init();
    tft.initDMA(); // Initialise the DMA engine (tested with STM32F446 and STM32F767)
    tft.getSPIinstance().setHwCs(false);
    tft.setRotation(3);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);

    tft.setCursor(0, 48 + 16, 4);
    tft.println(F("I'm alive!"));
    TFTScreenBacklight(true);
}

void mountSDCard()
{
    /*
SD CARD TEST
*/
    ESP_LOGI(TAG, "Mounting SD card");

    if (SD.begin(SDCARD_CHIPSELECT, vspi))
    {
        uint8_t cardType = SD.cardType();
        if (cardType == CARD_NONE)
        {
            ESP_LOGI(TAG, "No SD card attached");
            Halt(RGBLED::Blue);
        }
        else
        {
            ESP_LOGI(TAG, "SD card available");
        }
    }
    else
    {
        ESP_LOGE(TAG, "Card Mount Failed");
        Halt(RGBLED::Blue);
    }
}
void testSerial()
{
    ESP_LOGI(TAG, "Test serial TX1/RX1");
    for (size_t i = 0; i < 10; i++)
    {
        delay(100);
        //Clear send and reply buffers
        SERIAL_DATA.flush();
        while (SERIAL_DATA.available())
        {
            SERIAL_DATA.read();
        }

        SERIAL_DATA.println("test!");

        Led(RGBLED::Blue);
        String reply = SERIAL_DATA.readString();

        SERIAL_DEBUG.print(i);
        SERIAL_DEBUG.print('=');
        SERIAL_DEBUG.print(reply.c_str());
        if (reply.startsWith("test!") == false)
        {
            Halt(RGBLED::White);
        }
        Led(RGBLED::OFF);
    }
}

void setup()
{
    //We are not testing ESP32, so switch off WIFI + BT
    WiFi.mode(WIFI_OFF);

    btStop();
    esp_log_level_set("*", ESP_LOG_DEBUG); // set all components to DEBUG level

    SERIAL_DEBUG.begin(115200, SERIAL_8N1);
    SERIAL_DEBUG.setDebugOutput(true);

    vspi = SPIClass(VSPI);

    ConfigurePins();
    ConfigureI2C();
    ConfigureVSPI();

    mountSDCard();

    //Create a test file

    File file;
    const char *filename = "/burnin.txt";

    if (SD.exists(filename))
    {
        file = SD.open(filename, FILE_APPEND);
        ESP_LOGD(TAG, "Open %s", filename);
    }
    else
    {
        File file = SD.open(filename, FILE_WRITE);
        if (file)
        {
            ESP_LOGI(TAG, "Created %s", filename);
        }
        else
        {
            Halt(RGBLED::Blue);
        }
    }

    file.println("Mary had a little lamb...");
    file.close();

    ESP_LOGI(TAG, "All good for SD card file %s", filename);
    SD.end();

    //Test SERIAL
    SERIAL_DATA.begin(2400, SERIAL_8N1, 2, 32); // Serial for comms to modules
    SERIAL_DATA.setRxBufferSize(128);
    testSerial();

    init_tft_display();

    touchscreen.begin(vspi);
}

void SetOutputState(uint8_t outputId, bool state)
{
    ESP_LOGD(TAG, "SetOutputState %u=%u", outputId, state);

    //Relays connected to TCA6408A
    //P4 = RELAY1 (outputId=0)
    //P5 = RELAY2 (outputId=1)
    //P6 = RELAY3_SSR (outputId=2)
    //P7 = EXT_IO_E (outputId=3)

    if (outputId <= 3)
    {
        TCA6408_Value = readByte(I2C_NUM_0, TCA6408_ADDRESS, TCA6408_INPUT);
        uint8_t bit = outputId + 4;
        TCA6408_Value = (state == true) ? (TCA6408_Value | (1 << bit)) : (TCA6408_Value & ~(1 << bit));
        esp_err_t ret = writeByte(I2C_NUM_0, TCA6408_ADDRESS, TCA6408_OUTPUT, TCA6408_Value);
        //ESP_LOGD(TAG, "TCA6408 reply %i", ret);

        if (ret != ESP_OK)
        {
            Halt(RGBLED::Green);
        }

        //TODO: Check return value
        TCA6408_Value = readByte(I2C_NUM_0, TCA6408_ADDRESS, TCA6408_INPUT);
    }
}

uint8_t output = 0;
bool state = true;
void loop()
{
    SetOutputState(output, state);
    output++;
    if (output > 3)
    {
        output = 0;
        state = !state;
    }

    if (touchscreen.tirqTouched() && touchscreen.touched())
    {
        TS_Point p = touchscreen.getPoint();

        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.setCursor(0, 100, 2);
        tft.print(millis());
        tft.print("Pressure = ");
        tft.print(p.z);
        tft.print(", x = ");
        tft.print(p.x);
        tft.print(", y = ");
        tft.print(p.y);
    }
    else
    {
        tft.fillRect(0, 100, 320, 16, TFT_WHITE);
    }

    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setCursor(0, 16, 4);
    tft.print(millis());

    Led(RGBLED::Green);
    //Don't do anything
    delay(250);
    Led(RGBLED::OFF);
    //Don't do anything
    delay(250);
}