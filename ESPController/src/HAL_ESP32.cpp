
#include "defines.h"
#include "HAL_ESP32.h"

uint8_t HAL_ESP32::readByte(i2c_port_t i2c_num, uint8_t dev, uint8_t reg)
{
    //We use the native i2c commands for ESP32 as the Arduino library
    //seems to have issues with corrupting i2c data if used from multiple threads
    if (Geti2cMutex())
    {

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

        Releasei2cMutex();
        return data;
    }
    else
    {
        return 0;
    }
}

//i2c: Writes a single byte to a slave devices register
esp_err_t HAL_ESP32::writeByte(i2c_port_t i2c_num, uint8_t deviceAddress, uint8_t i2cregister, uint8_t data)
{
    if (Geti2cMutex())
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

        Releasei2cMutex();
        return ret;
    }
    else
    {
        return ESP_ERR_INVALID_STATE;
    }
}

uint8_t HAL_ESP32::ReadTCA6408InputRegisters()
{
    TCA6408_Value = readByte(I2C_NUM_0, TCA6408_ADDRESS, TCA6408_INPUT);
    return TCA6408_Value & TCA6408_INPUTMASK;
}

uint8_t HAL_ESP32::ReadTCA9534InputRegisters()
{
    TCA9534APWR_Value = readByte(i2c_port_t::I2C_NUM_0, TCA9534APWR_ADDRESS, TCA9534APWR_INPUT);
    return TCA9534APWR_Value & TCA9534APWR_INPUTMASK;
}

void HAL_ESP32::SetOutputState(uint8_t outputId, RelayState state)
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
        TCA6408_Value = (state == RelayState::RELAY_ON) ? (TCA6408_Value | (1 << bit)) : (TCA6408_Value & ~(1 << bit));
        esp_err_t ret = writeByte(I2C_NUM_0, TCA6408_ADDRESS, TCA6408_OUTPUT, TCA6408_Value);
        ESP_LOGD(TAG, "TCA6408 reply %i", ret);
        //TODO: Check return value
        TCA6408_Value = readByte(I2C_NUM_0, TCA6408_ADDRESS, TCA6408_INPUT);
    }
}

void HAL_ESP32::Led(uint8_t bits)
{
    //Clear LED pins
    TCA9534APWR_Value = TCA9534APWR_Value & B11111000;
    //Set on
    TCA9534APWR_Value = TCA9534APWR_Value | (bits & B00000111);
    //esp_err_t ret =
    ESP_ERROR_CHECK_WITHOUT_ABORT(writeByte(I2C_NUM_0, TCA9534APWR_ADDRESS, TCA9534APWR_OUTPUT, TCA9534APWR_Value));

    //ESP_LOGD(TAG,"TCA9534 LED reply %i",ret);
    //TODO: Check return value
}

// Control Silent mode control input on TJA1051T/3
// True = enable CANBUS
void HAL_ESP32::CANBUSEnable(bool value)
{
    //Pin P5
    //Low = Normal mode
    //High = Silent
    TCA9534APWR_Value = TCA9534APWR_Value & B11011111;

    if (value == false)
    {
        //Set on
        TCA9534APWR_Value = TCA9534APWR_Value | B00100000;
    }

    ESP_ERROR_CHECK_WITHOUT_ABORT(writeByte(I2C_NUM_0, TCA9534APWR_ADDRESS, TCA9534APWR_OUTPUT, TCA9534APWR_Value));
}

// Control TFT backlight LED
void HAL_ESP32::TFTScreenBacklight(bool value)
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

void HAL_ESP32::ConfigurePins(void (*WiFiPasswordResetInterrupt)(void))
{
    //GPIO39 is interrupt pin from TCA6408 (doesnt have pull up/down resistors)
    pinMode(TCA6408_INTERRUPT_PIN, INPUT);

    //GPIO34 is interrupt pin from TCA9534A (doesnt have pull up/down resistors)
    pinMode(TCA9534A_INTERRUPT_PIN, INPUT);

    //BOOT Button on ESP32 module is used for resetting wifi details
    pinMode(GPIO_NUM_0, INPUT_PULLUP);
    attachInterrupt(GPIO_NUM_0, WiFiPasswordResetInterrupt, CHANGE);

    //For touch screen
    //GPIO_NUM_36 no internal PULLUP
    pinMode(TOUCH_IRQ, INPUT);
    //attachInterrupt(GPIO_NUM_36, TFTScreenTouch, FALLING);

    //Configure the CHIP SELECT pins as OUTPUT and set HIGH
    pinMode(TOUCH_CHIPSELECT, OUTPUT);
    digitalWrite(TOUCH_CHIPSELECT, HIGH);
    pinMode(SDCARD_CHIPSELECT, OUTPUT);
    digitalWrite(SDCARD_CHIPSELECT, HIGH);

    pinMode(RS485_ENABLE, OUTPUT);
    //Enable receive
    digitalWrite(RS485_ENABLE, LOW);
}

void HAL_ESP32::SwapGPIO0ToOutput()
{
    //BOOT Button on ESP32 module is used for resetting wifi details
    detachInterrupt(GPIO_NUM_0);
    pinMode(GPIO_NUM_0, OUTPUT);
    digitalWrite(GPIO_NUM_0, HIGH);
}

void HAL_ESP32::ConfigureI2C(void (*TCA6408Interrupt)(void), void (*TCA9534AInterrupt)(void))
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

    // https://datasheet.lcsc.com/szlcsc/1809041633_Texas-Instruments-TCA9534APWR_C206010.pdf
    // TCA9534APWR Remote 8-Bit I2C and Low-Power I/O Expander With Interrupt Output and Configuration Registers
    // https://lcsc.com/product-detail/Interface-I-O-Expanders_Texas-Instruments-TCA9534APWR_C206010.html
    // A0/A1/A2 are LOW, so i2c address is 0x38

    //PINS
    //P0= BLUE
    //P1= RED
    //P2= GREEN
    //P3= DISPLAY BACKLIGHT LED
    //P4= SPARE on J13
    //P5= Canbus RS
    //P6= SPARE on J13
    //P7= ESTOP (pull to ground to trigger)
    //INTERRUPT PIN = ESP32 IO34

    //BIT  76543210
    //PORT 76543210
    //MASK=10000000

    //All off
    esp_err_t ret = writeByte(I2C_NUM_0, TCA9534APWR_ADDRESS, TCA9534APWR_OUTPUT, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "TCA9534APWR Error");
        Halt(RGBLED::Purple);
    }

    //0×03 Configuration, P7 (estop) and P4 (remote touch) as input, others outputs (0=OUTPUT)
    ret = writeByte(I2C_NUM_0, TCA9534APWR_ADDRESS, TCA9534APWR_CONFIGURATION, TCA9534APWR_INPUTMASK);

    //0×02 Polarity Inversion, zero = off
    //writeByte(TCA9534APWR_ADDRESS, TCA9534APWR_POLARITY_INVERSION, 0);
    TCA9534APWR_Value = readByte(I2C_NUM_0, TCA9534APWR_ADDRESS, TCA9534APWR_INPUT);
    //SERIAL_DEBUG.println("Found TCA9534APWR");

    attachInterrupt(TCA9534A_INTERRUPT_PIN, TCA9534AInterrupt, FALLING);

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

    //Ports A/B/C/D/E inputs, RELAY1/2/3 outputs
    ret = writeByte(I2C_NUM_0, TCA6408_ADDRESS, TCA6408_CONFIGURATION, TCA6408_INPUTMASK);
    //ret =writeByte(i2c_port_t::I2C_NUM_0,TCA6408_ADDRESS, TCA6408_POLARITY_INVERSION, B00000000);
    TCA6408_Value = readByte(I2C_NUM_0, TCA6408_ADDRESS, TCA6408_INPUT);
    //TODO: Validate if there was a read error or not.

    ESP_LOGI(TAG, "Found TCA6408");

    attachInterrupt(TCA6408_INTERRUPT_PIN, TCA6408Interrupt, FALLING);
}
