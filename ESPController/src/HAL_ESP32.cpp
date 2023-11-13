#define USE_ESP_IDF_LOG 1
static constexpr const char *const TAG = "diybms-hal";

#define CONFIG_DISABLE_HAL_LOCKS 1

#include <esp_ipc.h>
#include "defines.h"
#include "HAL_ESP32.h"

// Move outside class so Arduino Framework 2.0.4+ work as expected
SPIClass vspi(VSPI);

SPIClass *HAL_ESP32::VSPI_Ptr() { return &vspi; }

bool HAL_ESP32::MountSDCard()
{
    bool result = false;
    if (GetVSPIMutex())
    {
        // Initialize SD card at 16Mhz SPI
        if (SD.begin(SDCARD_CHIPSELECT, vspi, 16000000U))
        {
            uint8_t cardType = SD.cardType();
            if (cardType == CARD_NONE)
            {
                ESP_LOGW(TAG, "No SD card attached");
            }
            else
            {
                ESP_LOGI(TAG, "SD card mounted, type %i", (int)cardType);
                result = true;
            }
        }
        else
        {
            ESP_LOGE(TAG, "Card mount failed");
        }
        ReleaseVSPIMutex();
    }
    return result;
}

void HAL_ESP32::UnmountSDCard()
{
    ESP_LOGI(TAG, "Unmounting SD card");
    if (GetVSPIMutex())
    {
        SD.end();
        ReleaseVSPIMutex();
    }
}

uint8_t HAL_ESP32::readByte(i2c_port_t i2c_num, uint8_t dev, uint8_t reg)
{
    // We use the native i2c commands for ESP32 as the Arduino library
    // seems to have issues with corrupting i2c data if used from multiple threads
    if (Geti2cMutex())
    {
        uint8_t data;
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        // Select the correct register on the i2c device
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (dev << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, reg, true);
        // Send repeated start, and read the register
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (dev << 1) | I2C_MASTER_READ, true);
        // Read single byte and expect NACK in reply
        i2c_master_read_byte(cmd, &data, i2c_ack_type_t::I2C_MASTER_NACK);
        i2c_master_stop(cmd);
        // esp_err_t ret =
        ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(100)));

        // ESP_LOGD(TAG,"I2C reply %i",ret);

        i2c_cmd_link_delete(cmd);

        Releasei2cMutex();
        return data;
    }
    else
    {
        return 0;
    }
}

uint16_t HAL_ESP32::read16bitWord(i2c_port_t i2c_num, uint8_t dev, uint8_t reg)
{
    // We use the native i2c commands for ESP32 as the Arduino library
    // seems to have issues with corrupting i2c data if used from multiple threads
    if (Geti2cMutex())
    {
        uint8_t data1;
        uint8_t data2;
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        // Select the correct register on the i2c device
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (dev << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, reg, true);
        // Send repeated start, and read the register
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (dev << 1) | I2C_MASTER_READ, true);
        // Read 1 byte and expect ACK in reply
        i2c_master_read(cmd, &data1, 1, i2c_ack_type_t::I2C_MASTER_ACK);
        // Read 1 byte and expect NACK in reply
        i2c_master_read(cmd, &data2, 1, i2c_ack_type_t::I2C_MASTER_NACK);
        i2c_master_stop(cmd);
        // esp_err_t ret =
        ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(100)));

        i2c_cmd_link_delete(cmd);

        Releasei2cMutex();

        return (((uint16_t)data2 << 8) | (uint16_t)data1);
    }
    else
    {
        return 0;
    }
}

// i2c: Writes a single byte to a slave devices register
esp_err_t HAL_ESP32::write16bitWord(i2c_port_t i2c_num, uint8_t deviceAddress, uint8_t i2cregister, uint16_t data)
{
    if (Geti2cMutex())
    {
        // We use the native i2c commands for ESP32 as the Arduino library
        // seems to have issues with corrupting i2c data if used from multiple threads
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (deviceAddress << 1) | I2C_MASTER_WRITE, true);

        uint8_t buffer[3];
        buffer[0] = i2cregister;
        buffer[1] = (uint8_t)(data & 0x00FF);
        buffer[2] = (uint8_t)(data >> 8);
        i2c_master_write(cmd, buffer, sizeof(buffer), true);
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

// i2c: Writes a single byte to a slave devices register
esp_err_t HAL_ESP32::writeByte(i2c_port_t i2c_num, uint8_t deviceAddress, uint8_t i2cregister, uint8_t data)
{
    if (Geti2cMutex())
    {
        // We use the native i2c commands for ESP32 as the Arduino library
        // seems to have issues with corrupting i2c data if used from multiple threads
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (deviceAddress << 1) | I2C_MASTER_WRITE, true);

        uint8_t buffer[2];
        buffer[0] = i2cregister;
        buffer[1] = data;
        i2c_master_write(cmd, buffer, 2, true);

        // i2c_master_write_byte(cmd, i2cregister, true);
        // i2c_master_write_byte(cmd, data, true);
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

// 16 bit register
uint16_t HAL_ESP32::ReadTCA6416InputRegisters()
{
    // Update the copy of the pin state
    TCA6416_Input = read16bitWord(I2C_NUM_0, TCA6416_ADDRESS, TCA6416_INPUT) & (uint16_t)TCA6416_INPUTMASK;

    // As the TCA6416 pins as mapped similarly to the older style controller with 2 I/O chips
    // we map the inputs to fake the other devices.

    // TCA6416A pins P0-7 match the old TCA6408 pin out
    TCA6408_Input = TCA6416_Input & 0x00FF;
    TCA9534APWR_Input = (TCA6416_Input >> 8) & (uint16_t)0x00FF;

    // ESP_LOGD(TAG, "TCA6416 input=%#04x, TCA6408=%x, TCA9534=%x", TCA6416_Input, TCA6408_Input, TCA9534APWR_Input);

    return TCA6416_Input;
}

// 8 bit reg, returns masked state and updates pin state variable
uint8_t HAL_ESP32::ReadTCA6408InputRegisters()
{
    // We should never get here if TCA6416_Fitted is true
    ESP_ERROR_CHECK(TCA6416_Fitted ? ESP_FAIL : ESP_OK);
    TCA6408_Input = readByte(I2C_NUM_0, TCA6408_ADDRESS, TCA6408_INPUT) & TCA6408_INPUTMASK;
    return TCA6408_Input;
}

// 8 bit reg, returns masked state and updates pin state variable
uint8_t HAL_ESP32::ReadTCA9534InputRegisters()
{
    // We should never get here if TCA6416_Fitted is true
    ESP_ERROR_CHECK(TCA6416_Fitted ? ESP_FAIL : ESP_OK);

    // Update pin state copy
    TCA9534APWR_Input = readByte(I2C_NUM_0, TCA9534APWR_ADDRESS, TCA9534APWR_INPUT) & TCA9534APWR_INPUTMASK;
    return TCA9534APWR_Input;
}

void HAL_ESP32::WriteTCA6416OutputState()
{
    // Emulate the 9534 + 6408 and set the state on the 16 bit output
    TCA6416_Output_Pins = ((uint16_t)TCA9534APWR_Output_Pins << 8) | TCA6408_Output_Pins;
    ESP_ERROR_CHECK_WITHOUT_ABORT(write16bitWord(I2C_NUM_0, TCA6416_ADDRESS, TCA6416_OUTPUT, TCA6416_Output_Pins));

    // Update the "slave" pins to emulate those devices being installed
    TCA6408_Output_Pins = TCA6416_Output_Pins & 0x00FF;
    TCA9534APWR_Output_Pins = (TCA6416_Output_Pins >> 8) & 0x00FF;
}

void HAL_ESP32::WriteTCA9534APWROutputState()
{
    if (TCA6416_Fitted)
    {
        WriteTCA6416OutputState();
    }
    else
    {
        ESP_ERROR_CHECK_WITHOUT_ABORT(writeByte(I2C_NUM_0, TCA9534APWR_ADDRESS, TCA9534APWR_OUTPUT, TCA9534APWR_Output_Pins));
    }
}
void HAL_ESP32::WriteTCA6408OutputState()
{
    if (TCA6416_Fitted)
    {
        WriteTCA6416OutputState();
    }
    else
    {
        ESP_ERROR_CHECK_WITHOUT_ABORT(writeByte(I2C_NUM_0, TCA6408_ADDRESS, TCA6408_OUTPUT, TCA6408_Output_Pins));
    }
}

void HAL_ESP32::SetOutputState(uint8_t outputId, RelayState state)
{
    ESP_LOGD(TAG, "SetOutputState %u=%u", outputId, state);

    // Relays connected to TCA6408A
    // P4 = RELAY1 (outputId=0)
    // P5 = RELAY2 (outputId=1)
    // P6 = RELAY3_SSR (outputId=2)
    // P7 = RELAY4_SSR (outputId=3)
    if (outputId <= 3)
    {
        uint8_t bit = outputId + 4;
        TCA6408_Output_Pins = (state == RelayState::RELAY_ON) ? (TCA6408_Output_Pins | (1 << bit)) : (TCA6408_Output_Pins & ~(1 << bit));
        WriteTCA6408OutputState();
    }
}

void HAL_ESP32::Led(uint8_t bits)
{
    // Clear LED pins
    TCA9534APWR_Output_Pins = TCA9534APWR_Output_Pins & B11111000;
    // Set on
    TCA9534APWR_Output_Pins = TCA9534APWR_Output_Pins | (bits & B00000111);
    WriteTCA9534APWROutputState();
}

void HAL_ESP32::ConfigureCAN(uint16_t canbusbaudrate) const
{
    // Initialize configuration structures using macro initializers
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(gpio_num_t::GPIO_NUM_16, gpio_num_t::GPIO_NUM_17, TWAI_MODE_NORMAL);

    twai_timing_config_t t_config;
    if (canbusbaudrate == 250)
    {
        t_config = TWAI_TIMING_CONFIG_250KBITS();
    }
    else
    {
        //Default 500K rate
        t_config = TWAI_TIMING_CONFIG_500KBITS();
    }


    // We need the CAN ids:
    // * Pylontech LV battery: 0x305, 0x307
    // * Pylontech Force HV battery: 0x4200, 0x8200, 0x8210
    // from these ids we _can not_ derive a filter that makes sense, i.e.
    // twai_filter_config_t f_config = {
    //    .acceptance_code = 0x305<<21 & 0x307<<21 & 0x4200<<3 & 0x8200<<3 & 0x8210<<3, // 0
    //    .acceptance_mask = 0x305<<21 | 0x307<<21 | 0x4200<<3 | 0x8200<<3 | 0x8210<<3, // 0x60E61080
    //    .single_filter = true
    // };
    // ----> acceptance_code == 0, so we can only set ALL
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // Install CAN driver
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK)
    {
        ESP_LOGI(TAG, "CAN driver installed.  Filter=%u Mask=%u", f_config.acceptance_code, f_config.acceptance_mask);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to install CAN driver");
    }

    // Start CAN driver
    if (twai_start() == ESP_OK)
    {
        ESP_LOGI(TAG, "CAN driver started");
    }
    else
    {
        ESP_LOGE(TAG, "Failed to start CAN driver");
    }
}

// Control Silent mode control input on TJA1051T/3
// True = enable CANBUS
void HAL_ESP32::CANBUSEnable(bool value)
{
    if (TCA6416_Fitted)
    {
        // Not fitted/used on newer style PCBs, so ignore request
        return;
    }

    // Pin P5
    // Low = Normal mode
    // High = Silent
    TCA9534APWR_Output_Pins = TCA9534APWR_Output_Pins & B11011111;

    if (value == false)
    {
        // Set on
        TCA9534APWR_Output_Pins = TCA9534APWR_Output_Pins | B00100000;
    }

    WriteTCA9534APWROutputState();
}

// Control TFT backlight LED
void HAL_ESP32::TFTScreenBacklight(bool value)
{
    // Clear LED pins
    TCA9534APWR_Output_Pins = TCA9534APWR_Output_Pins & B11110111;

    if (value == true)
    {
        // Set on
        TCA9534APWR_Output_Pins = TCA9534APWR_Output_Pins | B00001000;
    }

    WriteTCA9534APWROutputState();
}

void HAL_ESP32::ConfigurePins()
{
    // GPIO39 is interrupt pin from TCA6408 (doesnt have pull up/down resistors)
    pinMode(TCA6408_INTERRUPT_PIN, INPUT);
    pinMode(TCA6416_INTERRUPT_PIN, INPUT);

    // GPIO34 is interrupt pin from TCA9534A (doesnt have pull up/down resistors)
    pinMode(TCA9534A_INTERRUPT_PIN, INPUT);

    // For touch screen
    // GPIO_NUM_36 no internal PULLUP
    pinMode(TOUCH_IRQ, INPUT);

    // Configure the CHIP SELECT pins as OUTPUT and set HIGH
    pinMode(TOUCH_CHIPSELECT, OUTPUT);
    digitalWrite(TOUCH_CHIPSELECT, HIGH);
    pinMode(SDCARD_CHIPSELECT, OUTPUT);
    digitalWrite(SDCARD_CHIPSELECT, HIGH);

    // Onboard INA229 current monitoring chip
    pinMode(INA229_INTERRUPT_PIN, INPUT);
    pinMode(INA229_CHIPSELECT, OUTPUT);
    digitalWrite(INA229_CHIPSELECT, HIGH);

    pinMode(RS485_ENABLE, OUTPUT);
    // Enable receive
    digitalWrite(RS485_ENABLE, LOW);

    pinMode(GPIO_NUM_0, OUTPUT);
    digitalWrite(GPIO_NUM_0, HIGH);

    pinMode(GPIO_NUM_35, INPUT);
    pinMode(GPIO_NUM_33, OUTPUT);
    digitalWrite(GPIO_NUM_33, HIGH);
}

struct isr_param
{
    uint8_t pin;
    void (*handler)(void);
};

static void ipc_interrupt_attach(void *param)
{
    isr_param *params = (isr_param *)param;
    attachInterrupt(params->pin, params->handler, FALLING);
}

// Attempts connection to i2c device
esp_err_t HAL_ESP32::Testi2cAddress(i2c_port_t port, uint8_t address)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(port, cmd, 100 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

void HAL_ESP32::ConfigureI2C(void (*TCA6408Interrupt)(void), void (*TCA9534AInterrupt)(void), void (*TCA6416Interrupt)(void))
{
    ESP_LOGI(TAG, "Configure I2C");

    // SDA / SCL
    // ESP32 = I2C0-SDA / I2C0-SCL
    // I2C Bus 1: uses GPIO 27 (SDA) and GPIO 26 (SCL);
    // I2C Bus 2: uses GPIO 33 (SDA) and GPIO 32 (SCL);

    // Initialize
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = gpio_num_t::GPIO_NUM_27,
        .scl_io_num = gpio_num_t::GPIO_NUM_26,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL};

    // 400khz
    conf.master.clk_speed = 400000;

    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0));

    // Test to see if we have a TCA6416 on the i2c bus, if we have, its a newer controller board V4.4+
    // on power up the chip has all ports are inputs

    ESP_LOGI(TAG, "Scanning i2c bus");
    for (uint8_t i = 1; i < 127; i++)
    {
        if (Testi2cAddress(I2C_NUM_0, i) == ESP_OK)
        {
            ESP_LOGI(TAG, "Found i2c device at address 0x%2x", i);
        }
    }

    if (Testi2cAddress(I2C_NUM_0, TCA6416_ADDRESS) == ESP_OK)
    {
        ESP_LOGI(TAG, "Found TCA6416A");

        ESP_ERROR_CHECK(write16bitWord(I2C_NUM_0, TCA6416_ADDRESS, TCA6416_OUTPUT, 0));

        TCA6416_Fitted = true;

        // Set configuration
        ESP_ERROR_CHECK(write16bitWord(I2C_NUM_0, TCA6416_ADDRESS, TCA6416_CONFIGURATION, TCA6416_INPUTMASK));

        // Update our copy of pin state registers
        ReadTCA6416InputRegisters();

        // All off
        TCA6416_Output_Pins = 0;
        WriteTCA6416OutputState();

        /*
    TCA6416A pins P0-7 match the old TCA6408 pin out
    */
        // P0=EXT_IO_A
        // P1=EXT_IO_B
        // P2=EXT_IO_C
        // P3=EXT_IO_D
        // P4=RELAY 1
        // P5=RELAY 2
        // P6=RELAY 3 (SSR)
        // P7=RELAY 4 (SSR)

        /*
    TCA6416 pins P10-17 match the old TCA9534 pin out
    */
        // P10= BLUE
        // P11= RED
        // P12= GREEN
        // P13= DISPLAY BACKLIGHT LED
        // P14= push button 1
        // P15= NOT USED
        // P16= push button 2
        // P17= ESTOP (pull to ground to trigger)

        // INTERRUPT PIN = ESP32 IO39
        isr_param tca6416_param = {.pin = TCA6416_INTERRUPT_PIN, .handler = TCA6416Interrupt};
        // ESP_ERROR_CHECK(esp_ipc_call_blocking(PRO_CPU_NUM, ipc_interrupt_attach, &tca6416_param));
        ipc_interrupt_attach(&tca6416_param);

        return;
    }
    else
    {
        ESP_LOGI(TAG, "TCA6416A not fitted, assume v4.2 board");

        // https://datasheet.lcsc.com/szlcsc/1809041633_Texas-Instruments-TCA9534APWR_C206010.pdf
        // TCA9534APWR Remote 8-Bit I2C and Low-Power I/O Expander With Interrupt Output and Configuration Registers
        // https://lcsc.com/product-detail/Interface-I-O-Expanders_Texas-Instruments-TCA9534APWR_C206010.html
        // A0/A1/A2 are LOW, so i2c address is 0x38

        // PINS
        // P0= BLUE
        // P1= RED
        // P2= GREEN
        // P3= DISPLAY BACKLIGHT LED
        // P4= SPARE on J13
        // P5= Canbus RS
        // P6= SPARE on J13
        // P7= ESTOP (pull to ground to trigger)
        // INTERRUPT PIN = ESP32 IO34

        // BIT  76543210
        // PORT 76543210
        // MASK=10000000

        if (Testi2cAddress(I2C_NUM_0, TCA9534APWR_ADDRESS) != ESP_OK)
        {
            ESP_LOGE(TAG, "TCA9534APWR Error");
            Halt(RGBLED::Purple);
        }

        // All off
        ESP_ERROR_CHECK(writeByte(I2C_NUM_0, TCA9534APWR_ADDRESS, TCA9534APWR_OUTPUT, 0));

        // 0×03 Configuration, P7 (estop) and P4 (remote touch) as input, others outputs (0=OUTPUT)
        ESP_ERROR_CHECK(writeByte(I2C_NUM_0, TCA9534APWR_ADDRESS, TCA9534APWR_CONFIGURATION, TCA9534APWR_INPUTMASK));
        ReadTCA9534InputRegisters();
        TCA9534APWR_Output_Pins = readByte(I2C_NUM_0, TCA9534APWR_ADDRESS, TCA9534APWR_OUTPUT);

        // ESP_LOGD(TAG, "About to configure interrupt...");
        isr_param tca9534_param = {.pin = TCA9534A_INTERRUPT_PIN, .handler = TCA9534AInterrupt};
        // ESP_ERROR_CHECK(esp_ipc_call_blocking(PRO_CPU_NUM, ipc_interrupt_attach, &tca9534_param));
        ipc_interrupt_attach(&tca9534_param);

        ESP_LOGI(TAG, "Found TCA9534A");

        /*
    Now for the TCA6408
    */

        // P0=EXT_IO_A
        // P1=EXT_IO_B
        // P2=EXT_IO_C
        // P3=EXT_IO_D
        // P4=RELAY 1
        // P5=RELAY 2
        // P6=RELAY 3 (SSR)
        // P7=RELAY 4 (SSR)

        if (Testi2cAddress(I2C_NUM_0, TCA6408_ADDRESS) != ESP_OK)
        {
            ESP_LOGE(TAG, "TCA6408 Error");
            Halt(RGBLED::Green);
        }

        // Set ports to off before we set configuration
        ESP_ERROR_CHECK(writeByte(I2C_NUM_0, TCA6408_ADDRESS, TCA6408_OUTPUT, 0));

        // Ports A/B/C/D/E inputs, RELAY1/2/3 outputs
        ESP_ERROR_CHECK(writeByte(I2C_NUM_0, TCA6408_ADDRESS, TCA6408_CONFIGURATION, TCA6408_INPUTMASK));
        ReadTCA6408InputRegisters();
        TCA6408_Output_Pins = readByte(I2C_NUM_0, TCA6408_ADDRESS, TCA6408_OUTPUT);

        ESP_LOGI(TAG, "Found TCA6408");
        isr_param tca6408_param = {.pin = TCA6408_INTERRUPT_PIN, .handler = TCA6408Interrupt};
        // ESP_ERROR_CHECK(esp_ipc_call_blocking(PRO_CPU_NUM, ipc_interrupt_attach, &tca6408_param));
        ipc_interrupt_attach(&tca6408_param);
        TCA6416_Fitted = false;

        return;
    }

    // Didn't find anything on i2c bus
    ESP_LOGE(TAG, "No TCA chip found on i2c bus");
    Halt(RGBLED::Cyan);
}

TouchScreenValues HAL_ESP32::TouchScreenUpdate()
{
    /*
        Features and Specification of XPT2046
        * 12 bit SAR type A/D converter with S/H circuit
        * Low voltage operations (VCC = 2.2V - 3.6V)
        * Low power consumption (260uA)
        * 125 kHz sampling frequency
        * On-chip reference voltage - 2.5V
        * Pen pressure measurement
        * On-chip thermo sensor
        * Direct battery measurement
        * 4-wire I/F

        DataSheet
        https://components101.com/admin/sites/default/files/component_datasheet/XPT2046-Datasheet.pdf

        // BIT7(MSB) BIT6 BIT5 BIT4 BIT3 BIT2      BIT1 BIT0(LSB)
        // S         A2   A1   A0   MODE SER/——DFR PD1  PD0

        // S = START BIT
        // A2/A1/A0 = Channel address bits 011 = Z1, 100=Z2, 001=X, 101=Y
        // MODE = 12bit/8bit (12bit=low)
        // SER/DFR = SingleEnded/Differential ADC mode (differential mode)
        // PD0 and PD1 control power delivery, 0/0 = all enabled, 1=1=disabled.
    */
    // Z is touch (depth)
    // const uint8_t ADD_TEMPERATURE = 0B00000000;
    // const uint8_t ADD_VBATTERY = 0B00100000;
    const uint8_t ADD_Z1 = 0B00110000;
    const uint8_t ADD_Z2 = 0B01000000;
    const uint8_t ADD_X = 0B00010000;
    const uint8_t ADD_Y = 0B01010000;
    const uint8_t STARTBIT = 0B10000000;
    // internal reference voltage is only used in the single-ended mode for battery monitoring, temperature measurement
    const uint8_t PWR_ALLOFF = 0B00000000;
    const uint8_t PWR_REFOFF_ADCON = 0B00000001;
    // const uint8_t PWR_REFON_ADCOFF = 0B00000010;
    // const uint8_t PWR_REFON_ADCON = 0B00000011;

    // Differential Reference Mode (SER/DFR low) should be used for reading X/Y/Z
    // Single ended mode (SER/DFR high) should be used for temperature and battery
    const uint8_t ADC_DIFFERENTIAL = 0B00000000;
    // const uint8_t ADC_SINGLEENDED = 0B00000100;

    const uint8_t ADC_8BIT = 0B00001000;
    const uint8_t ADC_12BIT = 0B00000000;

    TouchScreenValues reply;
    reply.touched = false;
    reply.pressure = 0;
    reply.X = 0;
    reply.Y = 0;

    // Landscape orientation screen
    // X is zero when not touched, left of screen is about 290, right of screen is about 3900
    // Y is 3130 when not touched, top of screen is 250, bottom is 3150

    if (GetVSPIMutex())
    {
        // Slow down to 2Mhz SPI bus
        vspi.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));

        digitalWrite(TOUCH_CHIPSELECT, LOW);

        // We don't need accurate touch pressure for this application, so
        // only read Z2 register, we just want a boolean value at the end of the day

        // The transfers are always a step behind, so the transfer reads the previous value/command
        vspi.write(STARTBIT | PWR_REFOFF_ADCON | ADC_8BIT | ADC_DIFFERENTIAL | ADD_Z1);

        // Read Z2 (1 byte) and request X
        reply.pressure = vspi.transfer(STARTBIT | PWR_REFOFF_ADCON | ADC_12BIT | ADC_DIFFERENTIAL | ADD_X);

        // Read X (2 bytes) and request Y
        reply.X = vspi.transfer16(STARTBIT | PWR_REFOFF_ADCON | ADC_12BIT | ADC_DIFFERENTIAL | ADD_Y) >> 3;
        // Take Y reading, then power down after this sample
        reply.Y = vspi.transfer16(STARTBIT | PWR_ALLOFF | ADC_8BIT | ADC_DIFFERENTIAL | ADD_Z2) >> 3;

        // Final transfer to ensure device goes into sleep
        // In order to turn the reference off, an additional write to the XPT2046 is required after the channel has been converted.  Page 24 datasheet
        // uint16_t Z2 =
        vspi.transfer(0);

        digitalWrite(TOUCH_CHIPSELECT, HIGH);
        vspi.endTransaction();

        ReleaseVSPIMutex();

        // Screen connected and not touched
        // Touch = 128, x=0 , y around 3130-3140

        // 135 controls the minimum amount of pressure needed to "touch"
        // X also needs to be greater than zero
        reply.touched = reply.pressure > 135 && reply.X > 0;

        // ESP_LOGI(TAG, "Touch = touch=%i pressure=%u x=%u y=%u", reply.touched, reply.pressure, reply.X, reply.Y);
    }

    return reply;
}

bool HAL_ESP32::IsScreenAttached()
{
    TouchScreenValues v = TouchScreenUpdate();
    // ESP_LOGD(TAG,"Touch pressure=%u, X=%u, Y=%u",v.pressure, v.X, v.Y);
    return !(v.pressure == 0 && v.X == 0 && v.Y == 0);
}

void HAL_ESP32::ConfigureVSPI()
{
    ESP_LOGD(TAG, "Configure VSPI");
    vspi.endTransaction();
    vspi.end();
    // VSPI

    // Don't use hardware chip selects on VSPI
    vspi.setHwCs(false);
    vspi.setBitOrder(MSBFIRST);
    vspi.setDataMode(SPI_MODE0);
    // 10mhz
    vspi.setFrequency(10000000);

    // GPIO23 (MOSI), GPIO19(MISO), GPIO18(CLK) and GPIO5 (CS)
    vspi.begin(GPIO_NUM_18, GPIO_NUM_19, GPIO_NUM_23, -1);
}
