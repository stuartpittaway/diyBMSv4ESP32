#if defined(ESP32)
#include "defines.h"
#include "HAL_ESP32.h"

bool writeByte(uint8_t dev, uint8_t reg, uint8_t data);
int8_t readBytes(uint8_t dev, uint8_t reg, uint8_t size, uint8_t *data);
uint8_t readByte(uint8_t dev, uint8_t reg);

int8_t readBytes(uint8_t dev, uint8_t reg, uint8_t size, uint8_t *data)
{
    Wire.beginTransmission(dev);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(dev, size);
    int8_t count = 0;
    while (Wire.available())
        data[count++] = Wire.read();
    return count;
}

uint8_t readByte(uint8_t dev, uint8_t reg)
{
    uint8_t data;
    readBytes(dev, reg, 1, &data);
    return data;
}

bool writeBit(uint8_t dev, uint8_t reg, uint8_t bit, uint8_t data)
{
    uint8_t b = readByte(dev, reg);
    b = (data != 0) ? (b | (1 << bit)) : (b & ~(1 << bit));
    return writeByte(dev, reg, b);
}

bool writeBytes(uint8_t dev, uint8_t reg, uint8_t size, uint8_t *data)
{
    Wire.beginTransmission(dev);
    Wire.write(reg);
    for (uint8_t i = 0; i < size; i++)
        Wire.write(data[i]);
    uint8_t sts = Wire.endTransmission();
    if (sts != 0)
    {
        SERIAL_DEBUG.print("I2C ERROR : ");
        SERIAL_DEBUG.println(sts);
    }
    return (sts == 0);
}

bool writeByte(uint8_t deviceAddress, uint8_t i2cregister, uint8_t data)
{
    Wire.beginTransmission(deviceAddress);
    Wire.write(i2cregister);
    Wire.write(data);
    uint8_t sts = Wire.endTransmission();
    if (sts != 0)
    {
        SERIAL_DEBUG.print("I2C ERROR : ");
        SERIAL_DEBUG.println(sts);
    }
    return (sts == 0);
}

uint8_t HAL_ESP32::ReadInputRegisters()
{
    return 0;
}

void HAL_ESP32::SetOutputState(uint8_t outputId, RelayState state)
{
    SERIAL_DEBUG.print("SetOutputState ");
    SERIAL_DEBUG.print(outputId);
    SERIAL_DEBUG.print("=");
    SERIAL_DEBUG.println(state);
    
    if (OutputsEnabled)
    {
        //Relays connected to TCA6408A
        //P4 = RELAY1 (outputId=0)
        //P5 = RELAY2 (outputId=1)
        //P6 = RELAY3_SSR (outputId=2)
        //P7 = EXT_IO_E (outputId=3)

        if (outputId >= 0 && outputId <= 3)
        {
            TCA6408_Value = readByte(TCA6408_ADDRESS, TCA6408_INPUT);
            uint8_t bit = outputId + 3;
            TCA6408_Value = (state == RelayState::RELAY_ON) ? (TCA6408_Value | (1 << bit)) : (TCA6408_Value & ~(1 << bit));
            writeByte(TCA6408_ADDRESS, TCA6408_OUTPUT, TCA6408_Value);
        }
    }
}

void HAL_ESP32::GreenLedOn()
{
    //Clear LED pins
    TCA9534APWR_Value = TCA9534APWR_Value & B11111000;
    //Green on
    TCA9534APWR_Value = TCA9534APWR_Value | B00000100;
    writeByte(TCA9534APWR_ADDRESS, TCA9534APWR_OUTPUT, TCA9534APWR_Value);
}

void HAL_ESP32::GreenLedOff()
{
    //Clear LED pins
    TCA9534APWR_Value = TCA9534APWR_Value & B11111000;
    writeByte(TCA9534APWR_ADDRESS, TCA9534APWR_OUTPUT, TCA9534APWR_Value);
}

void HAL_ESP32::ConfigurePins()
{
}

void HAL_ESP32::ConfigureI2C(void (*ExternalInputInterrupt)(void))
{
    SERIAL_DEBUG.println("ConfigureI2C");

    //SDA / SCL
    //ESP32 = I2C0-SDA / I2C0-SCL
    //I2C Bus 1: uses GPIO 27 (SDA) and GPIO 26 (SCL);
    //I2C Bus 2: uses GPIO 33 (SDA) and GPIO 32 (SCL);

    //Use 100khz i2c
    Wire.begin(27, 26, 100000UL);

    /*
    byte count = 0;
    for (byte i = 8; i < 120; i++)
    {
        Wire.beginTransmission(i);       // Begin I2C transmission Address (i)
        if (Wire.endTransmission() == 0) // Receive 0 = success (ACK response)
        {
            SERIAL_DEBUG.print("Found address: ");
            SERIAL_DEBUG.print(i, DEC);
            SERIAL_DEBUG.print(" (0x");
            SERIAL_DEBUG.print(i, HEX); // PCF8574 7 bit address
            SERIAL_DEBUG.println(")");
            count++;
        }
    }
    SERIAL_DEBUG.print("Found ");
    SERIAL_DEBUG.print(count, DEC); // numbers of devices
    SERIAL_DEBUG.println(" device(s).");
*/
    // https://datasheet.lcsc.com/szlcsc/1809041633_Texas-Instruments-TCA9534APWR_C206010.pdf
    // TCA9534APWR Remote 8-Bit I2C and Low-Power I/O Expander With Interrupt Output and Configuration Registers
    // https://lcsc.com/product-detail/Interface-I-O-Expanders_Texas-Instruments-TCA9534APWR_C206010.html
    // A0/A1/A2 are LOW, so i2c address is 0x38

    //PINS
    //P0= BLUE
    //P1= RED
    //P2= GREEN
    //P3= DISPLAY BACKLIGHT LED
    //P4= AVRISP RESET
    //P5/P6/P7 = EXTRA I/O (on internal header breakout pins)
    //INTERRUPT PIN = ESP32 IO34

    //Config all OUTPUT
    //BIT  76543210
    //PORT 76543210

    Wire.beginTransmission(TCA9534APWR_ADDRESS); // Begin I2C transmission Address (i)
    if (Wire.endTransmission() == 0)             // Receive 0 = success (ACK response)
    {
        //All off
        writeByte(0x38, TCA9534APWR_OUTPUT, 0);
        //0×03 Configuration, P5/6/7=inputs, others outputs (0=OUTPUT)
        writeByte(TCA9534APWR_ADDRESS, TCA9534APWR_CONFIGURATION, B11100000);
        //0×02 Polarity Inversion, zero = off
        //writeByte(TCA9534APWR_ADDRESS, TCA9534APWR_POLARITY_INVERSION, 0);
        TCA9534APWR_Value = readByte(TCA9534APWR_ADDRESS, 0);
        SERIAL_DEBUG.println("Found TCA9534APWR");
    }
    else
    {
        SERIAL_DEBUG.println("** Missing TCA9534APWR **");
    }

    //uint8_t portIOValue;
    //Read single byte
    //readBytes(0x38, 1, 1, &portIOValue);
    /*
while (1) {
    //portIOValue = 0xFF;
    //0×01 Output Port
    writeByte(0x38, 1, 0);    
    delay(500);

    writeByte(0x38, 1, B00000001);    
    delay(500);

    writeByte(0x38, 1, B00000010);    
    delay(500);
    writeByte(0x38, 1, B00000100);    
    delay(500);

    writeByte(0x38, 1, B00000110);    
    delay(500);

    writeByte(0x38, 1, B00000101);    
    delay(500);
    writeByte(0x38, 1, B00000011);    
    delay(500);
}
*/

    //Need to configure interrupt pins!

    Wire.beginTransmission(TCA6408_ADDRESS); // Begin I2C transmission Address (i)
    if (Wire.endTransmission() == 0)         // Receive 0 = success (ACK response)
    {
        // 0x20 https://www.ti.com/lit/ds/symlink/tca6408a.pdf
        // TCA6408A TCA6408A Low-Voltage 8-Bit I2C and SMBus I/O Expander With Interrupt Output, Reset, and Configuration Registers

        //Set ports to off before we set configuration
        writeByte(TCA6408_ADDRESS, TCA6408_OUTPUT, 0);
        //Ports A/B inputs, C/D outputs, RELAY1/2/3/SPARE outputs
        writeByte(TCA6408_ADDRESS, TCA6408_CONFIGURATION, B00000011);
        //writeByte(TCA6408_ADDRESS, TCA6408_POLARITY_INVERSION, B11111111);
        TCA6408_Value = readByte(TCA6408_ADDRESS, TCA6408_INPUT);
        OutputsEnabled = true;
        InputsEnabled = true;
        SERIAL_DEBUG.println("Found TCA6408");
    }
    else
    {
        SERIAL_DEBUG.println("** Missing TCA6408 **");
    }

    //Need to configure interrupt pins!
}

#endif