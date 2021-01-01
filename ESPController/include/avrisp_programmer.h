
#ifndef avrisp_program_H_
#define avrisp_program_H_

#include <Arduino.h>

enum AVRISP_PROGRAMMER_RESULT : uint8_t
{
    SUCCESS = 0,
    WRONG_DEVICE_ID = 1,
    COMMIT_FAIL = 2,
    FAILED_ENTER_PROG_MODE = 3,
    FAIL_VERIFY = 4
};

class AVRISP_PROGRAMMER
{
public:
    AVRISP_PROGRAMMER(SPIClass *spi, uint8_t resetGPIO, bool resetActiveHigh, uint8_t spiClockGPIO)
    {
        _spi = spi;
        _resetGPIO = resetGPIO;
        _resetActiveHigh=resetActiveHigh;
        _spiClockGPIO=spiClockGPIO;
    }

    uint32_t device_signature;
    //Fuse Low
    uint8_t lfuse;
    //Fuse High
    uint8_t hfuse;
    //Fuse Extended
    uint8_t efuse;

    AVRISP_PROGRAMMER_RESULT ProgramAVRDevice(uint32_t deviceid, uint32_t byteLength, const uint8_t *dataToProgram)
    {
        uint32_t freq = (1000000 / 6);
        _spi->setFrequency(freq);
        _spi->beginTransaction(SPISettings(freq, MSBFIRST, SPI_MODE0));

        avr_reset_target(true);

        digitalWrite(_spiClockGPIO, LOW);
        delay(20); // discharge PIN_SCK, value arbitrarily chosen
        avr_reset_target(false);
        // Pulse must be minimum 2 target CPU clock cycles so 100 usec is ok for CPU speeds above 20 KHz
        delayMicroseconds(100);
        avr_reset_target(true);

        // Send the enable programming command:
        delay(30); // datasheet: must be > 20 msec
        uint8_t programmingreply = TransferByte3(0xAC, 0x53, 0x00, 0x00);

        if (programmingreply != 0x53)
        {
            //SERIAL_DEBUG.println("** Start programming mode failed **");
            return AVRISP_PROGRAMMER_RESULT::FAILED_ENTER_PROG_MODE;
        }

        device_signature = (TransferByte4(0x30, 0x00, 0x00, 0x00) << 16) + (TransferByte4(0x30, 0x00, 0x01, 0x00) << 8) + TransferByte4(0x30, 0x00, 0x02, 0x00);

        //SERIAL_DEBUG.print("Device Signature=");
        //SERIAL_DEBUG.println(device_signature, HEX);

        if (device_signature != deviceid)
        {
            return AVRISP_PROGRAMMER_RESULT::WRONG_DEVICE_ID;
        }

        //ATTINY841
        if (device_signature == 0x1e9315)
        {
            //Data for ATTINY841
            //Page Size = 8 words (16 bytes), Flash = 4K words (8kbytes), 512 Pages
            //Wait delays for programming, WD_FLASH=4.5ms, WD_EEROM=3.6ms, WD_ERASE=9.0m

            //Set the timing values for the ATTINY841 (plus extra milliseconds for good luck)
            WD_FLASH = 8;
            WD_ERASE = 10;
            WD_EEPROM = 5;
            PAGE_SIZE_WORDS = 8;
        }

        //SERIAL_DEBUG.println("ATTINY841");
        //Read Fuse bits
        lfuse = TransferByte4(0x50, 0x00, 0x00, 0x00);
        //Read Fuse High bits
        hfuse = TransferByte4(0x58, 0x08, 0x00, 0x00);
        //Read Fuse Extended Bits
        efuse = TransferByte4(0x50, 0x08, 0x00, 0x00);

        /*
            SERIAL_DEBUG.print("Fuses (");
            SERIAL_DEBUG.print(" E:");
            dumpByte(efuse);
            SERIAL_DEBUG.print(" H:");
            dumpByte(hfuse);
            SERIAL_DEBUG.print(" L:");
            dumpByte(lfuse);
            SERIAL_DEBUG.println(")");
*/

        //Chip ERASE....
        TransferByte4(0xAC, 0x80, 0, 0);
        delay(WD_ERASE);

        //Set Address = 0
        //HERE is in WORDS (16 bit)
        uint16_t page = 0;
        uint16_t x = 0;

        //Program page by page
        //uint16_t page = current_page(here, 16);
        while (x < byteLength)
        {
            //SERIAL_DEBUG.printf("%.4X", x);
            //SERIAL_DEBUG.print(' ');

            uint8_t lowbyte;
            uint8_t highbyte;
            //Count up in WORDS
            for (size_t i = 0; i < PAGE_SIZE_WORDS; i++)
            {
                if (x < byteLength)
                {
                    lowbyte = dataToProgram[x];
                    x++;
                    highbyte = dataToProgram[x];
                    x++;
                }
                else
                {
                    //Prevent buffer overrun... Fill remaining page bytes with 0xFF
                    lowbyte = 0xFF;
                    highbyte = 0xFF;
                    x = x + 2;
                }

                //This will need modifying if pages are larger than 64 bytes

                //Load Program Memory Page, Low byte
                TransferByte4(0x40,
                              0,
                              i & B00111111,
                              lowbyte);

                //Load Program Memory Page, High byte
                TransferByte4(0x48,
                              0,
                              i & B00111111,
                              highbyte);
            }

            //Commit page to FLASH
            uint8_t reply = TransferByte4(0x4C, (page >> 8) & 0xFF, page & 0xFF, 0);

            if (reply != (page & 0xFF))
            {
                return AVRISP_PROGRAMMER_RESULT::COMMIT_FAIL;
            }

            //SERIAL_DEBUG.println();

            page += PAGE_SIZE_WORDS;

            //Delay (as per data sheet) to allow FLASH to program....
            delay(WD_FLASH);
        }

        //Now verify the program was written correctly...

        x = 0;
        for (size_t i = 0; i < 512 * 8; i++)
        {
            /*
            if (i % 8 == 0)
            {
                SERIAL_DEBUG.println();
                SERIAL_DEBUG.printf("%.4X", x);
                SERIAL_DEBUG.print(' ');
            }
            */

            uint8_t MSB = (i >> 8) & 0xFF;
            uint8_t LSB = i & 0x00FF;
            //Read Program Memory, Low byte
            uint8_t lowbyte = TransferByte4(0x20, MSB, LSB, 0);
            //dumpByte(lowbyte);

            if (x < byteLength && dataToProgram[x] != lowbyte)
            {
                //SERIAL_DEBUG.print("*FAIL*");
                return AVRISP_PROGRAMMER_RESULT::FAIL_VERIFY;
            }

            x++;

            //Read Program Memory, High byte
            uint8_t highbyte = TransferByte4(0x28,
                                             MSB,
                                             LSB,
                                             0);
            //dumpByte(highbyte);

            if (x < byteLength && dataToProgram[x] != highbyte)
            {
                //SERIAL_DEBUG.print("*FAIL*");
                return AVRISP_PROGRAMMER_RESULT::FAIL_VERIFY;
            }

            x++;

            if (x >= byteLength)
                break;
        }
        //SERIAL_DEBUG.println();

        //Exit programming mode

        _spi->endTransaction();

        avr_reset_target(false);

        return AVRISP_PROGRAMMER_RESULT::SUCCESS;
    }

private:
    SPIClass *_spi = NULL;
    
    uint8_t _spiClockGPIO;
    uint8_t _resetGPIO;
    bool _resetActiveHigh;

    uint8_t WD_FLASH = 50;
    uint8_t WD_ERASE = 50;
    uint8_t WD_EEPROM = 50;
    uint8_t PAGE_SIZE_WORDS = 8;

    void avr_reset_target(bool reset)
    {
        digitalWrite(_resetGPIO, ((reset && _resetActiveHigh) || (!reset && !_resetActiveHigh)) ? HIGH : LOW);
    }

    uint8_t TransferByte4(uint8_t a, uint8_t b, uint8_t c, uint8_t d)
    {
        /*
        SERIAL_DEBUG.print("[");
        dumpByte(a);SERIAL_DEBUG.print(",");
        dumpByte(b);SERIAL_DEBUG.print(",");
        dumpByte(c);SERIAL_DEBUG.print(",");
        dumpByte(d);SERIAL_DEBUG.println("]");
*/
        _spi->transfer(a);
        _spi->transfer(b);
        _spi->transfer(c);
        return _spi->transfer(d);
    }

    uint8_t TransferByte3(uint8_t a, uint8_t b, uint8_t c, uint8_t d)
    {
        //Returns the result of the 3rd byte (used for checking programming mode)
        _spi->transfer(a);
        _spi->transfer(b);
        uint8_t replyc = _spi->transfer(c);
        _spi->transfer(d);
        return replyc;
    }
};

#endif
