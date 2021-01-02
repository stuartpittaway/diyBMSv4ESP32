#include "avrisp_programmer.h"

AVRISP_PROGRAMMER_RESULT AVRISP_PROGRAMMER::ProgramAVRDevice(uint32_t deviceid, uint32_t byteLength, const uint8_t *dataToProgram, uint8_t fuse, uint8_t fusehigh, uint8_t fuseext)
{
    //May need a slower frequency for different devices
    _freq = 1000000 / 2;
    
    //ATTINY841
    if (deviceid == 0x1e9315)
    {
        //Data for ATTINY841
        //Page Size = 8 words (16 bytes), Flash = 4K words (8kbytes), 512 Pages
        //Wait delays for programming, WD_FLASH=4.5ms, WD_EEROM=3.6ms, WD_ERASE=9.0m

        //Set the timing values for the ATTINY841 (plus extra milliseconds for good luck)
        WD_FLASH = 5;
        WD_ERASE = 10;
        WD_EEPROM = 4;
        PAGE_SIZE_WORDS = 8;
        NUMBER_OF_PAGES = 512;
    }

    //ATMEGA328P (typical Arduino)
    if (deviceid == 0x1E950F)
    {
        //Data for ATMEGA328P
        //https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf
        //Page Size = 64 words (128 bytes), Flash = 16K words (32Kbytes), 256 Pages
        //Wait delays for programming, WD_FLASH=4.5ms, WD_EEROM=3.6ms, WD_ERASE=9.0m

        WD_FLASH = 5;
        WD_ERASE = 10;
        WD_EEPROM = 4;

        PAGE_SIZE_WORDS = 64;
        NUMBER_OF_PAGES = 256;
    }

    
    _spi->setFrequency(_freq);
    _spi->beginTransaction(SPISettings(_freq, MSBFIRST, SPI_MODE0));

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

    if (PAGE_SIZE_WORDS * NUMBER_OF_PAGES == 0)
    {
        return AVRISP_PROGRAMMER_RESULT::INCORRECT_PAGE_CONFIG;
    }

    //Chip ERASE....
    TransferByte4(0xAC, 0x80, 0, 0);
    delay(WD_ERASE);

    //Set Address = 0
    //HERE is in WORDS (16 bit)
    uint16_t page = 0;
    uint16_t byteCounter = 0;

    //Program page by page
    //uint16_t page = current_page(here, 16);
    while (byteCounter < byteLength)
    {
        //SERIAL_DEBUG.printf("%.4X", x);
        //SERIAL_DEBUG.print(' ');

        uint8_t lowbyte;
        uint8_t highbyte;
        //Count up in WORDS
        for (size_t i = 0; i < PAGE_SIZE_WORDS; i++)
        {
            if (byteCounter < byteLength)
            {
                lowbyte = dataToProgram[byteCounter];
                byteCounter++;
                highbyte = dataToProgram[byteCounter];
                byteCounter++;
            }
            else
            {
                //Prevent buffer overrun... Fill remaining page bytes with 0xFF
                lowbyte = 0xFF;
                highbyte = 0xFF;
                byteCounter = byteCounter + 2;
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

    byteCounter = 0;
    for (size_t wordPtr = 0; wordPtr < NUMBER_OF_PAGES * PAGE_SIZE_WORDS; wordPtr++)
    {
        /*
            if (i % 8 == 0)
            {
                SERIAL_DEBUG.println();
                SERIAL_DEBUG.printf("%.4X", x);
                SERIAL_DEBUG.print(' ');
            }
            */

        uint8_t MSB = (wordPtr >> 8) & 0xFF;
        uint8_t LSB = wordPtr & 0x00FF;
        //Read Program Memory, Low byte
        uint8_t lowbyte = TransferByte4(0x20, MSB, LSB, 0);
        //dumpByte(lowbyte);

        if (byteCounter < byteLength && dataToProgram[byteCounter] != lowbyte)
        {
            //SERIAL_DEBUG.print("*FAIL*");
            return AVRISP_PROGRAMMER_RESULT::FAIL_VERIFY;
        }

        byteCounter++;

        //Read Program Memory, High byte
        uint8_t highbyte = TransferByte4(0x28,
                                         MSB,
                                         LSB,
                                         0);
        //dumpByte(highbyte);

        if (byteCounter < byteLength && dataToProgram[byteCounter] != highbyte)
        {
            return AVRISP_PROGRAMMER_RESULT::FAIL_VERIFY;
        }

        byteCounter++;

        if (byteCounter >= byteLength)
            break;
    }
    //SERIAL_DEBUG.println();

    //Read Fuse bytes
    lfuse = TransferByte4(0x50, 0x00, 0x00, 0x00);
    //Read Fuse High
    hfuse = TransferByte4(0x58, 0x08, 0x00, 0x00);
    //Read Fuse Extended
    efuse = TransferByte4(0x50, 0x08, 0x00, 0x00);

    //Program fuse bytes (if needed)
    if (fuse != lfuse)
    {
        TransferByte4(0xAC, 0xA0, 0x00, fuse);
        delay(WD_FLASH);
    }
    if (fusehigh != hfuse)
    {
        TransferByte4(0xAC, 0xA8, 0x00, fusehigh);
        delay(WD_FLASH);
    }
    if (fuseext != efuse)
    {
        TransferByte4(0xAC, 0xA4, 0x00, fuseext);
        delay(WD_FLASH);
    }

    //Read Fuse bits to confirm correct programming
    lfuse = TransferByte4(0x50, 0x00, 0x00, 0x00);
    hfuse = TransferByte4(0x58, 0x08, 0x00, 0x00);
    efuse = TransferByte4(0x50, 0x08, 0x00, 0x00);

    if (lfuse != fuse || hfuse != fusehigh || efuse != fuseext)
    {
        return AVRISP_PROGRAMMER_RESULT::FAIL_PROG_FUSE;
    }

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

    //Exit programming mode

    _spi->endTransaction();

    avr_reset_target(false);

    return AVRISP_PROGRAMMER_RESULT::SUCCESS;
}
