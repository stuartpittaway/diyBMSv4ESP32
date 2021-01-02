
#ifndef avrisp_program_H_
#define avrisp_program_H_

#include <Arduino.h>
#include <SPI.h>

enum AVRISP_PROGRAMMER_RESULT : uint8_t
{
    SUCCESS = 0,
    WRONG_DEVICE_ID = 1,
    COMMIT_FAIL = 2,
    FAILED_ENTER_PROG_MODE = 3,
    FAIL_VERIFY = 4,
    FAIL_PROG_FUSE=5,
    INCORRECT_PAGE_CONFIG=6
};

class AVRISP_PROGRAMMER
{
public:
    AVRISP_PROGRAMMER(SPIClass *spi, uint8_t resetGPIO, bool resetActiveHigh, uint8_t spiClockGPIO)
    {
        _spi = spi;
        _resetGPIO = resetGPIO;
        _resetActiveHigh = resetActiveHigh;
        _spiClockGPIO = spiClockGPIO;
    }

    uint32_t device_signature;
    //Fuse Low
    uint8_t lfuse;
    //Fuse High
    uint8_t hfuse;
    //Fuse Extended
    uint8_t efuse;

    AVRISP_PROGRAMMER_RESULT ProgramAVRDevice(uint32_t deviceid, uint32_t byteLength, const uint8_t *dataToProgram, uint8_t fuse, uint8_t fusehigh, uint8_t fuseext);

private:
    SPIClass *_spi = NULL;

    uint8_t _spiClockGPIO;
    uint8_t _resetGPIO;
    bool _resetActiveHigh;
    uint32_t _freq;

    uint8_t WD_FLASH = 50;
    uint8_t WD_ERASE = 50;
    uint8_t WD_EEPROM = 50;
    uint8_t PAGE_SIZE_WORDS = 0;
    uint16_t NUMBER_OF_PAGES=0;

    void avr_reset_target(bool reset)
    {
        digitalWrite(_resetGPIO, ((reset && _resetActiveHigh) || (!reset && !_resetActiveHigh)) ? HIGH : LOW);
    }

    //Returns the 4th byte of the SPI transfer
    uint8_t TransferByte4(uint8_t a, uint8_t b, uint8_t c, uint8_t d)
    {
        _spi->transfer(a);
        _spi->transfer(b);
        _spi->transfer(c);
        return _spi->transfer(d);
    }

    //Returns the result of the 3rd byte (used for checking programming mode)
    uint8_t TransferByte3(uint8_t a, uint8_t b, uint8_t c, uint8_t d)
    {
        _spi->transfer(a);
        _spi->transfer(b);
        uint8_t replyc = _spi->transfer(c);
        _spi->transfer(d);
        return replyc;
    }
};

#endif
