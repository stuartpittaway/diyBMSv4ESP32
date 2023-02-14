/*
(c)2021 Stuart Pittaway

LICENSE
Attribution-NonCommercial-ShareAlike 2.0 UK: England & Wales (CC BY-NC-SA 2.0 UK)
https://creativecommons.org/licenses/by-nc-sa/2.0/uk/

* Non-Commercial — You may not use the material for commercial purposes.
* Attribution — You must give appropriate credit, provide a link to the license, and indicate if changes were made.
  You may do so in any reasonable manner, but not in any way that suggests the licensor endorses you or your use.
* ShareAlike — If you remix, transform, or build upon the material, you must distribute your   contributions under the same license as the original.
* No additional restrictions — You may not apply legal terms or technological measures that legally restrict others from doing anything the license permits.
*/

#ifndef avrisp_program_H_
#define avrisp_program_H_

#define CONFIG_DISABLE_HAL_LOCKS 1
#include <Arduino.h>
#include <SPI.h>
#include "FS.h"

enum AVRISP_PROGRAMMER_RESULT : uint8_t
{
    SUCCESS = 0,
    WRONG_DEVICE_ID = 1,
    COMMIT_FAIL = 2,
    FAILED_ENTER_PROG_MODE = 3,
    FAIL_VERIFY = 4,
    FAIL_PROG_FUSE = 5,
    INCORRECT_PAGE_CONFIG = 6,
    OTHER_FAILURE=7
};

class AVRISP_PROGRAMMER
{
public:
    AVRISP_PROGRAMMER(SPIClass *spi, uint8_t resetGPIO, bool resetActiveHigh, gpio_num_t spiClockGPIO)
    {
        _spi = spi;
        _resetGPIO = resetGPIO;
        _resetActiveHigh = resetActiveHigh;
        _spiClockGPIO = spiClockGPIO;

        //May need a slower frequency for different devices
        //200khz
        _freq = 200000;
    }

    uint32_t device_signature;
    //Fuse Low
    uint8_t lfuse;
    //Fuse High
    uint8_t hfuse;
    //Fuse Extended
    uint8_t efuse;

    AVRISP_PROGRAMMER_RESULT ProgramAVRDevice(void (*avrprogrammer_progress_callback)(uint8_t,size_t, size_t),
                                              uint32_t deviceid,
                                              uint32_t byteLength,
                                              File &file,
                                              uint8_t fuse,
                                              uint8_t fusehigh,
                                              uint8_t fuseext);

private:
    SPIClass *_spi = NULL;

    gpio_num_t _spiClockGPIO;
    uint8_t _resetGPIO;
    bool _resetActiveHigh;
    uint32_t _freq;

    TickType_t WD_FLASH;
    TickType_t WD_ERASE;
    TickType_t WD_EEPROM;
    uint8_t PAGE_SIZE_WORDS = 0;
    uint16_t NUMBER_OF_PAGES = 0;

    void avr_reset_target(bool reset);
    uint8_t TransferByte4(uint8_t a, uint8_t b, uint8_t c, uint8_t d);
    uint8_t TransferByte3(uint8_t a, uint8_t b, uint8_t c, uint8_t d);
};

#endif
