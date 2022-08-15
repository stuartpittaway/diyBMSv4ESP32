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

#define USE_ESP_IDF_LOG 1
static constexpr const char * const TAG = "diybms-avrisp";

#include "avrisp_programmer.h"
#include <esp_task_wdt.h>

void AVRISP_PROGRAMMER::avr_reset_target(bool reset)
{
    digitalWrite(_resetGPIO, ((reset && _resetActiveHigh) || (!reset && !_resetActiveHigh)) ? HIGH : LOW);
}

//Returns the 4th byte of the SPI transfer
uint8_t AVRISP_PROGRAMMER::TransferByte4(uint8_t a, uint8_t b, uint8_t c, uint8_t d)
{
    _spi->transfer(a);
    _spi->transfer(b);
    _spi->transfer(c);
    return _spi->transfer(d);
}

//Returns the result of the 3rd byte (used for checking programming mode)
uint8_t AVRISP_PROGRAMMER::TransferByte3(uint8_t a, uint8_t b, uint8_t c, uint8_t d)
{
    _spi->transfer(a);
    _spi->transfer(b);
    uint8_t replyc = _spi->transfer(c);
    _spi->transfer(d);
    return replyc;
}

AVRISP_PROGRAMMER_RESULT AVRISP_PROGRAMMER::ProgramAVRDevice(void (*avrprogrammer_progress_callback)(uint8_t, size_t, size_t),
                                                             uint32_t deviceid,
                                                             uint32_t byteLength,
                                                             File &file,
                                                             uint8_t fuse,
                                                             uint8_t fusehigh,
                                                             uint8_t fuseext)

{

    WD_FLASH = pdMS_TO_TICKS(25);
    WD_ERASE = pdMS_TO_TICKS(25);
    WD_EEPROM = pdMS_TO_TICKS(25);

    avrprogrammer_progress_callback(0, 0, byteLength);

    //ATTINY841
    if (deviceid == 0x1e9315)
    {
        //Data for ATTINY841
        //Page Size = 8 words (16 bytes), Flash = 4K words (8kbytes), 512 Pages
        //Wait delays for programming, WD_FLASH=4.5ms, WD_EEROM=3.6ms, WD_ERASE=9.0m

        //Set the timing values for the ATTINY841 (plus extra milliseconds for good luck)
        WD_FLASH = pdMS_TO_TICKS(9);
        WD_ERASE = pdMS_TO_TICKS(12);
        WD_EEPROM = pdMS_TO_TICKS(6);
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

        WD_FLASH = pdMS_TO_TICKS(5);
        WD_ERASE = pdMS_TO_TICKS(10);
        WD_EEPROM = pdMS_TO_TICKS(4);

        PAGE_SIZE_WORDS = 64;
        NUMBER_OF_PAGES = 256;
    }

    ESP_LOGI(TAG, "AVR setting e=%02X h=%02X l=%02X mcu=%08X len=%u", fuseext, fusehigh, fuse, deviceid, byteLength);

    _spi->setFrequency(_freq);
    _spi->beginTransaction(SPISettings(_freq, MSBFIRST, SPI_MODE0));

    avr_reset_target(true);

    digitalWrite(_spiClockGPIO, LOW);
    //delay(20); // discharge PIN_SCK, value arbitrarily chosen
    vTaskDelay(pdMS_TO_TICKS(20));
    avr_reset_target(false);
    // Pulse must be minimum 2 target CPU clock cycles so 100 usec is ok for CPU speeds above 20 KHz
    //delayMicroseconds(100);
    vTaskDelay(pdMS_TO_TICKS(0.1));
    avr_reset_target(true);

    // Send the enable programming command:
    // datasheet: must be > 20 msec
    vTaskDelay(pdMS_TO_TICKS(30));
    uint8_t programmingreply = TransferByte3(0xAC, 0x53, 0x00, 0x00);

    if (programmingreply != 0x53)
    {
        return AVRISP_PROGRAMMER_RESULT::FAILED_ENTER_PROG_MODE;
    }

    device_signature = (TransferByte4(0x30, 0x00, 0x00, 0x00) << 16) + (TransferByte4(0x30, 0x00, 0x01, 0x00) << 8) + TransferByte4(0x30, 0x00, 0x02, 0x00);

    ESP_LOGI(TAG, "Found device %08X", device_signature);

    if (device_signature != deviceid)
    {
        return AVRISP_PROGRAMMER_RESULT::WRONG_DEVICE_ID;
    }

    if (PAGE_SIZE_WORDS * NUMBER_OF_PAGES == 0)
    {
        return AVRISP_PROGRAMMER_RESULT::INCORRECT_PAGE_CONFIG;
    }

    //Chip ERASE....
    ESP_LOGI(TAG, "Chip erase");
    TransferByte4(0xAC, 0x80, 0, 0);
    vTaskDelay(WD_ERASE);

    //Set Address = 0
    //HERE is in WORDS (16 bit)
    uint16_t page = 0;
    uint16_t byteCounter = 0;

    //Start at first byte in file
    file.seek(0);

    //Buffer to hold stream data
    char buffer[2];

    //Program page by page
    ESP_LOGI(TAG, "Chip programming");
    while (byteCounter < byteLength)
    {
        //Feed the watchdog a bone every page to avoid WDT errors
        //esp_task_wdt_reset();

        uint8_t lowbyte;
        uint8_t highbyte;

        //Count up in WORDS
        for (size_t i = 0; i < PAGE_SIZE_WORDS; i++)
        {
            if (byteCounter < byteLength)
            {
                //Todo we should check this returns 2 bytes...
                file.readBytes(buffer, 2);
                lowbyte = buffer[0];
                highbyte = buffer[1];
            }
            else
            {
                //Prevent buffer overrun... Fill remaining page bytes with 0xFF
                lowbyte = 0xFF;
                highbyte = 0xFF;
            }
            byteCounter = byteCounter + 2;

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

        page += PAGE_SIZE_WORDS;

        //Delay (as per data sheet) to allow FLASH to program....
        vTaskDelay(WD_FLASH);

        avrprogrammer_progress_callback(0, byteCounter, byteLength);
    }

    //Now verify the program was written correctly...
    //Start at first byte in file
    file.seek(0);

    ESP_LOGI(TAG, "Chip verify");
    byteCounter = 0;
    avrprogrammer_progress_callback(1, 0, byteLength);
    for (size_t wordPtr = 0; wordPtr < NUMBER_OF_PAGES * PAGE_SIZE_WORDS; wordPtr++)
    {
        uint8_t MSB = (wordPtr >> 8) & 0xFF;
        uint8_t LSB = wordPtr & 0x00FF;
        //Read Program Memory, Low byte
        uint8_t lowbyte = TransferByte4(0x20, MSB, LSB, 0);

        file.readBytes(buffer, 2);

        if (byteCounter < byteLength && buffer[0] != lowbyte)
        {
            ESP_LOGD(TAG, "L verify fail %u", byteCounter);
            return AVRISP_PROGRAMMER_RESULT::FAIL_VERIFY;
        }

        byteCounter++;

        //Read Program Memory, High byte
        uint8_t highbyte = TransferByte4(0x28, MSB, LSB, 0);
        if (byteCounter < byteLength && buffer[1] != highbyte)
        {
            ESP_LOGD(TAG, "H verify fail %u", byteCounter);
            return AVRISP_PROGRAMMER_RESULT::FAIL_VERIFY;
        }

        byteCounter++;

        if (byteCounter >= byteLength)
            break;

        if (byteCounter % PAGE_SIZE_WORDS == 0)
        {
            avrprogrammer_progress_callback(1, byteCounter, byteLength);
        }
    }

    //Read Fuse bytes
    lfuse = TransferByte4(0x50, 0x00, 0x00, 0x00);
    //Read Fuse High
    hfuse = TransferByte4(0x58, 0x08, 0x00, 0x00);
    //Read Fuse Extended
    efuse = TransferByte4(0x50, 0x08, 0x00, 0x00);

    //Program fuse bytes (if needed)
    if (fuse != lfuse)
    {
        ESP_LOGI(TAG, "Prog lfuse");
        TransferByte4(0xAC, 0xA0, 0x00, fuse);
        vTaskDelay(WD_FLASH);
    }
    if (fusehigh != hfuse)
    {
        ESP_LOGI(TAG, "Prog hfuse");
        TransferByte4(0xAC, 0xA8, 0x00, fusehigh);
        vTaskDelay(WD_FLASH);
    }
    if (fuseext != efuse)
    {
        ESP_LOGI(TAG, "Prog efuse");
        TransferByte4(0xAC, 0xA4, 0x00, fuseext);
        vTaskDelay(WD_FLASH);
    }

    //Read Fuse bits to confirm correct programming
    lfuse = TransferByte4(0x50, 0x00, 0x00, 0x00);
    hfuse = TransferByte4(0x58, 0x08, 0x00, 0x00);
    efuse = TransferByte4(0x50, 0x08, 0x00, 0x00);

    if (lfuse != fuse || hfuse != fusehigh || efuse != fuseext)
    {
        return AVRISP_PROGRAMMER_RESULT::FAIL_PROG_FUSE;
    }

    //Exit programming mode

    _spi->endTransaction();

    avr_reset_target(false);

    ESP_LOGI(TAG, "Complete");
    return AVRISP_PROGRAMMER_RESULT::SUCCESS;
}
