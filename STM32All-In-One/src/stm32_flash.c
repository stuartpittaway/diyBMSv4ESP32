/*
This is a clone of the stm32_eeprom.c library with modifications to remove the internal RAM buffer
and specifically targets 1K page sizes and the STM32F030K6T6 chip.
The top 1K of FLASH is used for this library - so ensure compiled code doesn't exceed 31744 bytes (adjust board config)

Stuart Pittaway
*/
/**
 ******************************************************************************
 * @file    stm32_flash.c
 * @brief
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2016-2021, STMicroelectronics
 * All rights reserved.
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
#include "stm32_flash.h"
#include <string.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* Be able to change FLASH_PAGE_NUMBER to use if relevant */
#if !defined(FLASH_PAGE_NUMBER) && defined(FLASH_PAGE_SIZE)
#define FLASH_PAGE_NUMBER ((uint32_t)(((LL_GetFlashSize() * 1024) / FLASH_PAGE_SIZE) - 1))
#endif /* !FLASH_PAGE_NUMBER */

#define FLASH_END FLASH_BANK1_END
#define FLASH_BASE_ADDRESS ((uint32_t)((FLASH_END + 1) - FLASH_PAGE_SIZE))
#define E2END (FLASH_PAGE_SIZE - 1)

#define FLASH_FLAG_ALL_ERRORS (FLASH_FLAG_WRPERR | FLASH_FLAG_PGERR)

    /**
     * @brief  This function writes the buffer content into the flash
     * @param  none
     * @retval none
     */
    void flash_erase_and_write(uint8_t *buffer, uint16_t buffer_size)
    {
        FLASH_EraseInitTypeDef EraseInitStruct;
        uint32_t offset = 0;
        uint32_t address = FLASH_BASE_ADDRESS;
        uint32_t address_end = FLASH_BASE_ADDRESS + E2END;
        uint32_t pageError = 0;
        uint64_t data = 0;

        /* ERASING page */
        EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
        EraseInitStruct.PageAddress = FLASH_BASE_ADDRESS;
        EraseInitStruct.NbPages = 1;

        if (HAL_FLASH_Unlock() == HAL_OK)
        {
            __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
            if (HAL_FLASHEx_Erase(&EraseInitStruct, &pageError) == HAL_OK)
            {

                while (address <= address_end)
                {
                    data = *((uint64_t *)(buffer + offset));

                    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, data) == HAL_OK)
                    {
                        address += 8;
                        offset += 8;
                    }
                    else
                    {
                        address = address_end + 1;
                    }

                    if (offset > buffer_size)
                    {
                        break;
                    }
                }
            }
            HAL_FLASH_Lock();
        }
    }

    /**
     * @brief  This function copies the data from flash into the RAM buffer
     * @param  none
     * @retval none
     */
    void flash_copy_buffer(uint8_t *buffer, uint16_t numberofbytes)
    {
        if (numberofbytes > E2END + 1)
        {
            numberofbytes = E2END + 1;
        }
        memcpy(buffer, (uint8_t *)(FLASH_BASE_ADDRESS), numberofbytes);
    }

#ifdef __cplusplus
}
#endif
