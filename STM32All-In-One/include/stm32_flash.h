
#ifndef __STM32_FLASH_H
#define __STM32_FLASH_H

/* Includes ------------------------------------------------------------------*/
#include "stm32_def.h"


void flash_erase_and_write(uint8_t* buffer, uint16_t buffer_size);
void flash_copy_buffer(uint8_t* buffer, uint16_t numberofbytes);

#endif /* __STM32_FLASH_H */
