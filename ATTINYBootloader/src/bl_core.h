/*
 * DIYBMS ATtiny bootloader - hardware independent part
 *
 * Compiled for the ATtiny (main.c) and for the PC simulation (test/).
 * The hardware functions bl_hal_* are provided by the respective environment.
 */
#ifndef DIYBMS_BL_CORE_H
#define DIYBMS_BL_CORE_H

#include <stdint.h>
#include "fwupdate_protocol.h"

#define BL_PACKET_SIZE 40
#define BL_MD 6 /* byte offset of moduledata[0] in the packet */

/* Return value of bl_handle_packet() */
#define BL_ACT_VALID 0x01 /* valid packet (CRC ok)                       */
#define BL_ACT_MINE 0x02  /* packet was addressed to this module         */
#define BL_ACT_RESET 0x04 /* start the application after forwarding      */

typedef struct
{
    uint16_t next;  /* next expected block */
    uint16_t len;   /* image length        */
    uint32_t crc;   /* expected CRC32      */
    uint8_t state;  /* FW_BL_*                    */
    uint8_t page0[FW_PAGE_SIZE];
} bl_state_t;

/* ---- provided by the environment ---- */
uint8_t bl_hal_flash_read(uint16_t addr);
void bl_hal_flash_fill(uint16_t addr, const uint8_t *src, uint8_t len); /* fill page buffer */
void bl_hal_flash_erase_write(uint16_t addr);                            /* erase + write page */
void bl_hal_flash_erase(uint16_t addr);                                  /* erase page only */
void bl_hal_flash_bufclr(void);                                          /* clear page buffer */
uint16_t bl_hal_crc16(const uint8_t *p, uint8_t len);                     /* XMODEM */

/* ---- core ---- */

void bl_init(bl_state_t *s);
uint8_t bl_app_valid(void);
uint8_t bl_handle_packet(bl_state_t *s, uint8_t *p);

#endif
