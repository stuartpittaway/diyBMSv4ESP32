/*
 * DIYBMS ATtiny-Bootloader - hardwareunabhaengiger Teil
 *
 * Wird fuer den ATtiny (main.c) und fuer die PC-Simulation (test/) uebersetzt.
 * Die Hardwarefunktionen bl_hal_* stellt die jeweilige Umgebung bereit.
 */
#ifndef DIYBMS_BL_CORE_H
#define DIYBMS_BL_CORE_H

#include <stdint.h>
#include "fwupdate_protocol.h"

#define BL_PACKET_SIZE 40
#define BL_MD 6 /* Byte-Offset von moduledata[0] im Paket */

/* Rueckgabe von bl_handle_packet() */
#define BL_ACT_VALID 0x01 /* gueltiges Paket (CRC ok)               */
#define BL_ACT_MINE 0x02  /* Paket war an dieses Modul adressiert   */
#define BL_ACT_RESET 0x04 /* nach dem Weiterleiten Anwendung starten */

typedef struct
{
    uint16_t next;  /* naechster erwarteter Block */
    uint16_t len;   /* Image-Laenge               */
    uint32_t crc;   /* erwartete CRC32            */
    uint8_t state;  /* FW_BL_*                    */
    uint8_t page0[FW_PAGE_SIZE];
} bl_state_t;

/* ---- von der Umgebung bereitzustellen ---- */
uint8_t bl_hal_flash_read(uint16_t addr);
void bl_hal_flash_fill(uint16_t addr, const uint8_t *src, uint8_t len); /* Seitenpuffer fuellen */
void bl_hal_flash_erase_write(uint16_t addr);                            /* Seite loeschen+schreiben */
void bl_hal_flash_erase(uint16_t addr);                                  /* Seite nur loeschen */
void bl_hal_flash_bufclr(void);                                          /* Seitenpuffer loeschen */
uint16_t bl_hal_crc16(const uint8_t *p, uint8_t len);                     /* XMODEM */

/* ---- Kern ---- */
void bl_init(bl_state_t *s);
uint8_t bl_app_valid(void);
uint8_t bl_handle_packet(bl_state_t *s, uint8_t *p);

#endif
