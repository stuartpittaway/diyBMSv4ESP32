/*
 * DIYBMS - cell module firmware update over the RX/TX module chain
 * ----------------------------------------------------------------
 * Shared protocol definition for
 *   - ATtiny bootloader   (ATTINYBootloader/)
 *   - module firmware     (ATTINYCellModule/)
 *   - ESP32 controller    (ESPController/)
 *
 * The existing 40 byte packet format (PacketStruct) and the existing framing
 * (SerialEncoder, 0xAA start / 0x88 escape) are used unchanged. Modules with OLD
 * firmware therefore forward the update packets as well: they do not know commands
 * 12/13, but increment "hops" as usual and pass the packet on to the next module.
 *
 * ATtiny1624 memory layout (16 KB flash, 64 byte pages):
 *   0x0000 - 0x05FF  bootloader  (FUSE BOOTEND = 0x06, 1.5 KB)
 *   0x0600 - 0x3FFF  application (max. 14848 bytes)
 *
 * Sequence (controlled by the ESP32):
 *   1. FW_ANNOUNCE  to all  -> every module compares length + CRC32 of the new image
 *                              with its own flash and sets its bit in
 *                              md[FW_MD_NEEDS_MASK] if it differs.
 *   2. for every module k that differs, one after the other:
 *        FW_ENTER_BOOT (k)  -> application starts the bootloader via software reset
 *        FW_START      (k)  -> bootloader erases the first application page (app invalid)
 *        FW_DATA       (k)  -> 32 bytes per packet, sequence = block number
 *        FW_FINISH     (k)  -> check CRC32, write first page, start application
 *        FW_ANNOUNCE   (k)  -> verification: module reports "up to date"
 *   Packets for module k pass through modules 0..k-1 and then through k+1..n-1
 *   back to the ESP32 (ring).
 */

#ifndef DIYBMS_FWUPDATE_PROTOCOL_H
#define DIYBMS_FWUPDATE_PROTOCOL_H

#include <stdint.h>

/* Commands (only the lower 4 bits are available) */
#define FW_CMD_CONTROL 12
#define FW_CMD_DATA 13
/* Query the firmware checksum (normal application): every module in the address range
   fills in the CRC32 of its own program, 2 words per module (low, high), max. 8 modules
   per packet: moduledata[2*(address-start)] = low, [2*(address-start)+1] = high. 0 = no value. */
#define FW_CMD_REPORT 14
#define FW_REPORT_MODULES_PER_PACKET 8

/* For FW_CMD_CONTROL the sub command is in the low byte of moduledata[0] */
#define FW_ANNOUNCE 0x01   /* application + bootloader, broadcast or single  */
#define FW_ENTER_BOOT 0x02 /* application: start the bootloader              */
#define FW_START 0x10      /* bootloader: begin receiving                     */
#define FW_STATUS 0x11     /* bootloader: query state                         */
#define FW_FINISH 0x12     /* bootloader: verify, finish, start application   */
#define FW_EXIT 0x13       /* bootloader: abort, start old application        */

/* Request parameters */
#define FW_MD_SUB 0      /* sub command                                       */
#define FW_MD_LEN 1      /* image length in bytes                             */
#define FW_MD_CRC_LO 2   /* CRC32 (IEEE 802.3) of the image, lower 16 bits    */
#define FW_MD_CRC_HI 3   /* upper 16 bits                                     */
#define FW_MD_BOARD 4    /* DIYBMSMODULEVERSION, e.g. 450                     */
#define FW_MD_GIT_HI 5   /* information only: GIT_VERSION_B1                  */
#define FW_MD_GIT_LO 6   /* information only: GIT_VERSION_B2                  */

/* Bootloader replies (in all packets, including FW_CMD_DATA) */
#define FW_MD_BL_NEXT 8    /* next expected block                             */
#define FW_MD_BL_STATUS 9  /* FW_BL_SIGNATURE | state                         */
#define FW_MD_BL_VERSION 10

/* Replies to FW_ANNOUNCE (one bit per module: 1 << (module address & 15)) */
#define FW_MD_SEEN_MASK 12   /* module has processed FW_ANNOUNCE              */
#define FW_MD_INBOOT_MASK 13 /* module is in the bootloader                   */
#define FW_MD_NEEDS_MASK 14  /* module needs the new image                    */
#define FW_MD_BADHW_MASK 15  /* image does not fit this hardware / size       */

/* Application reply to FW_ENTER_BOOT */
#define FW_MD_APP_ACK 8
#define FW_APP_ACK_ENTERING 0xB007

#define FW_BL_SIGNATURE 0xB000
#define FW_BL_STATE_MASK 0x00FF
#define FW_BL_IDLE 0x00
#define FW_BL_RECEIVING 0x01
#define FW_BL_DONE 0x02
#define FW_BL_ERR_CRC 0x03
#define FW_BL_ERR_LEN 0x04

#define FW_BL_PROTOCOL_VERSION 1

#define FW_CHUNK_SIZE 32

/* ATtiny1624 */
#define FW_APP_START 0x0600u
#define FW_FLASH_END 0x4000u
#define FW_APP_MAX (FW_FLASH_END - FW_APP_START)
#define FW_PAGE_SIZE 64u

/* Handover application -> bootloader: 4 bytes at the start of SRAM + software reset */
#define FW_BOOT_MAGIC 0xB0071624UL

/* Interval between two update packets = packet time + this gap (ms).
   During the gap the bootloader writes a flash page (CPU halted for about 4 ms);
   the line has to be idle then, otherwise bytes are lost. */
#define FW_PACING_GAP_MS 16

/* CRC32 IEEE 802.3 (same as zlib/binascii.crc32), bitwise */
static inline uint32_t fw_crc32_update(uint32_t crc, uint8_t b)
{
    crc ^= b;
    for (uint8_t i = 0; i < 8; i++)
    {
        crc = (crc >> 1) ^ (0xEDB88320UL & (uint32_t)(-(int32_t)(crc & 1)));
    }
    return crc;
}

#endif
