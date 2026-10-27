/*
 * DIYBMS - firmware update over the RX/TX module chain, module firmware part
 *
 * The application only handles two sub commands of FW_CMD_CONTROL:
 *   FW_ANNOUNCE   : version comparison. The module calculates the CRC32 over its own
 *                   program memory (length of the new image) and sets its bit in
 *                   md[FW_MD_NEEDS_MASK] if it differs from the announced one.
 *   FW_ENTER_BOOT : software reset into the bootloader (only AFTER the reply has been forwarded).
 * Everything else (receiving data, writing flash) is done by the bootloader.
 *
 * Only active when built with FW_WITH_BOOTLOADER (environments V450_BL_*). Without
 * bootloader the module answers FW_ANNOUNCE with "does not fit" (FW_MD_BADHW_MASK) and
 * can never be sent into the bootloader.

 */
#ifndef DIYBMS_FWUPDATE_MODULE_H
#define DIYBMS_FWUPDATE_MODULE_H

#include <stdint.h>
#include "fwupdate_protocol.h"

struct PacketStruct;

// Handles FW_CMD_CONTROL. Returns true = processed (set bit 7 in the command).
bool FwUpdate_HandleControl(PacketStruct *buffer, uint8_t moduleAddress, bool *enterBootloaderRequested);

// Jumps into the bootloader via software reset. Does not return.
void FwUpdate_EnterBootloader() __attribute__((noreturn));

// CRC32 over our own program memory from FW_APP_START
uint32_t FwUpdate_AppCrc32(uint16_t length);

// once at start up: calculate and store the CRC32 of our own program (full length)
void FwUpdate_InitOwnCrc();
// answer FW_CMD_REPORT (fill in the stored value), true = processed

bool FwUpdate_HandleReport(PacketStruct *buffer, uint8_t moduleAddress);

#endif
