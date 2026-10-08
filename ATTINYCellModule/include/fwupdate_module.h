/*
 * DIYBMS - Firmware-Update ueber die RX/TX-Kette, Teil der Modulfirmware
 *
 * Die Anwendung kennt nur zwei Unterkommandos von FW_CMD_CONTROL:
 *   FW_ANNOUNCE   : Versionsvergleich. Das Modul berechnet die CRC32 ueber seinen
 *                   eigenen Programmspeicher (Laenge des neuen Images) und setzt sein
 *                   Bit in md[FW_MD_NEEDS_MASK], wenn sie von der angekuendigten abweicht.
 *   FW_ENTER_BOOT : Software-Reset in den Bootloader (erst NACH dem Weiterleiten der Antwort).
 * Alles Weitere (Daten empfangen, Flash schreiben) macht der Bootloader.
 *
 * Nur aktiv, wenn mit FW_WITH_BOOTLOADER gebaut (Umgebungen V450_BL_*). Ohne Bootloader
 * meldet das Modul bei FW_ANNOUNCE "passt nicht" (FW_MD_BADHW_MASK) und laesst sich nie
 * in den Bootloader schicken.
 */
#ifndef DIYBMS_FWUPDATE_MODULE_H
#define DIYBMS_FWUPDATE_MODULE_H

#include <stdint.h>
#include "fwupdate_protocol.h"

struct PacketStruct;

// Wertet FW_CMD_CONTROL aus. Rueckgabe true = verarbeitet (Bit 7 im Kommando setzen).
bool FwUpdate_HandleControl(PacketStruct *buffer, uint8_t moduleAddress, bool *enterBootloaderRequested);

// Springt per Software-Reset in den Bootloader. Kehrt nicht zurueck.
void FwUpdate_EnterBootloader() __attribute__((noreturn));

// CRC32 ueber den eigenen Programmspeicher ab FW_APP_START
uint32_t FwUpdate_AppCrc32(uint16_t length);

#endif
