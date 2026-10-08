/*
 * DIYBMS - Firmware-Update der Zellmodule ueber die RX/TX-Kette
 * ------------------------------------------------------------
 * Gemeinsame Protokolldefinition fuer
 *   - ATtiny-Bootloader   (ATTINYBootloader/)
 *   - Modul-Firmware      (ATTINYCellModule/)
 *   - ESP32-Controller    (ESPController/)
 *
 * Es wird das vorhandene 40-Byte-Paketformat (PacketStruct) und die vorhandene
 * Rahmung (SerialEncoder, 0xAA Start / 0x88 Escape) unveraendert benutzt.
 * Dadurch leiten auch Module mit ALTER Firmware die Update-Pakete weiter:
 * sie kennen die Kommandos 12/13 nicht, erhoehen aber wie immer "hops" und
 * schicken das Paket zum naechsten Modul.
 *
 * Speicheraufteilung ATtiny1624 (16 KB Flash, 64-Byte-Seiten):
 *   0x0000 - 0x05FF  Bootloader (FUSE BOOTEND = 0x06, 1,5 KB)
 *   0x0600 - 0x3FFF  Anwendung  (max. 14848 Byte)
 *
 * Ablauf (vom ESP32 gesteuert):
 *   1. FW_ANNOUNCE  an alle  -> jedes Modul vergleicht Laenge+CRC32 des neuen
 *                              Images mit seinem eigenen Flash und setzt sein Bit
 *                              in md[FW_MD_NEEDS_MASK], wenn es abweicht.
 *   2. fuer jedes abweichende Modul k nacheinander:
 *        FW_ENTER_BOOT (k)  -> Anwendung startet per Software-Reset den Bootloader
 *        FW_START      (k)  -> Bootloader loescht die erste App-Seite (App ungueltig)
 *        FW_DATA       (k)  -> 32 Byte je Paket, sequence = Blocknummer
 *        FW_FINISH     (k)  -> CRC32 pruefen, erste Seite schreiben, App starten
 *        FW_ANNOUNCE   (k)  -> Kontrolle: Modul meldet "aktuell"
 *   Pakete an Modul k laufen durch die Module 0..k-1 und von dort weiter durch
 *   k+1..n-1 zurueck zum ESP32 (Ring).
 */

#ifndef DIYBMS_FWUPDATE_PROTOCOL_H
#define DIYBMS_FWUPDATE_PROTOCOL_H

#include <stdint.h>

/* Kommandos (nur die unteren 4 Bit stehen zur Verfuegung) */
#define FW_CMD_CONTROL 12
#define FW_CMD_DATA 13

/* Unterkommando steht bei FW_CMD_CONTROL im unteren Byte von moduledata[0] */
#define FW_ANNOUNCE 0x01   /* App + Bootloader, Broadcast oder einzeln      */
#define FW_ENTER_BOOT 0x02 /* App: Bootloader starten                       */
#define FW_START 0x10      /* Bootloader: Empfang beginnen                  */
#define FW_STATUS 0x11     /* Bootloader: Status abfragen                   */
#define FW_FINISH 0x12     /* Bootloader: pruefen, abschliessen, App starten */
#define FW_EXIT 0x13       /* Bootloader: abbrechen, alte App starten       */

/* Parameter in der Anfrage */
#define FW_MD_SUB 0      /* Unterkommando                                    */
#define FW_MD_LEN 1      /* Image-Laenge in Byte                             */
#define FW_MD_CRC_LO 2   /* CRC32 (IEEE 802.3) des Images, untere 16 Bit     */
#define FW_MD_CRC_HI 3   /* obere 16 Bit                                     */
#define FW_MD_BOARD 4    /* DIYBMSMODULEVERSION, z.B. 450                    */
#define FW_MD_GIT_HI 5   /* nur Information: GIT_VERSION_B1                  */
#define FW_MD_GIT_LO 6   /* nur Information: GIT_VERSION_B2                  */

/* Antworten des Bootloaders (in allen Paketen, auch FW_CMD_DATA) */
#define FW_MD_BL_NEXT 8    /* naechster erwarteter Block                     */
#define FW_MD_BL_STATUS 9  /* FW_BL_SIGNATURE | Zustand                      */
#define FW_MD_BL_VERSION 10

/* Antworten auf FW_ANNOUNCE (je Modul ein Bit: 1 << (Moduladresse & 15)) */
#define FW_MD_SEEN_MASK 12   /* Modul hat FW_ANNOUNCE ausgewertet          */
#define FW_MD_INBOOT_MASK 13 /* Modul steht im Bootloader                    */
#define FW_MD_NEEDS_MASK 14  /* Modul braucht das neue Image                 */
#define FW_MD_BADHW_MASK 15  /* Image passt nicht zu dieser Hardware/Groesse */

/* Antwort der Anwendung auf FW_ENTER_BOOT */
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

/* Uebergabe Anwendung -> Bootloader: 4 Byte am Anfang des SRAM + Software-Reset */
#define FW_BOOT_MAGIC 0xB0071624UL

/* Sicherheitsabstand zwischen zwei Paketen: Paketlaenge (mit Rahmung max. ~44 Byte)
   mal diesem Faktor. Der Bootloader schreibt Flash direkt nach einem Paket
   (CPU steht dann ca. 4 ms), danach muss die Leitung frei sein. */
#define FW_PACING_FACTOR_X10 25

/* CRC32 IEEE 802.3 (wie zlib/binascii.crc32), bitweise */
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
