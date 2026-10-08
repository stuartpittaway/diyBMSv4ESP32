/*
 * DIYBMS - Firmware-Update ueber die RX/TX-Kette, Teil der Modulfirmware
 * (siehe include/fwupdate_module.h und ../common/fwupdate_protocol.h)
 */

// Nur fuer tinyAVR-2 (ATtiny1624/3224) und die PC-Simulation
#if !defined(FW_HOST_TEST)
#include <Arduino.h> // definiert __AVR_ATtinyx24__ (megaTinyCore)
#endif
#if defined(FW_HOST_TEST) || defined(__AVR_ATtinyx24__)

#if defined(FW_HOST_TEST)
#include <stdint.h>
#include <string.h>
#include "packet_struct_host.h"
uint8_t fw_hal_flash_read(uint16_t addr);
uint8_t fw_hal_bootend();
uint16_t fw_hal_board_version();
#define DIYBMSMODULEVERSION fw_hal_board_version()
#else
#include <Arduino.h>
#include "packet_processor.h"
#include "diybms_tinyAVR2.h"
#endif

#include "fwupdate_module.h"

#if !defined(FW_HOST_TEST)
static inline uint8_t fw_hal_flash_read(uint16_t addr)
{
#if defined(MAPPED_PROGMEM_START)
  return *(const volatile uint8_t *)(MAPPED_PROGMEM_START + addr);
#else
  return pgm_read_byte(addr);
#endif
}
static inline uint8_t fw_hal_bootend()
{
#if defined(FUSE_BOOTEND)
  return FUSE_BOOTEND;
#else
  return 0;
#endif
}
#endif

// Erwarteter Wert der BOOTEND-Fuse (Bootloadergroesse / 256)
#define FW_EXPECTED_BOOTEND (FW_APP_START / 256)

// CRC32 mit 16er-Tabelle (schneller als bitweise, 64 Byte Tabelle)
static const uint32_t crc_nibble[16]
#if !defined(FW_HOST_TEST)
    PROGMEM
#endif
    = {
        0x00000000UL, 0x1DB71064UL, 0x3B6E20C8UL, 0x26D930ACUL,
        0x76DC4190UL, 0x6B6B51F4UL, 0x4DB26158UL, 0x5005713CUL,
        0xEDB88320UL, 0xF00F9344UL, 0xD6D6A3E8UL, 0xCB61B38CUL,
        0x9B64C2B0UL, 0x86D3D2D4UL, 0xA00AE278UL, 0xBDBDF21CUL};

static inline uint32_t nibble(uint8_t i)
{
#if defined(FW_HOST_TEST)
  return crc_nibble[i];
#else
  return pgm_read_dword(&crc_nibble[i]);
#endif
}

uint32_t FwUpdate_AppCrc32(uint16_t length)
{
  uint32_t crc = 0xFFFFFFFFUL;
  for (uint16_t i = 0; i < length; i++)
  {
    crc ^= fw_hal_flash_read(FW_APP_START + i);
    crc = (crc >> 4) ^ nibble(crc & 0x0F);
    crc = (crc >> 4) ^ nibble(crc & 0x0F);
#if !defined(FW_HOST_TEST)
    if ((i & 0x3FF) == 0)
    {
      wdt_reset();
    }
#endif
  }
  return ~crc;
}

bool FwUpdate_HandleControl(PacketStruct *buffer, uint8_t moduleAddress, bool *enterBootloaderRequested)
{
  uint16_t bit = (uint16_t)1 << (moduleAddress & 0x0F);

  switch (buffer->moduledata[FW_MD_SUB] & 0xFF)
  {
  case FW_ANNOUNCE:
  {
    buffer->moduledata[FW_MD_SEEN_MASK] |= bit;
#if defined(FW_WITH_BOOTLOADER) || defined(FW_HOST_TEST)
    uint16_t len = buffer->moduledata[FW_MD_LEN];
    bool hardware_ok = (buffer->moduledata[FW_MD_BOARD] == DIYBMSMODULEVERSION) && (len > 0) && (len <= FW_APP_MAX) &&
                       (fw_hal_bootend() == FW_EXPECTED_BOOTEND);
    if (!hardware_ok)
    {
      buffer->moduledata[FW_MD_BADHW_MASK] |= bit;
      return true;
    }

    // Versionsvergleich: CRC32 des eigenen Programms (gleiche Laenge) gegen die angekuendigte
    uint32_t announced = (uint32_t)buffer->moduledata[FW_MD_CRC_LO] | ((uint32_t)buffer->moduledata[FW_MD_CRC_HI] << 16);
    if (FwUpdate_AppCrc32(len) != announced)
    {
      buffer->moduledata[FW_MD_NEEDS_MASK] |= bit;
    }
#else
    // Diese Firmware wurde ohne Bootloader gebaut -> kein Update ueber die Kette moeglich
    buffer->moduledata[FW_MD_BADHW_MASK] |= bit;
#endif
    return true;
  }

  case FW_ENTER_BOOT:
  {
#if defined(FW_WITH_BOOTLOADER) || defined(FW_HOST_TEST)
    buffer->moduledata[FW_MD_APP_ACK] = FW_APP_ACK_ENTERING;
    *enterBootloaderRequested = true;
    return true;
#else
    return false;
#endif
  }

  default:
    // FW_START/FW_STATUS/FW_FINISH/FW_EXIT beantwortet nur der Bootloader
    return false;
  }
}

#if defined(FW_WITH_BOOTLOADER) && !defined(FW_HOST_TEST)
void FwUpdate_EnterBootloader()
{
  cli();
  diyBMSHAL::DumpLoadOff();
  diyBMSHAL::NotificationLedOff();

  // Uebergabe an den Bootloader: Magic am SRAM-Anfang, danach sofort Software-Reset
  *(volatile uint32_t *)RAMSTART = FW_BOOT_MAGIC;
  _PROTECTED_WRITE(RSTCTRL.SWRR, RSTCTRL_SWRE_bm);
  for (;;)
  {
  }
}
#endif

#endif // FW_HOST_TEST || __AVR_ATtinyx24__
