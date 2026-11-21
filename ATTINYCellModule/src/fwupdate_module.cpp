/*
 * DIYBMS - firmware update over the RX/TX module chain, module firmware part
 * (see include/fwupdate_module.h and ../common/fwupdate_protocol.h)
 */

// Only for tinyAVR-2 (ATtiny1624/3224) and the PC simulation
#if !defined(FW_HOST_TEST)
#include <Arduino.h> // defines __AVR_ATtinyx24__ (megaTinyCore)
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

// Expected value of the BOOTEND fuse (bootloader size / 256)
#define FW_EXPECTED_BOOTEND (FW_APP_START / 256)

// CRC32 with a 16 entry table (faster than bitwise, 64 byte table)
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

static uint32_t own_crc = 0;

#if defined(FW_HOST_TEST)
uint16_t fw_hal_own_length();
#else
// End of the program in flash (end of .text + .rodata + .data initial values) from the linker script
extern "C" char __data_load_end[];
static inline uint16_t fw_hal_own_length()
{
  uint16_t end = (uint16_t)(uintptr_t)__data_load_end;
  return end > FW_APP_START ? (uint16_t)(end - FW_APP_START) : 0;
}
#endif

void FwUpdate_InitOwnCrc()
{
#if defined(FW_WITH_BOOTLOADER) || defined(FW_HOST_TEST)
  uint16_t len = fw_hal_own_length();
  own_crc = (len > 0 && len <= FW_APP_MAX) ? FwUpdate_AppCrc32(len) : 0;
#else
  own_crc = 0;
#endif
}

bool FwUpdate_HandleReport(PacketStruct *buffer, uint8_t moduleAddress)
{
  uint8_t rel = moduleAddress - buffer->start_address;
  if (rel >= FW_REPORT_MODULES_PER_PACKET)
    return false;
  buffer->moduledata[2 * rel] = (uint16_t)(own_crc & 0xFFFF);
  buffer->moduledata[2 * rel + 1] = (uint16_t)(own_crc >> 16);
  return true;
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

    // Version comparison: CRC32 of our own program (same length) against the announced one
    uint32_t announced = (uint32_t)buffer->moduledata[FW_MD_CRC_LO] | ((uint32_t)buffer->moduledata[FW_MD_CRC_HI] << 16);
    if (FwUpdate_AppCrc32(len) != announced)
    {
      buffer->moduledata[FW_MD_NEEDS_MASK] |= bit;
    }
#else
    // This firmware was built without bootloader -> no update over the chain possible
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
    // FW_START/FW_STATUS/FW_FINISH/FW_EXIT are only answered by the bootloader
    return false;
  }
}

#if defined(FW_WITH_BOOTLOADER) && !defined(FW_HOST_TEST)
void FwUpdate_EnterBootloader()
{
  cli();
  diyBMSHAL::DumpLoadOff();
  diyBMSHAL::NotificationLedOff();

  // Handover to the bootloader: magic at the start of SRAM, then immediate software reset

  *(volatile uint32_t *)RAMSTART = FW_BOOT_MAGIC;
  _PROTECTED_WRITE(RSTCTRL.SWRR, RSTCTRL_SWRE_bm);
  for (;;)
  {
  }
}
#endif

#endif // FW_HOST_TEST || __AVR_ATtinyx24__
