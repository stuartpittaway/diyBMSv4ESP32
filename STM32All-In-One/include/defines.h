/*
LICENSE
Attribution-NonCommercial-ShareAlike 2.0 UK: England & Wales (CC BY-NC-SA 2.0 UK)
https://creativecommons.org/licenses/by-nc-sa/2.0/uk/

* Non-Commercial — You may not use the material for commercial purposes.
* Attribution — You must give appropriate credit, provide a link to the license, and indicate if changes were made.
  You may do so in any reasonable manner, but not in any way that suggests the licensor endorses you or your use.
* ShareAlike — If you remix, transform, or build upon the material, you must distribute your
  contributions under the same license as the original.
* No additional restrictions — You may not apply legal terms or technological measures
  that legally restrict others from doing anything the license permits.
*/

#include "EmbeddedFiles_Defines.h"
#include "cell.h"

#ifndef DIYBMS_DEFINES_H // include guard
#define DIYBMS_DEFINES_H

#if !defined(DIYBMSMODULEVERSION)
#error You need to specify the DIYBMSMODULEVERSION define
#endif

#if defined(DIYBMSMODULEVERSION) && DIYBMSMODULEVERSION != 490
#error Incorrect value for DIYBMSMODULEVERSION
#endif

// Only the lowest 4 bits can be used!
enum COMMAND : uint8_t
{
  ResetBadPacketCounter = 0,
  ReadVoltageAndStatus = 1,
  Identify = 2,
  ReadTemperature = 3,
  ReadBadPacketCounter = 4,
  ReadSettings = 5,
  WriteSettings = 6,
  ReadBalancePowerPWM = 7,
  Timing = 8,
  ReadBalanceCurrentCounter = 9,
  ReadPacketReceivedCounter = 10,
  ResetBalanceCurrentCounter = 11,
  ReadAdditionalSettings = 12,
  WriteAdditionalSettings = 13
};

// Default values
struct CellModuleConfig
{
  uint16_t magicnumber;
  uint16_t sizeofconfig;
  uint8_t BypassTemperatureSetPoint;
  uint16_t BypassThresholdmV;
  float Calibration;
  uint16_t relay_range;
  uint16_t relay_minimummv;
  uint8_t fanswitchontemperature;
  uint16_t RunAwayCellMinimumVoltage;
  uint16_t RunAwayCellDifferential;
} __attribute__((packed));

using CellData = std::array<Cell, 16>;

#endif
