/*
____  ____  _  _  ____  __  __  ___
(  _ \(_  _)( \/ )(  _ \(  \/  )/ __)
)(_) )_)(_  \  /  ) _ < )    ( \__ \
(____/(____) (__) (____/(_/\/\_)(___/

DIYBMS V4

V490 UP TO 16S CELL MONITORING MODULES

STM32F030K6T6

(c)2023 Stuart Pittaway

COMPILE THIS CODE USING PLATFORM.IO

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

#include "packet_processor.h"

// Run when a new packet is received over serial
bool PacketProcessor::onPacketReceived(PacketStruct *receivebuffer, uint8_t number_of_active_cells, CellData &cells)
{
  // packet counter to see where packets get lost
  PacketReceivedCounter++;

  // Calculate the CRC and compare to received
  uint16_t validateCRC = CRC16::CalculateArray((unsigned char *)receivebuffer, sizeof(PacketStruct) - 2);

  if (validateCRC != receivebuffer->crc)
  {
    // The packet received was not correct, failed CRC check
    badpackets++;
    return false;
  }

  // Loop through all the cells this module is "pretending" to be
  for (uint8_t cellindex = 0; cellindex < number_of_active_cells; cellindex++)
  {
    auto mymoduleaddress = receivebuffer->hops;

    bool isPacketForMe = receivebuffer->start_address <= mymoduleaddress && receivebuffer->end_address >= mymoduleaddress;

    // Increment the hops no matter what (on valid CRC)
    receivebuffer->hops++;

    // It's a good packet
    if (isPacketForMe && processPacket(receivebuffer, mymoduleaddress, cells.at(cellindex)))
    {
      // Set flag to indicate we processed packet (other modules may also do this)
      receivebuffer->command = receivebuffer->command | B10000000;
    }
  } // end for

  // Calculate new checksum over whole buffer (as hops have been increased)
  receivebuffer->crc = CRC16::CalculateArray((unsigned char *)receivebuffer, sizeof(PacketStruct) - 2);

  // Return TRUE if one of the "modules" we are pretending to be processed the command
  return ((receivebuffer->command & B10000000) > 0);
}

// Process the request in the received packet
// command byte
// RRRR CCCC
// X    = 1 bit indicate if packet processed
// R    = 3 bits reserved not used
// C    = 4 bits command (16 possible commands)
bool PacketProcessor::processPacket(PacketStruct *buffer, uint8_t mymoduleaddress, Cell &cell)
{
  uint8_t moduledata_index = mymoduleaddress % maximum_cell_modules;
  switch (buffer->command & 0x0F)
  {
  case COMMAND::ResetBadPacketCounter:
    badpackets = 0;
    PacketReceivedCounter = 0;
    return true;

  case COMMAND::ReadVoltageAndStatus:
  {
    // Read voltage of VCC

    // Maximum voltage 8191mV
    buffer->moduledata[moduledata_index] = cell.getCellVoltage() & 0x1FFF;

    // 3 top bits
    // X = In bypass
    // Y = Bypass over temperature
    // Z = Not used

    if (cell.BypassOverheatCheck())
    {
      // Set bit
      buffer->moduledata[moduledata_index] = buffer->moduledata[moduledata_index] | 0x4000;
    }

    if (cell.IsBypassActive())
    {
      // Set bit
      buffer->moduledata[moduledata_index] = buffer->moduledata[moduledata_index] | 0x8000;
    }

    return true;
  }

  case COMMAND::Timing:
  {
    // Do nothing just accept and pass on the packet
    return true;
  }

  case COMMAND::Identify:
  {
    // identify module
    // Ignored for 16S boards (meaningless)
    return true;
  }

  case COMMAND::ReadTemperature:
  {
    // Return the last known temperature values recorded - both internal and external.
    // output bytes are cast to a uint16_t for transport to host
    buffer->moduledata[moduledata_index] = cell.CombineTemperatures();
    return true;
  }

  case COMMAND::ReadBalancePowerPWM:
  {
    // Doesn't really mean anything on the all-in-one board, on or off...
    // If we are over temperature, then return zero
    buffer->moduledata[moduledata_index] = (Cell::getOverTemperature() == false && cell.IsBypassActive()) ? 255U : 0;
    return true;
  }

  case COMMAND::ReadBadPacketCounter:
  {
    // Report number of bad packets
    buffer->moduledata[moduledata_index] = badpackets;
    return true;
  }

  case COMMAND::ReadSettings:
  {
    // Report settings/configuration, fills whole moduledata buffer up

    FLOATUNION_t myFloat;
    myFloat.number = BalanceBoardInstalled ? (float)LOAD_RESISTANCE : 0.0F;
    buffer->moduledata[0] = myFloat.word[0];
    buffer->moduledata[1] = myFloat.word[1];

    myFloat.number = Cell::getCalibration();
    buffer->moduledata[2] = myFloat.word[0];
    buffer->moduledata[3] = myFloat.word[1];

    // Millivolts per ADC step (14 bit ADC in MAX chip)
    myFloat.number = ((float)DIYBMSREFMILLIVOLT) / 16384.0F;
    buffer->moduledata[4] = myFloat.word[0];
    buffer->moduledata[5] = myFloat.word[1];

    buffer->moduledata[6] = Cell::getBypassTemperatureSetPoint();

    // If changes are NOT allowed, set the high bit of moduledata[6], this allows the user interface to disable changes
    buffer->moduledata[6] |= cell.changesAllowed() ? 0 : 0x8000;

    buffer->moduledata[7] = Cell::getBypassThresholdmV();
    buffer->moduledata[8] = INT_BCOEFFICIENT;
    buffer->moduledata[9] = EXT_BCOEFFICIENT;
    buffer->moduledata[10] = DIYBMSMODULEVERSION;

    // Version of firmware (taken automatically from GIT)
    buffer->moduledata[14] = GIT_VERSION_B1;
    buffer->moduledata[15] = GIT_VERSION_B2;
    return true;
  }

  case COMMAND::WriteSettings:
  {
    if (cell.changesAllowed())
    {
      FLOATUNION_t myFloat;

      myFloat.word[0] = buffer->moduledata[0];
      myFloat.word[1] = buffer->moduledata[1];

      myFloat.word[0] = buffer->moduledata[2];
      myFloat.word[1] = buffer->moduledata[3];
      if (myFloat.number < 0xFFFF)
      {
        Cell::setCalibration(myFloat.number);
      }
      if (buffer->moduledata[6] != 0xFF)
      {
        Cell::setBypassTemperatureSetPoint((uint8_t)buffer->moduledata[6]);
      }
      if (buffer->moduledata[7] != 0xFFFF)
      {
        Cell::setBypassThresholdmV(buffer->moduledata[7]);
      }
      SettingsHaveChanged = true;
    }

    return true;
  }

  case COMMAND::ReadBalanceCurrentCounter:
  {
    buffer->moduledata[moduledata_index] = (uint16_t)cell.getMilliAmpHourBalanceCounter();
    return true;
  }

  case COMMAND::ReadPacketReceivedCounter:
  {
    buffer->moduledata[moduledata_index] = PacketReceivedCounter;
    return true;
  }

  case COMMAND::ResetBalanceCurrentCounter:
  {
    cell.setMilliAmpHourBalanceCounter(0);
    return true;
  }

  case COMMAND::ReadAdditionalSettings:
  {
    memset(buffer->moduledata, 0, sizeof(buffer->moduledata));
    buffer->moduledata[0] = Cell::getFanSwitchOnTemperature();
    buffer->moduledata[1] = Cell::getRelayMinmV();
    buffer->moduledata[2] = Cell::getRelayRange();
    buffer->moduledata[3] = 0;  //cell.getParasiteVoltage();
    buffer->moduledata[4] = getRunAwayCellMinimumVoltage();
    buffer->moduledata[5] = getRunAwayCellDifferential();
    return true;
  }

  case COMMAND::WriteAdditionalSettings:
  {
    Cell::setFanSwitchOnTemperature((uint8_t)buffer->moduledata[0]);
    Cell::setRelayMinmV(buffer->moduledata[1]);
    Cell::setRelayRange(buffer->moduledata[2]);
    // 3=ParasiteVoltage
    setRunAwayCellMinimumVoltage(buffer->moduledata[4]);
    setRunAwayCellDifferential(buffer->moduledata[5]);
    return true;
  }

  default:
    return false;
  }
}
