#ifndef DIYBMS_PACKETPROCESSOR_H // include guard
#define DIYBMS_PACKETPROCESSOR_H

#include <Arduino.h>

#if (!defined(DIYBMSMODULEVERSION))
#error You need to specify the DIYBMSMODULEVERSION define
#endif

#include "defines.h"
#include "crc16.h"
#include "cell.h"

#define maximum_cell_modules 16

// NOTE THIS MUST BE EVEN IN SIZE (BYTES) ESP8266 IS 32 BIT AND WILL ALIGN AS SUCH!
struct PacketStruct
{
  uint8_t start_address;
  uint8_t end_address;
  uint8_t command;
  uint8_t hops;
  uint16_t sequence;
  uint16_t moduledata[maximum_cell_modules];
  uint16_t crc;
} __attribute__((packed));

typedef union
{
  float number;
  uint8_t bytes[4];
  uint16_t word[2];
} FLOATUNION_t;

class PacketProcessor
{
public:
  PacketProcessor() = default;
  ~PacketProcessor() = default;

  bool onPacketReceived(PacketStruct *receivebuffer, uint8_t number_of_active_cells, CellData &cells);
  uint16_t IncrementWatchdogCounter()
  {
    watchdog_counter++;
    return watchdog_counter;
  }
  uint16_t getPacketReceivedCounter() const { return PacketReceivedCounter; }

  auto getSettingsHaveChanged() const { return SettingsHaveChanged; }
  void clearSettingsHaveChanged() { SettingsHaveChanged = false; }

  auto getRunAwayCellMinimumVoltage() const { return RunAwayCellMinimumVoltage; }
  auto getRunAwayCellDifferential() const { return RunAwayCellDifferential; }

  void setRunAwayCellMinimumVoltage(uint16_t v) { RunAwayCellMinimumVoltage = v; }
  void setRunAwayCellDifferential(uint16_t v) { RunAwayCellDifferential = v; }

  /// @brief TRUE if daughter board is installed (only checked on boot)
  bool BalanceBoardInstalled = false;

private:
  bool processPacket(PacketStruct *buffer, uint8_t, Cell &cell);

  // Count of bad packets of data received, most likely with corrupt data or crc errors
  uint16_t badpackets{0};
  // Count of number of WDT events which have triggered, could indicate standalone mode or problems with serial comms
  uint16_t watchdog_counter{0};

  uint16_t PacketReceivedCounter{0};

  bool SettingsHaveChanged{false};

  uint16_t RunAwayCellMinimumVoltage;
  uint16_t RunAwayCellDifferential;
};

#endif
