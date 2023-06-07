#define USE_ESP_IDF_LOG 1
static constexpr const char *const TAG = "diybms-tx";

#include "PacketRequestGenerator.h"

void PacketRequestGenerator::clearSettingsForAllModules()
{
  // Force refresh of settings
  for (size_t i = 0; i < maximum_controller_cell_modules; i++)
  {
    cmi[i].settingsCached = false;
  }
}

bool PacketRequestGenerator::sendSaveGlobalSetting(uint16_t BypassThresholdmV, uint8_t BypassOverTempShutdown)
{
  PacketStruct _packetbuffer;
  clearPacket(&_packetbuffer);

  // Ask all modules to set bypass and temperature value
  setPacketAddressBroadcast(&_packetbuffer);
  // Command - WriteSettings
  _packetbuffer.command = COMMAND::WriteSettings;

  // Fill packet with 0xFFFF values - module ignores settings with this value
  setmoduledataFFFF(&_packetbuffer);
  _packetbuffer.moduledata[6] = BypassOverTempShutdown;
  _packetbuffer.moduledata[7] = BypassThresholdmV;
  if (pushPacketToQueue(&_packetbuffer))
  {
    clearSettingsForAllModules();

    return true;
  }

  return false;
}

bool PacketRequestGenerator::sendSaveAdditionalSetting(uint8_t m, int16_t FanSwitchOnT, uint16_t RelayMinV, uint16_t RelayRange, uint16_t RunAwayCellMinimumVoltagemV, uint16_t RunAwayCellDifferentialmV)
{
  PacketStruct _packetbuffer;
  clearPacket(&_packetbuffer);
  setPacketAddressModuleRange(&_packetbuffer, m, m);
  // Command - WriteSettings
  _packetbuffer.command = COMMAND::WriteAdditionalSettings;

  // Fill packet with 0xFFFF values - module ignores settings with this value
  setmoduledataFFFF(&_packetbuffer);

  // Force refresh of settings
  cmi[m].settingsCached = false;

  _packetbuffer.moduledata[0] = (uint16_t)FanSwitchOnT;
  _packetbuffer.moduledata[1] = RelayMinV;
  _packetbuffer.moduledata[2] = RelayRange;
  _packetbuffer.moduledata[3] = 0; //[3] = Parasite voltage (not editable)
  _packetbuffer.moduledata[4] = RunAwayCellMinimumVoltagemV;
  _packetbuffer.moduledata[5] = RunAwayCellDifferentialmV;

  return pushPacketToQueue(&_packetbuffer);
}
bool PacketRequestGenerator::sendSaveSetting(uint8_t m, uint16_t BypassThresholdmV, uint8_t BypassOverTempShutdown, float Calibration)
{
  PacketStruct _packetbuffer;
  clearPacket(&_packetbuffer);
  setPacketAddressModuleRange(&_packetbuffer, m, m);
  // Command - WriteSettings
  _packetbuffer.command = COMMAND::WriteSettings;

  // Fill packet with 0xFFFF values - module ignores settings with this value
  setmoduledataFFFF(&_packetbuffer);

  // Force refresh of settings
  cmi[m].settingsCached = false;

  FLOATUNION_t myFloat;

  // myFloat.number = LoadResistance;
  //_packetbuffer.moduledata[0] = myFloat.word[0];
  //_packetbuffer.moduledata[1] = myFloat.word[1];

  // Arduino float(4 byte)
  myFloat.number = Calibration;
  _packetbuffer.moduledata[2] = myFloat.word[0];
  _packetbuffer.moduledata[3] = myFloat.word[1];

  // Arduino float(4 byte)
  // myFloat.number = mVPerADC;
  //_packetbuffer.moduledata[4] = myFloat.word[0];
  //_packetbuffer.moduledata[5] = myFloat.word[1];

  _packetbuffer.moduledata[6] = BypassOverTempShutdown;
  _packetbuffer.moduledata[7] = BypassThresholdmV;
  //_packetbuffer.moduledata[8] = Internal_BCoefficient;
  //_packetbuffer.moduledata[9] = External_BCoefficient;
  return pushPacketToQueue(&_packetbuffer);
}

bool PacketRequestGenerator::sendReadBadPacketCounter(uint8_t startmodule, uint8_t endmodule)
{
  return BuildAndSendRequest(COMMAND::ReadBadPacketCounter, startmodule, endmodule);
}

bool PacketRequestGenerator::sendCellVoltageRequest(uint8_t startmodule, uint8_t endmodule)
{
  return BuildAndSendRequest(COMMAND::ReadVoltageAndStatus, startmodule, endmodule);
}

bool PacketRequestGenerator::sendIdentifyModuleRequest(uint8_t cellid)
{
  return BuildAndSendRequest(COMMAND::Identify, cellid, cellid);
}

bool PacketRequestGenerator::sendTimingRequest()
{
  // Ask all modules to simple pass on a NULL request/packet for timing purposes
  return BuildAndSendRequest(COMMAND::Timing);
}
bool PacketRequestGenerator::sendGetAdditionalSettingsRequest(uint8_t cellid)
{
  return BuildAndSendRequest(COMMAND::ReadAdditionalSettings, cellid, cellid);
}
bool PacketRequestGenerator::sendGetSettingsRequest(uint8_t cellid)
{
  return BuildAndSendRequest(COMMAND::ReadSettings, cellid, cellid);
}

bool PacketRequestGenerator::sendCellTemperatureRequest(uint8_t startmodule, uint8_t endmodule)
{
  return BuildAndSendRequest(COMMAND::ReadTemperature, startmodule, endmodule);
}

bool PacketRequestGenerator::sendReadBalanceCurrentCountRequest(uint8_t startmodule, uint8_t endmodule)
{
  return BuildAndSendRequest(COMMAND::ReadBalanceCurrentCounter, startmodule, endmodule);
}

bool PacketRequestGenerator::sendReadPacketsReceivedRequest(uint8_t startmodule, uint8_t endmodule)
{
  return BuildAndSendRequest(COMMAND::ReadPacketReceivedCounter, startmodule, endmodule);
}

bool PacketRequestGenerator::sendReadBalancePowerRequest(uint8_t startmodule, uint8_t endmodule)
{
  return BuildAndSendRequest(COMMAND::ReadBalancePowerPWM, startmodule, endmodule);
}

bool PacketRequestGenerator::sendBadPacketCounterReset()
{
  return BuildAndSendRequest(COMMAND::ResetBadPacketCounter);
}
bool PacketRequestGenerator::sendResetBalanceCurrentCounter()
{
  return BuildAndSendRequest(COMMAND::ResetBalanceCurrentCounter);
}

bool PacketRequestGenerator::BuildAndSendRequest(COMMAND command)
{
  // ESP_LOGD(TAG,"Build %u",command);

  PacketStruct _packetbuffer;
  clearPacket(&_packetbuffer);
  setPacketAddressBroadcast(&_packetbuffer);
  _packetbuffer.command = command;
  return pushPacketToQueue(&_packetbuffer);
}

bool PacketRequestGenerator::BuildAndSendRequest(COMMAND command, uint8_t startmodule, uint8_t endmodule)
{
  // ESP_LOGD(TAG,"Build %u, %u to %u",command,startmodule,endmodule);
  PacketStruct _packetbuffer;
  clearPacket(&_packetbuffer);
  setPacketAddressModuleRange(&_packetbuffer, startmodule, endmodule);
  _packetbuffer.command = command;
  return pushPacketToQueue(&_packetbuffer);
}

// Blocking call to push packet - blocks until there is space on the queue
bool PacketRequestGenerator::pushPacketToQueue(PacketStruct *_packetbuffer)
{
  return pushPacketToQueue(_packetbuffer, portMAX_DELAY);
}

bool PacketRequestGenerator::pushPacketToQueue(PacketStruct *_packetbuffer, TickType_t ticksToWait)
{
  if (xQueueSendToBack(_requestq, _packetbuffer, ticksToWait) != pdPASS)
  {
    // Failed to post the message, even after delay
    return false;
  }

  packetsGenerated++;
  return true;
}

void PacketRequestGenerator::setPacketAddressModuleRange(PacketStruct *_packetbuffer, uint8_t startmodule, uint8_t endmodule)
{
  _packetbuffer->start_address = startmodule;
  _packetbuffer->end_address = endmodule;
}

void PacketRequestGenerator::setPacketAddressBroadcast(PacketStruct *_packetbuffer)
{
  setPacketAddressModuleRange(_packetbuffer, 0, maximum_controller_cell_modules);
}

// Fill packet with 0xFFFF values - module ignores settings with this value
void PacketRequestGenerator::setmoduledataFFFF(PacketStruct *_packetbuffer)
{
  for (int a = 0; a < maximum_cell_modules_per_packet; a++)
  {
    _packetbuffer->moduledata[a] = 0xFFFF;
  }
}
