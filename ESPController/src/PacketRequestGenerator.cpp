#include "PacketRequestGenerator.h"

void PacketRequestGenerator::clearSettingsForAllModules()
{
  // Force refresh of settings
  for (size_t i = 0; i < maximum_controller_cell_modules; i++)
  {
    cmi[i].settingsCached = false;
  }
}

void PacketRequestGenerator::sendSaveGlobalSetting(uint16_t BypassThresholdmV, uint8_t BypassOverTempShutdown)
{
  PacketStruct _packetbuffer;
  clearPacket(&_packetbuffer);

  //Ask all modules to set bypass and temperature value
  setPacketAddressBroadcast(&_packetbuffer);
  //Command - WriteSettings
  _packetbuffer.command = COMMAND::WriteSettings;

  //Fill packet with 0xFFFF values - module ignores settings with this value
  setmoduledataFFFF(&_packetbuffer);
  _packetbuffer.moduledata[6] = BypassOverTempShutdown;
  _packetbuffer.moduledata[7] = BypassThresholdmV;
  pushPacketToQueue(&_packetbuffer);

  clearSettingsForAllModules();
}

void PacketRequestGenerator::sendSaveSetting(uint8_t m, uint16_t BypassThresholdmV, uint8_t BypassOverTempShutdown, float Calibration)
{
  PacketStruct _packetbuffer;
  clearPacket(&_packetbuffer);
  setPacketAddressModuleRange(&_packetbuffer, m, m);
  //Command - WriteSettings
  _packetbuffer.command = COMMAND::WriteSettings;

  //Fill packet with 0xFFFF values - module ignores settings with this value
  setmoduledataFFFF(&_packetbuffer);

  // Force refresh of settings
  cmi[m].settingsCached = false;

  FLOATUNION_t myFloat;

  //myFloat.number = LoadResistance;
  //_packetbuffer.moduledata[0] = myFloat.word[0];
  //_packetbuffer.moduledata[1] = myFloat.word[1];

  // Arduino float(4 byte)
  myFloat.number = Calibration;
  _packetbuffer.moduledata[2] = myFloat.word[0];
  _packetbuffer.moduledata[3] = myFloat.word[1];

  // Arduino float(4 byte)
  //myFloat.number = mVPerADC;
  //_packetbuffer.moduledata[4] = myFloat.word[0];
  //_packetbuffer.moduledata[5] = myFloat.word[1];

  _packetbuffer.moduledata[6] = BypassOverTempShutdown;
  _packetbuffer.moduledata[7] = BypassThresholdmV;
  //_packetbuffer.moduledata[8] = Internal_BCoefficient;
  //_packetbuffer.moduledata[9] = External_BCoefficient;
  pushPacketToQueue(&_packetbuffer);
}

void PacketRequestGenerator::sendReadBadPacketCounter(uint8_t startmodule, uint8_t endmodule)
{
  BuildAndSendRequest(COMMAND::ReadBadPacketCounter, startmodule, endmodule);
}

void PacketRequestGenerator::sendCellVoltageRequest(uint8_t startmodule, uint8_t endmodule)
{
  BuildAndSendRequest(COMMAND::ReadVoltageAndStatus, startmodule, endmodule);
}

void PacketRequestGenerator::sendIdentifyModuleRequest(uint8_t cellid)
{
  BuildAndSendRequest(COMMAND::Identify, cellid, cellid);
}

void PacketRequestGenerator::sendTimingRequest()
{
  //Ask all modules to simple pass on a NULL request/packet for timing purposes
  BuildAndSendRequest(COMMAND::Timing);
}

void PacketRequestGenerator::sendGetSettingsRequest(uint8_t cellid)
{
  BuildAndSendRequest(COMMAND::ReadSettings, cellid, cellid);
}

void PacketRequestGenerator::sendCellTemperatureRequest(uint8_t startmodule, uint8_t endmodule)
{
  BuildAndSendRequest(COMMAND::ReadTemperature, startmodule, endmodule);
}

void PacketRequestGenerator::sendReadBalanceCurrentCountRequest(uint8_t startmodule, uint8_t endmodule)
{
  BuildAndSendRequest(COMMAND::ReadBalanceCurrentCounter, startmodule, endmodule);
}

void PacketRequestGenerator::sendReadPacketsReceivedRequest(uint8_t startmodule, uint8_t endmodule)
{
  BuildAndSendRequest(COMMAND::ReadPacketReceivedCounter, startmodule, endmodule);
}

void PacketRequestGenerator::sendReadBalancePowerRequest(uint8_t startmodule, uint8_t endmodule)
{
  BuildAndSendRequest(COMMAND::ReadBalancePowerPWM, startmodule, endmodule);
}

void PacketRequestGenerator::sendBadPacketCounterReset()
{
  BuildAndSendRequest(COMMAND::ResetBadPacketCounter);
}
void PacketRequestGenerator::sendResetBalanceCurrentCounter()
{
  BuildAndSendRequest(COMMAND::ResetBalanceCurrentCounter);
}


void PacketRequestGenerator::BuildAndSendRequest(COMMAND command)
{
  PacketStruct _packetbuffer;
  clearPacket(&_packetbuffer);
  setPacketAddressBroadcast(&_packetbuffer);
  _packetbuffer.command = command;
  pushPacketToQueue(&_packetbuffer);
}

void PacketRequestGenerator::BuildAndSendRequest(COMMAND command, uint8_t startmodule, uint8_t endmodule)
{
  PacketStruct _packetbuffer;
  clearPacket(&_packetbuffer);
  setPacketAddressModuleRange(&_packetbuffer, startmodule, endmodule);
  _packetbuffer.command = command;
  pushPacketToQueue(&_packetbuffer);
}

void PacketRequestGenerator::pushPacketToQueue(PacketStruct *_packetbuffer)
{
  _requestq->push(_packetbuffer);
  packetsGenerated++;
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

//Fill packet with 0xFFFF values - module ignores settings with this value
void PacketRequestGenerator::setmoduledataFFFF(PacketStruct *_packetbuffer)
{
  for (int a = 0; a < maximum_cell_modules_per_packet; a++)
  {
    _packetbuffer->moduledata[a] = 0xFFFF;
  }
}
