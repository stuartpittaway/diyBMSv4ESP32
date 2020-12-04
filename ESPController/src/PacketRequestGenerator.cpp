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
  setPacketAddressSingle(&_packetbuffer, m);
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
  //Read bad packet count (broadcast) to bank
  PacketStruct _packetbuffer;
  clearPacket(&_packetbuffer);
  setPacketAddressModuleRange(&_packetbuffer, startmodule, endmodule);
  _packetbuffer.command = COMMAND::ReadBadPacketCounter;
  //AVR MCUs are little endian (least significant byte first in memory)
  
  pushPacketToQueue(&_packetbuffer);
}

void PacketRequestGenerator::sendCellVoltageRequest(uint8_t startmodule, uint8_t endmodule)
{
  //Read voltage
  PacketStruct _packetbuffer;
  clearPacket(&_packetbuffer);
  setPacketAddressModuleRange(&_packetbuffer, startmodule, endmodule);
  //Command 1 - read voltage
  _packetbuffer.command = COMMAND::ReadVoltageAndStatus;
  //AVR MCUs are little endian (least significant byte first in memory)
  pushPacketToQueue(&_packetbuffer);
}

void PacketRequestGenerator::sendIdentifyModuleRequest(uint8_t cellid)
{
  //Read settings from single module
  PacketStruct _packetbuffer;
  clearPacket(&_packetbuffer);
  setPacketAddressSingle(&_packetbuffer, cellid);
  //Command 3 - identify
  _packetbuffer.command = COMMAND::Identify;
  pushPacketToQueue(&_packetbuffer);
}

void PacketRequestGenerator::sendTimingRequest()
{
  //Ask all modules to simple pass on a NULL request/packet for timing purposes
  PacketStruct _packetbuffer;
  clearPacket(&_packetbuffer);
  setPacketAddressBroadcast(&_packetbuffer);
  //Command - Timing
  _packetbuffer.command = COMMAND::Timing;
  pushPacketToQueue(&_packetbuffer);
}

void PacketRequestGenerator::sendGetSettingsRequest(uint8_t cellid)
{
  //SERIAL_DEBUG.println("sendGetSettingsRequest");
  PacketStruct _packetbuffer;
  clearPacket(&_packetbuffer);

  //Read settings from single module
  setPacketAddressSingle(&_packetbuffer, cellid);
  //Command 5 - read settings
  _packetbuffer.command = COMMAND::ReadSettings;
  pushPacketToQueue(&_packetbuffer);
}

void PacketRequestGenerator::sendCellTemperatureRequest(uint8_t startmodule, uint8_t endmodule)
{
  PacketStruct _packetbuffer;
  clearPacket(&_packetbuffer);
  //Read temperature (broadcast) to bank
  setPacketAddressModuleRange(&_packetbuffer, startmodule, endmodule);
  //Command 3 - read temperatures
  _packetbuffer.command = COMMAND::ReadTemperature;
  pushPacketToQueue(&_packetbuffer);
}

void PacketRequestGenerator::sendReadBalancePowerRequest(uint8_t startmodule, uint8_t endmodule)
{
  PacketStruct _packetbuffer;
  clearPacket(&_packetbuffer);
  //Read PWM value
  setPacketAddressModuleRange(&_packetbuffer, startmodule, endmodule);
  //Command 7 - read PWM
  _packetbuffer.command = COMMAND::ReadBalancePowerPWM;
  pushPacketToQueue(&_packetbuffer);
}

void PacketRequestGenerator::sendBadPacketCounterReset()
{
  PacketStruct _packetbuffer;
  clearPacket(&_packetbuffer);
  setPacketAddressBroadcast(&_packetbuffer);
  //Command 0
  _packetbuffer.command = COMMAND::ResetBadPacketCounter;
  pushPacketToQueue(&_packetbuffer);
}

void PacketRequestGenerator::pushPacketToQueue(PacketStruct *_packetbuffer)
{
  _requestq->push(_packetbuffer);
  packetsGenerated++;
}

void PacketRequestGenerator::setPacketAddressSingle(PacketStruct *_packetbuffer, uint8_t module)
{
  setPacketAddressModuleRange(_packetbuffer, module, module);
}
void PacketRequestGenerator::setPacketAddressModuleRange(PacketStruct *_packetbuffer, uint8_t startmodule, uint8_t endmodule)
{
  //This is already zero from clearPacket
  //_packetbuffer->hops = 0;
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
