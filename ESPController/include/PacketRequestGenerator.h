#ifndef PacketRequestGenerator_H_
#define PacketRequestGenerator_H_

#include <Arduino.h>
#include <defines.h>

// command byte
//  WRRR CCCC
//  W    = 1 bit indicator packet was processed (controller send (0) module processed (1))
//  R    = 3 bits reserved not used
//  C    = 4 bits command (16 possible commands)

// commands
//  1000 0000  = set bank identity
//  0000 0001  = read voltage and status
//  0000 0010  = identify module (flash leds)
//  0000 0011  = Read temperature
//  0000 0100  = Report number of bad packets
//  0000 0101  = Report settings/configuration

class PacketRequestGenerator
{
public:
  bool sendGetSettingsRequest(uint8_t cellid);
  bool sendIdentifyModuleRequest(uint8_t cellid);
  bool sendSaveSetting(uint8_t m, uint16_t BypassThresholdmV, uint8_t BypassOverTempShutdown, float Calibration);
  bool sendSaveGlobalSetting(uint16_t BypassThresholdmV, uint8_t BypassOverTempShutdown);
  bool sendReadBadPacketCounter(uint8_t startmodule, uint8_t endmodule);

  bool sendCellVoltageRequest(uint8_t startmodule, uint8_t endmodule);
  bool sendCellTemperatureRequest(uint8_t startmodule, uint8_t endmodule);
  bool sendReadBalancePowerRequest(uint8_t startmodule, uint8_t endmodule);
  bool sendReadBalanceCurrentCountRequest(uint8_t startmodule, uint8_t endmodule);
  bool sendReadPacketsReceivedRequest(uint8_t startmodule, uint8_t endmodule);
  bool sendBadPacketCounterReset();
  bool sendTimingRequest();
  bool sendResetBalanceCurrentCounter();
  bool sendGetAdditionalSettingsRequest(uint8_t cellid);
  bool sendSaveAdditionalSetting(uint8_t cellid, int16_t FanSwitchOnT, uint16_t RelayMinV, uint16_t RelayRange, uint16_t RunAwayCellMinimumVoltagemV, uint16_t RunAwayCellDifferentialmV);

  uint16_t queueLength()
  {
    return (uint16_t)uxQueueMessagesWaiting(_requestq);
  }

  void ResetCounters()
  {
    packetsGenerated = 0;
  }

  void setQueueHandle(QueueHandle_t q)
  {
    _requestq = q;
  }

  uint32_t packetsGenerated = 0;

private:
  QueueHandle_t _requestq = nullptr;
  bool pushPacketToQueue(PacketStruct *_packetbuffer, TickType_t ticksToWait);
  bool pushPacketToQueue(PacketStruct *_packetbuffer);
  void setPacketAddress(PacketStruct *_packetbuffer, uint8_t module);
  void setPacketAddressModuleRange(PacketStruct *_packetbuffer, uint8_t startmodule, uint8_t endmodule);
  void setPacketAddressBroadcast(PacketStruct *_packetbuffer);
  void setmoduledataFFFF(PacketStruct *_packetbuffer);
  void clearSettingsForAllModules();
  bool BuildAndSendRequest(COMMAND command, uint8_t startmodule, uint8_t endmodule);
  bool BuildAndSendRequest(COMMAND command);
  void clearPacket(PacketStruct *_packetbuffer)
  {
    memset(_packetbuffer, 0, sizeof(PacketStruct));
  }
};

#endif
