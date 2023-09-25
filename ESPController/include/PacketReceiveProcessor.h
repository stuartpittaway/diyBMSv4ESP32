#ifndef PacketReceiveProcessor_H_
#define PacketReceiveProcessor_H_

#include <Arduino.h>
#include <defines.h>

#include "crc16.h"

class PacketReceiveProcessor
{
public:
  PacketReceiveProcessor() {}
  ~PacketReceiveProcessor() {}
  bool ProcessReply(const PacketStruct *receivebuffer);
  bool HasCommsTimedOut()  const;

  uint16_t totalCRCErrors = 0;
  uint16_t totalOutofSequenceErrors = 0;
  uint16_t totalNotProcessedErrors = 0;
  uint32_t packetsReceived = 0;
  uint8_t totalModulesFound = 0;

  //Duration (ms) for a packet to travel through the string (default to 60 seconds at startup)
  uint32_t packetTimerMillisecond = 60 * 1000;
  uint32_t packetLastReceivedMillisecond = 0;
  uint16_t packetLastReceivedSequence = 0;

  void ResetCounters()
  {
    totalCRCErrors = 0;
    totalNotProcessedErrors = 0;
    packetsReceived = 0;
    totalOutofSequenceErrors = 0;
  }

private:
  PacketStruct _packetbuffer;
  //uint8_t ReplyFromBank() {return (_packetbuffer.address & B00110000) >> 4;}
  //See issue 11 - if we receive zero for the address then we have 16 modules or no modules and a loop
  uint8_t ReplyForCommand() { return (_packetbuffer.command & 0x0F); }
  bool ReplyWasProcessedByAModule() { return (_packetbuffer.command & B10000000) > 0; }

  void ProcessReplySettings();
  void ProcessReplyVoltage();
  void ProcessReplyTemperature();

  void ProcessReplyBadPacketCount();
  void ProcessReplyBalancePower();
  void ProcessReplyReadBalanceCurrentCounter();
  void ProcessReplyReadPacketReceivedCounter();
  void ProcessReplyAdditionalSettings();
};

extern TaskHandle_t voltageandstatussnapshot_task_handle;

#endif
