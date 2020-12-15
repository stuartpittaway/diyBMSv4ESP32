#include "PacketReceiveProcessor.h"

bool PacketReceiveProcessor::HasCommsTimedOut()
{
  //We timeout the comms if we don't receive a packet within 3 times the normal
  //round trip time of the packets through the modules (minimum of 10 seconds to cater for low numbers of modules)
  uint32_t millisecondSinceLastPacket = millis() - packetLastReceivedMillisecond;
  return ((millisecondSinceLastPacket > 5 * packetTimerMillisecond) && (millisecondSinceLastPacket > 10000));
}

bool PacketReceiveProcessor::ProcessReply(PacketStruct *receivebuffer)
{
  packetsReceived++;

  //TODO: VALIDATE REPLY START/END RANGES ARE VALID TO AVOID MEMORY BUFFER OVERRUNS

  // Copy to our buffer (probably don't need to do this), just use pointer instead
  memcpy(&_packetbuffer, receivebuffer, sizeof(_packetbuffer));

  // Calculate the CRC and compare to received
  uint16_t validateCRC = CRC16::CalculateArray((uint8_t *)&_packetbuffer, sizeof(_packetbuffer) - 2);

  if (validateCRC == _packetbuffer.crc)
  {
    //Its a valid packet...
    packetLastReceivedMillisecond = millis();

    totalModulesFound = _packetbuffer.hops;

    if (packetLastReceivedSequence > 0 && _packetbuffer.sequence != packetLastReceivedSequence + 1)
    {
      SERIAL_DEBUG.println();
      SERIAL_DEBUG.print(F("OOS Error, expected="));
      SERIAL_DEBUG.print(packetLastReceivedSequence+1, HEX);
      SERIAL_DEBUG.print(", got=");
      SERIAL_DEBUG.println(_packetbuffer.sequence, HEX);
      totalOutofSequenceErrors++;
    }

    packetLastReceivedSequence = _packetbuffer.sequence;

    if (ReplyWasProcessedByAModule())
    {

      switch (ReplyForCommand())
      {
      case COMMAND::ResetBadPacketCounter:
        break; // Ignore reply

      case COMMAND::Timing:
      {
        //uint32_t tnow = millis();
        uint32_t tnow = (_packetbuffer.moduledata[2] << 16) + _packetbuffer.moduledata[3];
        uint32_t tprevious = (_packetbuffer.moduledata[0] << 16) + _packetbuffer.moduledata[1];

        //Check millis time hasn't rolled over
        if (tnow > tprevious)
        {
          packetTimerMillisecond = tnow - tprevious;
        }

        break;
      }
      case COMMAND::ReadVoltageAndStatus:
        ProcessReplyVoltage();
        break;

      case COMMAND::ReadBadPacketCounter:
        ProcessReplyBadPacketCount();
        break;

      case COMMAND::Identify:
        break; // Ignore reply

      case COMMAND::ReadTemperature:
        ProcessReplyTemperature();
        break;

      case COMMAND::ReadSettings:
        ProcessReplySettings();
        break;

      case COMMAND::ReadBalancePowerPWM:
        ProcessReplyBalancePower();
        break;

    case COMMAND::ReadBalanceCurrentCounter:
        ProcessReplyReadBalanceCurrentCounter();
        break;
    case COMMAND::ReadPacketReceivedCounter:
        ProcessReplyReadPacketReceivedCounter();
        break;

      }

#if defined(PACKET_LOGGING_RECEIVE)
      SERIAL_DEBUG.println(F("*OK*"));
#endif
      return true;
    }
    else
    {
      //Error count for a request that was not processed by any module in the string
      totalNotProcessedErrors++;
#if defined(PACKET_LOGGING_RECEIVE)
      SERIAL_DEBUG.println(F("*IGNORE*"));
#endif
    }
  }
  else
  {
    //crc error
    totalCRCErrors++;
#if defined(PACKET_LOGGING_RECEIVE)
    SERIAL_DEBUG.println(F("*CRC Error*"));
#endif
  }
 
  return false;
}

void PacketReceiveProcessor::ProcessReplyBadPacketCount()
{
  // Called when a decoded packet has arrived in buffer for command
  uint8_t q = 0;
  for (uint8_t i = _packetbuffer.start_address; i <= _packetbuffer.end_address; i++)
  {
    cmi[i].badPacketCount = _packetbuffer.moduledata[q];
    q++;
  }
}

void PacketReceiveProcessor::ProcessReplyTemperature()
{
  // Called when a decoded packet has arrived in buffer for command 3

  // 40 offset for below zero temps
  uint8_t q = 0;
  for (uint8_t i = _packetbuffer.start_address; i <= _packetbuffer.end_address; i++)
  {
    cmi[i].internalTemp = ((_packetbuffer.moduledata[q] & 0xFF00) >> 8) - 40;
    cmi[i].externalTemp = (_packetbuffer.moduledata[q] & 0x00FF) - 40;
    q++;
  }
}


void PacketReceiveProcessor::ProcessReplyReadBalanceCurrentCounter()
{
  uint8_t q = 0;
  for (uint8_t i = _packetbuffer.start_address; i <= _packetbuffer.end_address; i++)
  {
    cmi[i].BalanceCurrentCount = _packetbuffer.moduledata[q];
    q++;
  }
}
void PacketReceiveProcessor::ProcessReplyReadPacketReceivedCounter()
{
  uint8_t q = 0;
  for (uint8_t i = _packetbuffer.start_address; i <= _packetbuffer.end_address; i++)
  {
    cmi[i].PacketReceivedCount = _packetbuffer.moduledata[q];
    q++;
  }
}


void PacketReceiveProcessor::ProcessReplyBalancePower()
{
  // Called when a decoded packet has arrived in _packetbuffer for command 1
  uint8_t q = 0;
  for (uint8_t i = _packetbuffer.start_address; i <= _packetbuffer.end_address; i++)
  {
    cmi[i].PWMValue = _packetbuffer.moduledata[q];
    q++;
  }
}

void PacketReceiveProcessor::ProcessReplyVoltage()
{
  // Called when a decoded packet has arrived in _packetbuffer for command 1

  if (_packetbuffer.end_address < _packetbuffer.start_address)
    return;

  for (uint8_t i = 0; i <= _packetbuffer.end_address - _packetbuffer.start_address; i++)
  {

    CellModuleInfo *cellptr = &cmi[_packetbuffer.start_address + i];

    // 3 top bits remaining
    // X = In bypass
    // Y = Bypass over temperature
    // Z = Not used

    cellptr->voltagemV = _packetbuffer.moduledata[i] & 0x1FFF;
    cellptr->inBypass = (_packetbuffer.moduledata[i] & 0x8000) > 0;
    cellptr->bypassOverTemp = (_packetbuffer.moduledata[i] & 0x4000) > 0;

    if (cellptr->voltagemV > cellptr->voltagemVMax)
    {
      cellptr->voltagemVMax = cellptr->voltagemV;
    }

    if (cellptr->voltagemV < cellptr->voltagemVMin)
    {
      cellptr->voltagemVMin = cellptr->voltagemV;
    }

    if (cellptr->voltagemV > 0)
    {
      cellptr->valid = true;
    }
  }
}

void PacketReceiveProcessor::ProcessReplySettings()
{

  uint8_t m = _packetbuffer.start_address;

  // TODO Validate b and m here to prevent array overflow
  cmi[m].settingsCached = true;

  FLOATUNION_t myFloat;

  myFloat.word[0] = _packetbuffer.moduledata[0];
  myFloat.word[1] = _packetbuffer.moduledata[1];

  // Arduino float (4 byte)
  cmi[m].LoadResistance = myFloat.number;
  // Arduino float(4 byte)
  myFloat.word[0] = _packetbuffer.moduledata[2];
  myFloat.word[1] = _packetbuffer.moduledata[3];
  cmi[m].Calibration = myFloat.number;

  // Arduino float(4 byte)
  myFloat.word[0] = _packetbuffer.moduledata[4];
  myFloat.word[1] = _packetbuffer.moduledata[5];
  cmi[m].mVPerADC = myFloat.number;
  // uint8_t
  cmi[m].BypassOverTempShutdown = _packetbuffer.moduledata[6] & 0x00FF;
  // uint16_t
  cmi[m].BypassThresholdmV = _packetbuffer.moduledata[7];
  // uint16_t
  cmi[m].Internal_BCoefficient = _packetbuffer.moduledata[8];
  // uint16_t
  cmi[m].External_BCoefficient = _packetbuffer.moduledata[9];
  // uint16_t
  cmi[m].BoardVersionNumber = _packetbuffer.moduledata[10];

  cmi[m].CodeVersionNumber = (_packetbuffer.moduledata[14] << 16) +_packetbuffer.moduledata[15];
}
