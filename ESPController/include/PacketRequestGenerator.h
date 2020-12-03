#ifndef PacketRequestGenerator_H_
#define PacketRequestGenerator_H_

#include <Arduino.h>
#include <cppQueue.h>
#include <defines.h>


//command byte
// WRRR CCCC
// W    = 1 bit indicator packet was processed (controller send (0) module processed (1))
// R    = 3 bits reserved not used
// C    = 4 bits command (16 possible commands)

//commands
// 1000 0000  = set bank identity
// 0000 0001  = read voltage and status
// 0000 0010  = identify module (flash leds)
// 0000 0011  = Read temperature
// 0000 0100  = Report number of bad packets
// 0000 0101  = Report settings/configuration


class PacketRequestGenerator {
   public:
     PacketRequestGenerator(cppQueue* requestQ) {_requestq=requestQ;}
     ~PacketRequestGenerator() {}
     void sendGetSettingsRequest(uint8_t cellid);
     void sendIdentifyModuleRequest(uint8_t cellid);
     void sendSaveSetting(uint8_t m, uint16_t BypassThresholdmV, uint8_t BypassOverTempShutdown, float Calibration);
     void sendSaveGlobalSetting(uint16_t BypassThresholdmV,uint8_t BypassOverTempShutdown);
     void sendReadBadPacketCounter(uint8_t startmodule,uint8_t endmodule);

     void sendCellVoltageRequest(uint8_t startmodule,uint8_t endmodule);
     void sendCellTemperatureRequest(uint8_t startmodule,uint8_t endmodule);
     void sendReadBalancePowerRequest(uint8_t startmodule,uint8_t endmodule);

     void sendBadPacketCounterReset();
     void sendTimingRequest();

     uint32_t packetsGenerated = 0;
     uint16_t QueueLength();

  private:
    cppQueue* _requestq;
    PacketStruct _packetbuffer;
    void pushPacketToQueue();    
    void setPacketAddress(uint8_t module);
    void setPacketAddressSingle(uint8_t module);
    void setPacketAddressModuleRange(uint8_t startmodule,uint8_t endmodule);
    void setPacketAddressBroadcast();
    void clearmoduledata();
    void setmoduledataFFFF();
    void clearSettingsForAllModules();
};



#endif
