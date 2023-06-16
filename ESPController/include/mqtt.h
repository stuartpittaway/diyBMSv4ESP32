#ifndef mqtt_H_
#define mqtt_H_

#pragma once

#include "defines.h"
#include "Rules.h"
#include "PacketRequestGenerator.h"
#include "PacketReceiveProcessor.h"

#include <mqtt_client.h>

void stopMqtt();
void connectToMqtt();
void mqtt3(const Rules *rules,const RelayState *previousRelayState);
void mqtt2(const PacketReceiveProcessor *receiveProc,
           const PacketRequestGenerator *prg,
           uint16_t requestq_count,
           const Rules *rules);
void mqtt1(const currentmonitoring_struct *currentMonitor,const Rules *rules);
void GeneralStatusPayload(const PacketRequestGenerator *prg, const PacketReceiveProcessor *receiveProc, uint16_t requestq_count,const Rules *rules);
void BankLevelInformation(const Rules *rules);
void RuleStatus(const Rules *rules);

extern uint8_t TotalNumberOfCells();
extern diybms_eeprom_settings mysettings;
extern bool wifi_isconnected;

#endif