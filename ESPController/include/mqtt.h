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
void mqtt2(PacketReceiveProcessor *receiveProc,PacketRequestGenerator *prg, uint16_t requestq_count,Rules *rules, RelayState *previousRelayState);
void mqtt1(currentmonitoring_struct *currentMonitor,Rules *rules);

extern uint16_t TotalNumberOfCells();
extern diybms_eeprom_settings mysettings;
extern bool wifi_isconnected;

#endif