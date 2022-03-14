#ifndef mqtt_H_
#define mqtt_H_

#pragma once

#include "defines.h"
#include "Rules.h"
#include "PacketRequestGenerator.h"
#include "PacketReceiveProcessor.h"

#include <mqtt_client.h>
#include <ArduinoJson.h>
#include <cppQueue.h>

static void mqtt_connected_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
static void mqtt_disconnected_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
static void mqtt_error_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
void stopMqtt();
void connectToMqtt();
void mqtt2(PacketReceiveProcessor *receiveProc,PacketRequestGenerator *prg, uint16_t requestq_count,Rules *rules, RelayState *previousRelayState);
void mqtt1(currentmonitoring_struct *currentMonitor,Rules *rules);

extern uint16_t TotalNumberOfCells();
extern diybms_eeprom_settings mysettings;


#endif