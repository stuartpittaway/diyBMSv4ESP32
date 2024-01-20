#ifndef DIYBMS_PYLONFORCE_CANBUS_H_
#define DIYBMS_PYLONFORCE_CANBUS_H_

#include "defines.h"
#include "Rules.h"
#include <driver/twai.h>


void pylonforce_handle_rx(twai_message_t *);
void pylonforce_handle_tx();


extern uint8_t TotalNumberOfCells();
extern Rules rules;
extern currentmonitoring_struct currentMonitor;
extern diybms_eeprom_settings mysettings;
extern std::string hostname;
extern ControllerState _controller_state;
extern uint32_t canbus_messages_failed_sent;
extern uint32_t canbus_messages_sent;
extern uint32_t canbus_messages_received;
extern bool wifi_isconnected;

extern void send_ext_canbus_message(const uint32_t identifier, const uint8_t *buffer, const uint8_t length);

#endif