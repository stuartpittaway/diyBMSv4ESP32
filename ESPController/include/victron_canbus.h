#ifndef DIYBMS_VICTRON_CANBUS_H_
#define DIYBMS_VICTRON_CANBUS_H_

#include "defines.h"
#include "Rules.h"
#include <driver/twai.h>

void victron_message_370_371();
void victron_message_35e();
void victron_message_351();
void victron_message_355();
void victron_message_356();
void victron_message_35a();
void victron_message_372();
void victron_message_373();
void victron_message_35f();
void victron_message_374_375_376_377();

extern void send_canbus_message(uint32_t identifier, const uint8_t *buffer,const uint8_t length);

extern uint8_t TotalNumberOfCells();
extern Rules rules;
extern currentmonitoring_struct currentMonitor;
extern diybms_eeprom_settings mysettings;
extern std::string hostname;
extern ControllerState _controller_state;
extern uint32_t canbus_messages_failed_sent;
extern uint32_t canbus_messages_sent;
extern uint32_t canbus_messages_received;

#endif
