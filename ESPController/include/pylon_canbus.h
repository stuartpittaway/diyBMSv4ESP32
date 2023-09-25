#ifndef DIYBMS_PYLON_CANBUS_H_
#define DIYBMS_PYLON_CANBUS_H_

#include "defines.h"
#include "Rules.h"
#include <driver/twai.h>

void pylon_message_356();
void pylon_message_35e();
void pylon_message_351();
void pylon_message_355();
void pylon_message_359();
void pylon_message_35c();


extern uint8_t TotalNumberOfCells();
extern Rules rules;
extern currentmonitoring_struct currentMonitor;
extern diybms_eeprom_settings mysettings;
extern std::string hostname;
extern ControllerState _controller_state;
extern uint32_t canbus_messages_failed_sent;
extern uint32_t canbus_messages_sent;
extern uint32_t canbus_messages_received;

extern void send_canbus_message(uint32_t identifier, const uint8_t *buffer,const uint8_t length);

#endif