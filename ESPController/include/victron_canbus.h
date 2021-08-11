#ifndef DIYBMS_VICTRON_CANBUS_H_
#define DIYBMS_VICTRON_CANBUS_H_

#include "defines.h"
#include "Rules.h"
#include <driver/can.h>

void victron_message_370_371();
void victron_message_35e();
void victron_message_351();
void victron_message_355();
void victron_message_356();
void victron_message_35a();
void victron_message_372();
void victron_message_373();
void victron_message_35f();
void victron_message_374();
void victron_message_375();
void victron_message_376();
void victron_message_377();

extern uint16_t TotalNumberOfCells();
extern Rules rules;
extern currentmonitoring_struct currentMonitor;
extern diybms_eeprom_settings mysettings;
extern char hostname[16];
extern ControllerState _controller_state;


#endif