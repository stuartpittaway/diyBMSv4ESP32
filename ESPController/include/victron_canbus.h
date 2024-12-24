#ifndef DIYBMS_VICTRON_CANBUS_H_
#define DIYBMS_VICTRON_CANBUS_H_

#include "defines.h"
#include "Rules.h"
#include <driver/twai.h>
#include "ControllerCAN.h"

void victron_message_370_371_35e();
void victron_message_35f();
void victron_message_373_374_375_376_377();
void victron_message_351();
void victron_message_355();
void victron_message_356();
void victron_message_372();
void victron_message_35a();
void victron_message_379();

extern ControllerCAN can;
extern bool controller_heartbeat(uint8_t ControllerID);
extern TaskHandle_t canbus_rx_task_handle, canbus_tx_task_handle;

#endif
