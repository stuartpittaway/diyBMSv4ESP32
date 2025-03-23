#ifndef DIYBMS_PYLON_CANBUS_H_
#define DIYBMS_PYLON_CANBUS_H_

#include "defines.h"
#include "Rules.h"
#include <driver/twai.h>
#include "ControllerCAN.h"

void pylon_message_351();
void pylon_message_355();
void pylon_message_356();
void pylon_message_359();
void pylon_message_35c();
void pylon_message_35e();

extern ControllerCAN can;
extern bool controller_heartbeat(uint8_t ControllerID);
extern TaskHandle_t canbus_rx_task_handle, canbus_tx_task_handle;
extern void send_canbus_message(CANframe *canframe);

#endif