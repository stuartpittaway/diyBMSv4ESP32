/*
 * DIYBMS ESP32 - interface of the module firmware update for main.cpp and the web server
 */
#ifndef ModuleFirmwareUpdateTask_H_
#define ModuleFirmwareUpdateTask_H_

#include <Arduino.h>
#include "defines.h"
#include "fwupdate_protocol.h"

// call once in setup() after request_q_handle has been created
void ModuleFwUpdate_Init(QueueHandle_t request_q);

// true for packets with command 12/13 (they belong to the update, not to PacketReceiveProcessor)
bool ModuleFwUpdate_IsFirmwarePacket(const PacketStruct *p);
// pass a reply packet to the update task
void ModuleFwUpdate_OnReply(const PacketStruct *p);

bool ModuleFwUpdate_Busy();
// next update packet for the transmit task (priority over normal requests), false = none
bool ModuleFwUpdate_NextPacket(PacketStruct *p);
// module in the bootloader (its readings are invalid) or -1
int16_t ModuleFwUpdate_ModuleInBootloader();
// minimum interval between two update packets in the transmit task
uint32_t ModuleFwUpdate_PacketIntervalMs();

// starts the update with entry manifestIndex from /avr/manifest.json
bool ModuleFwUpdate_Start(uint16_t manifestIndex, uint8_t numModules, uint32_t baudRate, char *message, size_t messageSize);
void ModuleFwUpdate_Abort();
// query the firmware checksum of all modules (result in the status JSON "fwcrc")
void ModuleFwUpdate_RequestCrcCheck(uint8_t numModules);
// controller baud rate (for display and as upload default)
void ModuleFwUpdate_SetControllerBaud(uint32_t baud);

// upload a firmware file: POST /upload/modfw?xss=..&name=..&baud=.. (body = image)
#include <esp_http_server.h>
esp_err_t ModuleFwUpdate_UploadHandler(httpd_req_t *req);

// remove a manifest entry (and its file)
bool ModuleFwUpdate_DeleteEntry(uint16_t manifestIndex, char *message, size_t messageSize);

// status as JSON, returns the length
int ModuleFwUpdate_StatusJson(char *buf, size_t size);

#endif
