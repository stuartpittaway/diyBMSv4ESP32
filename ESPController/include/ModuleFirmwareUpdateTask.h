/*
 * DIYBMS ESP32 - Schnittstelle des Modul-Firmware-Updates fuer main.cpp und Webserver
 */
#ifndef ModuleFirmwareUpdateTask_H_
#define ModuleFirmwareUpdateTask_H_

#include <Arduino.h>
#include "defines.h"
#include "fwupdate_protocol.h"

// einmal in setup() nach dem Anlegen von request_q_handle aufrufen
void ModuleFwUpdate_Init(QueueHandle_t request_q);

// true fuer Pakete mit Kommando 12/13 (gehoeren dem Update, nicht dem PacketReceiveProcessor)
bool ModuleFwUpdate_IsFirmwarePacket(const PacketStruct *p);
// Antwortpaket an den Update-Task weitergeben
void ModuleFwUpdate_OnReply(const PacketStruct *p);

bool ModuleFwUpdate_Busy();
// Modul im Bootloader (seine Messwerte sind ungueltig) oder -1
int16_t ModuleFwUpdate_ModuleInBootloader();
// Mindestabstand zwischen zwei Update-Paketen im Sende-Task
uint32_t ModuleFwUpdate_PacketIntervalMs();

// Startet das Update mit Eintrag manifestIndex aus /avr/manifest.json
bool ModuleFwUpdate_Start(uint16_t manifestIndex, uint8_t numModules, uint32_t baudRate, char *message, size_t messageSize);
void ModuleFwUpdate_Abort();
// Status als JSON, Rueckgabe = Laenge
int ModuleFwUpdate_StatusJson(char *buf, size_t size);

#endif
