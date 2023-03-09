#ifndef influxdb_H_
#define influxdb_H_

#pragma once

#include "defines.h"

void influx_task_action();

extern uint8_t TotalNumberOfCells();
extern diybms_eeprom_settings mysettings;
extern bool wifi_isconnected;

#endif