/*
 ____  ____  _  _  ____  __  __  ___
(  _ \(_  _)( \/ )(  _ \(  \/  )/ __)
 )(_) )_)(_  \  /  ) _ < )    ( \__ \
(____/(____) (__) (____/(_/\/\_)(___/

  (c) 2022 Stuart Pittaway

This code communicates emulates a PYLON TECH BATTERY using CANBUS @ 500kbps and 11 bit addresses.

*/

#define USE_ESP_IDF_LOG 1
static constexpr const char * const TAG = "diybms-pylon";

#include "pylon_canbus.h"

void pylon_message_35e()
{
  //Send 8 byte "magic string" PYLON (with 3 trailing zero bytes)
  const char pylon[]="\x50\x59\x4c\x4f\x4e\x00\x00\x00";
  send_canbus_message(0x35e, (uint8_t *)&pylon, sizeof(pylon)-1);
}


// Battery voltage
void pylon_message_356()
{
  struct data356
  {
    int16_t voltage;
    int16_t current;
    int16_t temperature;
  };

  data356 data;

  // Use highest pack voltage calculated by controller and modules
  data.voltage = rules.highestPackVoltage / 10;

  // If current shunt is installed, use the voltage from that as it should be more accurate
  if (mysettings.currentMonitoringEnabled && currentMonitor.validReadings)
  {
    data.voltage = currentMonitor.modbus.voltage * 100.0;
  }

  data.current = 0;
  // If current shunt is installed, use it
  if (mysettings.currentMonitoringEnabled && currentMonitor.validReadings)
  {
    data.current = currentMonitor.modbus.current * 10;
  }

  // Temperature 0.1 C using external temperature sensor
  if (rules.moduleHasExternalTempSensor)
  {
    data.temperature = (int16_t)rules.highestExternalTemp * (int16_t)10;
  }
  else
  {
    // No external temp sensors
    data.temperature = 0;
  }

  send_canbus_message(0x356, (uint8_t *)&data, sizeof(data356));
}
