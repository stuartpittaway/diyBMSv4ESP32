/*
 ____  ____  _  _  ____  __  __  ___
(  _ \(_  _)( \/ )(  _ \(  \/  )/ __)
 )(_) )_)(_  \  /  ) _ < )    ( \__ \
(____/(____) (__) (____/(_/\/\_)(___/

  (c) 2022 Stuart Pittaway

This code communicates emulates a PYLON TECH BATTERY using CANBUS @ 500kbps and 11 bit addresses.

*/

#define USE_ESP_IDF_LOG 1
static constexpr const char *const TAG = "diybms-pylon";

#include "pylon_canbus.h"

// 0x351 – 14 02 74 0E 74 0E CC 01 – Battery voltage + current limits
void pylon_message_351()
{

  uint8_t number_of_active_errors = 0;

  if (_controller_state == ControllerState::Running)
  {
    number_of_active_errors += (rules.rule_outcome[Rule::BankOverVoltage] ? 1 : 0);
    // Battery high voltage alarm
    number_of_active_errors += (rules.rule_outcome[Rule::BankUnderVoltage] ? 1 : 0);
    // Battery high temperature alarm
    if (rules.moduleHasExternalTempSensor)
    {
      number_of_active_errors += (rules.rule_outcome[Rule::ModuleOverTemperatureExternal] ? 1 : 0);
    }

    if (rules.moduleHasExternalTempSensor)
    {
      number_of_active_errors += (rules.rule_outcome[Rule::ModuleUnderTemperatureExternal] ? 1 : 0);
    }

    number_of_active_errors += ((rules.rule_outcome[Rule::BMSError] | rules.rule_outcome[Rule::EmergencyStop]) ? 1 : 0);
  }

  struct data351
  {
    uint16_t battery_charge_voltage;
    // positive number
    int16_t battery_charge_current_limit;
    // negative number
    int16_t battery_discharge_current_limit;
    uint16_t battery_discharge_voltage;
  };

  data351 data;

  if ((_controller_state != ControllerState::Running) || (number_of_active_errors > 0))
  {
    ESP_LOGW(TAG, "active_errors=%u", number_of_active_errors);
    // Error condition
    data.battery_charge_voltage = mysettings.cvl[VictronDVCC::ControllerError];
    data.battery_charge_current_limit = mysettings.ccl[VictronDVCC::ControllerError];
    data.battery_discharge_current_limit = -(mysettings.dcl[VictronDVCC::ControllerError]);
  }
  else if (rules.numberOfBalancingModules > 0)
  {
    // Balancing
    data.battery_charge_voltage = mysettings.cvl[VictronDVCC::Balance];
    data.battery_charge_current_limit = mysettings.ccl[VictronDVCC::Balance];
    data.battery_discharge_current_limit = -(mysettings.dcl[VictronDVCC::Balance]);
  }
  else
  {
    // Default - normal behaviour
    data.battery_charge_voltage = mysettings.cvl[VictronDVCC::Default];
    data.battery_charge_current_limit = mysettings.ccl[VictronDVCC::Default];
    data.battery_discharge_current_limit = -(mysettings.dcl[VictronDVCC::Default]);
  }

  // Hardcoded for now!
  data.battery_discharge_voltage = data.battery_charge_voltage - 5;

  send_canbus_message(0x351, (uint8_t *)&data, sizeof(data351));
}

// 0x355 – 1A 00 64 00 – State of Health (SOH) / State of Charge (SOC)
void pylon_message_355()
{
}

// 0x359 – 00 00 00 00 0A 50 4E – Protection & Alarm flags
void pylon_message_359()
{
}

// 0x35C – C0 00 – Battery charge request flags
void pylon_message_35c()
{
}

// 0x35E – 50 59 4C 4F 4E 20 20 20 – Manufacturer name ("PYLON ")
void pylon_message_35e()
{
  // Send 8 byte "magic string" PYLON (with 3 trailing spaces)
  const char pylon[] = "\x50\x59\x4c\x4f\x4e\x20\x20\x20";
  send_canbus_message(0x35e, (uint8_t *)&pylon, sizeof(pylon) - 1);
}

// Battery voltage - 0x356 – 4e 13 02 03 04 05 – Voltage / Current / Temp
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
