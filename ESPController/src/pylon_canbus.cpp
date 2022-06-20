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
// 0x351 - 0x44 0x02 0x64 0x00 0x9C 0xFF 0x3F 0x02
//  0x0214 = 53.2V
//  0x0e74 = B111001110100 = 3700
//  0x0e74 = B111001110100 = 3700
//  0x01CC = 46.0V
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
    // ESP_LOGW(TAG, "active_errors=%u", number_of_active_errors);
    //  Error condition
    data.battery_charge_voltage = mysettings.cvl[VictronDVCC::ControllerError];
    data.battery_charge_current_limit = mysettings.ccl[VictronDVCC::ControllerError];
    data.battery_discharge_current_limit = mysettings.dcl[VictronDVCC::ControllerError];
  }
  else if (rules.numberOfBalancingModules > 0)
  {
    // Balancing
    data.battery_charge_voltage = mysettings.cvl[VictronDVCC::Balance];
    data.battery_charge_current_limit = mysettings.ccl[VictronDVCC::Balance];
    data.battery_discharge_current_limit = mysettings.dcl[VictronDVCC::Balance];
  }
  else
  {
    // Default - normal behaviour
    data.battery_charge_voltage = mysettings.cvl[VictronDVCC::Default];
    data.battery_charge_current_limit = mysettings.ccl[VictronDVCC::Default];
    data.battery_discharge_current_limit = mysettings.dcl[VictronDVCC::Default];
  }

  // Hardcoded for now!
  data.battery_discharge_voltage = 0; // data.battery_charge_voltage - 50;

  send_canbus_message(0x351, (uint8_t *)&data, sizeof(data351));
}

// 0x355 – 1A 00 64 00 – State of Health (SOH) / State of Charge (SOC)
void pylon_message_355()
{
  struct data355
  {
    uint16_t stateofchargevalue;
    uint16_t stateofhealthvalue;
  };

  if (_controller_state == ControllerState::Running && mysettings.currentMonitoringEnabled && currentMonitor.validReadings && mysettings.currentMonitoringDevice == CurrentMonitorDevice::DIYBMS_CURRENT_MON)
  {
    data355 data;
    // 0 SOC value un16 1 %
    data.stateofchargevalue = currentMonitor.stateofcharge;
    // Fake SOC based on cell voltage
    //data.stateofchargevalue = min((uint16_t)100,(uint16_t)(100/(3.5 - 3.0)*(((rules.highestPackVoltage/10)/mysettings.totalNumberOfSeriesModules)-3.0)));
    //  2 SOH value un16 1 %
    data.stateofhealthvalue = 100;
    send_canbus_message(0x355, (uint8_t *)&data, sizeof(data355));
  }
}

// 0x359 – 00 00 00 00 0A 50 4E – Protection & Alarm flags
void pylon_message_359()
{
  struct data359
  {
    uint8_t byte0;
    uint8_t byte1;
    uint8_t byte2;
    uint8_t byte3;
    uint8_t byte4;
    uint8_t byte5;
    uint8_t byte6;
    uint8_t byte7;
  };

  data359 data;

  memset(&data, 0, sizeof(data359));

  // Byte0 = Protection 1
  // Byte1 = Protection 2
  // Byte2 = Warning 1
  // Byte3 = Warning 2
  // Byte4 = Quantity of packs in parallel
  // Byte5 = unused
  // Byte6 = unused
  // Byte7 = Address of packs in parallel

  if (_controller_state == ControllerState::Running)
  {
    //(bit 1) Battery high voltage alarm
    data.byte0 |= ((rules.rule_outcome[Rule::BankOverVoltage] | rules.rule_outcome[Rule::CurrentMonitorOverVoltage]) ? B00000010 : 0);
    //(bit 2) Battery low voltage alarm
    data.byte0 |= ((rules.rule_outcome[Rule::BankUnderVoltage] | rules.rule_outcome[Rule::CurrentMonitorUnderVoltage]) ? B00000100 : 0);

    //(bit 3) Battery high temperature alarm
    if (rules.moduleHasExternalTempSensor)
    {
      data.byte0 |= (rules.rule_outcome[Rule::ModuleOverTemperatureExternal] ? B00001000 : 0);
    }

    // (bit 4) Battery low temperature alarm
    if (rules.moduleHasExternalTempSensor)
    {
      data.byte0 |= (rules.rule_outcome[Rule::ModuleUnderTemperatureExternal] ? B00010000 : 0);
    }
  }

  data.byte3 |= ((rules.rule_outcome[Rule::BMSError] | rules.rule_outcome[Rule::EmergencyStop]) ? B00001000 : 0);
  data.byte3 |= ((_controller_state != ControllerState::Running) ? B00001000 : 0);

  if (mysettings.currentMonitoringEnabled && currentMonitor.validReadings)
  {
    // Pylon can have multiple battery each 74Ah capacity, so emulate this based on total Ah capacity
    data.byte4 = min((uint8_t)1, (uint8_t)round(currentMonitor.modbus.batterycapacityamphour / 74.0));
  }
  else
  {
    // Default 1 battery
    data.byte4 = 1;
  }

  data.byte5 = 0x50; // P
  data.byte6 = 0x4e; // N

  send_canbus_message(0x359, (uint8_t *)&data, sizeof(data359));
}

// 0x35C – C0 00 – Battery charge request flags
void pylon_message_35c()
{
  struct data35c
  {
    uint8_t byte0;
    uint8_t byte1;
  };

  data35c data;

  // Charge enable/Discharge enable
  data.byte0 = B11000000;
  data.byte1 = 0;

  send_canbus_message(0x35c, (uint8_t *)&data, sizeof(data35c));
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
