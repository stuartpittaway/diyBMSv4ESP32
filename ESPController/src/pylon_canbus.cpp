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

bool IsChargeAllowed()
{
  if (_controller_state != ControllerState::Running)
    return false;

  if (mysettings.preventcharging == true)
    return false;

  if (rules.moduleHasExternalTempSensor == false)
    return false;

  if (rules.lowestExternalTemp<mysettings.chargetemplow | rules.highestExternalTemp> mysettings.chargetemphigh)
  {
    // Stop charge - temperature out of range
    // ESP_LOGW(TAG, "Stop charge - temperature out of range");
    return false;
  }

  if (rules.highestPackVoltage > mysettings.chargevolt)
    return false;

  // Any errors, stop charge
  if (rules.numberOfActiveErrors > 0)
    return false;

  // Battery high voltage alarm
  if (rules.rule_outcome[Rule::BankOverVoltage])
    return false;
  // Low voltage alarm
  if (rules.rule_outcome[Rule::BankUnderVoltage])
    return false;
  // Battery high temperature alarm
  if (rules.rule_outcome[Rule::ModuleOverTemperatureExternal])
    return false;
  // Battery low temperature alarm
  if (rules.rule_outcome[Rule::ModuleUnderTemperatureExternal])
    return false;

  return true;
}
bool IsDischargeAllowed()
{
  if (_controller_state != ControllerState::Running)
    return false;

  if (mysettings.preventdischarge == true)
    return false;

  if (rules.moduleHasExternalTempSensor == false)
    return false;

  // Check battery temperature against charge/discharge parameters
  if (rules.lowestExternalTemp<mysettings.dischargetemplow | rules.highestExternalTemp> mysettings.dischargetemphigh)
  {
    // ESP_LOGW(TAG, "Stop discharge - temperature out of range");
    return false;
  }

  if (rules.lowestPackVoltage < mysettings.dischargevolt)
    return false;

  // Any errors, stop discharge
  if (rules.numberOfActiveErrors > 0)
    return false;

  // Battery high voltage alarm
  if (rules.rule_outcome[Rule::BankOverVoltage])
    return false;
  // Low voltage alarm
  if (rules.rule_outcome[Rule::BankUnderVoltage])
    return false;
  // Battery high temperature alarm
  if (rules.rule_outcome[Rule::ModuleOverTemperatureExternal])
    return false;
  // Battery low temperature alarm
  if (rules.rule_outcome[Rule::ModuleUnderTemperatureExternal])
    return false;

  return true;
}

// 0x351 – Battery voltage + current limits
void pylon_message_351()
{
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

  //  Defaults (do nothing)
  data.battery_charge_voltage = 0;
  data.battery_charge_current_limit = 0;
  data.battery_discharge_current_limit = 0;
  data.battery_discharge_voltage = mysettings.dischargevolt;

  if (IsChargeAllowed())
  {
    if (rules.numberOfBalancingModules > 0 && mysettings.stopchargebalance == true)
    {
      // Balancing is active, so stop charging
      data.battery_charge_voltage = 0;
      data.battery_charge_current_limit = 0;
    }
    else
    {
      // Default - normal behaviour
      data.battery_charge_voltage = mysettings.chargevolt;
      data.battery_charge_current_limit = mysettings.chargecurrent;
    }
  }

  if (IsDischargeAllowed())
  {
    data.battery_discharge_current_limit = mysettings.dischargecurrent;
  }

  send_canbus_message(0x351, (uint8_t *)&data, sizeof(data351));
}
// 0x355 – 1A 00 64 00 – State of Health (SOH) / State of Charge (SOC)
void pylon_message_355()
{
  if (_controller_state != ControllerState::Running)
    return;

  struct data355
  {
    uint16_t stateofchargevalue;
    uint16_t stateofhealthvalue;
  };

  if (mysettings.currentMonitoringEnabled && currentMonitor.validReadings && mysettings.currentMonitoringDevice == CurrentMonitorDevice::DIYBMS_CURRENT_MON)
  {
    data355 data;
    // 0 SOC value un16 1 %
    data.stateofchargevalue = currentMonitor.stateofcharge;

    if (mysettings.socoverride)
    {
      // Force SOC of 85% to the inverter, to force it to continue charging the battery
      // this is helpful when first commissioning as most inverters stop charging at 100% SOC
      // even though the battery may not be full, and the DIYBMS current monitor has not learnt capacity
      // yet.
      data.stateofchargevalue = 85;
    }

    //  2 SOH value un16 1 %
    data.stateofhealthvalue = 100;

    // Limit to 100% maximum
    if (data.stateofchargevalue > 100)
    {
      data.stateofchargevalue = 100;
    }

    send_canbus_message(0x355, (uint8_t *)&data, sizeof(data355));
  }
}

// 0x359 – 00 00 00 00 0A 50 4E – Protection & Alarm flags
void pylon_message_359()
{
  struct data359
  {
    // Protection - Table 1
    uint8_t byte0;
    // Protection - Table 2
    uint8_t byte1;
    // Warnings - Table
    uint8_t byte2;
    // Warnings - Table 4
    uint8_t byte3;
    // Quantity of packs in parall
    uint8_t byte4;
    uint8_t byte5;
    uint8_t byte6;
    // Online address of packs in parallel - Table 5
    uint8_t byte7;
  };

  data359 data;

  memset(&data, 0, sizeof(data359));

  if (_controller_state == ControllerState::Running)
  {
    // bit 0 = unused
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
    // bit 5 = unused
    // bit 6 = unused
    // bit 7 = Discharge over current

    // Byte2, Warnings - Table 3
    data.byte2 = 0;

    // WARNING:Battery high voltage
    if (rules.highestPackVoltage > mysettings.chargevolt)
    {
      data.byte2 |= B00000010;
    }

    // WARNING:Battery low voltage
    if (rules.lowestPackVoltage < mysettings.dischargevolt)
    {
      data.byte2 |= B00000100;
    }

    // WARNING: Battery high temperature
    if (rules.moduleHasExternalTempSensor && rules.highestExternalTemp > mysettings.chargetemphigh)
    {
      data.byte2 |= B00001000;
    }

    // WARNING: Battery low temperature
    if (rules.moduleHasExternalTempSensor && rules.lowestExternalTemp < mysettings.chargetemplow)
    {
      data.byte2 |= B00010000;
    }
  }

  // byte3,table4, Bit 3 = Internal communication failure
  data.byte3 |= ((rules.rule_outcome[Rule::BMSError] | rules.rule_outcome[Rule::EmergencyStop]) ? B00001000 : 0);
  data.byte3 |= ((_controller_state != ControllerState::Running) ? B00001000 : 0);

  if (mysettings.currentMonitoringEnabled && currentMonitor.validReadings)
  {
    // Pylon can have multiple battery each of 74Ah capacity, so emulate this based on total Ah capacity
    // this drives the inverter to assume certain charge/discharge parameters based on number of battery packs installed
    // Set inverter to use "Pylontech US3000C 3.5kWh" in its settings (these are 74Ah each)
    data.byte4 = min((uint8_t)1, (uint8_t)round(mysettings.nominalbatcap / 74.0));
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
    //uint8_t byte1;
  };

  data35c data;

  //bit 0/1/2/3 unused
  //bit 4 Force charge 2
  //bit 5 Force charge 1
  //bit 6 Discharge enable
  //bit 7 Charge enable
  data.byte0 = 0;
  //data.byte1 = 0;

  if (IsChargeAllowed())
  {
    data.byte0 = data.byte0 | B10000000;
  }

  if (IsDischargeAllowed())
  {
    data.byte0 = data.byte0 | B01000000;
  }

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

  // If current shunt is installed, use the voltage from that as it should be more accurate
  if (mysettings.currentMonitoringEnabled && currentMonitor.validReadings)
  {
    data.voltage = currentMonitor.modbus.voltage * 100.0;
    data.current = currentMonitor.modbus.current * 10;
  }
  else
  {
    // Use highest pack voltage calculated by controller and modules
    data.voltage = rules.highestPackVoltage / 10;
    data.current = 0;
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