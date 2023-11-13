/*
 ____  ____  _  _  ____  __  __  ___
(  _ \(_  _)( \/ )(  _ \(  \/  )/ __)
 )(_) )_)(_  \  /  ) _ < )    ( \__ \
(____/(____) (__) (____/(_/\/\_)(___/

  (c) 2023 Patrick Prasse

This code communicates emulates a PYLON FORCE BATTERY using CANBUS @ 500kbps and 29 bit addresses.

MOSTLY: https://onlineshop.gcsolar.co.za/wp-content/uploads/2021/07/CAN-Bus-Protocol-Sermatec-high-voltage-V1.1810kW.pdf
and https://www.eevblog.com/forum/programming/pylontech-sc0500-protocol-hacking/msg3742672/#msg3742672
and https://u.pcloud.link/publink/show?code=XZCKP5VZGOrK3QVaYLuy4XWqcwWvsJUUpO4y (README Growatt-Battery-BMS.pdf)
and Deye 2_CAN-Bus-Protocol-high-voltag-V1.17.pdf
*/

#define USE_ESP_IDF_LOG 1
static constexpr const char *const TAG = "diybms-pyforce";

#include "pylonforce_canbus.h"
#include "mqtt.h"

extern bool mqttClient_connected;
extern esp_mqtt_client_handle_t mqtt_client;

// we want packed structures so the compiler adds no padding
#pragma pack(push, 1)

// 0x7310+Addr - Versions
void pylonforce_message_7310()
{
  struct data7310
  {
    uint8_t hardware_version;
    uint8_t reserve1;
    uint8_t hardware_version_major;
    uint8_t hardware_version_minor;
    uint8_t software_version_major;
    uint8_t software_version_minor;
    uint8_t software_version_development_major;
    uint8_t software_version_development_minor;
  };

  data7310 data;
  memset(&data, 0, sizeof(data7310));
  // I don't know if we could actually send our git version bytes...
  data.hardware_version = 0x01;
  data.hardware_version_major = 0x02;
  data.hardware_version_minor = 0x01;
  data.software_version_major = 0x01;
  data.software_version_minor = 0x02;

  send_ext_canbus_message(0x7310+mysettings.canbus_equipment_addr, (uint8_t *)&data, sizeof(data7310));
}

// 0x7320+Addr - Module / cell quantities
void pylonforce_message_7320()
{
  struct data7320
  {
    uint16_t battery_series_cells;  // number of battery cells in series (over all modules/boxes)
    uint8_t battery_module_in_series_qty;  // number of battery modules (i.e. boxes of cell_qty_in_module) in series
    uint8_t cell_qty_in_module;  // number of series cells per module (boxes)
    uint16_t voltage_level;  // resolution 1V, offset 0V
    uint16_t ah_number;  // resolution 1Ah, offset 0V
  };

  data7320 data;
  memset(&data, 0, sizeof(data7320));

  data.battery_series_cells = mysettings.totalNumberOfSeriesModules;
  data.battery_module_in_series_qty = mysettings.totalNumberOfBanks;
  data.cell_qty_in_module = mysettings.totalNumberOfSeriesModules / data.battery_module_in_series_qty;
  data.voltage_level = (uint16_t)((uint32_t)mysettings.cellmaxmv * (uint32_t)mysettings.totalNumberOfSeriesModules / (uint32_t)1000);
  data.ah_number = mysettings.nominalbatcap;

  send_ext_canbus_message(0x7320+mysettings.canbus_equipment_addr, (uint8_t *)&data, sizeof(data7320));
}


// 0x7330+Addr / 0x7340+Addr - Transmit the DIYBMS hostname via two CAN Messages
// Sermatec PDF
// same as 0x42e0+Addr / 0x42f0+Addr in Deye
void pylonforce_message_7330_7340()
{
  char buffer[16+1];
  memset( buffer, 0, sizeof(buffer) );
  strncpy(buffer,hostname.c_str(),sizeof(buffer));
  send_ext_canbus_message(0x7330+mysettings.canbus_equipment_addr, (uint8_t *)&hostname, 8);
  vTaskDelay(pdMS_TO_TICKS(60));
  send_ext_canbus_message(0x7340+mysettings.canbus_equipment_addr, (uint8_t *)&hostname[8], 8);
}


// 0x4210+Addr - Total Voltage / total current / SOC / SOH
void pylonforce_message_4210()
{
  struct data4210
  {
    uint16_t voltage;  // Battery Pile Total Voltage, 100mV (0.1V) resolution
    uint16_t current;  // Battery Pile Current, 100mA (0.1A) resolution with offset 3000A
    uint16_t temperature; // second level BMS temperature, 0.1°C resolution, offset 100°C
    uint8_t stateofchargevalue;  // SOC, Resolution 1%, offset 0
    uint8_t stateofhealthvalue;  // SOH, Resolution 1%, offset 0
  };

  data4210 data;
  memset(&data, 0, sizeof(data4210));

  // If current shunt is installed, use the voltage from that as it should be more accurate
  if (mysettings.currentMonitoringEnabled && currentMonitor.validReadings)
  {
    data.voltage = currentMonitor.modbus.voltage * 10;
    data.current = (currentMonitor.modbus.current-3000) * 10;
  }
  else
  {
    // Use highest bank voltage calculated by controller and modules
    data.voltage = rules.highestBankVoltage / 100;
    data.current = 0;
  }


  // Temperature 0.1 C using external temperature sensor
  if (rules.moduleHasExternalTempSensor)
  {
    data.temperature = (uint16_t)max(0, (int16_t)(rules.highestExternalTemp + 100) * (int16_t)10);
  }
  else
  {
    // No external temp sensors
    data.temperature = 121+1000;  // 12.1 °C
  }

    // TODO: Need to determine this based on age of battery/cycles etc.
  data.stateofhealthvalue = 100;

  // Only send CANBUS message if we have a current monitor enabled & valid
  if (mysettings.currentMonitoringEnabled && currentMonitor.validReadings && (mysettings.currentMonitoringDevice == CurrentMonitorDevice::DIYBMS_CURRENT_MON_MODBUS || mysettings.currentMonitoringDevice == CurrentMonitorDevice::DIYBMS_CURRENT_MON_INTERNAL))
  {
    data.stateofchargevalue = rules.StateOfChargeWithRulesApplied(&mysettings, currentMonitor.stateofcharge);
  }
  else
  {
    data.stateofchargevalue = 50;
  }

  send_ext_canbus_message(0x4210+mysettings.canbus_equipment_addr, (uint8_t *)&data, sizeof(data4210));
}


// 0x4220+Addr - Battery cutoff voltages + current limits
void pylonforce_message_4220()
{
  struct data4220
  {
    uint16_t battery_charge_voltage; // Charge cutoff voltage, resolution 0.1V, offset 0
    uint16_t battery_discharge_voltage; // Discharge cutoff voltage, resolution 0.1V, offset 0
    
    // TODO: these two might be swapped, as stated in README Growatt-Battery-BMS.pdf
    uint16_t battery_charge_current_limit; // Max charge current, resolution 0.1A, offset 3000A  (therefore logically >=30000)
    uint16_t battery_discharge_current_limit; // Max discharge current (negative), resolution 0.1A, offset 3000A (therefore logically <=30000, as discharge current is negative Amps)
  };

  data4220 data;
  memset(&data, 0, sizeof(data4220));

  //  Defaults (do nothing)
  data.battery_charge_voltage = 0;
  data.battery_charge_current_limit = 30000;  // effective zero after applied offsets
  data.battery_discharge_current_limit = 30000; // effective zero after applied offsets
  data.battery_discharge_voltage = mysettings.dischargevolt;

  if (rules.IsChargeAllowed(&mysettings))
  {
    data.battery_charge_voltage = rules.DynamicChargeVoltage();
    data.battery_charge_current_limit = 30000 + (uint16_t)max((int16_t)0,rules.DynamicChargeCurrent());
  }
  else
  {
    ESP_LOGV(TAG, "Charging not allowed in message 4220");
  }

  if (rules.IsDischargeAllowed(&mysettings))
  {
    data.battery_discharge_current_limit = 30000 - (uint16_t)max((uint16_t)0,mysettings.dischargecurrent);
  }
  else
  {
    ESP_LOGV(TAG, "Discharging not allowed in message 4220");
  }

  send_ext_canbus_message(0x4220+mysettings.canbus_equipment_addr, (uint8_t *)&data, sizeof(data4220));
}


// 0x4230+Addr - Highest / lowest cell voltages
void pylonforce_message_4230()
{
  struct data4230
  {
    uint16_t max_single_battery_cell_voltage; // Voltage of the highest cell, resolution 0.001V
    uint16_t min_single_battery_cell_voltage; // Voltage of the lowest cell, resolution 0.001V
    uint16_t max_battery_cell_number; // Number of the highest voltage cell, 0 - X
    uint16_t min_battery_cell_number; // Number of the lowest voltage cell, 0 - X
  };

  data4230 data;
  memset(&data, 0, sizeof(data4230));

  data.max_single_battery_cell_voltage = rules.highestCellVoltage;
  data.min_single_battery_cell_voltage = rules.lowestCellVoltage;
  data.max_battery_cell_number = rules.address_HighestCellVoltage;
  data.min_battery_cell_number = rules.address_LowestCellVoltage;

  send_ext_canbus_message(0x4230+mysettings.canbus_equipment_addr, (uint8_t *)&data, sizeof(data4230));
}

// 0x4240+Addr - Highest / lowest cell temperatures
void pylonforce_message_4240()
{
  struct data4240
  {
    uint16_t max_single_battery_cell_temperature; // temperature of the highest cell, resolution 0.1°C, offset 100°C
    uint16_t min_single_battery_cell_temperature; // temperature of the lowest cell, resolution 0.1°C, offset 100°C
    uint16_t max_battery_cell_number; // Number of the highest temperature cell, 0 - X
    uint16_t min_battery_cell_number; // Number of the lowest temperature cell, 0 - X
  };

  data4240 data;
  memset(&data, 0, sizeof(data4240));

  if (rules.moduleHasExternalTempSensor)
  {
    data.max_single_battery_cell_temperature = (rules.highestExternalTemp+100)*10;
    data.min_single_battery_cell_temperature = (rules.lowestExternalTemp+100)*10;
    data.max_battery_cell_number = rules.address_highestExternalTemp;
    data.min_battery_cell_number = rules.address_lowestExternalTemp;
  }
  else
  {
    data.max_single_battery_cell_temperature = 110;  // 10°C
    data.min_single_battery_cell_temperature = 110;  // 0°C
    data.max_battery_cell_number = 1;
    data.min_battery_cell_number = 2;
  }

  send_ext_canbus_message(0x4240+mysettings.canbus_equipment_addr, (uint8_t *)&data, sizeof(data4240));
}


#define BASIC_STATUS_SLEEP 0
#define BASIC_STATUS_CHARGE 1
#define BASIC_STATUS_DISCHARGE 2
#define BASIC_STATUS_IDLE 3

// 0x4250+Addr - Status
void pylonforce_message_4250()
{
  struct data4250
  {
    // use uint8_t for bitfields as otherwise there may be issues with endian order (see C99 6.7.2.1-11)

    uint8_t basic_status_status : 3; // 0: sleep, 1: charge, 2: discharge, 3: idle, 4-7: reserved
    uint8_t basic_status_force_charge_request : 1;
    uint8_t basic_status_balance_charge_request : 1;
    uint8_t basic_status_reserve5 : 1;
    uint8_t basic_status_reserve6 : 1;
    uint8_t basic_status_reserve7 : 1;

    uint16_t cycle_period;

    uint8_t error_volt_sensor: 1; // voltage sensor error
    uint8_t error_tmpr: 1; // temperature sensor error
    uint8_t error_in_comm: 1; // internal communication error
    uint8_t error_dcov: 1; // input over voltage error
    uint8_t error_rv: 1; // input reversal error
    uint8_t error_relay: 1; // relay check error
    uint8_t error_damage: 1; // Deepl translation from chinese: "Battery damage malfunction (caused by battery overdischarge, etc.)"
    uint8_t error_other: 1; // Other error (deepl translation from chinese: "Other malfunctions (see malfunction extensions for details)")

    // datasheet says "告警 Alarm", translation from chinese: warning
    // if we set a bit here they are shown in Goodwe app PV Master under "BMS Status"
    // alarm_cht is shown as "Charge over-temp. 2" which I suppose is a alarm/error state
    uint8_t alarm_blv: 1; // single cell low voltage alarm
    uint8_t alarm_bhv: 1; // single cell high voltage alarm
    uint8_t alarm_plv: 1; // charge system low voltage alarm
    uint8_t alarm_phv: 1; // charge system high voltage alarm
    uint8_t alarm_clt: 1; // charge cell low temperature alarm
    uint8_t alarm_cht: 1; // charge cell high temperature alarm
    uint8_t alarm_dlt: 1; // discharge cell low temperature alarm
    uint8_t alarm_dht: 1; // discharge cell high temperature alarm
    uint8_t alarm_coca: 1; // charge over current alarm
    uint8_t alarm_doca: 1; // discharge over current alarm
    uint8_t alarm_mlv: 1; // module low voltage alarm
    uint8_t alarm_mhv: 1; // module high voltage alarm
    uint8_t alarm_reserve12: 1;
    uint8_t alarm_reserve13: 1;
    uint8_t alarm_reserve14: 1;
    uint8_t alarm_reserve15: 1;

    // datasheet says "保护 Protection", translation from chinese: safeguard
    // if we set a bit here they are shown in Goodwe app PV Master under "Battery warning"
    // protect_cht is shown as "Charge over-temp. 1" which I suppose is a pre-warning state
    uint8_t protect_blv: 1; // single cell low voltage protect
    uint8_t protect_bhv: 1; // single cell high voltage protect
    uint8_t protect_plv: 1; // charge system low voltage protect
    uint8_t protect_phv: 1; // charge system high voltage protect
    uint8_t protect_clt: 1; // charge cell low temperature protect
    uint8_t protect_cht: 1; // charge cell high temperature protect
    uint8_t protect_dlt: 1; // discharge cell low temperature protect
    uint8_t protect_dht: 1; // discharge cell high temperature protect
    uint8_t protect_coca: 1; // charge over current protect
    uint8_t protect_doca: 1; // discharge over current protect
    uint8_t protect_mlv: 1; // module low voltage protect
    uint8_t protect_mhv: 1; // module high voltage protect
    uint8_t protect_reserve12: 1;
    uint8_t protect_reserve13: 1;
    uint8_t protect_reserve14: 1;
    uint8_t protect_reserve15: 1;
  };

  data4250 data;
  memset(&data, 0, sizeof(data4250));

  if (_controller_state == ControllerState::Running)
  {
    if (mysettings.currentMonitoringEnabled && currentMonitor.validReadings)
    {
      data.basic_status_status = currentMonitor.modbus.current > 0 ? 
        BASIC_STATUS_CHARGE : 
        currentMonitor.modbus.current < 0 ? BASIC_STATUS_DISCHARGE : BASIC_STATUS_IDLE;
    }
    else
    {
      // we don't know because we have no current monitor
      data.basic_status_status = BASIC_STATUS_IDLE;
    }

    data.alarm_mhv = ((rules.ruleOutcome(Rule::BankOverVoltage) || rules.ruleOutcome(Rule::CurrentMonitorOverVoltage)) ? 1 : 0);
    data.alarm_mlv = ((rules.ruleOutcome(Rule::BankUnderVoltage) || rules.ruleOutcome(Rule::CurrentMonitorUnderVoltage)) ? 1 : 0);

    // TODO: maybe calculate from dynamic charge current?
    data.alarm_coca = rules.ruleOutcome(Rule::CurrentMonitorOverCurrentAmps) ? 1 : 0;
    data.alarm_doca = rules.ruleOutcome(Rule::CurrentMonitorOverCurrentAmps) ? 1 : 0;

    data.alarm_bhv = ((rules.ruleOutcome(Rule::ModuleOverVoltage)) ? 1 : 0);
    data.alarm_blv = ((rules.ruleOutcome(Rule::ModuleUnderVoltage)) ? 1 : 0);

    if (rules.moduleHasExternalTempSensor)
    {
      data.alarm_cht = (rules.ruleOutcome(Rule::ModuleOverTemperatureExternal) ? 1 : 0);
      data.alarm_clt = (rules.ruleOutcome(Rule::ModuleUnderTemperatureExternal) ? 1 : 0);
    }

    // charge system high voltage alarm
    if (rules.highestBankVoltage / 100 > mysettings.chargevolt)
    {
      data.alarm_phv = 1;
    }

    // charge system low voltage alarm
    if (rules.lowestBankVoltage / 100 < mysettings.dischargevolt)
    {
      data.alarm_plv = 1;
    }

    // charge cell high temperature alarm
    // discharge cell high temperature alarm
    if (rules.moduleHasExternalTempSensor && rules.highestExternalTemp > mysettings.chargetemphigh)
    {
      data.alarm_cht = 1;
      data.alarm_dht = 1;
    }

    // charge cell low temperature alarm
    // discharge cell low temperature alarm
    if (rules.moduleHasExternalTempSensor && rules.lowestExternalTemp < mysettings.chargetemplow)
    {
      data.alarm_clt = 1;
      data.alarm_dlt = 1;
    }
  }
  else
  {
    data.basic_status_status = BASIC_STATUS_SLEEP;
  }

//  data.error_in_comm = ((rules.ruleOutcome(Rule::BMSError) | rules.ruleOutcome(Rule::EmergencyStop)) ? 1 : 0);
  data.error_other = ((_controller_state != ControllerState::Running || 
      rules.ruleOutcome(Rule::BMSError) || 
      rules.ruleOutcome(Rule::EmergencyStop)) ? 1 : 0);
//  data.error_other = 1;
//  data.protect_plv = 1;
//  data.alarm_cht = 1;
//  data.protect_cht = 1;

  send_ext_canbus_message(0x4250+mysettings.canbus_equipment_addr, (uint8_t *)&data, sizeof(data4250));
}


// 0x4260+Addr - Highest / lowest module (diyBMS "bank") voltages
void pylonforce_message_4260()
{
  struct data4260
  {
    uint16_t max_single_battery_module_voltage; // Voltage of the highest module, resolution 0.001V
    uint16_t min_single_battery_module_voltage; // Voltage of the lowest module, resolution 0.001V
    uint16_t max_battery_module_number; // Number of the highest voltage module, 0 - X
    uint16_t min_battery_module_number; // Number of the lowest voltage module, 0 - X
  };

  data4260 data;
  memset(&data, 0, sizeof(data4260));

  data.max_single_battery_module_voltage = rules.highestBankVoltage;
  data.min_single_battery_module_voltage = rules.lowestBankVoltage;
  data.max_battery_module_number = rules.address_highestBankVoltage;
  data.min_battery_module_number = rules.address_lowestBankVoltage;

  send_ext_canbus_message(0x4260+mysettings.canbus_equipment_addr, (uint8_t *)&data, sizeof(data4260));
}


// 0x4270+Addr - Highest / lowest module (diyBMS "bank") temperatures
void pylonforce_message_4270()
{
  struct data4270
  {
    uint16_t max_single_battery_module_temperature; // temperature of the highest module, resolution 0.1°C, offset 100°C
    uint16_t min_single_battery_module_temperature; // temperature of the lowest module, resolution 0.1°C, offset 100°C
    uint16_t max_battery_module_number; // Number of the highest temperature module, 0 - X
    uint16_t min_battery_module_number; // Number of the lowest temperature module, 0 - X
  };

  data4270 data;
  memset(&data, 0, sizeof(data4270));

  if (rules.moduleHasExternalTempSensor)
  {
    data.max_single_battery_module_temperature = (rules.highestExternalTemp+100)*10;
    data.min_single_battery_module_temperature = (rules.lowestExternalTemp+100)*10;
    data.max_battery_module_number = 0; // TODO
    data.min_battery_module_number = 0; // TODO
  }
  else
  {
    data.max_single_battery_module_temperature = 1000+121;  // 12.1°C
    data.min_single_battery_module_temperature = 1000+121;  // 12.1°C
    data.max_battery_module_number = 0;
    data.min_battery_module_number = 0;
  }

  send_ext_canbus_message(0x4270+mysettings.canbus_equipment_addr, (uint8_t *)&data, sizeof(data4270));
}


// 0x4280+Addr - Status
void pylonforce_message_4280()
{
  struct data4280
  {
    uint8_t charge_forbidden_mark;
    uint8_t discharge_forbidden_mark;

    uint8_t reserve2;
    uint8_t reserve3;
    uint8_t reserve4;
    uint8_t reserve5;
    uint8_t reserve6;
    uint8_t reserve7;
  };

  data4280 data;
  memset(&data, 0, sizeof(data4280));

  if (_controller_state != ControllerState::Running || !rules.IsChargeAllowed(&mysettings))
  {
    data.charge_forbidden_mark = 0xAA;
  }

  if (_controller_state != ControllerState::Running || !rules.IsDischargeAllowed(&mysettings))
  {
    data.discharge_forbidden_mark = 0xAA;
  }

  send_ext_canbus_message(0x4280+mysettings.canbus_equipment_addr, (uint8_t *)&data, sizeof(data4280));
}


// 0x4290+Addr - Guessed: Startup faults
void pylonforce_message_4290()
{
  struct data4290
  {
    uint8_t fault_expansion_reserve7 : 1;
    uint8_t fault_expansion_reserve6 : 1;
    uint8_t fault_expansion_reserve5 : 1;
    uint8_t fault_expansion_abnormal_safety_functions : 1;
    uint8_t fault_expansion_abnormal_power_on_self_test : 1;
    uint8_t fault_expansion_abnormal_internal_bus : 1;
    uint8_t fault_expansion_abnormal_bmic : 1;
    uint8_t fault_expansion_abnormal_shutdown_circuit : 1;

    uint8_t reserve1;
    uint8_t reserve2;
    uint8_t reserve3;
    uint8_t reserve4;
    uint8_t reserve5;
    uint8_t reserve6;
    uint8_t reserve7;
  };

  data4290 data;
  memset(&data, 0, sizeof(data4290));

  send_ext_canbus_message(0x4290+mysettings.canbus_equipment_addr, (uint8_t *)&data, sizeof(data4290));
}

// 0x42e0+Addr / 0x42f0+Addr - Transmit the DIYBMS hostname via two CAN Messages
// Deye 2 CAN Bus Protocol V1.17
// seems to be also in SolArk
// the same as 0x7330+Addr / 0x7340+Addr in Sermatec HV
void pylonforce_message_42e0_42f0()
{
  char buffer[16+1];
  memset( buffer, 0, sizeof(buffer) );
  strncpy(buffer,hostname.c_str(),sizeof(buffer));
  send_ext_canbus_message(0x42e0+mysettings.canbus_equipment_addr, (uint8_t *)&buffer, 8);
  vTaskDelay(pdMS_TO_TICKS(60));
  send_ext_canbus_message(0x42f0+mysettings.canbus_equipment_addr, (uint8_t *)&buffer[8], 8);
}




// have we seen the ensemble_information message from inverter?
bool seen_ensemble_information = false;

// have we seen the identify message from inverter?
bool seen_identify_message = false;

// is this the first call of handle_tx?
bool first_handle_tx = true;

void pylonforce_handle_tx()
{
  ESP_LOGV(TAG, "pylonforce_handle_tx\n");

  if( seen_identify_message || first_handle_tx )
  {
    ESP_LOGV(TAG, "seen_identify_message\n");
    vTaskDelay(pdMS_TO_TICKS(60));
    pylonforce_message_7310();
    vTaskDelay(pdMS_TO_TICKS(60));
    pylonforce_message_7320();
    vTaskDelay(pdMS_TO_TICKS(60));
    pylonforce_message_7330_7340();
    
    seen_identify_message = false; // message answered
  }
  // no else here
  if( seen_ensemble_information || first_handle_tx )
  {
    ESP_LOGV(TAG, "seen_ensemble_information\n");
    vTaskDelay(pdMS_TO_TICKS(60));
    pylonforce_message_4210();
    vTaskDelay(pdMS_TO_TICKS(60));
    pylonforce_message_4220();
    vTaskDelay(pdMS_TO_TICKS(60));
    pylonforce_message_4230();
    vTaskDelay(pdMS_TO_TICKS(60));
    pylonforce_message_4240();
    vTaskDelay(pdMS_TO_TICKS(60));
    pylonforce_message_4250();
    vTaskDelay(pdMS_TO_TICKS(60));
    pylonforce_message_4260();
    vTaskDelay(pdMS_TO_TICKS(60));
    pylonforce_message_4270();
    vTaskDelay(pdMS_TO_TICKS(60));
    pylonforce_message_4280();
    vTaskDelay(pdMS_TO_TICKS(60));
    pylonforce_message_4290();
    vTaskDelay(pdMS_TO_TICKS(60));
    pylonforce_message_42e0_42f0();

    seen_ensemble_information = false; // message answered
  }

  first_handle_tx = false;
}

void pylonforce_handle_rx(twai_message_t *message)
{
  if( !(message->flags & TWAI_MSG_FLAG_EXTD) )  // no 29bit addresses, can not be for us
    return;
  if( message->identifier == 0x4200 )
  {
    if( message->data[0] == 0x02 )  // question from inverter: system equipment information
    {
      seen_identify_message = true;
    }
    else if( message->data[0] == 0x00 )  // question from inverter: ensemble information
    {
      seen_ensemble_information = true;
    }
  }
  else if( message->identifier == (0x8200+mysettings.canbus_equipment_addr) )
  {
    // Sleep / Awake Command
    // diyBMS is always awake
    // no reply

    // byte0 == 0x55: Control device enter sleep status;
    // byte0 == 0xAA: Control device quit sleep status;
    // Others: Null
  }
  else if( message->identifier == (0x8210+mysettings.canbus_equipment_addr) )
  {
    // Charge/Discharge Command
    // diyBMS relay control is scope of rules, not CAN
    // no reply

    /* From documentation:
*Note:
1. Charge Command: When the battery is in under-voltage protection, the relay is open. When
EMS or PCS is going to charge the battery, send this command, then the battery will close
the main relay. If the battery is in sleep status, wake up first then use this command.
2. Discharge Command: When the battery is in over-voltage protection, the relay is open.
When EMS or PCS is going to discharge the battery, send this command, then the battery
will close the main relay. If the battery is in sleep status, wake up first then use this
command.
     */
    
    // byte0 == 0xAA: effect; Others: Null (* Note 1)
    // byte1 == 0xAA: effect; Others: Null (* Note 2)
  }
  else if( message->identifier == (0x8240+mysettings.canbus_equipment_addr) )
  {
    // Temporary masking "external communication error" command

    /* From documentation:
Note:
After receive this command, BMS will estimate the condition and give reply.
If meet the condition, in 5 minutes, BMS will ignore the “external communication fail” alarm, which
means relay will keep ON while no communication between BMS and EMS/PCS.
In this 5 minutes, if there is a protection alarm, BMS will cut off the relay as normal
     */
    

    uint8_t data[8];
    memset(&data, 0, sizeof(data));

    // TODO: reply: OK, will act this command immediately
    //data[0] = 0xAA;

    // reply: won`t act this command
    data[0] = 0x00;

    send_ext_canbus_message(0x8250+mysettings.canbus_equipment_addr, (uint8_t *)&data, sizeof(data));
  }
}

#pragma pack(pop)

