/*
 ____  ____  _  _  ____  __  __  ___
(  _ \(_  _)( \/ )(  _ \(  \/  )/ __)
 )(_) )_)(_  \  /  ) _ < )    ( \__ \
(____/(____) (__) (____/(_/\/\_)(___/

  (c) 2021 Stuart Pittaway

This code communicates with VICTRON CERBO GX style devices using CANBUS @ 500kbps and 11 bit addresses.

The code supports the VICTRON CAN BUS BMS style messages.

*/

#define USE_ESP_IDF_LOG 1
static constexpr const char * const TAG = "diybms-victron";

#include "victron_canbus.h"

void send_canbus_message(uint32_t identifier, uint8_t *buffer, uint8_t length)
{
  //static const char *TAG = "canbus";

  twai_message_t message;
  message.identifier = identifier;
  message.flags = TWAI_MSG_FLAG_NONE;
  message.data_length_code = length;

  memcpy(&message.data, buffer, length);

  // Queue message for transmission
  if (twai_transmit(&message, pdMS_TO_TICKS(250)) != ESP_OK)
  {
    ESP_LOGE(TAG, "Fail to queue message");
    canbus_messages_failed_sent++;
  }

  else
  {
    // ESP_LOGD(TAG, "Sent CAN message %u", identifier);
    // ESP_LOG_BUFFER_HEX_LEVEL(TAG, &message, sizeof(can_message_t), esp_log_level_t::ESP_LOG_DEBUG);
    canbus_messages_sent++;
  }
}
// Transmit the DIYBMS hostname via two CAN Messages
void victron_message_370_371()
{
  send_canbus_message(0x370, (uint8_t *)&hostname, 8);
  send_canbus_message(0x371, (uint8_t *)&hostname[8], 8);
}

void victron_message_35e()
{
  send_canbus_message(0x35e, (uint8_t *)&hostname, 6);
}

void victron_message_35f()
{
  struct data35f
  {
    // Not used
    uint16_t BatteryModel;
    uint16_t Firmwareversion;
    uint16_t OnlinecapacityinAh;
  };

  data35f data;

  // Not used by Victron
  data.BatteryModel = 0;
  // Need to swap bytes for this to make sense.
  data.Firmwareversion = ((uint16_t)COMPILE_WEEK_NUMBER_BYTE << 8) | COMPILE_YEAR_BYTE;

  if (mysettings.currentMonitoringEnabled && mysettings.currentMonitoringDevice == CurrentMonitorDevice::DIYBMS_CURRENT_MON)
  {
    data.OnlinecapacityinAh = currentMonitor.modbus.batterycapacityamphour;
  }
  else
  {
    data.OnlinecapacityinAh = 0;
  }

  send_canbus_message(0x35f, (uint8_t *)&data, sizeof(data35f));
}

void SetBankAndModuleText(char *buffer, uint8_t cellid)
{
  uint8_t bank = cellid / mysettings.totalNumberOfSeriesModules;
  uint8_t module = cellid - (bank * mysettings.totalNumberOfSeriesModules);

  // Clear all 8 bytes
  memset(buffer, 0, 8);

  snprintf(buffer, 8, "b%d m%d", bank, module);
}

void victron_message_374_375_376_377()
{
  struct candata
  {
    char text[8];
  };

  candata data;

  if (rules.address_LowestCellVoltage < maximum_controller_cell_modules)
  {
    SetBankAndModuleText(data.text, rules.address_LowestCellVoltage);
    // Min. cell voltage id string [1]
    send_canbus_message(0x374, (uint8_t *)&data, sizeof(candata));
  }

  if (rules.address_HighestCellVoltage < maximum_controller_cell_modules)
  {
    SetBankAndModuleText(data.text, rules.address_HighestCellVoltage);
    // Max. cell voltage id string [1]
    send_canbus_message(0x375, (uint8_t *)&data, sizeof(candata));
  }

  if (rules.address_lowestExternalTemp < maximum_controller_cell_modules)
  {
    SetBankAndModuleText(data.text, rules.address_lowestExternalTemp);
    // Min. cell voltage id string [1]
    send_canbus_message(0x376, (uint8_t *)&data, sizeof(candata));
  }

  if (rules.address_highestExternalTemp < maximum_controller_cell_modules)
  {
    SetBankAndModuleText(data.text, rules.address_highestExternalTemp);
    // Min. cell voltage id string [1]
    send_canbus_message(0x377, (uint8_t *)&data, sizeof(candata));
  }
}

// CVL & CCL implementation
/*
At a fundamental level, Victron's approach to interacting with BMSes is in the form of 3 setpoint parameters:
* Charge Current Limit (CCL)
* Charge Voltage Limit (CVL)
* Discharge Current Limit (DCL)
Once the BMS is communicating with the GX device over CAN-bus, these parameters are listed in the "Parameters" menu in the battery device on the Remote Console.

All three parameter, but at least the CVL (Charge Voltage Limit) and CCL (Charge Current Limit), are sent by the BMS to the Victron system.
These are the only parameters that control the charging behaviour.

Victron's approach to BMSes and chargers is focused DC-Coupled solar systems where there can be multiple
charging sources, usually a combination of MPPT solar charger and charger/inverters.

AC-Coupled solar systems (non-Victron)

Typically, BMSes designed for the common AC-Coupled Energy Storage Type systems hardcode a
CVL, which remains the same in all situations. And will reduce CCL to 0, or near-zero, once the battery
is fully charged, and/or one of the cells is fully charged.
That strategy does not work with a Victron system.
*/
void victron_message_351()
{

  uint8_t number_of_active_errors = 0;

  if (_controller_state == ControllerState::Running)
  {
    number_of_active_errors += (rules.rule_outcome[Rule::BankOverVoltage] ? 1 : 0);
    //(bit 4+5) Battery high voltage alarm
    number_of_active_errors += (rules.rule_outcome[Rule::BankUnderVoltage] ? 1 : 0);
    //(bit 6+7) Battery high temperature alarm
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
    // CVL
    uint16_t chargevoltagelimit;
    // CCL
    int16_t maxchargecurrent;
    // DCL
    int16_t maxdischargecurrent;
    // Not currently used by Victron
    // uint16_t dischargevoltage;
  };

  data351 data;

  if ((_controller_state != ControllerState::Running) || (number_of_active_errors > 0))
  {
    ESP_LOGW(TAG, "active_errors=%u", number_of_active_errors);
    // Error condition
    data.chargevoltagelimit = mysettings.cvl[VictronDVCC::ControllerError];
    data.maxchargecurrent = mysettings.ccl[VictronDVCC::ControllerError];
    data.maxdischargecurrent = mysettings.dcl[VictronDVCC::ControllerError];
    // data.dischargevoltage = 0;
  }
  else if (rules.numberOfBalancingModules > 0)
  {
    // Balancing
    data.chargevoltagelimit = mysettings.cvl[VictronDVCC::Balance];
    data.maxchargecurrent = mysettings.ccl[VictronDVCC::Balance];
    data.maxdischargecurrent = mysettings.dcl[VictronDVCC::Balance];
    // data.dischargevoltage = 0;
  }
  else
  {
    // Default - normal behaviour
    data.chargevoltagelimit = mysettings.cvl[VictronDVCC::Default];
    data.maxchargecurrent = mysettings.ccl[VictronDVCC::Default];
    data.maxdischargecurrent = mysettings.dcl[VictronDVCC::Default];
    // data.dischargevoltage = 0;
  }

  send_canbus_message(0x351, (uint8_t *)&data, sizeof(data351));
}

// S.o.C value
void victron_message_355()
{
  struct data355
  {
    uint16_t stateofchargevalue;
    // uint16_t stateofhealthvalue;
    // uint16_t highresolutionsoc;
  };

  if (_controller_state == ControllerState::Running && mysettings.currentMonitoringEnabled && currentMonitor.validReadings && mysettings.currentMonitoringDevice == CurrentMonitorDevice::DIYBMS_CURRENT_MON)
  {
    data355 data;
    // 0 SOC value un16 1 %
    data.stateofchargevalue = currentMonitor.stateofcharge;
    // 2 SOH value un16 1 %
    // data.stateofhealthvalue = 100;

    send_canbus_message(0x355, (uint8_t *)&data, sizeof(data355));
  }
}

// Battery voltage
void victron_message_356()
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

// Send alarm details to Victron over CANBUS
void victron_message_35a()
{

  struct data35a
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

  data35a data;

  memset(&data, 0, sizeof(data35a));

  // B00 = Alarm not supported
  // B10 = Alarm/warning active
  // B01 = Alarm/warning inactive (status = OK)

  // These constants are actually bit swapped compared to the notes above
  // as the Victron kit uses little-endian ordering when transmitting on CANBUS
  const uint8_t BIT01_ALARM = B00000001;
  const uint8_t BIT23_ALARM = B00000100;
  const uint8_t BIT45_ALARM = B00010000;
  const uint8_t BIT67_ALARM = B01000000;

  const uint8_t BIT01_OK = B00000010;
  const uint8_t BIT23_OK = B00001000;
  const uint8_t BIT45_OK = B00100000;
  const uint8_t BIT67_OK = B10000000;

  // const uint8_t BIT01_NOTSUP = B00000011;
  // const uint8_t BIT23_NOTSUP = B00001100;
  // const uint8_t BIT45_NOTSUP = B00110000;
  // const uint8_t BIT67_NOTSUP = B11000000;

  if (_controller_state == ControllerState::Running)
  {
    /*
    ESP_LOGI(TAG, "Rule PackOverVoltage=%u, PackUnderVoltage=%u, OverTemp=%u, UnderTemp=%u",
             rules.rule_outcome[Rule::PackOverVoltage],
             rules.rule_outcome[Rule::PackUnderVoltage],
             rules.rule_outcome[Rule::IndividualcellovertemperatureExternal],
             rules.rule_outcome[Rule::IndividualcellundertemperatureExternal]);
  */

    // BYTE 0
    //(bit 0+1) General alarm (not implemented)
    //(bit 2+3) Battery low voltage alarm
    data.byte0 |= ((rules.rule_outcome[Rule::BankOverVoltage] | rules.rule_outcome[Rule::CurrentMonitorOverVoltage]) ? BIT23_ALARM : BIT23_OK);
    //(bit 4+5) Battery high voltage alarm
    data.byte0 |= ((rules.rule_outcome[Rule::BankUnderVoltage] | rules.rule_outcome[Rule::CurrentMonitorUnderVoltage]) ? BIT45_ALARM : BIT45_OK);

    //(bit 6+7) Battery high temperature alarm
    if (rules.moduleHasExternalTempSensor)
    {
      data.byte0 |= (rules.rule_outcome[Rule::ModuleOverTemperatureExternal] ? BIT67_ALARM : BIT67_OK);
    }

    // BYTE 1
    // 1 (bit 0+1) Battery low temperature alarm
    if (rules.moduleHasExternalTempSensor)
    {
      data.byte1 |= (rules.rule_outcome[Rule::ModuleUnderTemperatureExternal] ? BIT01_ALARM : BIT01_OK);
    }
    // 1 (bit 2+3) Battery high temperature charge alarm
    // data.byte1 |= BIT23_NOTSUP;
    // 1 (bit 4+5) Battery low temperature charge alarm
    // data.byte1 |= BIT45_NOTSUP;
    // 1 (bit 6+7) Battery high current alarm
    // data.byte1 |= BIT67_NOTSUP;
  }

  // 2 (bit 0+1) Battery high charge current alarm
  // data.byte2 |= BIT01_NOTSUP;
  // 2 (bit 2+3) Contactor Alarm (not implemented)
  // data.byte2 |= BIT23_NOTSUP;
  // 2 (bit 4+5) Short circuit Alarm (not implemented)
  // data.byte2 |= BIT45_NOTSUP;

  // ESP_LOGI(TAG, "Rule BMSError=%u, EmergencyStop=%u", rules.rule_outcome[Rule::BMSError], rules.rule_outcome[Rule::EmergencyStop]);

  // 2 (bit 6+7) BMS internal alarm
  data.byte2 |= ((rules.rule_outcome[Rule::BMSError] | rules.rule_outcome[Rule::EmergencyStop]) ? BIT67_ALARM : BIT67_OK);

  // 3 (bit 0+1) Cell imbalance alarm
  // data.byte3 |= BIT01_NOTSUP;
  // 3 (bit 2+3) Reserved
  // 3 (bit 4+5) Reserved
  // 3 (bit 6+7) Reserved

  // 4 (bit 0+1) General warning (not implemented)
  // data.byte4 |= BIT01_NOTSUP;
  // 4 (bit 2+3) Battery low voltage warning
  // data.byte4 |= BIT23_NOTSUP;
  // 4 (bit 4+5) Battery high voltage warning
  // data.byte4 |= BIT45_NOTSUP;
  // 4 (bit 6+7) Battery high temperature warning
  // data.byte4 |= BIT67_NOTSUP;

  // 5 (bit 0+1) Battery low temperature warning
  // data.byte5 |= BIT01_NOTSUP;
  // 5 (bit 2+3) Battery high temperature charge warning
  // data.byte5 |= BIT23_NOTSUP;
  // 5 (bit 4+5) Battery low temperature charge warning
  // data.byte5 |= BIT45_NOTSUP;
  // 5 (bit 6+7) Battery high current warning
  // data.byte5 |= BIT67_NOTSUP;

  // 6 (bit 0+1) Battery high charge current warning
  // data.byte6 |= BIT01_NOTSUP;
  // 6 (bit 2+3) Contactor warning (not implemented)
  // data.byte6 |= BIT23_NOTSUP;
  // 6 (bit 4+5) Short circuit warning (not implemented)
  // data.byte6 |= BIT45_NOTSUP;
  // 6 (bit 6+7) BMS internal warning
  // data.byte6 |= (rules.numberOfActiveWarnings > 0 ? BIT67_ALARM : BIT67_OK);

  // ESP_LOGI(TAG, "numberOfBalancingModules=%u", rules.numberOfBalancingModules);

  // 7 (bit 0+1) Cell imbalance warning
  data.byte7 |= (rules.numberOfBalancingModules > 0 ? BIT01_ALARM : BIT01_OK);
  // 7 (bit 2+3) System status (online/offline) [1]
  data.byte7 |= ((_controller_state != ControllerState::Running) ? BIT23_ALARM : BIT23_OK);
  // 7 (rest) Reserved

  send_canbus_message(0x35a, (uint8_t *)&data, sizeof(data35a));
}

void victron_message_372()
{
  struct data372
  {
    uint16_t numberofmodulesok;
    // uint16_t numberofmodulesblockingcharge;
    // uint16_t numberofmodulesblockingdischarge;
    // uint16_t numberofmodulesoffline;
  };

  data372 data;

  data.numberofmodulesok = TotalNumberOfCells() - rules.invalidModuleCount;
  // data.numberofmodulesblockingcharge = 0;
  // data.numberofmodulesblockingdischarge = 0;
  // data.numberofmodulesoffline = rules.invalidModuleCount;

  send_canbus_message(0x372, (uint8_t *)&data, sizeof(data372));
}

void victron_message_373()
{

  struct data373
  {
    uint16_t mincellvoltage;
    uint16_t maxcellvoltage;
    uint16_t lowestcelltemperature;
    uint16_t highestcelltemperature;
  };

  data373 data;

  data.lowestcelltemperature = 273 + rules.lowestExternalTemp;
  data.highestcelltemperature = 273 + rules.highestExternalTemp;
  data.maxcellvoltage = rules.highestCellVoltage;
  data.mincellvoltage = rules.lowestCellVoltage;

  send_canbus_message(0x373, (uint8_t *)&data, sizeof(data373));
}