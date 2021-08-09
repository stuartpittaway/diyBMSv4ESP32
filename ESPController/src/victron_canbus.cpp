#include "victron_canbus.h"
/*
 ____  ____  _  _  ____  __  __  ___ 
(  _ \(_  _)( \/ )(  _ \(  \/  )/ __)
 )(_) )_)(_  \  /  ) _ < )    ( \__ \
(____/(____) (__) (____/(_/\/\_)(___/

  (c) 2021 Stuart Pittaway

This code communicates with VICTRON CERBO GX style devices using CANBUS @ 500kbps and 11 bit addresses.

The code supports the VICTRON CAN BUS BMS style messages.

*/

//Transmit the DIYBMS hostname via two CAN Messages
void victron_message_370_371()
{

  can_message_t message;
  message.identifier = 0x370;
  message.flags = CAN_MSG_FLAG_NONE;
  message.data_length_code = CAN_MAX_DATA_LEN;

  memcpy(&message.data, &hostname, CAN_MAX_DATA_LEN);

  //Queue message for transmission
  if (can_transmit(&message, pdMS_TO_TICKS(200)) != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to queue message for transmission");
  }

  message.identifier = 0x371;
  message.flags = CAN_MSG_FLAG_NONE;
  message.data_length_code = CAN_MAX_DATA_LEN;

  memcpy(&message.data, &hostname[CAN_MAX_DATA_LEN], CAN_MAX_DATA_LEN);

  //Queue message for transmission
  if (can_transmit(&message, pdMS_TO_TICKS(200)) != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to queue message for transmission");
  }
}

void victron_message_35e()
{
  can_message_t message;
  message.identifier = 0x35E;
  message.flags = CAN_MSG_FLAG_NONE;
  message.data_length_code = 6;

  //Copy first 8 bytes of hostname
  memcpy(&message.data, &hostname, CAN_MAX_DATA_LEN);

  //Queue message for transmission
  if (can_transmit(&message, pdMS_TO_TICKS(200)) != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to queue message for transmission");
  }
}

//CVL & CCL implementation
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
  struct data351
  {
    //CVL
    uint16_t chargevoltagelimit;
    //CCL
    int16_t maxchargecurrent;
    //DCL
    int16_t maxdischargecurrent;
    //Not currently used by Victron
    //uint16_t dischargevoltage;
  };

  can_message_t message;
  message.identifier = 0x351;
  message.flags = CAN_MSG_FLAG_NONE;
  message.data_length_code = sizeof(data351);

  data351 data;

  data.chargevoltagelimit = 0;
  data.maxchargecurrent = 0;
  data.maxdischargecurrent = 0;
  //data.dischargevoltage = 0;

  memcpy(&message.data, &data, sizeof(data351));

  //Queue message for transmission
  if (can_transmit(&message, pdMS_TO_TICKS(200)) != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to queue message for transmission");
  }
}

void victron_message_355()
{

  struct data355
  {
    uint16_t stateofchargevalue;
    //uint16_t stateofhealthvalue;
    //uint16_t highresolutionsoc;
  };

  can_message_t message;
  message.identifier = 0x355;
  message.flags = CAN_MSG_FLAG_NONE;
  message.data_length_code = sizeof(data355);

  data355 data;
  //0 SOC value un16 1 %
  data.stateofchargevalue = 100;
  //2 SOH value un16 1 %
  //data.stateofhealthvalue = 100;

  memcpy(&message.data, &data, sizeof(data355));

  //Queue message for transmission
  if (can_transmit(&message, pdMS_TO_TICKS(200)) != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to queue message for transmission");
  }
}

void victron_message_356()
{
  can_message_t message;
  message.identifier = 0x356;
  message.flags = CAN_MSG_FLAG_NONE;
  message.data_length_code = 6;

  //Clear buffer
  memset(&message.data, 0, CAN_MAX_DATA_LEN);

  int16_t output = rules.highestPackVoltage / 10;

  //If current shunt is installed, use the voltage from that as it should be more accurate
  if (mysettings.currentMonitoringEnabled && currentMonitor.validReadings)
  {
    output = currentMonitor.voltage * 100.0;
  }

  //ESP_LOGI(TAG, "Voltage %i", output);

  //Battery Voltage 0.01V scale
  memcpy(&message.data[0], &output, 2);

  //If current shunt is installed, use the voltage from that as it should be more accurate
  if (mysettings.currentMonitoringEnabled && currentMonitor.validReadings)
  {
    int16_t amps = currentMonitor.current * 10;

    //Current 0.1 Amp scale
    memcpy(&message.data[2], &amps, 2);
  }

  //Temperature 0.1 C
  int16_t temperature = (int16_t)rules.highestExternalTemp * (int16_t)10;
  memcpy(&message.data[4], &temperature, 2);

  //Queue message for transmission
  if (can_transmit(&message, pdMS_TO_TICKS(200)) != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to queue message for transmission");
  }
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

  can_message_t message;
  message.identifier = 0x35a;
  message.flags = CAN_MSG_FLAG_NONE;
  message.data_length_code = 8;

  data35a data;

  memset(&data, 0, sizeof(data35a));

  //B00 = Alarm not supported
  //B10 = Alarm/warning active
  //B01 = Alarm/warning inactive (status = OK)

  const uint8_t BIT01_ALARM = B00000001;
  const uint8_t BIT23_ALARM = B00000100;
  const uint8_t BIT45_ALARM = B00010000;
  const uint8_t BIT67_ALARM = B01000000;

  const uint8_t BIT01_OK = B00000010;
  const uint8_t BIT23_OK = B00001000;
  const uint8_t BIT45_OK = B00100000;
  const uint8_t BIT67_OK = B10000000;

  //const uint8_t BIT01_NOTSUP = B00000011;
  //const uint8_t BIT23_NOTSUP = B00001100;
  //const uint8_t BIT45_NOTSUP = B00110000;
  //const uint8_t BIT67_NOTSUP = B11000000;

  if (_controller_state == ControllerState::Running)
  {
    /*
    ESP_LOGI(TAG, "Rule PackOverVoltage=%u, PackUnderVoltage=%u, OverTemp=%u, UnderTemp=%u",
             rules.rule_outcome[Rule::PackOverVoltage],
             rules.rule_outcome[Rule::PackUnderVoltage],
             rules.rule_outcome[Rule::IndividualcellovertemperatureExternal],
             rules.rule_outcome[Rule::IndividualcellundertemperatureExternal]);
  */

    //BYTE 0
    //(bit 0+1) General alarm (not implemented)
    //(bit 2+3) Battery low voltage alarm
    data.byte0 |= (rules.rule_outcome[Rule::PackOverVoltage] ? BIT23_ALARM : BIT23_OK);
    //(bit 4+5) Battery high voltage alarm
    data.byte0 |= (rules.rule_outcome[Rule::PackUnderVoltage] ? BIT45_ALARM : BIT45_OK);
    //(bit 6+7) Battery high temperature alarm
    if (rules.moduleHasExternalTempSensor)
    {
      data.byte0 |= (rules.rule_outcome[Rule::IndividualcellovertemperatureExternal] ? BIT67_ALARM : BIT67_OK);
    }

    //BYTE 1
    //1 (bit 0+1) Battery low temperature alarm
    if (rules.moduleHasExternalTempSensor)
    {
      data.byte1 |= (rules.rule_outcome[Rule::IndividualcellundertemperatureExternal] ? BIT01_ALARM : BIT01_OK);
    }
    //1 (bit 2+3) Battery high temperature charge alarm
    //data.byte1 |= BIT23_NOTSUP;
    //1 (bit 4+5) Battery low temperature charge alarm
    //data.byte1 |= BIT45_NOTSUP;
    //1 (bit 6+7) Battery high current alarm
    //data.byte1 |= BIT67_NOTSUP;
  }

  //2 (bit 0+1) Battery high charge current alarm
  //data.byte2 |= BIT01_NOTSUP;
  //2 (bit 2+3) Contactor Alarm (not implemented)
  //data.byte2 |= BIT23_NOTSUP;
  //2 (bit 4+5) Short circuit Alarm (not implemented)
  //data.byte2 |= BIT45_NOTSUP;

  //ESP_LOGI(TAG, "Rule BMSError=%u, EmergencyStop=%u", rules.rule_outcome[Rule::BMSError], rules.rule_outcome[Rule::EmergencyStop]);

  //2 (bit 6+7) BMS internal alarm
  data.byte2 |= ((rules.rule_outcome[Rule::BMSError] | rules.rule_outcome[Rule::EmergencyStop]) ? BIT67_ALARM : BIT67_OK);

  //3 (bit 0+1) Cell imbalance alarm
  //data.byte3 |= BIT01_NOTSUP;
  //3 (bit 2+3) Reserved
  //3 (bit 4+5) Reserved
  //3 (bit 6+7) Reserved

  //4 (bit 0+1) General warning (not implemented)
  //data.byte4 |= BIT01_NOTSUP;
  //4 (bit 2+3) Battery low voltage warning
  //data.byte4 |= BIT23_NOTSUP;
  //4 (bit 4+5) Battery high voltage warning
  //data.byte4 |= BIT45_NOTSUP;
  //4 (bit 6+7) Battery high temperature warning
  //data.byte4 |= BIT67_NOTSUP;

  //5 (bit 0+1) Battery low temperature warning
  //data.byte5 |= BIT01_NOTSUP;
  //5 (bit 2+3) Battery high temperature charge warning
  //data.byte5 |= BIT23_NOTSUP;
  //5 (bit 4+5) Battery low temperature charge warning
  //data.byte5 |= BIT45_NOTSUP;
  //5 (bit 6+7) Battery high current warning
  //data.byte5 |= BIT67_NOTSUP;

  //6 (bit 0+1) Battery high charge current warning
  //data.byte6 |= BIT01_NOTSUP;
  //6 (bit 2+3) Contactor warning (not implemented)
  //data.byte6 |= BIT23_NOTSUP;
  //6 (bit 4+5) Short circuit warning (not implemented)
  //data.byte6 |= BIT45_NOTSUP;
  //6 (bit 6+7) BMS internal warning
  data.byte6 |= (rules.numberOfActiveWarnings > 0 ? BIT67_ALARM : BIT67_OK);

  //ESP_LOGI(TAG, "numberOfBalancingModules=%u", rules.numberOfBalancingModules);

  //7 (bit 0+1) Cell imbalance warning
  data.byte7 |= (rules.numberOfBalancingModules > 0 ? BIT01_ALARM : BIT01_OK);
  //7 (bit 2+3) System status (online/offline) [1]
  data.byte7 |= ((_controller_state != ControllerState::Running) ? BIT23_ALARM : BIT23_OK);
  //7 (rest) Reserved

  memcpy(&message.data, &data, sizeof(data35a));

  //Queue message for transmission
  if (can_transmit(&message, pdMS_TO_TICKS(200)) != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to queue message for transmission");
  }
}

void victron_message_372()
{
  struct data372
  {
    uint16_t numberofmodulesok;
    //uint16_t numberofmodulesblockingcharge;
    //uint16_t numberofmodulesblockingdischarge;
    //uint16_t numberofmodulesoffline;
  };

  can_message_t message;
  message.identifier = 0x372;
  message.flags = CAN_MSG_FLAG_NONE;
  message.data_length_code = sizeof(data372);

  data372 data;

  data.numberofmodulesok = TotalNumberOfCells() - rules.invalidModuleCount;
  //data.numberofmodulesblockingcharge = 0;
  //data.numberofmodulesblockingdischarge = 0;
  //data.numberofmodulesoffline = rules.invalidModuleCount;

  memcpy(&message.data, &data, sizeof(data372));

  //Queue message for transmission
  if (can_transmit(&message, pdMS_TO_TICKS(200)) != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to queue message for transmission");
  }
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

  can_message_t message;
  message.identifier = 0x373;
  message.flags = CAN_MSG_FLAG_NONE;
  message.data_length_code = 8;

  data373 data;

  data.lowestcelltemperature = 273 + rules.lowestExternalTemp;
  data.highestcelltemperature = 273 + rules.highestExternalTemp;
  data.maxcellvoltage = rules.highestCellVoltage;
  data.mincellvoltage = rules.lowestCellVoltage;

  memcpy(&message.data, &data, CAN_MAX_DATA_LEN);

  //Queue message for transmission
  if (can_transmit(&message, pdMS_TO_TICKS(200)) != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to queue message for transmission");
  }
}