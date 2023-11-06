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

// 0x351 – Battery voltage + current limits
/*
See ControllerCAN section for detailed description of DVCC parameters for each controller

For aggregation:

For the maxchargevoltage reported to the inverter we will just use the minimum maxchargevoltage reported by any controller.

For the maxchargecurrent reported to the inverter, we will use the minimum maxchargecurrent reported by any controller multiplied by the number of conrollers online.
*/
void  pylon_message_351()
{
    CANframe candata;
    candata.dlc = TWAI_FRAME_MAX_DLC;
    candata.identifier = 0x351;
    memset(&candata.data, 0, sizeof(candata.data));


    // if no controllers are networked then just send the local values. It won't matter what controllerID is selected on the config page
    if (mysettings.controllerNet == 1)
    {
        memcpy(&candata.data, &CAN.data[0][mysettings.controllerID][0], candata.dlc);

    }

    // aggregate DVCC data from networked controllers and use the minimum for each parameter
    else
    {

        uint16_t chargevoltagelimit;
        uint16_t maxchargecurrent;
        uint16_t maxdischargecurrent;
        uint16_t dischargevoltage;

        chargevoltagelimit = *(uint16_t*)&CAN.data[0][mysettings.controllerID][0];
        maxchargecurrent = *(uint16_t*)&CAN.data[0][mysettings.controllerID][2];
        maxdischargecurrent = *(uint16_t*)&CAN.data[0][mysettings.controllerID][4];
        dischargevoltage = *(uint16_t*)&CAN.data[0][mysettings.controllerID][6];

        for (int8_t i = 0; i < MAX_NUM_CONTROLLERS; i++)
        {
            if (CAN.data[2][i][0] != 0)  //check bitmsgs timestamp so we only include online controllers
            {
                if ((*(uint16_t*)&CAN.data[0][i][0] <= chargevoltagelimit))  // find minimum
                {
                    chargevoltagelimit = *(uint16_t*)&CAN.data[0][i][0];    
                }
                if ((*(uint16_t*)&CAN.data[0][i][2] <= maxchargecurrent))    // find minimum
                {
                    maxchargecurrent = *(uint16_t*)&CAN.data[0][i][2];
                }
                if ((*(uint16_t*)&CAN.data[0][i][4] <= maxdischargecurrent))  // find minimum
                {
                    maxdischargecurrent = *(uint16_t*)&CAN.data[0][i][4];
                }
                if ((*(uint16_t*)&CAN.data[0][i][6] <= dischargevoltage))  // find minimum
                {
                    dischargevoltage = *(uint16_t*)&CAN.data[0][i][6];    
                }   
            }
        }
            maxchargecurrent = maxchargecurrent * CAN.online_controller_count;  //use minimum multiplied by # of online controllers
            maxdischargecurrent = maxdischargecurrent * CAN.online_controller_count;    //use minimum multiplied by # of online controllers




        memcpy(&candata.data[0], &chargevoltagelimit, sizeof(chargevoltagelimit));                  // fill in 1-8 data bytes
        memcpy(&candata.data[2], &maxchargecurrent, sizeof(maxchargecurrent));                  // fill in 1-8 data bytes
        memcpy(&candata.data[4], &maxdischargecurrent, sizeof(maxdischargecurrent));                  // fill in 1-8 data bytes
        memcpy(&candata.data[6], &dischargevoltage, sizeof(dischargevoltage));                  // fill in 1-8 data bytes


    }
        
        if (mysettings.controllerID==CAN.master)
        {
            // send to tx routine , block 50ms 
            if (xQueueSendToBack(CANtx_q_handle, &candata, pdMS_TO_TICKS(50)) != pdPASS)
            {
                ESP_LOGE(TAG, "Failed to Q 0x%x (queue full)",candata.identifier);
            }
        }

    
}

// 0x355 – 1A 00 64 00 – State of Health (SOH) / State of Charge (SOC)
void pylon_message_355()
{
    CANframe candata;
    candata.dlc = 2;
    candata.identifier = 0x355;
    memset(&candata.data, 0, sizeof(candata.data));

    if (mysettings.controllerNet == 1)
    {
        memcpy(&candata.data, &CAN.data[4][mysettings.controllerID][0], candata.dlc);
    }

    else
    {     //SOC (weighted average based on nominal Ah of each controller)
        uint16_t Total_Ah = 0;
        uint16_t Total_Weighted_Ah = 0;
        uint16_t Weighted_SOC = 0;

        for (int8_t i = 0; i < MAX_NUM_CONTROLLERS; i++)
        {
            if (CAN.data[2][i][0] != 0)  //check bitmsgs timestamp so we only include online controllers
            {
            Total_Ah = Total_Ah + *(uint16_t*)&(CAN.data[5][i][4]);
            Total_Weighted_Ah = Total_Weighted_Ah + *(uint16_t*)&CAN.data[4][i][0] * (*(uint16_t*)&CAN.data[5][i][4]);  //SOC x Online capacity


            }

        }

        Weighted_SOC = Total_Weighted_Ah / Total_Ah;

        memcpy(&candata.data[0], &Weighted_SOC, sizeof(Weighted_SOC));

    }
        if (mysettings.controllerID==CAN.master)
        {
            // send to tx routine , block 50ms 
            if (xQueueSendToBack(CANtx_q_handle, &candata, pdMS_TO_TICKS(50)) != pdPASS)
            {
                ESP_LOGE(TAG, "Failed to Q 0x%x (queue full)",candata.identifier);
            }
        }

}







// STILL NEED TO AGGREGATE THESE!!!!
// 0x359 – 00 00 00 00 0A 50 4E – Protection & Alarm flags
/* void pylon_message_359()
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
    // Quantity of banks in parallel
    uint8_t byte4;
    uint8_t byte5;
    uint8_t byte6;
    // Online address of banks in parallel - Table 5
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
    if (rules.highestBankVoltage / 100 > mysettings.chargevolt)
    {
      data.byte2 |= B00000010;
    }

    // WARNING:Battery low voltage
    // dischargevolt=490, lowestbankvoltage=48992 (scale down 100)
    if (rules.lowestBankVoltage / 100 < mysettings.dischargevolt)
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
    // this drives the inverter to assume certain charge/discharge parameters based on number of battery banks installed
    // Set inverter to use "Pylontech US3000C 3.5kWh" in its settings (these are 74Ah each)
    data.byte4 = max((uint8_t)1, (uint8_t)round(mysettings.nominalbatcap / 74.0));
  }
  else
  {
    // Default 1 battery
    data.byte4 = 1;
  }

  data.byte5 = 0x50; // P
  data.byte6 = 0x4e; // N

  send_canbus_message(0x359, (uint8_t *)&data, sizeof(data359));
} */

void pylon_message_359()   //temporary for testing purposes DO NOT USE FOR FINAL
{
  CANframe candata;
    candata.dlc = 7;
    candata.identifier = 0x359;
    memset(&candata.data, 0, sizeof(candata.data));    
    candata.data[4] = 1;
    candata.data[5] = 0x50;
    candata.data[6] = 0x4e;

        if (mysettings.controllerID==CAN.master)
        {
            // send to tx routine , block 50ms 
            if (xQueueSendToBack(CANtx_q_handle, &candata, pdMS_TO_TICKS(50)) != pdPASS)
            {
                ESP_LOGE(TAG, "Failed to Q 0x%x (queue full)",candata.identifier);
            }
        }
}







// 0x35C – C0 00 – Battery charge request flags
// Raise charge/discharge flag if ANY controller is currently allowing
void pylon_message_35c()
{
    CANframe candata;
    candata.dlc = 1;
    candata.identifier = 0x35c;
    memset(&candata.data, 0, sizeof(candata.data));

     int8_t byte0 = 0;
    // data.byte1 = 0;

    for (int8_t i = 0; i < MAX_NUM_CONTROLLERS; i++)
    {
        byte0 = byte0 || CAN.data[2][i][1];  //byte 1 of bitmsgs is the charge/discharge request flag
    }

    memcpy(&candata.data[0], &byte0, sizeof(byte0));


        if (mysettings.controllerID==CAN.master)
        {
            // send to tx routine , block 50ms 
            if (xQueueSendToBack(CANtx_q_handle, &candata, pdMS_TO_TICKS(50)) != pdPASS)
            {
                ESP_LOGE(TAG, "Failed to Q 0x%x (queue full)",candata.identifier);
            }
        }
}


// 0x35E – 50 59 4C 4F 4E 20 20 20 – Manufacturer name ("PYLON ")
void pylon_message_35e()
{
    CANframe candata;
    candata.dlc = TWAI_FRAME_MAX_DLC;
    candata.identifier = 0x35e;
    memset(&candata.data, 0, sizeof(candata.data));

          // Send 8 byte "magic string" PYLON (with 3 trailing spaces)
          const char pylon[] = "\x50\x59\x4c\x4f\x4e\x20\x20\x20";
          memcpy(&candata.data[0], &pylon[0], TWAI_FRAME_MAX_DLC);

        if (mysettings.controllerID==CAN.master)
        {
            // send to tx routine , block 50ms 
            if (xQueueSendToBack(CANtx_q_handle, &candata, pdMS_TO_TICKS(50)) != pdPASS)
            {
                ESP_LOGE(TAG, "Failed to Q 0x%x (queue full)",candata.identifier);
            }
        }

}

// Battery voltage - 0x356 – 4e 13 02 03 04 05 – Voltage / Current / Temp
void pylon_message_356()
{
    CANframe candata;
    candata.dlc = 6;
    candata.identifier = 0x356;
    memset(&candata.data, 0, sizeof(candata.data));

    if (mysettings.controllerNet == 1)
    {
        memcpy(&candata.data[0], &CAN.data[6][mysettings.controllerID][0], candata.dlc);
    }

    else
    
    {
         int16_t voltage, current, temperature;
        for (int8_t i = 0; i < MAX_NUM_CONTROLLERS; i++)
        {
            if (CAN.data[2][i][0] != 0)  // only use 0 values from online controllers
            {
                voltage = voltage + *(int16_t*)&CAN.data[6][i][0];
                current = current + *(int16_t*)&CAN.data[6][i][2];
                temperature = temperature + *(int16_t*)&CAN.data[6][i][4];
            }
        }

        voltage = voltage / CAN.online_controller_count;
        temperature = temperature / CAN.online_controller_count;


    memcpy(&candata.data[0], &voltage, sizeof(voltage));
    memcpy(&candata.data[2], &current, sizeof(current));
    memcpy(&candata.data[4], &temperature, sizeof(temperature));
    }
        if (mysettings.controllerID==CAN.master)
        {
            // send to tx routine , block 50ms 
            if (xQueueSendToBack(CANtx_q_handle, &candata, pdMS_TO_TICKS(50)) != pdPASS)
            {
                ESP_LOGE(TAG, "Failed to Q 0x%x (queue full)",candata.identifier);
            }
        }

}
