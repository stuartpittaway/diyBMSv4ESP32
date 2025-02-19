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
    uint16_t chargevoltagelimit;
    uint16_t maxchargecurrent;
    uint16_t maxdischargecurrent;
    uint16_t dischargevoltage;

    // if no controllers are networked then just send the local values. It won't matter what controllerID is selected on the config page
    if (mysettings.controllerNet == 1)
    {
        memcpy(&chargevoltagelimit, &can.data[0][mysettings.controllerID][0], sizeof(chargevoltagelimit));
        memcpy(&maxchargecurrent, &can.data[0][mysettings.controllerID][2], sizeof(maxchargecurrent));
        memcpy(&maxdischargecurrent, &can.data[0][mysettings.controllerID][4], sizeof(maxdischargecurrent));
        memcpy(&dischargevoltage, &can.data[0][mysettings.controllerID][6], sizeof(dischargevoltage));
    }

    // aggregate DVCC data from networked controllers
    else
    {
    // Wait for permission from Canbus_RX task to edit data array
    if (!xSemaphoreTake(can.dataMutex[0], pdMS_TO_TICKS(100)))
    {
      ESP_LOGE(TAG, "CANBUS RX/TX intertask notification timeout")  ;
    }

        chargevoltagelimit = *(uint16_t*)&can.data[0][mysettings.controllerID][0];
        maxchargecurrent = *(uint16_t*)&can.data[0][mysettings.controllerID][2];
        maxdischargecurrent = *(uint16_t*)&can.data[0][mysettings.controllerID][4];
        dischargevoltage = *(uint16_t*)&can.data[0][mysettings.controllerID][6];

        for (int8_t i = 0; i < MAX_NUM_CONTROLLERS; i++)
        {
            //only include online controllers
            if (can.heartbeat[i] && can.data[2][i][1]) 
            {
                if ((*(uint16_t*)&can.data[0][i][0] <= chargevoltagelimit))  // find minimum
                {
                    chargevoltagelimit = *(uint16_t*)&can.data[0][i][0];    
                }
                if ((*(uint16_t*)&can.data[0][i][2] <= maxchargecurrent))    // find minimum
                {
                    maxchargecurrent = *(uint16_t*)&can.data[0][i][2];
                }
                if ((*(uint16_t*)&can.data[0][i][4] <= maxdischargecurrent))  // find minimum
                {
                    maxdischargecurrent = *(uint16_t*)&can.data[0][i][4];
                }
                if ((*(uint16_t*)&can.data[0][i][6] <= dischargevoltage))  // find minimum
                {
                    dischargevoltage = *(uint16_t*)&can.data[0][i][6];    
                }   
            }
        }

        // Return permission to Canbus_RX task 
        xSemaphoreGive(can.dataMutex[0]); 
        
        maxchargecurrent = maxchargecurrent *can.online_controller_count;  //use minimum multiplied by # of online controllers
        maxdischargecurrent = maxdischargecurrent *can.online_controller_count;    //use minimum multiplied by # of online controllers      

    }
        
        memcpy(&candata.data[0], &chargevoltagelimit, sizeof(chargevoltagelimit));   
        memcpy(&candata.data[2], &maxchargecurrent, sizeof(maxchargecurrent));      
        memcpy(&candata.data[4], &maxdischargecurrent, sizeof(maxdischargecurrent));  
        memcpy(&candata.data[6], &dischargevoltage, sizeof(dischargevoltage)); 
        ESP_LOGI(TAG, "Charge Voltage Limit = %d",chargevoltagelimit);
        ESP_LOGI(TAG, "Max Charge Current = %d",maxchargecurrent);
        ESP_LOGI(TAG, "Max Discharge Current = %d",maxdischargecurrent);
        ESP_LOGI(TAG, "Discharge Voltage Limit = %d",dischargevoltage);  

            // send to tx routine , block 50ms 
            if (xQueueSendToBack(CANtx_q_handle, &candata, pdMS_TO_TICKS(50)) != pdPASS)
            {
                ESP_LOGE(TAG, "Failed to Q 0x%x (queue full)",candata.identifier);
            }
    
}

// 0x355 – 1A 00 64 00 – State of Health (SOH) / State of Charge (SOC)
void pylon_message_355()
{
    CANframe candata;
    candata.dlc = 4;
    candata.identifier = 0x355;
    memset(&candata.data, 0, sizeof(candata.data));

    if (mysettings.controllerNet == 1)  //copy over local values for SOC
    {
        memcpy(&candata.data, &can.data[4][mysettings.controllerID][0], candata.dlc);
    }
    else
    {
        // Wait for permission from Canbus_RX task to edit data array
        if (!(xSemaphoreTake(can.dataMutex[4], pdMS_TO_TICKS(100)) &
            xSemaphoreTake(can.dataMutex[5], pdMS_TO_TICKS(100))))
        {
        ESP_LOGE(TAG, "CANBUS RX/TX intertask notification timeout")  ;
        }

        //SOC (weighted average based on nominal Ah of each controller)
        //SOH (display the minmimum health value of all online packs which could be more useful than an averaged health value)
        uint16_t Total_Ah = 0;
        uint32_t Total_Weighted_Ah = 0;
        uint16_t Weighted_SOC = 0;
        uint16_t SOH = *(uint16_t*)&can.data[4][mysettings.controllerID][0];  //start with this controllers SOH  
        
        for (int8_t i = 0; i < MAX_NUM_CONTROLLERS; i++)
        {
            if (can.heartbeat[i] && can.data[2][i][1])
            {
                Total_Ah = Total_Ah + *(uint16_t*)&can.data[5][i][4];     //online capacity
                Total_Weighted_Ah = Total_Weighted_Ah + (*(uint16_t*)&can.data[4][i][0]) * (*(uint16_t*)&can.data[5][i][4]);  //SOC x Online capacity
                
                // use minimum
                if (*(uint16_t*)&can.data[4][i][2] < SOH)
                {
                    SOH = *(uint16_t*)&can.data[4][i][2];
                }        
            }
        }

        // Return permission to Canbus_RX task 
        xSemaphoreGive(can.dataMutex[4]);
        xSemaphoreGive(can.dataMutex[5]); 

        if (Total_Ah != 0)  //avoid divide by zero (we won't have useable values during CAN initialization)
        {
            Weighted_SOC = Total_Weighted_Ah / Total_Ah;
        }

        memcpy(&candata.data[0], &Weighted_SOC, sizeof(Weighted_SOC));
        memcpy(&candata.data[2], &SOH, sizeof(SOH));

    }    

}

//Helper function to re-organize bit alarms
// q = row ofcan.data
// s = slice ofcan.data
// bitsource = the specific bit we're looking for atcan.data[q][r][s]
// bitdest = the bitmask we want to move it to
uint8_t alarm_align(uint8_t q, uint8_t s, uint8_t bitsource, uint8_t bitdest)
{
    bitsource = 1 << bitsource;
    uint8_t bitmask = 0;
    for (int8_t r = 0; r < MAX_NUM_CONTROLLERS; r++) 
    {
    bitmask |= bitsource &can.data[q][r][s];
    }
    bitmask = (bitmask >> bitsource) << bitdest; //re-index the bitmask to the desired destination bit
    return bitmask;
    
}


// 0x359 – 00 00 00 00 0A 50 4E – Protection & Alarm flags
 void pylon_message_359()
{
    CANframe candata;
    candata.dlc = 8;
    candata.identifier = 0x359;
    memset(&candata.data, 0, sizeof(candata.data));  

    if (mysettings.controllerNet == 1)  //copy over local values
    {
        memcpy(&candata.data, &can.data[1][mysettings.controllerID][0], candata.dlc);
    }
    else
    {
    // Wait for permission from Canbus_RX task to edit data array
    if (!xSemaphoreTake(can.dataMutex[5], pdMS_TO_TICKS(100)))
    {
      ESP_LOGE(TAG, "CANBUS RX/TX intertask notification timeout")  ;
    }

    //byte 0
    //(bit 0) = unused
    //(bit 1) Battery high voltage alarm
    candata.data[0] |= alarm_align(1,0,2,1);   //look at alarm found incan.data[1,n,0] , bit position 2 and translate to a bitmask at position 1 (=B0000010)
    //(bit 2) Battery low voltage alarm 
    candata.data[0] |= alarm_align(1,0,4,2);
    //(bit 3) Battery high temperature alarm
    candata.data[0] |= alarm_align(1,1,0,3);
    //(bit 4) Battery low temperature alarm
    candata.data[0] |= alarm_align(1,0,6,4);
    //(bit 5) = unused
    //(bit 6) = unused
    //(bit 7) = Discharge over current

    //byte 2
    // WARNING:Battery high voltage
    candata.data[2] |= alarm_align(1,4,2,1);
    // WARNING:Battery low voltage
    candata.data[2] |= alarm_align(1,4,4,1);
    // WARNING: Battery high temperature
    candata.data[2] |= alarm_align(1,4,6,3);
    // WARNING: Battery low temperature
    candata.data[2] |= alarm_align(1,5,0,4);
 
     //byte 3
    // iNTERNAL COMMUNICATION ERRROR
    candata.data[2] |= alarm_align(1,6,6,3);

    // byte 4
    // Pylon can have multiple battery each of 74Ah capacity, so emulate this based on total Ah capacity
    // this drives the inverter to assume certain charge/discharge parameters based on number of battery banks installed
    // Set inverter to use "Pylontech US3000C 3.5kWh" in its settings (these are 74Ah each)
    uint16_t totalnominalbatcap = 0;
    for (uint8_t i=0; i<MAX_NUM_CONTROLLERS; i++)
    {
    totalnominalbatcap = totalnominalbatcap + can.data[5][i][4];
    }
    candata.data[4] = max((uint8_t)1, (uint8_t)round(totalnominalbatcap / 74.0));
    candata.data[5] = 0x50;
    candata.data[6] = 0x4e;

    // Return permission to Canbus_RX task 
    xSemaphoreGive(can.dataMutex[5]);

    }
            // send to tx routine , block 50ms 
            if (xQueueSendToBack(CANtx_q_handle, &candata, pdMS_TO_TICKS(50)) != pdPASS)
            {
                ESP_LOGE(TAG, "Failed to Q 0x%x (queue full)",candata.identifier);
            }
}

// 0x35C – C0 00 – Battery charge request flags
void pylon_message_35c()
{
    CANframe candata;
    candata.dlc = 1;
    candata.identifier = 0x35c;
    memset(&candata.data, 0, sizeof(candata.data));

     int8_t byte0 = 0;
    // data.byte1 = 0;

    // Wait for permission from Canbus_RX task to edit data array
    if (!xSemaphoreTake(can.dataMutex[2], pdMS_TO_TICKS(100)))
    {
      ESP_LOGE(TAG, "CANBUS RX/TX intertask notification timeout")  ;
    }

    for (int8_t i = 0; i < MAX_NUM_CONTROLLERS; i++)
    {
        if (can.heartbeat[i] && can.data[2][i][1] &&can.data[2][i][2] == 0)  // don't factor in charge request flags from controllers operating under NetworkedControllerRules since it will prevent charging other controllers
        {
            byte0 = byte0 |can.data[2][i][1];  //byte 1 of bitmsgs is the charge/discharge request flag
        }     
    }

    // Return permission to Canbus_RX task 
    xSemaphoreGive(can.dataMutex[2]);  

    memcpy(&candata.data[0], &byte0, sizeof(byte0));


            // send to tx routine , block 50ms 
            if (xQueueSendToBack(CANtx_q_handle, &candata, pdMS_TO_TICKS(50)) != pdPASS)
            {
                ESP_LOGE(TAG, "Failed to Q 0x%x (queue full)",candata.identifier);
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
          uint8_t pylon[] = {0x50, 0x59, 0x4c, 0x4f, 0x4e, 0x20, 0x20, 0x20};;
          memcpy(&candata.data[0], &pylon[0], TWAI_FRAME_MAX_DLC);


            // send to tx routine , block 50ms 
            if (xQueueSendToBack(CANtx_q_handle, &candata, pdMS_TO_TICKS(50)) != pdPASS)
            {
                ESP_LOGE(TAG, "Failed to Q 0x%x (queue full)",candata.identifier);
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
        memcpy(&candata.data[0], &can.data[6][mysettings.controllerID][0], candata.dlc);
    }

    else
    {
    // Wait for permission from Canbus_RX task to edit data array
    if (!xSemaphoreTake(can.dataMutex[6], pdMS_TO_TICKS(100)))
    {
      ESP_LOGE(TAG, "CANBUS RX/TX intertask notification timeout")  ;
    }

         int16_t voltage = 0;
         int16_t current = 0;
         int16_t temperature = 0;

        for (int8_t i = 0; i < MAX_NUM_CONTROLLERS; i++)
        {
            if (can.heartbeat[i] && can.data[2][i][1])  // only use values from online controllers
            {
                voltage = voltage + *(int16_t*)&can.data[6][i][0];
                current = current + *(int16_t*)&can.data[6][i][2];
                temperature = temperature + *(int16_t*)&can.data[6][i][4];
            }
        }

        // Return permission to Canbus_RX task 
        xSemaphoreGive(can.dataMutex[6]); 
        
        voltage = voltage /can.online_controller_count;
        temperature = temperature /can.online_controller_count;


    memcpy(&candata.data[0], &voltage, sizeof(voltage));
    memcpy(&candata.data[2], &current, sizeof(current));
    memcpy(&candata.data[4], &temperature, sizeof(temperature));
    }


            // send to tx routine , block 50ms 
            if (xQueueSendToBack(CANtx_q_handle, &candata, pdMS_TO_TICKS(50)) != pdPASS)
            {
                ESP_LOGE(TAG, "Failed to Q 0x%x (queue full)",candata.identifier);
            }

}
