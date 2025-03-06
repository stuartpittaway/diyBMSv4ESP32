/*

This code formats local controller data into "industry standard" CAN messages for the purpose of reporting to various inverter manufacturers. It also establishes Intra-Controller communication of these messages to be aggregated and reported to an inverter

*/


#define USE_ESP_IDF_LOG 1
static constexpr const char *const TAG = "diybms-ControllerCAN";

#include "ControllerCAN.h"

ControllerCAN can;

void ControllerCAN::init_hash_table() {
  memset(hash_table, 0, sizeof(hash_table));

  for (uint8_t i = 0; i < MAX_CAN_PARAMETERS; i++)          //traverse rows of id[]
              {
                  for (uint8_t j = 0; j < MAX_NUM_CONTROLLERS; j++)      //traverse columns of id[]
                  {
                      hash_table[id[i][j] - 600] = i * 100 + j;		//hash store indices
        }
      };
}

//TimerHandle_t error_debounce_timer;
bool ControllerCAN::NetworkedControllerRules()
{
  if (!canDisconnect)
    {
      if (rules.moduleHasExternalTempSensor == false)
      {
         if( !xTimerIsTimerActive( error_debounce_timer ))
         {
           xTimerStart(error_debounce_timer, 0);
           ESP_LOGD(TAG,"disconnect timer started");
         }
        return true;
      }

      if (rules.invalidModuleCount > 0)
      {  
        if( !xTimerIsTimerActive(error_debounce_timer ))
        {
          xTimerStart(error_debounce_timer, 0);
          ESP_LOGD(TAG,"disconnect timer started");
        }
        return true;
      }

      if (rules.numberOfActiveErrors > 0)
      {   
        if( !xTimerIsTimerActive(error_debounce_timer ))
        {
          xTimerStart(error_debounce_timer, 0);
          ESP_LOGD(TAG,"disconnect timer started");
        }
          return true;
      }

      // Clear the Timer if everything is good
      xTimerStop(error_debounce_timer, pdMS_TO_TICKS(20));

      return false;
    }
  
    // display disconnect warning 
  else 
  {     
      // reset WARNING if CANBUS emulation is manually re-enabled
      if (mysettings.protocol != ProtocolEmulation::EMULATION_DISABLED)
      {
        canDisconnect = false;
        return false;
      }

      ESP_LOGE(TAG,"CANBUS WAS DISCONNECTED DUE TO INTERNAL ERROR - MANUAL RESTART REQUIRED");

      // Add some additional delay here to prevent flooding the error log every 900ms (CanbusTX task has nothing else 
      // to do at this moment anyhow)
      vTaskDelay(pdMS_TO_TICKS(10000));

      return true;
    }
}

void ControllerCAN::clearvalues()
{
   online_controller_count = 1; 
   master = 0;
      // Zero Data Array 
    memset(&data, 0, sizeof(data));
    memset(&timestampBuffer, 0, sizeof(timestampBuffer));
}

// Check controller network status & update heartbeat  Returns:  0=OK  1=controller offline  2=controller network configuration error
uint8_t ControllerCAN::controllerNetwork_status()
{
  uint8_t returnvalue = 0;
  uint8_t controller_count = 0; 
  uint8_t addressbitmask = 0;
  uint8_t integrated_controllers = 0;
  uint8_t high_availability = mysettings.highAvailable;

  // Wait for permission from Canbus_RX task to edit data array
  if (!xSemaphoreTake(dataMutex[2], pdMS_TO_TICKS(50)))
  {
    ESP_LOGE(TAG, "CANBUS RX/TX intertask notification timeout") ;
    return returnvalue; 
  }

  memset(&controller_is_online[0], 0, sizeof(controller_is_online));
  memset(&controller_is_integrated[0], 0, sizeof(controller_is_integrated));

  for (uint8_t i = 0; i < MAX_NUM_CONTROLLERS; i++)
  {
    if ((esp_timer_get_time() - timestampBuffer[i]) < HEARTBEAT_PERIOD) // only poll online controllers
    {
      controller_count++;
      controller_is_online[i] = true;

      // how many are integrated (not isolated)
      if (data[2][i][1])
      {
        integrated_controllers++;
        controller_is_integrated[i] = true;
      }

      // checksum for controller address overlap
      addressbitmask &= data[2][i][0]; // this should should never be elevated above 0 or there is an overlap
      
      // check that local high_availability setting matches the network  
      if (mysettings.highAvailable != data[2][i][3])
      {
      ESP_LOGE(TAG, "Local 'High Availability' settings do not match with networked ControllerID %d",i);
      returnvalue = 2;
      } 
      // check that # networked controllers setting matches the network
      if (mysettings.controllerNet != data[2][i][4])
      {
      ESP_LOGE(TAG, "Local '# of Networked Controllers' settings do not match with networked ControllerID %d",i);
      returnvalue = 2; 
      }
    }
  }

  // Return permission to Canbus_RX task 
  xSemaphoreGive(dataMutex[2]); 
  
  if (returnvalue == 2)
  {
    return returnvalue;
  }

  if (addressbitmask != 0)
  {
    ESP_LOGE(TAG, "Controller address conflict");
    returnvalue = 2;
    online_controller_count = controller_count;

    return returnvalue;
  }

   // checksum for connected controllers 
  if (controller_count > mysettings.controllerNet)
  {
      ESP_LOGE(TAG, "Controller network count discrepancy");
      returnvalue = 2;
      online_controller_count = controller_count;

      return returnvalue;
  }
  else if (controller_count < mysettings.controllerNet)
  {
      ESP_LOGE(TAG, "A controller is offline. #Online Controllers=%d/%d",online_controller_count,mysettings.controllerNet);
      returnvalue = 1; 
      online_controller_count = controller_count;

      return returnvalue;
  }

  else
  {
      online_controller_count = controller_count;
      integrated_count = integrated_controllers;
      return returnvalue;
  }


 }

void ControllerCAN::who_is_master()  // Decide which controller is master ( == the lowest integer-addressed controller number with a valid heartbeat)
{   

      for (uint8_t i = 0; i < MAX_NUM_CONTROLLERS; i++)
      {
        if (controller_is_online[i])
        {
            master=i;
            
            ESP_LOGI(TAG, "Current master controller ID: %d", master);
            break; 
        }
      }
}

void ControllerCAN::SetBankAndModuleText(char *buffer, uint8_t cellid)       //function used for cell i.d.'s
{

  uint8_t bank = cellid / mysettings.totalNumberOfSeriesModules;
  uint8_t module = cellid - (bank * mysettings.totalNumberOfSeriesModules);

  // Clear all 8 bytes
  memset(buffer, 0, 8);

    if (mysettings.controllerNet != 1) 
    {
      snprintf(buffer, 8, "%db%dm%d", mysettings.controllerID, bank, module);      
    }                                                                   
    else
      snprintf(buffer, 8, "b%d m%d", bank, module);

}

void ControllerCAN::c2c_DVCC()    //DVCC settings
{
    CANframe candata(TWAI_FRAME_MAX_DLC, id[0][mysettings.controllerID]);  // initialize struct with dlc and identifier
    CANframe* ptrFrame = &candata;

    // CVL - 0.1V scale
    uint16_t chargevoltagelimit;
    // CCL - 0.1A scale
    int16_t maxchargecurrent;
    // DCL - 0.1A scale
    int16_t maxdischargecurrent;
    // Not currently used by Victron
    // 0.1V scale
    uint16_t dischargevoltage = mysettings.dischargevolt;


    // THESE DEFAULTS (do nothing) are dependant on the particular inverter 
      uint16_t default_charge_voltage;         
      int16_t default_charge_current_limit;   
      int16_t default_discharge_current_limit; 

      if (mysettings.protocol == ProtocolEmulation::CANBUS_VICTRON)
      {
        // Don't use zero for voltage - this indicates to Victron an over voltage situation, and Victron gear attempts to dump
        // the whole battery contents!  (feedback from end users)
        default_charge_voltage = rules.lowestBankVoltage / 100;   
        default_charge_current_limit = 0;    
        default_discharge_current_limit = 0; 
      }
      else if ((mysettings.protocol == ProtocolEmulation::CANBUS_PYLONTECH) && (mysettings.canbusinverter == CanBusInverter::INVERTER_DEYE))
      {
        // FOR DEYE INVERTERS APPLY DIFFERENT LOGIC TO PREVENT "W31" ERRORS
        // ISSUE #216
        default_charge_voltage = rules.lowestBankVoltage / 100;
        default_charge_current_limit = 0;
        default_discharge_current_limit = 0;
      }
      else if ((mysettings.protocol == ProtocolEmulation::CANBUS_PYLONTECH) && (mysettings.canbusinverter == CanBusInverter::INVERTER_GENERIC))
      { // If we pass ZERO's to SOFAR inverter it appears to ignore them
        // so send 0.1V and 0.1Amps instead to indicate "stop"
        default_charge_voltage = 1;         // 0.1V
        default_charge_current_limit = 1;    // 0.1A
        default_discharge_current_limit = 1; // 0.1A
      }


  // Charge settings...
  if (rules.IsChargeAllowed(&mysettings))
  {
    if (rules.numberOfBalancingModules > 0 && mysettings.stopchargebalance == true)
    {
      // Balancing, stop charge
    chargevoltagelimit = default_charge_voltage;
    maxchargecurrent = default_charge_current_limit;
    }
    else
    {
      // Default - normal behaviour
      chargevoltagelimit = rules.DynamicChargeVoltage();
      maxchargecurrent = rules.DynamicChargeCurrent();
    }
  }
  else
  {
    chargevoltagelimit = default_charge_voltage;
    maxchargecurrent = default_charge_current_limit;
  }

  // Discharge settings....
  if (rules.IsDischargeAllowed(&mysettings))
  {
    maxdischargecurrent = mysettings.dischargecurrent;
  }
  else
  {
    maxdischargecurrent = default_discharge_current_limit;
  }
  

    memcpy(&candata.data[0],&chargevoltagelimit,sizeof(chargevoltagelimit));        
    memcpy(&candata.data[2],&maxchargecurrent,sizeof(maxchargecurrent));            
    memcpy(&candata.data[4],&maxdischargecurrent,sizeof(maxdischargecurrent));     
    memcpy(&candata.data[6],&dischargevoltage,sizeof(dischargevoltage));           

    // Wait for permission from Canbus_RX task to edit data array
    if (!xSemaphoreTake(dataMutex[0], pdMS_TO_TICKS(50)))
    {
      ESP_LOGE(TAG, "CANBUS RX/TX intertask notification timeout");
      return; 
    }

        memcpy(&can.data[0][mysettings.controllerID][0], &candata.data, candata.dlc);       //copy calculated values to array

        // Return permission to Canbus_RX task 
        xSemaphoreGive(dataMutex[0]);

    if (mysettings.controllerNet != 1)
    {     
      send_canbus_message(ptrFrame);
    }

}

// diyBMS will use Victron CAN alarm structure to report alarms. These will need reorganized durring aggregation for other inverters  as necessary
void ControllerCAN::c2c_ALARMS()      //Inverter Alarms 
{
  CANframe candata(TWAI_FRAME_MAX_DLC, id[1][mysettings.controllerID]);  // initialize struct with dlc and identifier
  CANframe* ptrFrame = &candata;

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
    // BYTE 0
    //(bit 0+1) General alarm (not implemented)
    //(bit 2+3) Battery high voltage alarm
    candata.data[0] |= ((rules.ruleOutcome(Rule::BankOverVoltage) | rules.ruleOutcome(Rule::CurrentMonitorOverVoltage)) ? BIT23_ALARM : BIT23_OK);
    
    //(bit 4+5) Battery low voltage alarm
    candata.data[0] |= ((rules.ruleOutcome(Rule::BankUnderVoltage) | rules.ruleOutcome(Rule::CurrentMonitorUnderVoltage)) ? BIT45_ALARM : BIT45_OK);

    //(bit 6+7) Battery high temperature alarm
    if (rules.moduleHasExternalTempSensor)
    {
      candata.data[0] |= (rules.ruleOutcome(Rule::ModuleOverTemperatureExternal) ? BIT67_ALARM : BIT67_OK);
    }

    // BYTE 1
    // 1 (bit 0+1) Battery low temperature alarm
    if (rules.moduleHasExternalTempSensor)
    {
      candata.data[1] |= (rules.ruleOutcome(Rule::ModuleUnderTemperatureExternal) ? BIT01_ALARM : BIT01_OK);
    }
    // 1 (bit 2+3) Battery high temperature charge alarm
    // byte1 |= BIT23_NOTSUP;
    // 1 (bit 4+5) Battery low temperature charge alarm
    // byte1 |= BIT45_NOTSUP;
    // 1 (bit 6+7) Battery high current alarm
    // byte1 |= BIT67_NOTSUP;
  }

  // 2 (bit 0+1) Battery high charge current alarm
  // byte2 |= BIT01_NOTSUP;
  // 2 (bit 2+3) Contactor Alarm (not implemented)
  // byte2 |= BIT23_NOTSUP;
  // 2 (bit 4+5) Short circuit Alarm (not implemented)
  // byte2 |= BIT45_NOTSUP;

  // ESP_LOGI(TAG, "Rule BMSError=%u, EmergencyStop=%u", rules.ruleOutcome[Rule::BMSError], rules.ruleOutcome[Rule::EmergencyStop]);

  // 2 (bit 6+7) BMS internal alarm
  candata.data[2] |= ((rules.ruleOutcome(Rule::BMSError) | rules.ruleOutcome(Rule::EmergencyStop)) ? BIT67_ALARM : BIT67_OK);
  // 3 (bit 0+1) Cell imbalance alarm
  // byte3 |= BIT01_NOTSUP;
  // 3 (bit 2+3) Reserved
  // 3 (bit 4+5) Reserved
  // 3 (bit 6+7) Reserved
  //  memset(&q_message[4], byte3, 1);

  // 4 (bit 0+1) General warning (not implemented)
  // byte4 |= BIT01_NOTSUP;
  // 4 (bit 2+3) Battery low voltage warning
    // dischargevolt=490, lowestbankvoltage=48992 (scale down 100)
    if (rules.lowestBankVoltage / 100 < mysettings.dischargevolt)
    {
      candata.data[4] |= BIT23_ALARM;
    }
  // 4 (bit 4+5) Battery high voltage warning
      if (rules.highestBankVoltage / 100 > mysettings.chargevolt)
    {
      candata.data[4] |= BIT45_ALARM;
    }

  // 4 (bit 6+7) Battery high temperature warning
    if (rules.moduleHasExternalTempSensor && rules.highestExternalTemp > mysettings.chargetemphigh)
    {
      candata.data[4] |= BIT67_ALARM;
    }
  // memset(&q_message[5], byte4, 1);

  // 5 (bit 0+1) Battery low temperature warning
    if (rules.moduleHasExternalTempSensor && rules.lowestExternalTemp < mysettings.chargetemplow)
    {
      candata.data[5] |= BIT01_ALARM;
    }
  // 5 (bit 2+3) Battery high temperature charge warning
  // byte5 |= BIT23_NOTSUP;
  // 5 (bit 4+5) Battery low temperature charge warning
  // byte5 |= BIT45_NOTSUP;
  // 5 (bit 6+7) Battery high current warning
  // byte5 |= BIT67_NOTSUP;
    //memset(&q_message[6], byte5, 1);

  // 6 (bit 0+1) Battery high charge current warning
  // byte6 |= BIT01_NOTSUP;
  // 6 (bit 2+3) Contactor warning (not implemented)
  // byte6 |= BIT23_NOTSUP;
  // 6 (bit 4+5) Short circuit warning (not implemented)
  // byte6 |= BIT45_NOTSUP;
  // 6 (bit 6+7) BMS internal warning
   candata.data[6] |= ((rules.ruleOutcome(Rule::BMSError) || rules.ruleOutcome(Rule::EmergencyStop)) ? BIT67_ALARM : 0);
   candata.data[6] |= ((_controller_state != ControllerState::Running) ? BIT67_ALARM : 0);


  // ESP_LOGI(TAG, "numberOfBalancingModules=%u", rules.numberOfBalancingModules);

  // 7 (bit 0+1) Cell imbalance warning
  // byte7 |= (rules.numberOfBalancingModules > 0 ? BIT01_ALARM : BIT01_OK);

  // 7 (bit 2+3) System status (online/offline) [1]
  candata.data[7] |= ((_controller_state != ControllerState::Running) ? BIT23_ALARM : BIT23_OK);
                                  

    // Wait for permission from Canbus_RX task to edit data array
    if (!xSemaphoreTake(dataMutex[1], pdMS_TO_TICKS(50)))
    {
      ESP_LOGE(TAG, "CANBUS RX/TX intertask notification timeout")  ;
      return; 
    }

    memcpy(&data[1][mysettings.controllerID][0], &candata.data, candata.dlc);                        //copy calculated values to array
    
    // Return permission to Canbus_RX task 
    xSemaphoreGive(dataMutex[1]);

    if (mysettings.controllerNet != 1)
    {
      send_canbus_message(ptrFrame);
    }
}

void ControllerCAN::c2c_DIYBMS_MSGS()      // diyBMS messaging/alarms
{
  CANframe candata(TWAI_FRAME_MAX_DLC, id[2][mysettings.controllerID]);  // initialize struct with dlc and identifier
  CANframe* ptrFrame = &candata;


// byte 0 - Broadcast the Controller ID
// 
// Bit position determines which controller number is set within configuration
  //CNTRL0 = B00000001  
  //CNTRL1 = B00000010   
  //CNTRL2 = B00000100
  //CNTRL3 = B00001000
  //CNTRL4 = B00010000
  //CNTRL5 = B00100000
  //CNTRL6 = B01000000
  //CNTRL7 = B10000000    
  candata.data[0] = 0x1 << mysettings.controllerID;   
 
// byte 1 - Is Charge/Discharge Allowed?
// This will work out to be zero (B00000000) if pack is in full Isolation
  candata.data[1] = 0;
  

  if (rules.IsChargeAllowed(&mysettings))
  {
      candata.data[1] = candata.data[1] | B10000000;
  }

  if (rules.IsDischargeAllowed(&mysettings))
  {
      candata.data[1] = candata.data[1] | B01000000;
  }



// byte 2 - reserved for future use


// byte 3 - HighAvailability setting
  candata.data[3] = mysettings.highAvailable;
// byte 4 - # Networked Controllers
  candata.data[4] = mysettings.controllerNet;
// byte 5 - reserved for future use 
// byte 6 - reserved for future use
// byte 7 - reserved for future use 


    // Wait for permission from Canbus_RX task to edit data array
    if (!xSemaphoreTake(dataMutex[2], pdMS_TO_TICKS(50)))
    {
      ESP_LOGE(TAG, "CANBUS RX/TX intertask notification timeout")  ;
      return; 
    }

    memcpy(&data[2][mysettings.controllerID][0], &candata.data, candata.dlc);    

    //update both heartbeat buffers for this controller
    can.timestampBuffer[mysettings.controllerID] = esp_timer_get_time();

    // Return permission to Canbus_RX task 
    xSemaphoreGive(dataMutex[2]);

    if (mysettings.controllerNet != 1)
    {
      send_canbus_message(ptrFrame);
    }
}

void ControllerCAN::c2c_MODULES()       // # of modules O.K. 
{
  CANframe candata(TWAI_FRAME_MAX_DLC, id[3][mysettings.controllerID]);  // initialize struct with dlc and identifier
  CANframe* ptrFrame = &candata;      

  
    uint16_t numberofmodulesok = TotalNumberOfCells() - rules.invalidModuleCount;
    // uint16_t numberofmodulesblockingcharge = 0;
    // uint16_t numberofmodulesblockingdischarge = 0;
    uint16_t numberofmodulesoffline = rules.invalidModuleCount;

    memcpy(&candata.data[0],&numberofmodulesok,sizeof(numberofmodulesok));
    memcpy(&candata.data[6],&numberofmodulesoffline,sizeof(numberofmodulesoffline));                                        

    // Wait for permission from Canbus_RX task to edit data array
    if (!xSemaphoreTake(dataMutex[3], pdMS_TO_TICKS(50)))
    {
      ESP_LOGE(TAG, "CANBUS RX/TX intertask notification timeout")  ;
      return; 
    }

    memcpy(&data[3][mysettings.controllerID][0], &candata.data, candata.dlc);                        //copy calculated values to array
 
    // Return permission to Canbus_RX task 
    xSemaphoreGive(dataMutex[3]);

    if (mysettings.controllerNet != 1)
    {
      send_canbus_message(ptrFrame);
    }
}

void ControllerCAN::c2c_SOC()      // SOC
{
  CANframe candata(4, id[4][mysettings.controllerID]);  // initialize struct with dlc and identifier
  CANframe* ptrFrame = &candata;      

    uint16_t stateofchargevalue;
    uint16_t stateofhealthvalue;
    // uint16_t highresolutionsoc;

  if (_controller_state == ControllerState::Running && mysettings.currentMonitoringEnabled && currentMonitor.validReadings && (mysettings.currentMonitoringDevice == CurrentMonitorDevice::DIYBMS_CURRENT_MON_MODBUS || mysettings.currentMonitoringDevice == CurrentMonitorDevice::DIYBMS_CURRENT_MON_INTERNAL))
  {

    stateofchargevalue = rules.StateOfChargeWithRulesApplied(&mysettings, currentMonitor.stateofcharge);
    stateofhealthvalue = (uint16_t)(trunc(mysettings.soh_percent));
                                             
    memcpy(&candata.data[0],&stateofchargevalue,sizeof(stateofchargevalue));  
    memcpy(&candata.data[2],&stateofhealthvalue,sizeof(stateofhealthvalue));                                         

    // Wait for permission from Canbus_RX task to edit data array
    if (!xSemaphoreTake(dataMutex[4], pdMS_TO_TICKS(50)))
    {
      ESP_LOGE(TAG, "CANBUS RX/TX intertask notification timeout")  ;
      return; 
    }

    memcpy(&data[4][mysettings.controllerID][0], candata.data,candata.dlc);                   //copy calculated values to array

    // Return permission to Canbus_RX task 
    xSemaphoreGive(dataMutex[4]);

    if (mysettings.controllerNet != 1)
    {
      send_canbus_message(ptrFrame);
    }
  }
}

void ControllerCAN::c2c_CAP()   // Online capacity and firmware version
{
  CANframe candata(TWAI_FRAME_MAX_DLC, id[5][mysettings.controllerID]);  // initialize struct with dlc and identifier
  CANframe* ptrFrame = &candata;      

    uint16_t BatteryModel;
    uint16_t Firmwareversion;
    uint16_t OnlinecapacityinAh;

  // Not used by Victron
  BatteryModel = 0;

  // Need to swap bytes for this to make sense.
  Firmwareversion = ((uint16_t)COMPILE_WEEK_NUMBER_BYTE << 8) | COMPILE_YEAR_BYTE;

  OnlinecapacityinAh = mysettings.nominalbatcap;

                                                        
    memcpy(&candata.data[0],&BatteryModel,sizeof(BatteryModel));                     
    memcpy(&candata.data[2],&Firmwareversion,sizeof(Firmwareversion));                
    memcpy(&candata.data[4],&OnlinecapacityinAh,sizeof(OnlinecapacityinAh));    

    // Wait for permission from Canbus_RX task to edit data array
    if (!xSemaphoreTake(dataMutex[5], pdMS_TO_TICKS(50)))
    {
      ESP_LOGE(TAG, "CANBUS RX/TX intertask notification timeout")  ;
      return; 
    }

      memcpy(&data[5][mysettings.controllerID][0], &candata.data, candata.dlc);                  //copy local values to array

      // Return permission to Canbus_RX task 
      xSemaphoreGive(dataMutex[5]);

    if (mysettings.controllerNet != 1)
    {
      send_canbus_message(ptrFrame);
    }
}

void ControllerCAN::c2c_VIT()   //Battery voltage, current, and temperature
{
  CANframe candata(6, id[6][mysettings.controllerID]);  // initialize struct with dlc and identifier
  CANframe* ptrFrame = &candata; 

    int16_t voltage; 
    int16_t current;
    int16_t temperature;  

  // Use highest bank voltage calculated by controller and modules
  // Scale 0.01V
   voltage = rules.highestBankVoltage / 10;

  // If current shunt is installed, use the voltage from that as it should be more accurate
  if (mysettings.currentMonitoringEnabled && currentMonitor.validReadings)
  {
   voltage = currentMonitor.modbus.voltage * 100.0;
  }

  
  current = 0;
  // If current shunt is installed, use it
  if (mysettings.currentMonitoringEnabled && currentMonitor.validReadings)
  {
    // Scale 0.1A
  current = currentMonitor.modbus.current * 10;
  }


  // Temperature 0.1C using external temperature sensor
  if (rules.moduleHasExternalTempSensor)
  {
   temperature = (int16_t)rules.highestExternalTemp * (int16_t)10;
  }
  else
  {
    // No external temp sensors
   temperature = 0;
  }

    memcpy(&candata.data[0],&voltage,sizeof(voltage));      
    memcpy(&candata.data[2],&current,sizeof(current));         
    memcpy(&candata.data[4],&temperature,sizeof(temperature));   
    
    // Wait for permission from Canbus_RX task to edit data array
    if (!xSemaphoreTake(dataMutex[6], pdMS_TO_TICKS(50)))
    {
      ESP_LOGE(TAG, "CANBUS RX/TX intertask notification timeout")  ;
      return; 
    }
    
      memcpy(&data[6][mysettings.controllerID][0], candata.data,candata.dlc);       //copy calculated values to local array

      // Return permission to Canbus_RX task 
      xSemaphoreGive(dataMutex[6]);

     // send to tx routine , block 50ms 
      if (mysettings.controllerNet != 1)
      {
        send_canbus_message(ptrFrame);
      }
}

void ControllerCAN::c2c_HOST()     //unique part of hostname
{
  CANframe candata(TWAI_FRAME_MAX_DLC, id[7][mysettings.controllerID]);  // initialize struct with dlc and identifier
  CANframe* ptrFrame = &candata;     
                                              
    memcpy(&candata.data[0],&hostname[7],sizeof(candata.data));          
    
    // Wait for permission from Canbus_RX task to edit data array
    if (!xSemaphoreTake(dataMutex[7], pdMS_TO_TICKS(50)))
    {
      ESP_LOGE(TAG, "CANBUS RX/TX intertask notification timeout")  ;
      return; 
    }

    memcpy(&data[7][mysettings.controllerID][0], candata.data,candata.dlc);       //copy calculated values to array

    // Return permission to Canbus_RX task 
    xSemaphoreGive(dataMutex[7]);

    if (mysettings.controllerNet != 1)
    {
      send_canbus_message(ptrFrame);
    }
}

void ControllerCAN::c2c_MINMAX_CELL_V_T()    // Min/Max Cell V & T
{
  CANframe candata(TWAI_FRAME_MAX_DLC, id[8][mysettings.controllerID]);  // initialize struct with dlc and identifier
  CANframe* ptrFrame = &candata;     

    uint16_t mincellvoltage;
    uint16_t maxcellvoltage;
    uint16_t lowestcelltemperature;
    uint16_t highestcelltemperature;
 
      lowestcelltemperature = 273 + rules.lowestExternalTemp;
      highestcelltemperature = 273 + rules.highestExternalTemp;
      maxcellvoltage = rules.highestCellVoltage;
      mincellvoltage = rules.lowestCellVoltage;
                                             
      memcpy(&candata.data[0],&mincellvoltage,sizeof(mincellvoltage));               
      memcpy(&candata.data[2],&maxcellvoltage,sizeof(maxcellvoltage));           
      memcpy(&candata.data[4],&lowestcelltemperature,sizeof(lowestcelltemperature));  
      memcpy(&candata.data[6],&highestcelltemperature,sizeof(highestcelltemperature));                                 
    
    // Wait for permission from Canbus_RX task to edit data array
    if (!xSemaphoreTake(dataMutex[8], pdMS_TO_TICKS(50)))
    {
      ESP_LOGE(TAG, "CANBUS RX/TX intertask notification timeout")  ;
      return; 
    }

      memcpy(&data[8][mysettings.controllerID][0], candata.data,candata.dlc);       //copy calculated values to array

      // Return permission to Canbus_RX task 
      xSemaphoreGive(dataMutex[8]);

    if (mysettings.controllerNet != 1)
    {
      send_canbus_message(ptrFrame);
    }

}

void ControllerCAN::c2c_CELL_IDS()   // Min/Max Cell V & T addresses
{
  CANframe candata(TWAI_FRAME_MAX_DLC, 0);  // initialize struct with dlc and identifier
  CANframe* ptrFrame = &candata;      
  
  char text[8];

  // Wait for permission from Canbus_RX task to edit data array
  if (!(xSemaphoreTake(dataMutex[9], pdMS_TO_TICKS(50)) &&
       xSemaphoreTake(dataMutex[10], pdMS_TO_TICKS(50)) &&
       xSemaphoreTake(dataMutex[11], pdMS_TO_TICKS(50)) &&
       xSemaphoreTake(dataMutex[12], pdMS_TO_TICKS(50))))
  {
    ESP_LOGE(TAG, "CANBUS RX/TX intertask notification timeout");

    xSemaphoreGive(dataMutex[9]); 
    xSemaphoreGive(dataMutex[10]); 
    xSemaphoreGive(dataMutex[11]);
    xSemaphoreGive(dataMutex[12]); 
    return; 
  }

    // Min. cell voltage id string [1]
  if (rules.address_LowestCellVoltage < maximum_controller_cell_modules)
  {
    SetBankAndModuleText(text, rules.address_LowestCellVoltage);

    memcpy(&candata.data[0],&text,sizeof(text));   
    candata.identifier = id[9][mysettings.controllerID]; 

    memcpy(&data[9][mysettings.controllerID][0], candata.data,candata.dlc);       //copy calculated values to array

     // send to tx routine 
    if (mysettings.controllerNet != 1)
    {
      send_canbus_message(ptrFrame);
    }
  }
    // Max. cell voltage id string [1]
  if (rules.address_HighestCellVoltage < maximum_controller_cell_modules)
  {
    memset(&candata.data, 0, TWAI_FRAME_MAX_DLC);
    SetBankAndModuleText(text, rules.address_HighestCellVoltage);

    memcpy(&candata.data[0],&text,sizeof(text));   
    candata.identifier = id[10][mysettings.controllerID]; 

    memcpy(&data[10][mysettings.controllerID][0], candata.data,candata.dlc);       //copy calculated values to array

     // send to tx routine
    if (mysettings.controllerNet != 1)
    {
      send_canbus_message(ptrFrame);
    }
  }
    // Min. cell temp id string [1]
  if (rules.address_lowestExternalTemp < maximum_controller_cell_modules)
  {
    memset(&candata.data, 0, TWAI_FRAME_MAX_DLC);
    SetBankAndModuleText(text, rules.address_lowestExternalTemp);

    memcpy(&candata.data[0],&text,sizeof(text));   
    candata.identifier = id[11][mysettings.controllerID]; 

    memcpy(&data[11][mysettings.controllerID][0], candata.data,candata.dlc);       //copy calculated values to array

     // send to tx routine 
    if (mysettings.controllerNet != 1)
    {
      send_canbus_message(ptrFrame);
    }
  }
  // Max. cell temp id string [1]
  if (rules.address_highestExternalTemp < maximum_controller_cell_modules)
  {
    memset(&candata.data, 0, TWAI_FRAME_MAX_DLC);
    SetBankAndModuleText(text, rules.address_highestExternalTemp);

    memcpy(&candata.data[0],&text,sizeof(text));   
    candata.identifier = id[12][mysettings.controllerID]; 

    memcpy(&data[12][mysettings.controllerID][0], candata.data,candata.dlc);       //copy calculated values to array

     // send to tx routine  
    if (mysettings.controllerNet != 1)
    {
      send_canbus_message(ptrFrame);
    }
  }
  
  xSemaphoreGive(dataMutex[9]); 
  xSemaphoreGive(dataMutex[10]); 
  xSemaphoreGive(dataMutex[11]);
  xSemaphoreGive(dataMutex[12]); 
}