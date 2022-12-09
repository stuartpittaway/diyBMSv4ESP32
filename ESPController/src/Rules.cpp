#define USE_ESP_IDF_LOG 1
static constexpr const char *const TAG = "diybms-rules";

#include "Rules.h"

// Its critical these are in the same order as "enum Rule", and occupy the same index position
const char *RuleTextDescription[] = {
    "EmergencyStop",
    "BMSError",
    "CurrentMonitorOverCurrentAmps",
    "ModuleOverVoltage",
    "ModuleUnderVoltage",
    "ModuleOverTemperatureInternal",
    "ModuleUnderTemperatureInternal",
    "ModuleOverTemperatureExternal",
    "ModuleUnderTemperatureExternal",
    "CurrentMonitorOverVoltage",
    "CurrentMonitorUnderVoltage",
    "BankOverVoltage",
    "BankUnderVoltage",
    "Timer2",
    "Timer1"};

void Rules::ClearValues()
{
    // Array to hold the total voltage of each bank/pack (in millivolts)
    for (uint8_t r = 0; r < maximum_number_of_banks; r++)
    {
        limitedpackvoltage[r] = 0;
        packvoltage[r] = 0;
        lowestvoltageinpack[r] = 0xFFFF;
        highestvoltageinpack[r] = 0;
    }

    highestPackVoltage = 0;
    lowestPackVoltage = 0xFFFFFFFF;
    highestCellVoltage = 0;
    lowestCellVoltage = 0xFFFF;
    highestExternalTemp = -127;
    lowestExternalTemp = 127;
    highestInternalTemp = -127;
    lowestInternalTemp = 127;
    zeroVoltageModuleCount = 0;
    invalidModuleCount = 0;
    moduleHasExternalTempSensor = false;

    address_LowestCellVoltage = maximum_controller_cell_modules + 1;
    address_lowestExternalTemp = maximum_controller_cell_modules + 1;
    address_highestExternalTemp = maximum_controller_cell_modules + 1;
    address_HighestCellVoltage = maximum_controller_cell_modules + 1;
    index_bank_HighestCellVoltage = 0;

    dynamicChargeVoltage = 0;
    dynamicChargeCurrent = 0;
}

// Looking at individual voltages and temperatures and sum up pack voltages.
void Rules::ProcessCell(uint8_t bank, uint8_t cellNumber, CellModuleInfo *c, uint16_t cellmaxmv)
{
    if (c->valid == false)
    {
        invalidModuleCount++;
        return;
    }

    packvoltage[bank] += c->voltagemV;
    limitedpackvoltage[bank] += min(c->voltagemV, cellmaxmv);

    // If the voltage of the module is zero, we probably haven't requested it yet (which happens during power up)
    // so keep count so we don't accidentally trigger rules.
    if (c->voltagemV == 0)
    {
        zeroVoltageModuleCount++;
    }

    if (c->voltagemV > highestvoltageinpack[bank])
    {
        highestvoltageinpack[bank] = c->voltagemV;
    }
    if (c->voltagemV < lowestvoltageinpack[bank])
    {
        lowestvoltageinpack[bank] = c->voltagemV;
    }

    if (c->voltagemV > highestCellVoltage)
    {
        highestCellVoltage = c->voltagemV;
        address_HighestCellVoltage = cellNumber;
        index_bank_HighestCellVoltage = bank;
    }

    if (c->voltagemV < lowestCellVoltage)
    {
        lowestCellVoltage = c->voltagemV;
        address_LowestCellVoltage = cellNumber;
    }

    if (c->externalTemp != -40)
    {
        // Record that we do have at least one external temperature sensor on a module
        moduleHasExternalTempSensor = true;

        if (c->externalTemp > highestExternalTemp)
        {
            highestExternalTemp = c->externalTemp;
            address_highestExternalTemp = cellNumber;
        }

        if (c->externalTemp < lowestExternalTemp)
        {
            lowestExternalTemp = c->externalTemp;
            address_lowestExternalTemp = cellNumber;
        }
    }

    if (c->internalTemp > highestInternalTemp)
    {
        highestInternalTemp = c->internalTemp;
    }

    if (c->externalTemp < lowestInternalTemp)
    {
        lowestInternalTemp = c->internalTemp;
    }
}

uint16_t Rules::VoltageRangeInBank(uint8_t bank)
{
    if (invalidModuleCount > 0)
        return 0;

    return highestvoltageinpack[bank] - lowestvoltageinpack[bank];
}

void Rules::ProcessBank(uint8_t bank)
{
    // Combine the voltages - work out the highest and lowest pack voltages
    if (packvoltage[bank] > highestPackVoltage)
    {
        highestPackVoltage = packvoltage[bank];
    }
    if (packvoltage[bank] < lowestPackVoltage)
    {
        lowestPackVoltage = packvoltage[bank];
    }
}

void Rules::SetWarning(InternalWarningCode warncode)
{
    if (warncode > MAXIMUM_InternalWarningCode)
        return;

    // Only set the warning once
    if (WarningCodes[warncode] != InternalWarningCode::NoWarning)
        return;

    WarningCodes[warncode] = warncode;
    numberOfActiveWarnings++;
    ESP_LOGI(TAG, "Set warning %i", warncode);
}

void Rules::SetError(InternalErrorCode err)
{
    if (err > MAXIMUM_InternalErrorCode)
        return;

    // Only set error once
    if (ErrorCodes[err] != InternalErrorCode::NoError)
        return;

    ErrorCodes[err] = err;
    numberOfActiveErrors++;
    ESP_LOGI(TAG, "Set error %i", err);
}

void Rules::RunRules(
    uint32_t *value,
    uint32_t *hysteresisvalue,
    bool emergencyStop,
    uint16_t mins,
    currentmonitoring_struct *currentMonitor)
{
    // Emergency stop signal...
    rule_outcome[Rule::EmergencyStop] = emergencyStop;

    // Timer 1 and Timer 2
    rule_outcome[Rule::Timer1] = (mins >= value[Rule::Timer1] && mins <= hysteresisvalue[Rule::Timer1]);
    rule_outcome[Rule::Timer2] = (mins >= value[Rule::Timer2] && mins <= hysteresisvalue[Rule::Timer2]);

    if (currentMonitor->validReadings)
    {
        // Currents can be both positive and negative (depending on current flow, we ABS that to get an always POSITIVE number)
        uint32_t integercurrent = (uint32_t)(abs(currentMonitor->modbus.current) + (float)0.5);

        if (integercurrent > value[Rule::CurrentMonitorOverCurrentAmps] && rule_outcome[Rule::CurrentMonitorOverCurrentAmps] == false)
        {
            // CurrentMonitorOverCurrentAmps - TRIGGERED
            rule_outcome[Rule::CurrentMonitorOverCurrentAmps] = true;
        }
        else if (integercurrent < hysteresisvalue[Rule::CurrentMonitorOverCurrentAmps] && rule_outcome[Rule::CurrentMonitorOverCurrentAmps] == true)
        {
            // CurrentMonitorOverCurrentAmps - HYSTERESIS RESET
            rule_outcome[Rule::CurrentMonitorOverCurrentAmps] = false;
        }

        uint32_t integervoltagemV = (uint32_t)((currentMonitor->modbus.voltage * 1000.0) + (float)0.5);

        if (integervoltagemV > value[Rule::CurrentMonitorOverVoltage] && rule_outcome[Rule::CurrentMonitorOverVoltage] == false)
        {
            // Rule - CURRENT MONITOR Pack over voltage (mV)
            rule_outcome[Rule::CurrentMonitorOverVoltage] = true;
        }
        else if (integervoltagemV < hysteresisvalue[Rule::CurrentMonitorOverVoltage] && rule_outcome[Rule::CurrentMonitorOverVoltage] == true)
        {
            // Rule - CURRENT MONITOR Pack over voltage (mV) - HYSTERESIS RESET
            rule_outcome[Rule::CurrentMonitorOverVoltage] = false;
        }

        if (integervoltagemV < value[Rule::CurrentMonitorUnderVoltage] && rule_outcome[Rule::CurrentMonitorUnderVoltage] == false)
        {
            // Rule - CURRENT MONITOR Pack under voltage (mV)
            rule_outcome[Rule::CurrentMonitorUnderVoltage] = true;
        }
        else if (integervoltagemV > hysteresisvalue[Rule::CurrentMonitorUnderVoltage] && rule_outcome[Rule::CurrentMonitorUnderVoltage] == true)
        {
            // Rule - CURRENT MONITOR Pack under voltage (mV) - HYSTERESIS RESET
            rule_outcome[Rule::CurrentMonitorUnderVoltage] = false;
        }
    }
    else
    {
        // We don't have valid current monitor readings, so the rule is ALWAYS false
        rule_outcome[Rule::CurrentMonitorOverCurrentAmps] = false;
        rule_outcome[Rule::CurrentMonitorOverVoltage] = false;
        rule_outcome[Rule::CurrentMonitorUnderVoltage] = false;
    }

    // At least 1 module is zero volt - not a problem whilst we are in stabilizing start up mode
    if (zeroVoltageModuleCount > 0)
    {
        rule_outcome[Rule::ModuleOverVoltage] = false;
        rule_outcome[Rule::ModuleUnderVoltage] = false;
        rule_outcome[Rule::ModuleOverTemperatureInternal] = false;
        rule_outcome[Rule::ModuleUnderTemperatureInternal] = false;
        rule_outcome[Rule::ModuleOverTemperatureExternal] = false;
        rule_outcome[Rule::ModuleUnderTemperatureExternal] = false;

        // Abort processing any more rules until controller is stable/running state
        return;
    }

    // Only rules which are based on temperature or voltage should go below this point....

    /*
  SERIAL_DEBUG.print("Rule Values: lowP=");
  SERIAL_DEBUG.print(lowestPackVoltage);
  SERIAL_DEBUG.print(" highP=");
  SERIAL_DEBUG.print(highestPackVoltage);

  SERIAL_DEBUG.print(" / lowC=");
  SERIAL_DEBUG.print(lowestCellVoltage);
  SERIAL_DEBUG.print(" highC=");
  SERIAL_DEBUG.print(highestCellVoltage);

  SERIAL_DEBUG.print(" / highTE=");
  SERIAL_DEBUG.print(highestExternalTemp);
  SERIAL_DEBUG.print(" lowTE=");
  SERIAL_DEBUG.print(lowestExternalTemp);
  SERIAL_DEBUG.println("");
*/

    if (highestCellVoltage > value[Rule::ModuleOverVoltage] && rule_outcome[Rule::ModuleOverVoltage] == false)
    {
        // Rule Individual cell over voltage - TRIGGERED
        rule_outcome[Rule::ModuleOverVoltage] = true;
    }
    else if (highestCellVoltage < hysteresisvalue[Rule::ModuleOverVoltage] && rule_outcome[Rule::ModuleOverVoltage] == true)
    {
        // Rule Individual cell over voltage - HYSTERESIS RESET
        rule_outcome[Rule::ModuleOverVoltage] = false;
    }

    if (lowestCellVoltage < value[Rule::ModuleUnderVoltage] && rule_outcome[Rule::ModuleUnderVoltage] == false)
    {
        // Rule Individual cell under voltage (mV) - TRIGGERED
        rule_outcome[Rule::ModuleUnderVoltage] = true;
    }
    else if (lowestCellVoltage > hysteresisvalue[Rule::ModuleUnderVoltage] && rule_outcome[Rule::ModuleUnderVoltage] == true)
    {
        // Rule Individual cell under voltage (mV) - HYSTERESIS RESET
        rule_outcome[Rule::ModuleUnderVoltage] = false;
    }

    // These rules only fire if external temp sensor actually exists
    if (moduleHasExternalTempSensor)
    {
        // Doesn't cater for negative temperatures on rule (int8 vs uint32)
        if (((uint8_t)highestExternalTemp > value[Rule::ModuleOverTemperatureExternal]) && rule_outcome[Rule::ModuleOverTemperatureExternal] == false)
        {
            // Rule Individual cell over temperature (external probe)
            rule_outcome[Rule::ModuleOverTemperatureExternal] = true;
        }
        else if (((uint8_t)highestExternalTemp < hysteresisvalue[Rule::ModuleOverTemperatureExternal]) && rule_outcome[Rule::ModuleOverTemperatureExternal] == true)
        {
            // Rule Individual cell over temperature (external probe) - HYSTERESIS RESET
            rule_outcome[Rule::ModuleOverTemperatureExternal] = false;
        }
        // Doesn't cater for negative temperatures on rule (int8 vs uint32)
        if (((uint8_t)lowestExternalTemp < value[Rule::ModuleUnderTemperatureExternal]) && rule_outcome[Rule::ModuleUnderTemperatureExternal] == false)
        {
            // Rule Individual cell UNDER temperature (external probe)
            rule_outcome[Rule::ModuleUnderTemperatureExternal] = true;
        }
        else if (((uint8_t)lowestExternalTemp > hysteresisvalue[Rule::ModuleUnderTemperatureExternal]) && rule_outcome[Rule::ModuleUnderTemperatureExternal] == true)
        {
            // Rule Individual cell UNDER temperature (external probe) - HYSTERESIS RESET
            rule_outcome[Rule::ModuleUnderTemperatureExternal] = false;
        }
    }
    else
    {
        rule_outcome[Rule::ModuleOverTemperatureExternal] = false;
        rule_outcome[Rule::ModuleUnderTemperatureExternal] = false;
    }

    // Internal temperature monitoring and rules
    // Does not cope with negative temperatures on rule (int8 vs uint32)
    if (((uint8_t)highestInternalTemp > value[Rule::ModuleOverTemperatureInternal]) && rule_outcome[Rule::ModuleOverTemperatureInternal] == false)
    {
        // Rule Individual cell over temperature (Internal probe)
        rule_outcome[Rule::ModuleOverTemperatureInternal] = true;
    }
    else if (((uint8_t)highestInternalTemp < hysteresisvalue[Rule::ModuleOverTemperatureInternal]) && rule_outcome[Rule::ModuleOverTemperatureInternal] == true)
    {
        // Rule Individual cell over temperature (Internal probe) - HYSTERESIS RESET
        rule_outcome[Rule::ModuleOverTemperatureInternal] = false;
    }

    // Doesn't cater for negative temperatures on rule (int8 vs uint32)
    if (((uint8_t)lowestInternalTemp < value[Rule::ModuleUnderTemperatureInternal]) && rule_outcome[Rule::ModuleUnderTemperatureInternal] == false)
    {
        // Rule Individual cell UNDER temperature (Internal probe)
        rule_outcome[Rule::ModuleUnderTemperatureInternal] = true;
    }
    else if (((uint8_t)lowestInternalTemp > hysteresisvalue[Rule::ModuleUnderTemperatureInternal]) && rule_outcome[Rule::ModuleUnderTemperatureInternal] == true)
    {
        // Rule Individual cell UNDER temperature (Internal probe) - HYSTERESIS RESET
        rule_outcome[Rule::ModuleUnderTemperatureInternal] = false;
    }

    // While Pack voltages
    if (highestPackVoltage > value[Rule::BankOverVoltage] && rule_outcome[Rule::BankOverVoltage] == false)
    {
        // Rule - Pack over voltage (mV)
        rule_outcome[Rule::BankOverVoltage] = true;
    }
    else if (highestPackVoltage < hysteresisvalue[Rule::BankOverVoltage] && rule_outcome[Rule::BankOverVoltage] == true)
    {
        // Rule - Pack over voltage (mV) - HYSTERESIS RESET
        rule_outcome[Rule::BankOverVoltage] = false;
    }

    if (lowestPackVoltage < value[Rule::BankUnderVoltage] && rule_outcome[Rule::BankUnderVoltage] == false)
    {
        // Rule - Pack under voltage (mV)
        rule_outcome[Rule::BankUnderVoltage] = true;
    }
    else if (lowestPackVoltage > hysteresisvalue[Rule::BankUnderVoltage] && rule_outcome[Rule::BankUnderVoltage] == true)
    {
        // Rule - Pack under voltage (mV) - HYSTERESIS RESET
        rule_outcome[Rule::BankUnderVoltage] = false;
    }

    // Total up the active rules
    active_rule_count = 0;

    for (size_t i = 0; i < RELAY_RULES; i++)
    {
        if (rule_outcome[i] == true)
            active_rule_count++;
    }
}

bool Rules::SharedChargingDischargingRules(diybms_eeprom_settings *mysettings)
{
    if (mysettings->canbusprotocol == CanBusProtocolEmulation::CANBUS_DISABLED)
        return false;

    if (invalidModuleCount > 0)
        return false;
    if (moduleHasExternalTempSensor == false)
        return false;
    // Any errors, stop charge
    if (numberOfActiveErrors > 0)
        return false;

    // Battery high voltage alarm
    if (rule_outcome[Rule::BankOverVoltage])
        return false;
    // Low voltage alarm
    if (rule_outcome[Rule::BankUnderVoltage])
        return false;
    // Battery high temperature alarm
    if (rule_outcome[Rule::ModuleOverTemperatureExternal])
        return false;
    // Battery low temperature alarm
    if (rule_outcome[Rule::ModuleUnderTemperatureExternal])
        return false;

    return true;
}
bool Rules::IsChargeAllowed(diybms_eeprom_settings *mysettings)
{
    if (SharedChargingDischargingRules(mysettings) == false)
        return false;

    if (mysettings->preventcharging == true)
        return false;

    if (lowestExternalTemp<mysettings->chargetemplow | highestExternalTemp> mysettings->chargetemphigh)
    {
        // Stop charge - temperature out of range
        // ESP_LOGW(TAG, "Stop charge - temperature out of range");
        return false;
    }

    // chargevolt = 560
    if ((highestPackVoltage / 100) > mysettings->chargevolt)
        return false;

    // Individual cell over voltage
    if (highestCellVoltage > mysettings->cellmaxmv)
        return false;

    return true;
}
bool Rules::IsDischargeAllowed(diybms_eeprom_settings *mysettings)
{
    if (SharedChargingDischargingRules(mysettings) == false)
        return false;

    if (mysettings->preventdischarge == true)
        return false;

    // Check battery temperature against charge/discharge parameters
    if (lowestExternalTemp<mysettings->dischargetemplow | highestExternalTemp> mysettings->dischargetemphigh)
    {
        // ESP_LOGW(TAG, "Stop discharge - temperature out of range");
        return false;
    }

    if ((lowestPackVoltage / 100) < mysettings->dischargevolt)
        return false;

    // Individual cell under voltage
    if (lowestCellVoltage < mysettings->cellminmv)
        return false;

    return true;
}

// Charge voltage calculated by CalculateDynamicChargeVoltage
// Scale 0.1 = 567 = 56.7V
uint16_t Rules::DynamicChargeVoltage()
{
    return dynamicChargeVoltage;
}
// Charge current calculated by CalculateDynamicChargeCurrent
// Scale 0.1 = 123 = 12.3Amps
int16_t Rules::DynamicChargeCurrent()
{
    return dynamicChargeCurrent;
}

// Apply "dynamic" charge current rules
// **TODO** At present, this is a fixed value based on user
void Rules::CalculateDynamicChargeCurrent(diybms_eeprom_settings *mysettings, CellModuleInfo *cellarray)
{
    // Remember dynamicChargeCurrent scale is 0.1
    dynamicChargeCurrent = mysettings->chargecurrent;
}

// Apply "dynamic" charge voltage rules
// This will always return a charge voltage - its the calling functions responsibility
// to check "IsChargeAllowed" function and take necessary action.
// Thanks to Matthias U (Smurfix) for the ideas and pseudo code https://community.openenergymonitor.org/u/smurfix/
// Output is cached in variable dynamicChargeVoltage as its used in multiple places
void Rules::CalculateDynamicChargeVoltage(diybms_eeprom_settings *mysettings, CellModuleInfo *cellarray)
{
    if (!mysettings->dynamiccharge)
    {
        // Its switched off, use default voltage
        dynamicChargeVoltage = mysettings->chargevolt;
        return;
    }

    // If the cells are all below the knee voltage, just carry on as normal
    /*
    if (highestCellVoltage <= mysettings->kneemv)
    {
        dynamicChargeVoltage = mysettings->chargevolt;
        return;
    }
    // Some cells are above the knee voltage....
    */


    // Are any cells at or over the maximum allowed? (panic!)
    if (highestCellVoltage >= mysettings->cellmaxmv)
    {
        ESP_LOGW(TAG, "Cell V>Max");
        // *** Stop charging, we are at or above maximum ***

        // Find the lowest "limited" pack voltage
        uint32_t lowest = 0xFFFFFFFF;
        for (uint8_t r = 0; r < mysettings->totalNumberOfBanks; r++)
        {
            if (limitedpackvoltage[r] < lowest)
            {
                lowest = limitedpackvoltage[r];
            }
        }

        lowest = lowest / 100;
        ESP_LOGD(TAG, "lowest=%u", lowest);

        // Return MIN of either the "lowest pack voltage" or the "user specified value"
        dynamicChargeVoltage = min(lowest, (uint32_t)mysettings->chargevolt);
        return;
    }

    // At this point all cell voltages are UNDER the target cellmaxmv

    // This is unlikely to work if the value is changed from 1 (an integer)
    const uint16_t UniformDerating = 1;

    // Calculate voltage range
    uint32_t R = min((mysettings->cellmaxmv - highestCellVoltage) * UniformDerating, (mysettings->cellmaxmv - mysettings->kneemv) / 3);
    ESP_LOGD(TAG, "R=%u", R);

    // We use the pack with the highest cell voltage for these calculations - although hopefully all packs are very similar :-)
    uint32_t S = packvoltage[index_bank_HighestCellVoltage];
    ESP_LOGD(TAG, "S=%u", S);

    uint32_t HminusR = (uint32_t)highestCellVoltage - R;
    ESP_LOGD(TAG, "HminusR=%u", HminusR);

    uint32_t MminusH = mysettings->cellmaxmv - highestCellVoltage;
    ESP_LOGD(TAG, "MminusH=%u", MminusH);

    // Jump to start of cells in the correct bank.
    uint8_t cellid = index_bank_HighestCellVoltage * mysettings->totalNumberOfSeriesModules;
    for (uint8_t i = 0; i < mysettings->totalNumberOfSeriesModules; i++)
    {
        if (cellarray[i].voltagemV >= HminusR)
        {
            S += ((MminusH) * (cellarray[i].voltagemV - (HminusR)) / R);
        }

        ESP_LOGD(TAG, "id=%u, V=%u, S=%u", cellid, cellarray[i].voltagemV, S);
    }

    // Scale down to 0.1V
    S = S / 100;
    ESP_LOGD(TAG, "S=%u", S);

    // Return MIN of either the above calculation or the "user specified value"
    dynamicChargeVoltage = min(S, (uint32_t)mysettings->chargevolt);
}

// Return SoC value after applying SOCFORCELOW and SOCOVERRIDE settings
// also limits output range between 0 and 100.
// SoC is rounded down to nearest integer
uint16_t Rules::StateOfChargeWithRulesApplied(diybms_eeprom_settings *mysettings, float realSOC)
{
    uint16_t value = floor(realSOC);

    // Deliberately force SoC to be reported as 2%, to trick external CANBUS devices into trickle charging
    if (mysettings->socforcelow)
    {
        value = 2;
    }

    if (mysettings->socoverride)
    {
        if (value > 90)
        {
            // Force inverter SoC reading to 90%, this should force it to continue charging the battery
            // this is helpful when first commissioning as most inverters stop charging at 100% SOC
            // even though the battery may not be full, and the DIYBMS current monitor has not learnt capacity yet.
            // This function should not be left permanently switched on - you could damage the battery.
            value = 90;
        }
        if (value < 21)
        {
            // Force minimum of 21% - some inverters (SoFAR) will force charge a battery lower than
            // this level limiting the charge current to 500W
            value = 21;
        }
        
        // Limit to 100% maximum, DIYBMS current monitor can go above 100%, so don't confuse inverter/chargers
        if (value > 100)
        {
            value = 100;
        }
    }

    return value;
}