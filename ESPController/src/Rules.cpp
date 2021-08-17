#include "Rules.h"

void Rules::ClearValues()
{
    //Array to hold the total voltage of each bank/pack (in millivolts)
    for (uint8_t r = 0; r < maximum_number_of_banks; r++)
    {
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
}

//Looking at individual voltages and temperatures and sum up pack voltages.
void Rules::ProcessCell(uint8_t bank, uint8_t cellNumber, CellModuleInfo *c)
{
    if (c->valid == false)
    {
        invalidModuleCount++;
        return;
    }

    packvoltage[bank] += c->voltagemV;

    //If the voltage of the module is zero, we probably haven't requested it yet (which happens during power up)
    //so keep count so we don't accidentally trigger rules.
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
    }

    if (c->voltagemV < lowestCellVoltage)
    {
        lowestCellVoltage = c->voltagemV;
        address_LowestCellVoltage = cellNumber;
    }

    if (c->externalTemp != -40)
    {
        //Record that we do have at least one external temperature sensor on a module
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
    return highestvoltageinpack[bank] - lowestvoltageinpack[bank];
}

void Rules::ProcessBank(uint8_t bank)
{
    //Combine the voltages - work out the highest and lowest pack voltages
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

    //Only set the warning once
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

    //Only set error once
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
    //Emergency stop signal...
    rule_outcome[Rule::EmergencyStop] = emergencyStop;

    //Timer 1 and Timer 2
    rule_outcome[Rule::Timer1] = (mins >= value[Rule::Timer1] && mins <= hysteresisvalue[Rule::Timer1]);
    rule_outcome[Rule::Timer2] = (mins >= value[Rule::Timer2] && mins <= hysteresisvalue[Rule::Timer2]);

    if (currentMonitor->validReadings)
    {
        //Currents can be both positive and negative (depending on current flow, we ABS that to get an always POSITIVE number)
        uint32_t integercurrent = (uint32_t)(abs(currentMonitor->modbus.current) + (float)0.5);

        if (integercurrent > value[Rule::CurrentMonitorOverCurrentAmps] && rule_outcome[Rule::CurrentMonitorOverCurrentAmps] == false)
        {
            //CurrentMonitorOverCurrentAmps - TRIGGERED
            rule_outcome[Rule::CurrentMonitorOverCurrentAmps] = true;
        }
        else if (integercurrent < hysteresisvalue[Rule::CurrentMonitorOverCurrentAmps] && rule_outcome[Rule::CurrentMonitorOverCurrentAmps] == true)
        {
            //CurrentMonitorOverCurrentAmps - HYSTERESIS RESET
            rule_outcome[Rule::CurrentMonitorOverCurrentAmps] = false;
        }

        uint32_t integervoltagemV = (uint32_t)((currentMonitor->modbus.voltage * 1000.0) + (float)0.5);

        if (integervoltagemV > value[Rule::CurrentMonitorOverVoltage] && rule_outcome[Rule::CurrentMonitorOverVoltage] == false)
        {
            //Rule - CURRENT MONITOR Pack over voltage (mV)
            rule_outcome[Rule::CurrentMonitorOverVoltage] = true;
        }
        else if (integervoltagemV < hysteresisvalue[Rule::CurrentMonitorOverVoltage] && rule_outcome[Rule::CurrentMonitorOverVoltage] == true)
        {
            //Rule - CURRENT MONITOR Pack over voltage (mV) - HYSTERESIS RESET
            rule_outcome[Rule::CurrentMonitorOverVoltage] = false;
        }

        if (integervoltagemV < value[Rule::CurrentMonitorUnderVoltage] && rule_outcome[Rule::CurrentMonitorUnderVoltage] == false)
        {
            //Rule - CURRENT MONITOR Pack under voltage (mV)
            rule_outcome[Rule::CurrentMonitorUnderVoltage] = true;
        }
        else if (integervoltagemV > hysteresisvalue[Rule::CurrentMonitorUnderVoltage] && rule_outcome[Rule::CurrentMonitorUnderVoltage] == true)
        {
            //Rule - CURRENT MONITOR Pack under voltage (mV) - HYSTERESIS RESET
            rule_outcome[Rule::CurrentMonitorUnderVoltage] = false;
        }
    }
    else
    {
        //We don't have valid current monitor readings, so the rule is ALWAYS false
        rule_outcome[Rule::CurrentMonitorOverCurrentAmps] = false;
        rule_outcome[Rule::CurrentMonitorOverVoltage] = false;
        rule_outcome[Rule::CurrentMonitorUnderVoltage] = false;
    }

    //At least 1 module is zero volt - not a problem whilst we are in stabilizing start up mode
    if (zeroVoltageModuleCount > 0)
    {
        rule_outcome[Rule::ModuleOverVoltage] = false;
        rule_outcome[Rule::ModuleUnderVoltage] = false;
        rule_outcome[Rule::ModuleOverTemperatureInternal] = false;
        rule_outcome[Rule::ModuleUnderTemperatureInternal] = false;
        rule_outcome[Rule::ModuleOverTemperatureExternal] = false;
        rule_outcome[Rule::ModuleUnderTemperatureExternal] = false;

        //Abort processing any more rules until controller is stable/running state
        return;
    }

    //Only rules which are based on temperature or voltage should go below this point....

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
        //Rule Individual cell over voltage - TRIGGERED
        rule_outcome[Rule::ModuleOverVoltage] = true;
    }
    else if (highestCellVoltage < hysteresisvalue[Rule::ModuleOverVoltage] && rule_outcome[Rule::ModuleOverVoltage] == true)
    {
        //Rule Individual cell over voltage - HYSTERESIS RESET
        rule_outcome[Rule::ModuleOverVoltage] = false;
    }

    if (lowestCellVoltage < value[Rule::ModuleUnderVoltage] && rule_outcome[Rule::ModuleUnderVoltage] == false)
    {
        //Rule Individual cell under voltage (mV) - TRIGGERED
        rule_outcome[Rule::ModuleUnderVoltage] = true;
    }
    else if (lowestCellVoltage > hysteresisvalue[Rule::ModuleUnderVoltage] && rule_outcome[Rule::ModuleUnderVoltage] == true)
    {
        //Rule Individual cell under voltage (mV) - HYSTERESIS RESET
        rule_outcome[Rule::ModuleUnderVoltage] = false;
    }

    //These rules only fire if external temp sensor actually exists
    if (moduleHasExternalTempSensor)
    {
        //Doesn't cater for negative temperatures on rule (int8 vs uint32)
        if (((uint8_t)highestExternalTemp > value[Rule::ModuleOverTemperatureExternal]) && rule_outcome[Rule::ModuleOverTemperatureExternal] == false)
        {
            //Rule Individual cell over temperature (external probe)
            rule_outcome[Rule::ModuleOverTemperatureExternal] = true;
        }
        else if (((uint8_t)highestExternalTemp < hysteresisvalue[Rule::ModuleOverTemperatureExternal]) && rule_outcome[Rule::ModuleOverTemperatureExternal] == true)
        {
            //Rule Individual cell over temperature (external probe) - HYSTERESIS RESET
            rule_outcome[Rule::ModuleOverTemperatureExternal] = false;
        }

        //Doesn't cater for negative temperatures on rule (int8 vs uint32)
        if (((uint8_t)lowestExternalTemp < value[Rule::ModuleUnderTemperatureExternal]) && rule_outcome[Rule::ModuleUnderTemperatureExternal] == false)
        {
            //Rule Individual cell UNDER temperature (external probe)
            rule_outcome[Rule::ModuleUnderTemperatureExternal] = true;
        }
        else if (((uint8_t)lowestExternalTemp > hysteresisvalue[Rule::ModuleUnderTemperatureExternal]) && rule_outcome[Rule::ModuleUnderTemperatureExternal] == true)
        {
            //Rule Individual cell UNDER temperature (external probe) - HYSTERESIS RESET
            rule_outcome[Rule::ModuleUnderTemperatureExternal] = false;
        }
    }
    else
    {
        rule_outcome[Rule::ModuleOverTemperatureExternal] = false;
        rule_outcome[Rule::ModuleUnderTemperatureExternal] = false;
    }

    //Internal temperature monitoring and rules
    //Does not cope with negative temperatures on rule (int8 vs uint32)
    if (((uint8_t)highestInternalTemp > value[Rule::ModuleOverTemperatureInternal]) && rule_outcome[Rule::ModuleOverTemperatureInternal] == false)
    {
        //Rule Individual cell over temperature (Internal probe)
        rule_outcome[Rule::ModuleOverTemperatureInternal] = true;
    }
    else if (((uint8_t)highestInternalTemp < hysteresisvalue[Rule::ModuleOverTemperatureInternal]) && rule_outcome[Rule::ModuleOverTemperatureInternal] == true)
    {
        //Rule Individual cell over temperature (Internal probe) - HYSTERESIS RESET
        rule_outcome[Rule::ModuleOverTemperatureInternal] = false;
    }

    //Doesn't cater for negative temperatures on rule (int8 vs uint32)
    if (((uint8_t)lowestInternalTemp < value[Rule::ModuleUnderTemperatureInternal]) && rule_outcome[Rule::ModuleUnderTemperatureInternal] == false)
    {
        //Rule Individual cell UNDER temperature (Internal probe)
        rule_outcome[Rule::ModuleUnderTemperatureInternal] = true;
    }
    else if (((uint8_t)lowestInternalTemp > hysteresisvalue[Rule::ModuleUnderTemperatureInternal]) && rule_outcome[Rule::ModuleUnderTemperatureInternal] == true)
    {
        //Rule Individual cell UNDER temperature (Internal probe) - HYSTERESIS RESET
        rule_outcome[Rule::ModuleUnderTemperatureInternal] = false;
    }

    //While Pack voltages
    if (highestPackVoltage > value[Rule::BankOverVoltage] && rule_outcome[Rule::BankOverVoltage] == false)
    {
        //Rule - Pack over voltage (mV)
        rule_outcome[Rule::BankOverVoltage] = true;
    }
    else if (highestPackVoltage < hysteresisvalue[Rule::BankOverVoltage] && rule_outcome[Rule::BankOverVoltage] == true)
    {
        //Rule - Pack over voltage (mV) - HYSTERESIS RESET
        rule_outcome[Rule::BankOverVoltage] = false;
    }

    if (lowestPackVoltage < value[Rule::BankUnderVoltage] && rule_outcome[Rule::BankUnderVoltage] == false)
    {
        //Rule - Pack under voltage (mV)
        rule_outcome[Rule::BankUnderVoltage] = true;
    }
    else if (lowestPackVoltage > hysteresisvalue[Rule::BankUnderVoltage] && rule_outcome[Rule::BankUnderVoltage] == true)
    {
        //Rule - Pack under voltage (mV) - HYSTERESIS RESET
        rule_outcome[Rule::BankUnderVoltage] = false;
    }

    //Total up the active rules
    active_rule_count = 0;

    for (size_t i = 0; i < RELAY_RULES; i++)
    {
        if (rule_outcome[i] == true)
            active_rule_count++;
    }
}