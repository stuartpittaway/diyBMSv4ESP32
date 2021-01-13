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
}

//Looking at individual voltages and temperatures and sum up pack voltages.
void Rules::ProcessCell(uint8_t bank, CellModuleInfo *c)
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
    }

    if (c->voltagemV < lowestCellVoltage)
    {
        lowestCellVoltage = c->voltagemV;
    }

    if (c->externalTemp != -40)
    {
        //Record that we do have at least one external temperature sensor on a module
        moduleHasExternalTempSensor = true;

        if (c->externalTemp > highestExternalTemp)
        {
            highestExternalTemp = c->externalTemp;
        }

        if (c->externalTemp < lowestExternalTemp)
        {
            lowestExternalTemp = c->externalTemp;
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
    for (size_t i = 0; i < sizeof(WarningCodes); i++)
    {
        if (WarningCodes[i] == warncode) {
            //We already have the warning in the list, so skip
            break;
        }
        //Find a free space
        if (WarningCodes[i] == InternalWarningCode::NoWarning)
        {
            WarningCodes[i] = warncode;
            //SERIAL_DEBUG.print("Added warning ");            SERIAL_DEBUG.println(warncode);
            break;
        }
    }
}

void Rules::SetError(InternalErrorCode err)
{
    for (size_t i = 0; i < sizeof(ErrorCodes); i++)
    {
        if (ErrorCodes[i] == err)
        {
            //Error is already in the array
            break;
        }
        //Find a free space
        if (ErrorCodes[i] == InternalErrorCode::NoError)
        {
            rule_outcome[Rule::BMSError] = true;

            ErrorCodes[i] = err;

            SERIAL_DEBUG.print("Error State=");SERIAL_DEBUG.println(err);
            break;
        }
    }
}

void Rules::RunRules(
    uint32_t *value,
    uint32_t *hysteresisvalue,
    bool emergencyStop,
    uint16_t mins)
{
    //Emergency stop signal...
    rule_outcome[Rule::EmergencyStop] = emergencyStop;

    //Timer 1 and Timer 2
    rule_outcome[Rule::Timer1] = (mins >= value[Rule::Timer1] && mins <= hysteresisvalue[Rule::Timer1]);
    rule_outcome[Rule::Timer2] = (mins >= value[Rule::Timer2] && mins <= hysteresisvalue[Rule::Timer2]);

    //At least 1 module is zero volt - not a problem whilst we are in stabilizing start up mode
    if (zeroVoltageModuleCount > 0)
    {
        rule_outcome[Rule::Individualcellovervoltage] = false;
        rule_outcome[Rule::Individualcellundervoltage] = false;
        rule_outcome[Rule::IndividualcellovertemperatureExternal] = false;
        rule_outcome[Rule::IndividualcellundertemperatureExternal] = false;
        rule_outcome[Rule::ModuleOverTemperatureInternal] = false;
        rule_outcome[Rule::ModuleUnderTemperatureInternal] = false;

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

    if (highestCellVoltage > value[Rule::Individualcellovervoltage] && rule_outcome[Rule::Individualcellovervoltage] == false)
    {
        //Rule Individual cell over voltage - TRIGGERED
        rule_outcome[Rule::Individualcellovervoltage] = true;
    }
    else if (highestCellVoltage < hysteresisvalue[Rule::Individualcellovervoltage] && rule_outcome[Rule::Individualcellovervoltage] == true)
    {
        //Rule Individual cell over voltage - HYSTERESIS RESET
        rule_outcome[Rule::Individualcellovervoltage] = false;
    }

    if (lowestCellVoltage < value[Rule::Individualcellundervoltage] && rule_outcome[Rule::Individualcellundervoltage] == false)
    {
        //Rule Individual cell under voltage (mV) - TRIGGERED
        rule_outcome[Rule::Individualcellundervoltage] = true;
    }
    else if (lowestCellVoltage > hysteresisvalue[Rule::Individualcellundervoltage] && rule_outcome[Rule::Individualcellundervoltage] == true)
    {
        //Rule Individual cell under voltage (mV) - HYSTERESIS RESET
        rule_outcome[Rule::Individualcellundervoltage] = false;
    }

    //These rules only fire if external temp sensor actually exists
    if (moduleHasExternalTempSensor)
    {
        //Doesn't cater for negative temperatures on rule (int8 vs uint32)
        if (((uint8_t)highestExternalTemp > value[Rule::IndividualcellovertemperatureExternal]) && rule_outcome[Rule::IndividualcellovertemperatureExternal] == false)
        {
            //Rule Individual cell over temperature (external probe)
            rule_outcome[Rule::IndividualcellovertemperatureExternal] = true;
        }
        else if (((uint8_t)highestExternalTemp < hysteresisvalue[Rule::IndividualcellovertemperatureExternal]) && rule_outcome[Rule::IndividualcellovertemperatureExternal] == true)
        {
            //Rule Individual cell over temperature (external probe) - HYSTERESIS RESET
            rule_outcome[Rule::IndividualcellovertemperatureExternal] = false;
        }

        //Doesn't cater for negative temperatures on rule (int8 vs uint32)
        if (((uint8_t)lowestExternalTemp < value[Rule::IndividualcellundertemperatureExternal]) && rule_outcome[Rule::IndividualcellundertemperatureExternal] == false)
        {
            //Rule Individual cell UNDER temperature (external probe)
            rule_outcome[Rule::IndividualcellundertemperatureExternal] = true;
        }
        else if (((uint8_t)lowestExternalTemp > hysteresisvalue[Rule::IndividualcellundertemperatureExternal]) && rule_outcome[Rule::IndividualcellundertemperatureExternal] == true)
        {
            //Rule Individual cell UNDER temperature (external probe) - HYSTERESIS RESET
            rule_outcome[Rule::IndividualcellundertemperatureExternal] = false;
        }
    }
    else
    {
        rule_outcome[Rule::IndividualcellovertemperatureExternal] = false;
        rule_outcome[Rule::IndividualcellundertemperatureExternal] = false;
    }

    //Internal temperatyre monitoring and rules
        //Doesn't cater for negative temperatures on rule (int8 vs uint32)
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
    if (highestPackVoltage > value[Rule::PackOverVoltage] && rule_outcome[Rule::PackOverVoltage] == false)
    {
        //Rule - Pack over voltage (mV)
        rule_outcome[Rule::PackOverVoltage] = true;
    }
    else if (highestPackVoltage < hysteresisvalue[Rule::PackOverVoltage] && rule_outcome[Rule::PackOverVoltage] == true)
    {
        //Rule - Pack over voltage (mV) - HYSTERESIS RESET
        rule_outcome[Rule::PackOverVoltage] = false;
    }

    if (lowestPackVoltage < value[Rule::PackUnderVoltage] && rule_outcome[Rule::PackUnderVoltage] == false)
    {
        //Rule - Pack under voltage (mV)
        rule_outcome[Rule::PackUnderVoltage] = true;
    }
    else if (lowestPackVoltage > hysteresisvalue[Rule::PackUnderVoltage] && rule_outcome[Rule::PackUnderVoltage] == true)
    {
        //Rule - Pack under voltage (mV) - HYSTERESIS RESET
        rule_outcome[Rule::PackUnderVoltage] = false;
    }
}