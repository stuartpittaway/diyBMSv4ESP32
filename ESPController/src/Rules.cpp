#include "Rules.h"

void Rules::ClearValues()
{
    //Array to hold the total voltage of each bank/pack (in millivolts)
    for (uint8_t r = 0; r < maximum_number_of_banks; r++)
    {
        packvoltage[r] = 0;
    }

    highestPackVoltage = 0;
    lowestPackVoltage = 0xFFFFFFFF;
    highestCellVoltage = 0;
    lowestCellVoltage = 0xFFFF;
    highestExternalTemp = -127;
    lowestExternalTemp = 127;
    zeroVoltageModuleCount = 0;
    moduleHasExternalTempSensor = false;
}

//Looking at individual voltages and temperatures and sum up pack voltages.
void Rules::ProcessCell(uint8_t bank, CellModuleInfo *c)
{
    packvoltage[bank] += c->voltagemV;

    //If the voltage of the module is zero, we probably haven't requested it yet (which happens during power up)
    //so keep count so we don't accidentally trigger rules.
    if (c->voltagemV == 0)
    {
        zeroVoltageModuleCount++;
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

void Rules::SetError(InternalErrorCode err) {
    if (ErrorCode!=err) {
        ErrorCode=err;
        rule_outcome[RULE_BMSError] =(err!=InternalErrorCode::NoError);

        SERIAL_DEBUG.print("Error State="); SERIAL_DEBUG.println(err);
    }
}

void Rules::RunRules(
    uint32_t *value,
    uint32_t *hysteresisvalue,
    bool emergencyStop, 
    bool commsTimedOut, 
    uint16_t mins)
{
    //Emergency stop signal...
    rule_outcome[RULE_EmergencyStop] = emergencyStop;

    //Communications error...
    if (commsTimedOut) {
        SetError(InternalErrorCode::CommunicationsError);
    } else {
        SetError(InternalErrorCode::NoError);
    }

    //Timer 1 and Timer 2
    rule_outcome[RULE_Timer1] = (mins >= value[RULE_Timer1] && mins <= hysteresisvalue[RULE_Timer1]);
    rule_outcome[RULE_Timer2] = (mins >= value[RULE_Timer2] && mins <= hysteresisvalue[RULE_Timer2]);

    //At least 1 module is zero volt - not a problem whilst we are in stabilizing start up mode
    if (zeroVoltageModuleCount > 0)
    {
        rule_outcome[RULE_Individualcellovervoltage] = false;
        rule_outcome[RULE_Individualcellundervoltage] = false;
        rule_outcome[RULE_IndividualcellovertemperatureExternal] = false;
        rule_outcome[RULE_IndividualcellundertemperatureExternal] = false;

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

    if (highestCellVoltage > value[RULE_Individualcellovervoltage] && rule_outcome[RULE_Individualcellovervoltage] == false)
    {
        //Rule Individual cell over voltage - TRIGGERED
        rule_outcome[RULE_Individualcellovervoltage] = true;
    }
    else if (highestCellVoltage < hysteresisvalue[RULE_Individualcellovervoltage] && rule_outcome[RULE_Individualcellovervoltage] == true)
    {
        //Rule Individual cell over voltage - HYSTERESIS RESET
        rule_outcome[RULE_Individualcellovervoltage] = false;
    }

    if (lowestCellVoltage < value[RULE_Individualcellundervoltage] && rule_outcome[RULE_Individualcellundervoltage] == false)
    {
        //Rule Individual cell under voltage (mV) - TRIGGERED
        rule_outcome[RULE_Individualcellundervoltage] = true;
    }
    else if (lowestCellVoltage > hysteresisvalue[RULE_Individualcellundervoltage] && rule_outcome[RULE_Individualcellundervoltage] == true)
    {
        //Rule Individual cell under voltage (mV) - HYSTERESIS RESET
        rule_outcome[RULE_Individualcellundervoltage] = false;
    }

    //These rules only fire if external temp sensor actually exists
    if (moduleHasExternalTempSensor)
    {
        //Doesn't cater for negative temperatures on rule (int8 vs uint32)
        if (((uint8_t)highestExternalTemp > value[RULE_IndividualcellovertemperatureExternal]) && rule_outcome[RULE_IndividualcellovertemperatureExternal] == false)
        {
            //Rule Individual cell over temperature (external probe)
            rule_outcome[RULE_IndividualcellovertemperatureExternal] = true;
        }
        else if (((uint8_t)highestExternalTemp < hysteresisvalue[RULE_IndividualcellovertemperatureExternal]) && rule_outcome[RULE_IndividualcellovertemperatureExternal] == true)
        {
            //Rule Individual cell over temperature (external probe) - HYSTERESIS RESET
            rule_outcome[RULE_IndividualcellovertemperatureExternal] = false;
        }

        //Doesn't cater for negative temperatures on rule (int8 vs uint32)
        if (((uint8_t)lowestExternalTemp < value[RULE_IndividualcellundertemperatureExternal]) && rule_outcome[RULE_IndividualcellundertemperatureExternal] == false)
        {
            //Rule Individual cell UNDER temperature (external probe)
            rule_outcome[RULE_IndividualcellundertemperatureExternal] = true;
        }
        else if (((uint8_t)lowestExternalTemp > hysteresisvalue[RULE_IndividualcellundertemperatureExternal]) && rule_outcome[RULE_IndividualcellundertemperatureExternal] == true)
        {
            //Rule Individual cell UNDER temperature (external probe) - HYSTERESIS RESET
            rule_outcome[RULE_IndividualcellundertemperatureExternal] = false;
        }
    }
    else
    {
        rule_outcome[RULE_IndividualcellovertemperatureExternal] = false;
        rule_outcome[RULE_IndividualcellundertemperatureExternal] = false;
    }

    //While Pack voltages
    if (highestPackVoltage > value[RULE_PackOverVoltage] && rule_outcome[RULE_PackOverVoltage] == false)
    {
        //Rule - Pack over voltage (mV)
        rule_outcome[RULE_PackOverVoltage] = true;
    }
    else if (highestPackVoltage < hysteresisvalue[RULE_PackOverVoltage] && rule_outcome[RULE_PackOverVoltage] == true)
    {
        //Rule - Pack over voltage (mV) - HYSTERESIS RESET
        rule_outcome[RULE_PackOverVoltage] = false;
    }

    if (lowestPackVoltage < value[RULE_PackUnderVoltage] && rule_outcome[RULE_PackUnderVoltage] == false)
    {
        //Rule - Pack under voltage (mV)
        rule_outcome[RULE_PackUnderVoltage] = true;
    }
    else if (lowestPackVoltage > hysteresisvalue[RULE_PackUnderVoltage] && rule_outcome[RULE_PackUnderVoltage] == true)
    {
        //Rule - Pack under voltage (mV) - HYSTERESIS RESET
        rule_outcome[RULE_PackUnderVoltage] = false;
    }
}