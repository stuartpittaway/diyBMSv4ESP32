#define USE_ESP_IDF_LOG 1
static constexpr const char *const TAG = "diybms-rules";

#include "Rules.h"

// Its critical these are in the same order as "enum Rule", and occupy the same index position
const std::array<std::string, 1 + MAXIMUM_RuleNumber> Rules::RuleTextDescription = {
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
    "BankRange",
    "Timer2",
    "Timer1"};

const std::array<std::string, 1 + MAXIMUM_InternalWarningCode> Rules::InternalWarningCodeDescription =
    {
        "NoWarning",
        "ModuleInconsistantBypassVoltage",
        "ModuleInconsistantBypassTemperature",
        "ModuleInconsistantCodeVersion",
        "ModuleInconsistantBoardRevision",
        "LoggingEnabledNoSDCard",
        "AVRProgrammingMode",
        "ChargePrevented",
        "DischargePrevented",
        "NoExternalTempSensor"};

const std::array<std::string, 1 + MAXIMUM_InternalErrorCode> Rules::InternalErrorCodeDescription =
    {
        "NoError",
        "CommunicationsError",
        "ModuleCountMismatch",
        "TooManyModules",
        "WaitingForModulesToReply",
        "ZeroVoltModule",
        "ControllerMemoryError",
        "ErrorEmergencyStop"};

void Rules::ClearValues()
{
    // Clear arrays
    bankvoltage.fill(0);
    limitedbankvoltage.fill(0);
    LowestCellVoltageInBank.fill(0xFFFF);
    HighestCellVoltageInBank.fill(0);

    highestBankVoltage = 0;
    address_highestBankVoltage = maximum_number_of_banks+1;
    lowestBankVoltage = 0xFFFFFFFF;
    address_lowestBankVoltage = maximum_number_of_banks+1;
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

// Looking at individual voltages and temperatures and sum up Bank voltages.
void Rules::ProcessCell(uint8_t bank, uint8_t cellNumber, const CellModuleInfo *c, uint16_t cellmaxmv)
{
    if (c->valid == false)
    {
        invalidModuleCount++;
        return;
    }

    bankvoltage.at(bank) += c->voltagemV;
    limitedbankvoltage.at(bank) += min(c->voltagemV, cellmaxmv);

    // If the voltage of the module is zero, we probably haven't requested it yet (which happens during power up)
    // so keep count so we don't accidentally trigger rules.
    if (c->voltagemV == 0)
    {
        zeroVoltageModuleCount++;
    }

    if (c->voltagemV > HighestCellVoltageInBank.at(bank))
    {
        HighestCellVoltageInBank.at(bank) = c->voltagemV;
    }
    if (c->voltagemV < LowestCellVoltageInBank.at(bank))
    {
        LowestCellVoltageInBank.at(bank) = c->voltagemV;
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

uint16_t Rules::VoltageRangeInBank(uint8_t bank) const
{
    if (invalidModuleCount > 0)
        return 0;

    return HighestCellVoltageInBank.at(bank) - LowestCellVoltageInBank.at(bank);
}

void Rules::ProcessBank(uint8_t bank)
{
    // Combine the voltages - work out the highest and lowest Bank voltages
    if (bankvoltage.at(bank) > highestBankVoltage)
    {
        highestBankVoltage = bankvoltage.at(bank);
        address_highestBankVoltage = bank;
    }
    if (bankvoltage.at(bank) < lowestBankVoltage)
    {
        lowestBankVoltage = bankvoltage.at(bank);
        address_lowestBankVoltage = bank;
    }

    if (VoltageRangeInBank(bank) > highestBankRange)
    {
        highestBankRange = VoltageRangeInBank(bank);
    }
}

void Rules::SetWarning(InternalWarningCode warncode)
{
    if (warncode > MAXIMUM_InternalWarningCode)
        return;

    // Only set the warning once
    if (WarningCodes.at(warncode) != InternalWarningCode::NoWarning)
        return;

    WarningCodes.at(warncode) = warncode;
    numberOfActiveWarnings++;
    ESP_LOGI(TAG, "Set warning %i:%s", warncode, InternalWarningCodeDescription.at(warncode).c_str());
}

void Rules::SetError(InternalErrorCode err)
{
    if (err > MAXIMUM_InternalErrorCode) {
        ESP_LOGE(TAG, "Error code doesn't exist %i",err);
        return;
    }

    // Only set error once
    if (ErrorCodes.at(err) != InternalErrorCode::NoError)
        return;

    ErrorCodes.at(err) = err;
    numberOfActiveErrors++;
    ESP_LOGI(TAG, "Set error %i:%s", err, InternalErrorCodeDescription.at(err).c_str());
}

/// @brief Run rules against cell/bank data
/// @param value
/// @param hysteresisvalue
/// @param emergencyStop TRUE is ESTOP is triggered
/// @param mins
/// @param currentMonitor
void Rules::RunRules(
    const int32_t *value,
    const int32_t *hysteresisvalue,
    bool emergencyStop,
    uint16_t mins,
    const currentmonitoring_struct *currentMonitor)
{
    // Emergency stop signal...
    setRuleStatus(Rule::EmergencyStop, emergencyStop);

    // Timer 1 and Timer 2
    setRuleStatus(Rule::Timer1, (mins >= value[Rule::Timer1] && mins <= hysteresisvalue[Rule::Timer1]));
    setRuleStatus(Rule::Timer2, (mins >= value[Rule::Timer2] && mins <= hysteresisvalue[Rule::Timer2]));

    if (currentMonitor->validReadings)
    {
        // Currents can be both positive and negative (depending on current flow, we ABS that to get an always POSITIVE number)
        auto integercurrent = (uint32_t)(abs(currentMonitor->modbus.current) + (float)0.5);

        if (integercurrent > value[Rule::CurrentMonitorOverCurrentAmps] && ruleOutcome(Rule::CurrentMonitorOverCurrentAmps) == false)
        {
            // CurrentMonitorOverCurrentAmps - TRIGGERED
            setRuleStatus(Rule::CurrentMonitorOverCurrentAmps, true);
        }
        else if (integercurrent < hysteresisvalue[Rule::CurrentMonitorOverCurrentAmps] && ruleOutcome(Rule::CurrentMonitorOverCurrentAmps) == true)
        {
            // CurrentMonitorOverCurrentAmps - HYSTERESIS RESET
            setRuleStatus(Rule::CurrentMonitorOverCurrentAmps, false);
        }

        auto integervoltagemV = (uint32_t)((currentMonitor->modbus.voltage * 1000.0) + (float)0.5);

        if (integervoltagemV > value[Rule::CurrentMonitorOverVoltage] && ruleOutcome(Rule::CurrentMonitorOverVoltage) == false)
        {
            // Rule - CURRENT MONITOR Bank over voltage (mV)
            setRuleStatus(Rule::CurrentMonitorOverVoltage, true);
        }
        else if (integervoltagemV < hysteresisvalue[Rule::CurrentMonitorOverVoltage] && ruleOutcome(Rule::CurrentMonitorOverVoltage) == true)
        {
            // Rule - CURRENT MONITOR Bank over voltage (mV) - HYSTERESIS RESET
            setRuleStatus(Rule::CurrentMonitorOverVoltage, false);
        }

        if (integervoltagemV < value[Rule::CurrentMonitorUnderVoltage] && ruleOutcome(Rule::CurrentMonitorUnderVoltage) == false)
        {
            // Rule - CURRENT MONITOR Bank under voltage (mV)
            setRuleStatus(Rule::CurrentMonitorUnderVoltage, true);
        }
        else if (integervoltagemV > hysteresisvalue[Rule::CurrentMonitorUnderVoltage] && ruleOutcome(Rule::CurrentMonitorUnderVoltage) == true)
        {
            // Rule - CURRENT MONITOR Bank under voltage (mV) - HYSTERESIS RESET
            setRuleStatus(Rule::CurrentMonitorUnderVoltage, false);
        }
    }
    else
    {
        // We don't have valid current monitor readings, so the rule is ALWAYS false
        setRuleStatus(Rule::CurrentMonitorOverCurrentAmps, false);
        setRuleStatus(Rule::CurrentMonitorOverVoltage, false);
        setRuleStatus(Rule::CurrentMonitorUnderVoltage, false);
    }

    // At least 1 module is zero volt - not a problem whilst we are in stabilizing start up mode
    if (zeroVoltageModuleCount > 0 || invalidModuleCount > 0)
    {
        setRuleStatus(Rule::ModuleOverVoltage, false);
        setRuleStatus(Rule::ModuleUnderVoltage, false);
        setRuleStatus(Rule::ModuleOverTemperatureInternal, false);
        setRuleStatus(Rule::ModuleUnderTemperatureInternal, false);
        setRuleStatus(Rule::ModuleOverTemperatureExternal, false);
        setRuleStatus(Rule::ModuleUnderTemperatureExternal, false);

        // Abort processing any more rules until controller is stable/running state
        return;
    }

    // Only rules which are based on temperature or voltage should go below this point....
    if (highestCellVoltage > value[Rule::ModuleOverVoltage] && ruleOutcome(Rule::ModuleOverVoltage) == false)
    {
        // Rule Individual cell over voltage - TRIGGERED
        setRuleStatus(Rule::ModuleOverVoltage, true);
    }
    else if (highestCellVoltage < hysteresisvalue[Rule::ModuleOverVoltage] && ruleOutcome(Rule::ModuleOverVoltage) == true)
    {
        // Rule Individual cell over voltage - HYSTERESIS RESET
        setRuleStatus(Rule::ModuleOverVoltage, false);
    }

    if (lowestCellVoltage < value[Rule::ModuleUnderVoltage] && ruleOutcome(Rule::ModuleUnderVoltage) == false)
    {
        // Rule Individual cell under voltage (mV) - TRIGGERED
        setRuleStatus(Rule::ModuleUnderVoltage, true);
    }
    else if (lowestCellVoltage > hysteresisvalue[Rule::ModuleUnderVoltage] && ruleOutcome(Rule::ModuleUnderVoltage) == true)
    {
        // Rule Individual cell under voltage (mV) - HYSTERESIS RESET
        setRuleStatus(Rule::ModuleUnderVoltage, false);
    }

    // These rules only fire if external temp sensor actually exists
    if (moduleHasExternalTempSensor)
    {
        // Doesn't cater for negative temperatures on rule (int8 vs uint32)
        if ((highestExternalTemp > value[Rule::ModuleOverTemperatureExternal]) && ruleOutcome(Rule::ModuleOverTemperatureExternal) == false)
        {
            // Rule Individual cell over temperature (external probe)
            setRuleStatus(Rule::ModuleOverTemperatureExternal, true);
        }
        else if ((highestExternalTemp < hysteresisvalue[Rule::ModuleOverTemperatureExternal]) && ruleOutcome(Rule::ModuleOverTemperatureExternal) == true)
        {
            // Rule Individual cell over temperature (external probe) - HYSTERESIS RESET
            setRuleStatus(Rule::ModuleOverTemperatureExternal, false);
        }
        // Doesn't cater for negative temperatures on rule (int8 vs uint32)
        if ((lowestExternalTemp < value[Rule::ModuleUnderTemperatureExternal]) && ruleOutcome(Rule::ModuleUnderTemperatureExternal) == false)
        {
            // Rule Individual cell UNDER temperature (external probe)
            setRuleStatus(Rule::ModuleUnderTemperatureExternal, true);
        }
        else if ((lowestExternalTemp > hysteresisvalue[Rule::ModuleUnderTemperatureExternal]) && ruleOutcome(Rule::ModuleUnderTemperatureExternal) == true)
        {
            // Rule Individual cell UNDER temperature (external probe) - HYSTERESIS RESET
            setRuleStatus(Rule::ModuleUnderTemperatureExternal, false);
        }
    }
    else
    {
        setRuleStatus(Rule::ModuleOverTemperatureExternal, false);
        setRuleStatus(Rule::ModuleUnderTemperatureExternal, false);
    }

    // Internal temperature monitoring and rules
    // Does not cope with negative temperatures on rule (int8 vs uint32)
    if ((highestInternalTemp > value[Rule::ModuleOverTemperatureInternal]) && ruleOutcome(Rule::ModuleOverTemperatureInternal) == false)
    {
        // Rule Individual cell over temperature (Internal probe)
        setRuleStatus(Rule::ModuleOverTemperatureInternal, true);
    }
    else if ((highestInternalTemp < hysteresisvalue[Rule::ModuleOverTemperatureInternal]) && ruleOutcome(Rule::ModuleOverTemperatureInternal) == true)
    {
        // Rule Individual cell over temperature (Internal probe) - HYSTERESIS RESET
        setRuleStatus(Rule::ModuleOverTemperatureInternal, false);
    }

    // Doesn't cater for negative temperatures on rule (int8 vs uint32)
    if ((lowestInternalTemp < value[Rule::ModuleUnderTemperatureInternal]) && ruleOutcome(Rule::ModuleUnderTemperatureInternal) == false)
    {
        // Rule Individual cell UNDER temperature (Internal probe)
        setRuleStatus(Rule::ModuleUnderTemperatureInternal, true);
    }
    else if ((lowestInternalTemp > hysteresisvalue[Rule::ModuleUnderTemperatureInternal]) && ruleOutcome(Rule::ModuleUnderTemperatureInternal) == true)
    {
        // Rule Individual cell UNDER temperature (Internal probe) - HYSTERESIS RESET
        setRuleStatus(Rule::ModuleUnderTemperatureInternal, false);
    }

    // Whole Bank voltages
    if (highestBankVoltage > value[Rule::BankOverVoltage] && ruleOutcome(Rule::BankOverVoltage) == false)
    {
        // Rule - Bank over voltage (mV)
        setRuleStatus(Rule::BankOverVoltage, true);
    }
    else if (highestBankVoltage < hysteresisvalue[Rule::BankOverVoltage] && ruleOutcome(Rule::BankOverVoltage) == true)
    {
        // Rule - Bank over voltage (mV) - HYSTERESIS RESET
        setRuleStatus(Rule::BankOverVoltage, false);
    }

    if (lowestBankVoltage < value[Rule::BankUnderVoltage] && ruleOutcome(Rule::BankUnderVoltage) == false)
    {
        // Rule - Bank under voltage (mV)
        setRuleStatus(Rule::BankUnderVoltage, true);
    }
    else if (lowestBankVoltage > hysteresisvalue[Rule::BankUnderVoltage] && ruleOutcome(Rule::BankUnderVoltage) == true)
    {
        // Rule - Bank under voltage (mV) - HYSTERESIS RESET
        setRuleStatus(Rule::BankUnderVoltage, false);
    }

    // While Bank voltages
    if (highestBankRange > value[Rule::BankRange] && ruleOutcome(Rule::BankRange) == false)
    {
        // Rule - Bank Range
        setRuleStatus(Rule::BankRange, true);
    }
    else if (highestBankRange < hysteresisvalue[Rule::BankRange] && ruleOutcome(Rule::BankRange) == true)
    {
        // Rule - Bank Range - HYSTERESIS RESET
        setRuleStatus(Rule::BankRange, false);
    }

    // Total up the active rules
    active_rule_count = 0;

    for (const auto &v : rule_outcome)
    {
        if (v == true)
            active_rule_count++;
    }
}

bool Rules::SharedChargingDischargingRules(const diybms_eeprom_settings *mysettings)
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

    // Battery high temperature alarm
    if (ruleOutcome(Rule::ModuleOverTemperatureExternal))
        return false;
    // Battery low temperature alarm
    if (ruleOutcome(Rule::ModuleUnderTemperatureExternal))
        return false;

    return true;
}
bool Rules::IsChargeAllowed(const diybms_eeprom_settings *mysettings)
{
    if (SharedChargingDischargingRules(mysettings) == false)
        return false;

    // Battery high voltage alarm - stop charging
    if (ruleOutcome(Rule::BankOverVoltage))
        return false;

    // Battery high voltage alarm - stop charging
    if (ruleOutcome(Rule::CurrentMonitorOverVoltage))
        return false;

    if (mysettings->preventcharging == true)
        return false;

    if ((lowestExternalTemp < mysettings->chargetemplow) || (highestExternalTemp > mysettings->chargetemphigh))
    {
        // Stop charge - temperature out of range
        // ESP_LOGW(TAG, "Stop charge - temperature out of range");
        return false;
    }

    // chargevolt = 560
    if ((highestBankVoltage / 100) > mysettings->chargevolt)
        return false;

    // Individual cell over voltage
    if (highestCellVoltage > mysettings->cellmaxmv)
        return false;

    // Prevent charging if we stopped it (after float/absorb)
    if (chargemode == ChargingMode::stopped)
        return false;

    return true;
}
bool Rules::IsDischargeAllowed(const diybms_eeprom_settings *mysettings)
{
    if (SharedChargingDischargingRules(mysettings) == false)
        return false;

    // Low voltage alarm - stop discharge
    if (ruleOutcome(Rule::BankUnderVoltage))
        return false;

    // Low voltage alarm - stop charging
    if (ruleOutcome(Rule::CurrentMonitorUnderVoltage))
        return false;

    if (mysettings->preventdischarge == true)
        return false;

    // Check battery temperature against charge/discharge parameters
    if ((lowestExternalTemp < mysettings->dischargetemplow) || (highestExternalTemp > mysettings->dischargetemphigh))
    {
        // ESP_LOGW(TAG, "Stop discharge - temperature out of range");
        return false;
    }

    if ((lowestBankVoltage / 100) < mysettings->dischargevolt)
        return false;

    // Individual cell under voltage
    if (lowestCellVoltage < mysettings->cellminmv)
        return false;

    return true;
}

// Charge voltage calculated by CalculateDynamicChargeVoltage
// Scale 0.1 = 567 = 56.7V
uint16_t Rules::DynamicChargeVoltage() const
{
    return dynamicChargeVoltage;
}
// Charge current calculated by CalculateDynamicChargeCurrent
// Scale 0.1 = 123 = 12.3Amps
int16_t Rules::DynamicChargeCurrent() const
{
    return dynamicChargeCurrent;
}

// Apply "dynamic" charge current rules
void Rules::CalculateDynamicChargeCurrent(const diybms_eeprom_settings *mysettings)
{
    // Remember dynamicChargeCurrent scale is 0.1
    dynamicChargeCurrent = mysettings->chargecurrent;

    if (!mysettings->dynamiccharge || mysettings->canbusprotocol == CanBusProtocolEmulation::CANBUS_DISABLED)
    {
        // Its switched off, use default
        return;
    }

    if (highestCellVoltage < mysettings->kneemv)
    {
        // Voltage of cell is below the knee voltage, so use full current
        return;
    }

    double value1 = mysettings->current_value1 / 10.0F;
    double value2 = mysettings->current_value2 / 10.0F;
    ESP_LOGD(TAG, "value1=%f", value1);
    ESP_LOGD(TAG, "value2=%f", value2);

    // This is always 1 :-)
    double knee_voltage = 0 / 100.0F;
    double at_knee = pow(value1, knee_voltage * pow(knee_voltage, value2));
    ESP_LOGD(TAG, "at_knee=%f", at_knee);

    double target_cell_voltage = (mysettings->cellmaxmv - mysettings->kneemv) / 100.0F;
    double at_target_cell_voltage = pow(value1, target_cell_voltage * pow(target_cell_voltage, value2));
    ESP_LOGD(TAG, "at_target_cell_voltage=%f", at_target_cell_voltage);

    double actual_cell_voltage = (highestCellVoltage - mysettings->kneemv) / 100.0F;
    double at_actual_cell_voltage = pow(value1, actual_cell_voltage * pow(actual_cell_voltage, value2));
    ESP_LOGD(TAG, "at_actual_cell_voltage=%f", at_actual_cell_voltage);

    double percent = 1 - (at_actual_cell_voltage / at_knee) / at_target_cell_voltage;
    ESP_LOGD(TAG, "percent=%f", percent);

    if (percent < 0.01)
    {
        // Catch small values and also negatives, 1% is the lowest we go...
        percent = 0.01;
    }

    // Use lowest of chargecurrent or calculation, just in case some math has gone wrong!
    ESP_LOGD(TAG, "percent=%f", percent);
    dynamicChargeCurrent = min(mysettings->chargecurrent, (uint16_t)round(mysettings->chargecurrent * percent));

    ESP_LOGD(TAG, "dynamicChargeCurrent=%u", dynamicChargeCurrent);
}

/// @brief Apply "dynamic" charge voltage rules
// This will always return a charge voltage - its the calling functions responsibility  to check "IsChargeAllowed" function and take necessary action.
// Thanks to Matthias U (Smurfix) for the ideas and pseudo code https://community.openenergymonitor.org/u/smurfix/
// Output is cached in variable dynamicChargeVoltage as its used in multiple places
void Rules::CalculateDynamicChargeVoltage(const diybms_eeprom_settings *mysettings, const CellModuleInfo *cellarray)
{
    if (!mysettings->dynamiccharge || mysettings->canbusprotocol == CanBusProtocolEmulation::CANBUS_DISABLED)
    {
        // Dynamic charge switched off, use default or float voltage
        dynamicChargeVoltage = (chargemode == ChargingMode::floating) ? mysettings->floatvoltage : mysettings->chargevolt;
        return;
    }

    // Some cells are above the knee voltage....

    // Are any cells at or over the maximum allowed? (panic!)
    if (highestCellVoltage >= mysettings->cellmaxmv)
    {
        ESP_LOGW(TAG, "Cell V>Max");
        // *** Stop charging, we are at or above maximum cell voltage ***

        // Find the lowest "limited" Bank voltage
        uint32_t lowest = 0xFFFFFFFF;
        for (uint8_t r = 0; r < mysettings->totalNumberOfBanks; r++)
        {
            if (limitedbankvoltage.at(r) < lowest)
            {
                lowest = limitedbankvoltage.at(r);
            }
        }

        lowest = lowest / 100;
        // ESP_LOGD(TAG, "lowest=%u", lowest);

        // Return MIN of either the "lowest Bank voltage" or the "user specified value"
        dynamicChargeVoltage = min(lowest, (uint32_t)mysettings->chargevolt);
        return;
    }

    // At this point all cell voltages are UNDER the target cellmaxmv

    // This is unlikely to work if the value is changed from 1 (an integer)
    const int16_t UniformDerating = 1;

    // Calculate voltage range
    uint32_t R = min(
        (int16_t)((mysettings->cellmaxmv - highestCellVoltage) * UniformDerating),
        (int16_t)((mysettings->cellmaxspikemv - mysettings->kneemv) / ((float)mysettings->sensitivity / 10.0F)));
    // Avoid future divide by zero errors
    if (R == 0)
    {
        R = 1;
    }
    ESP_LOGD(TAG, "R=%u", R);

    // We use the Bank with the highest cell voltage for these calculations - although hopefully all banks are very similar :-)
    uint32_t S = bankvoltage.at(index_bank_HighestCellVoltage);
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

    // If we are floating, then use lowest of calculated voltage or float voltage
    if (chargemode == ChargingMode::floating)
    {
        dynamicChargeVoltage = min(dynamicChargeVoltage, mysettings->floatvoltage);
    }
}

/// @brief Return SoC value after applying SOCFORCELOW and SOCOVERRIDE settings
/// @param mysettings
/// @param realSOC True value of SoC
/// @return SoC is rounded down to nearest integer and limits output range between 0 and 100.
uint16_t Rules::StateOfChargeWithRulesApplied(const diybms_eeprom_settings *mysettings, float realSOC) const
{
    uint16_t value = floor(realSOC);

    // Deliberately force SoC to be reported as 2%, to trick external CANBUS devices into trickle charging (if they support it)
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
        if (value < 25)
        {
            // Force minimum of 25% - some inverters (SoFAR) will force charge a battery lower than
            // this level limiting the charge current to 500W
            value = 25;
        }
    }

    // Limit to 100% maximum, DIYBMS current monitor can go above 100%, so don't confuse inverter/chargers
    if (value > 100)
    {
        value = 100;
    }
    return value;
}

/// @brief Determine which charging mode the controller should be operating in
/// @param mysettings
/// @param currentMonitor
void Rules::CalculateChargingMode(const diybms_eeprom_settings *mysettings, const currentmonitoring_struct *currentMonitor)
{
    // If we are not using CANBUS - ignore the charge mode, it doesn't mean anything
    if (mysettings->canbusprotocol == CanBusProtocolEmulation::CANBUS_DISABLED)
    {
        return;
    }

    // Determine charging mode - this is only possible if current monitor/state of charge calculation is functioning/installed
    if (IsStateOfChargeValid(mysettings, currentMonitor) == false)
    {
        return;
    }

    ChargingMode mode = getChargingMode();
    if (currentMonitor->stateofcharge < (float)mysettings->stateofchargeresumevalue || mysettings->socforcelow || mysettings->socoverride)
    {
        // If S.o.C FORCE LOW or OVERRIDE are enabled, assume standard charging modes.
        // or battery is below resume level so normal charging operation in progress

        // No difference in STANDARD or DYNAMIC modes - purely visual on screen/cosmetic
        if (mysettings->dynamiccharge == true && mysettings->canbusprotocol != CanBusProtocolEmulation::CANBUS_DISABLED)
        {
            setChargingMode(ChargingMode::dynamic);
        }
        else
        {
            setChargingMode(ChargingMode::standard);
        }
        return;
    }

    // S.o.C is above stateofchargeresumevalue (so nearly fully charged)

    auto time_now = esp_timer_get_time();
    if ((mode == ChargingMode::standard || mode == ChargingMode::dynamic) && (currentMonitor->stateofcharge >= 99.0F))
    {
        // Battery is almost full at over 99.0%, therefore enable "absorb mode"
        setChargingMode(ChargingMode::absorb);
        ChargingTimer = FutureTime(mysettings->absorptiontimer);
        return;
    }

    // Has time interval passed?
    if (time_now > ChargingTimer)
    {
        if (mode == ChargingMode::absorb)
        {
            // Absorb has finished, switch to float
            setChargingMode(ChargingMode::floating);
            ChargingTimer = FutureTime(mysettings->floatvoltagetimer);
            return;
        }

        if (mode == ChargingMode::floating)
        {
            // Floating has finished, stop charging completely
            // Charge will not start again until state of charge drops
            setChargingMode(ChargingMode::stopped);
            ChargingTimer = 0;
            return;
        }
    }
}