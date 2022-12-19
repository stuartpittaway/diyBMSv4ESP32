#ifndef Rules_H_
#define Rules_H_

#pragma once

#include "defines.h"

// Needs to match the ordering on the HTML screen
// You also need to update "RuleTextDescription"
// Define a max constant for the highest value (change if you add more rules)
#define MAXIMUM_RuleNumber 14
enum Rule : uint8_t
{
    EmergencyStop = 0,
    BMSError = 1,
    CurrentMonitorOverCurrentAmps = 2,
    ModuleOverVoltage = 3,
    ModuleUnderVoltage = 4,
    ModuleOverTemperatureInternal = 5,
    ModuleUnderTemperatureInternal = 6,
    ModuleOverTemperatureExternal = 7,
    ModuleUnderTemperatureExternal = 8,
    CurrentMonitorOverVoltage = 9,
    CurrentMonitorUnderVoltage = 10,
    BankOverVoltage = 11,
    BankUnderVoltage = 12,
    Timer2 = 13,
    Timer1 = 14
};

// Define a max constant for the highest value (change if you add more warnings)
#define MAXIMUM_InternalWarningCode 9
enum InternalWarningCode : uint8_t
{
    NoWarning = 0,
    ModuleInconsistantBypassVoltage = 1,
    ModuleInconsistantBypassTemperature = 2,
    ModuleInconsistantCodeVersion = 3,
    ModuleInconsistantBoardRevision = 4,
    LoggingEnabledNoSDCard = 5,
    AVRProgrammingMode = 6,
    ChargePrevented = 7,
    DischargePrevented = 8,
    NoExternalTempSensor = 9
};

// Define a max constant for the highest value (change if you add more errors)
#define MAXIMUM_InternalErrorCode 7
enum InternalErrorCode : uint8_t
{
    NoError = 0,
    CommunicationsError = 1,
    ModuleCountMismatch = 2,
    TooManyModules = 3,
    WaitingForModulesToReply = 4,
    ZeroVoltModule = 5,
    ControllerMemoryError = 6,
    ErrorEmergencyStop = 7
};

class Rules
{
private:
    uint16_t dynamicChargeVoltage;
    uint16_t dynamicChargeCurrent;

    bool SharedChargingDischargingRules(diybms_eeprom_settings *mysettings);

public:
    bool rule_outcome[RELAY_RULES];
    // Number of TRUE values in array rule_outcome
    uint8_t active_rule_count;

    // Actual pack voltage reported by the modules
    uint32_t packvoltage[maximum_number_of_banks];
    // As above, but each voltage reading limited to "cellmaxmv" setting (used for charge voltage calc)
    uint32_t limitedpackvoltage[maximum_number_of_banks];

    uint16_t lowestvoltageinpack[maximum_number_of_banks];
    uint16_t highestvoltageinpack[maximum_number_of_banks];

    uint8_t zeroVoltageModuleCount;

    uint32_t highestPackVoltage;
    uint32_t lowestPackVoltage;
    uint16_t highestCellVoltage;
    uint16_t lowestCellVoltage;

    // Identify address (id) of which module reports the highest/lowest values
    uint8_t address_HighestCellVoltage;
    // Bank with the highest cell voltage in it
    uint8_t index_bank_HighestCellVoltage;
    uint8_t address_LowestCellVoltage;
    uint8_t address_highestExternalTemp;
    uint8_t address_lowestExternalTemp;

    int8_t highestExternalTemp;
    int8_t lowestExternalTemp;
    int8_t highestInternalTemp;
    int8_t lowestInternalTemp;

    InternalErrorCode ErrorCodes[1 + MAXIMUM_InternalErrorCode];
    InternalWarningCode WarningCodes[1 + MAXIMUM_InternalWarningCode];

    // True if at least 1 module has an external temp sensor fitted
    bool moduleHasExternalTempSensor;

    // Number of modules which have not yet reported back to the controller
    uint8_t invalidModuleCount;

    int8_t numberOfActiveErrors;
    int8_t numberOfActiveWarnings;
    int8_t numberOfBalancingModules;

    void ClearValues();
    void ProcessCell(uint8_t bank, uint8_t cellNumber, CellModuleInfo *c, uint16_t cellmaxmv);
    void ProcessBank(uint8_t bank);
    void SetWarning(InternalWarningCode warncode);

    void ClearWarnings()
    {
        memset(&WarningCodes, 0, sizeof(WarningCodes));
        numberOfActiveWarnings = 0;
    }

    void ClearErrors()
    {
        memset(&ErrorCodes, 0, sizeof(ErrorCodes));
        numberOfActiveErrors = 0;
    }

    void SetError(InternalErrorCode err);
    uint16_t VoltageRangeInBank(uint8_t bank);
    void RunRules(
        int32_t *value,
        int32_t *hysteresisvalue,
        bool emergencyStop,
        uint16_t mins, currentmonitoring_struct *currentMonitor);

    bool IsChargeAllowed(diybms_eeprom_settings *mysettings);
    bool IsDischargeAllowed(diybms_eeprom_settings *mysettings);
    void CalculateDynamicChargeVoltage(diybms_eeprom_settings *mysettings, CellModuleInfo *cellarray);
    void CalculateDynamicChargeCurrent(diybms_eeprom_settings *mysettings, CellModuleInfo *cellarray);
    uint16_t DynamicChargeVoltage();
    int16_t DynamicChargeCurrent();
    uint16_t StateOfChargeWithRulesApplied(diybms_eeprom_settings *mysettings, float realSOC);
};

#endif