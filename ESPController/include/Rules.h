#ifndef Rules_H_
#define Rules_H_

#pragma once

#include "defines.h"

//Needs to match the ordering on the HTML screen
enum Rule : uint8_t
{
    EmergencyStop = 0,
    BMSError = 1,
    Individualcellovervoltage = 2,
    Individualcellundervoltage = 3,
    ModuleOverTemperatureInternal = 4,
    ModuleUnderTemperatureInternal = 5,
    IndividualcellovertemperatureExternal = 6,
    IndividualcellundertemperatureExternal = 7,
    PackOverVoltage = 8,
    PackUnderVoltage = 9,
    Timer2 = 10,
    Timer1 = 11

};

//Define a max constant for the highest value (change if you add more warnings)
#define MAXIMUM_InternalWarningCode 6
enum InternalWarningCode : uint8_t
{
    NoWarning = 0,
    ModuleInconsistantBypassVoltage = 1,
    ModuleInconsistantBypassTemperature = 2,
    ModuleInconsistantCodeVersion = 3,
    ModuleInconsistantBoardRevision = 4,
    LoggingEnabledNoSDCard = 5,
    AVRProgrammingMode =6
};

//Define a max constant for the highest value (change if you add more errors)
#define MAXIMUM_InternalErrorCode 6
enum InternalErrorCode : uint8_t
{
    NoError = 0,
    CommunicationsError = 1,
    ModuleCountMismatch = 2,
    TooManyModules = 3,
    WaitingForModulesToReply = 4,
    ZeroVoltModule = 5,
    ControllerMemoryError = 6
};

class Rules
{

public:
    bool rule_outcome[RELAY_RULES];
    uint32_t packvoltage[maximum_number_of_banks];

    uint16_t lowestvoltageinpack[maximum_number_of_banks];
    uint16_t highestvoltageinpack[maximum_number_of_banks];

    uint8_t zeroVoltageModuleCount;

    uint32_t highestPackVoltage;
    uint32_t lowestPackVoltage;
    uint16_t highestCellVoltage;
    uint16_t lowestCellVoltage;
    int8_t highestExternalTemp;
    int8_t lowestExternalTemp;
    int8_t highestInternalTemp;
    int8_t lowestInternalTemp;
    InternalErrorCode ErrorCodes[1 + MAXIMUM_InternalErrorCode];
    InternalWarningCode WarningCodes[1 + MAXIMUM_InternalWarningCode];
    bool moduleHasExternalTempSensor;
    uint8_t invalidModuleCount;

    int8_t numberOfActiveErrors;
    int8_t numberOfActiveWarnings;
    int8_t numberOfBalancingModules;

    void ClearValues();
    void ProcessCell(uint8_t bank, CellModuleInfo *c);
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
        uint32_t *value,
        uint32_t *hysteresisvalue,
        bool emergencyStop,
        uint16_t mins);
};

#endif