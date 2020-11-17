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
    IndividualcellovertemperatureExternal = 4,
    IndividualcellundertemperatureExternal = 5,
    PackOverVoltage = 6,
    PackUnderVoltage = 7,
    Timer2 = 8,
    Timer1 = 9
};

enum InternalWarningCode : uint8_t
{
    NoWarning = 0,
    ModuleInconsistantBypassVoltage = 1,
    ModuleInconsistantBypassTemperature = 2
};

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
    InternalErrorCode ErrorCode;
    InternalWarningCode WarningCode;
    bool moduleHasExternalTempSensor;
    uint8_t invalidModuleCount;

    void ClearValues();
    void ProcessCell(uint8_t bank, CellModuleInfo *c);
    void ProcessBank(uint8_t bank);
    void SetWarning(InternalWarningCode warncode);
    void SetError(InternalErrorCode err);
    uint16_t VoltageRangeInBank(uint8_t bank);
    void RunRules(
        uint32_t *value,
        uint32_t *hysteresisvalue,
        bool emergencyStop,
        uint16_t mins);
};

#endif