#ifndef Rules_H_
#define Rules_H_

#include "defines.h"

//Needs to match the ordering on the HTML screen
#define RULE_EmergencyStop 0
#define RULE_BMSError 1
#define RULE_Individualcellovervoltage 2
#define RULE_Individualcellundervoltage 3
#define RULE_IndividualcellovertemperatureExternal 4
#define RULE_IndividualcellundertemperatureExternal 5
#define RULE_PackOverVoltage 6
#define RULE_PackUnderVoltage 7
#define RULE_Timer2 8
#define RULE_Timer1 9

enum InternalErrorCode : uint8_t
{
    NoError=0,
    CommunicationsError=1,
    ModuleCountMismatch=2,
    TooManyModules=3
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
    bool moduleHasExternalTempSensor;

    void ClearValues();
    void ProcessCell(uint8_t bank, CellModuleInfo *c);
    void ProcessBank(uint8_t bank);
    void SetError(InternalErrorCode err);
    uint16_t VoltageRangeInBank(uint8_t bank);
    void RunRules(
        uint32_t *value,
        uint32_t *hysteresisvalue,
        bool emergencyStop, 
        uint16_t mins);
};

#endif