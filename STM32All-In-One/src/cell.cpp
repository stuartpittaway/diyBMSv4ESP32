/*
____  ____  _  _  ____  __  __  ___
(  _ \(_  _)( \/ )(  _ \(  \/  )/ __)
)(_) )_)(_  \  /  ) _ < )    ( \__ \
(____/(____) (__) (____/(_/\/\_)(___/

DIYBMS V4

V490 UP TO 16S CELL MONITORING MODULES

STM32F030K6T6

(c)2023 Stuart Pittaway

COMPILE THIS CODE USING PLATFORM.IO

LICENSE
Attribution-NonCommercial-ShareAlike 2.0 UK: England & Wales (CC BY-NC-SA 2.0 UK)
https://creativecommons.org/licenses/by-nc-sa/2.0/uk/

* Non-Commercial — You may not use the material for commercial purposes.
* Attribution — You must give appropriate credit, provide a link to the license, and indicate if changes were made.
  You may do so in any reasonable manner, but not in any way that suggests the licensor endorses you or your use.
* ShareAlike — If you remix, transform, or build upon the material, you must distribute your
  contributions under the same license as the original.
* No additional restrictions — You may not apply legal terms or technological measures
  that legally restrict others from doing anything the license permits.
*/

#include "cell.h"

// These variables are shared across all instances of this class
uint8_t Cell::BypassTemperatureSetPoint;
uint16_t Cell::BypassThresholdmV;
float Cell::Calibration;
uint8_t Cell::BypassTemperatureHysteresis;

// Maximum temperature to allow
const int16_t Cell::SafetyTemperatureCutoff = 80;
bool Cell::OverTemperature = false;

uint16_t Cell::relay_range;
uint16_t Cell::relay_minimummv;
uint8_t Cell::fanswitchontemperature;
