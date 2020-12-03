/*
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

#ifndef DIYBMS_DEFINES_H // include guard
#define DIYBMS_DEFINES_H

//Incremented when major revisions of code are made and new features
#define MODULE_FIRMWARE_VERSION 1

#if (!defined(DIYBMSMODULEVERSION))
#error You need to enable one of the DIYBMSMODULEVERSION define statements
#endif

#if defined(DIYBMSMODULEVERSION) && DIYBMSMODULEVERSION > 430
#error Incorrect value for DIYBMSMODULEVERSION
#endif

#if defined(DIYBMSMODULEVERSION) && DIYBMSMODULEVERSION < 430
  #define COMMS_BAUD_RATE 2400
#endif

#if defined(DIYBMSMODULEVERSION) && DIYBMSMODULEVERSION == 430
  #define COMMS_BAUD_RATE 2400
#endif

//This is where the data begins in EEPROM
#define EEPROM_CONFIG_ADDRESS 0

#define nop  __asm__("nop\n\t");


enum COMMAND: uint8_t
{
    ResetBadPacketCounter = B00000000,
    ReadVoltageAndStatus=B00000001,
    Identify=B00000010,
    ReadTemperature=B00000011,
    ReadBadPacketCounter=B00000100,
    ReadSettings=B00000101,
    WriteSettings=B00000110,
    ReadBalancePowerPWM=B00000111,
    Timing=B00001000
   
    // 0000 0000  = set bank identity [obsolete]
    // 0000 0001  = read voltage and status
    // 0000 0010  = identify module (flash leds)
    // 0000 0011  = Read temperature
    // 0000 0100  = Report number of bad packets
    // 0000 0101  = Report settings/configuration
    // 0000 0110  = Write settings/configuration
    // 0000 0111  = Read current level of PWM for power balance
    // 0000 1000  = Timing of the commands through the string of modules
};


//Default values
struct CellModuleConfig {
  //uint8_t mybank;
  uint8_t BypassTemperatureSetPoint;
  uint16_t BypassThresholdmV;

  // Resistance of bypass load
  //float LoadResistance;
  //Voltage Calibration
  float Calibration;
  //Reference voltage (millivolt) normally 2.00mV
  //float mVPerADC;
  //Internal Thermistor settings
  //uint16_t Internal_BCoefficient;
  //External Thermistor settings
  //uint16_t External_BCoefficient;
} __attribute__((packed));

#endif
