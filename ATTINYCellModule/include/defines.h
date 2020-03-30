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

/*
IMPORTANT YOU MUST CONFIGURE THE VERSION OF THE MODULE/BOARD YOU ARE COMPILING FOR

POSSIBLE VALUES
400 = Original board (marked DIYBMS v4 on silkscreen) - has 8 large resistors (marked 2R20) and likely handsoldered using 0805 sized parts
410 = JLCPCB built board (marked DIYBMS v4 on silkscreen) - has 8 large resistors (marked 2R00) and machine soldered using 0603 sized parts
420 = JLCPCB built board (marked DIYBMS v4.2 on silkscreen) - has 20 small resistors (marked 6R20) and machine soldered using 0603 sized parts (R20 is in middle of resistor array)
421 = JLCPCB built board (marked DIYBMS v4.21 on silkscreen) - has 20 small resistors (marked 6R20) and machine soldered using 0603 sized parts (R19 is in middle of resistor array)
*/

// ONLY ENABLE ONE OF THE BELOW....
#define DIYBMSMODULEVERSION 400
//#define DIYBMSMODULEVERSION 410
//#define DIYBMSMODULEVERSION 420
//#define DIYBMSMODULEVERSION 421

#if (!defined(DIYBMSMODULEVERSION))
#error You need to enable one of the DIYBMS_MOD_VER define statements
#endif

#if defined(DIYBMSMODULEVERSION) && DIYBMSMODULEVERSION > 421
#error Incorrect value for DIYBMS_MOD_VER
#endif

#if defined(DIYBMSMODULEVERSION) && DIYBMSMODULEVERSION < 430
  #define COMMS_BAUD_RATE 2400
#else
  #error Incorrect value for DIYBMSMODULEVERSION
#endif

//This is where the data begins in EEPROM
#define EEPROM_CONFIG_ADDRESS 0

#define nop  __asm__("nop\n\t");




enum COMMAND: uint8_t
{
    SetBankIdentity=B00000000,
    ReadVoltageAndStatus=B00000001,
    Identify=B00000010,
    ReadTemperature=B00000011,
    ReadBadPacketCounter=B00000100,
    ReadSettings=B00000101,
    WriteSettings=B00000110

    // 0000 0000  = set bank identity
    // 0000 0001  = read voltage and status
    // 0000 0010  = identify module (flash leds)
    // 0000 0011  = Read temperature
    // 0000 0100  = Report number of bad packets
    // 0000 0101  = Report settings/configuration
    // 0000 0110  = Write settings/configuration
};


//Default values
struct CellModuleConfig {
  uint8_t mybank;
  uint8_t BypassOverTempShutdown;
  uint16_t BypassThresholdmV;

  // Resistance of bypass load
  float LoadResistance;
  //Voltage Calibration
  float Calibration;
  //Reference voltage (millivolt) normally 2.00mV
  float mVPerADC;
  //Internal Thermistor settings
  uint16_t Internal_BCoefficient;
  //External Thermistor settings
  uint16_t External_BCoefficient;
} __attribute__((packed));


#endif
