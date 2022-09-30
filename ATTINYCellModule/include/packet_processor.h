#ifndef DIYBMS_PACKETPROCESSOR_H // include guard
#define DIYBMS_PACKETPROCESSOR_H

#include <Arduino.h>

#if (!defined(DIYBMSMODULEVERSION))
#error You need to specify the DIYBMSMODULEVERSION define
#endif


//Include both of these, they have #define checks to work out what to do
#include "diybms_tinyAVR2.h"
#include "diybms_attiny841.h"

#include "Steinhart.h"
#include "defines.h"
#include "settings.h"
#include "crc16.h"

#define ADC_CELL_VOLTAGE 0
#define ADC_INTERNAL_TEMP 1
#define ADC_EXTERNAL_TEMP 2

//Define maximum allowed temperature as safety cut off
#define DIYBMS_MODULE_SafetyTemperatureCutoff (int16_t)90

#define maximum_cell_modules 16

//NOTE THIS MUST BE EVEN IN SIZE (BYTES) ESP8266 IS 32 BIT AND WILL ALIGN AS SUCH!
struct PacketStruct
{
  uint8_t start_address;
  uint8_t end_address;
  uint8_t command;
  uint8_t hops;
  uint16_t sequence;
  uint16_t moduledata[maximum_cell_modules];
  uint16_t crc;
} __attribute__((packed));

typedef union
{
  float number;
  uint8_t bytes[4];
  uint16_t word[2];
} FLOATUNION_t;

class PacketProcessor
{
public:
  PacketProcessor(CellModuleConfig *config)
  {
    _config = config;
    SettingsHaveChanged = false;
    WeAreInBypass = false;
    bypassCountDown = 0;
    bypassHasJustFinished = 0;
  }
  ~PacketProcessor() {}

  PacketProcessor(); 

  bool onPacketReceived(PacketStruct *receivebuffer);

  void ADCReading(uint16_t value);
  void TakeAnAnalogueReading(uint8_t mode);
  uint16_t CellVoltage();

  uint16_t IncrementWatchdogCounter()
  {
    watchdog_counter++;
    return watchdog_counter;
  }

  bool BypassCheck();
  uint16_t TemperatureMeasurement();
  uint8_t identifyModule;
  bool BypassOverheatCheck();

  int16_t InternalTemperature();

  volatile float MilliAmpHourBalanceCounter = 0;

  //Returns TRUE if the module is in "bypassing current" mode
  bool WeAreInBypass;

  //Value of PWM 0-255
  volatile uint8_t PWMSetPoint;
  volatile bool SettingsHaveChanged;

  //Count down which runs whilst bypass is in operation,  zero = bypass stopped/off
  uint16_t bypassCountDown;

  //Count down which starts after the current cycle of bypass has completed (aka cool down period whilst voltage may rise again)
  uint8_t bypassHasJustFinished;

  bool IsBypassActive()
  {
    return WeAreInBypass || bypassHasJustFinished > 0;
  }

private:
  CellModuleConfig *_config;

  bool processPacket(PacketStruct *buffer);

  volatile bool ModuleAddressAssignedFlag = false;
  volatile uint8_t adcmode = 0;
  volatile uint16_t raw_adc_voltage;
  volatile uint16_t raw_adc_onboard_temperature;
  volatile uint16_t raw_adc_external_temperature;

  //Cell number in the string (updated dynamically)
  volatile uint8_t mymoduleaddress = 0;
  //Count of bad packets of data received, most likely with corrupt data or crc errors
  volatile uint16_t badpackets = 0;
  //Count of number of WDT events which have triggered, could indicate standalone mode or problems with serial comms
  volatile uint16_t watchdog_counter = 0;

  uint16_t PacketReceivedCounter = 0;

#if (SAMPLEAVERAGING > 1)
  volatile uint16_t readings[SAMPLEAVERAGING]; // the readings from the analog input
  volatile uint16_t readIndex = 0;             // the index of the current reading
  volatile uint16_t total = 0;                 // the running total
  volatile uint16_t average = 0;               // the average
#endif
};

#endif
