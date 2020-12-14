#include <Arduino.h>

#include "EmbeddedFiles_Defines.h"

#ifndef DIYBMS_DEFINES_H_
#define DIYBMS_DEFINES_H_

#if defined(ESP32)
//Data uses Rx2/TX2 and debug logs go to serial0 - USB
#define SERIAL_DATA Serial2
#define SERIAL_DEBUG Serial

//Total number of cells a single controler can handle (memory limitation)
#define maximum_controller_cell_modules 250

#endif

#if defined(ESP8266)
#define SERIAL_DATA Serial
#define SERIAL_DEBUG Serial1

//Total number of cells a single controler can handle (memory limitation)
#define maximum_controller_cell_modules 100

#endif

//Maximum of 16 cell modules (don't change this!) number of cells to process in a single packet of data
#define maximum_cell_modules_per_packet 16

//Maximum number of banks allowed
//This also needs changing in default.htm (MAXIMUM_NUMBER_OF_BANKS)
#define maximum_number_of_banks 16



//Version 4.XX of DIYBMS modules operate at 2400 baud
#define COMMS_BAUD_RATE 2400

#define EEPROM_SETTINGS_START_ADDRESS 256

enum enumInputState : uint8_t
{
  INPUT_HIGH = 0xFF,
  INPUT_LOW = 0x99,
  INPUT_UNKNOWN = 0x00
};

struct i2cQueueMessage
{
    uint8_t command;
    uint8_t data;
};


enum RelayState : uint8_t
{
  RELAY_ON = 0xFF,
  RELAY_OFF = 0x99,
  RELAY_X = 0x00
};

enum RelayType : uint8_t
{
  RELAY_STANDARD = 0x00,
  RELAY_PULSE = 0x01
};

#define RELAY_RULES 12
//Number of relays on board (4)
#define RELAY_TOTAL 4

//5 inputs on board
#define INPUTS_TOTAL 5

#define SHOW_TIME_PERIOD 5000
#define NTP_TIMEOUT 1500

struct diybms_eeprom_settings
{
  uint8_t totalNumberOfBanks;
  uint8_t totalNumberOfSeriesModules;

  uint32_t rulevalue[RELAY_RULES];
  uint32_t rulehysteresis[RELAY_RULES];

  //Use a bit pattern to indicate the relay states
  RelayState rulerelaystate[RELAY_RULES][RELAY_TOTAL];
  //Default starting state
  RelayState rulerelaydefault[RELAY_TOTAL];
  //Default starting state for relay types
  RelayType relaytype[RELAY_TOTAL];

  float graph_voltagehigh;
  float graph_voltagelow;

  uint8_t BypassOverTempShutdown;
  uint16_t BypassThresholdmV;

  int8_t timeZone;        // = 0;
  int8_t minutesTimeZone; // = 0;
  bool daylight;          //=false;
  char ntpServer[64 + 1]; // = "time.google.com";

  //NOTE this array is subject to buffer overflow vulnerabilities!
  bool mqtt_enabled;
  uint16_t mqtt_port;
  char mqtt_server[64 + 1];
  char mqtt_topic[32 + 1];
  char mqtt_username[32 + 1];
  char mqtt_password[32 + 1];

  bool influxdb_enabled;
  uint16_t influxdb_httpPort;
  char influxdb_host[64 + 1];
  char influxdb_database[32 + 1];
  char influxdb_user[32 + 1];
  char influxdb_password[32 + 1];
};

typedef union
{
  float number;
  uint8_t bytes[4];
  uint16_t word[2];
} FLOATUNION_t;

// Only the lowest 4 bits can be used!
enum COMMAND: uint8_t
{
    ResetBadPacketCounter = 0,
    ReadVoltageAndStatus=1,
    Identify=2,
    ReadTemperature=3,
    ReadBadPacketCounter=4,
    ReadSettings=5,
    WriteSettings=6,
    ReadBalancePowerPWM=7,
    Timing=8,
    ReadBalanceCurrentCounter=9,
    ReadPacketReceivedCounter=10   
};

//NOTE THIS MUST BE EVEN IN SIZE (BYTES) ESP8266 IS 32 BIT AND WILL ALIGN AS SUCH!
struct PacketStruct
{
  uint8_t start_address;
  uint8_t end_address;
  uint8_t command;
  uint8_t hops;
  uint16_t sequence;
  uint16_t moduledata[maximum_cell_modules_per_packet];
  uint16_t crc;
} __attribute__((packed));

struct CellModuleInfo
{
  //Used as part of the enquiry functions
  bool settingsCached : 1;
  //Set to true once the module has replied with data
  bool valid : 1;
  //Bypass is active
  bool inBypass : 1;
  //Bypass active and temperature over set point
  bool bypassOverTemp : 1;

  uint16_t voltagemV;
  uint16_t voltagemVMin;
  uint16_t voltagemVMax;
  //Signed integer byte (negative temperatures)
  int8_t internalTemp;
  int8_t externalTemp;

  uint8_t BypassOverTempShutdown;
  uint16_t BypassThresholdmV;
  uint16_t badPacketCount;

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
  //Version number returned by code of module
  uint16_t BoardVersionNumber;
  //Last 4 bytes of GITHUB version
  uint32_t CodeVersionNumber;
  //Value of PWM timer for load shedding
  uint16_t PWMValue;

  uint16_t BalanceCurrentCount;
  uint16_t PacketReceivedCount;
};

// This enum holds the states the controller goes through whilst
// it stabilizes and moves into running state.
enum ControllerState : uint8_t
{
  Unknown=0,
  PowerUp = 1,
  Stabilizing = 2,
  ConfigurationSoftAP=3,
  Running = 255,  
};

//This holds all the cell information in a large array array
extern CellModuleInfo cmi[maximum_controller_cell_modules];

#endif
