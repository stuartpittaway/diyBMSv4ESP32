#include <Arduino.h>

#include <driver/uart.h>

#include "EmbeddedFiles_Defines.h"

#include "EmbeddedFiles_Integrity.h"

//#define TOUCH_SCREEN

#ifndef DIYBMS_DEFINES_H_
#define DIYBMS_DEFINES_H_

// Needs to be at least 1550 bytes...
#define BUFSIZE 1800

// Data uses Rx2/TX2 and debug logs go to serial0 - USB
#define SERIAL_DATA Serial2
#define SERIAL_DEBUG Serial
#define SERIAL_RS485 Serial1

// Total number of cells a single controler can handle (memory limitation)
#define maximum_controller_cell_modules 128

typedef union
{
  float value;
  uint16_t word[2];
} FloatUnionType;

enum RGBLED : uint8_t
{
  OFF = 0,
  Blue = B00000001,
  Red = B00000010,
  Purple = B00000011,
  Green = B00000100,
  Cyan = B00000101,
  Yellow = B00000110,
  White = B00000111
};

enum ISRTYPE : uint32_t
{
  // Must be unique BIT pattern
  TCA6408A = 1 << 0,
  TCA9534 = 1 << 1,
  TCA6416A = 1 << 2,
  TFTTOUCH = 1 << 3
};

enum VictronDVCC : uint8_t
{
  Default = 0,
  Balance = 1,
  ControllerError = 2
};

// Maximum of 16 cell modules (don't change this!) number of cells to process in a single packet of data
#define maximum_cell_modules_per_packet 16

// Maximum number of banks allowed
// This also needs changing in default.htm (MAXIMUM_NUMBER_OF_BANKS)
#define maximum_number_of_banks 16

// Version 4.XX of DIYBMS modules operate at 5000 baud (since 26 Jan 2021)
//#define COMMS_BAUD_RATE 5000

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

enum CurrentMonitorDevice : uint8_t
{
  DIYBMS_CURRENT_MON = 0x00,
  PZEM_017 = 0x01
};

// Number of rules as defined in Rules.h (enum Rule)
#define RELAY_RULES 15

// Number of relays on board (4)
#define RELAY_TOTAL 4

// 7 inputs on board
#define INPUTS_TOTAL 7

#define SHOW_TIME_PERIOD 5000
#define NTP_TIMEOUT 1500

struct diybms_eeprom_settings
{
  uint8_t totalNumberOfBanks;
  uint8_t totalNumberOfSeriesModules;
  uint16_t baudRate;
  uint16_t interpacketgap;

  uint32_t rulevalue[RELAY_RULES];
  uint32_t rulehysteresis[RELAY_RULES];

  // Use a bit pattern to indicate the relay states
  RelayState rulerelaystate[RELAY_RULES][RELAY_TOTAL];
  // Default starting state
  RelayState rulerelaydefault[RELAY_TOTAL];
  // Default starting state for relay types
  RelayType relaytype[RELAY_TOTAL];

  float graph_voltagehigh;
  float graph_voltagelow;

  uint8_t BypassOverTempShutdown;
  uint16_t BypassThresholdmV;

  int8_t timeZone;        // = 0;
  int8_t minutesTimeZone; // = 0;
  bool daylight;          //=false;
  char ntpServer[64 + 1]; // = "time.google.com";

  bool loggingEnabled;
  uint16_t loggingFrequencySeconds;

  bool currentMonitoringEnabled;
  uint8_t currentMonitoringModBusAddress;
  CurrentMonitorDevice currentMonitoringDevice;

  int rs485baudrate;
  uart_word_length_t rs485databits;
  uart_parity_t rs485parity;
  uart_stop_bits_t rs485stopbits;

  char language[2 + 1];

  uint16_t cvl[3];
  int16_t ccl[3];
  int16_t dcl[3];

  bool VictronEnabled;

  // NOTE this array is subject to buffer overflow vulnerabilities!
  bool mqtt_enabled;
  char mqtt_uri[128 + 1];
  char mqtt_topic[32 + 1];
  char mqtt_username[32 + 1];
  char mqtt_password[32 + 1];

  bool influxdb_enabled;
  // uint16_t influxdb_httpPort;
  char influxdb_serverurl[128 + 1];
  char influxdb_databasebucket[64 + 1];
  char influxdb_apitoken[128 + 1];
  char influxdb_orgid[128 + 1];
  uint8_t influxdb_loggingFreqSeconds;
};

typedef union
{
  float number;
  uint8_t bytes[4];
  uint16_t word[2];
} FLOATUNION_t;

// Only the lowest 4 bits can be used!
enum COMMAND : uint8_t
{
  ResetBadPacketCounter = 0,
  ReadVoltageAndStatus = 1,
  Identify = 2,
  ReadTemperature = 3,
  ReadBadPacketCounter = 4,
  ReadSettings = 5,
  WriteSettings = 6,
  ReadBalancePowerPWM = 7,
  Timing = 8,
  ReadBalanceCurrentCounter = 9,
  ReadPacketReceivedCounter = 10,
  ResetBalanceCurrentCounter = 11
};

// NOTE THIS MUST BE EVEN IN SIZE (BYTES) ESP8266 IS 32 BIT AND WILL ALIGN AS SUCH!
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
  // Used as part of the enquiry functions
  bool settingsCached : 1;
  // Set to true once the module has replied with data
  bool valid : 1;
  // Bypass is active
  bool inBypass : 1;
  // Bypass active and temperature over set point
  bool bypassOverTemp : 1;

  uint16_t voltagemV;
  uint16_t voltagemVMin;
  uint16_t voltagemVMax;
  // Signed integer byte (negative temperatures)
  int8_t internalTemp;
  int8_t externalTemp;

  uint8_t BypassOverTempShutdown;
  uint16_t BypassThresholdmV;
  uint16_t badPacketCount;

  // Resistance of bypass load
  float LoadResistance;
  // Voltage Calibration
  float Calibration;
  // Reference voltage (millivolt) normally 2.00mV
  float mVPerADC;
  // Internal Thermistor settings
  uint16_t Internal_BCoefficient;
  // External Thermistor settings
  uint16_t External_BCoefficient;
  // Version number returned by code of module
  uint16_t BoardVersionNumber;
  // Last 4 bytes of GITHUB version
  uint32_t CodeVersionNumber;
  // Value of PWM timer for load shedding
  uint16_t PWMValue;

  uint16_t BalanceCurrentCount;
  uint16_t PacketReceivedCount;
};

// This enum holds the states the controller goes through whilst
// it stabilizes and moves into running state.
enum ControllerState : uint8_t
{
  Unknown = 0,
  PowerUp = 1,
  Stabilizing = 2,
  NoWifiConfiguration=3,
  Running = 255,
};

enum CardAction : uint8_t
{
  Idle = 0,
  Mount = 1,
  Unmount = 2,
  Remount = 3
};


// This holds all the cell information in a large array array
extern CellModuleInfo cmi[maximum_controller_cell_modules];

struct avrprogramsettings
{
  uint8_t efuse;
  uint8_t hfuse;
  uint8_t lfuse;
  uint32_t mcu;
  uint32_t duration;
  bool inProgress;
  char filename[64];
  uint8_t progresult;
  size_t programsize;

  bool programmingModeEnabled;
};

struct currentmonitor_raw_modbus
{
  // These variables are in STRICT order
  // and must match the MODBUS register sequence and data types!!

  // Voltage
  float voltage;
  // Current in AMPS
  float current;
  uint32_t milliamphour_out;
  uint32_t milliamphour_in;
  int16_t temperature;
  uint16_t flags;
  float power;
  float shuntmV;
  float currentlsb;
  float shuntresistance;
  uint16_t shuntmaxcurrent;
  uint16_t shuntmillivolt;
  uint16_t batterycapacityamphour;
  float fullychargedvoltage;
  float tailcurrentamps;
  uint16_t raw_chargeefficiency;
  uint16_t raw_stateofcharge;
  uint16_t shuntcal;
  int16_t temperaturelimit;
  float overvoltagelimit;
  float undervoltagelimit;
  float overcurrentlimit;
  float undercurrentlimit;
  float overpowerlimit;
  uint16_t shunttempcoefficient;
  uint16_t modelnumber;
  uint32_t firmwareversion;
  uint32_t firmwaredatetime;
  uint16_t watchdogcounter;
} __attribute__((packed));

struct currentmonitoring_struct
{
  currentmonitor_raw_modbus modbus;

  // Uses float as these are 4 bytes on ESP32
  int64_t timestamp;
  bool validReadings;

  float chargeefficiency;
  float stateofcharge;

  bool TemperatureOverLimit : 1;
  bool CurrentOverLimit : 1;
  bool CurrentUnderLimit : 1;
  bool VoltageOverlimit : 1;
  bool VoltageUnderlimit : 1;
  bool PowerOverLimit : 1;
  bool TempCompEnabled : 1;
  bool ADCRange4096mV : 1;

  bool RelayTriggerTemperatureOverLimit : 1;
  bool RelayTriggerCurrentOverLimit : 1;
  bool RelayTriggerCurrentUnderLimit : 1;
  bool RelayTriggerVoltageOverlimit : 1;
  bool RelayTriggerVoltageUnderlimit : 1;
  bool RelayTriggerPowerOverLimit : 1;
  bool RelayState : 1;
};


enum DIAG_ALRT_FIELD : uint16_t
{
  ALATCH = 15,
  CNVR = 14,
  SLOWALERT = 13,
  APOL = 12,
  ENERGYOF = 11,
  CHARGEOF = 10,
  MATHOF = 9,
  RESERVED = 8,
  TMPOL = 7,
  SHNTOL = 6,
  SHNTUL = 5,
  BUSOL = 4,
  BUSUL = 3,
  POL = 2,
  CNVRF = 1,
  MEMSTAT = 0
};


//Where in EEPROM do we store the configuration
#define EEPROM_WIFI_START_ADDRESS 0

struct wifi_eeprom_settings
{
  char wifi_ssid[32 + 1];
  char wifi_passphrase[63 + 1];
};

#endif
