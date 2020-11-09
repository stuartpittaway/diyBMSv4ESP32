#include <Arduino.h>

#ifndef DIYBMS_DEFINES_H_
#define DIYBMS_DEFINES_H_

//Maximum of 16 cell modules (dont change this!)
//number of cells to process in a single packet of data
#define maximum_cell_modules_per_packet 16

//Total number of cells a single controler can handle (memory limitation)
#define maximum_controller_cell_modules 64

//Maximum number of banks allowed
//This also needs changing in default.htm (MAXIMUM_NUMBER_OF_BANKS)
#define maximum_number_of_banks 16

//Version 4.XX of DIYBMS modules operate at 2400 baud
#define COMMS_BAUD_RATE 2400

#if defined(ESP8266)
#define RESET_WIFI_PIN D3
#define PFC_INTERRUPT_PIN D5
#define GREEN_LED D0
#define SERIAL_DATA Serial
#define SERIAL_DEBUG Serial1

//Debug flags for ntpclientlib
#define DBG_PORT Serial1
//#define DEBUG_NTPCLIENT
#endif

#if defined(ESP32)
#define GREEN_LED 2
//34,35,36,39 are input only and dont have pullup/down resistors
#define RESET_WIFI_PIN 32
#define PFC_INTERRUPT_PIN 33
//Data uses Rx2/TX2 and debug logs go to serial0 - USB
#define SERIAL_DATA Serial2
#define SERIAL_DEBUG Serial
#endif


#define GREEN_LED_ON digitalWrite(GREEN_LED, HIGH)
#define GREEN_LED_OFF digitalWrite(GREEN_LED, LOW)

#define EEPROM_SETTINGS_START_ADDRESS 256

#define RELAY_ON 0xFF
#define RELAY_OFF 0x99
#define RELAY_X 0x00

#define RELAY_RULES 10
//Number of relays on board (4)
#define RELAY_TOTAL 4

#define RELAY_STANDARD 0x00
#define RELAY_PULSE 0x01

#define SHOW_TIME_PERIOD 5000
#define NTP_TIMEOUT 1500

struct diybms_eeprom_settings
{
  uint8_t totalNumberOfBanks;
  uint8_t totalNumberOfSeriesModules;

  uint32_t rulevalue[RELAY_RULES];
  uint32_t rulehysteresis[RELAY_RULES]; 

  //Use a bit pattern to indicate the relay states
  uint8_t rulerelaystate[RELAY_RULES][RELAY_TOTAL];
  //Default starting state
  uint8_t rulerelaydefault[RELAY_TOTAL];
  //Default starting state for relay types
  uint8_t relaytype[RELAY_TOTAL];

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

typedef union {
  float number;
  uint8_t bytes[4];
  uint16_t word[2];
} FLOATUNION_t;

enum COMMAND : uint8_t
{
  //SetBankIdentity = B00000000,
  ReadVoltageAndStatus = B00000001,
  Identify = B00000010,
  ReadTemperature = B00000011,
  ReadBadPacketCounter = B00000100,
  ReadSettings = B00000101,
  WriteSettings = B00000110,
  ReadBalancePowerPWM = B00000111

  // 0000 0000  = set bank identity
  // 0000 0001  = read voltage and status
  // 0000 0010  = identify module (flash leds)
  // 0000 0011  = Read temperature
  // 0000 0100  = Report number of bad packets
  // 0000 0101  = Report settings/configuration
  // 0000 0110  = Write settings/configuration
  // 0000 0111  = Read current level of PWM for power balance
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
  //TODO: These boolean values could be moved into bit's of a byte to save RAM

  //Used as part of the enquiry functions
  bool settingsCached;
  //Set to true once the module has replied with data
  bool valid;
  //Bypass is active
  bool inBypass;
  //Bypass active and temperature over set point
  bool bypassOverTemp;

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
  //Value of PWM timer for load shedding
  uint16_t PWMValue;
};


// This enum holds the states the controller goes through whilst
// it stabilizes and moves into running state.
enum ControllerState : uint8_t
{
  PowerUp = 1,
  Stabilizing = 2,
  Running = 255,
};


//This holds all the cell information in a large array array (64)
extern CellModuleInfo cmi[maximum_controller_cell_modules];

#endif
