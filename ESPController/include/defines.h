#include <Arduino.h>

#ifndef DIYBMS_DEFINES_H_
#define DIYBMS_DEFINES_H_

//Maximum of 16 cell modules (dont change this!)
#define maximum_cell_modules 16
#define maximum_bank_of_modules 4

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

//Needs to match the ordering on the HTML screen
#define RULE_EmergencyStop 0
#define RULE_CommunicationsError 1
#define RULE_Individualcellovervoltage 2
#define RULE_Individualcellundervoltage 3
#define RULE_IndividualcellovertemperatureExternal 4
#define RULE_IndividualcellundertemperatureExternal 5
#define RULE_PackOverVoltage 6
#define RULE_PackUnderVoltage 7
#define RULE_Timer2 8
#define RULE_Timer1 9

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
  bool combinationParallel;
  uint8_t totalNumberOfBanks;

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
  SetBankIdentity = B00000000,
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
struct packet
{
  uint8_t address;
  uint8_t command;
  uint16_t sequence;
  uint16_t moduledata[maximum_cell_modules];
  uint16_t crc;
} __attribute__((packed));

struct CellModuleInfo
{
  //Used as part of the enquiry functions
  bool settingsCached;

  uint16_t voltagemV;
  uint16_t voltagemVMin;
  uint16_t voltagemVMax;
  //Signed integer - should these be byte?
  int8_t internalTemp;
  int8_t externalTemp;

  bool inBypass;
  bool bypassOverTemp;

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


//This holds all the cell information in a large array 2D array (4x16)
extern CellModuleInfo cmi[maximum_bank_of_modules][maximum_cell_modules];
extern uint8_t numberOfModules[maximum_bank_of_modules];

#endif
