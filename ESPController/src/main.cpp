/*

 ____  ____  _  _  ____  __  __  ___    _  _  __
(  _ \(_  _)( \/ )(  _ \(  \/  )/ __)  ( \/ )/. |
 )(_) )_)(_  \  /  ) _ < )    ( \__ \   \  /(_  _)
(____/(____) (__) (____/(_/\/\_)(___/    \/   (_)

  (c) 2017 to 2021 Stuart Pittaway

  This is the code for the controller - it talks to the V4.X cell modules over isolated serial bus

  This code runs on ESP32 DEVKIT-C and compiles with VS CODE and PLATFORM IO environment
*/

#if defined(ESP8266)
#error ESP8266 is not supported by this code
#endif

#undef CONFIG_DISABLE_HAL_LOCKS

static const char *TAG = "diybms";

#include "esp_log.h"
#include <Arduino.h>

//#define PACKET_LOGGING_RECEIVE
//#define PACKET_LOGGING_SEND
//#define RULES_LOGGING
//#define MQTT_LOGGING

#include "FS.h"
#include <LITTLEFS.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <SPI.h>
#include "time.h"
#include <esp_wifi.h>
#include <Preferences.h>
#include "tft_splash_image.h"

// Libraries for SD card
#include "SD.h"
#include "driver/gpio.h"
//#include "driver/can.h"
#include <SDM.h>
#include <Ticker.h>
#include <ESPAsyncWebServer.h>
#include <AsyncMqttClient.h>
#include <ArduinoOTA.h>
#include <SerialEncoder.h>
#include <cppQueue.h>
#include <XPT2046_Touchscreen.h>

#include "defines.h"
#include "HAL_ESP32.h"

SDM sdm(SERIAL_RS485, 9600, RS485_ENABLE, SERIAL_8N1, RS485_RX, RS485_TX); // pins for DIYBMS => RX pin 21, TX pin 22

#include "Modbus.h"

ModbusInfo ModBus[MODBUS_NUM];
ModbusVal ModBusVal[MODBUS_NUM];

#include "Rules.h"
#include "avrisp_programmer.h"

//void InitModbus();

void setModbus(int dev, uint8_t addr, uint32_t min, uint32_t max, uint16_t reg, char *name, char *unit, char *desc);
void setModbusName(int dev, char *cp) { strncpy(ModBus[dev].name, cp, MODBUS_NAME_LEN); }
void setModbusUnit(int dev, char *cp) { strncpy(ModBus[dev].unit, cp, MODBUS_UNIT_LEN); }
void setModbusDesc(int dev, char *cp) { strncpy(ModBus[dev].desc, cp, MODBUS_DESC_LEN); }

/*
#define USER_SETUP_LOADED

#define USE_DMA_TO_TFT
// Color depth has to be 16 bits if DMA is used to render image
#define COLOR_DEPTH 16
#define ILI9341_DRIVER
//#define SPI_FREQUENCY 40000000
//#define SPI_READ_FREQUENCY 20000000
//#define SPI_TOUCH_FREQUENCY 2500000

#define LOAD_GLCD  // Font 1. Original Adafruit 8 pixel font needs ~1820 bytes in FLASH
#define LOAD_FONT2 // Font 2. Small 16 pixel high font, needs ~3534 bytes in FLASH, 96 characters
#define LOAD_FONT4 // Font 4. Medium 26 pixel high font, needs ~5848 bytes in FLASH, 96 characters
#define LOAD_FONT6 // Font 6. Large 48 pixel font, needs ~2666 bytes in FLASH, only characters 1234567890:-.apm
#define LOAD_FONT7 // Font 7. 7 segment 48 pixel font, needs ~2438 bytes in FLASH, only characters 1234567890:.
#define LOAD_FONT8 // Font 8. Large 75 pixel font needs ~3256 bytes in FLASH, only characters 1234567890:-.
#define LOAD_GFXFF // FreeFonts. Include access to the 48 Adafruit_GFX free fonts FF1 to FF48 and custom fonts

//#define SMOOTH_FONT

//#define TFT_MISO 12
//#define TFT_MOSI 13
//#define TFT_SCLK 14
#define TFT_DC 15  // Data Command control pin
#define TFT_RST -1 // Reset pin (could connect to RST pin)

#define USE_HSPI_PORT
#define SUPPORT_TRANSACTIONS

//Our CS pin is directly connected to ground as the TFT display is the only item on the HSPI bus
#undef TFT_CS
*/
#include "TFT_eSPI.h"

TFT_eSPI tft = TFT_eSPI();
HAL_ESP32 hal;
XPT2046_Touchscreen touchscreen(TOUCH_CHIPSELECT, TOUCH_IRQ); // Param 2 - Touch IRQ Pin - interrupt enabled polling

volatile bool emergencyStop = false;
volatile bool WifiDisconnected = true;

Rules rules;

bool _sd_card_installed = false;

diybms_eeprom_settings mysettings;
uint16_t ConfigHasChanged = 0;

uint16_t TotalNumberOfCells() { return mysettings.totalNumberOfBanks * mysettings.totalNumberOfSeriesModules; }

bool server_running = false;
RelayState previousRelayState[RELAY_TOTAL];
bool previousRelayPulse[RELAY_TOTAL];

volatile enumInputState InputState[INPUTS_TOTAL];

AsyncWebServer server(80);

static TaskHandle_t i2c_task_handle = NULL;
static TaskHandle_t ledoff_task_handle = NULL;
static TaskHandle_t wifiresetdisable_task_handle = NULL;
static TaskHandle_t modbuscomms_task_handle = NULL;
static TaskHandle_t sdcardlog_task_handle = NULL;
static TaskHandle_t sdcardlog_outputs_task_handle = NULL;
static QueueHandle_t queue_i2c = NULL;

//This large array holds all the information about the modules
//up to 4x16
CellModuleInfo cmi[maximum_controller_cell_modules];

#include "crc16.h"

#include "settings.h"
#include "SoftAP.h"
#include "DIYBMSServer.h"
#include "PacketRequestGenerator.h"
#include "PacketReceiveProcessor.h"

// Instantiate queue to hold packets ready for transmission
cppQueue requestQueue(sizeof(PacketStruct), 24, FIFO);

cppQueue replyQueue(sizeof(PacketStruct), 8, FIFO);

PacketRequestGenerator prg = PacketRequestGenerator(&requestQueue);

PacketReceiveProcessor receiveProc = PacketReceiveProcessor();

// Memory to hold in and out serial buffer
uint8_t SerialPacketReceiveBuffer[2 * sizeof(PacketStruct)];

SerialEncoder myPacketSerial;

Ticker myTimerRelay;
Ticker myTimer;
Ticker myTransmitTimer;
Ticker myReplyTimer;
Ticker myLazyTimer;
Ticker wifiReconnectTimer;
Ticker mqttReconnectTimer;
Ticker myTimerSendMqttPacket;
Ticker myTimerSendMqttStatus;
Ticker myTimerSendInfluxdbPacket;
Ticker myTimerSwitchPulsedRelay;

uint16_t sequence = 0;

ControllerState ControlState = ControllerState::Unknown;

AsyncMqttClient mqttClient;

void setModbus(int dev, uint8_t addr, uint32_t min, uint32_t max, uint16_t reg, char *name, char *unit, char *desc)
{
  ModBus[dev].addr = addr;
  ModBus[dev].op = MB_READ_REGISTER;
  ModBus[dev].min = min;
  ModBus[dev].max = max;
  ModBus[dev].reg = reg;

  setModbusName(dev, name);
  setModbusUnit(dev, unit);
  setModbusDesc(dev, desc);

  ESP_LOGD(TAG, "%d %s %s %s", dev, (char *)ModBus[dev].name, (char *)ModBus[dev].unit, (char *)ModBus[dev].desc);
}

void InitModbus()
{
  sdm.begin();

  memset(ModBusVal, 0, sizeof(ModbusVal) * MODBUS_NUM); //initialize SDM communication

  setModbus(0, 31, 60, 3600, SDM_TOTAL_ACTIVE_ENERGY, (char *)"BAT_IN_E", (char *)"kWh", (char *)"Powersupply Energy");
  setModbus(1, 31, 10, 3600, SDM_PHASE_1_POWER, (char *)"BAT_IN_P", (char *)"W", (char *)"Powersupply Power");
  setModbus(2, 31, 10, 3600, SDM_PHASE_1_VOLTAGE, (char *)"BAT_IN_U", (char *)"V", (char *)"Powersupply Voltage");
  setModbus(3, 31, 60, 3600, SDM_PHASE_1_CURRENT, (char *)"BAT_IN_I", (char *)"A", (char *)"Powersupply Current");
  setModbus(4, 31, 10, 3600, SDM_FREQUENCY, (char *)"BAT_IN_F", (char *)"Hz", (char *)"Powersupply Frequency");
  setModbus(5, 31, 60, 3600, SDM_TOTAL_ACTIVE_ENERGY, (char *)"BAT_OUT_E", (char *)"kWh", (char *)"Powerwall AC Energy");
  setModbus(6, 31, 10, 3600, SDM_PHASE_1_POWER, (char *)"BAT_OUT_P", (char *)"W", (char *)"Powerwall AC Power");
  setModbus(7, 31, 1, 3600, SDM_PHASE_1_VOLTAGE, (char *)"BAT_OUT_U", (char *)"V", (char *)"Powerwall AC Voltage");
  setModbus(8, 31, 60, 3600, SDM_PHASE_1_CURRENT, (char *)"BAT_OUT_I", (char *)"A", (char *)"Powerwall AC Current");
  setModbus(9, 31, 5, 3600, SDM_FREQUENCY, (char *)"BAT_OUT_F", (char *)"Hz", (char *)"Powerwall AC Freqency");
}

void QueueLED(uint8_t bits)
{
  i2cQueueMessage m;
  //3 = LED
  m.command = 0x03;
  //Lowest 3 bits are RGB led GREEN/RED/BLUE
  m.data = bits & B00000111;
  xQueueSendToBack(queue_i2c, &m, 10 / portTICK_PERIOD_MS);
}

//Output a status log to the SD Card in CSV format
void sdcardlog_task(void *param)
{
  for (;;)
  {
    //Wait X seconds
    for (size_t i = 0; i < mysettings.loggingFrequencySeconds; i++)
    {
      //Delay 1 second
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    if (_sd_card_installed && mysettings.loggingEnabled && ControlState == ControllerState::Running)
    {
      //ESP_LOGD(TAG, "sdcardlog_task");

      struct tm timeinfo;
      //getLocalTime has delay() functions in it :-(
      if (getLocalTime(&timeinfo, 1))
      {
        timeinfo.tm_year += 1900;
        //Month is 0 to 11 based!
        timeinfo.tm_mon++;

        //ESP_LOGD(TAG, "%04u-%02u-%02u %02u:%02u:%02u", timeinfo.tm_year, timeinfo.tm_mon, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);

        char filename[32];
        sprintf(filename, "/data_%04u%02u%02u.csv", timeinfo.tm_year, timeinfo.tm_mon, timeinfo.tm_mday);

        File file;

        //Prevent other devices using the VSPI bus
        if (hal.GetVSPIMutex())
        {
          if (SD.exists(filename))
          {
            //Open existing file (assumes there is enough SD card space to log)
            file = SD.open(filename, FILE_APPEND);

            //ESP_LOGD(TAG, "Open log %s", filename);
          }
          else
          {
            //Create a new file
            uint64_t freeSpace = SD.totalBytes() - SD.usedBytes();

            //Ensure there is more than 25MB of free space on SD card before creating a file
            if (freeSpace > (uint64_t)(25 * 1024 * 1024))
            {
              //Create the file
              File file = SD.open(filename, FILE_WRITE);
              if (file)
              {
                //ESP_LOGD(TAG, "Create log %s", filename);

                file.print("DateTime,");

                for (uint8_t i = 0; i < TotalNumberOfCells(); i++)
                {
                  file.print("VoltagemV_");
                  file.print(i);
                  file.print(",InternalTemp_");
                  file.print(i);
                  file.print(",ExternalTemp_");
                  file.print(i);
                  file.print(",Bypass_");
                  file.print(i);
                  file.print(",PWM_");
                  file.print(i);
                  file.print(",BypassOverTemp_");
                  file.print(i);
                  file.print(",BadPackets_");
                  file.print(i);
                  file.print(",BalancemAh_");
                  file.print(i);

                  if (i < TotalNumberOfCells() - 1)
                  {
                    file.print(',');
                  }
                }
                file.println();
              }
            }
            else
            {
              ESP_LOGE(TAG, "SD card has less than 25MiB remaining, logging stopped");
              //We had an error, so switch off logging (this is only in memory so not written perm.)
              mysettings.loggingEnabled = false;
            }
          }

          if (file && mysettings.loggingEnabled)
          {
            char dataMessage[255];

            sprintf(dataMessage, "%04u-%02u-%02u %02u:%02u:%02u,", timeinfo.tm_year, timeinfo.tm_mon, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
            file.print(dataMessage);

            for (uint8_t i = 0; i < TotalNumberOfCells(); i++)
            {
              //This may output invalid data when controller is first powered up
              sprintf(dataMessage, "%u,%i,%i,%c,%u,%c,%u,%u",
                      cmi[i].voltagemV, cmi[i].internalTemp,
                      cmi[i].externalTemp, cmi[i].inBypass ? 'Y' : 'N',
                      (int)((float)cmi[i].PWMValue / (float)255.0 * 100), cmi[i].bypassOverTemp ? 'Y' : 'N',
                      cmi[i].badPacketCount, cmi[i].BalanceCurrentCount);
              file.print(dataMessage);
              if (i < TotalNumberOfCells() - 1)
              {
                file.print(',');
              }
            }
            file.println();
            file.close();

            ESP_LOGD(TAG, "Wrote to SD log");
          }
          else
          {
            ESP_LOGE(TAG, "Failed to create/append SD logging file");
            //We had an error opening the file, so switch off logging
            //mysettings.loggingEnabled = false;
          }
        }
        else
        {
          ESP_LOGE(TAG, "Invalid datetime");
        }

        //Must be the last thing...
        hal.ReleaseVSPIMutex();
      }
    }
  } //end for loop

  //vTaskDelete( NULL );
}

//Writes a status log of the OUTPUT STATUES to the SD Card in CSV format
void sdcardlog_outputs_task(void *param)
{
  for (;;)
  {
    //Wait until this task is triggered https://www.freertos.org/ulTaskNotifyTake.html
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if (_sd_card_installed && mysettings.loggingEnabled)
    {
      ESP_LOGD(TAG, "sdcardlog_outputs_task");

      struct tm timeinfo;
      //getLocalTime has delay() functions in it :-(
      if (getLocalTime(&timeinfo, 1))
      {
        timeinfo.tm_year += 1900;
        //Month is 0 to 11 based!
        timeinfo.tm_mon++;

        //ESP_LOGD(TAG, "%04u-%02u-%02u %02u:%02u:%02u", timeinfo.tm_year, timeinfo.tm_mon, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);

        char filename[32];
        sprintf(filename, "/output_status_%04u%02u%02u.csv", timeinfo.tm_year, timeinfo.tm_mon, timeinfo.tm_mday);

        File file;

        //Prevent other devices using the VSPI bus
        if (hal.GetVSPIMutex())
        {

          if (SD.exists(filename))
          {
            //Open existing file (assumes there is enough SD card space to log)
            file = SD.open(filename, FILE_APPEND);

            //ESP_LOGD(TAG, "Open log %s", filename);
          }
          else
          {
            //Create a new file
            uint64_t freeSpace = SD.totalBytes() - SD.usedBytes();

            //Ensure there is more than 25MB of free space on SD card before creating a file
            if (freeSpace > (uint64_t)(25 * 1024 * 1024))
            {
              //Create the file
              File file = SD.open(filename, FILE_WRITE);
              if (file)
              {
                //ESP_LOGD(TAG, "Create log %s", filename);

                file.print("DateTime,Bits,");

                for (uint8_t i = 0; i < RELAY_TOTAL; i++)
                {
                  file.print("Output_");
                  file.print(i);
                  if (i < RELAY_TOTAL - 1)
                  {
                    file.print(',');
                  }
                }
                file.println();
              }
            }
            else
            {
              ESP_LOGE(TAG, "SD card has less than 25MiB remaining, logging stopped");
              //We had an error, so switch off logging
              //mysettings.loggingEnabled = false;
            }
          }

          if (file && mysettings.loggingEnabled)
          {
            char dataMessage[255];

            sprintf(dataMessage, "%04u-%02u-%02u %02u:%02u:%02u,", timeinfo.tm_year, timeinfo.tm_mon, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
            file.print(dataMessage);
            file.print(hal.LastTCA6408Value(), BIN);
            file.print(',');

            for (uint8_t i = 0; i < RELAY_TOTAL; i++)
            {
              //This may output invalid data when controller is first powered up
              sprintf(dataMessage, "%c", previousRelayState[i] == RelayState::RELAY_ON ? 'Y' : 'N');
              file.print(dataMessage);
              if (i < RELAY_TOTAL - 1)
              {
                file.print(',');
              }
            }
            file.println();
            file.close();

            ESP_LOGD(TAG, "Wrote to SD log");
          }
          else
          {
            ESP_LOGE(TAG, "Failed to create/append SD logging file");
            //We had an error opening the file, so switch off logging
            //mysettings.loggingEnabled = false;
          }
        }
        else
        {
          ESP_LOGE(TAG, "Invalid datetime");
        }

        //Must be the last thing...
        hal.ReleaseVSPIMutex();
      } //end if
    }   //end if
  }     //end for loop

  //vTaskDelete( NULL );
}

void modbuscomms_task(void *param)
{
  static uint8_t ind = 0;

  for (;;)
  {
    //Wait until this task is triggered https://www.freertos.org/ulTaskNotifyTake.html
    //ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    //Wait 5 seconds
    vTaskDelay(5000 / portTICK_PERIOD_MS);

    if (MODBUS_NUM)
    {

      //if (ModBus[ind].min < (ts - ModBusVal[ind].last) / 1000)
      //{

      /*
      ModBusVal[ind].val = sdm.readVal(ModBus[ind].reg, ModBus[ind].addr);
*/
      //ModBusVal[ind].last = ts;

      //      SERIAL_DEBUG.printf("Read Modbus: %d %s: %f\n", ind, ModBus[ind].name, ModBusVal[ind].val);
      //}

      if (++ind >= MODBUS_NUM)
        ind = 0;
    }
  }

  //vTaskDelete( NULL );
}
//Disable the BOOT button from acting as a WIFI RESET
//button which clears the EEPROM settings for WIFI connection
void wifiresetdisable_task(void *param)
{
  for (;;)
  {
    //Wait until this task is triggered https://www.freertos.org/ulTaskNotifyTake.html
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    //Wait for 20 seconds before disabling button/pin
    for (size_t i = 0; i < 20; i++)
    {
      //Wait 1 second
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    hal.SwapGPIO0ToOutput();
  }

  //vTaskDelete( NULL );
}

void ledoff_task(void *param)
{
  for (;;)
  {
    //Wait until this task is triggered https://www.freertos.org/ulTaskNotifyTake.html
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    //Wait 100ms
    vTaskDelay(100 / portTICK_PERIOD_MS);
    //LED OFF
    QueueLED(RGBLED::OFF);
  }
}

// Handle all calls to i2c devices in this single task
// Provides thread safe mechanism to talk to i2c
void i2c_task(void *param)
{
  for (;;)
  {
    i2cQueueMessage m;

    if (xQueueReceive(queue_i2c, &m, portMAX_DELAY) == pdPASS)
    {
      // do some i2c task
      if (m.command == 0x01)
      {
        // Read ports A/B/C/D inputs (on TCA6408)
        uint8_t v = hal.ReadTCA6408InputRegisters();
        //P0=A
        InputState[0] = (v & B00000001) == 0 ? enumInputState::INPUT_LOW : enumInputState::INPUT_HIGH;
        //P1=B
        InputState[1] = (v & B00000010) == 0 ? enumInputState::INPUT_LOW : enumInputState::INPUT_HIGH;
        //P2=C
        InputState[2] = (v & B00000100) == 0 ? enumInputState::INPUT_LOW : enumInputState::INPUT_HIGH;
        //P3=D
        InputState[3] = (v & B00001000) == 0 ? enumInputState::INPUT_LOW : enumInputState::INPUT_HIGH;
        //P7=E
        InputState[4] = (v & B10000000) == 0 ? enumInputState::INPUT_LOW : enumInputState::INPUT_HIGH;
      }

      if (m.command == 0x02)
      {
        //Read ports
        //The 9534 deals with internal LED outputs and spare IO on J10
        uint8_t v = hal.ReadTCA9534InputRegisters();
        //P6 = spare I/O (on PCB pin)
        InputState[5] = (v & B01000000) == 0 ? enumInputState::INPUT_LOW : enumInputState::INPUT_HIGH;
        //P7 = Emergency Stop
        InputState[6] = (v & B10000000) == 0 ? enumInputState::INPUT_LOW : enumInputState::INPUT_HIGH;

        //Emergency Stop (J1) has triggered
        if (InputState[6] == enumInputState::INPUT_LOW)
        {
          emergencyStop = true;
        }
      }

      if (m.command == 0x03)
      {
        hal.Led(m.data);
      }

      if (m.command == 0x04)
      {
        hal.TFTScreenBacklight(m.data);
      }

      if (m.command >= 0xE0 && m.command <= 0xE0 + RELAY_TOTAL)
      {
        //Set state of relays
        hal.SetOutputState(m.command - 0xe0, (RelayState)m.data);
      }
    }
  }
}

volatile uint32_t WifiPasswordClearTime;
volatile bool ResetWifi = false;

// Check if BOOT button is pressed, if held down for more than 4 seconds
// trigger a wifi password reset/clear from EEPROM.
void IRAM_ATTR WifiPasswordClear()
{
  if (digitalRead(GPIO_NUM_0) == LOW)
  {
    //Button pressed, store time
    WifiPasswordClearTime = millis() + 4000;
    ResetWifi = false;
  }
  else
  {
    //Button released
    //Did user press button for longer than 4 seconds?
    if (millis() > WifiPasswordClearTime)
    {
      ResetWifi = true;
    }
  }
}

void IRAM_ATTR TCA6408Interrupt()
{
  if (queue_i2c == NULL)
    return;
  i2cQueueMessage m;
  m.command = 0x01;
  m.data = 0;
  xQueueSendToBackFromISR(queue_i2c, &m, NULL);
}

void IRAM_ATTR TCA9534AInterrupt()
{
  if (queue_i2c == NULL)
    return;
  i2cQueueMessage m;
  m.command = 0x02;
  m.data = 0;
  xQueueSendToBackFromISR(queue_i2c, &m, NULL);
}

/*
void dumpByte(uint8_t data)
{
  if (data <= 0x0F)
  {
    SERIAL_DEBUG.print('0');
  }
  SERIAL_DEBUG.print(data, HEX);
}
*/

const char *packetType(uint8_t cmd)
{
  switch (cmd)
  {
  case COMMAND::ResetBadPacketCounter:
    return "ResetC";
    break;
  case COMMAND::ReadVoltageAndStatus:
    return "RdVolt";
    break;
  case COMMAND::Identify:
    return "Ident";
    break;
  case COMMAND::ReadTemperature:
    return "RdTemp";
    break;
  case COMMAND::ReadBadPacketCounter:
    return "RdBadPkC";
    break;
  case COMMAND::ReadSettings:
    return "RdSettin";
    break;
  case COMMAND::WriteSettings:
    return "WriteSet";
    break;
  case COMMAND::ReadBalancePowerPWM:
    return "RdBalanc";
    break;
  case COMMAND::Timing:
    return "Timing";
    break;
  case COMMAND::ReadBalanceCurrentCounter:
    return "Current";
    break;
  case COMMAND::ReadPacketReceivedCounter:
    return "PktRvd";
    break;
  }

  return " ??????   ";
}

void dumpPacketToDebug(char indicator, PacketStruct *buffer)
{
  //Filter on some commands
  //if ((buffer->command & 0x0F) != COMMAND::Timing)    return;

  ESP_LOGD(TAG, "%c %02X-%02X H:%02X C:%02X SEQ:%04X CRC:%04X %s",
           indicator,
           buffer->start_address,
           buffer->end_address,
           buffer->hops,
           buffer->command,
           buffer->sequence,
           buffer->crc,
           packetType(buffer->command & 0x0F));

  //ESP_LOG_BUFFER_HEX("packet", &(buffer->moduledata[0]), sizeof(buffer->moduledata), ESP_LOG_DEBUG);
}

const char *ControllerStateString(ControllerState value)
{
  switch (value)
  {
  case ControllerState::PowerUp:
    return "PowerUp";
  case ControllerState::ConfigurationSoftAP:
    return "ConfigurationSoftAP";
  case ControllerState::Stabilizing:
    return "Stabilizing";
  case ControllerState::Running:
    return "Running";
  case ControllerState::Unknown:
    return "Unknown";
  }

  return "?";
}

void SetControllerState(ControllerState newState)
{
  if (ControlState != newState)
  {
    ESP_LOGI(TAG, "** Controller changed state from %s to %s **", ControllerStateString(ControlState), ControllerStateString(newState));

    ControlState = newState;

    switch (ControlState)
    {
    case ControllerState::PowerUp:
      //Purple during start up, don't use the QueueLED as thats not setup at this state
      hal.Led(RGBLED::Purple);
      break;
    case ControllerState::ConfigurationSoftAP:
      //Don't use the QueueLED as thats not setup at this state
      hal.Led(RGBLED::White);
      break;
    case ControllerState::Stabilizing:
      QueueLED(RGBLED::Yellow);
      break;
    case ControllerState::Running:
      QueueLED(RGBLED::Green);
      //Fire task to switch off BOOT button after 30 seconds
      xTaskNotify(wifiresetdisable_task_handle, 0x00, eNotifyAction::eNoAction);
      break;
    case ControllerState::Unknown:
      //Do nothing
      break;
    }
  }
}

uint16_t minutesSinceMidnight()
{
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo))
  {
    return 0;
  }
  else
  {
    return (timeinfo.tm_hour * 60) + timeinfo.tm_min;
  }
}

void serviceReplyQueue()
{
  //if (replyQueue.isEmpty()) return;

  while (!replyQueue.isEmpty())
  {
    PacketStruct ps;
    replyQueue.pop(&ps);

#if defined(PACKET_LOGGING_RECEIVE)
    // Process decoded incoming packet
    dumpPacketToDebug('R', &ps);
#endif

    if (receiveProc.ProcessReply(&ps))
    {
      //Success, do nothing
    }
    else
    {

      //Error blue
      QueueLED(RGBLED::Blue);

      ESP_LOGE(TAG, "Packet Failed");

      //SERIAL_DEBUG.print(F("*FAIL*"));
      dumpPacketToDebug('F', &ps);
    }
  }
}

void onPacketReceived()
{

  PacketStruct ps;
  memcpy(&ps, SerialPacketReceiveBuffer, sizeof(PacketStruct));

  if ((ps.command & 0x0F) == COMMAND::Timing)
  {
    //Timestamp at the earliest possible moment
    uint32_t t = millis();
    ps.moduledata[2] = (t & 0xFFFF0000) >> 16;
    ps.moduledata[3] = t & 0x0000FFFF;
    //Ensure CRC is correct
    ps.crc = CRC16::CalculateArray((uint8_t *)&ps, sizeof(PacketStruct) - 2);
  }

  if (!replyQueue.push(&ps))
  {
    ESP_LOGE(TAG, "*Failed to queue reply*");
  }
}

void timerTransmitCallback()
{
  if (requestQueue.isEmpty())
    return;

  // Called to transmit the next packet in the queue need to ensure this procedure
  // is called more frequently than items are added into the queue

  PacketStruct transmitBuffer;

  requestQueue.pop(&transmitBuffer);
  sequence++;
  transmitBuffer.sequence = sequence;

  if (transmitBuffer.command == COMMAND::Timing)
  {
    //Timestamp at the last possible moment
    uint32_t t = millis();
    transmitBuffer.moduledata[0] = (t & 0xFFFF0000) >> 16;
    transmitBuffer.moduledata[1] = t & 0x0000FFFF;
  }

  transmitBuffer.crc = CRC16::CalculateArray((uint8_t *)&transmitBuffer, sizeof(PacketStruct) - 2);
  myPacketSerial.sendBuffer((byte *)&transmitBuffer);

// Output the packet we just transmitted to debug console
#if defined(PACKET_LOGGING_SEND)
  dumpPacketToDebug('S', &transmitBuffer);
#endif
}

//Runs the rules and populates rule_outcome array with true/false for each rule
//Rules based on module parameters/readings like voltage and temperature
//are only processed once every module has returned at least 1 reading/communication
void ProcessRules()
{
  rules.ClearValues();
  rules.ClearWarnings();
  rules.ClearErrors();

  rules.rule_outcome[Rule::BMSError] = false;

  uint16_t totalConfiguredModules = TotalNumberOfCells();
  if (totalConfiguredModules > maximum_controller_cell_modules)
  {
    //System is configured with more than maximum modules - abort!
    rules.SetError(InternalErrorCode::TooManyModules);
  }

  if (receiveProc.totalModulesFound > 0 && receiveProc.totalModulesFound != totalConfiguredModules)
  {
    //Found more or less modules than configured for
    rules.SetError(InternalErrorCode::ModuleCountMismatch);
  }

  //Communications error...
  if (receiveProc.HasCommsTimedOut())
  {
    rules.SetError(InternalErrorCode::CommunicationsError);
  }

  uint8_t cellid = 0;
  for (int8_t bank = 0; bank < mysettings.totalNumberOfBanks; bank++)
  {
    for (int8_t i = 0; i < mysettings.totalNumberOfSeriesModules; i++)
    {
      rules.ProcessCell(bank, &cmi[cellid]);

      if (cmi[cellid].valid && cmi[cellid].settingsCached)
      {
        if (cmi[cellid].BypassThresholdmV != mysettings.BypassThresholdmV)
        {
          rules.SetWarning(InternalWarningCode::ModuleInconsistantBypassVoltage);
        }

        if (cmi[cellid].BypassOverTempShutdown != mysettings.BypassOverTempShutdown)
        {
          rules.SetWarning(InternalWarningCode::ModuleInconsistantBypassTemperature);
        }

        if (cmi[0].settingsCached && cmi[cellid].CodeVersionNumber != cmi[0].CodeVersionNumber)
        {
          //Do all the modules have the same version of code as module zero?
          rules.SetWarning(InternalWarningCode::ModuleInconsistantCodeVersion);
        }

        if (cmi[0].settingsCached && cmi[cellid].BoardVersionNumber != cmi[0].BoardVersionNumber)
        {
          //Do all the modules have the same hardware revision?
          rules.SetWarning(InternalWarningCode::ModuleInconsistantBoardRevision);
        }
      }

      cellid++;
    }
    rules.ProcessBank(bank);
  }

  if (mysettings.loggingEnabled && !_sd_card_installed)
  {
    rules.SetWarning(InternalWarningCode::LoggingEnabledNoSDCard);
  }

  if (rules.invalidModuleCount > 0)
  {
    //Some modules are not yet valid
    rules.SetError(InternalErrorCode::WaitingForModulesToReply);
  }

  if (ControlState == ControllerState::Running && rules.zeroVoltageModuleCount > 0)
  {
    rules.SetError(InternalErrorCode::ZeroVoltModule);
  }

  rules.RunRules(
      mysettings.rulevalue,
      mysettings.rulehysteresis,
      emergencyStop,
      minutesSinceMidnight());

  if (ControlState == ControllerState::Stabilizing)
  {
    //Check for zero volt modules - not a problem whilst we are in stabilizing start up mode
    if (rules.zeroVoltageModuleCount == 0 && rules.invalidModuleCount == 0)
    {
      //Every module has been read and they all returned a voltage move to running state
      SetControllerState(ControllerState::Running);
    }
  }

  if (emergencyStop)
  {
    //Lowest 3 bits are RGB led GREEN/RED/BLUE
    QueueLED(RGBLED::Red);
  }
}

void timerSwitchPulsedRelay()
{
  //Set defaults based on configuration
  for (int8_t y = 0; y < RELAY_TOTAL; y++)
  {
    if (previousRelayPulse[y])
    {
      //We now need to rapidly turn off the relay after a fixed period of time (pulse mode)
      //However we leave the relay and previousRelayState looking like the relay has triggered (it has!)
      //to prevent multiple pulses being sent on each rule refresh

      i2cQueueMessage m;
      //Different command for each relay
      m.command = 0xE0 + y;
      m.data = previousRelayState[y] == RelayState::RELAY_ON ? RelayState::RELAY_OFF : RelayState::RELAY_ON;
      xQueueSendToBack(queue_i2c, &m, 10 / portTICK_PERIOD_MS);

      previousRelayPulse[y] = false;
    }
  }

  //This only fires once
  myTimerSwitchPulsedRelay.detach();

  //Fire task to record state of outputs to SD Card
  xTaskNotify(sdcardlog_outputs_task_handle, 0x00, eNotifyAction::eNoAction);
}

void timerProcessRules()
{

  //Run the rules
  ProcessRules();

#if defined(RULES_LOGGING)
  for (int8_t r = 0; r < RELAY_RULES; r++)
  {
    if (rules.rule_outcome[r])
    {
      ESP_LOGD(TAG, "Rule outcome %i=TRUE", r);
    }
  }
#endif

  RelayState relay[RELAY_TOTAL];

  //Set defaults based on configuration
  for (int8_t y = 0; y < RELAY_TOTAL; y++)
  {
    relay[y] = mysettings.rulerelaydefault[y] == RELAY_ON ? RELAY_ON : RELAY_OFF;
  }

  //Test the rules (in reverse order)
  for (int8_t n = RELAY_RULES - 1; n >= 0; n--)
  {
    if (rules.rule_outcome[n] == true)
    {

      for (int8_t y = 0; y < RELAY_TOTAL; y++)
      {
        //Dont change relay if its set to ignore/X
        if (mysettings.rulerelaystate[n][y] != RELAY_X)
        {
          if (mysettings.rulerelaystate[n][y] == RELAY_ON)
          {
            relay[y] = RELAY_ON;
          }
          else
          {
            relay[y] = RELAY_OFF;
          }
        }
      }
    }
  }

  uint8_t changes = 0;
  for (int8_t n = 0; n < RELAY_TOTAL; n++)
  {
    if (previousRelayState[n] != relay[n])
    {
      changes++;
//Would be better here to use the WRITE8 to lower i2c traffic
#if defined(RULES_LOGGING)
      ESP_LOGI(TAG, "Relay %i=%i", n, relay[n]);
#endif
      //This would be better if we worked out the bit pattern first and then just
      //submitted that as a single i2c read/write transaction

      i2cQueueMessage m;
      //Different command for each relay
      m.command = 0xE0 + n;
      m.data = relay[n];
      xQueueSendToBack(queue_i2c, &m, 10 / portTICK_PERIOD_MS);

      previousRelayState[n] = relay[n];

      if (mysettings.relaytype[n] == RELAY_PULSE)
      {
        //TODO: This needs changing to task notify passing in the relay number to that task

        //If its a pulsed relay, invert the output quickly via a single shot timer
        //a pulse is unlikely to be captured in the SDCard log, as its over quickly
        previousRelayPulse[n] = true;
        myTimerSwitchPulsedRelay.attach(0.2, timerSwitchPulsedRelay);

#if defined(RULES_LOGGING)
        ESP_LOGI(TAG, "Relay %i PULSED", n);
#endif
      }
    }
  }

  if (changes)
  {
    //Fire task to record state of outputs to SD Card
    xTaskNotify(sdcardlog_outputs_task_handle, 0x00, eNotifyAction::eNoAction);
  }
}

void timerEnqueueCallback()
{
  QueueLED(RGBLED::Green);
  //Fire task to switch off LED in a few ms
  xTaskNotify(ledoff_task_handle, 0x00, eNotifyAction::eNoAction);

  //this is called regularly on a timer, it determines what request to make to the modules (via the request queue)
  uint16_t i = 0;
  uint16_t max = TotalNumberOfCells();

  uint8_t startmodule = 0;

  while (i < max)
  {
    uint16_t endmodule = (startmodule + maximum_cell_modules_per_packet) - 1;

    //Limit to number of modules we have configured
    if (endmodule > max)
    {
      endmodule = max - 1;
    }

    //Need to watch overflow of the uint8 here...
    prg.sendCellVoltageRequest(startmodule, endmodule);
    prg.sendCellTemperatureRequest(startmodule, endmodule);

    //If any module is in bypass then request PWM reading for whole bank
    for (uint8_t m = startmodule; m <= endmodule; m++)
    {
      if (cmi[m].inBypass)
      {
        prg.sendReadBalancePowerRequest(startmodule, endmodule);
        //We only need 1 reading for whole bank
        break;
      }
    }

    //Move to the next bank
    startmodule = endmodule + 1;
    i += maximum_cell_modules_per_packet;
  }
}

void connectToWifi()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    WiFi.mode(WIFI_STA);

    char hostname[40];

    uint32_t chipId = 0;
    for (int i = 0; i < 17; i = i + 8)
    {
      chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
    }
    sprintf(hostname, "DIYBMS-%08X", chipId);
    WiFi.setHostname(hostname);

    ESP_LOGI(TAG, "Hostname: %s", hostname);

    WiFi.begin(DIYBMSSoftAP::WifiSSID(), DIYBMSSoftAP::WifiPassword());
  }

  WifiDisconnected = false;
}

void connectToMqtt()
{
  ESP_LOGI(TAG, "Connecting to MQTT...");
  mqttClient.connect();
}

static AsyncClient *aClient = NULL;

void setupInfluxClient()
{

  if (aClient) //client already exists
    return;

  aClient = new AsyncClient();
  if (!aClient) //could not allocate client
    return;

  aClient->onError([](void *arg, AsyncClient *client, err_t error) {
    ESP_LOGE(TAG, "Influx connect error");

    aClient = NULL;
    delete client;
  },
                   NULL);

  aClient->onConnect([](void *arg, AsyncClient *client) {
    ESP_LOGI("Influx connected");

    //Send the packet here

    aClient->onError(NULL, NULL);

    client->onDisconnect([](void *arg, AsyncClient *c) {
      ESP_LOGI("Influx disconnected");
      aClient = NULL;
      delete c;
    },
                         NULL);

    client->onData([](void *arg, AsyncClient *c, void *data, size_t len) {
      //Data received
      ESP_LOGD("Influx data received");
      //SERIAL_DEBUG.print(F("\r\nData: "));
      //SERIAL_DEBUG.println(len);
      //uint8_t* d = (uint8_t*)data;
      //for (size_t i = 0; i < len; i++) {SERIAL_DEBUG.write(d[i]);}
    },
                   NULL);

    //send the request

    //Construct URL for the influxdb
    //See API at https://docs.influxdata.com/influxdb/v1.7/tools/api/#write-http-endpoint

    String poststring;

    for (uint8_t bank = 0; bank < mysettings.totalNumberOfBanks; bank++)
    {
      //TODO: We should send a request per bank not just a single POST as we are likely to exceed capabilities of ESP
      for (uint8_t i = 0; i < mysettings.totalNumberOfSeriesModules; i++)
      {
        //Data in LINE PROTOCOL format https://docs.influxdata.com/influxdb/v1.7/write_protocols/line_protocol_tutorial/
        poststring = poststring + "cells," + "cell=" + String(bank) + "_" + String(i) + " v=" + String((float)cmi[i].voltagemV / 1000.0, 3) + ",i=" + String(cmi[i].internalTemp) + "i" + ",e=" + String(cmi[i].externalTemp) + "i" + ",b=" + (cmi[i].inBypass ? String("true") : String("false")) + "\n";
      }
    }

    //TODO: Need to URLEncode these values
    String url = "/write?db=" + String(mysettings.influxdb_database) + "&u=" + String(mysettings.influxdb_user) + "&p=" + String(mysettings.influxdb_password);
    String header = "POST " + url + " HTTP/1.1\r\n" + "Host: " + String(mysettings.influxdb_host) + "\r\n" + "Connection: close\r\n" + "Content-Length: " + poststring.length() + "\r\n" + "Content-Type: text/plain\r\n" + "\r\n";

    //SERIAL_DEBUG.println(header.c_str());
    //SERIAL_DEBUG.println(poststring.c_str());

    client->write(header.c_str());
    client->write(poststring.c_str());

    ESP_LOGD("Influx data sent");
  },
                     NULL);
}

void SendInfluxdbPacket()
{
  if (!mysettings.influxdb_enabled)
    return;

  ESP_LOGI("SendInfluxdbPacket");

  setupInfluxClient();

  if (!aClient->connect(mysettings.influxdb_host, mysettings.influxdb_httpPort))
  {
    ESP_LOGE(TAG, "Influxdb connect fail");
    AsyncClient *client = aClient;
    aClient = NULL;
    delete client;
  }
}

void startTimerToInfluxdb()
{
  myTimerSendInfluxdbPacket.attach(30, SendInfluxdbPacket);
}

void SetupOTA()
{

  ArduinoOTA.setPort(3232);

  ArduinoOTA.setPassword("1jiOOx12AQgEco4e");

  ArduinoOTA
      .onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
          type = "sketch";
        else // U_SPIFFS
          type = "filesystem";

        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
        ESP_LOGI(TAG, "Start updating %s", type);
      });
  ArduinoOTA.onEnd([]() {
    ESP_LOGD(TAG, "\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    ESP_LOGD(TAG, "Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    ESP_LOGD(TAG, "Error [%u]: ", error);
    if (error == OTA_AUTH_ERROR)
      ESP_LOGE(TAG, "Auth Failed");
    else if (error == OTA_BEGIN_ERROR)
      ESP_LOGE(TAG, "Begin Failed");
    else if (error == OTA_CONNECT_ERROR)
      ESP_LOGE(TAG, "Connect Failed");
    else if (error == OTA_RECEIVE_ERROR)
      ESP_LOGE(TAG, "Receive Failed");
    else if (error == OTA_END_ERROR)
      ESP_LOGE(TAG, "End Failed");
  });

  ArduinoOTA.setHostname(WiFi.getHostname());
  ArduinoOTA.setMdnsEnabled(true);
  ArduinoOTA.begin();
}


void mountSDCard()
{
  /*
SD CARD TEST
*/
  ESP_LOGI(TAG, "Mounting SD card");

  hal.GetVSPIMutex();
  // Initialize SD card
  SD.begin(SDCARD_CHIPSELECT, hal.vspi);
  if (SD.begin(SDCARD_CHIPSELECT))
  {
    uint8_t cardType = SD.cardType();
    if (cardType == CARD_NONE)
    {
      ESP_LOGI(TAG, "No SD card attached");
    }
    else
    {
      ESP_LOGI(TAG, "SD card available");
      _sd_card_installed = true;
    }
  }
  else
  {
    ESP_LOGE(TAG, "Card Mount Failed");
  }
  hal.ReleaseVSPIMutex();
}

void sdcardaction_callback(uint8_t action)
{
  switch (action)
  {
  case 0:
    //Unmount
    ESP_LOGI(TAG, "Unmounting SD card");
    hal.GetVSPIMutex();
    SD.end();
    hal.ReleaseVSPIMutex();
    _sd_card_installed = false;
    break;
  case 1:
    mountSDCard();
    break;
  }
}

void onWifiConnect(WiFiEvent_t event, WiFiEventInfo_t info)
{

  ESP_LOGI(TAG, "Wi-Fi status=%i", (uint16_t)WiFi.status());

  ESP_LOGI(TAG, "Request NTP from %s", mysettings.ntpServer);

  //Use native ESP32 code
  configTime(mysettings.minutesTimeZone * 60, mysettings.daylight * 60, mysettings.ntpServer);

  /*
  TODO: CHECK ERROR CODES BETTER!
  0 : WL_IDLE_STATUS when Wi-Fi is in process of changing between statuses
  1 : WL_NO_SSID_AVAIL in case configured SSID cannot be reached
  3 : WL_CONNECTED after successful connection is established
  4 : WL_CONNECT_FAILED if password is incorrect
  6 : WL_DISCONNECTED if module is not configured in station mode
  */
  if (!server_running)
  {
    DIYBMSServer::StartServer(&server, &mysettings, &SD, &prg, &receiveProc, &ControlState, &rules, &ModBus, &ModBusVal, &sdcardaction_callback,&hal);
    server_running = true;
    
  }

  if (mysettings.mqtt_enabled)
  {
    connectToMqtt();
  }

  if (mysettings.influxdb_enabled)
  {
    startTimerToInfluxdb();
  }

  SetupOTA();

  // Set up mDNS responder:
  // - first argument is the domain name, in this example
  //   the fully-qualified domain name is "esp8266.local"
  // - second argument is the IP address to advertise
  //   we send our IP address on the WiFi network
  if (!MDNS.begin(WiFi.getHostname()))
  {
    ESP_LOGE(TAG, "Error setting up MDNS responder!");
  }
  else
  {
    ESP_LOGI("mDNS responder started");
    // Add service to MDNS-SD
    MDNS.addService("http", "tcp", 80);
  }
}

void onWifiDisconnect(WiFiEvent_t event, WiFiEventInfo_t info)
{
  ESP_LOGE(TAG, "Disconnected from Wi-Fi.");

  //Indicate to loop() to reconnect, seems to be
  //ESP issues using Wifi from timers - https://github.com/espressif/arduino-esp32/issues/2686
  WifiDisconnected = true;

  // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
  mqttReconnectTimer.detach();
  myTimerSendMqttPacket.detach();
  myTimerSendMqttStatus.detach();
  myTimerSendInfluxdbPacket.detach();

  //wifiReconnectTimer.once(2, connectToWifi);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
  ESP_LOGE(TAG, "Disconnected from MQTT.");

  myTimerSendMqttPacket.detach();
  myTimerSendMqttStatus.detach();

  if (WiFi.isConnected())
  {
    mqttReconnectTimer.once(2, connectToMqtt);
  }
}

void sendMqttStatus()
{
  if (!mysettings.mqtt_enabled && !mqttClient.connected())
    return;

  char topic[80];
  char jsonbuffer[220];
  DynamicJsonDocument doc(220);
  JsonObject root = doc.to<JsonObject>();

  root["banks"] = mysettings.totalNumberOfBanks;
  root["cells"] = mysettings.totalNumberOfSeriesModules;
  root["uptime"] = millis() / 1000; // I want to know the uptime of the device.

  JsonArray bankVoltage = root.createNestedArray("bankVoltage");
  for (int8_t bank = 0; bank < mysettings.totalNumberOfBanks; bank++)
  {
    bankVoltage.add((float)rules.packvoltage[bank] / (float)1000.0);
  }

  JsonObject monitor = root.createNestedObject("monitor");

  // Set error flag if we have attempted to send 2*number of banks without a reply
  monitor["commserr"] = receiveProc.HasCommsTimedOut() ? 1 : 0;
  monitor["sent"] = prg.packetsGenerated;
  monitor["received"] = receiveProc.packetsReceived;
  monitor["badcrc"] = receiveProc.totalCRCErrors;
  monitor["ignored"] = receiveProc.totalNotProcessedErrors;
  monitor["oos"] = receiveProc.totalOutofSequenceErrors;
  monitor["roundtrip"] = receiveProc.packetTimerMillisecond;

  serializeJson(doc, jsonbuffer, sizeof(jsonbuffer));
  sprintf(topic, "%s/status", mysettings.mqtt_topic);
  mqttClient.publish(topic, 0, false, jsonbuffer);
#if defined(MQTT_LOGGING)
  ESP_LOGD(TAG, "MQTT %s %s", topic, jsonbuffer);
//SERIAL_DEBUG.print("MQTT - ");SERIAL_DEBUG.print(topic);  SERIAL_DEBUG.print('=');  SERIAL_DEBUG.println(jsonbuffer);
#endif

  //Using Json for below reduced MQTT messages from 14 to 2. Could be combined into same json object too. But even better is status + event driven.
  doc.clear(); // Need to clear the json object for next message
  sprintf(topic, "%s/rule", mysettings.mqtt_topic);
  for (uint8_t i = 0; i < RELAY_RULES; i++)
  {
    doc[(String)i] = rules.rule_outcome[i] ? 1 : 0; // String conversion should be removed but just quick to get json format nice
  }
  serializeJson(doc, jsonbuffer, sizeof(jsonbuffer));
#if defined(MQTT_LOGGING)
  ESP_LOGD(TAG, "MQTT %s %s", topic, jsonbuffer);
#endif
  mqttClient.publish(topic, 0, false, jsonbuffer);

  doc.clear(); // Need to clear the json object for next message
  sprintf(topic, "%s/output", mysettings.mqtt_topic);
  for (uint8_t i = 0; i < RELAY_TOTAL; i++)
  {
    doc[(String)i] = (previousRelayState[i] == RelayState::RELAY_ON) ? 1 : 0;
  }

  serializeJson(doc, jsonbuffer, sizeof(jsonbuffer));
#if defined(MQTT_LOGGING)
  ESP_LOGD(TAG, "MQTT %s %s", topic, jsonbuffer);
#endif
  mqttClient.publish(topic, 0, false, jsonbuffer);
}

//Send a few MQTT packets and keep track so we send the next batch on following calls
uint8_t mqttStartModule = 0;

void sendMqttPacket()
{
#if defined(MQTT_LOGGING)
  ESP_LOGI(TAG, "Send MQTT Packet");
#endif

  if (!mysettings.mqtt_enabled && !mqttClient.connected())
    return;

  char topic[80];
  char jsonbuffer[200];
  StaticJsonDocument<200> doc;

  //If the BMS is in error, stop sending MQTT packets for the data
  if (!rules.rule_outcome[Rule::BMSError])
  {
    uint8_t counter = 0;
    for (uint8_t i = mqttStartModule; i < TotalNumberOfCells(); i++)
    {
      //Only send valid module data
      if (cmi[i].valid)
      {
        uint8_t bank = i / mysettings.totalNumberOfSeriesModules;
        uint8_t module = i - (bank * mysettings.totalNumberOfSeriesModules);

        doc.clear();
        doc["voltage"] = (float)cmi[i].voltagemV / (float)1000.0;
        doc["vMax"] = (float)cmi[i].voltagemVMax / (float)1000.0;
        doc["vMin"] = (float)cmi[i].voltagemVMin / (float)1000.0;
        doc["inttemp"] = cmi[i].internalTemp;
        doc["exttemp"] = cmi[i].externalTemp;
        doc["bypass"] = cmi[i].inBypass ? 1 : 0;
        doc["PWM"] = (int)((float)cmi[i].PWMValue / (float)255.0 * 100);
        doc["bypassT"] = cmi[i].bypassOverTemp ? 1 : 0;
        doc["bpc"] = cmi[i].badPacketCount;
        doc["mAh"] = cmi[i].BalanceCurrentCount;
        serializeJson(doc, jsonbuffer, sizeof(jsonbuffer));

        sprintf(topic, "%s/%d/%d", mysettings.mqtt_topic, bank, module);

        mqttClient.publish(topic, 0, false, jsonbuffer);

#if defined(MQTT_LOGGING)
        ESP_LOGI(TAG, "MQTT %s %s", topic, jsonbuffer);
#endif
      }

      counter++;

      //After transmitting this many packets over MQTT, store our current state and exit the function.
      //this prevents flooding the ESP controllers wifi stack and potentially causing reboots/fatal exceptions
      if (counter == 6)
      {
        mqttStartModule = i + 1;

        if (mqttStartModule > TotalNumberOfCells())
        {
          mqttStartModule = 0;
        }

        return;
      }
    }

    //Completed the loop, start at zero
    mqttStartModule = 0;
  }
}

void onMqttConnect(bool sessionPresent)
{
  ESP_LOGI("Connected to MQTT");
  myTimerSendMqttPacket.attach(5, sendMqttPacket);
  myTimerSendMqttStatus.attach(25, sendMqttStatus);
}

void LoadConfiguration()
{
  if (Settings::ReadConfig("diybms", (char *)&mysettings, sizeof(mysettings)))
    return;

  ESP_LOGI("Apply default config");

  //Zero all the bytes
  memset(&mysettings, 0, sizeof(mysettings));

  //Default to a single module
  mysettings.totalNumberOfBanks = 1;
  mysettings.totalNumberOfSeriesModules = 1;
  mysettings.BypassOverTempShutdown = 65;
  //4.10V bypass
  mysettings.BypassThresholdmV = 4100;
  mysettings.graph_voltagehigh = 4.5;
  mysettings.graph_voltagelow = 2.75;

  //EEPROM settings are invalid so default configuration
  mysettings.mqtt_enabled = false;
  mysettings.mqtt_port = 1883;

  mysettings.loggingEnabled = false;
  mysettings.loggingFrequencySeconds = 15;

  //Default to EMONPI default MQTT settings
  strcpy(mysettings.mqtt_topic, "diybms");
  strcpy(mysettings.mqtt_server, "192.168.0.26");
  strcpy(mysettings.mqtt_username, "emonpi");
  strcpy(mysettings.mqtt_password, "emonpimqtt2016");

  mysettings.influxdb_enabled = false;
  strcpy(mysettings.influxdb_host, "myinfluxserver");
  strcpy(mysettings.influxdb_database, "database");
  strcpy(mysettings.influxdb_user, "user");
  strcpy(mysettings.influxdb_password, "");

  mysettings.timeZone = 0;
  mysettings.minutesTimeZone = 0;
  mysettings.daylight = false;
  strcpy(mysettings.ntpServer, "time.google.com");

  for (size_t x = 0; x < RELAY_TOTAL; x++)
  {
    mysettings.rulerelaydefault[x] = RELAY_OFF;
  }

  //Emergency stop
  mysettings.rulevalue[Rule::EmergencyStop] = 0;
  //Internal BMS error (communication issues, fault readings from modules etc)
  mysettings.rulevalue[Rule::BMSError] = 0;
  //Individual cell over voltage
  mysettings.rulevalue[Rule::Individualcellovervoltage] = 4150;
  //Individual cell under voltage
  mysettings.rulevalue[Rule::Individualcellundervoltage] = 3000;
  //Individual cell over temperature (external probe)
  mysettings.rulevalue[Rule::IndividualcellovertemperatureExternal] = 55;
  //Pack over voltage (mV)
  mysettings.rulevalue[Rule::IndividualcellundertemperatureExternal] = 5;
  //Pack under voltage (mV)
  mysettings.rulevalue[Rule::PackOverVoltage] = 4200 * 8;
  //RULE_PackUnderVoltage
  mysettings.rulevalue[Rule::PackUnderVoltage] = 3000 * 8;
  mysettings.rulevalue[Rule::Timer1] = 60 * 8;  //8am
  mysettings.rulevalue[Rule::Timer2] = 60 * 17; //5pm

  mysettings.rulevalue[Rule::ModuleOverTemperatureInternal] = 60;
  mysettings.rulevalue[Rule::ModuleUnderTemperatureInternal] = 50;

  for (size_t i = 0; i < RELAY_RULES; i++)
  {
    mysettings.rulehysteresis[i] = mysettings.rulevalue[i];

    //Set all relays to don't care
    for (size_t x = 0; x < RELAY_TOTAL; x++)
    {
      mysettings.rulerelaystate[i][x] = RELAY_X;
    }
  }

  for (size_t x = 0; x < RELAY_TOTAL; x++)
  {
    mysettings.relaytype[x] = RELAY_STANDARD;
  }
}

uint8_t lazyTimerMode = 0;
//Do activities which are not critical to the system like background loading of config, or updating timing results etc.
void timerLazyCallback()
{
  if (requestQueue.getRemainingCount() < 6)
  {
    //Exit here to avoid overflowing the queue
    ESP_LOGE(TAG, "ERR: Lazy overflow Q=%i", requestQueue.getRemainingCount());
    return;
  }

  lazyTimerMode++;

  if (lazyTimerMode == 1)
  {
    //Send a "ping" message through the cells to get a round trip time
    prg.sendTimingRequest();
    return;
  }

  if (lazyTimerMode == 2)
  {
    uint8_t counter = 0;
    //Find modules that don't have settings cached and request them
    for (uint8_t module = 0; module < TotalNumberOfCells(); module++)
    {
      if (cmi[module].valid && !cmi[module].settingsCached)
      {
        if (requestQueue.getRemainingCount() < 6)
        {
          //Exit here to avoid flooding the queue
          return;
        }

        prg.sendGetSettingsRequest(module);
        counter++;
      }
    }

    return;
  }

  //Send these requests to all banks of modules
  uint16_t i = 0;
  uint16_t max = TotalNumberOfCells();

  uint8_t startmodule = 0;

  while (i < max)
  {
    uint16_t endmodule = (startmodule + maximum_cell_modules_per_packet) - 1;

    //Limit to number of modules we have configured
    if (endmodule > max)
    {
      endmodule = max - 1;
    }

    //Need to watch overflow of the uint8 here...
    //prg.sendCellVoltageRequest(startmodule, endmodule);

    if (lazyTimerMode == 3)
    {
      prg.sendReadBalanceCurrentCountRequest(startmodule, endmodule);
    }

    if (lazyTimerMode == 4)
    {
      //Just for debug, only do the first 16 modules
      prg.sendReadPacketsReceivedRequest(startmodule, endmodule);
    }

    //Ask for bad packet count (saves battery power if we dont ask for this all the time)
    if (lazyTimerMode == 5)
    {
      prg.sendReadBadPacketCounter(startmodule, endmodule);
    }

    //Move to the next bank
    startmodule = endmodule + 1;
    i += maximum_cell_modules_per_packet;
  }

  //Reset at end of cycle
  if (lazyTimerMode >= 5)
  {
    lazyTimerMode = 0;
  }
}

void resetAllRules()
{
  //Clear all rules
  for (int8_t r = 0; r < RELAY_RULES; r++)
  {
    rules.rule_outcome[r] = false;
  }
}

bool CaptureSerialInput(HardwareSerial stream, char *buffer, int buffersize, bool OnlyDigits, bool ShowPasswordChar)
{
  int length = 0;
  unsigned long timer = millis() + 30000;

  while (true)
  {

    //Abort after 30 seconds of inactivity
    if (millis() > timer)
      return false;

    //We should add a timeout in here, and return FALSE when we abort....
    while (stream.available())
    {
      //Reset timer on serial input
      timer = millis() + 30000;

      int data = stream.read();
      if (data == '\b' || data == '\177')
      { // BS and DEL
        if (length)
        {
          length--;
          stream.write("\b \b");
        }
      }
      else if (data == '\n')
      {
        //Ignore
      }
      else if (data == '\r')
      {
        if (length > 0)
        {
          stream.write("\r\n"); // output CRLF
          buffer[length] = '\0';

          //Soak up any other characters on the buffer and throw away
          while (stream.available())
          {
            stream.read();
          }

          //Return to caller
          return true;
        }

        length = 0;
      }
      else if (length < buffersize - 1)
      {
        if (OnlyDigits && (data < '0' || data > '9'))
        {
          //We need to filter out non-digit characters
        }
        else
        {
          buffer[length++] = data;
          if (ShowPasswordChar)
          {
            //Hide real character
            stream.write('*');
          }
          else
          {
            stream.write(data);
          }
        }
      }
    }
  }
}

void TerminalBasedWifiSetup()
{
  SERIAL_DEBUG.println(F("\r\n\r\nDIYBMS CONTROLLER - Scanning Wifi"));

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  int n = WiFi.scanNetworks();

  if (n == 0)
    SERIAL_DEBUG.println(F("no networks found"));
  else
  {
    for (int i = 0; i < n; ++i)
    {
      if (i < 10)
      {
        SERIAL_DEBUG.print(' ');
      }
      SERIAL_DEBUG.print(i);
      SERIAL_DEBUG.print(':');
      SERIAL_DEBUG.print(WiFi.SSID(i));

      //Pad out the wifi names into 2 columns
      for (size_t spaces = WiFi.SSID(i).length(); spaces < 36; spaces++)
      {
        SERIAL_DEBUG.print(' ');
      }

      if ((i + 1) % 2 == 0)
      {
        SERIAL_DEBUG.println();
      }
      delay(5);
    }
    SERIAL_DEBUG.println();
  }

  WiFi.mode(WIFI_OFF);

  SERIAL_DEBUG.print(F("Enter the NUMBER of the Wifi network to connect to:"));

  bool result;
  char buffer[10];
  result = CaptureSerialInput(SERIAL_DEBUG, buffer, 10, true, false);
  if (result)
  {
    int index = String(buffer).toInt();
    SERIAL_DEBUG.print(F("Enter the password to use when connecting to '"));
    SERIAL_DEBUG.print(WiFi.SSID(index));
    SERIAL_DEBUG.print("':");

    char passwordbuffer[80];
    result = CaptureSerialInput(SERIAL_DEBUG, passwordbuffer, 80, false, true);

    if (result)
    {
      wifi_eeprom_settings config;
      memset(&config, 0, sizeof(config));
      WiFi.SSID(index).toCharArray(config.wifi_ssid, sizeof(config.wifi_ssid));
      strcpy(config.wifi_passphrase, passwordbuffer);
      Settings::WriteConfig("diybmswifi", (char *)&config, sizeof(config));
    }
  }

  SERIAL_DEBUG.println(F("REBOOTING IN 5..."));
  delay(5000);
  ESP.restart();
}

void tft_display_off()
{
  tft.fillScreen(TFT_BLACK);
  //hal.TFTScreenBacklight(true);

  //Queue up i2c message
  i2cQueueMessage m;
  //4 = TFT backlight LED
  m.command = 0x04;
  //Lowest 3 bits are RGB led GREEN/RED/BLUE
  m.data = false;
  xQueueSendToBack(queue_i2c, &m, 10 / portTICK_PERIOD_MS);
}

void init_tft_display()
{
  tft.init();
  tft.initDMA(); // Initialise the DMA engine (tested with STM32F446 and STM32F767)
  tft.getSPIinstance().setHwCs(false);
  tft.setRotation(3);
  tft.fillScreen(SplashLogoPalette[0]);

  //SplashLogoGraphic_Height
  tft.pushImage((int32_t)TFT_HEIGHT / 2 - SplashLogoGraphic_Width / 2, (int32_t)4, (int32_t)152, (int32_t)48, SplashLogoGraphic, false, SplashLogoPalette);

  tft.setTextColor(TFT_WHITE, SplashLogoPalette[0]);

  tft.setCursor(0, 48 + 16, 4);
  tft.print(F("Ver:"));
  tft.println(GIT_VERSION_SHORT);
  tft.println(F("Build Date:"));
  tft.println(COMPILE_DATE_TIME_SHORT);

  hal.TFTScreenBacklight(true);
}

void createFile(fs::FS &fs, const char *path, const char *message)
{
  //ESP_LOGD(TAG,"Writing file: %s", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file)
  {
    ESP_LOGE(TAG, "Failed to open file for writing");
    return;
  }
  if (file.print(message))
  {
    ESP_LOGD(TAG, "File written");
  }
  else
  {
    ESP_LOGE(TAG, "Write failed");
  }
  file.close();
}

void appendFile(fs::FS &fs, const char *path, const char *message)
{
  //ESP_LOGD(TAG,("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file)
  {
    ESP_LOGE(TAG, "Failed to open file for appending");
    return;
  }
  if (file.print(message))
  {
    ESP_LOGD(TAG, "Message appended");
  }
  else
  {
    ESP_LOGE(TAG, "Append failed");
  }
  file.close();
}

/*
void listDir(fs::FS &fs, const char *dirname, uint8_t levels)
{
  SERIAL_DEBUG.printf("Listing directory: %s\r\n", dirname);

  File root = fs.open(dirname);
  if (!root)
  {
    SERIAL_DEBUG.println("- failed to open directory");
    return;
  }
  if (!root.isDirectory())
  {
    SERIAL_DEBUG.println(" - not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file)
  {
    if (file.isDirectory())
    {
      SERIAL_DEBUG.print("  DIR : ");

#ifdef CONFIG_LITTLEFS_FOR_IDF_3_2
      SERIAL_DEBUG.println(file.name());
#else
      SERIAL_DEBUG.print(file.name());
      time_t t = file.getLastWrite();
      struct tm *tmstruct = localtime(&t);
      SERIAL_DEBUG.printf("  LAST WRITE: %d-%02d-%02d %02d:%02d:%02d\n", (tmstruct->tm_year) + 1900, (tmstruct->tm_mon) + 1, tmstruct->tm_mday, tmstruct->tm_hour, tmstruct->tm_min, tmstruct->tm_sec);
#endif

      if (levels)
      {
        listDir(fs, file.name(), levels - 1);
      }
    }
    else
    {
      SERIAL_DEBUG.print("  FILE: ");
      SERIAL_DEBUG.print(file.name());
      SERIAL_DEBUG.print("  SIZE: ");

#ifdef CONFIG_LITTLEFS_FOR_IDF_3_2
      SERIAL_DEBUG.println(file.size());
#else
      SERIAL_DEBUG.print(file.size());
      time_t t = file.getLastWrite();
      struct tm *tmstruct = localtime(&t);
      SERIAL_DEBUG.printf("  LAST WRITE: %d-%02d-%02d %02d:%02d:%02d\n", (tmstruct->tm_year) + 1900, (tmstruct->tm_mon) + 1, tmstruct->tm_mday, tmstruct->tm_hour, tmstruct->tm_min, tmstruct->tm_sec);
#endif
    }
    file = root.openNextFile();
  }
}
*/

/*
static const char *ESP32_CAN_STATUS_STRINGS[] = {
    "STOPPED",               // CAN_STATE_STOPPED
    "RUNNING",               // CAN_STATE_RUNNING
    "OFF / RECOVERY NEEDED", // CAN_STATE_BUS_OFF
    "RECOVERY UNDERWAY"      // CAN_STATE_RECOVERING
};
*/

void setup()
{
  WiFi.mode(WIFI_OFF);

  btStop();
  esp_log_level_set("*", ESP_LOG_DEBUG);    // set all components to WARN level
  esp_log_level_set("wifi", ESP_LOG_WARN);  // enable WARN logs from WiFi stack
  esp_log_level_set("dhcpc", ESP_LOG_WARN); // enable INFO logs from DHCP client

  const char *diybms_logo = "\r\n\r\n\r\n                _          __ \r\n    _|  o      |_)  |\\/|  (_  \r\n   (_|  |  \\/  |_)  |  |  __) \r\n           /                  ";

  //ESP32 we use the USB serial interface for console/debug messages
  SERIAL_DEBUG.begin(115200, SERIAL_8N1);
  SERIAL_DEBUG.setDebugOutput(true);

  SERIAL_DEBUG.println(diybms_logo);

  ESP_LOGI(TAG, "CONTROLLER - ver:%s compiled %s", GIT_VERSION, COMPILE_DATE_TIME);

  esp_chip_info_t chip_info;
  esp_chip_info(&chip_info);

  ESP_LOGI(TAG, "ESP32 Chip model = %u, Rev %u, Cores=%u, Features=%u", chip_info.model, chip_info.revision, chip_info.cores, chip_info.features);

  //We generate a unique number which is used in all following JSON requests
  //we use this as a simple method to avoid cross site scripting attacks
  DIYBMSServer::generateUUID();

  hal.ConfigurePins(WifiPasswordClear);
  hal.ConfigureI2C(TCA6408Interrupt, TCA9534AInterrupt);
  hal.ConfigureVSPI();

  SetControllerState(ControllerState::PowerUp);

  hal.Led(0);

  if (!LITTLEFS.begin(false))
  {
    ESP_LOGE(TAG, "LITTLEFS mount failed, did you upload file system image? - SYSTEM HALTED");

    hal.Halt();
  }
  else
  {
    ESP_LOGI(TAG, "LITTLEFS mounted, totalBytes=%u, usedBytes=%u", LITTLEFS.totalBytes(), LITTLEFS.usedBytes());
    //listDir(LITTLEFS, "/", 0);
  }

  mountSDCard();

  /*
  SERIAL_DEBUG.println("Start ATMEL ISP programming...");

  hal.SwapGPIO0ToOutput();

//const size_t size_file_diybms_module_firmware_400_avrbin = 7718;
//const size_t size_file_diybms_module_firmware_blinky_avrbin = 584;

  uint32_t starttime=millis();
//  AVRISP_PROGRAMMER isp = AVRISP_PROGRAMMER(&hal.vspi, GPIO_NUM_0, false, VSPI_SCK);
//  bool progresult = isp.ProgramAVRDevice((uint32_t)0x1e9315, size_file_diybms_module_firmware_blinky_avrbin, file_diybms_module_firmware_blinky_avrbin, 0b11100010, 0b11010110, 0b11111110);
//  bool progresult = isp.ProgramAVRDevice((uint32_t)0x1e9315, size_file_diybms_module_firmware_400_avrbin, file_diybms_module_firmware_400_avrbin, 0b11100010, 0b11010110, 0b11111110);
//328P -Uefuse:w:0xFD:m -Uhfuse:w:0xDA:m -Ulfuse:w:0xFF:m
//  bool progresult = isp.ProgramAVRDevice((uint32_t)0x1E950F, size_file_atmega328p_fade_avrbin, file_atmega328p_fade_avrbin, 0xFF, 0xDA, 0xFD);

  uint32_t endtime=millis();

  SERIAL_DEBUG.println(progresult);
  SERIAL_DEBUG.println(isp.device_signature,HEX);
  SERIAL_DEBUG.print("Duration: ");
  SERIAL_DEBUG.println(endtime-starttime);

delay(5000);
  //Must reconfigure SPI after using ISP Programmer (changes clock speed etc.)
  hal.ConfigureVSPI();
  SERIAL_DEBUG.println("FINISH");
  SERIAL_DEBUG.flush();
*/

  /* TEST RS485 */
  /*
  SERIAL_DEBUG.println("TEST RS485");
  SERIAL_RS485.begin(38400, SERIAL_8N1, RS485_RX, RS485_TX, false, 20000UL);

  while (1)
  {
    digitalWrite(RS485_ENABLE, HIGH);
    SERIAL_RS485.write((char)'A');
    SERIAL_RS485.write((char)'B');
    SERIAL_RS485.write((char)'C');
    SERIAL_RS485.write((char)'D');
    SERIAL_RS485.write((char)'E');
    SERIAL_RS485.write((char)'F');
    SERIAL_RS485.flush();
    digitalWrite(RS485_ENABLE, LOW);

    //Allow responses to build up in buffer
    delay(50);

    while (SERIAL_RS485.available())
    {
      SERIAL_DEBUG.print((char)SERIAL_RS485.read());
    }

    delay(100);
  }
*/

  /*
TEST CAN BUS
*/

  /*
  //Initialize configuration structures using macro initializers
  can_general_config_t g_config = CAN_GENERAL_CONFIG_DEFAULT(gpio_num_t::GPIO_NUM_16, gpio_num_t::GPIO_NUM_17, CAN_MODE_NORMAL);
  g_config.mode = CAN_MODE_NORMAL;

  can_timing_config_t t_config = CAN_TIMING_CONFIG_125KBITS();
  can_filter_config_t f_config = CAN_FILTER_CONFIG_ACCEPT_ALL();

  //Install CAN driver
  if (can_driver_install(&g_config, &t_config, &f_config) == ESP_OK)
  {
    SERIAL_DEBUG.printf("Driver installed\n");
  }
  else
  {
    SERIAL_DEBUG.printf("Failed to install driver\n");
  }

  //Start CAN driver
  if (can_start() == ESP_OK)
  {
    SERIAL_DEBUG.println("Driver started\n");
  }
  else
  {
    SERIAL_DEBUG.println("Failed to start driver\n");
  }

  while (1)
  {

    //Wait for message to be received
    can_message_t message;
    esp_err_t res = can_receive(&message, pdMS_TO_TICKS(5000));
    if (res == ESP_OK)
    {
      //SERIAL_DEBUG.println("Message received\n");

      SERIAL_DEBUG.printf("\nID is %d=", message.identifier);
      if (!(message.flags & CAN_MSG_FLAG_RTR))
      {
        for (int i = 0; i < message.data_length_code; i++)
        {
          dumpByte(message.data[i]);
        }
      }
      //SERIAL_DEBUG.println();
    }
    else if (res == ESP_ERR_TIMEOUT)
    {
      /// ignore the timeout or do something
      SERIAL_DEBUG.println("Timeout");
    }

    // check the health of the bus
    can_status_info_t status;
    can_get_status_info(&status);
    SERIAL_DEBUG.printf("  rx-q:%d, tx-q:%d, rx-err:%d, tx-err:%d, arb-lost:%d, bus-err:%d, state: %s",
                        status.msgs_to_rx, status.msgs_to_tx, status.rx_error_counter, status.tx_error_counter, status.arb_lost_count,
                        status.bus_error_count, ESP32_CAN_STATUS_STRINGS[status.state]);
    if (status.state == CAN_STATE_BUS_OFF)
    {
      // When the bus is OFF we need to initiate recovery, transmit is
      // not possible when in this state.
      SERIAL_DEBUG.printf("ESP32-CAN: initiating recovery");
      can_initiate_recovery();
    }
    else if (status.state == CAN_STATE_RECOVERING)
    {
      // when the bus is in recovery mode transmit is not possible.
      delay(200);
    }
    delay(100);
  }
*/

  hal.ConfigureVSPI();
  init_tft_display();

  //Init the touch screen
  touchscreen.begin(hal.vspi);
  touchscreen.setRotation(3);

  //All comms to i2c needs to go through this single task
  //to prevent issues with thread safety on the i2c hardware/libraries
  queue_i2c = xQueueCreate(10, sizeof(i2cQueueMessage));

  //Needs to be before xTaskCreate(modbuscomms_task
  InitModbus();

  //Create i2c task on CPU 0 (normal code runs on CPU 1)
  xTaskCreatePinnedToCore(i2c_task, "i2c", 2048, nullptr, 2, &i2c_task_handle, 0);
  xTaskCreatePinnedToCore(ledoff_task, "ledoff", 1048, nullptr, 1, &ledoff_task_handle, 0);
  xTaskCreate(wifiresetdisable_task, "wifidbl", 1048, nullptr, 1, &wifiresetdisable_task_handle);
  xTaskCreate(modbuscomms_task, "modbusc", 2048, nullptr, 1, &modbuscomms_task_handle);
  xTaskCreate(sdcardlog_task, "sdlog", 4096, nullptr, 1, &sdcardlog_task_handle);
  xTaskCreate(sdcardlog_outputs_task, "sdout", 4096, nullptr, 1, &sdcardlog_outputs_task_handle);

  //Pre configure the array
  memset(&cmi, 0, sizeof(cmi));
  for (size_t i = 0; i < maximum_controller_cell_modules; i++)
  {
    DIYBMSServer::clearModuleValues(i);
  }

  resetAllRules();

  //Receive is IO2 which means the RX1 plug must be disconnected for programming to work!
  SERIAL_DATA.begin(COMMS_BAUD_RATE, SERIAL_8N1, 2, 32); // Serial for comms to modules

  myPacketSerial.begin(&SERIAL_DATA, &onPacketReceived, sizeof(PacketStruct), SerialPacketReceiveBuffer, sizeof(SerialPacketReceiveBuffer));

  LoadConfiguration();

  //Set relay defaults
  for (int8_t y = 0; y < RELAY_TOTAL; y++)
  {
    previousRelayState[y] = mysettings.rulerelaydefault[y];
    //Set relay defaults
    hal.SetOutputState(y, mysettings.rulerelaydefault[y]);
  }
  //Fire task to record state of outputs to SD Card
  xTaskNotify(sdcardlog_outputs_task_handle, 0x00, eNotifyAction::eNoAction);

  //Allow user to press SPACE BAR key on serial terminal
  //to enter text based WIFI setup
  SERIAL_DEBUG.print(F("Press SPACE BAR to enter terminal based configuration...."));
  for (size_t i = 0; i < (3000 / 250); i++)
  {
    SERIAL_DEBUG.print('.');
    while (SERIAL_DEBUG.available())
    {
      int x = SERIAL_DEBUG.read();
      //SPACE BAR
      if (x == 32)
      {
        TerminalBasedWifiSetup();
      }
    }
    delay(250);
  }
  SERIAL_DEBUG.println(F("skipped"));

  //Temporarly force WIFI settings
  //wifi_eeprom_settings xxxx;
  //strcpy(xxxx.wifi_ssid,"XXXXXX");
  //strcpy(xxxx.wifi_passphrase,"XXXXXX");
  //Settings::WriteConfig("diybmswifi",(char *)&config, sizeof(config));
  //clearAPSettings = 0;

  if (!DIYBMSSoftAP::LoadConfigFromEEPROM())
  {
    //We have just started up and the EEPROM is empty of configuration
    SetControllerState(ControllerState::ConfigurationSoftAP);

    //SERIAL_DEBUG.print(F("Clear AP settings"));
    //SERIAL_DEBUG.println(clearAPSettings);
    ESP_LOGI(TAG, "Setup Access Point");
    //We are in initial power on mode (factory reset)
    DIYBMSSoftAP::SetupAccessPoint(&server);
  }
  else
  {

    ESP_LOGI(TAG, "Connecting to WIFI");

    /* Explicitly set the ESP to be a WiFi-client, otherwise by default,
      would try to act as both a client and an access-point */

    WiFi.onEvent(onWifiConnect, system_event_id_t::SYSTEM_EVENT_STA_GOT_IP);
    WiFi.onEvent(onWifiDisconnect, system_event_id_t::SYSTEM_EVENT_STA_DISCONNECTED);
    //Newed IDF version will need this...
    //WiFi.onEvent(onWifiConnect, arduino_event_id_t::ARDUINO_EVENT_WIFI_STA_GOT_IP);
    //WiFi.onEvent(onWifiDisconnect, arduino_event_id_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);

    mqttClient.onConnect(onMqttConnect);
    mqttClient.onDisconnect(onMqttDisconnect);

    if (mysettings.mqtt_enabled)
    {
      ESP_LOGD(TAG, "MQTT Enabled");
      mqttClient.setServer(mysettings.mqtt_server, mysettings.mqtt_port);
      mqttClient.setCredentials(mysettings.mqtt_username, mysettings.mqtt_password);
    }

    //Ensure we service the cell modules every 6 seconds
    myTimer.attach(6, timerEnqueueCallback);

    //Process rules every 5 seconds
    myTimerRelay.attach(5, timerProcessRules);

    //We process the transmit queue every 1 second (this needs to be lower delay than the queue fills)
    //and slower than it takes a single module to process a command (about 300ms)
    myTransmitTimer.attach(1, timerTransmitCallback);

    //Service reply queue
    myReplyTimer.attach(1, serviceReplyQueue);

    //This is a lazy timer for low priority tasks
    myLazyTimer.attach(8, timerLazyCallback);

    //We have just started...
    SetControllerState(ControllerState::Stabilizing);

    tft_display_off();
  }
}

void loop()
{
  //Allow CPU to sleep some
  //delay(10);
  //ESP_LOGW("LOOP","LOOP");

  if (touchscreen.tirqTouched())
  {
    if (touchscreen.touched())
    {
      /*
      TS_Point p = touchscreen.getPoint();
      SERIAL_DEBUG.print("Pressure = ");
      SERIAL_DEBUG.print(p.z);
      SERIAL_DEBUG.print(", x = ");
      SERIAL_DEBUG.print(p.x);
      SERIAL_DEBUG.print(", y = ");
      SERIAL_DEBUG.print(p.y);
      SERIAL_DEBUG.println();
      */
    }
  }

  if (WifiDisconnected && ControlState != ControllerState::ConfigurationSoftAP)
  {
    connectToWifi();
  }

  if (ResetWifi)
  {
    //Password reset, turn LED CYAN
    QueueLED(RGBLED::Cyan);

    //Wipe EEPROM WIFI setting
    DIYBMSSoftAP::FactoryReset();
  }

  ArduinoOTA.handle();

  // Call update to receive, decode and process incoming packets.
  myPacketSerial.checkInputStream();

  /*
  if (ConfigHasChanged > 0)
  {
    //Auto reboot if needed (after changing MQTT or INFLUX settings)
    //Ideally we wouldn't need to reboot if the code could sort itself out!
    ConfigHasChanged--;
    if (ConfigHasChanged == 0)
    {
      ESP_LOGI(TAG, "RESTART AFTER CONFIG CHANGE");
      //Stop networking
      if (mqttClient.connected())
      {
        mqttClient.disconnect(true);
      }
      WiFi.disconnect();
      ESP.restart();
    }
  }
*/
}
