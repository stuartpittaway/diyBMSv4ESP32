/*

 ____  ____  _  _  ____  __  __  ___    _  _  __
(  _ \(_  _)( \/ )(  _ \(  \/  )/ __)  ( \/ )/. |
 )(_) )_)(_  \  /  ) _ < )    ( \__ \   \  /(_  _)
(____/(____) (__) (____/(_/\/\_)(___/    \/   (_)

  (c) 2017 to 2022 Stuart Pittaway

  This is the code for the ESP32 controller - it talks to the V4.X cell modules over isolated serial bus

  This code runs on ESP32 DEVKIT-C and compiles with VS CODE and PLATFORM IO environment.

  Unless you are making code changes, please use the pre-compiled version from GITHUB instead.
*/

#if defined(ESP8266)
#error ESP8266 is not supported by this code
#endif

#define USE_ESP_IDF_LOG 1
static constexpr const char *const TAG = "diybms";

#define CONFIG_DISABLE_HAL_LOCKS 1

#include "esp_log.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_ota_ops.h"
#include "esp_flash_partitions.h"
#include "esp_partition.h"
#include "nvs_flash.h"
#include "nvs.h"
extern "C"
{
#include "esp_core_dump.h"
}

#include <Arduino.h>

// #define PACKET_LOGGING_RECEIVE
// #define PACKET_LOGGING_SEND
// #define RULES_LOGGING

#include "FS.h"
#include "LittleFS.h"
#include <ESPmDNS.h>

#include "time.h"
#include <esp_ipc.h>
#include <esp_wifi.h>
#include <esp_bt.h>

#include <esp_event.h>

// Libraries for SD card
#include "SD.h"
#include "driver/gpio.h"
#include "driver/twai.h"
#include "driver/adc.h"
#include <driver/uart.h>
#include <esp_http_server.h>
#include <esp_sntp.h>
#include <SerialEncoder.h>

#include <ArduinoJson.h>
#include "defines.h"
#include "HAL_ESP32.h"
#include "Rules.h"
#include "settings.h"
#include "avrisp_programmer.h"
#include "tft.h"
#include "influxdb.h"
#include "mqtt.h"
#include "victron_canbus.h"
#include "pylon_canbus.h"
#include "pylonforce_canbus.h"
#include "string_utils.h"

#include <SPI.h>
#include "CurrentMonitorINA229.h"

#include "history.h"

CurrentMonitorINA229 currentmon_internal = CurrentMonitorINA229();
extern void randomCharacters(char *value, int length);
const uart_port_t rs485_uart_num = UART_NUM_1;

const std::string wificonfigfilename("/diybms/wifi.json");

HAL_ESP32 hal;

volatile bool emergencyStop = false;
bool _sd_card_installed = false;

// Used for WIFI hostname and also sent to Victron over CANBUS
std::string hostname;
std::string ip_string; // xxx.xxx.xxx.xxx

bool wifi_isconnected = false;

History history = History();

// holds modbus data
uint8_t frame[256];

CardAction card_action = CardAction::Idle;

// Screen variables in tft.cpp
extern bool _tft_screen_available;
extern uint8_t tftsleep_timer;
extern volatile bool _screen_awake;
extern bool force_tft_wake;
extern TimerHandle_t tftwake_timer;
extern void tftwakeup(TimerHandle_t xTimer);

// HTTPD server handle in webserver.cpp
extern httpd_handle_t _myserver;

wifi_eeprom_settings _wificonfig;

Rules rules;
diybms_eeprom_settings mysettings;
uint8_t TotalNumberOfCells() { return mysettings.totalNumberOfBanks * mysettings.totalNumberOfSeriesModules; }

uint32_t canbus_messages_received = 0;
uint32_t canbus_messages_received_error = 0;
uint32_t canbus_messages_sent = 0;
uint32_t canbus_messages_failed_sent = 0;
int64_t canbus_last_305_message_time = 0;

bool server_running = false;
RelayState previousRelayState[RELAY_TOTAL];
bool previousRelayPulse[RELAY_TOTAL];

volatile enumInputState InputState[INPUTS_TOTAL];

currentmonitoring_struct currentMonitor;

TimerHandle_t led_off_timer;
TimerHandle_t pulse_relay_off_timer;

TaskHandle_t sdcardlog_task_handle = nullptr;
TaskHandle_t sdcardlog_outputs_task_handle = nullptr;
TaskHandle_t rule_state_change_task_handle = nullptr;
TaskHandle_t avrprog_task_handle = nullptr;
TaskHandle_t enqueue_task_handle = nullptr;
TaskHandle_t transmit_task_handle = nullptr;
TaskHandle_t replyqueue_task_handle = nullptr;
TaskHandle_t lazy_task_handle = nullptr;
TaskHandle_t rule_task_handle = nullptr;

TaskHandle_t voltageandstatussnapshot_task_handle = nullptr;
TaskHandle_t updatetftdisplay_task_handle = nullptr;
TaskHandle_t periodic_task_handle = nullptr;
TaskHandle_t interrupt_task_handle = nullptr;
TaskHandle_t rs485_tx_task_handle = nullptr;
TaskHandle_t rs485_rx_task_handle = nullptr;
TaskHandle_t service_rs485_transmit_q_task_handle = nullptr;
TaskHandle_t canbus_tx_task_handle = nullptr;
TaskHandle_t canbus_rx_task_handle = nullptr;

// This large array holds all the information about the modules
CellModuleInfo cmi[maximum_controller_cell_modules];

avrprogramsettings _avrsettings;

// Number of bytes of the largest MODBUS request we make
const uint16_t MAX_SEND_RS485_PACKET_LENGTH = 36;

QueueHandle_t rs485_transmit_q_handle;
QueueHandle_t request_q_handle;
QueueHandle_t reply_q_handle;

#include "crc16.h"
#include "settings.h"

#include "PacketRequestGenerator.h"
#include "PacketReceiveProcessor.h"
#include "webserver.h"

PacketRequestGenerator prg = PacketRequestGenerator();
PacketReceiveProcessor receiveProc = PacketReceiveProcessor();

// Memory to hold in and out serial buffer
uint8_t SerialPacketReceiveBuffer[2 * sizeof(PacketStruct)];

SerialEncoder myPacketSerial;

uint16_t sequence = 0;

ControllerState _controller_state = ControllerState::Unknown;

uint32_t time100 = 0;
uint32_t time20 = 0;
uint32_t time10 = 0;

void LED(uint8_t bits)
{
  hal.Led(bits);
}

// When triggered, the VOLTAGE and STATUS in the CellModuleInfo structure are accurate and consistant at this point in time.
// Good point to apply rules and update screen/statistics
[[noreturn]] void voltageandstatussnapshot_task(void *)
{
  for (;;)
  {
    // Wait until this task is triggered, when
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if (_tft_screen_available)
    {
      // Refresh the TFT display
      xTaskNotify(updatetftdisplay_task_handle, 0x00, eNotifyAction::eNoAction);
    }

  } // end for
}

std::string ip4_to_string(const uint32_t ipaddr)
{
  return std::to_string((uint8_t)(ipaddr))
      .append(".")
      .append(std::to_string((uint8_t)(ipaddr >> 8)))
      .append(".")
      .append(std::to_string((uint8_t)(ipaddr >> 16)))
      .append(".")
      .append(std::to_string((uint8_t)(ipaddr >> 24)));
}

// Sets the RS485 serial parameters after they have been changed
void ConfigureRS485()
{

  if (hal.GetRS485Mutex())
  {
    ESP_LOGD(TAG, "Configure RS485");
    ESP_ERROR_CHECK_WITHOUT_ABORT(uart_set_parity(rs485_uart_num, mysettings.rs485parity));
    ESP_ERROR_CHECK_WITHOUT_ABORT(uart_set_stop_bits(rs485_uart_num, mysettings.rs485stopbits));
    ESP_ERROR_CHECK_WITHOUT_ABORT(uart_set_baudrate(rs485_uart_num, mysettings.rs485baudrate));
    ESP_ERROR_CHECK_WITHOUT_ABORT(uart_set_word_length(rs485_uart_num, mysettings.rs485databits));

    hal.ReleaseRS485Mutex();
  }
  else
  {
    ESP_ERROR_CHECK(ESP_FAIL);
  }
}

void SetupRS485()
{
  ESP_LOGD(TAG, "Setup RS485");
  /* TEST RS485 */

  // Zero all data to start with
  memset(&currentMonitor, 0, sizeof(currentmonitoring_struct));

  uart_config_t uart_config = {
      .baud_rate = mysettings.rs485baudrate,
      .data_bits = mysettings.rs485databits,
      .parity = mysettings.rs485parity,
      .stop_bits = mysettings.rs485stopbits,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .rx_flow_ctrl_thresh = 122,
  };

  // Configure UART parameters
  ESP_ERROR_CHECK(uart_param_config(rs485_uart_num, &uart_config));

  // Set UART1 pins(TX: IO23, RX: I022, RTS: IO18, CTS: Not used)
  ESP_ERROR_CHECK(uart_set_pin(rs485_uart_num, RS485_TX, RS485_RX, RS485_ENABLE, UART_PIN_NO_CHANGE));

  // Install UART driver (we don't need an event queue here)
  ESP_ERROR_CHECK(uart_driver_install(rs485_uart_num, 256, 256, 0, nullptr, 0));

  // Set RS485 half duplex mode
  ESP_ERROR_CHECK(uart_set_mode(rs485_uart_num, uart_mode_t::UART_MODE_RS485_HALF_DUPLEX));

  ConfigureRS485();
}

void mountSDCard()
{
  card_action = CardAction::Idle;

  if (_avrsettings.programmingModeEnabled)
  {
    ESP_LOGW(TAG, "Attempt to mount sd but AVR prog mode enabled");
    return;
  }

  ESP_LOGI(TAG, "Mounting SD card");

  _sd_card_installed = hal.MountSDCard();
}

void unmountSDCard()
{
  card_action = CardAction::Idle;

  if (_sd_card_installed == false)
    return;

  if (_avrsettings.programmingModeEnabled)
  {
    ESP_LOGW(TAG, "Attempt to UNMOUNT SD card but AVR prog mode enabled");
    return;
  }

  ESP_LOGI(TAG, "Unmounting SD card");
  hal.UnmountSDCard();
  _sd_card_installed = false;
}

void wake_up_tft(bool force)
{
  // Wake up the display
  if (tftwake_timer != nullptr)
  {
    force_tft_wake = force;
    if (xTimerStart(tftwake_timer, pdMS_TO_TICKS(50)) != pdPASS)
    {
      ESP_LOGE(TAG, "TFT wake timer error");
    }
  }
}

[[noreturn]] void avrprog_task(void *param)
{
  for (;;)
  {
    // Wait until this task is triggered
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    wake_up_tft(true);

    // TODO: This needs to be passed into this as a parameter
    avrprogramsettings *s;
    s = (avrprogramsettings *)param;

    ESP_LOGD(TAG, "AVR inprogress=%i", s->inProgress);
    ESP_LOGI(TAG, "AVR setting e=%02X h=%02X l=%02X mcu=%08X file=%s", s->efuse, s->hfuse, s->lfuse, s->mcu, s->filename);

    // Now we load the file into program array, from LITTLEFS (SPIFF)
    if (LittleFS.exists(s->filename))
    {
      File binaryfile = LittleFS.open(s->filename);

      s->programsize = binaryfile.size();

      // Reserve the SPI bus for programming purposes
      if (hal.GetVSPIMutex())
      {
        // This will block for the 6 seconds it takes to program ATTINY841...
        // although AVRISP_PROGRAMMER will call the watchdog to prevent reboots

        auto starttime = millis();

        auto isp = AVRISP_PROGRAMMER(hal.VSPI_Ptr(), GPIO_NUM_0, false, VSPI_SCK);

        ESP_LOGI(TAG, "Programming AVR");

        s->progresult = isp.ProgramAVRDevice(&tftdisplay_avrprogrammer_progress, s->mcu, s->programsize, binaryfile, s->lfuse, s->hfuse, s->efuse);
        s->duration = (uint32_t)(millis() - starttime);

        // Return VSPI to default speed/settings
        hal.ConfigureVSPI();

        hal.ReleaseVSPIMutex();

        if (s->progresult == AVRISP_PROGRAMMER_RESULT::SUCCESS)
        {
          ESP_LOGI(TAG, "Success");
        }
        else
        {
          ESP_LOGE(TAG, "Failed %i", s->progresult);
        }

        binaryfile.close();
      }
      else
      {
        s->progresult = AVRISP_PROGRAMMER_RESULT::OTHER_FAILURE;
        ESP_LOGE(TAG, "Unable to obtain Mutex");
      }
    }
    else
    {
      s->progresult = AVRISP_PROGRAMMER_RESULT::OTHER_FAILURE;
      ESP_LOGE(TAG, "AVR file not found %s", s->filename);
    }

    s->inProgress = false;

    // Refresh the display, after programming is complete
    xTaskNotify(updatetftdisplay_task_handle, 0x00, eNotifyAction::eNoAction);
  } // end for
}

bool check_sdcard_freespace()
{
  // Create a new file
  uint64_t freeSpace = SD.totalBytes() - SD.usedBytes();

  // Ensure there is more than 25MB of free space on SD card before creating a file
  if (freeSpace < (uint64_t)(25 * 1024 * 1024))
  {
    ESP_LOGE(TAG, "SD card has less than 25MiB remaining, logging stopped");
    // We had an error, so switch off logging (this is only in memory so not written perm.)
    mysettings.loggingEnabled = false;
    return false;
  }

  return true;
}

/// @brief Log cell monitoring data to SDCARD
/// @param filename
/// @param timeinfo
void log_cell_monitoring_data_to_sdcard(std::string filename, const tm timeinfo)
{
  if (!check_sdcard_freespace())
  {
    return;
  }

  File file;
  auto exists = SD.exists(filename.c_str());

  // Open/create file
  ESP_LOGD(TAG, "%s log %s", exists ? "Append" : "Create", filename.c_str());
  file = SD.open(filename.c_str(), exists ? FILE_APPEND : FILE_WRITE);

  if (!exists)
  {
    file.print("DateTime,");

    std::string header;
    header.reserve(150);

    for (auto i = 0; i < TotalNumberOfCells(); i++)
    {
      std::string n;
      n = std::to_string(i);

      header.clear();

      header.append("VoltagemV_")
          .append(n)
          .append(",InternalTemp_")
          .append(n)
          .append(",ExternalTemp_")
          .append(n)
          .append(",Bypass_")
          .append(n)
          .append(",PWM_")
          .append(n)
          .append(",BypassOverTemp_")
          .append(n)
          .append(",BadPackets_")
          .append(n)
          .append(",BalancemAh_")
          .append(n);

      if (i < TotalNumberOfCells() - 1)
      {
        header.append(",");
      }
      else
      {
        header.append("\r\n");
      }

      file.write((const uint8_t *)header.c_str(), header.length());
    }
  }

  std::string dataMessage;
  dataMessage.reserve(150);

  dataMessage.append(pad_zero(4, (uint16_t)timeinfo.tm_year))
      .append("-")
      .append(pad_zero(2, (uint16_t)timeinfo.tm_mon))
      .append("-")
      .append(pad_zero(2, (uint16_t)timeinfo.tm_mday))
      .append(" ")
      .append(pad_zero(2, (uint16_t)timeinfo.tm_hour))
      .append(":")
      .append(pad_zero(2, (uint16_t)timeinfo.tm_min))
      .append(":")
      .append(pad_zero(2, (uint16_t)timeinfo.tm_sec))
      .append(",");

  for (auto i = 0; i < TotalNumberOfCells(); i++)
  {
    // This may output invalid data when controller is first powered up
    dataMessage.append(std::to_string(cmi[i].voltagemV))
        .append(",")
        .append(std::to_string(cmi[i].internalTemp))
        .append(",")
        .append(std::to_string(cmi[i].externalTemp))
        .append(",")
        .append(cmi[i].inBypass ? "Y" : "N")
        .append(",")
        .append(std::to_string((int)((float)cmi[i].PWMValue / (float)255.0 * 100)))
        .append(",")
        .append(cmi[i].bypassOverTemp ? "Y" : "N")
        .append(",")
        .append(std::to_string(cmi[i].badPacketCount))
        .append(",")
        .append(std::to_string(cmi[i].BalanceCurrentCount));

    if (i < TotalNumberOfCells() - 1)
    {
      dataMessage.append(",");
    }
    else
    {
      dataMessage.append("\r\n");
    }

    // Write the string out to file on each cell to avoid generating a huge string
    file.write((const uint8_t *)dataMessage.c_str(), dataMessage.length());

    // Start another string
    dataMessage.clear();
  }
  file.flush();
  file.close();

  ESP_LOGI(TAG, "Cell monitor log file");
}

/// @brief Log current monitoring data to SD CARD
/// @param cmon_filename
/// @param timeinfo
void log_current_data_to_sdcard(std::string cmon_filename, const tm timeinfo)
{
  if (!check_sdcard_freespace())
  {
    return;
  }

  auto exists = SD.exists(cmon_filename.c_str());

  File file2;

  // Open existing file (assumes there is enough SD card space to log)
  file2 = SD.open(cmon_filename.c_str(), exists ? FILE_APPEND : FILE_WRITE);
  ESP_LOGD(TAG, "%s file %s", exists ? "Append" : "Create", cmon_filename.c_str());

  if (!exists)
  {
    file2.println("DateTime,valid,voltage,current,mAhIn,mAhOut,DailymAhIn,DailymAhOut,power,temperature,relayState");
  }

  std::string dataMessage;
  dataMessage.reserve(128);

  dataMessage.append(pad_zero(4, (uint16_t)timeinfo.tm_year))
      .append("-")
      .append(pad_zero(2, (uint16_t)timeinfo.tm_mon))
      .append("-")
      .append(pad_zero(2, (uint16_t)timeinfo.tm_mday))
      .append(" ")
      .append(pad_zero(2, (uint16_t)timeinfo.tm_hour))
      .append(":")
      .append(pad_zero(2, (uint16_t)timeinfo.tm_min))
      .append(":")
      .append(pad_zero(2, (uint16_t)timeinfo.tm_sec))
      .append(",");

  dataMessage.append(currentMonitor.validReadings ? "1" : "0")
      .append(",")
      .append(float_to_string(currentMonitor.modbus.voltage))
      .append(",")
      .append(float_to_string(currentMonitor.modbus.current))
      .append(",")
      .append(std::to_string(currentMonitor.modbus.milliamphour_in))
      .append(",")
      .append(std::to_string(currentMonitor.modbus.milliamphour_out))
      .append(",")
      .append(std::to_string(currentMonitor.modbus.daily_milliamphour_in))
      .append(",")
      .append(std::to_string(currentMonitor.modbus.daily_milliamphour_out))
      .append(",")
      .append(float_to_string(currentMonitor.modbus.power))
      .append(",")
      .append(std::to_string(currentMonitor.modbus.temperature))
      .append(",")
      .append(currentMonitor.RelayState ? "1" : "0")
      .append("\r\n");

  file2.write((const uint8_t *)dataMessage.c_str(), dataMessage.length());
  file2.flush();
  file2.close();

  ESP_LOGI(TAG, "Current monitor log file");
}

// Output a status log to the SD Card in CSV format
[[noreturn]] void sdcardlog_task(void *)
{
  for (;;)
  {
    // Wait X seconds
    for (size_t i = 0; i < mysettings.loggingFrequencySeconds; i++)
    {
      // Delay 1 second
      vTaskDelay(pdMS_TO_TICKS(1000));
    }

    if (_sd_card_installed &&
        !_avrsettings.programmingModeEnabled &&
        mysettings.loggingEnabled &&
        _controller_state == ControllerState::Running)
    {

      struct tm timeinfo;
      // getLocalTime has delay() functions in it :-(
      if (getLocalTime(&timeinfo, 1))
      {
        timeinfo.tm_year += 1900;
        // Month is 0 to 11 based!
        timeinfo.tm_mon++;

        std::string filename;
        filename.reserve(32);
        filename.append("/data_").append(std::to_string(timeinfo.tm_year)).append(pad_zero(2, (uint16_t)timeinfo.tm_mon)).append(pad_zero(2, (uint16_t)timeinfo.tm_mday)).append(".csv");

        // Prevent other devices using the VSPI bus
        if (hal.GetVSPIMutex())
        {
          log_cell_monitoring_data_to_sdcard(filename, timeinfo);

          // Now log the current monitor
          if (mysettings.currentMonitoringEnabled)
          {
            std::string cmon_filename;
            cmon_filename.reserve(32);
            cmon_filename.append("/modbus")
                .append(pad_zero(2, mysettings.currentMonitoringModBusAddress))
                .append("_")
                .append(std::to_string(timeinfo.tm_year))
                .append(pad_zero(2, (uint16_t)timeinfo.tm_mon))
                .append(pad_zero(2, (uint16_t)timeinfo.tm_mday))
                .append(".csv");

            log_current_data_to_sdcard(cmon_filename, timeinfo);

          } // end of logging for current monitor
        }

        // Must be the last thing...
        hal.ReleaseVSPIMutex();
      }
      else
      {
        ESP_LOGE(TAG, "Invalid datetime");
      }
    }
  } // end for loop
}

// Triggered when
[[noreturn]] void rule_state_change_task(void *)
{
  for (;;)
  {
    // Wait until this task is triggered https://www.freertos.org/ulTaskNotifyTake.html
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    mqtt3(&rules, previousRelayState);
  }
}

void sdcardlog_output(std::string filename, const tm timeinfo)
{
  if (!check_sdcard_freespace())
  {
    return;
  }

  File file;

  auto exists = SD.exists(filename.c_str());
  file = SD.open(filename.c_str(), exists ? FILE_APPEND : FILE_WRITE);
  if (!exists)
  {
    // Create the file
    ESP_LOGD(TAG, "Create log %s", filename.c_str());

    std::string header;
    header.reserve(128);

    header.append("DateTime,TCA6408,TCA9534,");

    for (uint8_t i = 0; i < RELAY_TOTAL; i++)
    {
      header.append("Output_").append(std::to_string(i));
      if (i < RELAY_TOTAL - 1)
      {
        header.append(",");
      }
    }
    header.append("\r\n");
    file.write((const uint8_t *)header.c_str(), header.length());
  }

  std::string dataMessage;
  dataMessage.reserve(128);

  dataMessage.append(pad_zero(4, (uint16_t)timeinfo.tm_year))
      .append("-")
      .append(pad_zero(2, (uint16_t)timeinfo.tm_mon))
      .append("-")
      .append(pad_zero(2, (uint16_t)timeinfo.tm_mday))
      .append(" ")
      .append(pad_zero(2, (uint16_t)timeinfo.tm_hour))
      .append(":")
      .append(pad_zero(2, (uint16_t)timeinfo.tm_min))
      .append(":")
      .append(pad_zero(2, (uint16_t)timeinfo.tm_sec))
      .append(",")
      .append(uint8_to_binary_string(hal.LastTCA6408Value()))
      .append(",")
      .append(uint8_to_binary_string(hal.LastTCA9534APWRValue()))
      .append(",");

  for (uint8_t i = 0; i < RELAY_TOTAL; i++)
  {
    // This may output invalid data when controller is first powered up
    dataMessage.append(previousRelayState[i] == RelayState::RELAY_ON ? "Y" : "N");
    if (i < RELAY_TOTAL - 1)
    {
      dataMessage.append(",");
    }
  }
  dataMessage.append("\r\n");
  file.write((const uint8_t *)dataMessage.c_str(), dataMessage.length());
  file.flush();
  file.close();

  ESP_LOGI(TAG, "Output State logging");
}

// Writes a status log of the OUTPUT STATUES to the SD Card in CSV format
[[noreturn]] void sdcardlog_outputs_task(void *)
{
  for (;;)
  {
    // Wait until this task is triggered https://www.freertos.org/ulTaskNotifyTake.html
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if (_sd_card_installed &&
        !_avrsettings.programmingModeEnabled &&
        mysettings.loggingEnabled &&
        _controller_state == ControllerState::Running)
    {
      ESP_LOGD(TAG, "sdcardlog_outputs_task");

      struct tm timeinfo;
      // getLocalTime has delay() functions in it :-(
      if (getLocalTime(&timeinfo, 1))
      {
        timeinfo.tm_year += 1900;
        // Month is 0 to 11 based!
        timeinfo.tm_mon++;

        std::string filename;
        filename.reserve(32);
        filename.append("/output_status_").append(std::to_string(timeinfo.tm_year)).append(pad_zero(2, (uint16_t)timeinfo.tm_mon)).append(pad_zero(2, (uint16_t)timeinfo.tm_mday)).append(".csv");

        // Prevent other devices using the VSPI bus
        if (hal.GetVSPIMutex())
        {
          sdcardlog_output(filename, timeinfo);
          // Must be the last thing...
          hal.ReleaseVSPIMutex();
        }
      }
      else
      {
        ESP_LOGE(TAG, "Invalid datetime");
      }
    } // end if
  }   // end for loop
}

// Switch the LED off (triggered by timer on 100ms delay)
void ledoff(const TimerHandle_t)
{
  LED(RGBLED::OFF);
}

void ProcessTCA6408Input_States(uint8_t v)
{
  // P0=A
  InputState[0] = (v & B00000001) == 0 ? enumInputState::INPUT_LOW : enumInputState::INPUT_HIGH;
  // P1=B
  InputState[1] = (v & B00000010) == 0 ? enumInputState::INPUT_LOW : enumInputState::INPUT_HIGH;
  // P2=C
  InputState[2] = (v & B00000100) == 0 ? enumInputState::INPUT_LOW : enumInputState::INPUT_HIGH;
  // P3=D
  InputState[3] = (v & B00001000) == 0 ? enumInputState::INPUT_LOW : enumInputState::INPUT_HIGH;
}

void ProcessTCA9534Input_States(uint8_t v)
{
  // P4= J13 PIN 1 = WAKE UP TFT FOR DISPLAYS WITHOUT TOUCH
  // Also SW1 on V4.4 boards
  InputState[4] = (v & B00010000) == 0 ? enumInputState::INPUT_LOW : enumInputState::INPUT_HIGH;
  // P6 = spare I/O (on PCB pin)
  // Also SW2 on V4.4 boards
  InputState[5] = (v & B01000000) == 0 ? enumInputState::INPUT_LOW : enumInputState::INPUT_HIGH;
  // P7 = Emergency Stop
  InputState[6] = (v & B10000000) == 0 ? enumInputState::INPUT_LOW : enumInputState::INPUT_HIGH;

  // Emergency Stop (J1) has triggered
  if (InputState[6] == enumInputState::INPUT_LOW)
  {
    emergencyStop = true;
  }

  if (InputState[4] == enumInputState::INPUT_LOW)
  {
    // LEFT BUTTON PUSH
    // Wake screen on pin going low (SW1 on V4.4 boards)
    if (_screen_awake)
    {
      PageBackward();
    }
    else
    {
      wake_up_tft(false);
    }
  }
  if (InputState[5] == enumInputState::INPUT_LOW)
  {
    // RIGHT BUTTON PUSH
    // Wake screen on pin going low (SW1 on V4.4 boards)
    if (_screen_awake)
    {
      PageForward();
    }
    else
    {
      wake_up_tft(false);
    }
  }
}

// Handles interrupt requests raised by ESP32 ISR routines
[[noreturn]] void interrupt_task(void *)
{
  uint32_t ulInterruptStatus;

  for (;;)
  {
    // ULONG_MAX
    xTaskNotifyWait(0, ULONG_MAX, &ulInterruptStatus, portMAX_DELAY);

    if ((ulInterruptStatus & ISRTYPE::TCA6416A) != 0x00)
    {
      ESP_LOGD(TAG, "tca6416a_isr");

      // Emulate both 9534 and 6408 being installed by mimicking the
      // registers and processing the status as needed.
      hal.ReadTCA6416InputRegisters();

      // Read ports
      // The 9534 deals with internal LED outputs and spare IO on J10
      ProcessTCA9534Input_States(hal.LastTCA9534APWRValue());
      // Read ports A/B/C/D inputs (on TCA6408)
      ProcessTCA6408Input_States(hal.LastTCA6408Value());
    }

    if ((ulInterruptStatus & ISRTYPE::TCA6408A) != 0x00)
    {
      ESP_LOGD(TAG, "tca6408_isr");
      // Read ports A/B/C/D inputs (on TCA6408)
      ProcessTCA6408Input_States(hal.ReadTCA6408InputRegisters());
    }

    if ((ulInterruptStatus & ISRTYPE::TCA9534) != 0x00)
    {
      ESP_LOGD(TAG, "tca9534_isr");
      // Read ports
      // The 9534 deals with internal LED outputs and spare IO on J10
      ProcessTCA9534Input_States(hal.ReadTCA9534InputRegisters());
    }
  }
}

void IRAM_ATTR InterruptTrigger(ISRTYPE isrvalue)
{
  if (interrupt_task_handle != nullptr)
  {
    auto wokenTask = pdFALSE;
    xTaskNotifyFromISR(interrupt_task_handle, isrvalue, eNotifyAction::eSetBits, &wokenTask);
    if (wokenTask == pdTRUE)
    {
      portYIELD_FROM_ISR(wokenTask);
    }
  }
}

// Triggered when TCA6416A INT pin goes LOW
// Found on V4.4 controller PCB's onwards
void IRAM_ATTR TCA6416AInterrupt()
{
  InterruptTrigger(ISRTYPE::TCA6416A);
}
// Triggered when TCA6408 INT pin goes LOW
void IRAM_ATTR TCA6408Interrupt()
{
  InterruptTrigger(ISRTYPE::TCA6408A);
}
// Triggered when TCA9534A INT pin goes LOW
void IRAM_ATTR TCA9534AInterrupt()
{
  InterruptTrigger(ISRTYPE::TCA9534);
}

const char *packetType(uint8_t cmd)
{
  switch (cmd)
  {
  case COMMAND::ResetBadPacketCounter:
    return "ResetC";
  case COMMAND::ReadVoltageAndStatus:
    return "RdVolt";
  case COMMAND::Identify:
    return "Ident";
  case COMMAND::ReadTemperature:
    return "RdTemp";
  case COMMAND::ReadBadPacketCounter:
    return "RdBadPkC";
  case COMMAND::ReadSettings:
    return "RdSettin";
  case COMMAND::WriteSettings:
    return "WriteSet";
  case COMMAND::ReadBalancePowerPWM:
    return "RdBalanc";
  case COMMAND::Timing:
    return "Timing";
  case COMMAND::ReadBalanceCurrentCounter:
    return "Current";
  case COMMAND::ReadPacketReceivedCounter:
    return "PktRvd";
  case COMMAND::ResetBalanceCurrentCounter:
    return "ResBal";
  case COMMAND::ReadAdditionalSettings:
    return "RdAddt";
  case COMMAND::WriteAdditionalSettings:
    return "WrAddt";
  default:
    return " ??????   ";
  }
}

void dumpPacketToDebug(char indicator, const PacketStruct *buffer)
{
  // Filter on some commands
  // if ((buffer->command & 0x0F) != COMMAND::Timing)    return;

  ESP_LOGD(TAG, "%c %02X-%02X H:%02X C:%02X SEQ:%04X CRC:%04X %s",
           indicator,
           buffer->start_address,
           buffer->end_address,
           buffer->hops,
           buffer->command,
           buffer->sequence,
           buffer->crc,
           packetType(buffer->command & 0x0F));

  // ESP_LOG_BUFFER_HEX("packet", &(buffer->moduledata[0]), sizeof(buffer->moduledata), ESP_LOG_DEBUG);
}

const char *ControllerStateString(ControllerState value)
{
  switch (value)
  {
  case ControllerState::PowerUp:
    return "PowerUp";
  case ControllerState::NoWifiConfiguration:
    return "NoWIFI";
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
  if (_controller_state != newState)
  {
    ESP_LOGI(TAG, "** Controller changed state from %s to %s **", ControllerStateString(_controller_state), ControllerStateString(newState));

    _controller_state = newState;

    switch (_controller_state)
    {
    case ControllerState::PowerUp:
      // Purple during start up, don't use the LED as thats not setup at this state
      hal.Led(RGBLED::Purple);
      break;
    case ControllerState::Stabilizing:
      LED(RGBLED::Yellow);
      break;
    case ControllerState::NoWifiConfiguration:
      LED(RGBLED::White);
      break;
    case ControllerState::Running:
      LED(RGBLED::Green);
      break;
    case ControllerState::Unknown:
      // Do nothing
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
    return ((uint16_t)timeinfo.tm_hour * 60) + (uint16_t)timeinfo.tm_min;
  }
}

[[noreturn]] void replyqueue_task(void *)
{
  for (;;)
  {
    if (reply_q_handle != nullptr)
    {
      PacketStruct ps;
      if (xQueueReceive(reply_q_handle, &ps, portMAX_DELAY) == pdPASS)
      {
#if defined(PACKET_LOGGING_RECEIVE)
// Process decoded incoming packet
// dumpPacketToDebug('R', &ps);
#endif

        if (!receiveProc.ProcessReply(&ps))
        {
          // Error blue
          LED(RGBLED::Blue);

          ESP_LOGE(TAG, "Packet Failed");

          // SERIAL_DEBUG.print(F("*FAIL*"));
          // dumpPacketToDebug('F', &ps);
        }
      }
    }
  }
}

void onPacketReceived()
{
  PacketStruct ps;
  memcpy(&ps, SerialPacketReceiveBuffer, sizeof(PacketStruct));

  if ((ps.command & 0x0F) == COMMAND::Timing)
  {
    // Timestamp at the earliest possible moment
    auto t = millis();
    ps.moduledata[2] = (t & 0xFFFF0000) >> 16;
    ps.moduledata[3] = t & (uint32_t)0x0000FFFF;
    // Ensure CRC is correct
    ps.crc = CRC16::CalculateArray((uint8_t *)&ps, sizeof(PacketStruct) - 2);
  }

  if (xQueueSendToBack(reply_q_handle, &ps, (TickType_t)100) != pdPASS)
  {
    ESP_LOGE(TAG, "Reply Q full");
  }

  // ESP_LOGI(TAG,"Reply Q length %i",replyQueue.getCount());
}

[[noreturn]] void transmit_task(void *)
{
  for (;;)
  {
    PacketStruct transmitBuffer;
    if (request_q_handle != nullptr)
    {
      if (xQueueReceive(request_q_handle, &transmitBuffer, portMAX_DELAY) == pdPASS)
      {

        sequence++;
        transmitBuffer.sequence = sequence;

        if (transmitBuffer.command == COMMAND::Timing)
        {
          // Timestamp at the last possible moment
          auto t = millis();
          transmitBuffer.moduledata[0] = (t & 0xFFFF0000) >> 16;
          transmitBuffer.moduledata[1] = t & (uint32_t)0x0000FFFF;
        }

        transmitBuffer.crc = CRC16::CalculateArray((uint8_t *)&transmitBuffer, sizeof(PacketStruct) - 2);
        myPacketSerial.sendBuffer((byte *)&transmitBuffer);

        // Output the packet we just transmitted to debug console
        // #if defined(PACKET_LOGGING_SEND)
        //      dumpPacketToDebug('S', &transmitBuffer);
        // #endif
      }

      // Delay based on comms speed, ensure the first module has time to process and clear the request
      // before sending another packet
      uint16_t delay_ms = 900;

      if (mysettings.baudRate == 10000)
      {
        delay_ms = 350;
      }
      else if (mysettings.baudRate == 9600)
      {
        delay_ms = 450;
      }
      else if (mysettings.baudRate == 5000)
      {
        delay_ms = 700;
      }

      // Delay whilst cell module processes request
      vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
  }
}

// Runs the rules and populates rule_outcome array with true/false for each rule
// Rules based on module parameters/readings like voltage and temperature
// are only processed once every module has returned at least 1 reading/communication
void ProcessRules()
{
  rules.ClearValues();
  rules.ClearWarnings();
  rules.ClearErrors();

  rules.setRuleStatus(Rule::BMSError, false);

  auto totalConfiguredModules = TotalNumberOfCells();
  if (totalConfiguredModules > maximum_controller_cell_modules)
  {
    // System is configured with more than maximum modules - abort!
    rules.SetError(InternalErrorCode::TooManyModules);
  }

  if (receiveProc.totalModulesFound > 0 && receiveProc.totalModulesFound != totalConfiguredModules)
  {
    // Found more or less modules than configured for
    rules.SetError(InternalErrorCode::ModuleCountMismatch);
  }

  // Communications error...
  if (receiveProc.HasCommsTimedOut())
  {
    rules.SetError(InternalErrorCode::CommunicationsError);
    rules.setRuleStatus(Rule::BMSError, true);
  }

  if (rules.ruleOutcome(Rule::EmergencyStop))
  {
    // Lowest 3 bits are RGB led GREEN/RED/BLUE
    rules.SetError(InternalErrorCode::ErrorEmergencyStop);
  }

  // Raise error is the current shunt stops responding for over 45 seconds
  if (mysettings.currentMonitoringEnabled)
  {
    auto secondsSinceLastMessage = (int64_t)((esp_timer_get_time() - currentMonitor.timestamp) / 1000000);
    if (secondsSinceLastMessage > 45)
    {
      rules.SetError(InternalErrorCode::CommunicationsError);
      rules.setRuleStatus(Rule::BMSError, true);
    }
  }

  rules.highestBankRange = 0;
  rules.numberOfBalancingModules = 0;
  uint8_t cellid = 0;
  for (int8_t bank = 0; bank < mysettings.totalNumberOfBanks; bank++)
  {
    for (int8_t i = 0; i < mysettings.totalNumberOfSeriesModules; i++)
    {
      rules.ProcessCell(bank, cellid, &cmi[cellid], mysettings.cellmaxmv);

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

        if (cmi[cellid].inBypass)
        {
          rules.numberOfBalancingModules++;
        }

        if (cmi[0].settingsCached && cmi[cellid].CodeVersionNumber != cmi[0].CodeVersionNumber)
        {
          // Do all the modules have the same version of code as module zero?
          rules.SetWarning(InternalWarningCode::ModuleInconsistantCodeVersion);
        }

        if (cmi[0].settingsCached && cmi[cellid].BoardVersionNumber != cmi[0].BoardVersionNumber)
        {
          // Do all the modules have the same hardware revision?
          rules.SetWarning(InternalWarningCode::ModuleInconsistantBoardRevision);
        }
      }

      cellid++;
    }
    rules.ProcessBank(bank);
  }

  rules.CalculateChargingMode(&mysettings, &currentMonitor);
  // Need to call these even if Dynamic is switched off, as it seeds the internal variables with the correct values
  rules.CalculateDynamicChargeVoltage(&mysettings, cmi);
  rules.CalculateDynamicChargeCurrent(&mysettings);

  if (mysettings.loggingEnabled && !_sd_card_installed && !_avrsettings.programmingModeEnabled)
  {
    rules.SetWarning(InternalWarningCode::LoggingEnabledNoSDCard);
  }

  if (_avrsettings.programmingModeEnabled)
  {
    rules.SetWarning(InternalWarningCode::AVRProgrammingMode);
  }

  if (rules.invalidModuleCount > 0)
  {
    // Some modules are not yet valid
    rules.SetError(InternalErrorCode::WaitingForModulesToReply);
  }

  if (_controller_state == ControllerState::Running && rules.zeroVoltageModuleCount > 0)
  {
    rules.SetError(InternalErrorCode::ZeroVoltModule);
    rules.setRuleStatus(Rule::BMSError, true);
  }

  rules.RunRules(
      mysettings.rulevalue,
      mysettings.rulehysteresis,
      emergencyStop,
      minutesSinceMidnight(),
      &currentMonitor);

  if (rules.moduleHasExternalTempSensor == false)
  {
    // NoExternalTempSensor
    rules.SetWarning(InternalWarningCode::NoExternalTempSensor);
  }

  if (mysettings.canbusprotocol != CanBusProtocolEmulation::CANBUS_DISABLED)
  {
    if (!rules.IsChargeAllowed(&mysettings))
    {
      // Charge prevented
      rules.SetWarning(InternalWarningCode::ChargePrevented);
    }
    if (!rules.IsDischargeAllowed(&mysettings))
    {
      // Discharge prevented
      rules.SetWarning(InternalWarningCode::DischargePrevented);
    }
  }

  if (_controller_state == ControllerState::Stabilizing)
  {
    // Check for zero volt modules - not a problem whilst we are in stabilizing start up mode
    if (rules.zeroVoltageModuleCount == 0 && rules.invalidModuleCount == 0)
    {
      // Every module has been read and they all returned a voltage move to running state
      SetControllerState(ControllerState::Running);
    }
  }

  if (rules.ruleOutcome(Rule::EmergencyStop))
  {
    // Lowest 3 bits are RGB led GREEN/RED/BLUE
    LED(RGBLED::Red);
  }

  if (rules.numberOfActiveErrors > 0 || rules.WarningCodes.at(InternalWarningCode::AVRProgrammingMode) != InternalWarningCode::NoWarning)
  {
    // We have active errors, or AVR programming mode is enabled
    ESP_LOGI(TAG, "Active errors=%u", rules.numberOfActiveErrors);

    // Wake up the screen, this will also trigger it to update the display
    wake_up_tft(true);
  }
}

void pulse_relay_off(const TimerHandle_t)
{
  for (int8_t y = 0; y < RELAY_TOTAL; y++)
  {
    if (previousRelayPulse[y])
    {
      // We now need to rapidly turn off the relay after a fixed period of time (pulse mode)
      // However we leave the relay and previousRelayState looking like the relay has triggered (it has!)
      // to prevent multiple pulses being sent on each rule refresh
      hal.SetOutputState(y, RelayState::RELAY_OFF);

      previousRelayPulse[y] = false;
    }
  }

  // Fire task to record state of outputs to SD Card
  xTaskNotify(sdcardlog_outputs_task_handle, 0x00, eNotifyAction::eNoAction);
}

[[noreturn]] void rules_task(void *)
{
  for (;;)
  {
    // 3 seconds
    vTaskDelay(pdMS_TO_TICKS(3000));

    // Run the rules
    ProcessRules();

    RelayState relay[RELAY_TOTAL];

    // Set defaults based on configuration
    for (int8_t y = 0; y < RELAY_TOTAL; y++)
    {
      relay[y] = mysettings.rulerelaydefault[y] == RELAY_ON ? RELAY_ON : RELAY_OFF;
    }

    // Test the rules (in reverse order)
    for (int8_t n = RELAY_RULES - 1; n >= 0; n--)
    {
      if (rules.ruleOutcome((Rule)n) == true)
      {
        for (int8_t y = 0; y < RELAY_TOTAL; y++)
        {
          // Dont change relay if its set to ignore/X
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
    bool firePulse = false;
    for (int8_t n = 0; n < RELAY_TOTAL; n++)
    {
      if (previousRelayState[n] != relay[n])
      {
        ESP_LOGI(TAG, "Set relay %i=%i", n, relay[n] == RelayState::RELAY_ON ? 1 : 0);
        changes++;

        // This would be better if we worked out the bit pattern first and then just submitted that as a single i2c read/write transaction
        hal.SetOutputState(n, relay[n]);

        // Record the previous state of the relay, to use on the next loop to prevent chatter
        previousRelayState[n] = relay[n];

        if (mysettings.relaytype[n] == RELAY_PULSE)
        {
          previousRelayPulse[n] = true;
          firePulse = true;
          ESP_LOGI(TAG, "Relay %i PULSED", n);
        }
      }
    }

    if (firePulse)
    {
      // Fire timer to switch off relay in a few ms
      if (xTimerStart(pulse_relay_off_timer, 10) != pdPASS)
      {
        ESP_LOGE(TAG, "Pulse timer start error");
      }
    }

    if (changes)
    {
      // Fire task to record state of outputs to SD Card
      xTaskNotify(sdcardlog_outputs_task_handle, 0x00, eNotifyAction::eNoAction);
    }

    if (changes || rules.anyRuleTriggered())
    {
      // A rule is TRUE or relay state has changed, so MQTT report it...
      xTaskNotify(rule_state_change_task_handle, 0x00, eNotifyAction::eNoAction);
    }
  }
}

// This task periodically adds requests to the queue
// to schedule reading data from the cell modules
// The actual serial comms is handled by the transmit task
[[noreturn]] void enqueue_task(void *)
{
  for (;;)
  {
    vTaskDelay(pdMS_TO_TICKS(mysettings.interpacketgap));

    LED(RGBLED::Green);

    // Fire timer to switch off LED in a few ms
    if (xTimerStart(led_off_timer, 5) != pdPASS)
    {
      ESP_LOGE(TAG, "Timer start error");
    }

    uint16_t i = 0;
    auto max = TotalNumberOfCells();
    uint8_t startmodule = 0;

    while (i < max)
    {
      uint8_t endmodule = (startmodule + maximum_cell_modules_per_packet) - 1;

      // Limit to number of modules we have configured
      if (endmodule > max)
      {
        endmodule = max - 1;
      }

      // Request voltage, but if queue is full, sleep and try again (other threads will reduce the queue)
      prg.sendCellVoltageRequest(startmodule, endmodule);
      // Same for temperature
      prg.sendCellTemperatureRequest(startmodule, endmodule);

      // If any module is in bypass then request PWM reading for whole bank
      for (uint8_t m = startmodule; m <= endmodule; m++)
      {
        if (cmi[m].inBypass)
        {
          prg.sendReadBalancePowerRequest(startmodule, endmodule);
          // We only need 1 reading for whole bank
          break;
        }
      }

      // Move to the next bank
      startmodule = endmodule + 1;
      i += maximum_cell_modules_per_packet;
    }
  }
}

static int wifi_ap_connect_retry_num = 0;

void formatCurrentDateTime(char *buf, size_t buf_size)
{
  time_t now;
  time(&now);
  struct tm timeinfo;
  localtime_r(&now, &timeinfo);
  strftime(buf, buf_size, "%c", &timeinfo);
}

static void setTimeZone(long offset, unsigned int daylight)
{
  char cst[17] = {0};
  char cdt[17] = "DST";
  char tz[33] = {0};

  if (offset % 3600)
  {
    sprintf(cst, "UTC%ld:%02ld:%02ld", offset / 3600, abs((offset % 3600) / 60), abs(offset % 60));
  }
  else
  {
    sprintf(cst, "UTC%ld", offset / 3600);
  }
  if (daylight != 3600)
  {
    long tz_dst = offset - daylight;
    if (tz_dst % 3600)
    {
      sprintf(cdt, "DST%ld:%02lu:%02lu", tz_dst / 3600, abs((tz_dst % 3600) / 60), abs(tz_dst % 60));
    }
    else
    {
      sprintf(cdt, "DST%ld", tz_dst / 3600);
    }
  }
  sprintf(tz, "%s%s", cst, cdt);
  setenv("TZ", tz, 1);
  ESP_LOGI(TAG, "Timezone=%s", tz);
  tzset();

  char strftime_buf[64];
  formatCurrentDateTime(strftime_buf, sizeof(strftime_buf));
  ESP_LOGI(TAG, "The current date/time is: %s", strftime_buf);
}

void configureSNTP(long gmtOffset_sec, int daylightOffset_sec, const char *server1)
{
  ESP_LOGI(TAG, "Request time from %s", server1);

  if (sntp_enabled())
  {
    sntp_stop();
  }
  sntp_setoperatingmode(SNTP_OPMODE_POLL);
  sntp_setservername(0, server1);
  // sntp_setservername(1, (char*)server2);
  // sntp_setservername(2, (char*)server3);
  sntp_init();

  // TODO: Fix this with native IDF library
  setTimeZone(-gmtOffset_sec, daylightOffset_sec);
  // setenv("TZ", tz, 1);
  // tzset();
}

static void stopMDNS()
{
  mdns_free();
}

static void startMDNS()
{

  // initialize mDNS service
  esp_err_t err = mdns_init();
  if (err != ESP_OK)
  {
    ESP_LOGE(TAG, "MDNS Init failed: %d", err);
  }
  else
  {
    mdns_hostname_set(hostname.c_str());
    mdns_instance_name_set("diybms");
    mdns_service_add(nullptr, "_http", "_tcp", 80, nullptr, 0);
  }
}

void ShutdownAllNetworkServices()
{
  // Shut down all TCP/IP reliant services
  if (server_running)
  {
    stop_webserver(_myserver);
    server_running = false;
    _myserver = nullptr;
  }
  stopMqtt();
  stopMDNS();
}

/// @brief Count of events of RSSI low
uint16_t wifi_count_rssi_low=0;
uint16_t wifi_count_sta_start=0;
/// @brief Count of events for WIFI connect
uint16_t wifi_count_sta_connected=0;
/// @brief Count of events for WIFI disconnect
uint16_t wifi_count_sta_disconnected=0;
uint16_t wifi_count_sta_lost_ip=0;
uint16_t wifi_count_sta_got_ip=0;

/// @brief WIFI Event Handler
/// @param
/// @param event_base
/// @param event_id
/// @param event_data
static void event_handler(void *, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
  // ESP_LOGD(TAG, "WIFI: event=%i, id=%i", event_base, event_id);

  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_BSS_RSSI_LOW)
  {
    ESP_LOGW(TAG, "WiFi signal strength low");
    wifi_count_rssi_low++;
  }
  else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
  {
    wifi_count_sta_start++;
    ESP_LOGI(TAG, "WIFI_EVENT_STA_START");
    wifi_isconnected = false;
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_connect());
  }
  else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED)
  {
    // We have joined the access point - now waiting for IP address IP_EVENT_STA_GOT_IP
    wifi_ap_connect_retry_num = 0;
    wifi_count_sta_connected++;

    wifi_ap_record_t ap;
    esp_wifi_sta_get_ap_info(&ap);

    ESP_LOGI(TAG, "WIFI_EVENT_STA_CONNECTED channel=%u, rssi=%i", ap.primary, ap.rssi);
  }
  else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
  {
    ESP_LOGI(TAG, "WIFI_EVENT_STA_DISCONNECTED");
    wifi_ap_connect_retry_num++;
    wifi_count_sta_disconnected++;

    if (wifi_isconnected)
    {
      ShutdownAllNetworkServices();
      wifi_isconnected = false;
    }

    if (wifi_ap_connect_retry_num < 25)
    {
      ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_connect());
      ESP_LOGI(TAG, "WIFI connect quick retry %i", wifi_ap_connect_retry_num);
    }
    else
    {
      ESP_LOGE(TAG, "Connect to WIFI AP failed, tried %i times", wifi_ap_connect_retry_num);
    }
  }
  else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_LOST_IP)
  {
    wifi_isconnected = false;
    ESP_LOGI(TAG, "IP_EVENT_STA_LOST_IP");
    wifi_count_sta_lost_ip++;

    ShutdownAllNetworkServices();
    wake_up_tft(true);
  }
  else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
  {
    wifi_count_sta_got_ip++;

    auto event = (ip_event_got_ip_t *)event_data;

    if (event->ip_changed)
    {
      ESP_LOGI(TAG, "IP ADDRESS HAS CHANGED");
      ShutdownAllNetworkServices();
    }

    // Start up all the services after TCP/IP is established
    configureSNTP(mysettings.timeZone * 3600 + mysettings.minutesTimeZone * 60, mysettings.daylight ? 3600 : 0, mysettings.ntpServer);

    if (!server_running)
    {
      // Start web server
      StartServer();
      server_running = true;
    }

    // This only exists in the loop()
    // connectToMqtt();
    startMDNS();

    ip_string = ip4_to_string(event->ip_info.ip.addr);

    ESP_LOGI(TAG, "You can access DIYBMS interface at http://%s.local or http://%s", hostname.c_str(), ip_string.c_str());

    wifi_isconnected = true;
    wake_up_tft(true);
  }
}

bool LoadWiFiConfig()
{
  auto reply = LoadWIFI(&_wificonfig);

  ESP_LOGI(TAG, "Wifi Config: SSID:%s, Manual Config:%u", _wificonfig.wifi_ssid, _wificonfig.manualConfig);
  ESP_LOGI(TAG, "Manual IP:%s, Netmask:%s, GW:%s, DNS1:%s, DNS2:%s",
           ip4_to_string(_wificonfig.wifi_ip).c_str(),
           ip4_to_string(_wificonfig.wifi_netmask).c_str(),
           ip4_to_string(_wificonfig.wifi_gateway).c_str(),
           ip4_to_string(_wificonfig.wifi_dns1).c_str(),
           ip4_to_string(_wificonfig.wifi_dns2).c_str());

  return reply;
}

bool SaveWIFIJson(const wifi_eeprom_settings *setting)
{
  if (!_sd_card_installed)
  {
    return false;
  }

  if (_avrsettings.programmingModeEnabled)
  {
    return false;
  }

  StaticJsonDocument<512> doc;

  JsonObject wifi = doc.createNestedObject("wifi");
  wifi["ssid"] = setting->wifi_ssid;
  wifi["password"] = setting->wifi_passphrase;

  // Manual IP settings
  wifi["ip"] = ip4_to_string(setting->wifi_ip);
  wifi["gateway"] = ip4_to_string(setting->wifi_gateway);
  wifi["netmask"] = ip4_to_string(setting->wifi_netmask);
  wifi["dns1"] = ip4_to_string(setting->wifi_dns1);
  wifi["dns2"] = ip4_to_string(setting->wifi_dns2);
  wifi["manualconfig"] = setting->manualConfig;

  if (hal.GetVSPIMutex())
  {
    ESP_LOGI(TAG, "Creating folder");
    SD.mkdir("/diybms");

    // Get the file
    ESP_LOGI(TAG, "Write SD file %s", wificonfigfilename.c_str());

    if (SD.exists(wificonfigfilename.c_str()))
    {
      ESP_LOGI(TAG, "Delete existing %s", wificonfigfilename.c_str());
      SD.remove(wificonfigfilename.c_str());
    }

    File file = SD.open(wificonfigfilename.c_str(), "w");
    serializeJson(doc, file);
    file.close();

    hal.ReleaseVSPIMutex();
    return true;
  }

  return false;
}

void BuildHostname()
{
  uint32_t chipId = 0;
  for (int i = 0; i < 17; i = i + 8)
  {
    chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
  }
  // DIYBMS-00000000
  char buffer[10];
  snprintf(buffer, sizeof(buffer), "%08X", chipId);

  hostname.clear();
  hostname.append("DIYBMS-").append(buffer);
}

void wifi_init_sta(void)
{
  // Don't do it if already connected....
  if (wifi_isconnected)
    return;

  ESP_LOGD(TAG, "starting wifi_init_sta");

  // Create ESP IDF default event loop to service WIFI events
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  ESP_ERROR_CHECK(esp_netif_init());

  esp_netif_t *netif = esp_netif_create_default_wifi_sta();
  assert(netif);

  if (_wificonfig.manualConfig == 1)
  {
    // Use static ipaddress
    ESP_ERROR_CHECK(esp_netif_dhcpc_stop(netif));

    esp_netif_ip_info_t ip_info;
    ip4_addr_set_u32(&ip_info.ip, _wificonfig.wifi_ip);
    ip4_addr_set_u32(&ip_info.gw, _wificonfig.wifi_gateway);
    ip4_addr_set_u32(&ip_info.netmask, _wificonfig.wifi_netmask);
    ESP_ERROR_CHECK(esp_netif_set_ip_info(netif, &ip_info));

    if (_wificonfig.wifi_dns1 != 0)
    {
      esp_netif_dns_info_t dns1;
      ip4_addr_set_u32(&dns1.ip.u_addr.ip4, _wificonfig.wifi_dns1);
      ESP_ERROR_CHECK(esp_netif_set_dns_info(netif, esp_netif_dns_type_t::ESP_NETIF_DNS_MAIN, &dns1));
    }

    if (_wificonfig.wifi_dns2 != 0)
    {
      esp_netif_dns_info_t dns2;
      ip4_addr_set_u32(&dns2.ip.u_addr.ip4, _wificonfig.wifi_dns2);
      ESP_ERROR_CHECK(esp_netif_set_dns_info(netif, esp_netif_dns_type_t::ESP_NETIF_DNS_BACKUP, &dns2));
    }
  }

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

  cfg.static_tx_buf_num = 0;
  cfg.dynamic_tx_buf_num = 32;
  cfg.tx_buf_type = 1;
  cfg.cache_tx_buf_num = 1;
  cfg.static_rx_buf_num = 6;
  cfg.dynamic_rx_buf_num = 32;

  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                      ESP_EVENT_ANY_ID,
                                                      &event_handler,
                                                      nullptr,
                                                      nullptr));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                      IP_EVENT_STA_GOT_IP,
                                                      &event_handler,
                                                      nullptr,
                                                      nullptr));

  wifi_config_t wifi_config;
  memset(&wifi_config, 0, sizeof(wifi_config));
  wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
  strncpy((char *)wifi_config.sta.ssid, _wificonfig.wifi_ssid, sizeof(wifi_config.sta.ssid));
  strncpy((char *)wifi_config.sta.password, _wificonfig.wifi_passphrase, sizeof(wifi_config.sta.password));

  ESP_LOGI(TAG, "WIFI SSID: %s", _wificonfig.wifi_ssid);

  // Avoid issues with GPIO39 interrupt firing all the time - disable power saving
  ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  // Generates WIFI_EVENT_STA_BSS_RSSI_LOW events when RSS goes low
  ESP_ERROR_CHECK(esp_wifi_set_rssi_threshold(-80));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_ERROR_CHECK_WITHOUT_ABORT(tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_STA, hostname.c_str()));
  ESP_ERROR_CHECK_WITHOUT_ABORT(esp_netif_set_hostname(netif, hostname.c_str()));

  ESP_LOGI(TAG, "Hostname: %s", hostname.c_str());

  ESP_LOGD(TAG, "wifi_init_sta finished");
}

uint16_t calculateCRC(const uint8_t *f, uint8_t bufferSize)
{
  uint16_t flag;
  uint16_t temp;
  temp = 0xFFFF;
  for (unsigned char i = 0; i < bufferSize; i++)
  {
    temp = temp ^ f[i];
    for (unsigned char j = 1; j <= 8; j++)
    {
      flag = temp & 0x0001;
      temp >>= 1;
      if (flag)
        temp ^= 0xA001;
    }
  }

  return temp;
}

uint8_t SetMobusRegistersFromFloat(uint8_t *cmd, uint8_t ptr, float value)
{
  FloatUnionType fut;
  fut.value = value;
  // 4 bytes
  cmd[ptr] = (uint8_t)(fut.word[0] >> 8);
  ptr++;
  cmd[ptr] = (uint8_t)(fut.word[0] & 0xFF);
  ptr++;
  cmd[ptr] = (uint8_t)(fut.word[1] >> 8);
  ptr++;
  cmd[ptr] = (uint8_t)(fut.word[1] & 0xFF);
  ptr++;

  return ptr;
}

void PZEM017_SetShuntType(uint8_t modbusAddress, uint16_t shuntMaxCurrent)
{
  // Default 100A
  uint8_t shuntType;

  switch (shuntMaxCurrent)
  {
  case 50:
    shuntType = 1;
    break;
  case 200:
    shuntType = 2;
    break;
  case 300:
    shuntType = 3;
    break;
  default:
    // 100amp
    shuntType = 0;
    break;
  }

  uint8_t cmd[MAX_SEND_RS485_PACKET_LENGTH];
  memset(&cmd, 0, sizeof(cmd));

  cmd[0] = modbusAddress;
  // Function Code 6
  cmd[1] = 0x06;
  // Address of the shunt register (3)
  cmd[3] = 0x03;
  // Value
  cmd[5] = shuntType;

  ESP_LOGD(TAG, "Set PZEM017 max current %uA=%u", shuntMaxCurrent, shuntType);

  // Blocking call
  xQueueSend(rs485_transmit_q_handle, &cmd, portMAX_DELAY);

  // Zero all data
  memset(&currentMonitor, 0, sizeof(currentmonitoring_struct));
  currentMonitor.validReadings = false;
}

void PZEM017_SetDeviceAddress(uint8_t newAddress)
{
  // First we force the PZEM device to assume the selected MODBUS address using
  // the special "broadcast" address of 0xF8.  Technically the PZEM devices
  // support multiple devices on same RS485 bus, but DIYBMS doesn't....

  uint8_t cmd[MAX_SEND_RS485_PACKET_LENGTH];
  memset(&cmd, 0, sizeof(cmd));

  // The configuration address (only 1 PZEM device can be connected)
  cmd[0] = 0xf8;
  // Function Code 6
  cmd[1] = 0x06;
  // Register 2
  // cmd[2] = 0;
  cmd[3] = 2;
  // value
  // cmd[4] = 0;
  cmd[5] = newAddress;

  ESP_LOGD(TAG, "Sent PZEM_017 change address");

  // Zero all data
  memset(&currentMonitor, 0, sizeof(currentmonitoring_struct));
  currentMonitor.validReadings = false;
  xQueueSend(rs485_transmit_q_handle, &cmd, portMAX_DELAY);
}

void currentMon_ConfigureBasic(uint16_t shuntmv, uint16_t shuntmaxcur, uint16_t batterycapacity, float fullchargevolt, float tailcurrent, float chargeefficiency)
{
  auto chargeeff = (uint16_t)(chargeefficiency * 100.0F);

  uint8_t cmd[MAX_SEND_RS485_PACKET_LENGTH];
  memset(&cmd, 0, sizeof(cmd));

  //	Write Multiple Holding Registers
  uint8_t cmd2[] = {
      // The Slave Address
      mysettings.currentMonitoringModBusAddress,
      // The Function Code 16
      16,
      // Data Address of the first register (zero based so 18 = register 40019)
      0,
      18,
      // number of registers to write
      0,
      8,
      // number of data bytes to follow (2 registers x 2 bytes each = 4 bytes)
      16,
      // value to write to register 40019 |40019|shunt_max_current  (unsigned int16)
      (uint8_t)(shuntmaxcur >> 8),
      (uint8_t)(shuntmaxcur & 0xFF),
      // value to write to register 40020 |40020|shunt_millivolt  (unsigned int16)
      (uint8_t)(shuntmv >> 8),
      (uint8_t)(shuntmv & 0xFF),
      //|40021|Battery Capacity (ah)  (unsigned int16)
      (uint8_t)(batterycapacity >> 8),
      (uint8_t)(batterycapacity & 0xFF),
      //|40022|Fully charged voltage (4 byte double)
      0,
      0,
      0,
      0,
      //|40024|Tail current (Amps) (4 byte double)
      0,
      0,
      0,
      0,
      //|40026|Charge efficiency factor % (unsigned int16) (scale x100 eg. 10000 = 100.00%, 9561 = 95.61%)
      (uint8_t)(chargeeff >> 8),
      (uint8_t)(chargeeff & 0xFF),
  };

  auto ptr = SetMobusRegistersFromFloat(cmd2, 13, fullchargevolt);
  ptr = SetMobusRegistersFromFloat(cmd2, ptr, tailcurrent);

  memcpy(&cmd, &cmd2, sizeof(cmd2));
  xQueueSend(rs485_transmit_q_handle, &cmd, portMAX_DELAY);

  // Zero all data
  // memset(&currentMonitor, 0, sizeof(currentmonitoring_struct));
  currentMonitor.validReadings = false;
}

void currentMon_SetSOC(float newSOC)
{
  auto value = (uint16_t)(newSOC * 100);

  //	Write Multiple Holding Registers
  uint8_t cmd2[] = {
      // The Slave Address
      mysettings.currentMonitoringModBusAddress,
      // The Function Code 16
      16,
      // Data Address of the first register (zero based so 26 = register 40027)
      0,
      26,
      // number of registers to write (1)
      0,
      1,
      // number of data bytes to follow (1 register x 2 bytes each = 2 bytes)
      2,
      // value to write to register |40027|State of charge % (unsigned int16)
      // (scale x100 eg. 10000 = 100.00%, 8012 = 80.12%, 100 = 1.00%)
      (uint8_t)(value >> 8),
      (uint8_t)(value & 0xFF)};

  xQueueSend(rs485_transmit_q_handle, &cmd2, portMAX_DELAY);
}

void currentMon_ResetDailyAmpHourCounters()
{
  //	Write Multiple Holding Registers
  uint8_t cmd2[] = {
      // The Slave Address
      mysettings.currentMonitoringModBusAddress,
      // The Function Code 16
      16,
      // Data Address of the first register (zero based so 11 = register 40012)
      0,
      11,
      // number of registers to write
      0,
      4,
      // number of data bytes to follow (2 registers x 2 bytes each = 4 bytes)
      8,
      // value to write to register 40012/13 |40012| daily milliamphour_out (uint32_t)
      0,
      0,
      0,
      0,
      // value to write to register 40014/15 |40014| daily milliamphour_in (uint32_t)
      0,
      0,
      0,
      0};

  xQueueSend(rs485_transmit_q_handle, &cmd2, portMAX_DELAY);
}

bool CurrentMonitorSetSOC(float newSOC)
{
  if (mysettings.currentMonitoringEnabled == true)
  {
    ESP_LOGI(TAG, "Set SOC");
    if (mysettings.currentMonitoringDevice == CurrentMonitorDevice::DIYBMS_CURRENT_MON_MODBUS)
    {
      currentMon_SetSOC(newSOC);
      return true;
    }

    if (mysettings.currentMonitoringDevice == CurrentMonitorDevice::DIYBMS_CURRENT_MON_INTERNAL)
    {
      auto value = (uint16_t)(newSOC * 100);
      currentmon_internal.SetSOC(value);
      return true;
    }
  }

  return false;
}

bool CurrentMonitorResetDailyAmpHourCounters()
{
  if (mysettings.currentMonitoringEnabled == true)
  {
    ESP_LOGI(TAG, "Reset daily Ah counter");
    if (mysettings.currentMonitoringDevice == CurrentMonitorDevice::DIYBMS_CURRENT_MON_MODBUS)
    {
      currentMon_ResetDailyAmpHourCounters();
      return true;
    }
    if (mysettings.currentMonitoringDevice == CurrentMonitorDevice::DIYBMS_CURRENT_MON_INTERNAL)
    {
      currentmon_internal.ResetDailyAmpHourCounters();
      return true;
    }
  }
  return false;
}

void CurrentMonitorSetBasicSettings(uint16_t shuntmv, uint16_t shuntmaxcur, uint16_t batterycapacity, float fullchargevolt, float tailcurrent, float chargeefficiency)
{

  if (mysettings.currentMonitoringDevice == CurrentMonitorDevice::DIYBMS_CURRENT_MON_MODBUS)
  {
    currentMon_ConfigureBasic(shuntmv, shuntmaxcur, batterycapacity, fullchargevolt, tailcurrent, chargeefficiency);
  }

  if (mysettings.currentMonitoringDevice == CurrentMonitorDevice::DIYBMS_CURRENT_MON_INTERNAL)
  {
    mysettings.currentMonitoring_shuntmv = shuntmv;
    mysettings.currentMonitoring_shuntmaxcur = shuntmaxcur;
    mysettings.currentMonitoring_batterycapacity = batterycapacity;
    mysettings.currentMonitoring_fullchargevolt = (uint16_t)(100.0 * fullchargevolt);
    mysettings.currentMonitoring_tailcurrent = (uint16_t)(100.0 * tailcurrent);
    mysettings.currentMonitoring_chargeefficiency = (uint16_t)(100.0 * chargeefficiency);
    // Reset stored shuntcal to zero to allow automatic calibration
    // this is updated by the CurrentMonitorSetAdvancedSettings call
    mysettings.currentMonitoring_shuntcal = 0;
    ValidateConfiguration(&mysettings);
    SaveConfiguration(&mysettings);

    if (hal.GetVSPIMutex())
    {
      currentmon_internal.Configure(
          mysettings.currentMonitoring_shuntmv,
          mysettings.currentMonitoring_shuntmaxcur,
          mysettings.currentMonitoring_batterycapacity,
          mysettings.currentMonitoring_fullchargevolt,
          mysettings.currentMonitoring_tailcurrent,
          mysettings.currentMonitoring_chargeefficiency,
          mysettings.currentMonitoring_shuntcal,
          mysettings.currentMonitoring_temperaturelimit,
          mysettings.currentMonitoring_overvoltagelimit,
          mysettings.currentMonitoring_undervoltagelimit,
          mysettings.currentMonitoring_overcurrentlimit,
          mysettings.currentMonitoring_undercurrentlimit,
          mysettings.currentMonitoring_overpowerlimit,
          mysettings.currentMonitoring_shunttempcoefficient,
          mysettings.currentMonitoring_tempcompenabled);

      hal.ReleaseVSPIMutex();
    }
  }

  if (mysettings.currentMonitoringDevice == CurrentMonitorDevice::PZEM_017)
  {
    PZEM017_SetDeviceAddress(mysettings.currentMonitoringModBusAddress);
    PZEM017_SetShuntType(mysettings.currentMonitoringModBusAddress, shuntmaxcur);
  }
}

// Save the current monitor advanced settings back to the internal device
void CurrentMonitorSetRelaySettingsInternal(currentmonitoring_struct newvalues)
{
  // Internal current shunt doesn't support any of the relay trigger values
  // so only TempCompEnabled is stored
  if (hal.GetVSPIMutex())
  {
    mysettings.currentMonitoring_tempcompenabled = newvalues.TempCompEnabled;

    currentmon_internal.Configure(
        mysettings.currentMonitoring_shuntmv,
        mysettings.currentMonitoring_shuntmaxcur,
        mysettings.currentMonitoring_batterycapacity,
        mysettings.currentMonitoring_fullchargevolt,
        mysettings.currentMonitoring_tailcurrent,
        mysettings.currentMonitoring_chargeefficiency,
        mysettings.currentMonitoring_shuntcal,
        mysettings.currentMonitoring_temperaturelimit,
        mysettings.currentMonitoring_overvoltagelimit,
        mysettings.currentMonitoring_undervoltagelimit,
        mysettings.currentMonitoring_overcurrentlimit,
        mysettings.currentMonitoring_undercurrentlimit,
        mysettings.currentMonitoring_overpowerlimit,
        mysettings.currentMonitoring_shunttempcoefficient,
        mysettings.currentMonitoring_tempcompenabled);

    mysettings.currentMonitoring_tempcompenabled = currentmon_internal.calc_tempcompenabled();

    hal.ReleaseVSPIMutex();
  }

  ValidateConfiguration(&mysettings);
  SaveConfiguration(&mysettings);
}

// Save the current monitor advanced settings back to the device over MODBUS/RS485
void CurrentMonitorSetRelaySettingsExternal(currentmonitoring_struct newvalues)
{
  uint8_t flag1 = 0;
  uint8_t flag2 = 0;

  flag1 += newvalues.TempCompEnabled ? B00000010 : 0;

  // Use the previous value for setting the ADCRange4096mV flag
  flag1 += currentMonitor.ADCRange4096mV ? B00000001 : 0;

  // Apply new settings
  flag2 += newvalues.RelayTriggerTemperatureOverLimit ? bit(DIAG_ALRT_FIELD::TMPOL) : 0;
  flag2 += newvalues.RelayTriggerCurrentOverLimit ? bit(DIAG_ALRT_FIELD::SHNTOL) : 0;
  flag2 += newvalues.RelayTriggerCurrentUnderLimit ? bit(DIAG_ALRT_FIELD::SHNTUL) : 0;
  flag2 += newvalues.RelayTriggerVoltageOverlimit ? bit(DIAG_ALRT_FIELD::BUSOL) : 0;
  flag2 += newvalues.RelayTriggerVoltageUnderlimit ? bit(DIAG_ALRT_FIELD::BUSUL) : 0;
  flag2 += newvalues.RelayTriggerPowerOverLimit ? bit(DIAG_ALRT_FIELD::POL) : 0;

  /*
Flag 1
10|Temperature compensation enabled|Read write
9|ADC Range 0=±163.84 mV, 1=±40.96 mV (only 40.96mV supported by diyBMS)|Read only

Flag 2
8|Relay Trigger on TMPOL|Read write
7|Relay Trigger on SHNTOL|Read write
6|Relay Trigger on SHNTUL|Read write
5|Relay Trigger on BUSOL|Read write
4|Relay Trigger on BUSUL|Read write
3|Relay Trigger on POL|Read write
*/

  uint8_t cmd[MAX_SEND_RS485_PACKET_LENGTH];
  memset(&cmd, 0, sizeof(cmd));

  //	Write Multiple Holding Registers

  // The Slave Address
  cmd[0] = mysettings.currentMonitoringModBusAddress;
  // The Function Code 16
  cmd[1] = 16;
  // Data Address of the first register (9=40010, Various status flags)
  cmd[2] = 0;
  cmd[3] = 9;
  // number of registers to write
  cmd[4] = 0;
  cmd[5] = 1;
  // number of data bytes to follow (2 registers x 2 bytes each = 4 bytes)
  cmd[6] = 2;
  // value to write to register 40010
  cmd[7] = flag1;
  cmd[8] = flag2;

  xQueueSend(rs485_transmit_q_handle, &cmd, portMAX_DELAY);

  ESP_LOGD(TAG, "Write register 10 = %u %u", flag1, flag2);

  // Zero all data
  // memset(&currentMonitor, 0, sizeof(currentmonitoring_struct));
  currentMonitor.validReadings = false;
}

// Calculate estimated time to various % SoC
void TimeToSoCCalculation()
{
  // Ideally this current value would be averaged over the past few minutes
  float current = currentMonitor.modbus.current;

  time20 = 0;
  time10 = 0;
  time100 = 0;

  // Avoid divide by zero errors
  if (current == 0)
    return;

  // Calculate how "full" the battery is based on SoC %
  float now_capacity_ah = currentMonitor.modbus.batterycapacityamphour / 100.0F * currentMonitor.stateofcharge;

  // Target 100% - only if we are charging
  if (currentMonitor.stateofcharge < 100.0F && current > 0)
  {
    // Gap between now and 100%
    float empty_ah = currentMonitor.modbus.batterycapacityamphour - now_capacity_ah;
    // Use instantaneous current value to predict amp-hour (in number of seconds)
    time100 = (uint32_t)((empty_ah / abs(current)) * 60 * 60);
  }
  else
  {
    time100 = 0;
  }

  // Target 20% - only if we are discharging
  if (currentMonitor.stateofcharge > 20.0 && current < 0)
  {
    // Gap between now and 20%
    float empty_ah = now_capacity_ah - (currentMonitor.modbus.batterycapacityamphour * 0.20F);
    time20 = (uint32_t)((empty_ah / abs(current)) * 60 * 60);
  }
  else
  {
    time20 = 0;
  }

  // Target 10% - only if we are discharging
  if (currentMonitor.stateofcharge > 10.0 && current < 0)
  {
    // Gap between now and 10%
    float empty_ah = now_capacity_ah - (currentMonitor.modbus.batterycapacityamphour * 0.10F);
    time10 = (empty_ah / abs(current)) * 60 * 60;
  }
  else
  {
    time10 = 0;
  }

  // Sensible maximums to avoid silly timespans - limit anything over 10 days remaining...
  const uint32_t limit = 10 * 86400U;

  if (time100 > limit)
  {
    time100 = 0;
  }
  if (time20 > limit)
  {
    time20 = 0;
  }
  if (time10 > limit)
  {
    time10 = 0;
  }
}

void currentMon_ConfigureAdvancedExternal(currentmonitoring_struct newvalues)
{
  //	Write Multiple Holding Registers
  uint8_t cmd2[] = {
      // The Slave Address
      mysettings.currentMonitoringModBusAddress,
      // The Function Code 16
      16,
      // Data Address of the first register (|40028|INA_REGISTER::SHUNT_CAL (unsigned int16))
      0,
      27,
      // number of registers to write
      0,
      13,
      // number of data bytes to follow (13 registers x 2 bytes each)
      2 * 13,
      // value to write to register 40028
      // 21 = shuntcal
      (uint8_t)(newvalues.modbus.shuntcal >> 8),
      (uint8_t)(newvalues.modbus.shuntcal & 0xFF),
      // value to write to register 40029
      // temperaturelimit
      (uint8_t)(newvalues.modbus.temperaturelimit >> 8),
      (uint8_t)(newvalues.modbus.temperaturelimit & 0xFF),
      // overvoltagelimit 40030
      0,
      0,
      0,
      0,
      // undervoltagelimit 40032
      0,
      0,
      0,
      0,
      // overcurrentlimit 40034
      0,
      0,
      0,
      0,
      // undercurrentlimit 40029
      0,
      0,
      0,
      0,
      // overpowerlimit 40038
      0,
      0,
      0,
      0,
      // shunttempcoefficient 40
      (uint8_t)(newvalues.modbus.shunttempcoefficient >> 8),
      (uint8_t)(newvalues.modbus.shunttempcoefficient & 0xFF),
  };

  // ESP_LOGD(TAG, "temp limit=%i", newvalues.temperaturelimit);
  // ESP_LOGD(TAG, "shuntcal=%u", newvalues.shuntcal);

  // Register 18 = shunt_max_current
  // Register 19 = shunt_millivolt

  uint8_t ptr = SetMobusRegistersFromFloat(cmd2, 11, newvalues.modbus.overvoltagelimit);
  ptr = SetMobusRegistersFromFloat(cmd2, ptr, newvalues.modbus.undervoltagelimit);
  ptr = SetMobusRegistersFromFloat(cmd2, ptr, newvalues.modbus.overcurrentlimit);
  ptr = SetMobusRegistersFromFloat(cmd2, ptr, newvalues.modbus.undercurrentlimit);
  ptr = SetMobusRegistersFromFloat(cmd2, ptr, newvalues.modbus.overpowerlimit);

  uint8_t cmd[MAX_SEND_RS485_PACKET_LENGTH];
  memset(&cmd, 0, sizeof(cmd));
  memcpy(&cmd, &cmd2, sizeof(cmd2));
  xQueueSend(rs485_transmit_q_handle, &cmd, portMAX_DELAY);
}

// Save the current monitor advanced settings back to the device over MODBUS/RS485
void CurrentMonitorSetAdvancedSettings(currentmonitoring_struct newvalues)
{

  if (mysettings.currentMonitoringDevice == CurrentMonitorDevice::DIYBMS_CURRENT_MON_MODBUS)
  {
    currentMon_ConfigureAdvancedExternal(newvalues);
  }

  if (mysettings.currentMonitoringDevice == CurrentMonitorDevice::DIYBMS_CURRENT_MON_INTERNAL)
  {
    mysettings.currentMonitoring_shuntcal = newvalues.modbus.shuntcal;
    mysettings.currentMonitoring_temperaturelimit = newvalues.modbus.temperaturelimit;
    mysettings.currentMonitoring_overvoltagelimit = 100 * newvalues.modbus.overvoltagelimit;
    mysettings.currentMonitoring_undervoltagelimit = 100 * newvalues.modbus.undervoltagelimit;
    mysettings.currentMonitoring_overcurrentlimit = 100 * newvalues.modbus.overcurrentlimit;
    mysettings.currentMonitoring_undercurrentlimit = 100 * newvalues.modbus.undercurrentlimit;
    mysettings.currentMonitoring_overpowerlimit = newvalues.modbus.overpowerlimit;
    mysettings.currentMonitoring_shunttempcoefficient = newvalues.modbus.shunttempcoefficient;
    ValidateConfiguration(&mysettings);
    SaveConfiguration(&mysettings);

    if (hal.GetVSPIMutex())
    {
      currentmon_internal.Configure(
          mysettings.currentMonitoring_shuntmv,
          mysettings.currentMonitoring_shuntmaxcur,
          mysettings.currentMonitoring_batterycapacity,
          mysettings.currentMonitoring_fullchargevolt,
          mysettings.currentMonitoring_tailcurrent,
          mysettings.currentMonitoring_chargeefficiency,
          mysettings.currentMonitoring_shuntcal,
          mysettings.currentMonitoring_temperaturelimit,
          mysettings.currentMonitoring_overvoltagelimit,
          mysettings.currentMonitoring_undervoltagelimit,
          mysettings.currentMonitoring_overcurrentlimit,
          mysettings.currentMonitoring_undercurrentlimit,
          mysettings.currentMonitoring_overpowerlimit,
          mysettings.currentMonitoring_shunttempcoefficient,
          mysettings.currentMonitoring_tempcompenabled);
      hal.ReleaseVSPIMutex();
    }
  }

  ESP_LOGD(TAG, "Advanced save settings");

  // Zero all data
  memset(&currentMonitor, 0, sizeof(currentmonitoring_struct));
  currentMonitor.validReadings = false;
}

// Swap the two 16 bit words in a 32bit word
static inline unsigned int word16swap32(unsigned int __bsx)
{
  return ((__bsx & 0xffff0000) >> 16) | ((__bsx & 0x0000ffff) << 16);
}

// Extract the current monitor MODBUS registers into our internal STRUCTURE variables
void ProcessDIYBMSCurrentMonitorRegisterReply(uint8_t length)
{
  // ESP_LOGD(TAG, "Modbus len=%i, struct len=%i", length, sizeof(currentmonitor_raw_modbus));

  // ESP_LOG_BUFFER_HEXDUMP(TAG, &frame[3], length, esp_log_level_t::ESP_LOG_DEBUG);

  if (sizeof(currentmonitor_raw_modbus) != length)
  {
    // Abort if the packet sizes are different
    memset(&currentMonitor.modbus, 0, sizeof(currentmonitor_raw_modbus));
    currentMonitor.validReadings = false;
    return;
  }

  // Now byte swap to align to ESP32 endiness, and copy as we go into new structure
  auto *ptr = (uint8_t *)&currentMonitor.modbus;
  for (size_t i = 0; i < length; i += 2)
  {
    uint8_t temp = frame[3 + i];
    ptr[i] = frame[i + 4];
    ptr[i + 1] = temp;
  }

  // Finally, we have to fix the 32 bit fields
  currentMonitor.modbus.milliamphour_out = word16swap32(currentMonitor.modbus.milliamphour_out);
  currentMonitor.modbus.milliamphour_in = word16swap32(currentMonitor.modbus.milliamphour_in);
  currentMonitor.modbus.daily_milliamphour_out = word16swap32(currentMonitor.modbus.daily_milliamphour_out);
  currentMonitor.modbus.daily_milliamphour_in = word16swap32(currentMonitor.modbus.daily_milliamphour_in);
  currentMonitor.modbus.firmwareversion = word16swap32(currentMonitor.modbus.firmwareversion);
  currentMonitor.modbus.firmwaredatetime = word16swap32(currentMonitor.modbus.firmwaredatetime);

  // ESP_LOG_BUFFER_HEXDUMP(TAG, &currentMonitor.modbus, sizeof(currentmonitor_raw_modbus), esp_log_level_t::ESP_LOG_DEBUG);

  currentMonitor.timestamp = esp_timer_get_time();

  // High byte
  auto flag1 = (uint8_t)(currentMonitor.modbus.flags >> 8);
  // Low byte
  auto flag2 = (uint8_t)(currentMonitor.modbus.flags);

  // ESP_LOGD(TAG, "Read relay trigger settings %u %u", flag1, flag2);

  /*
16|TMPOL|Read only
15|SHNTOL|Read only
14|SHNTUL|Read only
13|BUSOL|Read only
12|BUSUL|Read only
11|POL|Read only
10|Temperature compensation enabled|Read write
9|ADC Range 0=±163.84 mV, 1=±40.96 mV (only 40.96mV supported by diyBMS)|Read only
*/

  currentMonitor.TemperatureOverLimit = flag1 & bit(DIAG_ALRT_FIELD::TMPOL);
  currentMonitor.CurrentOverLimit = flag1 & bit(DIAG_ALRT_FIELD::SHNTOL);
  currentMonitor.CurrentUnderLimit = flag1 & bit(DIAG_ALRT_FIELD::SHNTUL);
  currentMonitor.VoltageOverlimit = flag1 & bit(DIAG_ALRT_FIELD::BUSOL);
  currentMonitor.VoltageUnderlimit = flag1 & bit(DIAG_ALRT_FIELD::BUSUL);
  currentMonitor.PowerOverLimit = flag1 & bit(DIAG_ALRT_FIELD::POL);

  currentMonitor.TempCompEnabled = flag1 & B00000010;
  currentMonitor.ADCRange4096mV = flag1 & B00000001;

  /*
8|Relay Trigger on TMPOL|Read write
7|Relay Trigger on SHNTOL|Read write
6|Relay Trigger on SHNTUL|Read write
5|Relay Trigger on BUSOL|Read write
4|Relay Trigger on BUSUL|Read write
3|Relay Trigger on POL|Read write
2|Existing Relay state (0=off)|Read write
1|Factory reset bit (always 0 when read)|Read write
*/
  currentMonitor.RelayTriggerTemperatureOverLimit = flag2 & bit(DIAG_ALRT_FIELD::TMPOL);
  currentMonitor.RelayTriggerCurrentOverLimit = flag2 & bit(DIAG_ALRT_FIELD::SHNTOL);
  currentMonitor.RelayTriggerCurrentUnderLimit = flag2 & bit(DIAG_ALRT_FIELD::SHNTUL);
  currentMonitor.RelayTriggerVoltageOverlimit = flag2 & bit(DIAG_ALRT_FIELD::BUSOL);
  currentMonitor.RelayTriggerVoltageUnderlimit = flag2 & bit(DIAG_ALRT_FIELD::BUSUL);
  currentMonitor.RelayTriggerPowerOverLimit = flag2 & bit(DIAG_ALRT_FIELD::POL);
  currentMonitor.RelayState = flag2 & B00000010;
  // Last bit is for factory reset (always zero)

  currentMonitor.chargeefficiency = ((float)currentMonitor.modbus.raw_chargeefficiency) / 100.0F;
  currentMonitor.stateofcharge = ((float)currentMonitor.modbus.raw_stateofcharge) / 100.0F;

  currentMonitor.validReadings = true;

  TimeToSoCCalculation();
}

// Extract onboard/internal current monitor values into our internal STRUCTURE variables
void ProcessDIYBMSCurrentMonitorInternal()
{
  memset(&currentMonitor.modbus, 0, sizeof(currentmonitor_raw_modbus));
  currentMonitor.validReadings = false;

  currentMonitor.timestamp = esp_timer_get_time();
  currentMonitor.modbus.milliamphour_out = currentmon_internal.calc_milliamphour_out();
  currentMonitor.modbus.milliamphour_in = currentmon_internal.calc_milliamphour_in();
  currentMonitor.modbus.daily_milliamphour_out = currentmon_internal.calc_daily_milliamphour_out();
  currentMonitor.modbus.daily_milliamphour_in = currentmon_internal.calc_daily_milliamphour_in();
  currentMonitor.modbus.firmwareversion = (((uint32_t)GIT_VERSION_B1) << 16) + (uint32_t)GIT_VERSION_B2;
  currentMonitor.modbus.firmwaredatetime = COMPILE_DATE_TIME_UTC_EPOCH;

  /*
16|TMPOL|Read only
15|SHNTOL|Read only
14|SHNTUL|Read only
13|BUSOL|Read only
12|BUSUL|Read only
11|POL|Read only
10|Temperature compensation enabled|Read write
9|ADC Range 0=±163.84 mV, 1=±40.96 mV (only 40.96mV supported by diyBMS)|Read only
*/
  uint16_t flag1 = currentmon_internal.calc_alerts();
  currentMonitor.TemperatureOverLimit = flag1 & bit(DIAG_ALRT_FIELD::TMPOL);
  currentMonitor.CurrentOverLimit = flag1 & bit(DIAG_ALRT_FIELD::SHNTOL);
  currentMonitor.CurrentUnderLimit = flag1 & bit(DIAG_ALRT_FIELD::SHNTUL);
  currentMonitor.VoltageOverlimit = flag1 & bit(DIAG_ALRT_FIELD::BUSOL);
  currentMonitor.VoltageUnderlimit = flag1 & bit(DIAG_ALRT_FIELD::BUSUL);
  currentMonitor.PowerOverLimit = flag1 & bit(DIAG_ALRT_FIELD::POL);

  currentMonitor.TempCompEnabled = currentmon_internal.calc_tempcompenabled();
  currentMonitor.ADCRange4096mV = true;
  // mysettings.currentMonitoring_tempcompenabled = currentmon_internal.calc_tempcompenabled();

  currentMonitor.RelayTriggerTemperatureOverLimit = false;
  currentMonitor.RelayTriggerCurrentOverLimit = false;
  currentMonitor.RelayTriggerCurrentUnderLimit = false;
  currentMonitor.RelayTriggerVoltageOverlimit = false;
  currentMonitor.RelayTriggerVoltageUnderlimit = false;
  currentMonitor.RelayTriggerPowerOverLimit = false;
  currentMonitor.RelayState = false;

  currentMonitor.modbus.temperature = currentmon_internal.calc_temperature();
  currentMonitor.modbus.temperaturelimit = currentmon_internal.calc_temperaturelimit();
  currentMonitor.modbus.shunttempcoefficient = currentmon_internal.calc_shunttempcoefficient();
  currentMonitor.modbus.batterycapacityamphour = currentmon_internal.calc_batterycapacityAh();
  currentMonitor.modbus.shuntmaxcurrent = currentmon_internal.calc_shuntmaxcurrent();
  currentMonitor.modbus.shuntmillivolt = currentmon_internal.calc_shuntmillivolt();
  currentMonitor.modbus.shuntcal = currentmon_internal.calc_shuntcalibration();
  currentMonitor.modbus.modelnumber = 0x229;
  currentMonitor.modbus.power = currentmon_internal.calc_power();
  currentMonitor.modbus.overpowerlimit = currentmon_internal.calc_overpowerlimit();
  currentMonitor.modbus.voltage = currentmon_internal.calc_voltage();
  currentMonitor.modbus.current = currentmon_internal.calc_current();
  currentMonitor.modbus.shuntresistance = currentmon_internal.calc_shuntresistance();
  currentMonitor.modbus.tailcurrentamps = currentmon_internal.calc_tailcurrentamps();
  currentMonitor.chargeefficiency = currentmon_internal.calc_charge_efficiency_factor();
  currentMonitor.stateofcharge = currentmon_internal.calc_state_of_charge();
  currentMonitor.modbus.fullychargedvoltage = currentmon_internal.calc_fullychargedvoltage();
  currentMonitor.modbus.overvoltagelimit = currentmon_internal.calc_overvoltagelimit();
  currentMonitor.modbus.undervoltagelimit = currentmon_internal.calc_undervoltagelimit();
  currentMonitor.modbus.overcurrentlimit = currentmon_internal.calc_overcurrentlimit();
  currentMonitor.modbus.undercurrentlimit = currentmon_internal.calc_undercurrentlimit();

  currentMonitor.validReadings = true;
  TimeToSoCCalculation();

  /*
  ESP_LOGD(TAG, "WDog = %u", currentMonitor.modbus.watchdogcounter);
  ESP_LOGD(TAG, "SOC = %i", currentMonitor.stateofcharge);

  ESP_LOGD(TAG, "Volt = %f", currentMonitor.modbus.voltage);
  ESP_LOGD(TAG, "Curr = %f", currentMonitor.modbus.current);
  ESP_LOGD(TAG, "Temp = %i", currentMonitor.modbus.temperature);

  ESP_LOGD(TAG, "Out = %f", currentMonitor.modbus.milliamphour_in);
  ESP_LOGD(TAG, "In = %f", currentMonitor.modbus.milliamphour_out);

  ESP_LOGD(TAG, "Ver = %x", currentMonitor.modbus.firmwareversion);
  ESP_LOGD(TAG, "Date = %u", currentMonitor.modbus.firmwaredatetime);
*/
}
/// ESP32 CAN bus status strings, used for periodic status reporting
static const char *ESP32_TWAI_STATUS_STRINGS[] = {
    "STOPPED",               // CAN_STATE_STOPPED
    "RUNNING",               // CAN_STATE_RUNNING
    "OFF / RECOVERY NEEDED", // CAN_STATE_BUS_OFF
    "RECOVERY UNDERWAY"      // CAN_STATE_RECOVERING
};

void _send_canbus_message(const uint32_t identifier, const uint8_t *buffer, const uint8_t length, const uint32_t flags)
{
  twai_message_t message;
  message.identifier = identifier;
  message.flags = flags;
  message.data_length_code = length;

  memcpy(&message.data, buffer, length);

  // If there is a bus error, we attempt to recover it later, transmitted messages are lost, but this
  // isn't a problem, as they are repeated every few seconds.
  esp_err_t result = twai_transmit(&message, pdMS_TO_TICKS(250));

  if (result == ESP_OK)
  {
    // Everything normal/good
    // ESP_LOGD(TAG, "Sent CAN message 0x%x", identifier);
    // ESP_LOG_BUFFER_HEX_LEVEL(TAG, &message, sizeof(twai_message_t), esp_log_level_t::ESP_LOG_DEBUG);
    canbus_messages_sent++;
    return;
  }

  // Something failed....
  ESP_LOGE(TAG, "Failed to queue CANBUS message (0x%x)", result);
  canbus_messages_failed_sent++;

  twai_status_info_t status;
  twai_get_status_info(&status);

  ESP_LOGI(TAG, "CAN STATUS: rx-q:%d, tx-q:%d, rx-err:%d, tx-err:%d, arb-lost:%d, bus-err:%d, state: %s",
           status.msgs_to_rx, status.msgs_to_tx,
           status.rx_error_counter, status.tx_error_counter,
           status.arb_lost_count,
           status.bus_error_count,
           ESP32_TWAI_STATUS_STRINGS[status.state]);

  if (status.state == twai_state_t::TWAI_STATE_BUS_OFF)
  {
    // When the bus is OFF we need to initiate recovery, transmit is not possible when in this state.
    // Recovery appears to force it into STOPPED state
    ESP_LOGW(TAG, "Initiating recovery");
    twai_initiate_recovery();
  }
  else if (status.state == twai_state_t::TWAI_STATE_STOPPED)
  {
    // bus has stopped - restart it
    esp_err_t startresult = twai_start();
    ESP_LOGI(TAG, "Starting CANBUS %s", esp_err_to_name(startresult));
  }
  else if (status.state == twai_state_t::TWAI_STATE_RECOVERING)
  {
    // when the bus is in recovery mode transmit is not possible, so wait...
    vTaskDelay(pdMS_TO_TICKS(250));
  }
}

void send_canbus_message(const uint32_t identifier, const uint8_t *buffer, const uint8_t length)
{
  _send_canbus_message(identifier, buffer, length, TWAI_MSG_FLAG_NONE);
}
void send_ext_canbus_message(const uint32_t identifier, const uint8_t *buffer, const uint8_t length)
{
  _send_canbus_message(identifier, buffer, length, TWAI_MSG_FLAG_EXTD);
}

[[noreturn]] void canbus_tx(void *)
{
  for (;;)
  {
    // Delay 1 second
    vTaskDelay(pdMS_TO_TICKS(1000));

    if (mysettings.canbusprotocol == CanBusProtocolEmulation::CANBUS_PYLONTECH)
    {
      // Pylon Tech Battery Emulation
      // https://github.com/PaulSturbo/DIY-BMS-CAN/blob/main/SEPLOS%20BMS%20CAN%20Protocoll%20V1.0.pdf
      // https://www.setfirelabs.com/green-energy/pylontech-can-reading-can-replication
      // https://github.com/juamiso/PYLON_EMU
      // https://www.studocu.com/row/document/abasyn-university/electronics-engineering/can-bus-protocol-pylon-low-voltage-v1/17205338

      /*
      PYLON TECH battery transmits these values....

      CAN ID – followed by 2 to 8 bytes of data:
      0x351 – 14 02 74 0E 74 0E CC 01 – Battery voltage + current limits
      0x355 – 1A 00 64 00 – State of Health (SOH) / State of Charge (SOC)
      0x356 – 4e 13 02 03 04 05 – Voltage / Current / Temp
      0x359 – 00 00 00 00 0A 50 4E – Protection & Alarm flags
      0x35C – C0 00 – Battery charge request flags
      0x35E – 50 59 4C 4F 4E 20 20 20 – Manufacturer name (“PYLON “)

      If you are watching the bus, you will also see a 0x305 ID message which is output by the inverter once per second.
*/
      pylon_message_351();
      vTaskDelay(pdMS_TO_TICKS(20));

      if (_controller_state == ControllerState::Running)
      {
        pylon_message_355();
        vTaskDelay(pdMS_TO_TICKS(20));
        pylon_message_356();
        vTaskDelay(pdMS_TO_TICKS(20));
      }
      pylon_message_359();
      vTaskDelay(pdMS_TO_TICKS(20));
      pylon_message_35c();
      vTaskDelay(pdMS_TO_TICKS(20));
      pylon_message_35e();
      // Delay a little whilst sending packets to give ESP32 some breathing room and not flood the CANBUS
      // vTaskDelay(pdMS_TO_TICKS(100));
    }
    else if (mysettings.canbusprotocol == CanBusProtocolEmulation::CANBUS_PYLONFORCEH2 )
    {
      pylonforce_handle_tx();
    }
    else if (mysettings.canbusprotocol == CanBusProtocolEmulation::CANBUS_VICTRON)
    {
      // minimum CAN-IDs required for the core functionality are 0x351, 0x355, 0x356 and 0x35A.

      // 351 message must be sent at least every 3 seconds - or Victron will stop charge/discharge
      victron_message_351();

      // Delay a little whilst sending packets to give ESP32 some breathing room and not flood the CANBUS
      vTaskDelay(pdMS_TO_TICKS(100));

      // Advertise the diyBMS name on CANBUS
      victron_message_370_371();
      victron_message_35e();
      victron_message_35a();
      victron_message_372();
      victron_message_35f();

      vTaskDelay(pdMS_TO_TICKS(100));

      if (_controller_state == ControllerState::Running)
      {
        victron_message_355();
        victron_message_356();

        vTaskDelay(pdMS_TO_TICKS(100));

        // Detail about individual cells
        victron_message_373();
        victron_message_374_375_376_377();
      }
    }
  }
}

[[noreturn]] void canbus_rx(void *)
{
  for (;;)
  {
    while (mysettings.canbusprotocol == CanBusProtocolEmulation::CANBUS_DISABLED)
    {
      // Canbus is disbled, sleep until this changes....
      vTaskDelay(pdMS_TO_TICKS(2000));
    }

    // Wait for message to be received, up to 20 seconds
    twai_message_t message;
    esp_err_t res = twai_receive(&message, pdMS_TO_TICKS(20000));
    if (res == ESP_OK)
    {
      canbus_messages_received++;
      ESP_LOGD(TAG, "CANBUS received message ID: %0x, DLC: %d, flags: %0x",
               message.identifier, message.data_length_code, message.flags);
      if (!(message.flags & TWAI_MSG_FLAG_RTR))   // we do not answer to Remote-Transmission-Requests
      {
//        ESP_LOG_BUFFER_HEXDUMP(TAG, message.data, message.data_length_code, ESP_LOG_DEBUG);
        if (mysettings.canbusprotocol == CanBusProtocolEmulation::CANBUS_PYLONFORCEH2 )
        {
          pylonforce_handle_rx(&message);
        }
        else
        {
          // Remote inverter should send a 305 message every few seconds
          // for now, keep track of last message.
          // TODO: in future, add timeout/error condition to shut down
          if (message.identifier == 0x305)
          {
            canbus_last_305_message_time = esp_timer_get_time();
          }
        }
      }
    }
    else
    {
      /// ignore the timeout or do something
      ESP_LOGE(TAG, "CANBUS error %s", esp_err_to_name(res));
      canbus_messages_received_error++;
      ESP_LOGI(TAG, "CANBUS error count %u", canbus_messages_received_error);
    }
  }
}

[[noreturn]] void service_rs485_transmit_q(void *)
{
  for (;;)
  {
    uint8_t cmd[MAX_SEND_RS485_PACKET_LENGTH];

    if (rs485_transmit_q_handle != nullptr)
    {
      // Wait for a item in the queue, blocking indefinately
      xQueueReceive(rs485_transmit_q_handle, &cmd, portMAX_DELAY);

      if (hal.GetRS485Mutex())
      {
        // Ensure we have empty receive buffer
        // uart_flush_input(rs485_uart_num);

        // Default of 8 bytes for a modbus request (including CRC)
        size_t packet_length = 8;

        if (cmd[1] == 15 || cmd[1] == 16)
        {
          // Calculate length of this packet, add on extra data
          // Force Multiple Coils (FC=15)
          // https://www.simplymodbus.ca/FC15.htm
          // Preset Multiple Registers (FC=16)
          // https://www.simplymodbus.ca/FC16.htm
          packet_length = 9 + cmd[6];
        }

        // Calculate the MODBUS CRC
        auto temp = calculateCRC(cmd, (uint8_t)(packet_length - 2));
        // Byte swap the Hi and Lo bytes
        auto crc16 = (uint16_t)(temp << 8) | (temp >> 8);
        cmd[packet_length - 2] = (uint8_t)(crc16 >> 8); // split crc into 2 bytes
        cmd[packet_length - 1] = (uint8_t)(crc16 & 0xFF);

        // Send the bytes (actually just put them into the TX FIFO buffer)
        uart_write_bytes(rs485_uart_num, (char *)cmd, packet_length);

        hal.ReleaseRS485Mutex();

        // Notify the receive task that a packet should be on its way
        if (rs485_rx_task_handle != nullptr)
        {
          xTaskNotify(rs485_rx_task_handle, 0x00, eNotifyAction::eNoAction);
        }

        ESP_LOGD(TAG, "Send addr=%u, func=%u, len=%u", cmd[0], cmd[1], packet_length);
        // Debug
        // ESP_LOG_BUFFER_HEXDUMP(TAG, cmd, packet_length, esp_log_level_t::ESP_LOG_DEBUG);

        // Once we have notified the receive task, we pause here to avoid sending
        // another request until the last one has been processed (or we timeout after 2 seconds)
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2000));
      }
    }
    else
    {
      ESP_LOGE(TAG, "rs485_transmit_q_handle is NULL");
    }
  }
}

// RS485 receive
[[noreturn]] void rs485_rx(void *)
{
  for (;;)
  {
    // Wait until this task is triggered (sending queue task triggers it)
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Delay 50ms for the data to arrive
    vTaskDelay(pdMS_TO_TICKS(50));

    uint16_t len = 0;

    if (hal.GetRS485Mutex())
    {
      // Wait 200ms before timeout
      len = (uint16_t)uart_read_bytes(rs485_uart_num, frame, sizeof(frame), pdMS_TO_TICKS(200));
      hal.ReleaseRS485Mutex();
    }

    // Min packet length of 5 bytes
    if (len > 5)
    {
      uint8_t id = frame[0];

      auto crc = (uint16_t)((frame[len - 2] << 8) | frame[len - 1]); // combine the crc Low & High bytes

      auto temp = calculateCRC(frame, (uint8_t)(len - 2));
      // Swap bytes to match MODBUS ordering
      auto calculatedCRC = (uint16_t)(temp << 8) | (uint16_t)(temp >> 8);

      // ESP_LOG_BUFFER_HEXDUMP(TAG, frame, len, esp_log_level_t::ESP_LOG_DEBUG);

      if (calculatedCRC == crc)
      {
        // if the calculated crc matches the recieved crc continue to process data...
        uint8_t RS485Error = frame[1] & B10000000;
        if (RS485Error == 0)
        {
          uint8_t cmd = frame[1] & B01111111;
          uint8_t length = frame[2];

          ESP_LOGD(TAG, "Recv %i bytes, id=%u, cmd=%u", len, id, cmd);
          // ESP_LOG_BUFFER_HEXDUMP(TAG, frame, len, esp_log_level_t::ESP_LOG_DEBUG);

          if (mysettings.currentMonitoringDevice == CurrentMonitorDevice::PZEM_017)
          {
            if (cmd == 6 && id == 248)
            {
              ESP_LOGI(TAG, "Reply to broadcast/change address");
            }
            if (cmd == 6 && id == mysettings.currentMonitoringModBusAddress)
            {
              ESP_LOGI(TAG, "Reply to set param");
            }
            else if (cmd == 3 && id == mysettings.currentMonitoringModBusAddress)
            {
              // 75mV shunt (hard coded for PZEM)
              currentMonitor.modbus.shuntmillivolt = 75;

              // Shunt type 0x0000 - 0x0003 (100A/50A/200A/300A)
              switch (((uint32_t)frame[9] << 8 | (uint32_t)frame[10]))
              {
              case 0:
                currentMonitor.modbus.shuntmaxcurrent = 100;
                break;
              case 1:
                currentMonitor.modbus.shuntmaxcurrent = 50;
                break;
              case 2:
                currentMonitor.modbus.shuntmaxcurrent = 200;
                break;
              case 3:
                currentMonitor.modbus.shuntmaxcurrent = 300;
                break;
              default:
                currentMonitor.modbus.shuntmaxcurrent = 0;
              }
            }
            else if (cmd == 4 && id == mysettings.currentMonitoringModBusAddress && len == 21)
            {
              // ESP_LOG_BUFFER_HEXDUMP(TAG, frame, len, esp_log_level_t::ESP_LOG_DEBUG);

              // memset(&currentMonitor.modbus, 0, sizeof(currentmonitor_raw_modbus));
              currentMonitor.validReadings = true;
              currentMonitor.timestamp = esp_timer_get_time();
              // voltage in 0.01V
              currentMonitor.modbus.voltage = (float)((uint32_t)frame[3] << 8 | (uint32_t)frame[4]) / (float)100.0;
              // current in 0.01A
              currentMonitor.modbus.current = (float)((uint32_t)frame[5] << 8 | (uint32_t)frame[6]) / (float)100.0;
              // power in 0.1W
              currentMonitor.modbus.power = ((uint32_t)frame[7] << 8 | (uint32_t)frame[8] | (uint32_t)frame[9] << 24 | (uint32_t)frame[10] << 16) / 10.0F;
            }
            else
            {
              // Dump out unhandled reply
              ESP_LOG_BUFFER_HEXDUMP(TAG, frame, len, esp_log_level_t::ESP_LOG_DEBUG);
            }
          }
          // ESP_LOGD(TAG, "CRC pass Id=%u F=%u L=%u", id, cmd, length);
          if (mysettings.currentMonitoringDevice == CurrentMonitorDevice::DIYBMS_CURRENT_MON_MODBUS)
          {
            if (id == mysettings.currentMonitoringModBusAddress && cmd == 3)
            {
              ProcessDIYBMSCurrentMonitorRegisterReply(length);

              if (_tft_screen_available)
              {
                // Refresh the TFT display
                xTaskNotify(updatetftdisplay_task_handle, 0x00, eNotifyAction::eNoAction);
              }
            }
            else if (id == mysettings.currentMonitoringModBusAddress && cmd == 16)
            {
              ESP_LOGI(TAG, "Write multiple regs, success");
            }
            else
            {
              // Dump out unhandled reply
              ESP_LOG_BUFFER_HEXDUMP(TAG, frame, len, esp_log_level_t::ESP_LOG_DEBUG);
            }
          }
        }
        else
        {
          ESP_LOGE(TAG, "RS485 error");
          ESP_LOG_BUFFER_HEXDUMP(TAG, frame, len, esp_log_level_t::ESP_LOG_DEBUG);
        }
      }
      else
      {
        ESP_LOGE(TAG, "CRC error");
      }
    }
    else
    {
      // We didn't receive anything on RS485, record error and mark current monitor as invalid
      ESP_LOGE(TAG, "Short packet %i bytes", len);

      // Indicate that the current monitor values are now invalid/unknown
      currentMonitor.validReadings = false;
    }

    // Notify sending queue, to continue
    xTaskNotify(service_rs485_transmit_q_task_handle, 0x00, eNotifyAction::eNoAction);

  } // infinite loop
}

// RS485 transmit
[[noreturn]] void rs485_tx(void *)
{
  uint8_t cmd[MAX_SEND_RS485_PACKET_LENGTH];
  memset(&cmd, 0, sizeof(cmd));

  for (;;)
  {
    // Delay 4 seconds between requests
    vTaskDelay(pdMS_TO_TICKS(4000));

    if (mysettings.currentMonitoringEnabled == true)
    {

      if (mysettings.currentMonitoringDevice == CurrentMonitorDevice::DIYBMS_CURRENT_MON_INTERNAL)
      {
        if (currentmon_internal.Available())
        {
          // Take readings from internal INA229 chip (on controller board)
          if (hal.GetVSPIMutex())
          {
            currentmon_internal.TakeReadings();
            ProcessDIYBMSCurrentMonitorInternal();
            hal.ReleaseVSPIMutex();
          }
        }
        else
        {
          currentMonitor.validReadings = false;
        }
      }

      if (mysettings.currentMonitoringDevice == CurrentMonitorDevice::DIYBMS_CURRENT_MON_MODBUS)
      {
        // This is the request we send to diyBMS current monitor, it pulls back 38 registers
        // this is all the registers diyBMS current monitor has
        // Holding Registers = command 3
        cmd[0] = mysettings.currentMonitoringModBusAddress;
        // Input registers - 46 of them (92 bytes + headers + crc = 83 byte reply)
        cmd[1] = 3;
        cmd[5] = 46;
        xQueueSend(rs485_transmit_q_handle, &cmd, portMAX_DELAY);
      }

      if (mysettings.currentMonitoringDevice == CurrentMonitorDevice::PZEM_017)
      {
        // ESP_LOGD(TAG, "RS485 TX");
        cmd[0] = mysettings.currentMonitoringModBusAddress;

        if (currentMonitor.modbus.shuntmillivolt == 0)
        {
          ESP_LOGD(TAG, "PZEM_017 Read params");
          cmd[1] = 0x03;
          cmd[5] = 0x04;
        }
        else
        {
          // ESP_LOGD(TAG, "PZEM_017 Read values");
          //  Read the standard voltage/current values
          //   Input registers
          cmd[1] = 0x04;
          // Read 8 registers (0 to 8)
          cmd[5] = 0x08;
        }
        xQueueSend(rs485_transmit_q_handle, &cmd, portMAX_DELAY);
      }
    } // end if
  }
}

[[noreturn]] void periodic_task(void *)
{
  uint8_t countdown_influx = mysettings.influxdb_loggingFreqSeconds;
  uint8_t countdown_mqtt1 = 5;
  uint8_t countdown_mqtt2 = 25;

  for (;;)
  {
    // Delay 1 second
    vTaskDelay(pdMS_TO_TICKS(1000));

    countdown_influx--;
    countdown_mqtt1--;
    countdown_mqtt2--;

    if (tftsleep_timer > 0)
    {
      // Timer can get to 0
      tftsleep_timer--;

      IncreaseDelayCounter();
    }

    // ESP_LOGI(TAG, "mqtt1=%u, mqtt2=%u, influx=%u, tftsleep=%u",countdown_mqtt1,countdown_mqtt2,countdown_influx,tftsleep_timer);

    // 5 seconds
    if (countdown_mqtt1 == 0)
    {
      mqtt1(&currentMonitor, &rules);
      countdown_mqtt1 = 5;
    }

    // 25 seconds
    if (countdown_mqtt2 == 0)
    {
      mqtt2(&receiveProc, &prg, prg.queueLength(), &rules);

      // Trigger mqtt3 as well (on a periodic schedule)
      xTaskNotify(rule_state_change_task_handle, 0x00, eNotifyAction::eNoAction);

      countdown_mqtt2 = 25;
    }

    // Influxdb - Variable interval
    if (countdown_influx == 0)
    {
      countdown_influx = mysettings.influxdb_loggingFreqSeconds;

      if (mysettings.influxdb_enabled && wifi_isconnected && rules.invalidModuleCount == 0 && _controller_state == ControllerState::Running && rules.ruleOutcome(Rule::BMSError) == false)
      {
        ESP_LOGI(TAG, "Influx task");
        influx_task_action();
      }
    }

    if (_screen_awake && tftsleep_timer == 0 &&
        (WhatScreenToDisplay() != ScreenTemplateToDisplay::Error && WhatScreenToDisplay() != ScreenTemplateToDisplay::AVRProgrammer))
    {
      // Screen off
      tftsleep();
    }
  }
}

// Do activities which are not critical to the system like background loading of config, or updating timing results etc.
[[noreturn]] void lazy_tasks(void *)
{
  int year_day = -1;
  time_t snapshot_time = 0;

  for (;;)
  {
    // TODO: Perhaps this should be based on some improved logic - based on number of modules in system?
    //  Delay 5.5 seconds

    // ESP_LOGI(TAG, "Sleep");
    auto delay_ticks = pdMS_TO_TICKS(5500);
    vTaskDelay(delay_ticks);

    // Task 1
    ESP_LOGI(TAG, "Task 1");
    //  Send a "ping" message through the cells to get a round trip time
    prg.sendTimingRequest();

    // Check if clock has rolled over to next day, this won't fire exactly on midnight
    // but should be within 0-20 seconds of it.
    if (sntp_enabled())
    {
      time_t now;
      time(&now);
      struct tm timeinfo;
      localtime_r(&now, &timeinfo);

      // Wait for SNTP to get a valid date/time
      if (timeinfo.tm_year > 70)
      {
        // Has day rolled over?
        if (year_day != timeinfo.tm_yday)
        {
          // Reset the current monitor at midnight (ish)
          CurrentMonitorResetDailyAmpHourCounters();

          for (uint8_t i = 0; i < maximum_controller_cell_modules; i++)
          {
            resetModuleMinMaxVoltage(i);
          }
        }
        year_day = timeinfo.tm_yday;

        // Has clock rolled over for snaps?
        if (snapshot_time != 0 && now > snapshot_time)
        {
          char strftime_buf[64];
          strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
          ESP_LOGI(TAG, "Snap timer: %s", strftime_buf);

          history.SnapshotHistory(now, &rules, &currentMonitor);
          snapshot_time = 0;
        }

        if (snapshot_time == 0)
        {
          // Calculate the next time to do a snapshot in 30 minutes
          snapshot_time = (now - (now % 1800)) + 1800;
        }
      }
    }

    // Sleep between sections to give the ESP a chance to do other stuff
    vTaskDelay(delay_ticks);

    // Task 2
    ESP_LOGI(TAG, "Task 2");
    //  Find modules that don't have settings cached and request them
    for (uint8_t m = 0; m < TotalNumberOfCells(); m++)
    {
      if (cmi[m].valid)
      {
        if (cmi[m].settingsCached == false)
        {
          // This will block if the queue length is reached
          prg.sendGetSettingsRequest(m);
        }
        else
        {
          if (cmi[m].BoardVersionNumber == 490 && cmi[m].FanSwitchOnTemperature == 0)
          {
            // 490=All in one 16S board, which has additional configuration options
            prg.sendGetAdditionalSettingsRequest(m);
          }
        }
      }
    }

    // Sleep between sections to give the ESP a chance to do other stuff
    vTaskDelay(delay_ticks);

    // Task 3
    //  Send these requests to all banks of modules
    uint8_t i = 0;
    uint8_t max = TotalNumberOfCells();

    uint8_t startmodule = 0;

    while (i < max)
    {
      uint8_t endmodule = (startmodule + maximum_cell_modules_per_packet) - 1;

      // Limit to number of modules we have configured
      if (endmodule > max)
      {
        endmodule = max - 1;
      }

      ESP_LOGD(TAG, "Task 3, s=%i e=%i", startmodule, endmodule);
      prg.sendReadBalanceCurrentCountRequest(startmodule, endmodule);
      prg.sendReadPacketsReceivedRequest(startmodule, endmodule);
      prg.sendReadBadPacketCounter(startmodule, endmodule);

      // Delay per bank/loop
      vTaskDelay(delay_ticks);

      // Move to the next bank
      startmodule = endmodule + 1;
      i += maximum_cell_modules_per_packet;
    } // end while

  } // end for
}

bool CaptureSerialInput(char *buffer, int buffersize, bool OnlyDigits, bool ShowPasswordChar)
{
  int length = 0;
  unsigned long timer = millis() + 30000;

  // Ensure buffer is all zeros
  memset(buffer, 0, buffersize);

  while (true)
  {
    // We should add a timeout in here, and return FALSE when we abort....

    // fgetc is blocking call
    int data = fgetc(stdin);

    if (data == EOF)
    {
      // Ignore
      // Abort after 30 seconds of inactivity
      if (millis() > timer)
        return false;
    }
    else if (data == '\b' || data == '\177')
    { // BS and DEL
      if (length)
      {
        length--;
        fputs("\b \b", stdout);
      }
    }

    else if (data == '\n' || data == '\r')
    {
      if (length > 0)
      {
        fputs("\n", stdout); // output newline

        // Mark end of string
        buffer[length] = '\0';

        // Return to caller
        return true;
      }

      length = 0;
    }
    else if (length < buffersize - 1)
    {
      // Reset timer on serial input
      timer = millis() + 30000;

      if (OnlyDigits && (data < '0' || data > '9'))
      {
        // We need to filter out non-digit characters
      }
      else
      {
        buffer[length++] = (char)data;
        fputc(ShowPasswordChar ? '*' : (char)data, stdout);
      }
    }
  }
}

bool DeleteWiFiConfigFromSDCard()
{
  bool ret = false;
  if (_sd_card_installed)
  {
    ESP_LOGI(TAG, "Delete file check %s", wificonfigfilename.c_str());

    if (hal.GetVSPIMutex())
    {
      if (SD.exists(wificonfigfilename.c_str()))
      {
        ESP_LOGI(TAG, "Deleted %s", wificonfigfilename.c_str());
        ret = SD.remove(wificonfigfilename.c_str());
      }

      hal.ReleaseVSPIMutex();
    }
  }

  return ret;
}

[[noreturn]] void TerminalBasedWifiSetup()
{
  SetControllerState(ControllerState::NoWifiConfiguration);

  if (_tft_screen_available)
  {
    // Show the prompt "configure wifi settings" on TFT screen
    if (hal.GetDisplayMutex())
    {
      PrepareTFT_ControlState();
      DrawTFT_ControlState();
      hal.ReleaseDisplayMutex();
      hal.TFTScreenBacklight(true);
    }
  }

  fputs("\n\nDIYBMS CONTROLLER - Scanning Wifi\n\n", stdout);

  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
  assert(sta_netif);
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_start());
  esp_wifi_scan_start(nullptr, true);

  // Max of 32 stations to return
  uint16_t number = 32;
  wifi_ap_record_t ap_info[32];
  uint16_t ap_count = 0;
  memset(ap_info, 0, sizeof(ap_info));

  ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&number, ap_info));
  ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_count));

  // ESP_LOGI(TAG, "Total APs scanned = %u", ap_count);

  fputs("Number\tSSID Name                             RSSI\tChannel\n", stdout);
  for (int i = 0; (i < 32) && (i < ap_count); i++)
  {
    char text[10];
    itoa(i, text, 10);
    fputs(text, stdout);
    fputs(":\t", stdout);
    fputs((char *)ap_info[i].ssid, stdout);

    // Pad out to 38 characters on screen
    size_t len = strlen((char *)ap_info[i].ssid);
    for (; len < 38; len++)
    {
      fputc(' ', stdout);
    }

    itoa(ap_info[i].rssi, text, 10);
    fputs(text, stdout);
    fputc('\t', stdout);
    itoa(ap_info[i].primary, text, 10);
    fputs(text, stdout);

    // New line
    fputc('\n', stdout);
  }

  // Green
  fputs("Enter the NUMBER of the Wifi network to connect to: ", stdout);

  bool result;
  char buffer[64];
  result = CaptureSerialInput(buffer, 3, true, false);
  if (result)
  {
    int index = atoi(buffer);
    fputs("", stdout);
    fputs("Enter the password to use when connecting to '", stdout);
    fputs((char *)ap_info[index].ssid, stdout);
    fputs("':", stdout);

    result = CaptureSerialInput(buffer, sizeof(buffer), false, true);

    if (result)
    {
      wifi_eeprom_settings config;
      // This wipes out any manual settings/config
      memset(&config, 0, sizeof(config));
      strncpy(config.wifi_ssid, (char *)ap_info[index].ssid, sizeof(config.wifi_ssid));
      strncpy(config.wifi_passphrase, buffer, sizeof(config.wifi_passphrase));
      SaveWIFI(&config);
      // Now delete the WIFICONFIG backup from the SDCard to prevent boot loops/resetting defaults
      DeleteWiFiConfigFromSDCard();
    }
  }

  fputc('\n', stdout);
  fputc('\n', stdout);
  for (size_t i = 5; i > 0; i--)
  {
    char n[5];
    itoa(i, n, 10);
    fputs("Rebooting in ", stdout);
    fputs(n, stdout);
    fputs(" seconds\n", stdout);
    delay(1000);
  }

  esp_restart();
}

/// @brief Compare two sets of wifi_eeprom_settings structures
/// @param a Set 1
/// @param b Set 2
/// @return TRUE if they are equal
bool AreWifiConfigurationsTheSame(const wifi_eeprom_settings *a, const wifi_eeprom_settings *b)
{
  if (a->wifi_ip != b->wifi_ip)
    return false;
  if (a->wifi_gateway != b->wifi_gateway)
    return false;
  if (a->wifi_netmask != b->wifi_netmask)
    return false;
  if (a->wifi_dns1 != b->wifi_dns1)
    return false;
  if (a->wifi_dns2 != b->wifi_dns2)
    return false;
  if (strncmp(a->wifi_ssid, b->wifi_ssid, sizeof(a->wifi_ssid)) != 0)
    return false;
  if (strncmp(a->wifi_passphrase, b->wifi_passphrase, sizeof(a->wifi_passphrase)) != 0)
    return false;

  if (a->manualConfig != b->manualConfig)
    return false;

  return true;
}

/// @brief CHECK FOR THE PRESENCE OF A wifi.json CONFIG FILE ON THE SD CARD AND AUTOMATICALLY CONFIGURE WIFI
/// @param existingConfigValid true if the wifi settings in FLASH are valid/correctly stored
/// @return TRUE if the WIFI config on the card is different, false if identical
bool LoadWiFiConfigFromSDCard(const bool existingConfigValid)
{
  StaticJsonDocument<2048> json;
  DeserializationError error;

  if (!_sd_card_installed)
    return false;

  ESP_LOGI(TAG, "Checking for %s", wificonfigfilename.c_str());

  if (hal.GetVSPIMutex())
  {
    if (SD.exists(wificonfigfilename.c_str()))
    {
      ESP_LOGD(TAG, "Found file %s", wificonfigfilename.c_str());

      File file = SD.open(wificonfigfilename.c_str());
      error = deserializeJson(json, file);
      file.close();
    }
    hal.ReleaseVSPIMutex();
  }

  if (error != DeserializationError::Ok)
  {
    ESP_LOGE(TAG, "Error deserialize JSON");
    return false;
  }

  ESP_LOGD(TAG, "Deserialized %s", wificonfigfilename.c_str());

  JsonObject wifi = json["wifi"];

  wifi_eeprom_settings _new_config;

  // Clear config
  memset(&_new_config, 0, sizeof(_new_config));

  wifi["ssid"].as<String>().toCharArray(_new_config.wifi_ssid, sizeof(_new_config.wifi_ssid));
  wifi["password"].as<String>().toCharArray(_new_config.wifi_passphrase, sizeof(_new_config.wifi_passphrase));

  if (wifi.containsKey("ip"))
  {
    // If the "ip" key exists, then it must be the newer format WIFI JSON document
    ip4_addr_t ipadd;

    if (ip4addr_aton(wifi["ip"].as<String>().c_str(), &ipadd))
    {
      _new_config.wifi_ip = ipadd.addr;
    }
    if (ip4addr_aton(wifi["gateway"].as<String>().c_str(), &ipadd))
    {
      _new_config.wifi_gateway = ipadd.addr;
    }
    if (ip4addr_aton(wifi["netmask"].as<String>().c_str(), &ipadd))
    {
      _new_config.wifi_netmask = ipadd.addr;
    }
    if (ip4addr_aton(wifi["dns1"].as<String>().c_str(), &ipadd))
    {
      _new_config.wifi_dns1 = ipadd.addr;
    }
    if (ip4addr_aton(wifi["dns2"].as<String>().c_str(), &ipadd))
    {
      _new_config.wifi_dns2 = ipadd.addr;
    }

    _new_config.manualConfig = wifi["manualconfig"].as<uint8_t>();
  }
  else
  {
    ESP_LOGI(TAG, "Older style WIFI format");
    // Its an original style WIFI JSON document - before manual IP was available
    // Copy over the existing parameters (probably all empty values)
    // this makes sure the AreWifiConfigurationsTheSame passes on the "old" JSON document style
    _new_config.wifi_ip = _wificonfig.wifi_ip;
    _new_config.wifi_gateway = _wificonfig.wifi_gateway;
    _new_config.wifi_netmask = _wificonfig.wifi_netmask;
    _new_config.wifi_dns1 = _wificonfig.wifi_dns1;
    _new_config.wifi_dns2 = _wificonfig.wifi_dns2;
    _new_config.manualConfig = _wificonfig.manualConfig;
  }

  // Our configuration is different, so store the details in EEPROM and flash the LED a few times
  if (existingConfigValid == false || AreWifiConfigurationsTheSame(&_wificonfig, &_new_config) == false)
  {
    memcpy(&_wificonfig, &_new_config, sizeof(_new_config));
    ESP_LOGI(TAG, "Wifi config is different, saving");
    SaveWIFI(&_new_config);
    return true;
  }

  ESP_LOGI(TAG, "Wifi JSON config is identical - ignoring");

  return false;
}

static void tft_interrupt_attach(void *)
{
  attachInterrupt(TOUCH_IRQ, TFTScreenTouchInterrupt, FALLING);
}

struct log_level_t
{
  const char *tag;
  esp_log_level_t level;
};

// Default log levels to use for various components.
const std::array<log_level_t, 22> log_levels =
    {
        log_level_t{.tag = "*", .level = ESP_LOG_DEBUG},
        {.tag = "wifi", .level = ESP_LOG_WARN},
        {.tag = "dhcpc", .level = ESP_LOG_WARN},
        {.tag = "diybms", .level = ESP_LOG_DEBUG},
        {.tag = "diybms-avrisp", .level = ESP_LOG_INFO},
        {.tag = "diybms-hal", .level = ESP_LOG_INFO},
        {.tag = "diybms-influxdb", .level = ESP_LOG_INFO},
        {.tag = "diybms-rx", .level = ESP_LOG_INFO},
        {.tag = "diybms-tx", .level = ESP_LOG_INFO},
        {.tag = "diybms-rules", .level = ESP_LOG_INFO},
        {.tag = "diybms-softap", .level = ESP_LOG_INFO},
        {.tag = "diybms-tft", .level = ESP_LOG_INFO},
        {.tag = "diybms-victron", .level = ESP_LOG_INFO},
        {.tag = "diybms-webfuncs", .level = ESP_LOG_INFO},
        {.tag = "diybms-webpost", .level = ESP_LOG_INFO},
        {.tag = "diybms-webreq", .level = ESP_LOG_INFO},
        {.tag = "diybms-web", .level = ESP_LOG_INFO},
        {.tag = "diybms-set", .level = ESP_LOG_INFO},
        {.tag = "diybms-mqtt", .level = ESP_LOG_INFO},
        {.tag = "diybms-pylon", .level = ESP_LOG_INFO},
        {.tag = "diybms-pyforce", .level = ESP_LOG_INFO},
        {.tag = "curmon", .level = ESP_LOG_INFO}};

void consoleConfigurationCheck()
{
  // Allow user to press SPACE BAR key on serial terminal to enter text based WIFI setup
  // Make sure this is always output regardless of the IDF logging level
  fputs("\n\n\nPress SPACE BAR to enter console WiFi configuration....\n\n\n", stdout);
  for (size_t i = 0; i < (5000 / 250); i++)
  {
    fputc('.', stdout);
    auto ch = (uint8_t)fgetc(stdin);
    // SPACE BAR
    if (ch == 32)
    {
      TerminalBasedWifiSetup();
    }
    delay(250);
  }
  fputs("\n\nNo key press detected\n", stdout);
}

// Mark running firmware as ok/valid.
// Believe this is also duplicated into ArduinoESP32 library/runtime
extern "C" void confirmFirmwareIsValid()
{
  const esp_partition_t *running = esp_ota_get_running_partition();
  esp_ota_img_states_t ota_state;
  if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK)
  {
    if (ota_state == ESP_OTA_IMG_PENDING_VERIFY)
    {
      // Validate image some how, then call:
      ESP_LOGI(TAG, "esp_ota_mark_app_valid_cancel_rollback");
      esp_ota_mark_app_valid_cancel_rollback();
      // If needed: esp_ota_mark_app_invalid_rollback_and_reboot();
    }
  }
}

void resumeTasksAfterFirmwareUpdateFailure()
{
  vTaskResume(updatetftdisplay_task_handle);
  vTaskResume(avrprog_task_handle);
  vTaskResume(sdcardlog_task_handle);
  vTaskResume(sdcardlog_outputs_task_handle);
  vTaskResume(rs485_tx_task_handle);
  vTaskResume(service_rs485_transmit_q_task_handle);
  vTaskResume(canbus_tx_task_handle);
  vTaskResume(transmit_task_handle);
  vTaskResume(lazy_task_handle);
  vTaskResume(canbus_rx_task_handle);
}
void suspendTasksDuringFirmwareUpdate()
{
  vTaskSuspend(updatetftdisplay_task_handle);
  vTaskSuspend(avrprog_task_handle);
  vTaskSuspend(sdcardlog_task_handle);
  vTaskSuspend(sdcardlog_outputs_task_handle);
  vTaskSuspend(rs485_tx_task_handle);
  vTaskSuspend(service_rs485_transmit_q_task_handle);
  vTaskSuspend(canbus_tx_task_handle);
  vTaskSuspend(transmit_task_handle);
  vTaskSuspend(lazy_task_handle);
  vTaskSuspend(canbus_rx_task_handle);
}

void setup()
{
  esp_chip_info_t chip_info;

  // Configure log levels
  for (auto log : log_levels)
  {
    esp_log_level_set(log.tag, log.level);
  }

  esp_chip_info(&chip_info);

  ESP_LOGI(TAG, R"(


               _          __ 
  _|  o       |_)  |\/|  (_  
 (_|  |  \/   |_)  |  |  __)
         /

CONTROLLER - ver:%s compiled %s
ESP32 Chip model = %u, Rev %u, Cores=%u, Features=%u)",
           GIT_VERSION, COMPILE_DATE_TIME,
           chip_info.model, chip_info.revision, chip_info.cores, chip_info.features);

  esp_core_dump_init();

  BuildHostname();
  hal.ConfigurePins();
  hal.ConfigureI2C(TCA6408Interrupt, TCA9534AInterrupt, TCA6416AInterrupt);
  hal.ConfigureVSPI();

  confirmFirmwareIsValid();

  _avrsettings.inProgress = false;
  _avrsettings.programmingModeEnabled = false;

  // See if we can get a sensible reading from the TFT touch chip XPT2046
  // if we can, then a screen is fitted, so enable it
  // Touch is on VSPI bus
  _tft_screen_available = hal.IsScreenAttached();
  SetControllerState(ControllerState::PowerUp);
  init_tft_display();
  hal.Led(0);

  InitializeNVS();

  if (!LittleFS.begin(false))
  {
    ESP_LOGE(TAG, "LittleFS mount failed, did you upload file system image?");
    hal.Halt(RGBLED::White);
  }

  ESP_LOGI(TAG, "LittleFS mounted, total=%u, used=%u", LittleFS.totalBytes(), LittleFS.usedBytes());

  mountSDCard();

  // consoleConfigurationCheck needs to be after SD card mount, as it attempts to remove wifi.json if it exists
  consoleConfigurationCheck();

  // Retrieve the EEPROM WIFI settings
  bool EepromConfigValid = LoadWiFiConfig();

  // Now compare the WIFI settings with any potential JSON files on the SDCARD (those override)
  if (LoadWiFiConfigFromSDCard(EepromConfigValid))
  {
    // We need to reload the configuration, as it was updated...
    EepromConfigValid = LoadWiFiConfig();
  }

  // Pre configure the array
  memset(&cmi, 0, sizeof(cmi));
  for (uint8_t i = 0; i < maximum_controller_cell_modules; i++)
  {
    clearModuleValues(i);
  }

  history.Clear();

  rules.resetAllRules();

  LoadConfiguration(&mysettings);
  ValidateConfiguration(&mysettings);

  if (strlen(mysettings.homeassist_apikey) == 0)
  {
    // Generate new key
    memset(&mysettings.homeassist_apikey, 0, sizeof(mysettings.homeassist_apikey));
    randomCharacters(mysettings.homeassist_apikey, sizeof(mysettings.homeassist_apikey) - 1);    
    saveConfiguration();
  }
  ESP_LOGI(TAG, "homeassist_apikey=%s", mysettings.homeassist_apikey);

  if (!EepromConfigValid)
  {
    // We don't have a valid WIFI configuration, so force terminal based setup
    TerminalBasedWifiSetup();
  }

  // Check and configure internal current monitor (if it exists)
  if (hal.GetVSPIMutex())
  {
    if (currentmon_internal.Initialise(hal.VSPI_Ptr(), INA229_CHIPSELECT))
    {
      ESP_LOGI(TAG, "Onboard/internal current monitoring chip available");

      currentmon_internal.Configure(
          mysettings.currentMonitoring_shuntmv,
          mysettings.currentMonitoring_shuntmaxcur,
          mysettings.currentMonitoring_batterycapacity,
          mysettings.currentMonitoring_fullchargevolt,
          mysettings.currentMonitoring_tailcurrent,
          mysettings.currentMonitoring_chargeefficiency,
          mysettings.currentMonitoring_shuntcal,
          mysettings.currentMonitoring_temperaturelimit,
          mysettings.currentMonitoring_overvoltagelimit,
          mysettings.currentMonitoring_undervoltagelimit,
          mysettings.currentMonitoring_overcurrentlimit,
          mysettings.currentMonitoring_undercurrentlimit,
          mysettings.currentMonitoring_overpowerlimit,
          mysettings.currentMonitoring_shunttempcoefficient,
          mysettings.currentMonitoring_tempcompenabled);

      currentmon_internal.GuessSOC();

      currentmon_internal.TakeReadings();
    }
    else
    {
      ESP_LOGI(TAG, "Onboard/internal current monitoring chip not installed");
    }
    hal.ReleaseVSPIMutex();
  }

  rules.setChargingMode(ChargingMode::standard);

  // Switch CAN chip TJA1051T/3 ON
  hal.CANBUSEnable(true);
  hal.ConfigureCAN(mysettings.canbusbaud);

  // Serial pins IO2/IO32
  SERIAL_DATA.begin(mysettings.baudRate, SERIAL_8N1, 2, 32); // Serial for comms to modules

  myPacketSerial.begin(&SERIAL_DATA, &onPacketReceived, sizeof(PacketStruct), SerialPacketReceiveBuffer, sizeof(SerialPacketReceiveBuffer));

  SetupRS485();

  // Create queue for transmit, each request could be MAX_SEND_RS485_PACKET_LENGTH bytes long, depth of 3 items
  rs485_transmit_q_handle = xQueueCreate(3, MAX_SEND_RS485_PACKET_LENGTH);
  assert(rs485_transmit_q_handle);

  request_q_handle = xQueueCreate(30, sizeof(PacketStruct));
  assert(request_q_handle);
  prg.setQueueHandle(request_q_handle);

  reply_q_handle = xQueueCreate(4, sizeof(PacketStruct));
  assert(reply_q_handle);

  led_off_timer = xTimerCreate("LEDOFF", pdMS_TO_TICKS(100), pdFALSE, (void *)1, &ledoff);
  assert(led_off_timer);

  pulse_relay_off_timer = xTimerCreate("PULSE", pdMS_TO_TICKS(250), pdFALSE, (void *)2, &pulse_relay_off);
  assert(pulse_relay_off_timer);

  tftwake_timer = xTimerCreate("TFTWAKE", pdMS_TO_TICKS(50), pdFALSE, (void *)3, &tftwakeup);
  assert(tftwake_timer);

  xTaskCreate(voltageandstatussnapshot_task, "snap", 1950, nullptr, 1, &voltageandstatussnapshot_task_handle);
  // Increased tftupd from 2000 to 2450 due to strange bug on rev3 ESP32 chips
  xTaskCreate(updatetftdisplay_task, "tftupd", 2450, nullptr, 0, &updatetftdisplay_task_handle);
  xTaskCreate(avrprog_task, "avrprog", 2450, &_avrsettings, configMAX_PRIORITIES - 3, &avrprog_task_handle);

  // High priority task
  xTaskCreate(interrupt_task, "int", 2050, nullptr, configMAX_PRIORITIES - 1, &interrupt_task_handle);
  xTaskCreate(sdcardlog_task, "sdlog", 3800, nullptr, 0, &sdcardlog_task_handle);
  xTaskCreate(sdcardlog_outputs_task, "sdout", 3200, nullptr, 0, &sdcardlog_outputs_task_handle);
  xTaskCreate(rule_state_change_task, "r_stat", 3000, nullptr, 0, &rule_state_change_task_handle);

  xTaskCreate(rs485_tx, "485_TX", 2940, nullptr, 1, &rs485_tx_task_handle);
  xTaskCreate(rs485_rx, "485_RX", 2940, nullptr, 1, &rs485_rx_task_handle);
  xTaskCreate(service_rs485_transmit_q, "485_Q", 2950, nullptr, 1, &service_rs485_transmit_q_task_handle);
  xTaskCreate(canbus_tx, "CAN_Tx", 4096, nullptr, 1, &canbus_tx_task_handle);
  xTaskCreate(canbus_rx, "CAN_Rx", 2950, nullptr, 1, &canbus_rx_task_handle);
  xTaskCreate(transmit_task, "Tx", 1950, nullptr, configMAX_PRIORITIES - 3, &transmit_task_handle);
  xTaskCreate(replyqueue_task, "rxq", 4096, nullptr, configMAX_PRIORITIES - 2, &replyqueue_task_handle);
  xTaskCreate(lazy_tasks, "lazyt", 2500, nullptr, 0, &lazy_task_handle);

  // Set relay defaults
  for (auto y = 0; y < RELAY_TOTAL; y++)
  {
    previousRelayState[y] = mysettings.rulerelaydefault[y];
    // Set relay defaults
    hal.SetOutputState(y, mysettings.rulerelaydefault[y]);
  }
  // Fire task to record state of outputs to SD Card
  xTaskNotify(sdcardlog_outputs_task_handle, 0x00, eNotifyAction::eNoAction);

  // We have just started...
  SetControllerState(ControllerState::Stabilizing);

  // Only run these after we have wifi...
  xTaskCreate(enqueue_task, "enqueue", 1580, nullptr, configMAX_PRIORITIES / 2, &enqueue_task_handle);
  xTaskCreate(rules_task, "rules", 3400, nullptr, configMAX_PRIORITIES - 5, &rule_task_handle);
  xTaskCreate(periodic_task, "period", 2850, nullptr, 0, &periodic_task_handle);

  // Start the wifi and connect to access point
  wifi_init_sta();

  if (_tft_screen_available)
  {
    ESP_LOGI(TAG, "TFT screen is INSTALLED");
    // Only attach, if device is fitted otherwise false triggers may occur
    // Touch screen IRQ (GPIO_NUM_36) is active LOW (XPT2046 chip)
    ESP_ERROR_CHECK(esp_ipc_call_blocking(PRO_CPU_NUM, tft_interrupt_attach, nullptr));
  }
  else
  {
    ESP_LOGI(TAG, "TFT screen is NOT installed");
  }

  // Force Refresh the TFT display
  wake_up_tft(true);
}

/// @brief Convert an ESP32 core dump stored in FLASH to a JSON object fragment
/// @param doc
void ESPCoreDumpToJSON(JsonObject &doc)
{
  if (esp_core_dump_image_check() == ESP_OK)
  {
    JsonObject core = doc.createNestedObject("coredump");
    // A valid core dump is in FLASH storage

    esp_core_dump_summary_t *summary = (esp_core_dump_summary_t *)malloc(sizeof(esp_core_dump_summary_t));
    if (summary)
    {
      if (esp_core_dump_get_summary(summary) == ESP_OK)
      {
        char outputString[16];

        core["exc_task"] = summary->exc_task;
        core["app_elf_sha256"] = (char *)summary->app_elf_sha256;
        core["dumpver"] = summary->core_dump_version;

        ultoa(summary->exc_pc, outputString, 16);
        core["exc_pc"] = outputString;

        ultoa(summary->exc_tcb, outputString, 16);
        core["exc_tcb"] = outputString;

        core["bt_corrupted"] = summary->exc_bt_info.corrupted;
        core["bt_depth"] = summary->exc_bt_info.depth;
        auto backtrace = core.createNestedArray("backtrace");
        for (auto value : summary->exc_bt_info.bt)
        {
          ultoa(value, outputString, 16);
          backtrace.add(outputString);
        }

        ltoa(summary->ex_info.epcx_reg_bits, outputString, 2);
        core["epcx_reg_bits"] = outputString;
        ltoa(summary->ex_info.exc_cause, outputString, 16);
        core["exc_cause"] = outputString;
        ultoa(summary->ex_info.exc_vaddr, outputString, 16);
        core["exc_vaddr"] = outputString;

        auto exc_a = core.createNestedArray("exc_a");
        for (auto value : summary->ex_info.exc_a)
        {
          ultoa(value, outputString, 16);
          exc_a.add(outputString);
        }
        auto epcx = core.createNestedArray("epcx");
        for (auto value : summary->ex_info.epcx)
        {
          ultoa(value, outputString, 16);
          epcx.add(outputString);
        }
      }
    }
    free(summary);
  }
}

/// @brief Generates a JSON document with diagnostic information about the running system
/// @param req
/// @param buffer
/// @param bufferLenMax
/// @return
esp_err_t diagnosticJSON(httpd_req_t *req, char buffer[], int bufferLenMax)
{
  DynamicJsonDocument doc(2048);
  JsonObject root = doc.to<JsonObject>();
  JsonObject diag = root.createNestedObject("diagnostic");

  diag["numtasks"] = uxTaskGetNumberOfTasks();
  auto tasks = diag.createNestedArray("tasks");

  // Array of pointers to the task handles we are going to examine
  const std::array<TaskHandle_t *, 18> task_handle_ptrs =
      {&sdcardlog_task_handle, &sdcardlog_outputs_task_handle, &rule_state_change_task_handle,
       &avrprog_task_handle, &enqueue_task_handle, &transmit_task_handle, &replyqueue_task_handle,
       &lazy_task_handle, &rule_task_handle, &voltageandstatussnapshot_task_handle, &updatetftdisplay_task_handle,
       &periodic_task_handle, &interrupt_task_handle, &rs485_tx_task_handle,
       &rs485_rx_task_handle, &service_rs485_transmit_q_task_handle,
       &canbus_tx_task_handle, &canbus_rx_task_handle};

  // Remember these are pointers to the handle
  for (auto h : task_handle_ptrs)
  {
    if (*h != nullptr)
    {
      JsonObject nested = tasks.createNestedObject();
      nested["name"] = pcTaskGetName(*h);
      nested["hwm"] = uxTaskGetStackHighWaterMark(*h);
    }
  };

  // Similar but for tasks we don't own - get handle by name (this is SLOW function call)
  // this is the actual handle value, not a pointer to it
  std::array<TaskHandle_t, 2> task_handles = {
      xTaskGetHandle("loopTask"),
      xTaskGetHandle("httpd")};

  for (auto h : task_handles)
  {
    if (h != nullptr)
    {
      JsonObject nested = tasks.createNestedObject();
      nested["name"] = pcTaskGetName(h);
      nested["hwm"] = uxTaskGetStackHighWaterMark(h);
    }
  };

  diag["FreeHeap"] = ESP.getFreeHeap();
  diag["MinFreeHeap"] = ESP.getMinFreeHeap();
  diag["HeapSize"] = ESP.getHeapSize();
  diag["SdkVersion"] = ESP.getSdkVersion();

  ESPCoreDumpToJSON(diag);

  int bufferused = 0;
  bufferused += serializeJson(doc, buffer, bufferLenMax);
  return httpd_resp_send(req, buffer, bufferused);
}

unsigned long wifitimer = 0;
unsigned long heaptimer = 0;
// unsigned long taskinfotimer = 0;

void logActualTime()
{
  if (sntp_enabled())
  {
    char strftime_buf[64];
    formatCurrentDateTime(strftime_buf, sizeof(strftime_buf));
    ESP_LOGI(TAG, "Time now: %s", strftime_buf);
  }
}

void loop()
{
  delay(100);

  unsigned long currentMillis = millis();

  if (card_action == CardAction::Mount)
  {
    mountSDCard();
  }
  if (card_action == CardAction::Unmount)
  {
    unmountSDCard();
  }

  if (card_action == CardAction::Remount)
  {
    unmountSDCard();
    mountSDCard();
  }

  // on first pass wifitimer is zero
  if (_controller_state != ControllerState::NoWifiConfiguration && currentMillis > wifitimer)
  {
    // Avoid triggering on the very first loop (causes ESP_ERR_WIFI_CONN warning)
    if (wifitimer > 0)
    {
      // Attempt to connect to WiFi every 30 seconds, this caters for when WiFi drops such as AP reboot
      if (wifi_isconnected)
      {
        // Attempt to connect to MQTT if enabled and not already connected
        connectToMqtt();
      }
      else
      {
        ESP_LOGI(TAG, "Trying to connect WIFI");
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_connect());
      }
    }
    // Wait another 30 seconds
    wifitimer = currentMillis + 30000;
  }

  // Call update to receive, decode and process incoming packets
  myPacketSerial.checkInputStream();

  if (currentMillis > heaptimer)
  {
    logActualTime();
    /*
     total_free_bytes;        Total free bytes in the heap. Equivalent to multi_free_heap_size().
     total_allocated_bytes;   Total bytes allocated to data in the heap.
     largest_free_block;      Size of largest free block in the heap. This is the largest malloc-able size.
     minimum_free_bytes;      Lifetime minimum free heap size. Equivalent to multi_minimum_free_heap_size().
     allocated_blocks;        Number of (variable size) blocks allocated in the heap.
     free_blocks;             Number of (variable size) free blocks in the heap.
     total_blocks;            Total number of (variable size) blocks in the heap.
    */

    multi_heap_info_t heap;
    heap_caps_get_info(&heap, MALLOC_CAP_INTERNAL);

    ESP_LOGD(TAG, "total_free_byte=%u total_allocated_byte=%u largest_free_blk=%u min_free_byte=%u alloc_blk=%u free_blk=%u total_blk=%u",
             heap.total_free_bytes,
             heap.total_allocated_bytes,
             heap.largest_free_block,
             heap.minimum_free_bytes,
             heap.allocated_blocks,
             heap.free_blocks,
             heap.total_blocks);

    // Report again in 30 seconds
    heaptimer = currentMillis + 30000;
  }
}
