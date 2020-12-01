/*

 ____  ____  _  _  ____  __  __  ___    _  _  __
(  _ \(_  _)( \/ )(  _ \(  \/  )/ __)  ( \/ )/. |
 )(_) )_)(_  \  /  ) _ < )    ( \__ \   \  /(_  _)
(____/(____) (__) (____/(_/\/\_)(___/    \/   (_)

  (c) 2017/18/19/20 Stuart Pittaway

  This is the code for the controller - it talks to the V4.X cell modules over isolated serial bus

  This code runs on ESP-8266 WEMOS D1 PRO and compiles with VS CODE and PLATFORM IO environment
*/
/*
*** NOTE IF YOU GET ISSUES WHEN COMPILING IN PLATFORM.IO ***
ERROR: "ESP Async WebServer\src\WebHandlers.cpp:67:64: error: 'strftime' was not declared in this scope"
Delete the file <project folder>\diyBMSv4\ESPController\.pio\libdeps\esp8266_d1minipro\Time\Time.h
The Time.h file in this library conflicts with the time.h file in the ESP core platform code

See reasons why here https://github.com/me-no-dev/ESPAsyncWebServer/issues/60
*/
/*
   ESP8266 PINS
   D0 = GREEN_LED
   D1 = i2c SDA
   D2 = i2c SCL
   D3 = switch to ground (reset WIFI configuration on power up)
   D4 = GPIO2 = TXD1 = TRANSMIT DEBUG SERIAL (and blue led on esp8266)
   D5 = GPIO14 = Interrupt in from PCF8574
   D7 = GPIO13 = RECEIVE SERIAL
   D8 = GPIO15 = TRANSMIT SERIAL

   DIAGRAM
   https://www.hackster.io/Aritro/getting-started-with-esp-nodemcu-using-arduinoide-aa7267
*/

// PacketSerial library takes 1691ms round trip with 8 modules, 212ms per module @ 2400baud

#include <Arduino.h>

#ifndef GIT_VERSION
#error GIT_VERSION not defined
#endif

//#define PACKET_LOGGING_RECEIVE
//#define PACKET_LOGGING_SEND
//#define RULES_LOGGING

#include "FS.h"

//Libraries just for ESP8266
#if defined(ESP8266)
#include <TimeLib.h>
#include <ESP8266WiFi.h>
#include <NtpClientLib.h>
#include <LittleFS.h>
#endif

//Libraries just for ESP32
#if defined(ESP32)
#include <SPIFFS.h>
#include <WiFi.h>
#include <SPI.h>
#include "time.h"
#include <esp_wifi.h>
#endif

//Shared libraries across processors
#include <Ticker.h>
#include <ESPAsyncWebServer.h>
#include <AsyncMqttClient.h>
#include <SerialEncoder.h>
#include <cppQueue.h>

#include "defines.h"

#include <ArduinoOTA.h>

#if defined(ESP8266)
#include "HAL_ESP8266.h"
HAL_ESP8266 hal;
#endif

#if defined(ESP32)
#include "HAL_ESP32.h"
HAL_ESP32 hal;
#endif

#include "Rules.h"

volatile bool emergencyStop = false;

Rules rules;

diybms_eeprom_settings mysettings;
uint16_t ConfigHasChanged = 0;

bool server_running = false;
RelayState previousRelayState[RELAY_TOTAL];
bool previousRelayPulse[RELAY_TOTAL];

volatile enumInputState InputState[INPUTS_TOTAL];

#if defined(ESP8266)
bool NTPsyncEventTriggered = false; // True if a time even has been triggered
NTPSyncEvent_t ntpEvent;            // Last triggered event
#endif

AsyncWebServer server(80);

#if defined(ESP32)
TaskHandle_t i2c_task_handle;
TaskHandle_t ledoff_task_handle;
QueueHandle_t queue_i2c;

void QueueLED(uint8_t bits)
{
  i2cQueueMessage m;
  //3 = LED
  m.command = 0x03;
  //Lowest 3 bits are RGB led GREEN/RED/BLUE
  m.data = bits & B00000111;
  xQueueSendToBack(queue_i2c, &m, 10 / portTICK_PERIOD_MS);
}

void ledoff_task(void *param)
{
  while (true)
  {
    //Wait until this task is triggered https://www.freertos.org/ulTaskNotifyTake.html
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    //Wait 100ms
    vTaskDelay(100 / portTICK_PERIOD_MS);
    //LED OFF
    QueueLED(0);
  }
}
// Handle all calls to i2c devices in this single task
// Provides thread safe mechanism to talk to i2c
void i2c_task(void *param)
{
  while (true)
  {
    i2cQueueMessage m;

    if (xQueueReceive(queue_i2c, &m, portMAX_DELAY) == pdPASS)
    {
      // do some i2c task
      if (m.command == 0x01)
      {
        // Read ports A/B inputs (on TCA6408)
        uint8_t v = hal.ReadTCA6408InputRegisters();
        InputState[0] = (v & B00000001) == 0 ? enumInputState::INPUT_LOW : enumInputState::INPUT_HIGH;
        InputState[1] = (v & B00000010) == 0 ? enumInputState::INPUT_LOW : enumInputState::INPUT_HIGH;

        //Emergency Stop (J1) has triggered
        if (InputState[0] == enumInputState::INPUT_HIGH)
        {
          emergencyStop = true;
        }
      }

      if (m.command == 0x02)
      {
        //Read ports
        //The 9534 deals with internal LED outputs and spare IO on J10
        //P5/P6/P7 = EXTRA I/O (on internal header breakout pins J10)
        uint8_t v = hal.ReadTCA9534InputRegisters();
        InputState[2] = (v & B00100000) == 0 ? enumInputState::INPUT_LOW : enumInputState::INPUT_HIGH;
        InputState[3] = (v & B01000000) == 0 ? enumInputState::INPUT_LOW : enumInputState::INPUT_HIGH;
        InputState[4] = (v & B10000000) == 0 ? enumInputState::INPUT_LOW : enumInputState::INPUT_HIGH;
      }

      if (m.command == 0x03)
      {
        hal.Led(m.data);
      }

      if (m.command >= 0xE0 && m.command <= 0xE0 + RELAY_TOTAL)
      {
        //Set state of relays
        hal.SetOutputState(m.command - 0xe0, (RelayState)m.data);
      }
    }
  }
}

void IRAM_ATTR TCA6408Interrupt()
{
  i2cQueueMessage m;
  m.command = 0x01;
  m.data = 0;
  xQueueSendToBackFromISR(queue_i2c, &m, NULL);
}

void IRAM_ATTR TCA9534AInterrupt()
{
  i2cQueueMessage m;
  m.command = 0x02;
  m.data = 0;
  xQueueSendToBackFromISR(queue_i2c, &m, NULL);
}
#endif

#if defined(ESP8266)
void IRAM_ATTR ExternalInputInterrupt()
{
  if ((hal.ReadInputRegisters() & B00010000) == 0)
  {
    //Emergency Stop (J1) has triggered
    emergencyStop = true;
  }
}
#endif

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
cppQueue requestQueue(sizeof(PacketStruct), 16, FIFO);

PacketRequestGenerator prg = PacketRequestGenerator(&requestQueue);

PacketReceiveProcessor receiveProc = PacketReceiveProcessor();

// Memory to hold in and out serial buffer
uint8_t SerialPacketReceiveBuffer[2 * sizeof(PacketStruct)];

SerialEncoder myPacketSerial;

#if defined(ESP8266)
WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
#endif

Ticker myTimerRelay;
Ticker myTimer;
Ticker myTransmitTimer;
Ticker myLazyTimer;
Ticker wifiReconnectTimer;
Ticker mqttReconnectTimer;
Ticker myTimerSendMqttPacket;
Ticker myTimerSendInfluxdbPacket;
Ticker myTimerSwitchPulsedRelay;

uint16_t sequence = 0;

ControllerState ControlState;

bool OutputsEnabled;
bool InputsEnabled;

AsyncMqttClient mqttClient;

void dumpPacketToDebug(PacketStruct *buffer)
{
  SERIAL_DEBUG.print(buffer->start_address, HEX);
  SERIAL_DEBUG.print('-');
  SERIAL_DEBUG.print(buffer->end_address, HEX);
  SERIAL_DEBUG.print('/');
  SERIAL_DEBUG.print(buffer->hops, HEX);
  SERIAL_DEBUG.print('/');
  SERIAL_DEBUG.print(buffer->command, HEX);

  if (buffer->command == ReadVoltageAndStatus)
  {
    SERIAL_DEBUG.print(F(" ReadVoltageStatus "));
  }
  if (buffer->command == ReadTemperature)
  {
    SERIAL_DEBUG.print(F(" ReadTemperature "));
  }
  if (buffer->command == ReadSettings)
  {
    SERIAL_DEBUG.print(F(" ReadSettings "));
  }

  SERIAL_DEBUG.print('/');
  SERIAL_DEBUG.print(buffer->sequence, HEX);
  SERIAL_DEBUG.print('=');
  for (size_t i = 0; i < maximum_cell_modules_per_packet; i++)
  {
    SERIAL_DEBUG.print(buffer->moduledata[i], HEX);
    SERIAL_DEBUG.print(" ");
  }
  SERIAL_DEBUG.print(" =");
  SERIAL_DEBUG.print(buffer->crc, HEX);
}

void SetControllerState(ControllerState newState)
{
  if (ControlState != newState)
  {
    ControlState = newState;

    SERIAL_DEBUG.println("");
    SERIAL_DEBUG.print(F("** Controller changed to state = "));
    SERIAL_DEBUG.println(newState, HEX);
  }
}

uint16_t minutesSinceMidnight()
{

#if defined(ESP8266)
  return (hour() * 60) + minute();
#endif

#if defined(ESP32)
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo))
  {
    return 0;
  }
  else
  {
    return (timeinfo.tm_hour * 60) + timeinfo.tm_min;
  }
#endif
}

#if defined(ESP8266)
void processSyncEvent(NTPSyncEvent_t ntpEvent)
{
  if (ntpEvent < 0)
  {
    SERIAL_DEBUG.printf("Time Sync error: %d\n", ntpEvent);
    if (ntpEvent == noResponse)
      SERIAL_DEBUG.println(F("NTP server not reachable"));
    else if (ntpEvent == invalidAddress)
      SERIAL_DEBUG.println(F("Invalid NTP server address"));
    else if (ntpEvent == errorSending)
      SERIAL_DEBUG.println(F("Error sending request"));
    else if (ntpEvent == responseError)
      SERIAL_DEBUG.println(F("NTP response error"));
  }
  else
  {
    if (ntpEvent == timeSyncd)
    {
      SERIAL_DEBUG.print(F("Got NTP time"));
      time_t lastTime = NTP.getLastNTPSync();
      SERIAL_DEBUG.println(NTP.getTimeDateString(lastTime));
      setTime(lastTime);
    }
  }
}
#endif

void onPacketReceived()
{
#if defined(ESP8266)
  hal.GreenLedOn();
#endif

#if defined(PACKET_LOGGING_RECEIVE)
  // Process decoded incoming packet
  SERIAL_DEBUG.print("R:");
  dumpPacketToDebug((PacketStruct *)SerialPacketReceiveBuffer);
#endif

  if (receiveProc.ProcessReply((PacketStruct *)SerialPacketReceiveBuffer))
  {
#if defined(ESP32)
    QueueLED(B00000100);
#endif
  }
  else
  {
#if defined(ESP32)
    //Error blue
    QueueLED(B00000001);
#endif
    SERIAL_DEBUG.print(F("**FAIL PROCESS REPLY**"));
  }

#if defined(PACKET_LOGGING_RECEIVE)
  SERIAL_DEBUG.println("");
  //SERIAL_DEBUG.print("Timing:");SERIAL_DEBUG.print(receiveProc.packetTimerMillisecond);SERIAL_DEBUG.println("ms");
#endif

#if defined(ESP8266)
  hal.GreenLedOff();
#else
  //Fire task to switch off LED in 100ms
  xTaskNotify(ledoff_task_handle, 0x00, eNotifyAction::eNoAction);
#endif
}

void timerTransmitCallback()
{
  // Called to transmit the next packet in the queue need to ensure this procedure is called more frequently than
  // items are added into the queue
  if (!requestQueue.isEmpty())
  {
    PacketStruct transmitBuffer;

    //Wake up the connected cell module from sleep
    //SERIAL_DATA.write(framingmarker);
    myPacketSerial.sendStartFrame();
    delay(3);

    requestQueue.pop(&transmitBuffer);
    sequence++;
    transmitBuffer.sequence = sequence;

    transmitBuffer.crc = CRC16::CalculateArray((uint8_t *)&transmitBuffer, sizeof(PacketStruct) - 2);
    myPacketSerial.sendBuffer((byte *)&transmitBuffer); //, sizeof(transmitBuffer));

    //Grab the time we sent this packet to time how long packets take to move
    //through the modules.  We only time the COMMAND::ReadVoltageAndStatus packets
    if (transmitBuffer.command == COMMAND::ReadVoltageAndStatus)
    {
      receiveProc.packetLastSentSequence = sequence;
      receiveProc.packetLastSentMillisecond = millis();
    }

    // Output the packet we just transmitted to debug console
#if defined(PACKET_LOGGING_SEND)
    SERIAL_DEBUG.print("S:");
    dumpPacketToDebug(&transmitBuffer);
    SERIAL_DEBUG.print("/Q:");
    SERIAL_DEBUG.println(requestQueue.getCount());
#endif
  }
}

//Runs the rules and populates rule_outcome array with true/false for each rule
//Rules based on module parameters/readings like voltage and temperature
//are only processed once every module has returned at least 1 reading/communication
void ProcessRules()
{
  rules.ClearValues();

  InternalErrorCode newErrCode = InternalErrorCode::NoError;
  InternalWarningCode newWarnCode = InternalWarningCode::NoWarning;

  uint16_t totalConfiguredModules = mysettings.totalNumberOfBanks * mysettings.totalNumberOfSeriesModules;
  if (totalConfiguredModules > maximum_controller_cell_modules)
  {
    //System is configured with more than maximum modules - abort!
    newErrCode = InternalErrorCode::TooManyModules;
  }

  if (receiveProc.totalModulesFound != totalConfiguredModules)
  {
    //Found more or less modules than configured for
    newErrCode = InternalErrorCode::ModuleCountMismatch;
  }

  if (rules.invalidModuleCount > 0)
  {
    //Some modules are not yet valid
    newErrCode = InternalErrorCode::WaitingForModulesToReply;
  }

  //Communications error...
  if (receiveProc.HasCommsTimedOut())
  {
    newErrCode = InternalErrorCode::CommunicationsError;
  }

  if (ControlState == ControllerState::Running && rules.zeroVoltageModuleCount > 0)
  {
    newErrCode = InternalErrorCode::ZeroVoltModule;
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
          newWarnCode = InternalWarningCode::ModuleInconsistantBypassVoltage;
        }
        else if (cmi[cellid].BypassOverTempShutdown != mysettings.BypassOverTempShutdown)
        {
          newWarnCode = InternalWarningCode::ModuleInconsistantBypassTemperature;
        }
      }

      cellid++;
    }
    rules.ProcessBank(bank);
  }

  rules.SetError(newErrCode);
  rules.SetWarning(newWarnCode);

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

#if defined(ESP32)
  if (emergencyStop)
  {
    //Lowest 3 bits are RGB led GREEN/RED/BLUE
    QueueLED(B00000011);
  }
#endif
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
#if defined(ESP8266)
      hal.SetOutputState(y, previousRelayState[y] == RelayState::RELAY_ON ? RelayState::RELAY_OFF : RelayState::RELAY_ON);
#endif

#if defined(ESP32)
      i2cQueueMessage m;
      //Different command for each relay
      m.command = 0xE0 + y;
      m.data = previousRelayState[y] == RelayState::RELAY_ON ? RelayState::RELAY_OFF : RelayState::RELAY_ON;
      xQueueSendToBack(queue_i2c, &m, 10 / portTICK_PERIOD_MS);
#endif

      previousRelayPulse[y] = false;
    }
  }

  //This only fires once
  myTimerSwitchPulsedRelay.detach();
}

void timerProcessRules()
{

  //Run the rules
  ProcessRules();

#if defined(RULES_LOGGING)
  SERIAL_DEBUG.print(F("Rules:"));
  for (int8_t r = 0; r < RELAY_RULES; r++)
  {
    SERIAL_DEBUG.print(rules.rule_outcome[r]);
  }
  SERIAL_DEBUG.print("=");
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

  //Perhaps we should publish the relay settings over MQTT and INFLUX/website?
  for (int8_t n = 0; n < RELAY_TOTAL; n++)
  {
    if (previousRelayState[n] != relay[n])
    {
      //Would be better here to use the WRITE8 to lower i2c traffic
#if defined(RULES_LOGGING)
      SERIAL_DEBUG.print(F("Relay:"));
      SERIAL_DEBUG.print(n);
      SERIAL_DEBUG.print("=");
      SERIAL_DEBUG.print(relay[n]);
#endif
      //hal.SetOutputState(n, relay[n]);

      //This would be better if we worked out the bit pattern first and then just
      //submitted that as a single i2c read/write transaction

#if defined(ESP8266)
      hal.SetOutputState(n, relay[n]);
#endif

#if defined(ESP32)
      i2cQueueMessage m;
      //Different command for each relay
      m.command = 0xE0 + n;
      m.data = relay[n];
      xQueueSendToBack(queue_i2c, &m, 10 / portTICK_PERIOD_MS);
#endif

      previousRelayState[n] = relay[n];

      if (mysettings.relaytype[n] == RELAY_PULSE)
      {
        //If its a pulsed relay, invert the output quickly via a single shot timer
        previousRelayPulse[n] = true;
        myTimerSwitchPulsedRelay.attach(0.1, timerSwitchPulsedRelay);
#if defined(RULES_LOGGING)
        SERIAL_DEBUG.print("P");
#endif
      }
    }
  }
#if defined(RULES_LOGGING)
  SERIAL_DEBUG.println("");
#endif
}

uint8_t counter = 0;

void timerEnqueueCallback()
{
  //this is called regularly on a timer, it determines what request to make to the modules (via the request queue)
  uint8_t b = 0;

  uint8_t max = mysettings.totalNumberOfBanks * mysettings.totalNumberOfSeriesModules;
  uint8_t startmodule = 0;
  //uint8_t endmodule=maximum_cell_modules_per_packet;
  while (b < max)
  {
    uint8_t endmodule = (startmodule + maximum_cell_modules_per_packet) - 1;

    //Limit to number of modules we have configured
    if (endmodule > max)
    {
      endmodule = max - 1;
    }

    prg.sendCellVoltageRequest(startmodule, endmodule);
    prg.sendCellTemperatureRequest(startmodule, endmodule);

    //If any module is in bypass then request PWM reading for whole bank
    for (uint8_t m = startmodule; m < endmodule; m++)
    {
      if (cmi[m].inBypass)
      {
        prg.sendReadBalancePowerRequest(startmodule, endmodule);
        break;
      }
    }

    //Every 50 loops also ask for bad packet count (saves battery power if we dont ask for this all the time)
    if (counter % 50 == 0)
    {
      prg.sendReadBadPacketCounter(startmodule, endmodule);
    }

    startmodule = endmodule + 1;
    b += maximum_cell_modules_per_packet;
  }

  //It's an unsigned byte, let it overflow to reset
  counter++;
}

void connectToWifi()
{
  SERIAL_DEBUG.println(F("Connecting to Wi-Fi..."));
  WiFi.mode(WIFI_STA);
#if defined(ESP8266)
  //Serial.printf(" ESP8266 Chip id = %08X\n", ESP.getChipId());
  wifi_station_set_hostname("diyBMSESP8266");
  WiFi.hostname("diyBMSESP8266");
#endif
#if defined(ESP32)
  WiFi.setHostname("diyBMSESP32");
#endif
  WiFi.begin(DIYBMSSoftAP::WifiSSID(), DIYBMSSoftAP::WifiPassword());
}

void connectToMqtt()
{
  SERIAL_DEBUG.println(F("Connecting to MQTT..."));
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
    SERIAL_DEBUG.println(F("Connect Error"));
    aClient = NULL;
    delete client;
  },
                   NULL);

  aClient->onConnect([](void *arg, AsyncClient *client) {
    SERIAL_DEBUG.println(F("Connected"));

    //Send the packet here

    aClient->onError(NULL, NULL);

    client->onDisconnect([](void *arg, AsyncClient *c) {
      SERIAL_DEBUG.println(F("Disconnected"));
      aClient = NULL;
      delete c;
    },
                         NULL);

    client->onData([](void *arg, AsyncClient *c, void *data, size_t len) {
      //Data received
      SERIAL_DEBUG.print(F("\r\nData: "));
      SERIAL_DEBUG.println(len);
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
  },
                     NULL);
}

void SendInfluxdbPacket()
{
  if (!mysettings.influxdb_enabled)
    return;

  SERIAL_DEBUG.println(F("SendInfluxdbPacket"));

  setupInfluxClient();

  if (!aClient->connect(mysettings.influxdb_host, mysettings.influxdb_httpPort))
  {
    SERIAL_DEBUG.println(F("Influxdb connect fail"));
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
  //ArduinoOTA.setHostname("diybmsesp32");
  ArduinoOTA.setPassword("1jiOOx12AQgEco4e");

  ArduinoOTA
      .onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
          type = "sketch";
        else // U_SPIFFS
          type = "filesystem";

        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
        SERIAL_DEBUG.println("Start updating " + type);
      });
  ArduinoOTA.onEnd([]() {
    SERIAL_DEBUG.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    SERIAL_DEBUG.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    SERIAL_DEBUG.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR)
      SERIAL_DEBUG.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR)
      SERIAL_DEBUG.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR)
      SERIAL_DEBUG.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR)
      SERIAL_DEBUG.println("Receive Failed");
    else if (error == OTA_END_ERROR)
      SERIAL_DEBUG.println("End Failed");
  });

  ArduinoOTA.begin();
}
#if defined(ESP8266)
void onWifiConnect(const WiFiEventStationModeGotIP &event)
{
#else
void onWifiConnect(WiFiEvent_t event, WiFiEventInfo_t info)
{
#endif

  SERIAL_DEBUG.print(F("DIYBMS Wi-Fi, "));
  SERIAL_DEBUG.print(WiFi.status());
  SERIAL_DEBUG.print(F(". Connected IP:"));
  SERIAL_DEBUG.println(WiFi.localIP());

  SERIAL_DEBUG.print(F("Requesting NTP from "));
  SERIAL_DEBUG.println(mysettings.ntpServer);

#if defined(ESP8266)
  //Update time every 10 minutes
  NTP.setInterval(600);
  NTP.setNTPTimeout(NTP_TIMEOUT);
  // String ntpServerName, int8_t timeZone, bool daylight, int8_t minutes, AsyncUDP* udp_conn
  NTP.begin(mysettings.ntpServer, mysettings.timeZone, mysettings.daylight, mysettings.minutesTimeZone);
#endif

#if defined(ESP32)
  //Use native ESP32 code
  configTime(mysettings.minutesTimeZone * 60, mysettings.daylight * 60, mysettings.ntpServer);
#endif

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
    DIYBMSServer::StartServer(&server);
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
}

#if defined(ESP8266)
void onWifiDisconnect(const WiFiEventStationModeDisconnected &event)
{
#else
void onWifiDisconnect(WiFiEvent_t event, WiFiEventInfo_t info)
{
#endif
  SERIAL_DEBUG.println(F("Disconnected from Wi-Fi."));
  // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
  mqttReconnectTimer.detach();
  myTimerSendMqttPacket.detach();
  myTimerSendInfluxdbPacket.detach();

  wifiReconnectTimer.once(2, connectToWifi);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
  SERIAL_DEBUG.println(F("Disconnected from MQTT."));

  myTimerSendMqttPacket.detach();

  if (WiFi.isConnected())
  {
    mqttReconnectTimer.once(2, connectToMqtt);
  }
}

//Send a few MQTT packets and keep track so we send the next batch on following calls
uint8_t mqttStartModule = 0;
void sendMqttPacket()
{
  if (!mysettings.mqtt_enabled && !mqttClient.connected())
    return;

  //SERIAL_DEBUG.println("Sending MQTT");

  char topic[80];
  char jsonbuffer[100];

  uint8_t counter = 0;

  for (uint8_t i = mqttStartModule; i < mysettings.totalNumberOfSeriesModules * mysettings.totalNumberOfBanks; i++)
  {

    //Only send valid module data
    if (cmi[i].valid)
    {
      uint8_t bank = i / mysettings.totalNumberOfSeriesModules;
      uint8_t module = i - (bank * mysettings.totalNumberOfSeriesModules);

      StaticJsonDocument<100> doc;
      doc["voltage"] = (float)cmi[i].voltagemV / 1000.0;
      doc["inttemp"] = cmi[i].internalTemp;
      doc["exttemp"] = cmi[i].externalTemp;
      doc["bypass"] = cmi[i].inBypass ? 1 : 0;
      serializeJson(doc, jsonbuffer, sizeof(jsonbuffer));

      sprintf(topic, "%s/%d/%d", mysettings.mqtt_topic, bank, module);

      //SERIAL_DEBUG.print("Sending MQTT - ");
      //SERIAL_DEBUG.println(topic);

      mqttClient.publish(topic, 0, false, jsonbuffer);
      //SERIAL_DEBUG.println(topic);
    }

    counter++;

    //After transmitting this many packets over MQTT, store our current state and exit the function.
    //this prevents flooding the ESP controllers wifi stack and potentially causing reboots/fatal exceptions
    if (counter == 6)
    {
      mqttStartModule = i + 1;

      if (mqttStartModule > mysettings.totalNumberOfSeriesModules * mysettings.totalNumberOfBanks)
      {
        mqttStartModule = 0;
      }

      return;
    }
  }

  //Completed the loop, start at zero
  mqttStartModule = 0;
}

void onMqttConnect(bool sessionPresent)
{
  SERIAL_DEBUG.println(F("Connected to MQTT."));
  myTimerSendMqttPacket.attach(5, sendMqttPacket);
}

void LoadConfiguration()
{
  if (Settings::ReadConfigFromEEPROM((char *)&mysettings, sizeof(mysettings), EEPROM_SETTINGS_START_ADDRESS))
    return;

  SERIAL_DEBUG.println(F("Apply default config"));

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

  //1. Emergency stop
  mysettings.rulevalue[Rule::EmergencyStop] = 0;
  //2. Internal BMS error (communication issues, fault readings from modules etc)
  mysettings.rulevalue[Rule::BMSError] = 0;
  //3. Individual cell over voltage
  mysettings.rulevalue[Rule::Individualcellovervoltage] = 4150;
  //4. Individual cell under voltage
  mysettings.rulevalue[Rule::Individualcellundervoltage] = 3000;
  //5. Individual cell over temperature (external probe)
  mysettings.rulevalue[Rule::IndividualcellovertemperatureExternal] = 55;
  //6. Pack over voltage (mV)
  mysettings.rulevalue[Rule::IndividualcellundertemperatureExternal] = 5;
  //7. Pack under voltage (mV)
  mysettings.rulevalue[Rule::PackOverVoltage] = 4200 * 8;
  //8. RULE_PackUnderVoltage
  mysettings.rulevalue[Rule::PackUnderVoltage] = 3000 * 8;
  mysettings.rulevalue[Rule::Timer1] = 60 * 8;  //8am
  mysettings.rulevalue[Rule::Timer2] = 60 * 17; //5pm

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

//Lazy load the config data - Every 10 seconds see if there is a module we don't have configuration data for, if so request it
void timerLazyCallback()
{
  uint8_t counter = 0;
  //Find the first module that doesn't have settings cached and request them
  for (uint8_t module = 0; module < (mysettings.totalNumberOfBanks * mysettings.totalNumberOfSeriesModules); module++)
  {
    if (cmi[module].valid && !cmi[module].settingsCached)
    {
      prg.sendGetSettingsRequest(module);
      counter++;

      if (counter > 4)
      {
        //Should exit here to avoid flooding the queue, so leave space
        return;
      }
    }
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

void setup()
{
  WiFi.mode(WIFI_OFF);
#if defined(ESP32)
  btStop();
  esp_log_level_set("*", ESP_LOG_WARN); // set all components to WARN level
  //esp_log_level_set("wifi", ESP_LOG_WARN);      // enable WARN logs from WiFi stack
  //esp_log_level_set("dhcpc", ESP_LOG_WARN);     // enable INFO logs from DHCP client
#endif

  //Debug serial output
#if defined(ESP8266)
  //ESP8266 uses dedicated 2nd serial port, but transmit only
  SERIAL_DEBUG.begin(115200, SERIAL_8N1, SERIAL_TX_ONLY);
  SERIAL_DEBUG.setDebugOutput(true);
#endif
#if defined(ESP32)
  //ESP32 we use the USB serial interface for console/debug messages
  SERIAL_DEBUG.begin(115200, SERIAL_8N1);
  SERIAL_DEBUG.setDebugOutput(true);
#endif

  //We generate a unique number which is used in all following JSON requests
  //we use this as a simple method to avoid cross site scripting attacks
  DIYBMSServer::generateUUID();

  SetControllerState(ControllerState::PowerUp);

  hal.ConfigurePins();
#if defined(ESP32)
  hal.ConfigureI2C(TCA6408Interrupt, TCA9534AInterrupt);
#endif
#if defined(ESP8266)
  hal.ConfigureI2C(ExternalInputInterrupt);
#endif

#if defined(ESP32)
  //All comms to i2c needs to go through this single task
  //to prevent issues with thread safety on the i2c hardware/libraries
  queue_i2c = xQueueCreate(10, sizeof(i2cQueueMessage));

  //Create i2c task on CPU 0 (normal code runs on CPU 1)
  xTaskCreatePinnedToCore(i2c_task, "i2c", 2048, nullptr, 2, &i2c_task_handle, 0);
  xTaskCreatePinnedToCore(ledoff_task, "ledoff", 1048, nullptr, 1, &ledoff_task_handle, 0);
#endif

  //Pretend the button is not pressed
  uint8_t clearAPSettings = 0xFF;
#if defined(ESP8266)
  //Fix for issue 5, delay for 3 seconds on power up with green LED lit so
  //people get chance to jump WIFI reset pin (d3)
  hal.GreenLedOn();
  delay(3000);
  //This is normally pulled high, D3 is used to reset WIFI details
  clearAPSettings = digitalRead(RESET_WIFI_PIN);
  hal.GreenLedOff();
#endif

  //Pre configure the array
  memset(&cmi, 0, sizeof(cmi));
  for (size_t i = 0; i < maximum_controller_cell_modules; i++)
  {
    DIYBMSServer::clearModuleValues(i);
  }

  resetAllRules();

  //512 bytes RAM for receive buffer
  SERIAL_DATA.setRxBufferSize(512);

#if defined(ESP32)
  //Receive is IO2 which means the RX1 plug must be disconnected for programming to work!
  SERIAL_DATA.begin(COMMS_BAUD_RATE, SERIAL_8N1, 2, 32); // Serial for comms to modules
#endif

#if defined(ESP8266)
  SERIAL_DATA.begin(COMMS_BAUD_RATE, SERIAL_8N1); // Serial for comms to modules
  //Use alternative GPIO pins of D7/D8
  //D7 = GPIO13 = RECEIVE SERIAL
  //D8 = GPIO15 = TRANSMIT SERIAL
  SERIAL_DATA.swap();
#endif

  myPacketSerial.begin(&SERIAL_DATA, &onPacketReceived, sizeof(PacketStruct), SerialPacketReceiveBuffer, sizeof(SerialPacketReceiveBuffer));

#if defined(ESP8266)
  // initialize LittleFS
  if (!LittleFS.begin())
#endif
#if defined(ESP32)
    // initialize LittleFS
    if (!SPIFFS.begin())
#endif
    {
      SERIAL_DEBUG.println(F("An Error has occurred while mounting LittleFS"));
    }

  LoadConfiguration();

  InputsEnabled = hal.InputsEnabled;
  OutputsEnabled = hal.OutputsEnabled;

  //Set relay defaults
  for (int8_t y = 0; y < RELAY_TOTAL; y++)
  {
    previousRelayState[y] = mysettings.rulerelaydefault[y];
    //Set relay defaults
    hal.SetOutputState(y, mysettings.rulerelaydefault[y]);
  }

  //Temporarly force WIFI settings
  //wifi_eeprom_settings xxxx;
  //strcpy(xxxx.wifi_ssid,"XXXXXX");
  //strcpy(xxxx.wifi_passphrase,"XXXXXX");
  //Settings::WriteConfigToEEPROM((char*)&xxxx, sizeof(xxxx), EEPROM_WIFI_START_ADDRESS);

  if (!DIYBMSSoftAP::LoadConfigFromEEPROM() || clearAPSettings == 0)
  {
    SERIAL_DEBUG.print(F("Clear AP settings"));
    SERIAL_DEBUG.println(clearAPSettings);
    SERIAL_DEBUG.println(F("Setup Access Point"));
    //We are in initial power on mode (factory reset)
    DIYBMSSoftAP::SetupAccessPoint(&server);
  }
  else
  {

#if defined(ESP8266)
    //Config NTP
    NTP.onNTPSyncEvent([](NTPSyncEvent_t event) {
      ntpEvent = event;
      NTPsyncEventTriggered = true;
    });
#endif

    SERIAL_DEBUG.println(F("Connecting to WIFI"));

    /* Explicitly set the ESP8266 to be a WiFi-client, otherwise by default,
      would try to act as both a client and an access-point */

#if defined(ESP8266)
    wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
    wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);
#endif

#if defined(ESP32)
    WiFi.onEvent(onWifiConnect, WiFiEvent_t::SYSTEM_EVENT_STA_GOT_IP);
    WiFi.onEvent(onWifiDisconnect, WiFiEvent_t::SYSTEM_EVENT_STA_DISCONNECTED);
#endif

    mqttClient.onConnect(onMqttConnect);
    mqttClient.onDisconnect(onMqttDisconnect);

    if (mysettings.mqtt_enabled)
    {
      SERIAL_DEBUG.println("MQTT Enabled");
      mqttClient.setServer(mysettings.mqtt_server, mysettings.mqtt_port);
      mqttClient.setCredentials(mysettings.mqtt_username, mysettings.mqtt_password);
    }

    connectToWifi();
  }

  //Ensure we service the cell modules every 4 seconds
  myTimer.attach(4, timerEnqueueCallback);

  //Process rules every 5 seconds
  myTimerRelay.attach(5, timerProcessRules);

  //We process the transmit queue every 1 seconds (this needs to be lower delay than the queue fills)
  //and slower than it takes a single module to process a command (about 300ms)
  myTransmitTimer.attach(1, timerTransmitCallback);

  //This is a lazy timer for low priority tasks
  myLazyTimer.attach(15, timerLazyCallback);

  //We have just started...
  SetControllerState(ControllerState::Stabilizing);
}

void loop()
{
  ArduinoOTA.handle();
  //ESP_LOGW("LOOP","LOOP");

  // Call update to receive, decode and process incoming packets.
  myPacketSerial.checkInputStream();

  if (ConfigHasChanged > 0)
  {
    //Auto reboot if needed (after changing MQTT or INFLUX settings)
    //Ideally we wouldn't need to reboot if the code could sort itself out!
    ConfigHasChanged--;
    if (ConfigHasChanged == 0)
    {
      SERIAL_DEBUG.println(F("RESTART AFTER CONFIG CHANGE"));
      //Stop networking
      if (mqttClient.connected())
      {
        mqttClient.disconnect(true);
      }
      WiFi.disconnect();
      ESP.restart();
    }
  }

#if defined(ESP8266)
  if (NTPsyncEventTriggered)
  {
    processSyncEvent(ntpEvent);
    NTPsyncEventTriggered = false;
  }
#endif
}
