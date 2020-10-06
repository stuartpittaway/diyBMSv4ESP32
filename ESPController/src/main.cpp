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

#include <Arduino.h>

#define PACKET_LOGGING
//#define RULES_LOGGING

#if defined(ESP8266)
#include <TimeLib.h>
#include <ESP8266WiFi.h>
#include <NtpClientLib.h>
#endif

#if defined(ESP32)
#include <WiFi.h>
#include <SPIFFS.h>
#include "time.h"
#endif

#include <Ticker.h>
#include <ESPAsyncWebServer.h>
#include <AsyncMqttClient.h>
#include <PacketSerial.h>
#include <cppQueue.h>
#include <pcf8574_esp.h>
#include <Wire.h>

#include "defines.h"

bool PCF8574Enabled;
volatile bool emergencyStop = false;
bool rule_outcome[RELAY_RULES];

uint16_t ConfigHasChanged = 0;
diybms_eeprom_settings mysettings;

bool server_running = false;
//bool wifiFirstConnected = false;
uint8_t packetType = 0;
uint8_t previousRelayState[RELAY_TOTAL];
bool previousRelayPulse[RELAY_TOTAL];

#if defined(ESP8266)
bool NTPsyncEventTriggered = false; // True if a time even has been triggered
NTPSyncEvent_t ntpEvent;            // Last triggered event
#endif

AsyncWebServer server(80);

//PCF8574P has an i2c address of 0x38 instead of the normal 0x20
PCF857x pcf8574(0x38, &Wire);

void ICACHE_RAM_ATTR PCFInterrupt()
{
  if ((pcf8574.read8() & B00010000) == 0)
  {
    //Emergency Stop (J1) has triggered
    emergencyStop = true;
  }
}

//This large array holds all the information about the modules
//up to 4x16
CellModuleInfo cmi[maximum_bank_of_modules][maximum_cell_modules];
uint8_t numberOfModules[maximum_bank_of_modules];

#include "crc16.h"

#include "settings.h"
#include "SoftAP.h"
#include "DIYBMSServer.h"
#include "PacketRequestGenerator.h"
#include "PacketReceiveProcessor.h"

// Instantiate queue to hold packets ready for transmission
Queue requestQueue(sizeof(packet), 16, FIFO);

PacketRequestGenerator prg = PacketRequestGenerator(&requestQueue);

PacketReceiveProcessor receiveProc = PacketReceiveProcessor();

#define framingmarker (uint8_t)0x00

PacketSerial_<COBS, framingmarker, 128> myPacketSerial;

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

AsyncMqttClient mqttClient;

void dumpPacketToDebug(packet *buffer)
{
  SERIAL_DEBUG.print(buffer->address, HEX);
  SERIAL_DEBUG.print('/');
  SERIAL_DEBUG.print(buffer->command, HEX);
  SERIAL_DEBUG.print('/');
  SERIAL_DEBUG.print(buffer->sequence, HEX);
  SERIAL_DEBUG.print('=');
  for (size_t i = 0; i < maximum_cell_modules; i++)
  {
    SERIAL_DEBUG.print(buffer->moduledata[i], HEX);
    SERIAL_DEBUG.print(" ");
  }
  SERIAL_DEBUG.print(" =");
  SERIAL_DEBUG.print(buffer->crc, HEX);
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
      SERIAL_DEBUG.println("NTP server not reachable");
    else if (ntpEvent == invalidAddress)
      SERIAL_DEBUG.println("Invalid NTP server address");
    else if (ntpEvent == errorSending)
      SERIAL_DEBUG.println("Error sending request");
    else if (ntpEvent == responseError)
      SERIAL_DEBUG.println("NTP response error");
  }
  else
  {
    if (ntpEvent == timeSyncd)
    {
      SERIAL_DEBUG.print("Got NTP time");
      time_t lastTime = NTP.getLastNTPSync();
      SERIAL_DEBUG.println(NTP.getTimeDateString(lastTime));
      setTime(lastTime);
    }
  }
}
#endif

void onPacketReceived(const uint8_t *receivebuffer, size_t len)
{
  //Note that this function gets called frequently with zero length packets
  //due to the way the modules operate
    GREEN_LED_ON;

  if (len == sizeof(packet))
  {

#if defined(PACKET_LOGGING)
    // Process decoded incoming packet
    SERIAL_DEBUG.print("R:");
    dumpPacketToDebug((packet *)receivebuffer);
#endif

    if (!receiveProc.ProcessReply(receivebuffer, sequence))
    {
      SERIAL_DEBUG.print("**FAIL PROCESS REPLY**");
    }
#if defined(PACKET_LOGGING)
    SERIAL_DEBUG.println("");
    //SERIAL_DEBUG.print("Timing:");SERIAL_DEBUG.print(receiveProc.packetTimerMillisecond);SERIAL_DEBUG.println("ms");
#endif
  }

    GREEN_LED_OFF;
}

void timerTransmitCallback()
{
  // Called to transmit the next packet in the queue need to ensure this procedure is called more frequently than
  // items are added into the queue
  if (!requestQueue.isEmpty())
  {
    packet transmitBuffer;


    //Wake up the connected cell module from sleep
    SERIAL_DATA.write(framingmarker);
    delay(3);

    requestQueue.pop(&transmitBuffer);
    sequence++;
    transmitBuffer.sequence = sequence;
    transmitBuffer.crc = CRC16::CalculateArray((uint8_t *)&transmitBuffer, sizeof(packet) - 2);
    myPacketSerial.send((byte *)&transmitBuffer, sizeof(transmitBuffer));

    //Grab the time we sent this packet to time how long packets take to move
    //through the modules.  We only time the COMMAND::ReadVoltageAndStatus packets
    if (transmitBuffer.command == COMMAND::ReadVoltageAndStatus)
    {
      receiveProc.packetLastSentSequence = sequence;
      receiveProc.packetLastSentMillisecond = millis();
    }

    // Output the packet we just transmitted to debug console
#if defined(PACKET_LOGGING)
    SERIAL_DEBUG.print("S:");
    dumpPacketToDebug(&transmitBuffer);
    SERIAL_DEBUG.print("/Q:");
    SERIAL_DEBUG.println(requestQueue.getCount());
#endif

  }
}

void ProcessRules()
{

  //Runs the rules and populates rule_outcome array with true/false for each rule

  uint32_t packvoltage[4];

  packvoltage[0] = 0;
  packvoltage[1] = 0;
  packvoltage[2] = 0;
  packvoltage[3] = 0;

  for (int8_t r = 0; r < RELAY_RULES; r++)
  {
    rule_outcome[r] = false;
  }

  //If we have a communications error
  if (emergencyStop)
  {
    rule_outcome[RULE_EmergencyStop] = true;
  }

  //If we have a communications error
  if (receiveProc.HasCommsTimedOut())
  {
    rule_outcome[RULE_CommunicationsError] = true;
  }

  //Loop through cells
  for (int8_t bank = 0; bank < mysettings.totalNumberOfBanks; bank++)
  {
    for (int8_t i = 0; i < numberOfModules[bank]; i++)
    {

      packvoltage[bank] += cmi[bank][i].voltagemV;

      if (cmi[bank][i].voltagemV > mysettings.rulevalue[RULE_Individualcellovervoltage])
      {
        //Rule Individual cell over voltage
        rule_outcome[RULE_Individualcellovervoltage] = true;
      }

      if (cmi[bank][i].voltagemV < mysettings.rulevalue[RULE_Individualcellundervoltage])
      {
        //Rule Individual cell under voltage (mV)
        rule_outcome[RULE_Individualcellundervoltage] = true;
      }

      if ((cmi[bank][i].externalTemp != -40) && (cmi[bank][i].externalTemp > mysettings.rulevalue[RULE_IndividualcellovertemperatureExternal]))
      {
        //Rule Individual cell over temperature (external probe)
        rule_outcome[RULE_IndividualcellovertemperatureExternal] = true;
      }

      if ((cmi[bank][i].externalTemp != -40) && (cmi[bank][i].externalTemp < mysettings.rulevalue[RULE_IndividualcellundertemperatureExternal]))
      {
        //Rule Individual cell UNDER temperature (external probe)
        rule_outcome[RULE_IndividualcellundertemperatureExternal] = true;
      }
    }
  }

  //Combine the voltages if we need to
  if (mysettings.combinationParallel)
  {
    //We have multiple banks which should be evaluated individually
    for (int8_t bank = 0; bank < mysettings.totalNumberOfBanks; bank++)
    {
      if (packvoltage[bank] > mysettings.rulevalue[RULE_PackOverVoltage])
      {
        //Rule - Pack over voltage (mV)
        rule_outcome[RULE_PackOverVoltage] = true;
      }

      if (packvoltage[bank] < mysettings.rulevalue[RULE_PackUnderVoltage])
      {
        //Rule - Pack under voltage (mV)
        rule_outcome[RULE_PackUnderVoltage] = true;
      }
    }
  }
  else
  {
    //We have multiple banks which should be evaluated as a whole
    if ((packvoltage[0] + packvoltage[1] + packvoltage[2] + packvoltage[3]) > mysettings.rulevalue[RULE_PackOverVoltage])
    {
      //Rule - Pack over voltage (mV)
      rule_outcome[RULE_PackOverVoltage] = true;
    }

    if ((packvoltage[0] + packvoltage[1] + packvoltage[2] + packvoltage[3]) < mysettings.rulevalue[RULE_PackUnderVoltage])
    {
      //Rule - Pack under voltage (mV)
      rule_outcome[RULE_PackUnderVoltage] = true;
    }
  }

  //Time based rules
  if (minutesSinceMidnight() >= mysettings.rulevalue[RULE_Timer1])
  {
    rule_outcome[RULE_Timer1] = true;
  }

  if (minutesSinceMidnight() >= mysettings.rulevalue[RULE_Timer2])
  {
    rule_outcome[RULE_Timer2] = true;
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
      pcf8574.write(y, previousRelayState[y] == HIGH ? LOW : HIGH);

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

  // DO NOTE: When you write LOW to a pin on a PCF8574 it becomes an OUTPUT.
  // It wouldn't generate an interrupt if you were to connect a button to it that pulls it HIGH when you press the button.
  // Any pin you wish to use as input must be written HIGH and be pulled LOW to generate an interrupt.

#if defined(RULES_LOGGING)
  SERIAL_DEBUG.print("Rules:");
  for (int8_t r = 0; r < RELAY_RULES; r++)
  {
    SERIAL_DEBUG.print(rule_outcome[r]);
  }
  SERIAL_DEBUG.print("=");
#endif

  uint8_t relay[RELAY_TOTAL];

  //Set defaults based on configuration
  for (int8_t y = 0; y < RELAY_TOTAL; y++)
  {
    relay[y] = mysettings.rulerelaydefault[y] == RELAY_ON ? LOW : HIGH;
  }

  //Test the rules (in reverse order)
  for (int8_t n = RELAY_RULES - 1; n >= 0; n--)
  {
    if (rule_outcome[n] == true)
    {

      for (int8_t y = 0; y < RELAY_TOTAL; y++)
      {
        //Dont change relay if its set to ignore/X
        if (mysettings.rulerelaystate[n][y] != RELAY_X)
        {
          //Logic is inverted on the PCF chip
          if (mysettings.rulerelaystate[n][y] == RELAY_ON)
          {
            relay[y] = LOW;
          }
          else
          {
            relay[y] = HIGH;
          }
        }
      }
    }
  }

  if (PCF8574Enabled)
  {
    //Perhaps we should publish the relay settings over MQTT and INFLUX/website?
    for (int8_t n = 0; n < RELAY_TOTAL; n++)
    {
      if (previousRelayState[n] != relay[n])
      {
        //Would be better here to use the WRITE8 to lower i2c traffic
#if defined(RULES_LOGGING)
        SERIAL_DEBUG.print("Relay:");
        SERIAL_DEBUG.print(n);
        SERIAL_DEBUG.print("=");
        SERIAL_DEBUG.print(relay[n]);
#endif
        //Set the relay
        pcf8574.write(n, relay[n]);

        previousRelayState[n] = relay[n];

        if (mysettings.relaytype[n] == RELAY_PULSE)
        {
          //If its a pulsed relay, invert the output quickly via a one time only timer
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
  else
  {
#if defined(RULES_LOGGING)
    SERIAL_DEBUG.println("N/F");
#endif
  }
}

uint8_t counter = 0;

void timerEnqueueCallback()
{

  //this is called regularly on a timer, it determines what request to make to the modules (via the request queue)
  for (uint8_t b = 0; b < mysettings.totalNumberOfBanks; b++)
  {

    prg.sendCellVoltageRequest(b);
    prg.sendCellTemperatureRequest(b);

    //If any module is in bypass then request PWM reading for whole bank
    for (uint8_t m = 0; m < numberOfModules[b]; m++)
    {
      if (cmi[b][m].inBypass)
      {
        prg.sendReadBalancePowerRequest(b);
        break;
      }
    }

    //Every 50 loops also ask for bad packet count (saves battery power if we dont ask for this all the time)
    if (counter % 50 == 0)
    {
      prg.sendReadBadPacketCounter(b);
    }
  }

  //It's an unsigned byte, let it overflow to reset
  counter++;
}

void connectToWifi()
{
  SERIAL_DEBUG.println("Connecting to Wi-Fi...");
#if defined(ESP8266)
  WiFi.hostname("diyBMS-ESP8266");
#endif
#if defined(ESP32)
  WiFi.setHostname("diyBMS-ESP32");
#endif
  WiFi.mode(WIFI_STA);
  WiFi.begin(DIYBMSSoftAP::WifiSSID(), DIYBMSSoftAP::WifiPassword());
}

void connectToMqtt()
{
  SERIAL_DEBUG.println("Connecting to MQTT...");
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
    SERIAL_DEBUG.println("Connect Error");
    aClient = NULL;
    delete client;
  },
                   NULL);

  aClient->onConnect([](void *arg, AsyncClient *client) {
    SERIAL_DEBUG.println("Connected");

    //Send the packet here

    aClient->onError(NULL, NULL);

    client->onDisconnect([](void *arg, AsyncClient *c) {
      SERIAL_DEBUG.println("Disconnected");
      aClient = NULL;
      delete c;
    },
                         NULL);

    client->onData([](void *arg, AsyncClient *c, void *data, size_t len) {
      //Data received
      SERIAL_DEBUG.print("\r\nData: ");
      SERIAL_DEBUG.println(len);
      //uint8_t* d = (uint8_t*)data;
      //for (size_t i = 0; i < len; i++) {SERIAL_DEBUG.write(d[i]);}
    },
                   NULL);

    //send the request

    //Construct URL for the influxdb
    //See API at https://docs.influxdata.com/influxdb/v1.7/tools/api/#write-http-endpoint

    String poststring;

    for (uint8_t bank = 0; bank < 4; bank++)
    {
      //TODO: We should send a request per bank not just a single POST as we are likely to exceed capabilities of ESP
      for (uint8_t i = 0; i < numberOfModules[bank]; i++)
      {

        //Data in LINE PROTOCOL format https://docs.influxdata.com/influxdb/v1.7/write_protocols/line_protocol_tutorial/
        poststring = poststring + "cells," + "cell=" + String(bank + 1) + "_" + String(i + 1) + " v=" + String((float)cmi[bank][i].voltagemV / 1000.0, 3) + ",i=" + String(cmi[bank][i].internalTemp) + "i" + ",e=" + String(cmi[bank][i].externalTemp) + "i" + ",b=" + (cmi[bank][i].inBypass ? String("true") : String("false")) + "\n";
      }
    }

    //TODO: Need to URLEncode these values
    //+ String(mysettings.influxdb_host) + ":" + String(mysettings.influxdb_httpPort)
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

  SERIAL_DEBUG.println("SendInfluxdbPacket");

  setupInfluxClient();

  if (!aClient->connect(mysettings.influxdb_host, mysettings.influxdb_httpPort))
  {
    SERIAL_DEBUG.println("Influxdb connect fail");
    AsyncClient *client = aClient;
    aClient = NULL;
    delete client;
  }
}

void startTimerToInfluxdb()
{
  myTimerSendInfluxdbPacket.attach(30, SendInfluxdbPacket);
}

#if defined(ESP8266)
void onWifiConnect(const WiFiEventStationModeGotIP &event)
{
#else
void onWifiConnect(WiFiEvent_t event, WiFiEventInfo_t info)
{
#endif

  SERIAL_DEBUG.print("DIYBMS Wi-Fi, ");
  SERIAL_DEBUG.print(WiFi.status());
  SERIAL_DEBUG.print(F(". Connected IP:"));
  SERIAL_DEBUG.println(WiFi.localIP());

  SERIAL_DEBUG.print("Requesting NTP from ");
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
}

#if defined(ESP8266)
void onWifiDisconnect(const WiFiEventStationModeDisconnected &event)
{
#else
void onWifiDisconnect(WiFiEvent_t event, WiFiEventInfo_t info)
{
#endif
  SERIAL_DEBUG.println("Disconnected from Wi-Fi.");

  // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
  mqttReconnectTimer.detach();
  myTimerSendMqttPacket.detach();
  myTimerSendInfluxdbPacket.detach();

  wifiReconnectTimer.once(2, connectToWifi);

  //DIYBMSServer::StopServer(&server);
  //server_running=false;
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
  SERIAL_DEBUG.println("Disconnected from MQTT.");

  myTimerSendMqttPacket.detach();

  if (WiFi.isConnected())
  {
    mqttReconnectTimer.once(2, connectToMqtt);
  }
}

void sendMqttPacket()
{
  if (!mysettings.mqtt_enabled && !mqttClient.connected())
    return;

  SERIAL_DEBUG.println("Sending MQTT");

  char topic[80];
  char jsonbuffer[100];
  //char value[20];
  //uint16_t reply;

  for (uint8_t bank = 0; bank < mysettings.totalNumberOfBanks; bank++) {
    for (uint8_t i = 0; i < numberOfModules[bank]; i++) {

      StaticJsonDocument<100> doc;
      doc["voltage"] = (float)cmi[bank][i].voltagemV/1000.0;
      doc["inttemp"] = cmi[bank][i].internalTemp;
      doc["exttemp"] = cmi[bank][i].externalTemp;
      doc["bypass"] = cmi[bank][i].inBypass ? 1:0;
      serializeJson(doc, jsonbuffer, sizeof(jsonbuffer));

      sprintf(topic, "%s/%d/%d", mysettings.mqtt_topic, bank, i);
      mqttClient.publish(topic, 0, false, jsonbuffer);
      SERIAL_DEBUG.println(topic);
      //SERIAL_DEBUG.print(" ");SERIAL_DEBUG.print(jsonbuffer);SERIAL_DEBUG.print(" ");SERIAL_DEBUG.println(reply);
    }
  }
}

void onMqttConnect(bool sessionPresent)
{
  SERIAL_DEBUG.println("Connected to MQTT.");
  myTimerSendMqttPacket.attach(30, sendMqttPacket);
}

void LoadConfiguration()
{

  if (Settings::ReadConfigFromEEPROM((char *)&mysettings, sizeof(mysettings), EEPROM_SETTINGS_START_ADDRESS))
    return;

  SERIAL_DEBUG.println("Apply default config");

  mysettings.totalNumberOfBanks = 1;
  mysettings.combinationParallel = true;

  //EEPROM settings are invalid so default configuration
  mysettings.mqtt_enabled = false;
  mysettings.mqtt_port = 1883;

  //Default to EMONPI default MQTT settings
  strcpy(mysettings.mqtt_topic,"diybms");
  strcpy(mysettings.mqtt_server, "192.168.0.26");
  strcpy(mysettings.mqtt_username, "emonpi");
  strcpy(mysettings.mqtt_password, "emonpimqtt2016");

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
  mysettings.rulevalue[RULE_EmergencyStop] = 0;
  //2. Communications error
  mysettings.rulevalue[RULE_CommunicationsError] = 0;
  //3. Individual cell over voltage
  mysettings.rulevalue[RULE_Individualcellovervoltage] = 4150;
  //4. Individual cell under voltage
  mysettings.rulevalue[RULE_Individualcellundervoltage] = 3000;
  //5. Individual cell over temperature (external probe)
  mysettings.rulevalue[RULE_IndividualcellovertemperatureExternal] = 55;
  //6. Pack over voltage (mV)
  mysettings.rulevalue[RULE_IndividualcellundertemperatureExternal] = 5;
  //7. Pack under voltage (mV)
  mysettings.rulevalue[RULE_PackOverVoltage] = 4200 * 8;
  //8. RULE_PackUnderVoltage
  mysettings.rulevalue[RULE_PackUnderVoltage] = 3000 * 8;
  mysettings.rulevalue[RULE_Timer1] = 60 * 8;  //8am
  mysettings.rulevalue[RULE_Timer2] = 60 * 17; //5pm

  //Set all relays to don't care
  for (size_t i = 0; i < RELAY_RULES; i++)
  {
    for (size_t x = 0; x < RELAY_TOTAL; x++)
    {
      mysettings.rulerelaystate[i][x] = RELAY_X;
    }
  }
}

void ConfigureI2C()
{
#if defined(ESP8266)
  //SDA / SCL
  //I'm sure this should be 4,5 !
  Wire.begin(5, 4);
#endif

#if defined(ESP32)
  //SDA / SCL
  //ESP32 = I2C0-SDA / I2C0-SCL
  //I2C Bus 1: uses GPIO 27 (SDA) and GPIO 26 (SCL);
  //I2C Bus 2: uses GPIO 33 (SDA) and GPIO 32 (SCL);
  Wire.begin(27, 26);
#endif

  Wire.setClock(100000L);

  //Make PINs 4-7 INPUTs - the interrupt fires when triggered
  pcf8574.begin();

  //We test to see if the i2c expander is actually fitted
  pcf8574.read8();

  //Set relay defaults
  for (int8_t y = 0; y < RELAY_TOTAL; y++)
  {
    previousRelayState[y] = mysettings.rulerelaydefault[y] == RELAY_ON ? LOW : HIGH;
  }

  if (pcf8574.lastError() == 0)
  {
    SERIAL_DEBUG.println("Found pcf8574");
    pcf8574.write(4, HIGH);
    pcf8574.write(5, HIGH);
    pcf8574.write(6, HIGH);
    pcf8574.write(7, HIGH);

    //Set relay defaults
    for (int8_t y = 0; y < RELAY_TOTAL; y++)
    {
      pcf8574.write(y, previousRelayState[y]);
    }
    PCF8574Enabled = true;
  }
  else
  {
    //Not fitted
    SERIAL_DEBUG.println("pcf8574 not fitted");
    PCF8574Enabled = false;
  }

  //internal pullup-resistor on the interrupt line via ESP8266
  pcf8574.resetInterruptPin();

  //TODO: Fix this for ESP32 different PIN
  attachInterrupt(digitalPinToInterrupt(PFC_INTERRUPT_PIN), PCFInterrupt, FALLING);
}

//Lazy load the config data - Every 10 seconds see if there is a module we don't have configuration data for, if so request it
void timerLazyCallback()
{
//Find the first module that doesn't have settings cached and request them
//we only do 1 module at a time to avoid flooding the queue
  for (uint8_t bank = 0; bank < 4; bank++)
  {
    for (uint8_t module = 0; module < numberOfModules[bank]; module++)
    {
      if (!cmi[bank][module].settingsCached)
      {
        prg.sendGetSettingsRequest(bank, module);
      }
    }
  }
}

void setup()
{
  WiFi.mode(WIFI_OFF);

#if defined(ESP32)
  btStop();
#endif

#if defined(ESP32)
  esp_log_level_set("*", ESP_LOG_WARN); // set all components to ERROR level
  //esp_log_level_set("wifi", ESP_LOG_WARN);      // enable WARN logs from WiFi stack
  //esp_log_level_set("dhcpc", ESP_LOG_INFO);     // enable INFO logs from DHCP client
#endif

  //Serial is used for communication to modules, SERIAL_DEBUG is for debug output
  pinMode(GREEN_LED, OUTPUT);
  //D3 is used to reset access point WIFI details on boot up
  pinMode(RESET_WIFI_PIN, INPUT_PULLUP);
  //D5 is interrupt pin from PCF8574
  pinMode(PFC_INTERRUPT_PIN, INPUT_PULLUP);

  //Fix for issue 5, delay for 3 seconds on power up with green LED lit so
  //people get chance to jump WIFI reset pin (d3)
  GREEN_LED_ON;
  delay(3000);
  //This is normally pulled high, D3 is used to reset WIFI details
  uint8_t clearAPSettings = digitalRead(RESET_WIFI_PIN);
  GREEN_LED_OFF;

  //We generate a unique number which is used in all following JSON requests
  //we use this as a simple method to avoid cross site scripting attacks
  DIYBMSServer::generateUUID();

  numberOfModules[0] = 0;
  numberOfModules[1] = 0;
  numberOfModules[2] = 0;
  numberOfModules[3] = 0;

  //Pre configure the array
  for (size_t i = 0; i < maximum_cell_modules; i++)
  {
    cmi[0][i].voltagemVMax = 0;
    cmi[0][i].voltagemVMin = 6000;
    cmi[1][i].voltagemVMax = 0;
    cmi[1][i].voltagemVMin = 6000;
    cmi[2][i].voltagemVMax = 0;
    cmi[2][i].voltagemVMin = 6000;
    cmi[3][i].voltagemVMax = 0;
    cmi[3][i].voltagemVMin = 6000;
  }

  SERIAL_DATA.begin(COMMS_BAUD_RATE, SERIAL_8N1); // Serial for comms to modules

#if defined(ESP8266)
  //Use alternative GPIO pins of D7/D8
  //D7 = GPIO13 = RECEIVE SERIAL
  //D8 = GPIO15 = TRANSMIT SERIAL
  SERIAL_DATA.swap();
#endif

  myPacketSerial.setStream(&SERIAL_DATA); // start serial for output
  myPacketSerial.setPacketHandler(&onPacketReceived);

  //Debug serial output
  SERIAL_DEBUG.begin(115200, SERIAL_8N1);
  SERIAL_DEBUG.setDebugOutput(true);

  // initialize SPIFFS
  if (!SPIFFS.begin())
  {
    SERIAL_DEBUG.println("An Error has occurred while mounting SPIFFS");
  }

  LoadConfiguration();

  ConfigureI2C();

  //Temporarly force WIFI settings
  //wifi_eeprom_settings xxxx;
  //strcpy(xxxx.wifi_ssid,"XXXXXX");
  //strcpy(xxxx.wifi_passphrase,"XXXXXX");
  //Settings::WriteConfigToEEPROM((char*)&xxxx, sizeof(xxxx), EEPROM_WIFI_START_ADDRESS);

  if (!DIYBMSSoftAP::LoadConfigFromEEPROM() || clearAPSettings == 0)
  {
    SERIAL_DEBUG.print("Clear AP settings");
    SERIAL_DEBUG.println(clearAPSettings);
    SERIAL_DEBUG.println("Setup Access Point");
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

    SERIAL_DEBUG.println("Connecting to WIFI");

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

  //Process rules every 5 seconds (this prevents the relays from clattering on and off)
  myTimerRelay.attach(5, timerProcessRules);

  //We process the transmit queue every 0.5 seconds (this needs to be lower delay than the queue fills)
  myTransmitTimer.attach(0.5, timerTransmitCallback);

  //This is my 10 second lazy timer
  myLazyTimer.attach(10, timerLazyCallback);
}

void loop()
{
  //ESP_LOGW("LOOP","LOOP");

  // Call update to receive, decode and process incoming packets.
  if (SERIAL_DATA.available())
  {
    myPacketSerial.update();
  }

  if (ConfigHasChanged > 0)
  {
    //Auto reboot if needed (after changing MQTT or INFLUX settings)
    //Ideally we wouldn't need to reboot if the code could sort itself out!
    ConfigHasChanged--;
    if (ConfigHasChanged == 0)
    {
      SERIAL_DEBUG.println("RESTART AFTER CONFIG CHANGE");
      //Stop networking
      if (mqttClient.connected())
      {
        mqttClient.disconnect(true);
      }
      WiFi.disconnect();
      ESP.restart();
    }
  }

  //if (emergencyStop) {    SERIAL_DEBUG.println("EMERGENCY STOP");  }

#if defined(ESP8266)
  if (NTPsyncEventTriggered)
  {
    processSyncEvent(ntpEvent);
    NTPsyncEventTriggered = false;
  }
#endif
}
