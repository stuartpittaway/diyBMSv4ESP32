
/*
  ____  ____  _  _  ____  __  __  ___
 (  _ \(_  _)( \/ )(  _ \(  \/  )/ __)
  )(_) )_)(_  \  /  ) _ < )    ( \__ \
 (____/(____) (__) (____/(_/\/\_)(___/

DIYBMS V4.0
ESP8266 MODULE

(c)2019 Stuart Pittaway

COMPILE THIS CODE USING PLATFORM.IO

LICENSE
Attribution-NonCommercial-ShareAlike 2.0 UK: England & Wales (CC BY-NC-SA 2.0 UK)
https://creativecommons.org/licenses/by-nc-sa/2.0/uk/

* Non-Commercial — You may not use the material for commercial purposes.
* Attribution — You must give appropriate credit, provide a link to the license, and indicate if changes were made.
  You may do so in any reasonable manner, but not in any way that suggests the licensor endorses you or your use.
* ShareAlike — If you remix, transform, or build upon the material, you must distribute your   contributions under the same license as the original.
* No additional restrictions — You may not apply legal terms or technological measures that legally restrict others from doing anything the license permits.
*/

#include "DIYBMSServer.h"
#include "ArduinoJson.h"
#include "defines.h"

#if defined(ESP8266)
#include "ESP8266TrueRandom.h"
#include <TimeLib.h>
#endif

#if defined(ESP32)
#include <SPIFFS.h>
#include "time.h"
#endif

#include "settings.h"

AsyncWebServer *DIYBMSServer::_myserver;
String DIYBMSServer::UUIDString;

#define REBOOT_COUNT_DOWN 2000

String DIYBMSServer::uuidToString(uint8_t* uuidLocation) {
  String string = "";
  int i;
  for (i=0; i<16; i++) {
    if (i==4) string += "-";
    if (i==6) string += "-";
    if (i==8) string += "-";
    if (i==10) string += "-";
    int topDigit = uuidLocation[i] >> 4;
    int bottomDigit = uuidLocation[i] & 0x0f;
    // High hex digit
    string += "0123456789abcdef"[topDigit];
    // Low hex digit
    string += "0123456789abcdef"[bottomDigit];
  }

  return string;
}

void DIYBMSServer::generateUUID() {
    //SERIAL_DEBUG.print("generateUUID=");
    byte uuidNumber[16]; // UUIDs in binary form are 16 bytes long
    #if defined(ESP8266)
    ESP8266TrueRandom.uuid(uuidNumber);
    #endif

    #if defined(ESP32)
    //ESP32 has inbuilt random number generator
    //https://techtutorialsx.com/2017/12/22/esp32-arduino-random-number-generation/
    for (uint8_t x=0;x<16;x--) uuidNumber[x] = random(0xFF);
    #endif

    UUIDString = uuidToString(uuidNumber);
}

bool DIYBMSServer::validateXSS(AsyncWebServerRequest* request)
{
    if (request->hasHeader("Cookie")) {
        AsyncWebHeader* cookie = request->getHeader("Cookie");
        if (cookie->value().startsWith("DIYBMS_XSS=")) {
            if (cookie->value().substring(11).equals(DIYBMSServer::UUIDString)) {
                if (request->hasParam("xss", true)) {
                    AsyncWebParameter* p1 = request->getParam("xss", true);

                    if (p1->value().equals(DIYBMSServer::UUIDString) == true) {
                        return true;
                    }
                }
            }
        }
    }
    request->send(500, "text/plain", "XSS invalid");
    return false;
}

void DIYBMSServer::SendSuccess(AsyncWebServerRequest *request) {
  AsyncResponseStream *response = request->beginResponseStream("application/json");
  StaticJsonDocument<100> doc;
  doc["success"] = true;
  serializeJson(doc, *response);
  request->send(response);
}

void DIYBMSServer::resetCounters(AsyncWebServerRequest *request) {
  if (!validateXSS(request)) return;

  receiveProc.totalCRCErrors=0;
  receiveProc.totalNotProcessedErrors=0;
  prg.packetsGenerated=0;
  receiveProc.packetsReceived=0;
  SendSuccess(request);
}


void DIYBMSServer::saveInfluxDBSetting(AsyncWebServerRequest *request) {
  if (!validateXSS(request)) return;

  if (request->hasParam("influxEnabled", true)) {
    AsyncWebParameter *p1 = request->getParam("influxEnabled", true);
    mysettings.influxdb_enabled =p1->value().equals("on") ? true:false;
  } else {
    mysettings.influxdb_enabled=false;
  }

  if (request->hasParam("influxPort", true)) {
    AsyncWebParameter *p1 = request->getParam("influxPort", true);
    mysettings.influxdb_httpPort =p1->value().toInt();
  }

  if (request->hasParam("influxServer", true)) {
    AsyncWebParameter *p1 = request->getParam("influxServer", true);
    p1->value().toCharArray(mysettings.influxdb_host,sizeof(mysettings.influxdb_host));
  }

  if (request->hasParam("influxDatabase", true)) {
    AsyncWebParameter *p1 = request->getParam("influxDatabase", true);
    p1->value().toCharArray(mysettings.influxdb_database,sizeof(mysettings.influxdb_database));
  }

  if (request->hasParam("influxUsername", true)) {
    AsyncWebParameter *p1 = request->getParam("influxUsername", true);
    p1->value().toCharArray(mysettings.influxdb_user,sizeof(mysettings.influxdb_user));
  }

  if (request->hasParam("influxPassword", true)) {
    AsyncWebParameter *p1 = request->getParam("influxPassword", true);
    p1->value().toCharArray(mysettings.influxdb_password,sizeof(mysettings.influxdb_password));
  }

  Settings::WriteConfigToEEPROM((char*)&mysettings, sizeof(mysettings), EEPROM_SETTINGS_START_ADDRESS);

  ConfigHasChanged = REBOOT_COUNT_DOWN;
  SendSuccess(request);
}



void DIYBMSServer::saveRuleConfiguration(AsyncWebServerRequest *request) {
  if (!validateXSS(request)) return;


  //relaytype
  for (int i = 0; i < RELAY_TOTAL; i++) {
    String name="relaytype";
    name=name+(i+1);
    if (request->hasParam(name.c_str(), true, false)) {
      AsyncWebParameter *p1 = request->getParam(name.c_str(), true, false);
      //Default
      mysettings.relaytype[i] =RELAY_STANDARD;
      if (p1->value().equals("Pulse")) {
        mysettings.relaytype[i] = RELAY_PULSE;
      }
    }
  }


  //Relay default
  for (int i = 0; i < RELAY_TOTAL; i++) {
    String name="defaultrelay";
    name=name+(i+1);
    if (request->hasParam(name.c_str(), true, false)) {
      AsyncWebParameter *p1 = request->getParam(name.c_str(), true, false);
      //Default
      mysettings.rulerelaydefault[i] =RELAY_OFF;
      if (p1->value().equals("On")) {
        mysettings.rulerelaydefault[i] = RELAY_ON;
      }
    }
  }


  for (int rule = 0; rule < RELAY_RULES; rule++) {

    //This STRING doesnt work properly if its on a single line!
    String name="rule";
    name=name+(rule+1);
    name=name+"value";

    if (request->hasParam(name, true)) {
      AsyncWebParameter *p1 = request->getParam(name, true);
      mysettings.rulevalue[rule] =p1->value().toInt();
    }

      //Rule/relay processing
      for (int i = 0; i < RELAY_TOTAL; i++) {
        //This STRING doesnt work properly if its on a single line!
        String name="rule";
        name=name+(rule+1);
        name=name+"relay";
        name=name+(i+1);
        if (request->hasParam(name, true)) {
          AsyncWebParameter *p1 = request->getParam(name, true);
          mysettings.rulerelaystate[rule][i] =p1->value().equals("X") ? RELAY_X: p1->value().equals("On") ? RELAY_ON:RELAY_OFF;
        }
      }
  }

//RELAY_TOTAL

  Settings::WriteConfigToEEPROM((char*)&mysettings, sizeof(mysettings), EEPROM_SETTINGS_START_ADDRESS);

  SendSuccess(request);
}


void DIYBMSServer::saveNTP(AsyncWebServerRequest *request) {
  if (!validateXSS(request)) return;

  if (request->hasParam("NTPServer", true)) {
    AsyncWebParameter *p1 = request->getParam("NTPServer", true);
    p1->value().toCharArray(mysettings.ntpServer,sizeof(mysettings.ntpServer));
  }

  if (request->hasParam("NTPZoneHour", true)) {
    AsyncWebParameter *p1 = request->getParam("NTPZoneHour", true);
    mysettings.timeZone  =p1->value().toInt();
  }

  if (request->hasParam("NTPZoneMin", true)) {
    AsyncWebParameter *p1 = request->getParam("NTPZoneMin", true);
    mysettings.minutesTimeZone =p1->value().toInt();
  }

  mysettings.daylight=false;
  if (request->hasParam("NTPDST", true)) {
    AsyncWebParameter *p1 = request->getParam("NTPDST", true);
    mysettings.daylight =p1->value().equals("on") ? true:false;
  }

  Settings::WriteConfigToEEPROM((char*)&mysettings, sizeof(mysettings), EEPROM_SETTINGS_START_ADDRESS);

  ConfigHasChanged = REBOOT_COUNT_DOWN;
  SendSuccess(request);
}


void DIYBMSServer::saveBankConfiguration(AsyncWebServerRequest *request) {
  if (!validateXSS(request)) return;

  if (request->hasParam("totalBanks", true)) {
    AsyncWebParameter *p1 = request->getParam("totalBanks", true);
    mysettings.totalNumberOfBanks =p1->value().toInt();
  }

  if (request->hasParam("combitype", true)) {
    AsyncWebParameter *p1 = request->getParam("combitype", true);
    mysettings.combinationParallel =p1->value().equals("Parallel") ? true:false;
  }

  Settings::WriteConfigToEEPROM((char*)&mysettings, sizeof(mysettings), EEPROM_SETTINGS_START_ADDRESS);

  //ConfigHasChanged = REBOOT_COUNT_DOWN;
  SendSuccess(request);

}


void DIYBMSServer::saveMQTTSetting(AsyncWebServerRequest *request) {
  if (!validateXSS(request)) return;

    if (request->hasParam("mqttEnabled", true)) {
      AsyncWebParameter *p1 = request->getParam("mqttEnabled", true);
      mysettings.mqtt_enabled = p1->value().equals("on") ? true:false;
    } else {
      mysettings.mqtt_enabled = false;
    }

    if (request->hasParam("mqttTopic", true)) {
      AsyncWebParameter *p1 = request->getParam("mqttTopic", true);
      p1->value().toCharArray(mysettings.mqtt_topic, sizeof(mysettings.mqtt_topic));
    } else {
      sprintf(mysettings.mqtt_topic, "diybms");
    }

    if (request->hasParam("mqttPort", true)) {
      AsyncWebParameter *p1 = request->getParam("mqttPort", true);
      mysettings.mqtt_port = p1->value().toInt();
    }

    if (request->hasParam("mqttServer", true)) {
      AsyncWebParameter *p1 = request->getParam("mqttServer", true);
      p1->value().toCharArray(mysettings.mqtt_server, sizeof(mysettings.mqtt_server));
    }

    if (request->hasParam("mqttUsername", true)) {
      AsyncWebParameter *p1 = request->getParam("mqttUsername", true);
      p1->value().toCharArray(mysettings.mqtt_username, sizeof(mysettings.mqtt_username));
    }

    if (request->hasParam("mqttPassword", true)) {
      AsyncWebParameter *p1 = request->getParam("mqttPassword", true);
      p1->value().toCharArray(mysettings.mqtt_password, sizeof(mysettings.mqtt_password));
    }

    Settings::WriteConfigToEEPROM((char*)&mysettings, sizeof(mysettings), EEPROM_SETTINGS_START_ADDRESS);

    ConfigHasChanged = REBOOT_COUNT_DOWN;
    SendSuccess(request);
}


void DIYBMSServer::saveGlobalSetting(AsyncWebServerRequest *request) {
  if (!validateXSS(request)) return;


  if (request->hasParam("BypassOverTempShutdown", true) && request->hasParam("BypassThresholdmV", true)) {

    AsyncWebParameter *p1 = request->getParam("BypassOverTempShutdown", true);
    uint8_t BypassOverTempShutdown=p1->value().toInt();

    AsyncWebParameter *p2 = request->getParam("BypassThresholdmV", true);
    uint16_t BypassThresholdmV=p2->value().toInt();

    prg.sendSaveGlobalSetting(mysettings.totalNumberOfBanks, BypassThresholdmV, BypassOverTempShutdown);

    //Just returns NULL
    SendSuccess(request);

  } else {
    request->send(500, "text/plain", "Missing parameters");
  }
}


void DIYBMSServer::handleNotFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

void DIYBMSServer::saveSetting(AsyncWebServerRequest *request) {
  if (!validateXSS(request)) return;

  if (request->hasParam("m", true) && request->hasParam("b", true)) {

    AsyncWebParameter *module = request->getParam("m", true);
    AsyncWebParameter *bank = request->getParam("b", true);

    int b = bank->value().toInt();
    int m = module->value().toInt();

    if ((b > 3) || (m + 1 > numberOfModules[b])) {
      request->send(500, "text/plain", "Wrong parameters");
    } else {

    uint8_t BypassOverTempShutdown=0xFF;
    uint16_t BypassThresholdmV=0xFFFF;

    // Resistance of bypass load
    float LoadResistance=0xFFFF;
    //Voltage Calibration
    float Calibration=0xFFFF;
    //Reference voltage (millivolt) normally 2.00mV
    float mVPerADC=0xFFFF;
    //Internal Thermistor settings
    uint16_t Internal_BCoefficient=0xFFFF;
    //External Thermistor settings
    uint16_t External_BCoefficient=0xFFFF;

    if (request->hasParam("BypassOverTempShutdown", true)) {
      AsyncWebParameter *p1 = request->getParam("BypassOverTempShutdown", true);
      BypassOverTempShutdown=p1->value().toInt();
    }

    if (request->hasParam("BypassThresholdmV", true)) {
      AsyncWebParameter *p1 = request->getParam("BypassThresholdmV", true);
      BypassThresholdmV=p1->value().toInt();
    }
    if (request->hasParam("IntBCoef", true)) {
      AsyncWebParameter *p1 = request->getParam("IntBCoef", true);
      Internal_BCoefficient=p1->value().toInt();
    }

    if (request->hasParam("ExtBCoef", true)) {
      AsyncWebParameter *p1 = request->getParam("ExtBCoef", true);
      External_BCoefficient=p1->value().toInt();
    }

    if (request->hasParam("Calib", true)) {
      AsyncWebParameter *p1 = request->getParam("Calib", true);
      Calibration=p1->value().toFloat();
    }

    if (request->hasParam("LoadRes", true)) {
      AsyncWebParameter *p1 = request->getParam("LoadRes", true);
      LoadResistance=p1->value().toFloat();
    }

    if (request->hasParam("mVPerADC", true)) {
      AsyncWebParameter *p1 = request->getParam("mVPerADC", true);
      mVPerADC=p1->value().toFloat();
    }

    prg.sendSaveSetting(b, m,BypassThresholdmV,BypassOverTempShutdown,LoadResistance,Calibration,mVPerADC,Internal_BCoefficient,External_BCoefficient);


    if (request->hasParam("movetobank", true)) {
      AsyncWebParameter *p1 = request->getParam("movetobank", true);
      int movetobank=p1->value().toInt();

      if (b!=movetobank) {
        //Requested to move this module to another bank
        prg.sendMoveToBank(b,m,movetobank);

        clearModuleValues(b,m);
        //Take 1 off and add 1 on...
        numberOfModules[b]--;
        numberOfModules[movetobank]++;
      }

    }

    SendSuccess(request);
}
  } else {
    request->send(500, "text/plain", "Missing parameters");
  }
}

void DIYBMSServer::clearModuleValues(uint8_t bank, uint8_t module) {
  cmi[bank][module].voltagemV=0;
  cmi[bank][module].voltagemVMin=6000;
  cmi[bank][module].voltagemVMax=0;
  cmi[bank][module].inBypass=false;
  cmi[bank][module].bypassOverTemp=false;
  cmi[bank][module].internalTemp=-40;
  cmi[bank][module].externalTemp=-40;
}

void DIYBMSServer::identifyModule(AsyncWebServerRequest *request) {
  if (request->hasParam("m") && request->hasParam("b")) {
    AsyncWebParameter *module = request->getParam("m");
    AsyncWebParameter *bank = request->getParam("b");

    int b = bank->value().toInt();
    int m = module->value().toInt();

    if ((b > 3) || (m + 1 > numberOfModules[b])) {
      request->send(500, "text/plain", "Wrong parameters");
    } else {
      prg.sendIdentifyModuleRequest(b, m);
      SendSuccess(request);
    }

  } else {
    request->send(500, "text/plain", "Missing parameters");
  }
}


void DIYBMSServer::rules(AsyncWebServerRequest *request) {
  AsyncResponseStream *response =
      request->beginResponseStream("application/json");

  DynamicJsonDocument doc(2048);
  JsonObject root = doc.to<JsonObject>();


#if defined(ESP8266)
 root["timenow"]=(hour() * 60) + minute();
#endif

#if defined(ESP32)
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    root["timenow"]=0;
  } else {
    root["timenow"]=(timeinfo.tm_hour * 60) + timeinfo.tm_min;
  }
#endif

  root["PCF8574"]=PCF8574Enabled;

  JsonArray defaultArray = root.createNestedArray("relaydefault");
  for (uint8_t relay = 0; relay < RELAY_TOTAL; relay++) {
    switch(mysettings.rulerelaydefault[relay]) {
      case RELAY_OFF: defaultArray.add(false);break;
      case RELAY_ON: defaultArray.add(true);break;
      default: defaultArray.add((char*)0);break;
    }
  }

  JsonArray typeArray = root.createNestedArray("relaytype");
  for (uint8_t relay = 0; relay < RELAY_TOTAL; relay++) {
    switch(mysettings.relaytype[relay]) {
      case RELAY_STANDARD: typeArray.add("Std");break;
      case RELAY_PULSE: typeArray.add("Pulse");break;
      default: typeArray.add((char*)0);break;
    }
  }


  JsonArray bankArray = root.createNestedArray("rules");

  for (uint8_t r = 0; r < RELAY_RULES; r++) {
    JsonObject rule1 = bankArray.createNestedObject();
    rule1["value"] =mysettings.rulevalue[r];
    rule1["triggered"] =rule_outcome[r];
    JsonArray data = rule1.createNestedArray("relays");

    for (uint8_t relay = 0; relay < RELAY_TOTAL; relay++) {
      switch(mysettings.rulerelaystate[r][relay]) {
        case RELAY_OFF: data.add(false);break;
        case RELAY_ON: data.add(true);break;
        default: data.add((char*)0);break;
      }
    }
  }

  serializeJson(doc, *response);
  request->send(response);
}

void DIYBMSServer::settings(AsyncWebServerRequest *request) {
  AsyncResponseStream *response =
      request->beginResponseStream("application/json");

  DynamicJsonDocument doc(2048);
  JsonObject root = doc.to<JsonObject>();

  JsonObject settings = root.createNestedObject("settings");
  settings["totalnumberofbanks"] =mysettings.totalNumberOfBanks;
  settings["combinationparallel"] =mysettings.combinationParallel;

  settings["NTPServerName"] =mysettings.ntpServer;
  settings["TimeZone"] =mysettings.timeZone;
  settings["MinutesTimeZone"] =mysettings.minutesTimeZone;
  settings["DST"] =mysettings.daylight;


#if defined(ESP8266)
  settings["now"]= now();
#endif

#if defined(ESP32)
  time_t now;
  if (time(&now)) {
    settings["now"]=now;
  }
#endif

  serializeJson(doc, *response);
  request->send(response);
}


void DIYBMSServer::integration(AsyncWebServerRequest *request) {
  AsyncResponseStream *response =
      request->beginResponseStream("application/json");

  DynamicJsonDocument doc(2048);
  JsonObject root = doc.to<JsonObject>();

  JsonObject mqtt = root.createNestedObject("mqtt");
  mqtt["enabled"] = mysettings.mqtt_enabled;
  mqtt["topic"] = mysettings.mqtt_topic;
  mqtt["port"] = mysettings.mqtt_port;
  mqtt["server"] = mysettings.mqtt_server;
  mqtt["username"] = mysettings.mqtt_username;
  //We don't output the password in the json file as this could breach security
  //mqtt["password"] =mysettings.mqtt_password;

  JsonObject influxdb = root.createNestedObject("influxdb");
  influxdb["enabled"] = mysettings.influxdb_enabled;
  influxdb["port"] = mysettings.influxdb_httpPort;
  influxdb["server"] = mysettings.influxdb_host;
  influxdb["database"] = mysettings.influxdb_database;
  influxdb["username"] = mysettings.influxdb_user;
  //We don't output the password in the json file as this could breach security
  //influxdb["password"] = mysettings.influxdb_password;

  serializeJson(doc, *response);
  request->send(response);
}


void DIYBMSServer::modules(AsyncWebServerRequest *request) {
  if (request->hasParam("m", false) && request->hasParam("b", false)) {
    AsyncWebParameter *module = request->getParam("m", false);
    AsyncWebParameter *bank = request->getParam("b", false);

    int b = bank->value().toInt();
    int m = module->value().toInt();

    if ((b > 3) || (m + 1 > numberOfModules[b])) {
      request->send(500, "text/plain", "Wrong parameters");
    } else {
      if (cmi[b][m].settingsCached == false) {
        prg.sendGetSettingsRequest(b, m);
      }

      AsyncResponseStream *response =
          request->beginResponseStream("application/json");

      DynamicJsonDocument doc(2048);
      JsonObject root = doc.to<JsonObject>();
      JsonObject settings = root.createNestedObject("settings");

      settings["bank"] = b;
      settings["module"] = m;
      settings["ver"] = cmi[b][m].BoardVersionNumber;

      settings["Cached"] = cmi[b][m].settingsCached;
      // settings["Requested"] = cmi[b][m].settingsRequested;

      settings["BypassOverTempShutdown"] = cmi[b][m].BypassOverTempShutdown;
      settings["BypassThresholdmV"] = cmi[b][m].BypassThresholdmV;

      settings["LoadRes"] = cmi[b][m].LoadResistance;
      settings["Calib"] = cmi[b][m].Calibration;
      settings["mVPerADC"] = cmi[b][m].mVPerADC;
      settings["IntBCoef"] = cmi[b][m].Internal_BCoefficient;
      settings["ExtBCoef"] = cmi[b][m].External_BCoefficient;



      serializeJson(doc, *response);
      request->send(response);
    }
  } else {
    request->send(500, "text/plain", "Missing parameters");
  }
}

/*
Restart controller from web interface
*/
void DIYBMSServer::handleRestartController(AsyncWebServerRequest *request) {
  ESP.restart();
}


void DIYBMSServer::monitor(AsyncWebServerRequest *request) {
  AsyncResponseStream *response =
      request->beginResponseStream("application/json");

  DynamicJsonDocument doc(4096);

  JsonObject root = doc.to<JsonObject>();

  root["parallel"] = mysettings.combinationParallel;
  root["banks"] = mysettings.totalNumberOfBanks;

  JsonObject monitor = root.createNestedObject("monitor");

  // Set error flag if we have attempted to send 2*number of banks without a reply
  monitor["commserr"] = receiveProc.HasCommsTimedOut();

  monitor["sent"] = prg.packetsGenerated;
  monitor["received"] = receiveProc.packetsReceived;
  monitor["badcrc"] = receiveProc.totalCRCErrors;
  monitor["ignored"] = receiveProc.totalNotProcessedErrors;
  monitor["roundtrip"] = receiveProc.packetTimerMillisecond;

  JsonArray bankArray = root.createNestedArray("bank");

  for (uint8_t bank = 0; bank < mysettings.totalNumberOfBanks; bank++) {
    JsonArray data = bankArray.createNestedArray();

    for (uint16_t i = 0; i < numberOfModules[bank]; i++) {
      JsonObject cell = data.createNestedObject();
      cell["v"] = cmi[bank][i].voltagemV;
      cell["minv"] = cmi[bank][i].voltagemVMin;
      cell["maxv"] = cmi[bank][i].voltagemVMax;
      cell["bypass"] = cmi[bank][i].inBypass;
      cell["bypasshot"] = cmi[bank][i].bypassOverTemp;
      cell["int"] = cmi[bank][i].internalTemp;
      cell["ext"] = cmi[bank][i].externalTemp;
      cell["badpkt"] = cmi[bank][i].badPacketCount;
      cell["pwm"] = cmi[bank][i].inBypass? cmi[bank][i].PWMValue:0;
    }
  }
  serializeJson(doc, *response);
  request->send(response);
}

String DIYBMSServer::TemplateProcessor(const String& var)
{
  if(var == "XSS_KEY")
    return DIYBMSServer::UUIDString;

#if defined(ESP8266)
  if(var == "PLATFORM")
    return String("ESP8266");
#endif

#if defined(ESP32)
  if(var == "PLATFORM")
    return String("ESP32");
#endif


  return String();
}

void DIYBMSServer::StartServer(AsyncWebServer *webserver) {

  _myserver = webserver;

  String cookieValue="DIYBMS_XSS=";
  cookieValue+=DIYBMSServer::UUIDString;
  cookieValue+=String("; path=/; HttpOnly");
  DefaultHeaders::Instance().addHeader("Set-Cookie", cookieValue);

  _myserver->on("/", HTTP_GET, [](AsyncWebServerRequest *request) {    
    request->redirect("/default.htm");
  });

  _myserver->serveStatic("/default.htm", SPIFFS, "/default.htm").setTemplateProcessor(DIYBMSServer::TemplateProcessor);
  _myserver->serveStatic("/files/", SPIFFS, "/files/").setCacheControl("max-age=600");

//Read endpoints
  _myserver->on("/monitor.json", HTTP_GET, DIYBMSServer::monitor);
  _myserver->on("/integration.json", HTTP_GET, DIYBMSServer::integration);
  _myserver->on("/modules.json", HTTP_GET, DIYBMSServer::modules);
  _myserver->on("/identifyModule.json", HTTP_GET, DIYBMSServer::identifyModule);
  _myserver->on("/settings.json", HTTP_GET, DIYBMSServer::settings);
  _myserver->on("/rules.json", HTTP_GET, DIYBMSServer::rules);

  //POST method endpoints
  _myserver->on("/savesetting.json", HTTP_POST, DIYBMSServer::saveSetting);
  _myserver->on("/saveglobalsetting.json", HTTP_POST, DIYBMSServer::saveGlobalSetting);
  _myserver->on("/savemqtt.json", HTTP_POST, DIYBMSServer::saveMQTTSetting);
  _myserver->on("/saveinfluxdb.json", HTTP_POST, DIYBMSServer::saveInfluxDBSetting);
  _myserver->on("/savebankconfig.json", HTTP_POST, DIYBMSServer::saveBankConfiguration);
  _myserver->on("/saverules.json", HTTP_POST, DIYBMSServer::saveRuleConfiguration);
  _myserver->on("/saventp.json", HTTP_POST, DIYBMSServer::saveNTP);

  _myserver->on("/resetcounters.json", HTTP_POST, DIYBMSServer::resetCounters);
  _myserver->on("/restartcontroller.json", HTTP_POST, DIYBMSServer::handleRestartController);

  _myserver->onNotFound(DIYBMSServer::handleNotFound);
  _myserver->begin();
}
