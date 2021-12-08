
#include "webserver.h"

#include <stdio.h>

httpd_handle_t _myserver;
String UUIDString;
String UUIDStringLast2Chars;

// Pointers to other classes (not always a good idea in static classes)
// static sdcard_info (*_sdcardcallback)();
fs::SDFS *_sdcard;
void (*_sdcardaction_callback)(uint8_t action);
PacketRequestGenerator *_prg;
PacketReceiveProcessor *_receiveProc;
diybms_eeprom_settings *_mysettings;
Rules *_rules;
ControllerState *_controlState;
HAL_ESP32 *_hal;

#define BUFSIZE 1500
// Shared buffer for all HTTP generated replies
char httpbuf[BUFSIZE];

// DIYBMS_XSS=768a5b9c-e37f-28f5-7bb4-38415df17bbf; path=/; HttpOnly
char cookie[90];

void generateUUID()
{
  uint8_t uuidNumber[16]; // UUIDs in binary form are 16 bytes long

  // ESP32 has inbuilt random number generator
  // https://techtutorialsx.com/2017/12/22/esp32-arduino-random-number-generation/
  for (uint8_t x = 0; x < 16; x++)
  {
    uuidNumber[x] = random(0xFF);
  }

  UUIDString = uuidToString(uuidNumber);

  // 481efb3f-0400-0000-101f-fb3fd01efb3f
  UUIDStringLast2Chars = UUIDString.substring(34);
}

String uuidToString(uint8_t *uuidLocation)
{
  const char hexchars[] = "0123456789abcdef";
  String string = "";
  int i;
  for (i = 0; i < 16; i++)
  {
    if (i == 4)
      string += "-";
    if (i == 6)
      string += "-";
    if (i == 8)
      string += "-";
    if (i == 10)
      string += "-";
    uint8_t topDigit = uuidLocation[i] >> 4;
    uint8_t bottomDigit = uuidLocation[i] & 0x0f;
    // High hex digit
    string += hexchars[topDigit];
    // Low hex digit
    string += hexchars[bottomDigit];
  }

  return string;
}

void setCacheControl(httpd_req_t *req)
{
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
}

String TemplateProcessor(const String &var)
{
  if (var == "XSS_KEY")
    return UUIDString;

#if defined(ESP32)
  if (var == "PLATFORM")
    return String("ESP32");
#endif

  if (var == "LANGUAGE")
    return String(_mysettings->language);

  if (var == "GIT_VERSION")
    return String(GIT_VERSION);

  if (var == "COMPILE_DATE_TIME")
    return String(COMPILE_DATE_TIME);

  if (var == "graph_voltagehigh")
    return String(_mysettings->graph_voltagehigh);

  if (var == "graph_voltagelow")
    return String(_mysettings->graph_voltagelow);

  if (var == "integrity_file_jquery_js")
    return String(integrity_file_jquery_js);

  if (var == "noofseriesmodules")
    return String(maximum_controller_cell_modules);

  if (var == "maxnumberofbanks")
    return String(maximum_number_of_banks);

  return String();
}

int printBoolean(char *buffer, size_t bufferLen, const char *fieldName, boolean value, boolean addComma)
{
  return snprintf(buffer, bufferLen,
                  "\"%s\":%s%s",
                  fieldName,
                  value ? "true" : "false",
                  addComma ? "," : "");
}

int printBoolean(char *buffer, size_t bufferLen, const char *fieldName, boolean value)
{
  return printBoolean(buffer, bufferLen, fieldName, value, true);
}

esp_err_t content_handler_avrstorage(httpd_req_t *req)
{
  httpd_resp_set_type(req, "application/json");
  setCacheControl(req);

  int bufferused = 0;

  //{"avrprog":{"avrprog":[{"board":"V400","efuse":"F4","hfuse":"D6","lfuse":"62","mcu":"1e9315","name":"fw_V400_ca58bde0.bin","ver":"ca58bde0"},{"board":"V410","efuse":"F4","hfuse":"D6","lfuse":"62","mcu":"1e9315","name":"fw_V410_ca58bde0.bin","ver":"ca58bde0"},{"board":"V420","efuse":"F4","hfuse":"D6","lfuse":"62","mcu":"1e9315","name":"fw_V420_ca58bde0.bin","ver":"ca58bde0"},{"board":"V420_SWAPR19R20","efuse":"F4","hfuse":"D6","lfuse":"62","mcu":"1e9315","name":"fw_V420_SWAPR19R20_ca58bde0.bin","ver":"ca58bde0"},{"board":"V421","efuse":"F4","hfuse":"D6","lfuse":"62","mcu":"1e9315","name":"fw_V421_ca58bde0.bin","ver":"ca58bde0"},{"board":"V421_LTO","efuse":"F4","hfuse":"D6","lfuse":"62","mcu":"1e9315","name":"fw_V421_LTO_ca58bde0.bin","ver":"ca58bde0"}]}}
  // See if we can open and process the AVR PROGRAMMER manifest file
  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "{");
  bufferused += printBoolean(&httpbuf[bufferused], BUFSIZE - bufferused, "ProgModeEnabled", _avrsettings.programmingModeEnabled);
  bufferused += printBoolean(&httpbuf[bufferused], BUFSIZE - bufferused, "InProgress", _avrsettings.inProgress);
  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "\"avrprog\":");

  String manifest = String("/avr/manifest.json");
  if (LITTLEFS.exists(manifest))
  {
    StaticJsonDocument<3000> doc;
    File file = LITTLEFS.open(manifest);
    DeserializationError error = deserializeJson(doc, file);
    if (error)
    {
      ESP_LOGE(TAG, "Error deserialize Json");
      bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "{}");
    }
    else
    {
      bufferused += serializeJson(doc, &httpbuf[bufferused], BUFSIZE - bufferused);
    }
    file.close();
  }
  else
  {
    // No files!
    bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "{}");
  }

  // The END...
  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "}");

  return httpd_resp_send(req, httpbuf, bufferused);
}

esp_err_t content_handler_currentmonitor(httpd_req_t *req)
{
  httpd_resp_set_type(req, "application/json");
  setCacheControl(req);

  int bufferused = 0;

  // Convert to milliseconds
  uint32_t x = 0;
  if (currentMonitor.validReadings)
  {
    x = (esp_timer_get_time() - currentMonitor.timestamp) / 1000;
  }

  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "{");
  bufferused += printBoolean(&httpbuf[bufferused], BUFSIZE - bufferused, "enabled", _mysettings->currentMonitoringEnabled);

  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused,
                         "\"address\":%u,\"timestampage\":%u,\"valid\":%s,\"batterycapacity\":%u,\"tailcurrent\":%.4f,\"fullchargevolt\":%.4f,\"chargeefficiency\":%.4f,",
                         _mysettings->currentMonitoringModBusAddress,
                         x,
                         currentMonitor.validReadings ? "true" : "false",
                         currentMonitor.modbus.batterycapacityamphour, currentMonitor.modbus.tailcurrentamps,
                         currentMonitor.modbus.fullychargedvoltage, currentMonitor.chargeefficiency);

  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused,
                         "\"address\":%u,\"timestampage\":%u,",
                         _mysettings->currentMonitoringModBusAddress,
                         x);
  // 1536432
  bufferused += printBoolean(&httpbuf[bufferused], BUFSIZE - bufferused, "valid", currentMonitor.validReadings);

  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused,
                         "\"batterycapacity\":%u,\"tailcurrent\":%.4f,\"fullchargevolt\":%.4f,\"chargeefficiency\":%.4f,",
                         currentMonitor.modbus.batterycapacityamphour, currentMonitor.modbus.tailcurrentamps,
                         currentMonitor.modbus.fullychargedvoltage, currentMonitor.chargeefficiency);

  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused,
                         "\"voltage\":%.4f,\"current\":%.4f,\"mahout\":%u,\"mahin\":%u,\"temperature\":%i,\"watchdog\":%u,\"power\":%.4f,\"actualshuntmv\":%.4f,\"currentlsb\":%.4f,\"resistance\":%.4f,\"calibration\":%u,\"templimit\":%i,\"undervlimit\":%.4f,\"overvlimit\":%.4f,\"overclimit\":%.4f,\"underclimit\":%.4f,\"overplimit\":%.4f,\"tempcoeff\":%u,\"model\":%u,\"firmwarev\":%u,\"firmwaredate\":%u,",
                         currentMonitor.modbus.voltage, currentMonitor.modbus.current, currentMonitor.modbus.milliamphour_out, currentMonitor.modbus.milliamphour_in,
                         currentMonitor.modbus.temperature, currentMonitor.modbus.watchdogcounter, currentMonitor.modbus.power,
                         currentMonitor.modbus.shuntmV, currentMonitor.modbus.currentlsb, currentMonitor.modbus.shuntresistance,
                         currentMonitor.modbus.shuntcal, currentMonitor.modbus.temperaturelimit,
                         currentMonitor.modbus.undervoltagelimit, currentMonitor.modbus.overvoltagelimit,
                         currentMonitor.modbus.overcurrentlimit, currentMonitor.modbus.undercurrentlimit,
                         currentMonitor.modbus.overpowerlimit, currentMonitor.modbus.shunttempcoefficient,
                         currentMonitor.modbus.modelnumber, currentMonitor.modbus.firmwareversion,
                         currentMonitor.modbus.firmwaredatetime);

  // Boolean flag values
  bufferused += printBoolean(&httpbuf[bufferused], BUFSIZE - bufferused, "TMPOL", currentMonitor.TemperatureOverLimit);
  bufferused += printBoolean(&httpbuf[bufferused], BUFSIZE - bufferused, "CURROL", currentMonitor.CurrentOverLimit);
  bufferused += printBoolean(&httpbuf[bufferused], BUFSIZE - bufferused, "CURRUL", currentMonitor.CurrentUnderLimit);
  bufferused += printBoolean(&httpbuf[bufferused], BUFSIZE - bufferused, "VOLTOL", currentMonitor.VoltageOverlimit);
  bufferused += printBoolean(&httpbuf[bufferused], BUFSIZE - bufferused, "VOLTUL", currentMonitor.VoltageUnderlimit);
  bufferused += printBoolean(&httpbuf[bufferused], BUFSIZE - bufferused, "POL", currentMonitor.PowerOverLimit);
  bufferused += printBoolean(&httpbuf[bufferused], BUFSIZE - bufferused, "TempCompEnabled", currentMonitor.TempCompEnabled);
  bufferused += printBoolean(&httpbuf[bufferused], BUFSIZE - bufferused, "ADCRange4096mV", currentMonitor.ADCRange4096mV);

  // Trigger values
  bufferused += printBoolean(&httpbuf[bufferused], BUFSIZE - bufferused, "T_TMPOL", currentMonitor.RelayTriggerTemperatureOverLimit);
  bufferused += printBoolean(&httpbuf[bufferused], BUFSIZE - bufferused, "T_CURROL", currentMonitor.RelayTriggerCurrentOverLimit);
  bufferused += printBoolean(&httpbuf[bufferused], BUFSIZE - bufferused, "T_CURRUL", currentMonitor.RelayTriggerCurrentUnderLimit);
  bufferused += printBoolean(&httpbuf[bufferused], BUFSIZE - bufferused, "T_VOLTOL", currentMonitor.RelayTriggerVoltageOverlimit);
  bufferused += printBoolean(&httpbuf[bufferused], BUFSIZE - bufferused, "T_VOLTUL", currentMonitor.RelayTriggerVoltageUnderlimit);
  bufferused += printBoolean(&httpbuf[bufferused], BUFSIZE - bufferused, "T_POL", currentMonitor.RelayTriggerPowerOverLimit);
  bufferused += printBoolean(&httpbuf[bufferused], BUFSIZE - bufferused, "RelayState", currentMonitor.RelayState);

  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused,
                         "\"shuntmv\":%u,\"shuntmaxcur\":%u}",
                         currentMonitor.modbus.shuntmillivolt, currentMonitor.modbus.shuntmaxcurrent);

  return httpd_resp_send(req, httpbuf, bufferused);
}

esp_err_t content_handler_rs485settings(httpd_req_t *req)
{
  httpd_resp_set_type(req, "application/json");
  setCacheControl(req);

  int bufferused = 0;

  // Output the first batch of settings/parameters/values
  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE,
                         "{\"baudrate\":%i,\"databits\":%i,\"parity\":%i,\"stopbits\":%i}",
                         _mysettings->rs485baudrate, _mysettings->rs485databits,
                         _mysettings->rs485parity, _mysettings->rs485stopbits);

  return httpd_resp_send(req, httpbuf, bufferused);
}

int fileSystemListDirectory(char *buffer, size_t bufferLen, fs::FS &fs, const char *dirname, uint8_t levels)
{
  // This needs to check for buffer overrun as too many files are likely to exceed the buffer capacity

  int bufferused = 0;
  File root = fs.open(dirname);
  if (!root)
  {
    ESP_LOGE(TAG, "Failed to open dir");
    return 0;
  }
  if (!root.isDirectory())
  {
    ESP_LOGE(TAG, "Not a dir");
    return 0;
  }

  File file = root.openNextFile();
  while (file)
  {
    if (file.isDirectory())
    {
      // Hide the diybms folder where the config files are kept
      if (levels && String(file.name()).startsWith("/diybms") == false)
      {
        bufferused += fileSystemListDirectory(&buffer[bufferused], bufferLen - bufferused, fs, file.name(), levels - 1);
        bufferused += snprintf(&buffer[bufferused], bufferLen - bufferused, ",");
      }
    }
    else
    {
      bufferused += snprintf(&buffer[bufferused], bufferLen - bufferused, "\"%s\",", file.name());
    }

    file = root.openNextFile();
  }

  // Trailing null to cope with trailing ','
  bufferused += snprintf(&buffer[bufferused], bufferLen - bufferused, "null");

  return bufferused;
}

esp_err_t content_handler_storage(httpd_req_t *req)
{
  httpd_resp_set_type(req, "application/json");
  setCacheControl(req);

  int bufferused = 0;

  sdcard_info info;

  info.available = _sd_card_installed;

  // Lock VSPI bus during operation (not sure if this is acutally needed, as the SD class may have cached these values)
  if (_hal->GetVSPIMutex())
  {
    // Convert to KiB
    info.totalkilobytes = SD.totalBytes() / 1024;
    info.usedkilobytes = SD.usedBytes() / 1024;
    _hal->ReleaseVSPIMutex();
  }
  else
  {
    info.totalkilobytes = 0;
    info.usedkilobytes = 0;
  }

  info.flash_totalkilobytes = LITTLEFS.totalBytes() / 1024;
  info.flash_usedkilobytes = LITTLEFS.usedBytes() / 1024;

  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "{\"storage\":{");
  bufferused += printBoolean(&httpbuf[bufferused], BUFSIZE - bufferused, "logging", _mysettings->loggingEnabled);
  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "\"frequency\":%u,\"sdcard\":{", _mysettings->loggingFrequencySeconds);
  bufferused += printBoolean(&httpbuf[bufferused], BUFSIZE - bufferused, "available", info.available);
  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "\"total\":%u,\"used\":%u,\"files\":[", info.totalkilobytes, info.usedkilobytes);

  //  Send it...
  httpd_resp_send_chunk(req, httpbuf, bufferused);
  bufferused = 0;

  // File listing goes here
  if (info.available)
  {
    if (_hal->GetVSPIMutex())
    {
      bufferused += fileSystemListDirectory(&httpbuf[bufferused], BUFSIZE - bufferused, SD, "/", 2);
      _hal->ReleaseVSPIMutex();
    }
  }

  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "]},\"flash\":{");

  //  Send it...
  httpd_resp_send_chunk(req, httpbuf, bufferused);

  bufferused = 0;

  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "\"total\":%u,\"used\":%u,\"files\":[", info.flash_totalkilobytes, info.flash_usedkilobytes);

  bufferused += fileSystemListDirectory(&httpbuf[bufferused], BUFSIZE - bufferused, LITTLEFS, "/", 0);

  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "]}}}");

  // ESP_LOGD(TAG, "bufferused=%i", bufferused);  ESP_LOGD(TAG, "monitor2: %s", buf);
  //  Send it...
  httpd_resp_send_chunk(req, httpbuf, bufferused);

  // Indicate last chunk (zero byte length)
  return httpd_resp_send_chunk(req, httpbuf, 0);
}


//TODO: FINISH THIS OFF
esp_err_t content_handler_downloadfile(httpd_req_t *req)
{
  bool valid = false;

  char param[128];
  String type;
  String file;

  char buf[256];
  size_t buf_len = httpd_req_get_url_query_len(req);
  if (buf_len > 1)
  {
    // Only allow up to our pre-defined buffer length (100 bytes)
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK)
    {
      // ESP_LOGD(TAG, "Found URL query => %s", buf);

      /* Get value of expected key from query string */
      if (httpd_query_key_value(buf, "type", param, sizeof(param)) == ESP_OK)
      {
        type = String(param);
        ESP_LOGD(TAG, "type=%s", type.c_str());

        if (httpd_query_key_value(buf, "file", param, sizeof(param)) == ESP_OK)
        {
          file = String(param);
          ESP_LOGD(TAG, "file=%s", file.c_str());
          valid = true;
        }
      }
    }
  }

  if (valid)
  {
    // Do some really simple validation here to prevent files being downloaded,
    // which could be a security risk.
    // See: directory traversal vulnerability
    if (file.startsWith("/") == false)
    {
      // All file names must start at the root
      return httpd_resp_send_err(req, httpd_err_code_t::HTTPD_400_BAD_REQUEST, "Bad request");
    }

    if (file.startsWith("//") == true)
    {
      // All file names must start at the root
      return httpd_resp_send_err(req, httpd_err_code_t::HTTPD_400_BAD_REQUEST, "Bad request");
    }

    if (file.startsWith("/diybms/") == true)
    {
      // Prevent downloads from /diybms/ folder
      // request->send(401); // 401 Unauthorized
      return httpd_resp_send_err(req, httpd_err_code_t::HTTPD_400_BAD_REQUEST, "Bad request");
    }

    if (type.equals("sdcard") && _sd_card_installed)
    {
      // Process file from SD card

      if (_hal->GetVSPIMutex())
      {
        // Get the file
        ESP_LOGI(TAG, "Download SDCard file %s", file.c_str());
        httpd_resp_set_type(req, "application/octet-stream");

        // request->send(*_sdcard, file, "application/octet-stream", true, nullptr);
        // httpd_resp_send_chunk(req, httpbuf, bufferused);

        _hal->ReleaseVSPIMutex();
        // Indicate last chunk (zero byte length)
        return httpd_resp_send_chunk(req, httpbuf, 0);
      }
    }

    if (type.equals("flash"))
    {
      // Process file from flash storage
      ESP_LOGI(TAG, "Download FLASH file %s", file.c_str());

      httpd_resp_set_type(req, "application/octet-stream");

      // request->send(LITTLEFS, file, "application/octet-stream", true, nullptr);
      //  httpd_resp_send_chunk(req, httpbuf, bufferused);

      // Indicate last chunk (zero byte length)
      return httpd_resp_send_chunk(req, httpbuf, 0);
    }
  }

  return httpd_resp_send_err(req, httpd_err_code_t::HTTPD_400_BAD_REQUEST, "Bad request");
}

esp_err_t content_handler_identifymodule(httpd_req_t *req)
{
  uint8_t c;
  bool valid = false;

  char buf[100];
  size_t buf_len = httpd_req_get_url_query_len(req);
  if (buf_len > 1)
  {
    // Only allow up to our pre-defined buffer length (100 bytes)
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK)
    {
      ESP_LOGD(TAG, "Found URL query => %s", buf);
      char param[8];
      /* Get value of expected key from query string */
      if (httpd_query_key_value(buf, "c", param, sizeof(param)) == ESP_OK)
      {
        c = atoi(param);

        ESP_LOGD(TAG, "Found URL query parameter => query1=%s (%u)", param, c);

        if (c <= _mysettings->totalNumberOfBanks * _mysettings->totalNumberOfSeriesModules)
        {
          valid = true;
        }
      }
    }
  }

  if (valid)
  {
    _prg->sendIdentifyModuleRequest(c);
    return SendSuccess(req);
  }
  else
  {
    // It failed!
    return httpd_resp_send_500(req);
  }
}

esp_err_t SendSuccess(httpd_req_t *req)
{
  StaticJsonDocument<100> doc;
  doc["success"] = true;
  int bufferused = 0;
  bufferused += serializeJson(doc, httpbuf, BUFSIZE);
  return httpd_resp_send(req, httpbuf, bufferused);
}

esp_err_t content_handler_modules(httpd_req_t *req)
{
  uint8_t c;
  bool valid = false;

  char buf[100];
  size_t buf_len = httpd_req_get_url_query_len(req);
  if (buf_len > 1)
  {
    // Only allow up to our pre-defined buffer length (100 bytes)
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK)
    {
      ESP_LOGI(TAG, "Found URL query => %s", buf);
      char param[8];
      /* Get value of expected key from query string */
      if (httpd_query_key_value(buf, "c", param, sizeof(param)) == ESP_OK)
      {
        c = atoi(param);

        ESP_LOGI(TAG, "Found URL query parameter => query1=%s (%u)", param, c);

        if (c <= _mysettings->totalNumberOfBanks * _mysettings->totalNumberOfSeriesModules)
        {
          valid = true;
        }
      }
    }
  }

  if (valid)
  {
    httpd_resp_set_type(req, "application/json");
    setCacheControl(req);

    if (cmi[c].settingsCached == false)
    {
      _prg->sendGetSettingsRequest(c);
    }

    DynamicJsonDocument doc(2048);
    JsonObject root = doc.to<JsonObject>();
    JsonObject settings = root.createNestedObject("settings");

    uint8_t b = c / _mysettings->totalNumberOfSeriesModules;
    uint8_t m = c - (b * _mysettings->totalNumberOfSeriesModules);
    settings["bank"] = b;
    settings["module"] = m;
    settings["id"] = c;
    settings["ver"] = cmi[c].BoardVersionNumber;
    settings["code"] = cmi[c].CodeVersionNumber;
    settings["Cached"] = cmi[c].settingsCached;

    if (cmi[c].settingsCached)
    {
      settings["BypassOverTempShutdown"] = cmi[c].BypassOverTempShutdown;
      settings["BypassThresholdmV"] = cmi[c].BypassThresholdmV;
      settings["LoadRes"] = cmi[c].LoadResistance;
      settings["Calib"] = cmi[c].Calibration;
      settings["mVPerADC"] = cmi[c].mVPerADC;
      settings["IntBCoef"] = cmi[c].Internal_BCoefficient;
      settings["ExtBCoef"] = cmi[c].External_BCoefficient;
    }

    int bufferused = 0;
    bufferused += serializeJson(doc, httpbuf, BUFSIZE);
    return httpd_resp_send(req, httpbuf, bufferused);
  }
  else
  {
    // It failed!
    return httpd_resp_send_500(req);
  }
}

esp_err_t content_handler_avrstatus(httpd_req_t *req)
{
  httpd_resp_set_type(req, "application/json");
  setCacheControl(req);

  int bufferused = 0;
  StaticJsonDocument<256> doc;
  doc["inprogress"] = _avrsettings.inProgress ? 1 : 0;
  doc["result"] = _avrsettings.progresult;
  doc["duration"] = _avrsettings.duration;
  doc["size"] = _avrsettings.programsize;
  doc["mcu"] = _avrsettings.mcu;

  bufferused += serializeJson(doc, httpbuf, BUFSIZE);

  return httpd_resp_send(req, httpbuf, bufferused);
}

esp_err_t content_handler_getvictron(httpd_req_t *req)
{
  httpd_resp_set_type(req, "application/json");
  setCacheControl(req);

  int bufferused = 0;

  DynamicJsonDocument doc(2048);
  JsonObject root = doc.to<JsonObject>();

  JsonObject settings = root.createNestedObject("victron");

  settings["enabled"] = _mysettings->VictronEnabled;

  JsonArray cvl = settings.createNestedArray("cvl");
  JsonArray ccl = settings.createNestedArray("ccl");
  JsonArray dcl = settings.createNestedArray("dcl");
  for (uint8_t i = 0; i < 3; i++)
  {
    cvl.add(_mysettings->cvl[i]);
    ccl.add(_mysettings->ccl[i]);
    dcl.add(_mysettings->dcl[i]);
  }

  bufferused += serializeJson(doc, httpbuf, BUFSIZE);

  return httpd_resp_send(req, httpbuf, bufferused);
}

esp_err_t content_handler_rules(httpd_req_t *req)
{
  httpd_resp_set_type(req, "application/json");
  setCacheControl(req);

  int bufferused = 0;

  DynamicJsonDocument doc(2048);
  JsonObject root = doc.to<JsonObject>();

  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 100))
  {
    root["timenow"] = 0;
  }
  else
  {
    root["timenow"] = (timeinfo.tm_hour * 60) + timeinfo.tm_min;
  }

  root["ControlState"] = (*_controlState);

  JsonArray defaultArray = root.createNestedArray("relaydefault");
  for (uint8_t relay = 0; relay < RELAY_TOTAL; relay++)
  {
    switch (_mysettings->rulerelaydefault[relay])
    {
    case RELAY_OFF:
      defaultArray.add(false);
      break;
    case RELAY_ON:
      defaultArray.add(true);
      break;
    default:
      defaultArray.add((char *)0);
      break;
    }
  }

  JsonArray typeArray = root.createNestedArray("relaytype");
  for (uint8_t relay = 0; relay < RELAY_TOTAL; relay++)
  {
    switch (_mysettings->relaytype[relay])
    {
    case RELAY_STANDARD:
      typeArray.add("Std");
      break;
    case RELAY_PULSE:
      typeArray.add("Pulse");
      break;
    default:
      typeArray.add((char *)0);
      break;
    }
  }

  JsonArray bankArray = root.createNestedArray("rules");

  for (uint8_t r = 0; r < RELAY_RULES; r++)
  {
    JsonObject rule = bankArray.createNestedObject();
    rule["value"] = _mysettings->rulevalue[r];
    rule["hysteresis"] = _mysettings->rulehysteresis[r];
    rule["triggered"] = _rules->rule_outcome[r];
    JsonArray data = rule.createNestedArray("relays");

    for (uint8_t relay = 0; relay < RELAY_TOTAL; relay++)
    {
      switch (_mysettings->rulerelaystate[r][relay])
      {
      case RELAY_OFF:
        data.add(false);
        break;
      case RELAY_ON:
        data.add(true);
        break;
      default:
        data.add((char *)0);
        break;
      }
    }
  }

  bufferused += serializeJson(doc, httpbuf, BUFSIZE);

  return httpd_resp_send(req, httpbuf, bufferused);
}

esp_err_t content_handler_settings(httpd_req_t *req)
{
  httpd_resp_set_type(req, "application/json");
  setCacheControl(req);

  int bufferused = 0;

  DynamicJsonDocument doc(2048);
  JsonObject root = doc.to<JsonObject>();

  JsonObject settings = root.createNestedObject("settings");

  settings["totalnumberofbanks"] = _mysettings->totalNumberOfBanks;
  settings["totalseriesmodules"] = _mysettings->totalNumberOfSeriesModules;
  settings["baudrate"] = _mysettings->baudRate;

  settings["bypassthreshold"] = _mysettings->BypassThresholdmV;
  settings["bypassovertemp"] = _mysettings->BypassOverTempShutdown;

  settings["NTPServerName"] = _mysettings->ntpServer;
  settings["TimeZone"] = _mysettings->timeZone;
  settings["MinutesTimeZone"] = _mysettings->minutesTimeZone;
  settings["DST"] = _mysettings->daylight;

  settings["FreeHeap"] = ESP.getFreeHeap();
  settings["MinFreeHeap"] = ESP.getMinFreeHeap();
  settings["HeapSize"] = ESP.getHeapSize();
  settings["SdkVersion"] = ESP.getSdkVersion();

  settings["HostName"] = WiFi.getHostname();

  time_t now;
  if (time(&now))
  {
    settings["now"] = now;
  }

  bufferused += serializeJson(doc, httpbuf, BUFSIZE);

  return httpd_resp_send(req, httpbuf, bufferused);
}

esp_err_t content_handler_integration(httpd_req_t *req)
{
  httpd_resp_set_type(req, "application/json");
  setCacheControl(req);

  int bufferused = 0;
  /*
    const char *nullstring = "null";

    // Output the first batch of settings/parameters/values
    bufferused += snprintf(&httpbuf[bufferused], BUFSIZE,
                           "{\"mqtt\":{\"enabled\":%s,\"topic\":\"%s\",\"port\":%u,\"server\":\"%s\",\"username\":\"%s\"}",
                           _mysettings->mqtt_enabled ? "true" : "false", _mysettings->mqtt_topic, _mysettings->mqtt_port, _mysettings->mqtt_server, _mysettings->mqtt_username);

    bufferused += snprintf(&httpbuf[bufferused], BUFSIZE,
                           ",\"influxdb\":{\"enabled\":%s,\"url\":\"%s\",\"bucket\":\"%s\",\"apitoken\":\"%s\",\"orgid\":\"%s\"}}",
                           _mysettings->influxdb_enabled ? "true" : "false", _mysettings->influxdb_serverurl, _mysettings->influxdb_databasebucket, _mysettings->influxdb_apitoken, _mysettings->influxdb_orgid);

    // ESP_LOGD(TAG, "bufferused=%i", bufferused);  ESP_LOGD(TAG, "monitor2: %s", buf);
    //  Send it...
    httpd_resp_send_chunk(req, httpbuf, bufferused);

    // Indicate last chunk (zero byte length)
    return httpd_resp_send_chunk(req, buf, 0);
  */

  DynamicJsonDocument doc(1024);
  JsonObject root = doc.to<JsonObject>();

  JsonObject mqtt = root.createNestedObject("mqtt");
  mqtt["enabled"] = _mysettings->mqtt_enabled;
  mqtt["topic"] = _mysettings->mqtt_topic;
  mqtt["port"] = _mysettings->mqtt_port;
  mqtt["server"] = _mysettings->mqtt_server;
  mqtt["username"] = _mysettings->mqtt_username;
  // We don't output the password in the json file as this could breach security
  // mqtt["password"] =_mysettings->mqtt_password;

  JsonObject influxdb = root.createNestedObject("influxdb");
  influxdb["enabled"] = _mysettings->influxdb_enabled;
  influxdb["url"] = _mysettings->influxdb_serverurl;
  influxdb["bucket"] = _mysettings->influxdb_databasebucket;
  influxdb["apitoken"] = _mysettings->influxdb_apitoken;
  influxdb["orgid"] = _mysettings->influxdb_orgid;

  bufferused += serializeJson(doc, httpbuf, BUFSIZE);

  return httpd_resp_send(req, httpbuf, bufferused);
}

esp_err_t content_handler_monitor3(httpd_req_t *req)
{
  httpd_resp_set_type(req, "application/json");
  setCacheControl(req);

  uint8_t totalModules = _mysettings->totalNumberOfBanks * _mysettings->totalNumberOfSeriesModules;
  uint8_t comma = totalModules - 1;

  int bufferused = 0;
  const char *nullstring = "null";

  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "{\"badpacket\":[");

  for (uint8_t i = 0; i < totalModules; i++)
  {
    if (cmi[i].valid)
    {
      bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "%u", cmi[i].badPacketCount);
    }
    else
    {
      // Return NULL
      bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "%s", nullstring);
    }
    if (i < comma)
    {
      bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, ",");
    }
  }
  //  Send it...
  httpd_resp_send_chunk(req, httpbuf, bufferused);

  // voltrange
  bufferused = 0;

  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "],\"balcurrent\":[");

  for (uint8_t i = 0; i < totalModules; i++)
  {
    if (cmi[i].valid)
    {
      bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "%u", cmi[i].BalanceCurrentCount);
    }
    else
    {
      // Return NULL
      bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "%s", nullstring);
    }
    if (i < comma)
    {
      bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, ",");
    }
  }

  //  Send it...
  httpd_resp_send_chunk(req, httpbuf, bufferused);

  // voltrange
  bufferused = 0;
  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "],\"pktrecvd\":[");

  for (uint8_t i = 0; i < totalModules; i++)
  {
    if (cmi[i].valid)
    {
      bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "%u", cmi[i].PacketReceivedCount);
    }
    else
    {
      // Return NULL
      bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "%s", nullstring);
    }
    if (i < comma)
    {
      bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, ",");
    }
  }

  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "]}");

  // ESP_LOGD(TAG, "bufferused=%i", bufferused);  ESP_LOGD(TAG, "monitor2: %s", buf);
  //  Send it...
  httpd_resp_send_chunk(req, httpbuf, bufferused);

  // Indicate last chunk (zero byte length)
  return httpd_resp_send_chunk(req, httpbuf, 0);
}

esp_err_t content_handler_monitor2(httpd_req_t *req)
{
  httpd_resp_set_type(req, "application/json");
  setCacheControl(req);

  uint8_t totalModules = _mysettings->totalNumberOfBanks * _mysettings->totalNumberOfSeriesModules;

  int bufferused = 0;
  const char *nullstring = "null";

  // Output the first batch of settings/parameters/values
  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE,
                         "{\"banks\":%u,\"seriesmodules\":%u,\"sent\":%u,\"received\":%u,\"modulesfnd\":%u,\"badcrc\":%u,\"ignored\":%u,\"roundtrip\":%u,\"oos\":%u,\"activerules\":%u,\"uptime\":%u,\"can_fail\":%u,\"can_sent\":%u,\"can_rec\":%u,\"sec\":\"%s\",",
                         _mysettings->totalNumberOfBanks, _mysettings->totalNumberOfSeriesModules,
                         _prg->packetsGenerated, _receiveProc->packetsReceived,
                         _receiveProc->totalModulesFound, _receiveProc->totalCRCErrors,
                         _receiveProc->totalNotProcessedErrors,
                         _receiveProc->packetTimerMillisecond, _receiveProc->totalOutofSequenceErrors, _rules->active_rule_count, (uint32_t)(esp_timer_get_time() / (uint64_t)1e+6), canbus_messages_failed_sent, canbus_messages_sent, canbus_messages_received, UUIDStringLast2Chars.c_str());

  // current
  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "\"current\":[");

  if (_mysettings->currentMonitoringEnabled && currentMonitor.validReadings)
  {

    // Output current monitor values, this is inside an array, so could be more than 1
    bufferused += snprintf(&httpbuf[bufferused], BUFSIZE,
                           "{\"c\":%.4f,\"v\":%.4f,\"mahout\":%u,\"mahin\":%u,\"p\":%.2f,\"soc\":%.2f}",
                           currentMonitor.modbus.current, currentMonitor.modbus.voltage, currentMonitor.modbus.milliamphour_out,
                           currentMonitor.modbus.milliamphour_in, currentMonitor.modbus.power, currentMonitor.stateofcharge);
  }
  else
  {
    bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "%s", nullstring);
  }

  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "],");

  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE, "\"errors\":[");
  uint8_t count = 0;

  for (size_t i = 0; i < sizeof(_rules->ErrorCodes); i++)
  {
    if (_rules->ErrorCodes[i] != InternalErrorCode::NoError)
    {
      // Comma if not zero
      if (count)
      {
        bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, ",");
      }
      bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "%u", _rules->ErrorCodes[i]);
      count++;
    }
  }
  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "],\"warnings\":[");

  count = 0;
  for (size_t i = 0; i < sizeof(_rules->WarningCodes); i++)
  {
    if (_rules->WarningCodes[i] != InternalWarningCode::NoWarning)
    {
      // Comma if not zero
      if (count)
      {
        bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, ",");
      }

      bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "%u", _rules->WarningCodes[i]);
      count++;
    }
  }
  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "],");

  // ESP_LOGD(TAG, "bufferused=%i", bufferused);  ESP_LOGD(TAG, "monitor2: %s", buf);

  // Send it...
  httpd_resp_send_chunk(req, httpbuf, bufferused);

  // voltages
  bufferused = 0;
  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "\"voltages\":[");

  for (uint8_t i = 0; i < totalModules; i++)
  {
    // Comma if not zero
    if (i)
    {
      bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, ",");
    }

    if (cmi[i].valid)
    {
      bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "%u", cmi[i].voltagemV);
    }
    else
    {
      // Module is not yet valid so return null values...
      bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "%s", nullstring);
    }
  }
  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "],");

  // ESP_LOGD(TAG, "bufferused=%i", bufferused);  ESP_LOGD(TAG, "monitor2: %s", buf);

  // Send it...
  httpd_resp_send_chunk(req, httpbuf, bufferused);

  bufferused = 0;
  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "\"minvoltages\":[");
  for (uint8_t i = 0; i < totalModules; i++)
  {
    // Comma if not zero
    if (i)
      bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, ",");

    if (cmi[i].valid)
    {
      bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "%u", cmi[i].voltagemVMin);
    }
    else
    {
      // Module is not yet valid so return null values...
      bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "%s", nullstring);
    }
  }
  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "],");

  // ESP_LOGD(TAG, "bufferused=%i", bufferused);  ESP_LOGD(TAG, "monitor2: %s", buf);
  //  Send it...
  httpd_resp_send_chunk(req, httpbuf, bufferused);

  // maxvoltages
  bufferused = 0;
  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "\"maxvoltages\":[");

  for (uint8_t i = 0; i < totalModules; i++)
  {
    // Comma if not zero
    if (i)
      bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, ",");

    if (cmi[i].valid)
    {
      bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "%u", cmi[i].voltagemVMin);
    }
    else
    {
      // Module is not yet valid so return null values...
      bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "%s", nullstring);
    }
  }
  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "],");

  // ESP_LOGD(TAG, "bufferused=%i", bufferused);  ESP_LOGD(TAG, "monitor2: %s", buf);
  //  Send it...
  httpd_resp_send_chunk(req, httpbuf, bufferused);

  // inttemp
  bufferused = 0;
  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "\"inttemp\":[");

  for (uint8_t i = 0; i < totalModules; i++)
  {
    // Comma if not zero
    if (i)
      bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, ",");

    if (cmi[i].valid && cmi[i].internalTemp != -40)
    {
      bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "%i", cmi[i].internalTemp);
    }
    else
    {
      // Module is not yet valid so return null values...
      bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "%s", nullstring);
    }
  }
  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "],");

  // ESP_LOGD(TAG, "bufferused=%i", bufferused);  ESP_LOGD(TAG, "monitor2: %s", buf);
  //  Send it...
  httpd_resp_send_chunk(req, httpbuf, bufferused);

  // exttemp
  bufferused = 0;
  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "\"exttemp\":[");

  for (uint8_t i = 0; i < totalModules; i++)
  {
    // Comma if not zero
    if (i)
      bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, ",");

    if (cmi[i].valid && cmi[i].externalTemp != -40)
    {
      bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "%i", cmi[i].externalTemp);
    }
    else
    {
      // Module is not yet valid so return null values...
      bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "%s", nullstring);
    }
  }
  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "],");

  // ESP_LOGD(TAG, "bufferused=%i", bufferused);  ESP_LOGD(TAG, "monitor2: %s", buf);
  //  Send it...
  httpd_resp_send_chunk(req, httpbuf, bufferused);

  // bypass
  bufferused = 0;
  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "\"bypass\":[");

  for (uint8_t i = 0; i < totalModules; i++)
  {
    // Comma if not zero
    if (i)
      bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, ",");

    if (cmi[i].valid && cmi[i].inBypass)
    {
      bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "1");
    }
    else
    {
      bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "0");
    }
  }
  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "],");

  // ESP_LOGD(TAG, "bufferused=%i", bufferused);  ESP_LOGD(TAG, "monitor2: %s", buf);
  //  Send it...
  httpd_resp_send_chunk(req, httpbuf, bufferused);

  // bypasshot
  bufferused = 0;
  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "\"bypasshot\":[");

  for (uint8_t i = 0; i < totalModules; i++)
  {
    // Comma if not zero
    if (i)
      bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, ",");

    if (cmi[i].valid && cmi[i].bypassOverTemp)
    {
      bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "1");
    }
    else
    {
      bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "0");
    }
  }
  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "],");

  // ESP_LOGD(TAG, "bufferused=%i", bufferused);  ESP_LOGD(TAG, "monitor2: %s", buf);
  //  Send it...
  httpd_resp_send_chunk(req, httpbuf, bufferused);

  // bypasspwm
  bufferused = 0;
  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "\"bypasspwm\":[");

  for (uint8_t i = 0; i < totalModules; i++)
  {
    // Comma if not zero
    if (i)
      bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, ",");

    if (cmi[i].valid && cmi[i].inBypass)
    {
      bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "%u", cmi[i].PWMValue);
    }
    else
    {
      bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "0");
    }
  }

  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "],");

  // ESP_LOGD(TAG, "bufferused=%i", bufferused);  ESP_LOGD(TAG, "monitor2: %s", buf);
  //  Send it...
  httpd_resp_send_chunk(req, httpbuf, bufferused);

  // bankv
  bufferused = 0;
  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "\"bankv\":[");

  for (uint8_t i = 0; i < _mysettings->totalNumberOfBanks; i++)
  {
    // Comma if not zero
    if (i)
      bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, ",");

    bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "%u", _rules->packvoltage[i]);
  }
  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "],");

  // ESP_LOGD(TAG, "bufferused=%i", bufferused);  ESP_LOGD(TAG, "monitor2: %s", buf);
  //  Send it...
  httpd_resp_send_chunk(req, httpbuf, bufferused);

  // voltrange
  bufferused = 0;
  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "\"voltrange\":[");

  for (uint8_t i = 0; i < _mysettings->totalNumberOfBanks; i++)
  {
    // Comma if not zero
    if (i)
    {
      bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, ",");
    }

    bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "%u", _rules->VoltageRangeInBank(i));
  }
  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "]}");

  // ESP_LOGD(TAG, "bufferused=%i", bufferused);  ESP_LOGD(TAG, "monitor2: %s", buf);
  //  Send it...
  httpd_resp_send_chunk(req, httpbuf, bufferused);

  // Indicate last chunk (zero byte length)
  return httpd_resp_send_chunk(req, httpbuf, 0);
}

// The main home page
esp_err_t default_htm_handler(httpd_req_t *req)
{
  httpd_resp_set_type(req, "text/html");
  setCacheControl(req);
  setCookie(req);

  char *file_pointer = (char *)file_default_htm;
  size_t max_len = size_file_default_htm;
  char *end_pointer = file_pointer + max_len;

  // Seek out the template strings to inline replace
  // this is likely to crash with poorly formatted HTML input
  ESP_LOGI(TAG, "default.htm");

  char *p = file_pointer;

  while (p < end_pointer)
  {
    const char *templateCharacter = "%";

    // Find the next escape character (start of a template keyword)
    char *start_ptr = (char *)memchr(p, '%', end_pointer - p);

    if (start_ptr != NULL)
    {
      // Output to client up to the start of the template string
      httpd_resp_send_chunk(req, p, start_ptr - p);

      // Skip over template character %
      p = start_ptr + 1;

      // Find the end of the template
      char *end_ptr = (char *)memchr(p, '%', end_pointer - p);

      if (end_ptr != NULL)
      {
        // Length of the template keyword
        int paramNameLength = end_ptr - p;

        if (paramNameLength > 0)
        {
          char buf[50 + 1];
          memcpy(buf, p, paramNameLength);
          buf[paramNameLength] = 0;
          String paramName = String(buf);
          String templateValue = TemplateProcessor(paramName);

          ESP_LOGD(TAG, "Template %s = '%s'", paramName.c_str(), templateValue.c_str());

          // Output the template answer
          httpd_resp_sendstr_chunk(req, templateValue.c_str());
        }
        else
        {
          // Its an empty keyword %% - so just output a % character
          httpd_resp_sendstr_chunk(req, templateCharacter);
        }

        end_ptr++;
        p = end_ptr;
      }
      else
      {
        break;
      }
    }
    else
    {
      break;
    }
  }

  // output the last chunk
  if (p < end_pointer)
  {
    httpd_resp_send_chunk(req, p, end_pointer - p);
  }

  // Indicate last chunk (zero byte length)
  return httpd_resp_send_chunk(req, p, 0);
}

void SetCacheAndETag(httpd_req_t *req, const char *ETag)
{
  httpd_resp_set_hdr(req, "ETag", ETag);
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache, max-age=86400");
}

typedef struct
{
  const uint8_t *resp;
  size_t resp_len;
  const char *etag;
  const char *mimetype;
} WEBKIT_RESPONSE_ARGS;

const char *const text_css = "text/css";
const char *const application_javascript = "application/javascript";
const char *const image_png = "image/png";
const char *const image_x_icon = "image/x-icon";

void setCookieValue()
{
  snprintf(cookie, sizeof(cookie), "DIYBMS_XSS=%s; path=/; HttpOnly", UUIDString.c_str());
}

void setCookie(httpd_req_t *req)
{
  return;
  httpd_resp_set_hdr(req, "Set-Cookie", cookie);
}

// Handle static files which are NOT GZIP compressed
esp_err_t static_content_handler(httpd_req_t *req)
{
  WEBKIT_RESPONSE_ARGS *args = (WEBKIT_RESPONSE_ARGS *)(req->user_ctx);

  httpd_resp_set_type(req, args->mimetype);

  // TODO: GET ETAGS WORKING AGAIN!!
  char buffer[50];

  if (httpd_req_get_hdr_value_str(req, "If-None-Match", buffer, sizeof(buffer)) == ESP_OK)
  {
    // We have a value in the eTag header

    if (strncmp(buffer, args->etag, strlen(args->etag)) == 0)
    {
      ESP_LOGD(TAG, "Cached response for %s", req->uri);
      // Matched
      httpd_resp_set_status(req, "304 Not Modified");
      httpd_resp_send(req, NULL, 0);
      return ESP_OK;
    }
  }

  ESP_LOGD(TAG, "Web serve %s", req->uri);
  SetCacheAndETag(req, args->etag);
  return httpd_resp_send(req, (const char *)args->resp, args->resp_len);
}

// Handle static files which are already GZIP compressed
esp_err_t static_content_handler_gzipped(httpd_req_t *req)
{
  WEBKIT_RESPONSE_ARGS *args = (WEBKIT_RESPONSE_ARGS *)(req->user_ctx);

  httpd_resp_set_type(req, args->mimetype);

  // TODO: GET ETAGS WORKING AGAIN!!
  char buffer[50];

  if (httpd_req_get_hdr_value_str(req, "If-None-Match", buffer, sizeof(buffer)) == ESP_OK)
  {
    // We have a value in the eTag header

    if (strncmp(buffer, args->etag, strlen(args->etag)) == 0)
    {
      ESP_LOGD(TAG, "Cached response for %s", req->uri);
      // Matched
      httpd_resp_set_status(req, "304 Not Modified");
      httpd_resp_send(req, NULL, 0);
      return ESP_OK;
    }
  }

  ESP_LOGD(TAG, "Web serve %s", req->uri);
  httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
  SetCacheAndETag(req, args->etag);
  return httpd_resp_send(req, (const char *)args->resp, args->resp_len);
}

/* Our URI handler function to be called during GET /uri request */
esp_err_t get_root_handler(httpd_req_t *req)
{
  httpd_resp_set_type(req, "text/html");
  httpd_resp_set_status(req, "301 Moved Permanently");
  httpd_resp_set_hdr(req, "Location", "/default.htm");
  httpd_resp_send(req, NULL, 0);
  return ESP_OK;
}

/* Our URI handler function to be called during POST /uri request */
esp_err_t post_handler(httpd_req_t *req)
{
  /* Destination buffer for content of HTTP POST request.
   * httpd_req_recv() accepts char* only, but content could
   * as well be any binary data (needs type casting).
   * In case of string data, null termination will be absent, and
   * content length would give length of string */
  char content[100];

  /* Truncate if content length larger than the buffer */
  size_t recv_size = min(req->content_len, sizeof(content));

  int ret = httpd_req_recv(req, content, recv_size);
  if (ret <= 0)
  { /* 0 return value indicates connection closed */
    /* Check if timeout occurred */
    if (ret == HTTPD_SOCK_ERR_TIMEOUT)
    {
      /* In case of timeout one can choose to retry calling
       * httpd_req_recv(), but to keep it simple, here we
       * respond with an HTTP 408 (Request Timeout) error */
      httpd_resp_send_408(req);
    }
    /* In case of error, returning ESP_FAIL will
     * ensure that the underlying socket is closed */
    return ESP_FAIL;
  }

  /* Send a simple response */
  const char resp[] = "URI POST Response";
  httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

/* URI handler structure for GET /uri */
httpd_uri_t uri_root_get = {.uri = "/", .method = HTTP_GET, .handler = get_root_handler, .user_ctx = NULL};

httpd_uri_t uri_defaulthtm_get = {.uri = "/default.htm", .method = HTTP_GET, .handler = default_htm_handler, .user_ctx = NULL};

WEBKIT_RESPONSE_ARGS webkit_style_css_args = {file_style_css_gz, size_file_style_css_gz, etag_file_style_css_gz, text_css};
httpd_uri_t uri_stylecss_get = {.uri = "/style.css", .method = HTTP_GET, .handler = static_content_handler_gzipped, .user_ctx = (void *)&webkit_style_css_args};

WEBKIT_RESPONSE_ARGS webkit_pagecode_js_args = {file_pagecode_js_gz, size_file_pagecode_js_gz, etag_file_pagecode_js_gz, application_javascript};
httpd_uri_t uri_pagecode_js_get = {.uri = "/pagecode.js", .method = HTTP_GET, .handler = static_content_handler_gzipped, .user_ctx = (void *)&webkit_pagecode_js_args};

WEBKIT_RESPONSE_ARGS webkit_jquery_js_args = {file_jquery_js_gz, size_file_jquery_js_gz, etag_file_jquery_js_gz, application_javascript};
httpd_uri_t uri_jquery_js_get = {.uri = "/jquery.js", .method = HTTP_GET, .handler = static_content_handler_gzipped, .user_ctx = (void *)&webkit_jquery_js_args};

WEBKIT_RESPONSE_ARGS webkit_notify_min_js_args = {file_notify_min_js_gz, size_file_notify_min_js_gz, etag_file_notify_min_js_gz, application_javascript};
httpd_uri_t uri_notify_min_js_get = {.uri = "/notify.min.js", .method = HTTP_GET, .handler = static_content_handler_gzipped, .user_ctx = (void *)&webkit_notify_min_js_args};

WEBKIT_RESPONSE_ARGS webkit_echarts_min_js_args = {file_echarts_min_js_gz, size_file_echarts_min_js_gz, etag_file_echarts_min_js_gz, application_javascript};
httpd_uri_t uri_echarts_min_js_get = {.uri = "/echarts.min.js", .method = HTTP_GET, .handler = static_content_handler_gzipped, .user_ctx = (void *)&webkit_echarts_min_js_args};

WEBKIT_RESPONSE_ARGS webkit_lang_ru_js_args = {file_lang_ru_js_gz, size_file_lang_ru_js_gz, etag_file_lang_ru_js_gz, application_javascript};
httpd_uri_t uri_lang_ru_js_get = {.uri = "/lang_ru.js", .method = HTTP_GET, .handler = static_content_handler_gzipped, .user_ctx = (void *)&webkit_lang_ru_js_args};

WEBKIT_RESPONSE_ARGS webkit_lang_hr_js_args = {file_lang_hr_js_gz, size_file_lang_hr_js_gz, etag_file_lang_hr_js_gz, application_javascript};
httpd_uri_t uri_lang_hr_js_get = {.uri = "/lang_hr.js", .method = HTTP_GET, .handler = static_content_handler_gzipped, .user_ctx = (void *)&webkit_lang_hr_js_args};

WEBKIT_RESPONSE_ARGS webkit_lang_nl_js_args = {file_lang_nl_js_gz, size_file_lang_nl_js_gz, etag_file_lang_nl_js_gz, application_javascript};
httpd_uri_t uri_lang_nl_js_get = {.uri = "/lang_nl.js", .method = HTTP_GET, .handler = static_content_handler_gzipped, .user_ctx = (void *)&webkit_lang_nl_js_args};

WEBKIT_RESPONSE_ARGS webkit_lang_pt_js_args = {file_lang_pt_js_gz, size_file_lang_pt_js_gz, etag_file_lang_pt_js_gz, application_javascript};
httpd_uri_t uri_lang_pt_js_get = {.uri = "/lang_pt.js", .method = HTTP_GET, .handler = static_content_handler_gzipped, .user_ctx = (void *)&webkit_lang_pt_js_args};

WEBKIT_RESPONSE_ARGS webkit_lang_de_js_args = {file_lang_de_js_gz, size_file_lang_de_js_gz, etag_file_lang_de_js_gz, application_javascript};
httpd_uri_t uri_lang_de_js_get = {.uri = "/lang_de.js", .method = HTTP_GET, .handler = static_content_handler_gzipped, .user_ctx = (void *)&webkit_lang_de_js_args};

WEBKIT_RESPONSE_ARGS webkit_lang_es_js_args = {file_lang_es_js_gz, size_file_lang_es_js_gz, etag_file_lang_es_js_gz, application_javascript};
httpd_uri_t uri_lang_es_js_get = {.uri = "/lang_es.js", .method = HTTP_GET, .handler = static_content_handler_gzipped, .user_ctx = (void *)&webkit_lang_es_js_args};

WEBKIT_RESPONSE_ARGS webkit_lang_en_js_args = {file_lang_en_js_gz, size_file_lang_en_js_gz, etag_file_lang_en_js_gz, application_javascript};
httpd_uri_t uri_lang_en_js_get = {.uri = "/lang_en.js", .method = HTTP_GET, .handler = static_content_handler_gzipped, .user_ctx = (void *)&webkit_lang_en_js_args};

WEBKIT_RESPONSE_ARGS webkit_favicon_ico_args = {file_favicon_ico_gz, size_file_favicon_ico_gz, etag_file_favicon_ico_gz, image_x_icon};
httpd_uri_t uri_favicon_ico_get = {.uri = "/favicon.ico", .method = HTTP_GET, .handler = static_content_handler_gzipped, .user_ctx = (void *)&webkit_favicon_ico_args};

WEBKIT_RESPONSE_ARGS webkit_logo_png_args = {file_logo_png, size_file_logo_png, etag_file_logo_png, image_png};
httpd_uri_t uri_logo_png_get = {.uri = "/logo.png", .method = HTTP_GET, .handler = static_content_handler, .user_ctx = (void *)&webkit_logo_png_args};

WEBKIT_RESPONSE_ARGS webkit_wait_png_args = {file_wait_png, size_file_wait_png, etag_file_wait_png, image_png};
httpd_uri_t uri_wait_png_get = {.uri = "/wait.png", .method = HTTP_GET, .handler = static_content_handler, .user_ctx = (void *)&webkit_wait_png_args};

WEBKIT_RESPONSE_ARGS webkit_patron_png_args = {file_patron_png, size_file_patron_png, etag_file_patron_png, image_png};
httpd_uri_t uri_patron_png_get = {.uri = "/patron.png", .method = HTTP_GET, .handler = static_content_handler, .user_ctx = (void *)&webkit_patron_png_args};

WEBKIT_RESPONSE_ARGS webkit_warning_png_args = {file_warning_png, size_file_warning_png, etag_file_warning_png, image_png};
httpd_uri_t uri_warning_png_get = {.uri = "/warning.png", .method = HTTP_GET, .handler = static_content_handler, .user_ctx = (void *)&webkit_warning_png_args};

httpd_uri_t uri_monitor2_json_get = {.uri = "/monitor2.json", .method = HTTP_GET, .handler = content_handler_monitor2, .user_ctx = NULL};
httpd_uri_t uri_monitor3_json_get = {.uri = "/monitor3.json", .method = HTTP_GET, .handler = content_handler_monitor3, .user_ctx = NULL};
httpd_uri_t uri_integration_json_get = {.uri = "/integration.json", .method = HTTP_GET, .handler = content_handler_integration, .user_ctx = NULL};
httpd_uri_t uri_settings_json_get = {.uri = "/settings.json", .method = HTTP_GET, .handler = content_handler_settings, .user_ctx = NULL};
httpd_uri_t uri_rules_json_get = {.uri = "/rules.json", .method = HTTP_GET, .handler = content_handler_rules, .user_ctx = NULL};
httpd_uri_t uri_getvictron_json_get = {.uri = "/getvictron.json", .method = HTTP_GET, .handler = content_handler_getvictron, .user_ctx = NULL};
httpd_uri_t uri_rs485settings_json_get = {.uri = "/rs485settings.json", .method = HTTP_GET, .handler = content_handler_rs485settings, .user_ctx = NULL};
httpd_uri_t uri_currentmonitor_json_get = {.uri = "/currentmonitor.json", .method = HTTP_GET, .handler = content_handler_currentmonitor, .user_ctx = NULL};
httpd_uri_t uri_avrstatus_json_get = {.uri = "/avrstatus.json", .method = HTTP_GET, .handler = content_handler_avrstatus, .user_ctx = NULL};
httpd_uri_t uri_modules_json_get = {.uri = "/modules.json", .method = HTTP_GET, .handler = content_handler_modules, .user_ctx = NULL};
httpd_uri_t uri_identifymodule_json_get = {.uri = "/identifyModule.json", .method = HTTP_GET, .handler = content_handler_identifymodule, .user_ctx = NULL};
httpd_uri_t uri_storage_json_get = {.uri = "/storage.json", .method = HTTP_GET, .handler = content_handler_storage, .user_ctx = NULL};
httpd_uri_t uri_avrstorage_json_get = {.uri = "/avrstorage.json", .method = HTTP_GET, .handler = content_handler_avrstorage, .user_ctx = NULL};

httpd_uri_t uri_download_get = {.uri = "/download", .method = HTTP_GET, .handler = content_handler_downloadfile, .user_ctx = NULL};

/* URI handler structure for POST /uri */
httpd_uri_t uri_post = {.uri = "/uri", .method = HTTP_POST, .handler = post_handler, .user_ctx = NULL};

void clearModuleValues(uint8_t module)
{
  cmi[module].valid = false;
  cmi[module].voltagemV = 0;
  cmi[module].voltagemVMin = 6000;
  cmi[module].voltagemVMax = 0;
  cmi[module].badPacketCount = 0;
  cmi[module].inBypass = false;
  cmi[module].bypassOverTemp = false;
  cmi[module].internalTemp = -40;
  cmi[module].externalTemp = -40;
}

/* Function for starting the webserver */
httpd_handle_t start_webserver(void)
{
  /* Generate default configuration */
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();

  config.max_uri_handlers = 48;
  config.max_open_sockets = 3;
  config.max_resp_headers = 16;

  /* Empty handle to esp_http_server */
  httpd_handle_t server = NULL;

  /* Start the httpd server */
  if (httpd_start(&server, &config) == ESP_OK)
  {

    /* Register URI handlers */
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_root_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_defaulthtm_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_stylecss_get));
    // Images
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_favicon_ico_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_warning_png_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_patron_png_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_logo_png_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_wait_png_get));

    // Javascript libs...
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_pagecode_js_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_jquery_js_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_notify_min_js_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_echarts_min_js_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_lang_ru_js_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_lang_hr_js_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_lang_nl_js_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_lang_pt_js_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_lang_de_js_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_lang_es_js_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_lang_en_js_get));

    // Web services/API
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_monitor2_json_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_monitor3_json_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_integration_json_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_settings_json_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_rules_json_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_getvictron_json_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_rs485settings_json_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_currentmonitor_json_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_avrstatus_json_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_modules_json_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_identifymodule_json_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_storage_json_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_avrstorage_json_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_download_get));

    // httpd_register_uri_handler(server, &uri_post);
  }
  /* If server failed to start, handle will be NULL */
  return server;
}

/* Function for stopping the webserver */
void stop_webserver(httpd_handle_t server)
{
  if (server)
  {
    /* Stop the httpd server */
    httpd_stop(server);
  }
}

// Start Web Server (crazy amount of pointer params!)
void StartServer(diybms_eeprom_settings *mysettings,
                 fs::SDFS *sdcard,
                 PacketRequestGenerator *prg,
                 PacketReceiveProcessor *pktreceiveproc,
                 ControllerState *controlState,
                 Rules *rules,
                 void (*sdcardaction_callback)(uint8_t action),
                 HAL_ESP32 *hal)
{
  // Generate the cookie value
  setCookieValue();
  _myserver = start_webserver();
  _hal = hal;
  _prg = prg;
  _controlState = controlState;
  _rules = rules;
  _sdcard = sdcard;
  _mysettings = mysettings;
  _receiveProc = pktreceiveproc;
  _sdcardaction_callback = sdcardaction_callback;
}
