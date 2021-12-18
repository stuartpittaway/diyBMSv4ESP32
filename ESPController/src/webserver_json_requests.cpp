#include "webserver.h"
#include "webserver_helper_funcs.h"
#include "webserver_json_requests.h"

esp_err_t content_handler_avrstorage(httpd_req_t *req)
{
  if (!validateXSS(req))
  {
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "application/json");
  setNoStoreCacheControl(req);

  int bufferused = 0;

  //{"ProgModeEnabled":false,"InProgress":false,"avrprog":{"avrprog":[{"board":"V400","efuse":"F4","hfuse":"D6","lfuse":"62","mcu":"1e9315","name":"fw_V400_4fc1ce41.bin","ver":"4fc1ce41"},{"board":"V410","efuse":"F4","hfuse":"D6","lfuse":"62","mcu":"1e9315","name":"fw_V410_4fc1ce41.bin","ver":"4fc1ce41"},{"board":"V420","efuse":"F4","hfuse":"D6","lfuse":"62","mcu":"1e9315","name":"fw_V420_4fc1ce41.bin","ver":"4fc1ce41"},{"board":"V420_SWAPR19R20","efuse":"F4","hfuse":"D6","lfuse":"62","mcu":"1e9315","name":"fw_V420_SWAPR19R20_4fc1ce41.bin","ver":"4fc1ce41"},{"board":"V421","efuse":"F4","hfuse":"D6","lfuse":"62","mcu":"1e9315","name":"fw_V421_4fc1ce41.bin","ver":"4fc1ce41"},{"board":"V421_LTO","efuse":"F4","hfuse":"D6","lfuse":"62","mcu":"1e9315","name":"fw_V421_LTO_4fc1ce41.bin","ver":"4fc1ce41"},{"board":"V440","efuse":"F4","hfuse":"D6","lfuse":"6C","mcu":"1e9315","name":"fw_V440_4fc1ce41.bin","ver":"4fc1ce41"}]}}
  // See if we can open and process the AVR PROGRAMMER manifest file
  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "{");
  bufferused += printBoolean(&httpbuf[bufferused], BUFSIZE - bufferused, "ProgModeEnabled", _avrsettings.programmingModeEnabled);
  bufferused += printBoolean(&httpbuf[bufferused], BUFSIZE - bufferused, "InProgress", _avrsettings.inProgress);
  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "\"avrprog\":");

  String manifest = String("/avr/manifest.json");
  if (LITTLEFS.exists(manifest))
  {
    // Use Dynamic to avoid head issues
    DynamicJsonDocument doc(3000);
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
  if (!validateXSS(req))
  {
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "application/json");
  setNoStoreCacheControl(req);

  int bufferused = 0;

  // Convert to milliseconds
  uint32_t x = 0;
  if (currentMonitor.validReadings)
  {
    x = (esp_timer_get_time() - currentMonitor.timestamp) / 1000;
  }

  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "{");
  bufferused += printBoolean(&httpbuf[bufferused], BUFSIZE - bufferused, "enabled", mysettings.currentMonitoringEnabled);

  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused,
                         "\"address\":%u,\"timestampage\":%u,\"valid\":%s,\"batterycapacity\":%u,\"tailcurrent\":%.4f,\"fullchargevolt\":%.4f,\"chargeefficiency\":%.4f,",
                         mysettings.currentMonitoringModBusAddress,
                         x,
                         currentMonitor.validReadings ? "true" : "false",
                         currentMonitor.modbus.batterycapacityamphour, currentMonitor.modbus.tailcurrentamps,
                         currentMonitor.modbus.fullychargedvoltage, currentMonitor.chargeefficiency);

  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused,
                         "\"address\":%u,\"timestampage\":%u,",
                         mysettings.currentMonitoringModBusAddress,
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
  if (!validateXSS(req))
  {
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "application/json");
  setNoStoreCacheControl(req);

  int bufferused = 0;

  // Output the first batch of settings/parameters/values
  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE,
                         "{\"baudrate\":%i,\"databits\":%i,\"parity\":%i,\"stopbits\":%i}",
                         mysettings.rs485baudrate, mysettings.rs485databits,
                         mysettings.rs485parity, mysettings.rs485stopbits);

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
  if (!validateXSS(req))
  {
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "application/json");
  setNoStoreCacheControl(req);

  int bufferused = 0;

  sdcard_info info;

  info.available = _sd_card_installed;

  // Lock VSPI bus during operation (not sure if this is acutally needed, as the SD class may have cached these values)
  if (hal.GetVSPIMutex())
  {
    // Convert to KiB
    info.totalkilobytes = SD.totalBytes() / 1024;
    info.usedkilobytes = SD.usedBytes() / 1024;
    hal.ReleaseVSPIMutex();
  }
  else
  {
    info.totalkilobytes = 0;
    info.usedkilobytes = 0;
  }

  info.flash_totalkilobytes = LITTLEFS.totalBytes() / 1024;
  info.flash_usedkilobytes = LITTLEFS.usedBytes() / 1024;

  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "{\"storage\":{");
  bufferused += printBoolean(&httpbuf[bufferused], BUFSIZE - bufferused, "logging", mysettings.loggingEnabled);
  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "\"frequency\":%u,\"sdcard\":{", mysettings.loggingFrequencySeconds);
  bufferused += printBoolean(&httpbuf[bufferused], BUFSIZE - bufferused, "available", info.available);
  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "\"total\":%u,\"used\":%u,\"files\":[", info.totalkilobytes, info.usedkilobytes);

  //  Send it...
  httpd_resp_send_chunk(req, httpbuf, bufferused);
  bufferused = 0;

  // File listing goes here
  if (info.available)
  {
    if (hal.GetVSPIMutex())
    {
      bufferused += fileSystemListDirectory(&httpbuf[bufferused], BUFSIZE - bufferused, SD, "/", 2);
      hal.ReleaseVSPIMutex();
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

esp_err_t SendFileInChunks(httpd_req_t *req, FS &filesystem, const char *filename)
{
  if (filesystem.exists(filename))
  {
    httpd_resp_set_type(req, "application/octet-stream");

    char *httpheader = NULL;

    asprintf(&httpheader, "attachment; filename=\"%s\"", filename);

    if (!httpheader)
    {
      ESP_LOGE(TAG, "No enough memory");
      free(httpheader);
      return ESP_ERR_NO_MEM;
    }
    httpd_resp_set_hdr(req, "Content-Disposition", httpheader);
    // ESP_LOGD(TAG, "Content-Disposition: %s", httpheader);

    ESP_LOGD(TAG, "Stream file %s", filename);
    File f = filesystem.open(filename, FILE_READ);

    size_t bytesRead = 0;
    do
    {
      bytesRead = f.read((uint8_t *)httpbuf, BUFSIZE);
      if (bytesRead > 0)
      {
        ESP_LOGD(TAG, "Stream chunk %i", bytesRead);
        ESP_ERROR_CHECK_WITHOUT_ABORT(httpd_resp_send_chunk(req, httpbuf, bytesRead));
      }
    } while (bytesRead == BUFSIZE);
    f.close();

    free(httpheader);
    return ESP_OK;
  }
  else
  {
    return ESP_ERR_NOT_FOUND;
  }
}

esp_err_t content_handler_downloadfile(httpd_req_t *req)
{
  if (!validateXSS(req))
  {
    return ESP_FAIL;
  }

  bool valid = false;

  char param[128];

  // TODO: No need for String here...
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
      // The components of URL query string (keys and values) are not URLdecoded
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
      // request.send(401); // 401 Unauthorized
      return httpd_resp_send_err(req, httpd_err_code_t::HTTPD_400_BAD_REQUEST, "Bad request");
    }

    if (type.equals("sdcard") && _sd_card_installed)
    {
      // Process file from SD card

      if (hal.GetVSPIMutex())
      {
        // Get the file
        ESP_LOGI(TAG, "Download SDCard file");
        SendFileInChunks(req, SD, file.c_str());
        hal.ReleaseVSPIMutex();
        // Indicate last chunk (zero byte length)
        return httpd_resp_send_chunk(req, httpbuf, 0);
      }
    }
    else if (type.equals("flash"))
    {
      // Process file from flash storage
      ESP_LOGI(TAG, "Download FLASH file");
      SendFileInChunks(req, LITTLEFS, file.c_str());
      // Indicate last chunk (zero byte length)
      return httpd_resp_send_chunk(req, httpbuf, 0);
    }
    else
    {
      return ESP_FAIL;
    }
  }

  return httpd_resp_send_err(req, httpd_err_code_t::HTTPD_400_BAD_REQUEST, "Bad request");
}

esp_err_t content_handler_identifymodule(httpd_req_t *req)
{
  if (!validateXSS(req))
  {
    return ESP_FAIL;
  }

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

        if (c <= mysettings.totalNumberOfBanks * mysettings.totalNumberOfSeriesModules)
        {
          valid = true;
        }
      }
    }
  }

  if (valid)
  {
    prg.sendIdentifyModuleRequest(c);
    return SendSuccess(req);
  }
  else
  {
    // It failed!
    return httpd_resp_send_500(req);
  }
}

esp_err_t content_handler_modules(httpd_req_t *req)
{
  if (!validateXSS(req))
  {
    return ESP_FAIL;
  }

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

        if (c <= mysettings.totalNumberOfBanks * mysettings.totalNumberOfSeriesModules)
        {
          valid = true;
        }
      }
    }
  }

  if (valid)
  {
    httpd_resp_set_type(req, "application/json");
    setNoStoreCacheControl(req);

    if (cmi[c].settingsCached == false)
    {
      prg.sendGetSettingsRequest(c);
    }

    DynamicJsonDocument doc(2048);
    JsonObject root = doc.to<JsonObject>();
    JsonObject settings = root.createNestedObject("settings");

    uint8_t b = c / mysettings.totalNumberOfSeriesModules;
    uint8_t m = c - (b * mysettings.totalNumberOfSeriesModules);
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
  if (!validateXSS(req))
  {
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "application/json");
  setNoStoreCacheControl(req);

  int bufferused = 0;
  StaticJsonDocument<200> doc;
  doc["inprogress"] = _avrsettings.inProgress ? 1 : 0;
  doc["result"] = _avrsettings.progresult;
  doc["duration"] = _avrsettings.duration;
  doc["size"] = _avrsettings.programsize;
  doc["mcu"] = _avrsettings.mcu;

  bufferused += serializeJson(doc, httpbuf, BUFSIZE);

  return httpd_resp_send(req, httpbuf, bufferused);
}

esp_err_t content_handler_victron(httpd_req_t *req)
{
  if (!validateXSS(req))
  {
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "application/json");
  setNoStoreCacheControl(req);

  int bufferused = 0;

  DynamicJsonDocument doc(2048);
  JsonObject root = doc.to<JsonObject>();

  JsonObject settings = root.createNestedObject("victron");

  settings["enabled"] = mysettings.VictronEnabled;

  JsonArray cvl = settings.createNestedArray("cvl");
  JsonArray ccl = settings.createNestedArray("ccl");
  JsonArray dcl = settings.createNestedArray("dcl");
  for (uint8_t i = 0; i < 3; i++)
  {
    cvl.add(mysettings.cvl[i]);
    ccl.add(mysettings.ccl[i]);
    dcl.add(mysettings.dcl[i]);
  }

  bufferused += serializeJson(doc, httpbuf, BUFSIZE);

  return httpd_resp_send(req, httpbuf, bufferused);
}

esp_err_t content_handler_rules(httpd_req_t *req)
{

  if (!validateXSS(req))
  {
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "application/json");
  setNoStoreCacheControl(req);

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

  root["ControlState"] = _controller_state;

  JsonArray defaultArray = root.createNestedArray("relaydefault");
  for (uint8_t relay = 0; relay < RELAY_TOTAL; relay++)
  {
    switch (mysettings.rulerelaydefault[relay])
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
    switch (mysettings.relaytype[relay])
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
    rule["value"] = mysettings.rulevalue[r];
    rule["hysteresis"] = mysettings.rulehysteresis[r];
    rule["triggered"] = rules.rule_outcome[r];
    JsonArray data = rule.createNestedArray("relays");

    for (uint8_t relay = 0; relay < RELAY_TOTAL; relay++)
    {
      switch (mysettings.rulerelaystate[r][relay])
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
  if (!validateXSS(req))
  {
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "application/json");
  setNoStoreCacheControl(req);

  int bufferused = 0;

  DynamicJsonDocument doc(2048);
  JsonObject root = doc.to<JsonObject>();

  JsonObject settings = root.createNestedObject("settings");

  settings["totalnumberofbanks"] = mysettings.totalNumberOfBanks;
  settings["totalseriesmodules"] = mysettings.totalNumberOfSeriesModules;
  settings["baudrate"] = mysettings.baudRate;

  settings["bypassthreshold"] = mysettings.BypassThresholdmV;
  settings["bypassovertemp"] = mysettings.BypassOverTempShutdown;

  settings["NTPServerName"] = mysettings.ntpServer;
  settings["TimeZone"] = mysettings.timeZone;
  settings["MinutesTimeZone"] = mysettings.minutesTimeZone;
  settings["DST"] = mysettings.daylight;

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
  if (!validateXSS(req))
  {
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "application/json");
  setNoStoreCacheControl(req);

  int bufferused = 0;
  /*
    const char *nullstring = "null";

    // Output the first batch of settings/parameters/values
    bufferused += snprintf(&httpbuf[bufferused], BUFSIZE,
                           "{\"mqtt\":{\"enabled\":%s,\"topic\":\"%s\",\"port\":%u,\"server\":\"%s\",\"username\":\"%s\"}",
                           mysettings.mqtt_enabled ? "true" : "false", mysettings.mqtt_topic, mysettings.mqtt_port, mysettings.mqtt_server, mysettings.mqtt_username);

    bufferused += snprintf(&httpbuf[bufferused], BUFSIZE,
                           ",\"influxdb\":{\"enabled\":%s,\"url\":\"%s\",\"bucket\":\"%s\",\"apitoken\":\"%s\",\"orgid\":\"%s\"}}",
                           mysettings.influxdb_enabled ? "true" : "false", mysettings.influxdb_serverurl, mysettings.influxdb_databasebucket, mysettings.influxdb_apitoken, mysettings.influxdb_orgid);

    // ESP_LOGD(TAG, "bufferused=%i", bufferused);  ESP_LOGD(TAG, "monitor2: %s", buf);
    //  Send it...
    httpd_resp_send_chunk(req, httpbuf, bufferused);

    // Indicate last chunk (zero byte length)
    return httpd_resp_send_chunk(req, buf, 0);
  */

  DynamicJsonDocument doc(1024);
  JsonObject root = doc.to<JsonObject>();

  JsonObject mqtt = root.createNestedObject("mqtt");
  mqtt["enabled"] = mysettings.mqtt_enabled;
  mqtt["topic"] = mysettings.mqtt_topic;
  mqtt["port"] = mysettings.mqtt_port;
  mqtt["server"] = mysettings.mqtt_server;
  mqtt["username"] = mysettings.mqtt_username;
  // We don't output the password in the json file as this could breach security
  // mqtt["password"] =mysettings.mqtt_password;

  JsonObject influxdb = root.createNestedObject("influxdb");
  influxdb["enabled"] = mysettings.influxdb_enabled;
  influxdb["url"] = mysettings.influxdb_serverurl;
  influxdb["bucket"] = mysettings.influxdb_databasebucket;
  influxdb["apitoken"] = mysettings.influxdb_apitoken;
  influxdb["orgid"] = mysettings.influxdb_orgid;

  bufferused += serializeJson(doc, httpbuf, BUFSIZE);

  return httpd_resp_send(req, httpbuf, bufferused);
}

esp_err_t content_handler_monitor3(httpd_req_t *req)
{
  if (!validateXSS(req))
  {
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "application/json");
  setNoStoreCacheControl(req);

  uint8_t totalModules = mysettings.totalNumberOfBanks * mysettings.totalNumberOfSeriesModules;
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
  // Don't valid the cookie here, allow it to return basic information
  // as read only

  httpd_resp_set_type(req, "application/json");
  setNoStoreCacheControl(req);

  uint8_t totalModules = mysettings.totalNumberOfBanks * mysettings.totalNumberOfSeriesModules;

  int bufferused = 0;
  const char *nullstring = "null";

  // Output the first batch of settings/parameters/values
  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE,
                         "{\"banks\":%u,\"seriesmodules\":%u,\"sent\":%u,\"received\":%u,\"modulesfnd\":%u,\"badcrc\":%u,\"ignored\":%u,\"roundtrip\":%u,\"oos\":%u,\"activerules\":%u,\"uptime\":%u,\"can_fail\":%u,\"can_sent\":%u,\"can_rec\":%u,\"sec\":\"%s\",\"qlen\":%u,",
                         mysettings.totalNumberOfBanks,
                         mysettings.totalNumberOfSeriesModules,
                         prg.packetsGenerated,
                         receiveProc.packetsReceived,
                         receiveProc.totalModulesFound,
                         receiveProc.totalCRCErrors,
                         receiveProc.totalNotProcessedErrors,
                         receiveProc.packetTimerMillisecond,
                         receiveProc.totalOutofSequenceErrors,
                         rules.active_rule_count, (uint32_t)(esp_timer_get_time() / (uint64_t)1e+6),
                         canbus_messages_failed_sent, canbus_messages_sent,
                         canbus_messages_received, &CookieValue[sizeof(CookieValue) - 3],
                         prg.queueLength());

  // current
  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "\"current\":[");

  if (mysettings.currentMonitoringEnabled && currentMonitor.validReadings)
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

  for (size_t i = 0; i < sizeof(rules.ErrorCodes); i++)
  {
    if (rules.ErrorCodes[i] != InternalErrorCode::NoError)
    {
      // Comma if not zero
      if (count)
      {
        bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, ",");
      }
      bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "%u", rules.ErrorCodes[i]);
      count++;
    }
  }
  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "],\"warnings\":[");

  count = 0;
  for (size_t i = 0; i < sizeof(rules.WarningCodes); i++)
  {
    if (rules.WarningCodes[i] != InternalWarningCode::NoWarning)
    {
      // Comma if not zero
      if (count)
      {
        bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, ",");
      }

      bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "%u", rules.WarningCodes[i]);
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

  for (uint8_t i = 0; i < mysettings.totalNumberOfBanks; i++)
  {
    // Comma if not zero
    if (i)
      bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, ",");

    bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "%u", rules.packvoltage[i]);
  }
  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "],");

  // ESP_LOGD(TAG, "bufferused=%i", bufferused);  ESP_LOGD(TAG, "monitor2: %s", buf);
  //  Send it...
  httpd_resp_send_chunk(req, httpbuf, bufferused);

  // voltrange
  bufferused = 0;
  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "\"voltrange\":[");

  for (uint8_t i = 0; i < mysettings.totalNumberOfBanks; i++)
  {
    // Comma if not zero
    if (i)
    {
      bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, ",");
    }

    bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "%u", rules.VoltageRangeInBank(i));
  }
  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "]}");

  // ESP_LOGD(TAG, "bufferused=%i", bufferused);  ESP_LOGD(TAG, "monitor2: %s", buf);
  //  Send it...
  httpd_resp_send_chunk(req, httpbuf, bufferused);

  // Indicate last chunk (zero byte length)
  return httpd_resp_send_chunk(req, httpbuf, 0);
}