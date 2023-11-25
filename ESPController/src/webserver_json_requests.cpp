#define USE_ESP_IDF_LOG 1
static constexpr const char *const TAG = "diybms-webreq";

#include "webserver.h"
#include "webserver_helper_funcs.h"
#include "webserver_json_requests.h"
#include <esp_netif.h>
#include <esp_wifi.h>
extern "C"
{
#include "esp_core_dump.h"
}

esp_err_t content_handler_avrstorage(httpd_req_t *req)
{
  int bufferused = 0;

  // See if we can open and process the AVR PROGRAMMER manifest file
  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "{");
  bufferused += printBoolean(&httpbuf[bufferused], BUFSIZE - bufferused, "ProgModeEnabled", _avrsettings.programmingModeEnabled);
  bufferused += printBoolean(&httpbuf[bufferused], BUFSIZE - bufferused, "InProgress", _avrsettings.inProgress);
  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "\"avrprog\":");

  auto manifest = String("/avr/manifest.json");
  if (LittleFS.exists(manifest))
  {
    // Use Dynamic to avoid head issues
    DynamicJsonDocument doc(3000);
    File file = LittleFS.open(manifest);
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
  int bufferused = 0;

  // Convert to milliseconds
  uint32_t timestampage = 0;
  if (currentMonitor.validReadings)
  {
    timestampage = (uint32_t)((esp_timer_get_time() - currentMonitor.timestamp) / 1000);
  }

  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "{");
  bufferused += printBoolean(&httpbuf[bufferused], BUFSIZE - bufferused, "enabled", mysettings.currentMonitoringEnabled);

  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused,
                         R"("address":%u,"devicetype":%u,"timestampage":%u,"valid":%s,"batterycapacity":%u,"tailcurrent":%.4f,"fullchargevolt":%.4f,"chargeefficiency":%.4f,)",
                         mysettings.currentMonitoringModBusAddress,
                         mysettings.currentMonitoringDevice,
                         timestampage,
                         currentMonitor.validReadings ? "true" : "false",
                         currentMonitor.modbus.batterycapacityamphour, currentMonitor.modbus.tailcurrentamps,
                         currentMonitor.modbus.fullychargedvoltage, currentMonitor.chargeefficiency);

  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused,
                         R"("voltage":%.4f,"current":%.4f,"mahout":%u,"mahin":%u,"temperature":%i,"watchdog":%u,"power":%.4f,"resistance":%.4f,"calibration":%u,"templimit":%i,"undervlimit":%.4f,"overvlimit":%.4f,"overclimit":%.4f,"underclimit":%.4f,"overplimit":%.4f,"tempcoeff":%u,"model":%u,"firmwarev":%u,"firmwaredate":%u,)",
                         currentMonitor.modbus.voltage, currentMonitor.modbus.current, currentMonitor.modbus.milliamphour_out, currentMonitor.modbus.milliamphour_in,
                         currentMonitor.modbus.temperature, currentMonitor.modbus.watchdogcounter, currentMonitor.modbus.power,
                         currentMonitor.modbus.shuntresistance,
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

  if (mysettings.currentMonitoringDevice == CurrentMonitorDevice::DIYBMS_CURRENT_MON_MODBUS)
  {
    bufferused += printBoolean(&httpbuf[bufferused], BUFSIZE - bufferused, "RelayState", currentMonitor.RelayState);
  }
  else
  {
    // Relay doesn't exist
    bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, R"("RelayState":null,)");
  }

  // Onboard INA229 current monitor chip
  bufferused += printBoolean(&httpbuf[bufferused], BUFSIZE - bufferused, "OnboardCM", currentmon_internal.Available());

  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused,
                         R"("shuntmv":%u,"shuntmaxcur":%u})",
                         currentMonitor.modbus.shuntmillivolt, currentMonitor.modbus.shuntmaxcurrent);

  return httpd_resp_send(req, httpbuf, bufferused);
}

esp_err_t content_handler_rs485settings(httpd_req_t *req)
{
  int bufferused = 0;

  // Output the first batch of settings/parameters/values
  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE,
                         R"({"baudrate":%i,"databits":%i,"parity":%i,"stopbits":%i})",
                         mysettings.rs485baudrate, mysettings.rs485databits,
                         mysettings.rs485parity, mysettings.rs485stopbits);

  return httpd_resp_send(req, httpbuf, bufferused);
}

int fileSystemListDirectory(httpd_req_t *r, char *buffer, size_t bufferLen, fs::FS &fs, const char *dirname, uint8_t)
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

  uint32_t filecounter = 0;
  File file = root.openNextFile();
  while (file)
  {
    if (file.isDirectory())
    {
      // Hide folders
    }
    else
    {
      bufferused += snprintf(&buffer[bufferused], bufferLen - bufferused, "\"%s\",", file.name());
      filecounter++;
    }

    file = root.openNextFile();

    if (filecounter > 50)
    {
      // Flush http buffer every X files to prevent overflows
      httpd_resp_send_chunk(r, buffer, bufferused);
      bufferused = 0;
    }
  }

  // Trailing null to cope with trailing ','
  bufferused += snprintf(&buffer[bufferused], bufferLen - bufferused, "null");

  return bufferused;
}

esp_err_t content_handler_diagnostic(httpd_req_t *req)
{
  return diagnosticJSON(req, httpbuf, BUFSIZE);
}

esp_err_t content_handler_history(httpd_req_t *req)
{
  return history.GenerateJSON(req, httpbuf, BUFSIZE);
}

esp_err_t content_handler_storage(httpd_req_t *req)
{
  int bufferused = 0;

  bool available;
  uint32_t totalkilobytes;
  uint32_t usedkilobytes;
  uint32_t flash_totalkilobytes;
  uint32_t flash_usedkilobytes;

  available = _sd_card_installed;

  if (_avrsettings.programmingModeEnabled)
  {
    // If programming mode is enabled, don't read SD card as the VSPI bus may be in use.
    // We can still return the remaining data/LittleFS stuff..
    available = false;
  }

  // Lock VSPI bus during operation (not sure if this is acutally needed, as the SD class may have cached these values)
  if (available && hal.GetVSPIMutex())
  {
    // Convert to KiB
    totalkilobytes = (uint32_t)(SD.totalBytes() / 1024);
    usedkilobytes = (uint32_t)(SD.usedBytes() / 1024);
    hal.ReleaseVSPIMutex();
  }
  else
  {
    totalkilobytes = 0;
    usedkilobytes = 0;
  }

  flash_totalkilobytes = (uint32_t)(LittleFS.totalBytes() / 1024);
  flash_usedkilobytes = (uint32_t)(LittleFS.usedBytes() / 1024);

  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, R"({"storage":{)");
  bufferused += printBoolean(&httpbuf[bufferused], BUFSIZE - bufferused, "logging", mysettings.loggingEnabled);
  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, R"("frequency":%u,"sdcard":{)", mysettings.loggingFrequencySeconds);
  bufferused += printBoolean(&httpbuf[bufferused], BUFSIZE - bufferused, "available", available);
  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, R"("total":%u,"used":%u,"files":[)", totalkilobytes, usedkilobytes);

  //  Send it...
  httpd_resp_send_chunk(req, httpbuf, bufferused);
  bufferused = 0;

  // File listing goes here
  if (available)
  {
    if (hal.GetVSPIMutex())
    {
      bufferused += fileSystemListDirectory(req, &httpbuf[bufferused], BUFSIZE - bufferused, SD, "/", 2);
      hal.ReleaseVSPIMutex();
    }
  }

  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, R"(]},"flash":{)");

  //  Send it...
  httpd_resp_send_chunk(req, httpbuf, bufferused);

  bufferused = 0;
  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, R"("total":%u,"used":%u,"files":[)", flash_totalkilobytes, flash_usedkilobytes);
  bufferused += fileSystemListDirectory(req, &httpbuf[bufferused], BUFSIZE - bufferused, LittleFS, "/", 0);
  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "]}}}");

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

    char *httpheader = nullptr;

    asprintf(&httpheader, "attachment; filename=\"%s\"", filename);

    if (!httpheader)
    {
      ESP_LOGE(TAG, "No enough memory");
      free(httpheader);
      return ESP_ERR_NO_MEM;
    }
    httpd_resp_set_hdr(req, "Content-Disposition", httpheader);

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
    ESP_LOGE(TAG, "File not found");
    return ESP_ERR_NOT_FOUND;
  }
}

esp_err_t content_handler_coredumpdownloadfile(httpd_req_t *req)
{
  if (!validateXSS(req))
  {
    return ESP_FAIL;
  }

  ESP_LOGI(TAG, "Download coredump");

  size_t size = 0;
  size_t address = 0;
  if (esp_core_dump_image_get(&address, &size) == ESP_OK)
  {
    const esp_partition_t *pt = NULL;
    pt = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_COREDUMP, "coredump");

    if (pt != NULL)
    {

      httpd_resp_set_type(req, "application/octet-stream");
      httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=\"diybms_coredump.esp32\"");

      // Stream it out...

      int16_t toRead;
      for (int16_t i = 0; i < (size / 256) + 1; i++)
      {
        toRead = (size - i * 256) > 256 ? 256 : (size - i * 256);

        esp_err_t er = esp_partition_read(pt, i * 256, httpbuf, toRead);
        if (er != ESP_OK)
        {
          ESP_LOGE(TAG, "Coredump download Fail [%x]\n", er);
          break;
        }

        ESP_ERROR_CHECK_WITHOUT_ABORT(httpd_resp_send_chunk(req, httpbuf, 256));
      }

      // After download, erase the core dump from flash
      ESP_ERROR_CHECK_WITHOUT_ABORT(esp_core_dump_image_erase());

      // Indicate last chunk (zero byte length)
      return httpd_resp_send_chunk(req, httpbuf, 0);
    }
  }

  return httpd_resp_send_err(req, httpd_err_code_t::HTTPD_400_BAD_REQUEST, "Bad request");
}

esp_err_t content_handler_downloadfile(httpd_req_t *req)
{
  if (!validateXSS(req))
  {
    return ESP_FAIL;
  }

  bool valid = false;

  char type[16];
  char file[64];

  char buf[128];
  size_t buf_len = httpd_req_get_url_query_len(req);
  if (buf_len > 1)
  {
    // Only allow up to our pre-defined buffer length (100 bytes)
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK)
    {
      // ESP_LOGD(TAG, "Found URL query => %s", buf);

      /* Get value of expected key from query string */
      // The components of URL query string (keys and values) are not URLdecoded
      if (httpd_query_key_value(buf, "type", type, sizeof(type)) == ESP_OK)
      {
        // ESP_LOGD(TAG, "type=%s", type);

        if (httpd_query_key_value(buf, "file", file, sizeof(file)) == ESP_OK)
        {
          // ESP_LOGD(TAG, "file=%s", file);
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
    if ((strncmp(file, "/", sizeof(file)) == 0) || (strncmp(file, "/diybms/", sizeof(file)) == 0))
    {
      // Avoid directory paths
      return httpd_resp_send_err(req, httpd_err_code_t::HTTPD_400_BAD_REQUEST, "Bad request");
    }

    // Prepend a / character to front of filename
    memset(buf, 0, sizeof(buf));
    buf[0] = '/';
    strcat(buf, file);
    strncpy(file, buf, sizeof(file));

    ESP_LOGI(TAG, "Download %s from %s", file, type);

    if ((strncmp(type, "sdcard", sizeof(type)) == 0) && _sd_card_installed)
    {
      // Process file from SD card
      if (hal.GetVSPIMutex())
      {
        // Get the file
        // ESP_LOGI(TAG, "Download SDCard file");
        esp_err_t result = SendFileInChunks(req, SD, file);
        ESP_LOGD(TAG, "Result %i", result);
        hal.ReleaseVSPIMutex();
        // Indicate last chunk (zero byte length)
        return httpd_resp_send_chunk(req, httpbuf, 0);
      }
    }
    else if ((strncmp(type, "flash", sizeof(type)) == 0))
    {
      // Process file from flash storage
      // ESP_LOGI(TAG, "Download FLASH file");
      esp_err_t result = SendFileInChunks(req, LittleFS, file);
      ESP_LOGD(TAG, "Result %i", result);
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
  uint8_t c;
  bool valid = false;

  char buf[100];
  size_t buf_len = httpd_req_get_url_query_len(req);
  if (buf_len > 1)
  {
    // Only allow up to our pre-defined buffer length (100 bytes)
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK)
    {
      char param[8];
      /* Get value of expected key from query string */
      if (httpd_query_key_value(buf, "c", param, sizeof(param)) == ESP_OK)
      {
        c = (uint8_t)atoi(param);

        if (c <= mysettings.totalNumberOfBanks * mysettings.totalNumberOfSeriesModules)
        {
          valid = true;
        }
      }
    }
  }

  if (valid)
  {
    ESP_LOGI(TAG, "Identify module %u", c);
    if (prg.sendIdentifyModuleRequest(c))
    {
      return SendSuccess(req);
    }
  }

  // It failed!
  return httpd_resp_send_500(req);
}

esp_err_t content_handler_modules(httpd_req_t *req)
{
  uint8_t c;
  bool valid = false;

  char buf[64];
  size_t buf_len = httpd_req_get_url_query_len(req);
  if (buf_len > 1)
  {
    // Only allow up to our pre-defined buffer length (100 bytes)
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK)
    {
      // ESP_LOGI(TAG, "Found URL query => %s", buf);
      char param[8];
      /* Get value of expected key from query string */
      if (httpd_query_key_value(buf, "c", param, sizeof(param)) == ESP_OK)
      {
        c = atoi(param);

        // ESP_LOGI(TAG, "Found URL query parameter => query1=%s (%u)", param, c);

        if (c <= mysettings.totalNumberOfBanks * mysettings.totalNumberOfSeriesModules)
        {
          valid = true;
        }
      }
    }
  }

  if (!valid)
  {
    return httpd_resp_send_500(req);
  }

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
    settings["Prohibited"] = cmi[c].ChangesProhibited;
    settings["FanSwitchOnT"] = cmi[c].FanSwitchOnTemperature;
    settings["RelayMinV"] = cmi[c].RelayMinmV;
    settings["RelayRange"] = cmi[c].RelayRangemV;
    settings["Parasite"] = cmi[c].ParasiteVoltagemV;
    settings["RunAwayMinmV"] = cmi[c].RunAwayCellMinimumVoltagemV;
    settings["RunAwayDiffmV"] = cmi[c].RunAwayCellDifferentialmV;
  }

  int bufferused = 0;
  bufferused += serializeJson(doc, httpbuf, BUFSIZE);
  return httpd_resp_send(req, httpbuf, bufferused);
}

esp_err_t content_handler_avrstatus(httpd_req_t *req)
{
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

esp_err_t content_handler_tileconfig(httpd_req_t *req)
{
  StaticJsonDocument<200> doc;
  JsonObject root = doc.to<JsonObject>();
  JsonObject settings = root.createNestedObject("tileconfig");
  JsonArray v = settings.createNestedArray("values");

  for (auto n : mysettings.tileconfig)
  {
    v.add(n);
  }

  int bufferused = serializeJson(doc, httpbuf, BUFSIZE);
  return httpd_resp_send(req, httpbuf, bufferused);
}

esp_err_t content_handler_chargeconfig(httpd_req_t *req)
{
  int bufferused = 0;

  DynamicJsonDocument doc(2048);
  JsonObject root = doc.to<JsonObject>();
  JsonObject settings = root.createNestedObject("chargeconfig");

  settings["canbusprotocol"] = mysettings.canbusprotocol;
  settings["canbusinverter"] = mysettings.canbusinverter;
  settings["canbusbaud"] = mysettings.canbusbaud;
  settings["equip_addr"] = mysettings.canbus_equipment_addr;
  settings["nominalbatcap"] = mysettings.nominalbatcap;
  settings["chargevolt"] = mysettings.chargevolt;
  settings["chargecurrent"] = mysettings.chargecurrent;
  settings["dischargecurrent"] = mysettings.dischargecurrent;
  settings["dischargevolt"] = mysettings.dischargevolt;
  settings["chargetemplow"] = mysettings.chargetemplow;
  settings["chargetemphigh"] = mysettings.chargetemphigh;
  settings["dischargetemplow"] = mysettings.dischargetemplow;
  settings["dischargetemphigh"] = mysettings.dischargetemphigh;
  settings["stopchargebalance"] = mysettings.stopchargebalance;
  settings["socoverride"] = mysettings.socoverride;
  settings["socforcelow"] = mysettings.socforcelow;
  settings["dynamiccharge"] = mysettings.dynamiccharge;
  settings["preventdischarge"] = mysettings.preventdischarge;
  settings["preventcharging"] = mysettings.preventcharging;
  settings["cellminmv"] = mysettings.cellminmv;
  settings["cellmaxmv"] = mysettings.cellmaxmv;
  settings["kneemv"] = mysettings.kneemv;
  settings["sensitivity"] = mysettings.sensitivity;
  settings["cellmaxspikemv"] = mysettings.cellmaxspikemv;

  settings["cur_val1"] = mysettings.current_value1;
  settings["cur_val2"] = mysettings.current_value2;

  settings["absorptimer"] = mysettings.absorptiontimer;
  settings["floattimer"] = mysettings.floatvoltagetimer;
  settings["socresume"] = mysettings.stateofchargeresumevalue;
  settings["floatvolt"] = mysettings.floatvoltage;

  bufferused += serializeJson(doc, httpbuf, BUFSIZE);

  return httpd_resp_send(req, httpbuf, bufferused);
}
esp_err_t content_handler_rules(httpd_req_t *req)
{
  int bufferused = 0;

  DynamicJsonDocument doc(3000);
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
  for (auto v : mysettings.rulerelaydefault)
  {
    switch (v)
    {
    case RELAY_OFF:
      defaultArray.add(false);
      break;
    case RELAY_ON:
      defaultArray.add(true);
      break;
    default:
      // Null value
      defaultArray.add(nullptr);
      break;
    }
  }

  JsonArray typeArray = root.createNestedArray("relaytype");
  for (auto v : mysettings.relaytype)
  {
    switch (v)
    {
    case RELAY_STANDARD:
      typeArray.add("Std");
      break;
    case RELAY_PULSE:
      typeArray.add("Pulse");
      break;
    default:
      typeArray.add(nullptr);
      break;
    }
  }

  JsonArray bankArray = root.createNestedArray("rules");

  for (uint8_t r = 0; r < RELAY_RULES; r++)
  {
    JsonObject rule = bankArray.createNestedObject();
    rule["value"] = mysettings.rulevalue[r];
    rule["hysteresis"] = mysettings.rulehysteresis[r];
    rule["triggered"] = rules.ruleOutcome((Rule)r);
    JsonArray data = rule.createNestedArray("relays");

    for (auto v : mysettings.rulerelaystate[r])
    {
      switch (v)
      {
      case RELAY_OFF:
        data.add(false);
        break;
      case RELAY_ON:
        data.add(true);
        break;
      default:
        // Null
        data.add(nullptr);
        break;
      }
    }
  }

  bufferused += serializeJson(doc, httpbuf, BUFSIZE);

  return httpd_resp_send(req, httpbuf, bufferused);
}

esp_err_t content_handler_settings(httpd_req_t *req)
{
  int bufferused = 0;

  DynamicJsonDocument doc(2048);
  JsonObject root = doc.to<JsonObject>();

  JsonObject settings = root.createNestedObject("settings");

  settings["totalnumberofbanks"] = mysettings.totalNumberOfBanks;
  settings["totalseriesmodules"] = mysettings.totalNumberOfSeriesModules;
  settings["baudrate"] = mysettings.baudRate;
  settings["interpacketgap"] = mysettings.interpacketgap;

  settings["bypassthreshold"] = mysettings.BypassThresholdmV;
  settings["bypassovertemp"] = mysettings.BypassOverTempShutdown;

  settings["NTPServerName"] = mysettings.ntpServer;
  settings["TimeZone"] = mysettings.timeZone;
  settings["MinutesTimeZone"] = mysettings.minutesTimeZone;
  settings["DST"] = mysettings.daylight;

  settings["HostName"] = hostname.c_str();

  time_t now;
  if (time(&now))
  {
    settings["now"] = now;
  }

  char strftime_buf[64];
  formatCurrentDateTime(strftime_buf, sizeof(strftime_buf));
  settings["datetime"] = std::string(strftime_buf);

  // Return running network settings
  if (tcpip_adapter_is_netif_up(TCPIP_ADAPTER_IF_STA))
  {
    tcpip_adapter_ip_info_t ipInfo;
    // Get actual/running IP networking for STA adapter...
    if (tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &ipInfo) == ESP_OK)
    {
      settings["run_ip"] = ip4_to_string(ipInfo.ip.addr);
      settings["run_netmask"] = ip4_to_string(ipInfo.netmask.addr);
      settings["run_gw"] = ip4_to_string(ipInfo.gw.addr);
    }

    tcpip_adapter_dns_info_t dnsInfo = {0};
    // Primary DNS
    if (tcpip_adapter_get_dns_info(TCPIP_ADAPTER_IF_STA, tcpip_adapter_dns_type_t::ESP_NETIF_DNS_MAIN, &dnsInfo) == ESP_OK)
    {
      if (dnsInfo.ip.type == IPADDR_TYPE_V4)
      {
        settings["run_dns1"] = ip4_to_string(dnsInfo.ip.u_addr.ip4.addr);
      }
    }
    // Secondary DNS
    if (tcpip_adapter_get_dns_info(TCPIP_ADAPTER_IF_STA, tcpip_adapter_dns_type_t::ESP_NETIF_DNS_BACKUP, &dnsInfo) == ESP_OK)
    {
      if (dnsInfo.ip.type == IPADDR_TYPE_V4)
      {
        settings["run_dns2"] = ip4_to_string(dnsInfo.ip.u_addr.ip4.addr);
      }
    }

    settings["man_ip"] = ip4_to_string(_wificonfig.wifi_ip);
    settings["man_netmask"] = ip4_to_string(_wificonfig.wifi_netmask);
    settings["man_gw"] = ip4_to_string(_wificonfig.wifi_gateway);
    settings["man_dns1"] = ip4_to_string(_wificonfig.wifi_dns1);
    settings["man_dns2"] = ip4_to_string(_wificonfig.wifi_dns2);
  }

  JsonObject wifi = root.createNestedObject("wifi");

  if (wifi_isconnected)
  {
    wifi_ap_record_t ap;
    esp_wifi_sta_get_ap_info(&ap);
    wifi["rssi"] = ap.rssi;
    wifi["ssid"] = ap.ssid;

    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
             ap.bssid[0], ap.bssid[1], ap.bssid[2], ap.bssid[3], ap.bssid[4], ap.bssid[5]);
    wifi["bssid"] = macStr;
  }
  else
  {
    wifi["rssi"] = 0;
    wifi["ssid"] = "";
    wifi["bssid"] = "";
  }

  wifi["rssi_low"] = wifi_count_rssi_low;
  wifi["sta_start"] = wifi_count_sta_start;
  wifi["sta_connected"] = wifi_count_sta_connected;
  wifi["sta_disconnected"] = wifi_count_sta_disconnected;
  wifi["sta_lost_ip"] = wifi_count_sta_lost_ip;
  wifi["sta_got_ip"] = wifi_count_sta_got_ip;

  bufferused += serializeJson(doc, httpbuf, BUFSIZE);

  return httpd_resp_send(req, httpbuf, bufferused);
}

esp_err_t content_handler_integration(httpd_req_t *req)
{
  int bufferused = 0;

  DynamicJsonDocument doc(1024);
  JsonObject root = doc.to<JsonObject>();

  JsonObject ha = root.createNestedObject("ha");
  ha["api"] = mysettings.homeassist_apikey;

  JsonObject mqtt = root.createNestedObject("mqtt");
  mqtt["enabled"] = mysettings.mqtt_enabled;
  mqtt["basiccellreporting"] = mysettings.mqtt_basic_cell_reporting;
  mqtt["topic"] = mysettings.mqtt_topic;
  mqtt["uri"] = mysettings.mqtt_uri;
  mqtt["username"] = mysettings.mqtt_username;

  mqtt["connected"] = mqttClient_connected;
  mqtt["err_conn_count"] = mqtt_error_connection_count;
  mqtt["err_trans_count"] = mqtt_error_transport_count;
  mqtt["conn_count"] = mqtt_connection_count;
  mqtt["disc_count"] = mqtt_disconnection_count;

  // We don't output the password in the json file as this could breach security
  // mqtt["password"] =mysettings.mqtt_password;

  JsonObject influxdb = root.createNestedObject("influxdb");
  influxdb["enabled"] = mysettings.influxdb_enabled;
  influxdb["url"] = mysettings.influxdb_serverurl;
  influxdb["bucket"] = mysettings.influxdb_databasebucket;
  influxdb["apitoken"] = mysettings.influxdb_apitoken;
  influxdb["orgid"] = mysettings.influxdb_orgid;
  influxdb["frequency"] = mysettings.influxdb_loggingFreqSeconds;

  bufferused += serializeJson(doc, httpbuf, BUFSIZE);

  return httpd_resp_send(req, httpbuf, bufferused);
}

esp_err_t content_handler_monitor3(httpd_req_t *req)
{
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
  uint8_t totalModules = mysettings.totalNumberOfBanks * mysettings.totalNumberOfSeriesModules;

  int bufferused = 0;
  const char *nullstring = "null";

  // Output the first batch of settings/parameters/values
  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused,
                         R"({"banks":%u,"seriesmodules":%u,"sent":%u,"received":%u,"modulesfnd":%u,"badcrc":%u,"ignored":%u,"roundtrip":%u,"oos":%u,"activerules":%u,"uptime":%u,"can_fail":%u,"can_sent":%u,"can_rec":%u,"can_r_err":%u,"qlen":%u,"cmode":%u,"ctime":%i,)",
                         mysettings.totalNumberOfBanks,
                         mysettings.totalNumberOfSeriesModules,
                         prg.packetsGenerated,
                         receiveProc.packetsReceived,
                         receiveProc.totalModulesFound,
                         receiveProc.totalCRCErrors,
                         receiveProc.totalNotProcessedErrors,
                         receiveProc.packetTimerMillisecond,
                         receiveProc.totalOutofSequenceErrors,
                         rules.active_rule_count,
                         (uint32_t)(esp_timer_get_time() / (uint64_t)1e+6),
                         canbus_messages_failed_sent,
                         canbus_messages_sent,
                         canbus_messages_received,
                         canbus_messages_received_error,
                         prg.queueLength(),
                         (unsigned int)rules.getChargingMode(),
                         rules.getChargingTimerSecondsRemaining());

  if (mysettings.canbusprotocol != CanBusProtocolEmulation::CANBUS_DISABLED && mysettings.dynamiccharge)
  {
    bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused,
                           R"("dyncv":%u,"dyncc":%u,)",
                           rules.DynamicChargeVoltage(),
                           rules.DynamicChargeCurrent());
  }

  // current
  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "\"current\":[");

  if (mysettings.currentMonitoringEnabled && currentMonitor.validReadings)
  {
    // Output current monitor values, this is inside an array, so could be more than 1
    bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused,
                           R"({"c":%.4f,"v":%.4f,"mahout":%u,"mahin":%u,"p":%.2f,"soc":%.2f,"dmahout":%u,"dmahin":%u)",
                           currentMonitor.modbus.current, currentMonitor.modbus.voltage, currentMonitor.modbus.milliamphour_out,
                           currentMonitor.modbus.milliamphour_in, currentMonitor.modbus.power, currentMonitor.stateofcharge,
                           currentMonitor.modbus.daily_milliamphour_out, currentMonitor.modbus.daily_milliamphour_in);

    bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, R"(,"time100":%u,"time20":%u,"time10":%u)", time100, time20, time10);

    bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "}");
  }
  else
  {
    bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "%s", nullstring);
  }

  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "],");

  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE, "\"errors\":[");
  int count = 0;
  for (auto v : rules.ErrorCodes)
  {
    if (v != InternalErrorCode::NoError)
    {
      // Comma if not zero
      if (count)
      {
        bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, ",");
      }
      bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "%u", v);
      count++;
    }
  }
  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "],\"warnings\":[");

  count = 0;
  for (auto v : rules.WarningCodes)
  {
    if (v != InternalWarningCode::NoWarning)
    {
      // Comma if not zero
      if (count)
      {
        bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, ",");
      }

      bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "%u", v);
      count++;
    }
  }
  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "],");

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
      bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "%u", cmi[i].voltagemVMax);
    }
    else
    {
      // Module is not yet valid so return null values...
      bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "%s", nullstring);
    }
  }
  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "],");

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

    bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "%u", rules.bankvoltage.at(i));
  }
  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "],");

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

  //  Send it...
  httpd_resp_send_chunk(req, httpbuf, bufferused);

  // Indicate last chunk (zero byte length)
  return httpd_resp_send_chunk(req, httpbuf, 0);
}

/// @brief
/// @param req Incoming HTTPD request handle
/// @return Error/success status
esp_err_t ha_handler(httpd_req_t *req)
{
  httpd_resp_set_type(req, "application/json");
  setNoStoreCacheControl(req);

  ESP_LOGI(TAG, "home assistant api request");

  char buffer[128];
  esp_err_t result = httpd_req_get_hdr_value_str(req, "ApiKey", buffer, sizeof(buffer));

  if (result != ESP_OK)
  {
    ESP_LOGE(TAG, "Missing header ApiKey");
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, nullptr);
  }

  if (strncmp(mysettings.homeassist_apikey, buffer, strlen(mysettings.homeassist_apikey)) != 0)
  {
    ESP_LOGE(TAG, "Unauthorized ApiKey=%s", buffer);
    return httpd_resp_send_err(req, HTTPD_401_UNAUTHORIZED, nullptr);
  }

  int bufferused = 0;

  // Output the first batch of settings/parameters/values
  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused,
                         R"({"activerules":%u,"chgmode":%u,"lowbankv":%u,"highbankv":%u,"lowcellv":%u,"highcellv":%u,"highextt":%i,"highintt":%i)",
                         rules.active_rule_count,
                         (unsigned int)rules.getChargingMode(),
                         rules.lowestBankVoltage,
                         rules.highestBankVoltage,
                         rules.lowestCellVoltage,
                         rules.highestCellVoltage,
                         rules.highestExternalTemp,
                         rules.highestInternalTemp);

  if (mysettings.currentMonitoringEnabled && currentMonitor.validReadings)
  {
    bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused,
                           R"(,"c":%.4f,"v":%.4f,"pwr":%.2f,"soc":%.2f)",
                           currentMonitor.modbus.current,
                           currentMonitor.modbus.voltage,
                           currentMonitor.modbus.power,
                           currentMonitor.stateofcharge);
  }

  if (mysettings.canbusprotocol != CanBusProtocolEmulation::CANBUS_DISABLED && mysettings.dynamiccharge)
  {
    bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused,
                           R"(,"dyncv":%u,"dyncc":%u)",
                           rules.DynamicChargeVoltage(),
                           rules.DynamicChargeCurrent());
  }

  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused,
                         R"(,"chgallow":%u,"dischgallow":%u)",
                         rules.IsChargeAllowed(&mysettings) ? 1 : 0,
                         rules.IsDischargeAllowed(&mysettings) ? 1 : 0);

  bufferused += snprintf(&httpbuf[bufferused], BUFSIZE - bufferused, "}");
  return httpd_resp_send(req, httpbuf, bufferused);
}

esp_err_t api_handler(httpd_req_t *req)
{
  if (!validateXSS(req))
  {
    return ESP_FAIL;
  }

  const std::array<std::string, 16> uri_array = {
      "monitor2", "monitor3", "integration",
      "settings", "rules", "rs485settings",
      "currentmonitor", "avrstatus", "modules",
      "identifyModule", "storage", "avrstorage",
      "chargeconfig", "tileconfig", "history",
      "diagnostic"};

  const std::array<std::function<esp_err_t(httpd_req_t * req)>, 16> func_ptr = {
      content_handler_monitor2, content_handler_monitor3, content_handler_integration,
      content_handler_settings, content_handler_rules, content_handler_rs485settings,
      content_handler_currentmonitor, content_handler_avrstatus, content_handler_modules,
      content_handler_identifymodule, content_handler_storage, content_handler_avrstorage,
      content_handler_chargeconfig, content_handler_tileconfig, content_handler_history,
      content_handler_diagnostic};

  // Ensure arrays are equal length
  assert(uri_array.size() == func_ptr.size());

  auto name = std::string(req->uri);
  if (name.rfind("/api/", 0) == 0)
  {
    // skip over first 5 characters "/api/" characters
    name = name.substr(5);
  }

  // Don't forget URLs can have a querystring attached to them!
  auto questionmark = name.find("?", 0);
  if (questionmark > 0)
  {
    // Strip string at querystring
    name = name.substr(0, questionmark);
  }

  for (size_t i = 0; i < uri_array.size(); i++)
  {
    if (name.compare(uri_array.at(i)) == 0)
    {
      // Found it
      ESP_LOGI(TAG, "API call: %s", name.c_str());
      httpd_resp_set_type(req, "application/json");
      setNoStoreCacheControl(req);
      return func_ptr.at(i)(req);
    }
  }

  ESP_LOGE(TAG, "No API match: %s", name.c_str());

  return httpd_resp_send_500(req);
}
