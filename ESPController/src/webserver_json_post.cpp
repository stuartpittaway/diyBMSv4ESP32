#include "webserver.h"
#include "webserver_json_post.h"
#include "webserver_helper_funcs.h"

#include "SoftAP.h"

esp_err_t post_savebankconfig_json_handler(httpd_req_t *req)
{
    if (!getPostDataIntoBuffer(req))
    {
        // Fail...
        return httpd_resp_send_500(req);
    }
    bool urlEncoded = HasURLEncodedHeader(req);

    // Need to validate POST variable XSS....
    if (!validateXSSWithPOST(req, httpbuf, urlEncoded))
    {
        return ESP_FAIL;
    }

    uint8_t totalSeriesModules = 1;
    uint8_t totalBanks = 1;
    uint16_t baudrate = COMMS_BAUD_RATE;

    uint32_t tempVariable;

    if (GetUint32FromKeyValue(httpbuf, "totalSeriesModules", &tempVariable, urlEncoded))
    {
        // Obviously could overflow
        totalSeriesModules = (uint8_t)tempVariable;

        if (GetUint32FromKeyValue(httpbuf, "totalBanks", &tempVariable, urlEncoded))
        {
            // Obviously could overflow
            totalBanks = (uint8_t)tempVariable;

            if (GetUint32FromKeyValue(httpbuf, "baudrate", &tempVariable, urlEncoded))
            {
                // Obviously could overflow
                baudrate = (uint16_t)tempVariable;

                if (totalSeriesModules * totalBanks <= maximum_controller_cell_modules)
                {
                    _mysettings->totalNumberOfSeriesModules = totalSeriesModules;
                    _mysettings->totalNumberOfBanks = totalBanks;
                    _mysettings->baudRate = baudrate;
                    saveConfiguration();

                    return SendSuccess(req);
                }
            }
        }
    }

    return SendFailure(req);
}

esp_err_t post_saventp_json_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "JSON call");

    if (!getPostDataIntoBuffer(req))
    {
        // Fail...
        return httpd_resp_send_500(req);
    }

    bool urlEncoded = HasURLEncodedHeader(req);

    // Need to validate POST variable XSS....
    if (!validateXSSWithPOST(req, httpbuf, urlEncoded))
    {
        return ESP_FAIL;
    }

    uint32_t tempVariable;
    if (GetUint32FromKeyValue(httpbuf, "NTPZoneHour", &tempVariable, urlEncoded))
    {
        _mysettings->timeZone = (uint8_t)tempVariable;
    }
    if (GetUint32FromKeyValue(httpbuf, "NTPZoneMin", &tempVariable, urlEncoded))
    {
        _mysettings->minutesTimeZone = (uint8_t)tempVariable;
    }

    if (GetTextFromKeyValue(httpbuf, "NTPServer", _mysettings->ntpServer, sizeof(_mysettings->ntpServer), urlEncoded))
    {
    }

    // HTML Boolean value, so element is not POST'ed if FALSE/OFF
    if (GetBoolFromKeyValue(httpbuf, "NTPDST", &_mysettings->daylight, urlEncoded))
    {
    }
    else
    {
        _mysettings->daylight = false;
    }

    saveConfiguration();

    return SendSuccess(req);
}

esp_err_t post_savemqtt_json_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "JSON call");

    if (!getPostDataIntoBuffer(req))
    {
        // Fail...
        return httpd_resp_send_500(req);
    }

    bool urlEncoded = HasURLEncodedHeader(req);

    // Need to validate POST variable XSS....
    if (!validateXSSWithPOST(req, httpbuf, urlEncoded))
    {
        return ESP_FAIL;
    }
    uint32_t tempVariable;

    _mysettings->mqtt_enabled = false;

    if (GetBoolFromKeyValue(httpbuf, "mqttEnabled", &_mysettings->mqtt_enabled, urlEncoded))
    {
    }

    if (GetTextFromKeyValue(httpbuf, "mqttTopic", _mysettings->mqtt_topic, sizeof(_mysettings->mqtt_topic), urlEncoded))
    {
    }

    if (GetUint32FromKeyValue(httpbuf, "mqttPort", &tempVariable, urlEncoded))
    {
        _mysettings->mqtt_port = (uint8_t)tempVariable;
    }

    if (GetTextFromKeyValue(httpbuf, "mqttServer", _mysettings->mqtt_server, sizeof(_mysettings->mqtt_server), urlEncoded))
    {
    }

    if (GetTextFromKeyValue(httpbuf, "mqttUsername", _mysettings->mqtt_username, sizeof(_mysettings->mqtt_username), urlEncoded))
    {
    }

    if (GetTextFromKeyValue(httpbuf, "mqttPassword", _mysettings->mqtt_password, sizeof(_mysettings->mqtt_password), urlEncoded))
    {
    }

    saveConfiguration();

    return SendSuccess(req);
}

esp_err_t post_saveglobalsetting_json_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "JSON call");

    if (!getPostDataIntoBuffer(req))
    {
        // Fail...
        return httpd_resp_send_500(req);
    }

    bool urlEncoded = HasURLEncodedHeader(req);

    // Need to validate POST variable XSS....
    if (!validateXSSWithPOST(req, httpbuf, urlEncoded))
    {
        return ESP_FAIL;
    }
    uint32_t tempVariable;

    if (GetUint32FromKeyValue(httpbuf, "BypassOverTempShutdown", &tempVariable, urlEncoded))
    {
        _mysettings->BypassOverTempShutdown = (uint8_t)tempVariable;

        if (GetUint32FromKeyValue(httpbuf, "BypassThresholdmV", &tempVariable, urlEncoded))
        {
            _mysettings->BypassThresholdmV = (uint16_t)tempVariable;

            if (_prg->sendSaveGlobalSetting(_mysettings->BypassThresholdmV, _mysettings->BypassOverTempShutdown))
            {

                saveConfiguration();
                uint8_t totalModules = _mysettings->totalNumberOfBanks * _mysettings->totalNumberOfSeriesModules;

                for (uint8_t i = 0; i < totalModules; i++)
                {
                    if (cmi[i].valid)
                    {
                        cmi[i].BypassThresholdmV = _mysettings->BypassThresholdmV;
                        cmi[i].BypassOverTempShutdown = _mysettings->BypassOverTempShutdown;
                    }
                }

                // Just returns NULL
                return SendSuccess(req);
            }
        }
    }

    return SendFailure(req);
}

esp_err_t post_saveinfluxdbsetting_json_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "JSON call");

    if (!getPostDataIntoBuffer(req))
    {
        // Fail...
        return httpd_resp_send_500(req);
    }

    bool urlEncoded = HasURLEncodedHeader(req);

    // Need to validate POST variable XSS....
    if (!validateXSSWithPOST(req, httpbuf, urlEncoded))
    {
        return ESP_FAIL;
    }

    _mysettings->influxdb_enabled = false;
    if (GetBoolFromKeyValue(httpbuf, "influxEnabled", &_mysettings->influxdb_enabled, urlEncoded))
    {
    }

    if (GetTextFromKeyValue(httpbuf, "influxUrl", _mysettings->influxdb_serverurl, sizeof(_mysettings->influxdb_serverurl), urlEncoded))
    {
    }
    if (GetTextFromKeyValue(httpbuf, "influxDatabase", _mysettings->influxdb_databasebucket, sizeof(_mysettings->influxdb_databasebucket), urlEncoded))
    {
    }
    if (GetTextFromKeyValue(httpbuf, "influxOrgId", _mysettings->influxdb_orgid, sizeof(_mysettings->influxdb_orgid), urlEncoded))
    {
    }
    if (GetTextFromKeyValue(httpbuf, "influxToken", _mysettings->influxdb_apitoken, sizeof(_mysettings->influxdb_apitoken), urlEncoded))
    {
    }

    saveConfiguration();

    return SendSuccess(req);
}

esp_err_t post_saveconfigurationtosdcard_json_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "JSON call");

    if (!getPostDataIntoBuffer(req))
    {
        // Fail...
        return httpd_resp_send_500(req);
    }

    bool urlEncoded = HasURLEncodedHeader(req);

    // Need to validate POST variable XSS....
    if (!validateXSSWithPOST(req, httpbuf, urlEncoded))
    {
        return ESP_FAIL;
    }

    if (!_sd_card_installed)
    {
        return SendFailure(req);
    }

    if (_hal->GetVSPIMutex())
    {

        struct tm timeinfo;

        // getLocalTime has delay() functions in it :-(
        if (getLocalTime(&timeinfo, 1))
        {
            timeinfo.tm_year += 1900;
            // Month is 0 to 11 based!
            timeinfo.tm_mon++;
        }
        else
        {
            memset(&timeinfo, 0, sizeof(tm));
        }

        char filename[128];
        sprintf(filename, "/backup_config_%04u%02u%02u_%02u%02u%02u.json", timeinfo.tm_year, timeinfo.tm_mon, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);

        // ESP_LOGI(TAG, "Creating folder");
        //_sdcard->mkdir("/diybms");

        // Get the file
        ESP_LOGI(TAG, "Generating SD file %s", filename);

        if (_sdcard->exists(filename))
        {
            ESP_LOGI(TAG, "Delete existing file %s", filename);
            _sdcard->remove(filename);
        }

        DynamicJsonDocument doc(4096);

        // This code builds up a JSON document which mirrors the structure "diybms_eeprom_settings"
        JsonObject root = doc.createNestedObject("diybms_settings");

        root["totalNumberOfBanks"] = _mysettings->totalNumberOfBanks;
        root["totalNumberOfSeriesModules"] = _mysettings->totalNumberOfSeriesModules;
        root["baudRate"] = _mysettings->baudRate;

        root["graph_voltagehigh"] = _mysettings->graph_voltagehigh;
        root["graph_voltagelow"] = _mysettings->graph_voltagelow;

        root["BypassOverTempShutdown"] = _mysettings->BypassOverTempShutdown;
        root["BypassThresholdmV"] = _mysettings->BypassThresholdmV;

        root["timeZone"] = _mysettings->timeZone;
        root["minutesTimeZone"] = _mysettings->minutesTimeZone;
        root["daylight"] = _mysettings->daylight;
        root["ntpServer"] = _mysettings->ntpServer;

        root["loggingEnabled"] = _mysettings->loggingEnabled;
        root["loggingFrequencySeconds"] = _mysettings->loggingFrequencySeconds;

        root["currentMonitoringEnabled"] = _mysettings->currentMonitoringEnabled;
        root["currentMonitoringModBusAddress"] = _mysettings->currentMonitoringModBusAddress;

        root["rs485baudrate"] = _mysettings->rs485baudrate;
        root["rs485databits"] = _mysettings->rs485databits;
        root["rs485parity"] = _mysettings->rs485parity;
        root["rs485stopbits"] = _mysettings->rs485stopbits;

        root["language"] = _mysettings->language;

        root["VictronEnabled"] = _mysettings->VictronEnabled;

        JsonObject mqtt = root.createNestedObject("mqtt");
        mqtt["enabled"] = _mysettings->mqtt_enabled;
        mqtt["port"] = _mysettings->mqtt_port;
        mqtt["server"] = _mysettings->mqtt_server;
        mqtt["topic"] = _mysettings->mqtt_topic;
        mqtt["username"] = _mysettings->mqtt_username;
        mqtt["password"] = _mysettings->mqtt_password;

        JsonObject influxdb = root.createNestedObject("influxdb");
        influxdb["enabled"] = _mysettings->influxdb_enabled;
        influxdb["apitoken"] = _mysettings->influxdb_apitoken;
        influxdb["bucket"] = _mysettings->influxdb_databasebucket;
        influxdb["org"] = _mysettings->influxdb_orgid;
        influxdb["url"] = _mysettings->influxdb_serverurl;

        JsonObject outputs = root.createNestedObject("outputs");

        JsonArray d = outputs.createNestedArray("default");
        JsonArray t = outputs.createNestedArray("type");
        for (uint8_t i = 0; i < RELAY_TOTAL; i++)
        {
            d.add(_mysettings->rulerelaydefault[i]);
            t.add(_mysettings->relaytype[i]);
        }

        JsonObject rules = root.createNestedObject("rules");
        for (uint8_t rr = 0; rr < RELAY_RULES; rr++)
        {
            String elementName = String("rule") + String(rr);

            // Map enum to string so when this file is re-imported we are not locked to specific index offsets
            // which may no longer map to the correct rule
            switch (rr)
            {
            case Rule::EmergencyStop:
                elementName = String("EmergencyStop");
                break;
            case Rule::BMSError:
                elementName = String("BMSError");
                break;
            case Rule::CurrentMonitorOverCurrentAmps:
                elementName = String("CurrentMonitorOverCurrentAmps");
                break;
            case Rule::ModuleOverVoltage:
                elementName = String("ModuleOverVoltage");
                break;
            case Rule::ModuleUnderVoltage:
                elementName = String("ModuleUnderVoltage");
                break;
            case Rule::ModuleOverTemperatureInternal:
                elementName = String("ModuleOverTemperatureInternal");
                break;
            case Rule::ModuleUnderTemperatureInternal:
                elementName = String("ModuleUnderTemperatureInternal");
                break;
            case Rule::ModuleOverTemperatureExternal:
                elementName = String("ModuleOverTemperatureExternal");
                break;
            case Rule::ModuleUnderTemperatureExternal:
                elementName = String("ModuleUnderTemperatureExternal");
                break;
            case Rule::CurrentMonitorOverVoltage:
                elementName = String("CurrentMonitorOverVoltage");
                break;
            case Rule::CurrentMonitorUnderVoltage:
                elementName = String("CurrentMonitorUnderVoltage");
                break;
            case Rule::BankOverVoltage:
                elementName = String("BankOverVoltage");
                break;
            case Rule::BankUnderVoltage:
                elementName = String("BankUnderVoltage");
                break;
            case Rule::Timer2:
                elementName = String("Timer2");
                break;
            case Rule::Timer1:
                elementName = String("Timer1");
                break;
            }

            JsonObject state = rules.createNestedObject(elementName);

            state["value"] = _mysettings->rulevalue[rr];
            state["hysteresis"] = _mysettings->rulehysteresis[rr];

            JsonArray relaystate = state.createNestedArray("state");
            for (uint8_t rt = 0; rt < RELAY_TOTAL; rt++)
            {
                relaystate.add(_mysettings->rulerelaystate[rr][rt]);
            }
        } // end for

        JsonObject victron = root.createNestedObject("victron");
        JsonArray cvl = victron.createNestedArray("cvl");
        JsonArray ccl = victron.createNestedArray("ccl");
        JsonArray dcl = victron.createNestedArray("dcl");
        for (uint8_t i = 0; i < 3; i++)
        {
            cvl.add(_mysettings->cvl[i]);
            ccl.add(_mysettings->ccl[i]);
            dcl.add(_mysettings->dcl[i]);
        }

        /*
    struct diybms_eeprom_settings
    {
      //Use a bit pattern to indicate the relay states
      RelayState rulerelaystate[RELAY_RULES][RELAY_TOTAL];
    };
    */

        // wifi["password"] = DIYBMSSoftAP::Config()->wifi_passphrase;

        File file = _sdcard->open(filename, "w");
        serializeJson(doc, file);
        file.close();

        _hal->ReleaseVSPIMutex();
    }

    return SendSuccess(req);
}

esp_err_t post_savewificonfigtosdcard_json_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "JSON call");

    if (!getPostDataIntoBuffer(req))
    {
        // Fail...
        return httpd_resp_send_500(req);
    }

    bool urlEncoded = HasURLEncodedHeader(req);

    // Need to validate POST variable XSS....
    if (!validateXSSWithPOST(req, httpbuf, urlEncoded))
    {
        return ESP_FAIL;
    }

    if (!_sd_card_installed)
    {
        return SendFailure(req);
    }

    if (_hal->GetVSPIMutex())
    {
        const char *wificonfigfilename = "/diybms/wifi.json";

        ESP_LOGI(TAG, "Creating folder");
        _sdcard->mkdir("/diybms");

        // Get the file
        ESP_LOGI(TAG, "Generating SD file %s", wificonfigfilename);
        StaticJsonDocument<512> doc;

        JsonObject wifi = doc.createNestedObject("wifi");
        wifi["ssid"] = DIYBMSSoftAP::Config()->wifi_ssid;
        wifi["password"] = DIYBMSSoftAP::Config()->wifi_passphrase;

        if (_sdcard->exists(wificonfigfilename))
        {
            ESP_LOGI(TAG, "Delete existing file %s", wificonfigfilename);
            _sdcard->remove(wificonfigfilename);
        }

        File file = _sdcard->open(wificonfigfilename, "w");
        serializeJson(doc, file);
        file.close();

        _hal->ReleaseVSPIMutex();
    }

    return SendSuccess(req);
}

esp_err_t post_savesetting_json_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "JSON call");

    if (!getPostDataIntoBuffer(req))
    {
        // Fail...
        return httpd_resp_send_500(req);
    }

    bool urlEncoded = HasURLEncodedHeader(req);

    // Need to validate POST variable XSS....
    if (!validateXSSWithPOST(req, httpbuf, urlEncoded))
    {
        return ESP_FAIL;
    }

    uint32_t tempVariable;

    if (GetUint32FromKeyValue(httpbuf, "m", &tempVariable, urlEncoded))
    {
        uint8_t m = (uint8_t)tempVariable;

        if (m > maximum_controller_cell_modules)
        {
            // request->send(500, "text/plain", "Wrong parameters");
            ESP_LOGE(TAG, "Invalid module %u", m);
            return httpd_resp_send_500(req);
        }
        else
        {

            uint8_t BypassOverTempShutdown = 0xFF;
            uint16_t BypassThresholdmV = 0xFFFF;

            // Resistance of bypass load
            // float LoadResistance = 0xFFFF;
            // Voltage Calibration
            float Calibration = 0xFFFF;
            // Reference voltage (millivolt) normally 2.00mV
            // float mVPerADC = 0xFFFF;
            // Internal Thermistor settings
            // uint16_t Internal_BCoefficient = 0xFFFF;
            // External Thermistor settings
            // uint16_t External_BCoefficient = 0xFFFF;

            if (GetUint32FromKeyValue(httpbuf, "BypassOverTempShutdown", &tempVariable, urlEncoded))
            {
                BypassOverTempShutdown = (uint8_t)tempVariable;

                if (GetUint32FromKeyValue(httpbuf, "BypassThresholdmV", &tempVariable, urlEncoded))
                {
                    BypassThresholdmV = (uint16_t)tempVariable;

                    if (GetFloatFromKeyValue(httpbuf, "Calib", &Calibration, urlEncoded))
                    {
                        if (_prg->sendSaveSetting(m, BypassThresholdmV, BypassOverTempShutdown, Calibration))
                        {
                            clearModuleValues(m);

                            return SendSuccess(req);
                        }
                    }
                }
            }
        }

        return SendFailure(req);
    }
}