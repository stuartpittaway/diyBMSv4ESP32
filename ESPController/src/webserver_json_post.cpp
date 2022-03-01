#include "webserver.h"
#include "webserver_json_post.h"
#include "webserver_helper_funcs.h"

#include "SoftAP.h"

esp_err_t post_savebankconfig_json_handler(httpd_req_t *req, bool urlEncoded)
{
    uint8_t totalSeriesModules = 1;
    uint8_t totalBanks = 1;
    uint16_t baudrate = COMMS_BAUD_RATE;
    uint16_t interpacketgap = 6000;

    uint32_t tempVariable;

    if (GetKeyValue(httpbuf, "totalSeriesModules", &tempVariable, urlEncoded))
    {
        // Obviously could overflow
        totalSeriesModules = (uint8_t)tempVariable;

        if (GetKeyValue(httpbuf, "totalBanks", &tempVariable, urlEncoded))
        {
            // Obviously could overflow
            totalBanks = (uint8_t)tempVariable;

            if (GetKeyValue(httpbuf, "baudrate", &baudrate, urlEncoded))
            {

                if (GetKeyValue(httpbuf, "interpacketgap", &interpacketgap, urlEncoded))
                {
                    if (totalSeriesModules * totalBanks <= maximum_controller_cell_modules)
                    {
                        mysettings.totalNumberOfSeriesModules = totalSeriesModules;
                        mysettings.totalNumberOfBanks = totalBanks;
                        mysettings.baudRate = baudrate;
                        mysettings.interpacketgap = interpacketgap;
                        saveConfiguration();

                        return SendSuccess(req);
                    }
                }
            }
        }
    }

    return SendFailure(req);
}

esp_err_t post_saventp_json_handler(httpd_req_t *req, bool urlEncoded)
{
    // uint32_t tempVariable;
    if (GetKeyValue(httpbuf, "NTPZoneHour", &mysettings.timeZone, urlEncoded))
    {
    }
    if (GetKeyValue(httpbuf, "NTPZoneMin", &mysettings.minutesTimeZone, urlEncoded))
    {
    }

    if (GetTextFromKeyValue(httpbuf, "NTPServer", mysettings.ntpServer, sizeof(mysettings.ntpServer), urlEncoded))
    {
    }

    // HTML Boolean value, so element is not POST'ed if FALSE/OFF
    mysettings.daylight = false;
    if (GetKeyValue(httpbuf, "NTPDST", &mysettings.daylight, urlEncoded))
    {
    }

    saveConfiguration();

    return SendSuccess(req);
}

esp_err_t post_savemqtt_json_handler(httpd_req_t *req, bool urlEncoded)
{
    mysettings.mqtt_enabled = false;

    if (GetKeyValue(httpbuf, "mqttEnabled", &mysettings.mqtt_enabled, urlEncoded))
    {
    }

    if (GetTextFromKeyValue(httpbuf, "mqttTopic", mysettings.mqtt_topic, sizeof(mysettings.mqtt_topic), urlEncoded))
    {
    }

    if (GetKeyValue(httpbuf, "mqttPort", &mysettings.mqtt_port, urlEncoded))
    {
    }

    if (GetTextFromKeyValue(httpbuf, "mqttServer", mysettings.mqtt_server, sizeof(mysettings.mqtt_server), urlEncoded))
    {
    }

    if (GetTextFromKeyValue(httpbuf, "mqttUsername", mysettings.mqtt_username, sizeof(mysettings.mqtt_username), urlEncoded))
    {
    }

    if (GetTextFromKeyValue(httpbuf, "mqttPassword", mysettings.mqtt_password, sizeof(mysettings.mqtt_password), urlEncoded))
    {
    }

    saveConfiguration();

    return SendSuccess(req);
}

esp_err_t post_saveglobalsetting_json_handler(httpd_req_t *req, bool urlEncoded)
{
    if (GetKeyValue(httpbuf, "BypassOverTempShutdown", &mysettings.BypassOverTempShutdown, urlEncoded))
    {

        if (GetKeyValue(httpbuf, "BypassThresholdmV", &mysettings.BypassThresholdmV, urlEncoded))
        {

            if (prg.sendSaveGlobalSetting(mysettings.BypassThresholdmV, mysettings.BypassOverTempShutdown))
            {

                saveConfiguration();
                uint8_t totalModules = mysettings.totalNumberOfBanks * mysettings.totalNumberOfSeriesModules;

                for (uint8_t i = 0; i < totalModules; i++)
                {
                    if (cmi[i].valid)
                    {
                        cmi[i].BypassThresholdmV = mysettings.BypassThresholdmV;
                        cmi[i].BypassOverTempShutdown = mysettings.BypassOverTempShutdown;
                    }
                }

                // Just returns NULL
                return SendSuccess(req);
            }
        }
    }

    return SendFailure(req);
}

/*
Restart controller from web interface
*/
esp_err_t post_restartcontroller_json_handler(httpd_req_t *req, bool urlEncoded)
{
    // Reboot!
    ESP.restart();

    return SendSuccess(req);
}

esp_err_t post_saveinfluxdbsetting_json_handler(httpd_req_t *req, bool urlEncoded)
{
    mysettings.influxdb_enabled = false;
    if (GetKeyValue(httpbuf, "influxEnabled", &mysettings.influxdb_enabled, urlEncoded))
    {
    }

    mysettings.influxdb_loggingFreqSeconds = 15;
    if (GetKeyValue(httpbuf, "influxFreq", &mysettings.influxdb_loggingFreqSeconds, urlEncoded))
    {
    }

    if (GetTextFromKeyValue(httpbuf, "influxUrl", mysettings.influxdb_serverurl, sizeof(mysettings.influxdb_serverurl), urlEncoded))
    {
    }
    if (GetTextFromKeyValue(httpbuf, "influxDatabase", mysettings.influxdb_databasebucket, sizeof(mysettings.influxdb_databasebucket), urlEncoded))
    {
    }
    if (GetTextFromKeyValue(httpbuf, "influxOrgId", mysettings.influxdb_orgid, sizeof(mysettings.influxdb_orgid), urlEncoded))
    {
    }
    if (GetTextFromKeyValue(httpbuf, "influxToken", mysettings.influxdb_apitoken, sizeof(mysettings.influxdb_apitoken), urlEncoded))
    {
    }

    if (mysettings.influxdb_loggingFreqSeconds < 5)
    {
        // Safety check
        mysettings.influxdb_loggingFreqSeconds = 5;
    }

    saveConfiguration();

    return SendSuccess(req);
}

esp_err_t post_saveconfigurationtosdcard_json_handler(httpd_req_t *req, bool urlEncoded)
{
    if (!_sd_card_installed)
    {
        return SendFailure(req);
    }

    if (hal.GetVSPIMutex())
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

        if (SD.exists(filename))
        {
            ESP_LOGI(TAG, "Delete existing file %s", filename);
            SD.remove(filename);
        }

        DynamicJsonDocument doc(4096);

        // This code builds up a JSON document which mirrors the structure "diybms_eeprom_settings"
        JsonObject root = doc.createNestedObject("diybms_settings");

        root["totalNumberOfBanks"] = mysettings.totalNumberOfBanks;
        root["totalNumberOfSeriesModules"] = mysettings.totalNumberOfSeriesModules;
        root["baudRate"] = mysettings.baudRate;
        root["interpacketgap"] = mysettings.interpacketgap;

        root["graph_voltagehigh"] = mysettings.graph_voltagehigh;
        root["graph_voltagelow"] = mysettings.graph_voltagelow;

        root["BypassOverTempShutdown"] = mysettings.BypassOverTempShutdown;
        root["BypassThresholdmV"] = mysettings.BypassThresholdmV;

        root["timeZone"] = mysettings.timeZone;
        root["minutesTimeZone"] = mysettings.minutesTimeZone;
        root["daylight"] = mysettings.daylight;
        root["ntpServer"] = mysettings.ntpServer;

        root["loggingEnabled"] = mysettings.loggingEnabled;
        root["loggingFrequencySeconds"] = mysettings.loggingFrequencySeconds;

        root["currentMonitoringEnabled"] = mysettings.currentMonitoringEnabled;
        root["currentMonitoringModBusAddress"] = mysettings.currentMonitoringModBusAddress;

        root["rs485baudrate"] = mysettings.rs485baudrate;
        root["rs485databits"] = mysettings.rs485databits;
        root["rs485parity"] = mysettings.rs485parity;
        root["rs485stopbits"] = mysettings.rs485stopbits;

        root["language"] = mysettings.language;

        root["VictronEnabled"] = mysettings.VictronEnabled;

        JsonObject mqtt = root.createNestedObject("mqtt");
        mqtt["enabled"] = mysettings.mqtt_enabled;
        mqtt["port"] = mysettings.mqtt_port;
        mqtt["server"] = mysettings.mqtt_server;
        mqtt["topic"] = mysettings.mqtt_topic;
        mqtt["username"] = mysettings.mqtt_username;
        mqtt["password"] = mysettings.mqtt_password;

        JsonObject influxdb = root.createNestedObject("influxdb");
        influxdb["enabled"] = mysettings.influxdb_enabled;
        influxdb["apitoken"] = mysettings.influxdb_apitoken;
        influxdb["bucket"] = mysettings.influxdb_databasebucket;
        influxdb["org"] = mysettings.influxdb_orgid;
        influxdb["url"] = mysettings.influxdb_serverurl;
        influxdb["logfreq"] = mysettings.influxdb_loggingFreqSeconds;

        JsonObject outputs = root.createNestedObject("outputs");

        JsonArray d = outputs.createNestedArray("default");
        JsonArray t = outputs.createNestedArray("type");
        for (uint8_t i = 0; i < RELAY_TOTAL; i++)
        {
            d.add(mysettings.rulerelaydefault[i]);
            t.add(mysettings.relaytype[i]);
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

            state["value"] = mysettings.rulevalue[rr];
            state["hysteresis"] = mysettings.rulehysteresis[rr];

            JsonArray relaystate = state.createNestedArray("state");
            for (uint8_t rt = 0; rt < RELAY_TOTAL; rt++)
            {
                relaystate.add(mysettings.rulerelaystate[rr][rt]);
            }
        } // end for

        JsonObject victron = root.createNestedObject("victron");
        JsonArray cvl = victron.createNestedArray("cvl");
        JsonArray ccl = victron.createNestedArray("ccl");
        JsonArray dcl = victron.createNestedArray("dcl");
        for (uint8_t i = 0; i < 3; i++)
        {
            cvl.add(mysettings.cvl[i]);
            ccl.add(mysettings.ccl[i]);
            dcl.add(mysettings.dcl[i]);
        }

        /*
    struct diybms_eeprom_settings
    {
      //Use a bit pattern to indicate the relay states
      RelayState rulerelaystate[RELAY_RULES][RELAY_TOTAL];
    };
    */

        // wifi["password"] = DIYBMSSoftAP::Config().wifi_passphrase;

        File file = SD.open(filename, "w");
        serializeJson(doc, file);
        file.close();

        hal.ReleaseVSPIMutex();
    }

    return SendSuccess(req);
}

esp_err_t post_savewificonfigtosdcard_json_handler(httpd_req_t *req, bool urlEncoded)
{
    if (!_sd_card_installed)
    {
        return SendFailure(req);
    }

    if (hal.GetVSPIMutex())
    {
        const char *wificonfigfilename = "/diybms/wifi.json";

        ESP_LOGI(TAG, "Creating folder");
        SD.mkdir("/diybms");

        // Get the file
        ESP_LOGI(TAG, "Generating SD file %s", wificonfigfilename);
        StaticJsonDocument<512> doc;

        JsonObject wifi = doc.createNestedObject("wifi");
        wifi["ssid"] = DIYBMSSoftAP::Config()->wifi_ssid;
        wifi["password"] = DIYBMSSoftAP::Config()->wifi_passphrase;

        if (SD.exists(wificonfigfilename))
        {
            ESP_LOGI(TAG, "Delete existing file %s", wificonfigfilename);
            SD.remove(wificonfigfilename);
        }

        File file = SD.open(wificonfigfilename, "w");
        serializeJson(doc, file);
        file.close();

        hal.ReleaseVSPIMutex();
    }

    return SendSuccess(req);
}

esp_err_t post_savesetting_json_handler(httpd_req_t *req, bool urlEncoded)
{
    uint32_t tempVariable;

    if (GetKeyValue(httpbuf, "m", &tempVariable, urlEncoded))
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

            if (GetKeyValue(httpbuf, "BypassOverTempShutdown", &BypassOverTempShutdown, urlEncoded))
            {
                if (GetKeyValue(httpbuf, "BypassThresholdmV", &BypassThresholdmV, urlEncoded))
                {
                    if (GetKeyValue(httpbuf, "Calib", &Calibration, urlEncoded))
                    {
                        if (prg.sendSaveSetting(m, BypassThresholdmV, BypassOverTempShutdown, Calibration))
                        {
                            clearModuleValues(m);

                            return SendSuccess(req);
                        }
                    }
                }
            }
        }
    }
    return SendFailure(req);
}

esp_err_t post_savestorage_json_handler(httpd_req_t *req, bool urlEncoded)
{
    // HTML Boolean value, so element is not POST'ed if FALSE/OFF
    mysettings.loggingEnabled = false;
    if (GetKeyValue(httpbuf, "loggingEnabled", &mysettings.loggingEnabled, urlEncoded))
    {
    }

    if (GetKeyValue(httpbuf, "loggingFreq", &mysettings.loggingFrequencySeconds, urlEncoded))
    {
    }

    // Validate
    if (mysettings.loggingFrequencySeconds < 15 || mysettings.loggingFrequencySeconds > 600)
    {
        mysettings.loggingFrequencySeconds = 15;
    }

    saveConfiguration();

    return SendSuccess(req);
}

esp_err_t post_savedisplaysetting_json_handler(httpd_req_t *req, bool urlEncoded)
{
    // uint32_t tempVariable;

    if (GetKeyValue(httpbuf, "VoltageHigh", &mysettings.graph_voltagehigh, urlEncoded))
    {
    }

    if (GetKeyValue(httpbuf, "VoltageLow", &mysettings.graph_voltagelow, urlEncoded))
    {
    }

    // Validate high is greater than low
    if (mysettings.graph_voltagelow > mysettings.graph_voltagehigh || mysettings.graph_voltagelow < 0)
    {
        mysettings.graph_voltagelow = 0;
    }

    if (GetTextFromKeyValue(httpbuf, "Language", mysettings.language, sizeof(mysettings.language), urlEncoded))
    {
    }

    saveConfiguration();

    return SendSuccess(req);
}

esp_err_t post_resetcounters_json_handler(httpd_req_t *req, bool urlEncoded)
{
    // Ask modules to reset bad packet counters
    // If this fails, queue could be full so return error
    if (prg.sendBadPacketCounterReset() && prg.sendResetBalanceCurrentCounter())
    {
        canbus_messages_failed_sent = 0;
        canbus_messages_received = 0;
        canbus_messages_sent = 0;

        for (uint8_t i = 0; i < maximum_controller_cell_modules; i++)
        {
            cmi[i].badPacketCount = 0;
            cmi[i].PacketReceivedCount = 0;
        }

        // Reset internal counters on CONTROLLER
        receiveProc.ResetCounters();
        prg.ResetCounters();

        return SendSuccess(req);
    }

    return SendFailure(req);
}

esp_err_t post_sdmount_json_handler(httpd_req_t *req, bool urlEncoded)
{
    if (_avrsettings.programmingModeEnabled)
    {
        return SendFailure(req);
    }

    sdcardaction_callback(1);

    return SendSuccess(req);
}
esp_err_t post_sdunmount_json_handler(httpd_req_t *req, bool urlEncoded)
{
    if (_avrsettings.programmingModeEnabled)
    {
        return SendFailure(req);
    }

    sdcardaction_callback(0);

    return SendSuccess(req);
}

esp_err_t post_enableavrprog_json_handler(httpd_req_t *req, bool urlEncoded)
{
    sdcardaction_callback(0);

    _avrsettings.programmingModeEnabled = true;

    return SendSuccess(req);
}
esp_err_t post_disableavrprog_json_handler(httpd_req_t *req, bool urlEncoded)
{
    _avrsettings.programmingModeEnabled = false;

    // Try and remount the SD card
    sdcardaction_callback(1);

    return SendSuccess(req);
}

esp_err_t post_savers485settings_json_handler(httpd_req_t *req, bool urlEncoded)
{
    uint32_t tempVariable;

    if (GetKeyValue(httpbuf, "rs485baudrate", &tempVariable, urlEncoded))
    {
        mysettings.rs485baudrate = (int)tempVariable;
    }

    if (GetKeyValue(httpbuf, "rs485databit", &tempVariable, urlEncoded))
    {
        mysettings.rs485databits = (uart_word_length_t)tempVariable;
    }

    if (GetKeyValue(httpbuf, "rs485parity", &tempVariable, urlEncoded))
    {
        mysettings.rs485parity = (uart_parity_t)tempVariable;
    }

    if (GetKeyValue(httpbuf, "rs485stopbit", &tempVariable, urlEncoded))
    {
        mysettings.rs485stopbits = (uart_stop_bits_t)tempVariable;
    }

    saveConfiguration();

    ConfigureRS485();
    return SendSuccess(req);
}

esp_err_t post_savevictron_json_handler(httpd_req_t *req, bool urlEncoded)
{
    // uint32_t tempVariable;

    mysettings.VictronEnabled = false;
    if (GetKeyValue(httpbuf, "VictronEnabled", &mysettings.VictronEnabled, urlEncoded))
    {
    }

    for (int i = 0; i < 3; i++)
    {
        // TODO: Check return values are correct here... floats vs ints?
        String name = "cvl";
        name = name + i;

        float tempFloat;

        if (GetKeyValue(httpbuf, name.c_str(), &tempFloat, urlEncoded))
        {
            mysettings.cvl[i] = tempFloat * 10;
        }

        name = "ccl";
        name = name + i;
        if (GetKeyValue(httpbuf, name.c_str(), &tempFloat, urlEncoded))
        {
            mysettings.ccl[i] = tempFloat * 10;
        }

        name = "dcl";
        name = name + i;
        if (GetKeyValue(httpbuf, name.c_str(), &tempFloat, urlEncoded))
        {
            mysettings.dcl[i] = tempFloat * 10;
        }
    }

    saveConfiguration();

    return SendSuccess(req);
}

esp_err_t post_savecmrelay_json_handler(httpd_req_t *req, bool urlEncoded)
{
    currentmonitoring_struct newvalues;
    // Set everything to zero/false
    memset(&newvalues, 0, sizeof(currentmonitoring_struct));

    newvalues.TempCompEnabled = false;

    bool tempBool;

    if (GetKeyValue(httpbuf, "TempCompEnabled", &tempBool, urlEncoded))
    {
        newvalues.TempCompEnabled = tempBool;
    }

    if (GetKeyValue(httpbuf, "cmTMPOL", &tempBool, urlEncoded))
    {
        newvalues.RelayTriggerTemperatureOverLimit = tempBool;
    }

    if (GetKeyValue(httpbuf, "cmCURROL", &tempBool, urlEncoded))
    {
        newvalues.RelayTriggerCurrentOverLimit = tempBool;
    }

    if (GetKeyValue(httpbuf, "cmCURRUL", &tempBool, urlEncoded))
    {
        newvalues.RelayTriggerCurrentUnderLimit = tempBool;
    }

    if (GetKeyValue(httpbuf, "cmVOLTOL", &tempBool, urlEncoded))
    {
        newvalues.RelayTriggerVoltageOverlimit = tempBool;
    }

    if (GetKeyValue(httpbuf, "cmVOLTUL", &tempBool, urlEncoded))
    {
        newvalues.RelayTriggerVoltageUnderlimit = tempBool;
    }
    if (GetKeyValue(httpbuf, "cmPOL", &tempBool, urlEncoded))
    {
        newvalues.RelayTriggerPowerOverLimit = tempBool;
    }

    CurrentMonitorSetRelaySettings(newvalues);

    return SendSuccess(req);
}

esp_err_t post_savecmbasic_json_handler(httpd_req_t *req, bool urlEncoded)
{
    int shuntmaxcur = 0;
    if (GetKeyValue(httpbuf, "shuntmaxcur", &shuntmaxcur, urlEncoded))
    {
        int shuntmv = 0;
        if (GetKeyValue(httpbuf, "shuntmv", &shuntmv, urlEncoded))
        {
            uint16_t batterycapacity = 0;
            if (GetKeyValue(httpbuf, "cmbatterycapacity", &batterycapacity, urlEncoded))
            {
                float fullchargevolt = 0;
                if (GetKeyValue(httpbuf, "cmfullchargevolt", &fullchargevolt, urlEncoded))
                {
                    float tailcurrent = 0;
                    if (GetKeyValue(httpbuf, "cmtailcurrent", &fullchargevolt, urlEncoded))
                    {
                        float chargeefficiency = 0;
                        if (GetKeyValue(httpbuf, "cmchargeefficiency", &fullchargevolt, urlEncoded))
                        {
                            CurrentMonitorSetBasicSettings(shuntmv, shuntmaxcur, batterycapacity, fullchargevolt, tailcurrent, chargeefficiency);
                            return SendSuccess(req);
                        }
                    }
                }
            }
        }
    }

    return SendFailure(req);
}

esp_err_t post_savecmadvanced_json_handler(httpd_req_t *req, bool urlEncoded)
{
    currentmonitoring_struct newvalues;
    // Set everything to zero/false
    memset(&newvalues, 0, sizeof(currentmonitoring_struct));

    // TODO: We need more validation here to check values are correct and all supplied.

    if (GetKeyValue(httpbuf, "cmcalibration", &newvalues.modbus.shuntcal, urlEncoded))
    {
    }
    if (GetKeyValue(httpbuf, "cmtemplimit", &newvalues.modbus.temperaturelimit, urlEncoded))
    {
    }
    if (GetKeyValue(httpbuf, "cmundervlimit", &newvalues.modbus.undervoltagelimit, urlEncoded))
    {
    }
    if (GetKeyValue(httpbuf, "cmovervlimit", &newvalues.modbus.overvoltagelimit, urlEncoded))
    {
    }
    if (GetKeyValue(httpbuf, "cmoverclimit", &newvalues.modbus.overcurrentlimit, urlEncoded))
    {
    }
    if (GetKeyValue(httpbuf, "cmunderclimit", &newvalues.modbus.undercurrentlimit, urlEncoded))
    {
    }
    if (GetKeyValue(httpbuf, "cmoverplimit", &newvalues.modbus.overpowerlimit, urlEncoded))
    {
    }
    if (GetKeyValue(httpbuf, "cmtempcoeff", &newvalues.modbus.shunttempcoefficient, urlEncoded))
    {
    }

    CurrentMonitorSetAdvancedSettings(newvalues);

    return SendSuccess(req);
}

esp_err_t post_avrprog_json_handler(httpd_req_t *req, bool urlEncoded)
{
    uint16_t filenumber;

    if (!GetKeyValue(httpbuf, "file", &filenumber, urlEncoded))
    {
        return SendFailure(req);
    }

    DynamicJsonDocument doc(512);

    int bufferused = 0;

    if (_sd_card_installed)
    {
        httpd_resp_set_type(req, "application/json");
        setNoStoreCacheControl(req);

        doc["message"] = "Failed: Unable to program AVR whilst SD Card is mounted";
        bufferused += serializeJson(doc, httpbuf, BUFSIZE);

        return httpd_resp_send(req, httpbuf, bufferused);
    }

    if (!_avrsettings.programmingModeEnabled)
    {
        httpd_resp_set_type(req, "application/json");
        setNoStoreCacheControl(req);

        doc["message"] = "Failed: Programming mode not enabled";
        bufferused += serializeJson(doc, httpbuf, BUFSIZE);

        return httpd_resp_send(req, httpbuf, bufferused);
    }

    String manifestfilename = String("/avr/manifest.json");

    if (LittleFS.exists(manifestfilename))
    {
        DynamicJsonDocument jsonmanifest(3000);
        File file = LittleFS.open(manifestfilename);
        DeserializationError error = deserializeJson(jsonmanifest, file);
        if (error != DeserializationError::Ok)
        {
            ESP_LOGE(TAG, "Error deserialize Json");
            return SendFailure(req);
        }
        else
        {
            // File open
            // ESP_LOGI(TAG, "Loaded manifest.json");

            JsonArray toplevel = jsonmanifest["avrprog"];

            int arraySize = jsonmanifest["avrprog"].size();

            if (filenumber > arraySize)
            {
                ESP_LOGE(TAG, "Index outsize array %i > %i", filenumber, arraySize);
                return SendFailure(req);
            }

            JsonObject x = toplevel[filenumber];

            _avrsettings.efuse = strtoul(x["efuse"].as<String>().c_str(), nullptr, 16);
            _avrsettings.hfuse = strtoul(x["hfuse"].as<String>().c_str(), nullptr, 16);
            _avrsettings.lfuse = strtoul(x["lfuse"].as<String>().c_str(), nullptr, 16);
            _avrsettings.mcu = strtoul(x["mcu"].as<String>().c_str(), nullptr, 16);

            String avrfilename = String("/avr/") + x["name"].as<String>();

            avrfilename.toCharArray(_avrsettings.filename, sizeof(_avrsettings.filename));
        }
        file.close();

        _avrsettings.progresult = 0xFF;
        _avrsettings.inProgress = true;

        ESP_LOGI(TAG, "Notify AVR task");
        // Fire task to start the AVR programming
        xTaskNotify(avrprog_task_handle, 0x00, eNotifyAction::eNoAction);
        httpd_resp_set_type(req, "application/json");
        setNoStoreCacheControl(req);

        doc["started"] = 1;
        doc["message"] = "Started";

        bufferused += serializeJson(doc, httpbuf, BUFSIZE);

        return httpd_resp_send(req, httpbuf, bufferused);
    }
    else
    {
        ESP_LOGE(TAG, "Cannot find file %s", manifestfilename);
    }

    // No files!
    return SendFailure(req);
}

esp_err_t post_savecurrentmon_json_handler(httpd_req_t *req, bool urlEncoded)
{
    mysettings.currentMonitoringEnabled = false;
    if (GetKeyValue(httpbuf, "CurrentMonEnabled", &mysettings.currentMonitoringEnabled, urlEncoded))
    {
    }

    if (GetKeyValue(httpbuf, "modbusAddress", &mysettings.currentMonitoringModBusAddress, urlEncoded))
    {
    }

    uint8_t CurrentMonDev;
    if (GetKeyValue(httpbuf, "CurrentMonDev", &CurrentMonDev, urlEncoded))
    {
        mysettings.currentMonitoringDevice = (CurrentMonitorDevice)CurrentMonDev;
    }

    if (mysettings.currentMonitoringEnabled == false)
    {
        // Switch off current monitor, clear out the values
        memset(&currentMonitor, 0, sizeof(currentmonitoring_struct));
        currentMonitor.validReadings = false;
        mysettings.currentMonitoringDevice = CurrentMonitorDevice::DIYBMS_CURRENT_MON;
        mysettings.currentMonitoringModBusAddress = 90;
    }

    saveConfiguration();

    return SendSuccess(req);
}
esp_err_t post_saverules_json_handler(httpd_req_t *req, bool urlEncoded)
{
    char textBuffer[32];

    // relaytype
    for (int i = 0; i < RELAY_TOTAL; i++)
    {
        String name = "relaytype";
        name = name + (i + 1);

        if (GetTextFromKeyValue(httpbuf, name.c_str(), textBuffer, sizeof(textBuffer), urlEncoded))
        {

            // Default
            RelayType oldValue = mysettings.relaytype[i];
            if (strcmp(textBuffer, "Pulse") == 0)
            {
                mysettings.relaytype[i] = RelayType::RELAY_PULSE;
            }
            else
            {
                mysettings.relaytype[i] = RelayType::RELAY_STANDARD;
            }

            if (oldValue != mysettings.relaytype[i])
            {
                // The type of relay has changed - we probably need to reset something here
                ESP_LOGI(TAG, "Type of relay has changed");
                previousRelayState[i] = RelayState::RELAY_X;
            }
        }
    }

    // Relay default
    for (int i = 0; i < RELAY_TOTAL; i++)
    {
        String name = "defaultrelay";
        name = name + (i + 1);
        if (GetTextFromKeyValue(httpbuf, name.c_str(), textBuffer, sizeof(textBuffer), urlEncoded))
        {
            // Default
            mysettings.rulerelaydefault[i] = RelayState::RELAY_OFF;
            if (strcmp(textBuffer, "On") == 0)
            {
                mysettings.rulerelaydefault[i] = RelayState::RELAY_ON;
            }
        }
    }

    for (int rule = 0; rule < RELAY_RULES; rule++)
    {

        // TODO: This STRING doesnt work properly if its on a single line!
        String name = "rule";
        name = name + (rule);
        name = name + "value";

        if (GetKeyValue(httpbuf, name.c_str(), &mysettings.rulevalue[rule], urlEncoded))
        {
        }

        // TODO: This STRING doesnt work properly if its on a single line!
        String hname = "rule";
        hname = hname + (rule);
        hname = hname + "hysteresis";
        if (GetKeyValue(httpbuf, name.c_str(), &mysettings.rulehysteresis[rule], urlEncoded))
        {
        }

        // Rule/relay processing
        for (int i = 0; i < RELAY_TOTAL; i++)
        {
            // TODO: This STRING doesnt work properly if its on a single line!
            String name = "rule";
            name = name + (rule);
            name = name + "relay";
            name = name + (i + 1);

            if (GetTextFromKeyValue(httpbuf, name.c_str(), textBuffer, sizeof(textBuffer), urlEncoded))
            {
                mysettings.rulerelaystate[rule][i] = strcmp(textBuffer, "X") == 0 ? RELAY_X : strcmp(textBuffer, "On") == 0 ? RelayState::RELAY_ON
                                                                                                                            : RelayState::RELAY_OFF;
            }
        }

        // Reset state of rules after updating the new values
        for (int8_t r = 0; r < RELAY_RULES; r++)
        {
            rules.rule_outcome[r] = false;
        }
    }

    saveConfiguration();

    return SendSuccess(req);
}

esp_err_t save_data_handler(httpd_req_t *req)
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

    const char *uri_array[] = {
        "savebankconfig", "saventp", "saveglobalsetting",
        "savemqtt", "saveinfluxdb", "saveconfigtofile", "wificonfigtofile", "savesetting",
        "restartcontroller", "saverules", "savedisplaysetting", "savestorage",
        "resetcounters", "sdmount", "sdunmount", "enableavrprog",
        "disableavrprog", "avrprog", "savers485settings", "savecurrentmon",
        "savecmbasic", "savecmadvanced", "savecmrelay", "savevictron"};

    esp_err_t (*func_ptr[])(httpd_req_t * req, bool urlEncoded) = {
        post_savebankconfig_json_handler, post_saventp_json_handler, post_saveglobalsetting_json_handler,
        post_savemqtt_json_handler, post_saveinfluxdbsetting_json_handler,
        post_saveconfigurationtosdcard_json_handler, post_savewificonfigtosdcard_json_handler,
        post_savesetting_json_handler, post_restartcontroller_json_handler, post_saverules_json_handler,
        post_savedisplaysetting_json_handler, post_savestorage_json_handler, post_resetcounters_json_handler,
        post_sdmount_json_handler, post_sdunmount_json_handler, post_enableavrprog_json_handler,
        post_disableavrprog_json_handler, post_avrprog_json_handler, post_savers485settings_json_handler,
        post_savecurrentmon_json_handler, post_savecmbasic_json_handler, post_savecmadvanced_json_handler,
        post_savecmrelay_json_handler, post_savevictron_json_handler};

    // Sanity check arrays are the same size
    ESP_ERROR_CHECK(sizeof(func_ptr) == sizeof(uri_array) ? ESP_OK : ESP_FAIL);

    // TODO: Improve the string comparision here to avoid any potential
    //       security/buffer overflows
    char name[32];
    memset(&name, 0, sizeof(name));
    //6= skip over /post/ characters
    strncpy(name, &(req->uri[6]), sizeof(name) - 1);

    for (size_t i = 0; i < sizeof(uri_array) / sizeof(unsigned int); i++)
    {
        if (strncmp(name, uri_array[i], strlen(uri_array[i])) == 0)
        {
            // Found it
            ESP_LOGI(TAG, "API post: %s", name);
            return func_ptr[i](req,urlEncoded);
        }
    }

    ESP_LOGE(TAG, "No API post match: %s", name);

    return httpd_resp_send_500(req);
}