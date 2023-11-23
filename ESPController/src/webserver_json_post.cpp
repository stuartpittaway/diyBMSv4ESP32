#define USE_ESP_IDF_LOG 1
static constexpr const char *const TAG = "diybms-webpost";

#include "webserver.h"
#include "webserver_json_post.h"
#include "webserver_helper_funcs.h"
#include <esp_netif.h>

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

    configureSNTP(mysettings.timeZone * 3600 + mysettings.minutesTimeZone * 60, mysettings.daylight ? 3600 : 0, mysettings.ntpServer);

    return SendSuccess(req);
}

esp_err_t post_savemqtt_json_handler(httpd_req_t *req, bool urlEncoded)
{
    // Default to off
    mysettings.mqtt_enabled = false;
    mysettings.mqtt_basic_cell_reporting = false;

    // Username and password are optional and may not be HTTP posted from web browser
    memset(mysettings.mqtt_username, 0, sizeof(mysettings.mqtt_username));
    memset(mysettings.mqtt_password, 0, sizeof(mysettings.mqtt_password));

    GetKeyValue(httpbuf, "mqttEnabled", &mysettings.mqtt_enabled, urlEncoded);

    GetKeyValue(httpbuf, "mqttBasicReporting", &mysettings.mqtt_basic_cell_reporting, urlEncoded);

    GetTextFromKeyValue(httpbuf, "mqttTopic", mysettings.mqtt_topic, sizeof(mysettings.mqtt_topic), urlEncoded);

    GetTextFromKeyValue(httpbuf, "mqttUri", mysettings.mqtt_uri, sizeof(mysettings.mqtt_uri), urlEncoded);

    GetTextFromKeyValue(httpbuf, "mqttUsername", mysettings.mqtt_username, sizeof(mysettings.mqtt_username), urlEncoded);

    GetTextFromKeyValue(httpbuf, "mqttPassword", mysettings.mqtt_password, sizeof(mysettings.mqtt_password), urlEncoded);

    saveConfiguration();

    // As the settings have changed, force stop of MQTT.
    // At a later point in time, the loop() will reconnect MQTT client
    stopMqtt();

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

// Saves all the BMS controller settings to a JSON file in FLASH
esp_err_t post_saveconfigurationtoflash_json_handler(httpd_req_t *req, bool urlEncoded)
{
    DynamicJsonDocument doc(5000);
    GenerateSettingsJSONDocument(&doc, &mysettings);

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

    // LittleFS only supports short filenames
    char filename[32];
    snprintf(filename, sizeof(filename), "/cfg_%04u%02u%02u_%02u%02u%02u.json", timeinfo.tm_year, timeinfo.tm_mon, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);

    // Get the file
    ESP_LOGI(TAG, "Generating LittleFS file %s", filename);

    // SD card not installed, so write to LITTLEFS instead (internal flash)
    File file = LittleFS.open(filename, "w");
    serializeJson(doc, file);
    file.close();

    return SendSuccess(req);
}

esp_err_t post_savewificonfigtosdcard_json_handler(httpd_req_t *req, bool)
{
    if (SaveWIFIJson(&_wificonfig))
    {
        return SendSuccess(req);
    }

    return SendFailure(req);
}

esp_err_t post_savesetting_json_handler(httpd_req_t *req, bool urlEncoded)
{
    uint32_t tempVariable;

    if (GetKeyValue(httpbuf, "m", &tempVariable, urlEncoded))
    {
        auto m = (uint8_t)tempVariable;

        if (m > maximum_controller_cell_modules)
        {
            // request->send(500, "text/plain", "Wrong parameters");
            ESP_LOGE(TAG, "Invalid module %u", m);
            return httpd_resp_send_500(req);
        }
        else
        {

            if (cmi[m].ChangesProhibited)
            {
                return SendFailure(req);
            }

            uint8_t BypassOverTempShutdown = 0xFF;
            uint16_t BypassThresholdmV = 0xFFFF;
            float Calibration = 0xFFFF;

            if (GetKeyValue(httpbuf, "BypassOverTempShutdown", &BypassOverTempShutdown, urlEncoded))
            {
                if (GetKeyValue(httpbuf, "BypassThresholdmV", &BypassThresholdmV, urlEncoded))
                {
                    if (GetKeyValue(httpbuf, "Calib", &Calibration, urlEncoded))
                    {
                        if (prg.sendSaveSetting(m, BypassThresholdmV, BypassOverTempShutdown, Calibration))
                        {

                            if (cmi[m].BoardVersionNumber == 490)
                            {
                                int16_t FanSwitchOnT = 30;
                                uint16_t RelayMinV = 1000;
                                uint16_t RelayRangemV = 5;
                                uint16_t RunAwayMinmV = 4000;
                                uint16_t RunAwayDiffmV = 100;

                                GetKeyValue(httpbuf, "FanSwitchOnT", &FanSwitchOnT, urlEncoded);
                                GetKeyValue(httpbuf, "RelayMinV", &RelayMinV, urlEncoded);
                                GetKeyValue(httpbuf, "RelayRange", &RelayRangemV, urlEncoded);

                                GetKeyValue(httpbuf, "RunAwayMinmV", &RunAwayMinmV, urlEncoded);
                                GetKeyValue(httpbuf, "RunAwayDiffmV", &RunAwayDiffmV, urlEncoded);

                                prg.sendSaveAdditionalSetting(m, FanSwitchOnT, RelayMinV, RelayRangemV, RunAwayMinmV, RunAwayDiffmV);
                            }

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

esp_err_t post_visibletiles_json_handler(httpd_req_t *req, bool urlEncoded)
{
    char keyBuffer[16];

    for (int i = 0; i < sizeof(mysettings.tileconfig) / sizeof(uint16_t); i++)
    {
        mysettings.tileconfig[i] = 0;
        snprintf(keyBuffer, sizeof(keyBuffer), "v%i", i);
        uint16_t temp;
        if (GetKeyValue(httpbuf, keyBuffer, &temp, urlEncoded))
        {
            ESP_LOGD(TAG, "%s=%u", keyBuffer, temp);
            mysettings.tileconfig[i] = temp;
        }
    }

    saveConfiguration();

    return SendSuccess(req);
}

esp_err_t post_savedisplaysetting_json_handler(httpd_req_t *req, bool urlEncoded)
{
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

esp_err_t post_resetcounters_json_handler(httpd_req_t *req, bool)
{
    // Ask modules to reset bad packet counters
    // If this fails, queue could be full so return error
    if (prg.sendBadPacketCounterReset() && prg.sendResetBalanceCurrentCounter())
    {
        canbus_messages_failed_sent = 0;
        canbus_messages_received = 0;
        canbus_messages_sent = 0;
        canbus_messages_received_error = 0;

        for (auto i = 0; i < maximum_controller_cell_modules; i++)
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

esp_err_t post_sdmount_json_handler(httpd_req_t *req, bool)
{
    if (_avrsettings.programmingModeEnabled)
    {
        return SendFailure(req);
    }

    card_action = CardAction::Mount;
    // mountSDCard();

    return SendSuccess(req);
}
esp_err_t post_sdunmount_json_handler(httpd_req_t *req, bool)
{
    if (_avrsettings.programmingModeEnabled)
    {
        return SendFailure(req);
    }

    // Raise a flag to indicate the action we want, the actual SD card mount/unmount
    // takes place in loop()
    // unmountSDCard();
    card_action = CardAction::Unmount;

    return SendSuccess(req);
}

esp_err_t post_enableavrprog_json_handler(httpd_req_t *req, bool)
{
    // unmountSDCard();

    _avrsettings.programmingModeEnabled = true;

    return SendSuccess(req);
}
esp_err_t post_disableavrprog_json_handler(httpd_req_t *req, bool)
{
    _avrsettings.programmingModeEnabled = false;

    // Try and remount the SD card
    // mountSDCard();

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

esp_err_t post_savechargeconfig_json_handler(httpd_req_t *req, bool urlEncoded)
{
    uint8_t temp;

    // If a user updates the charge config, reset the charging mode as well
    rules.setChargingMode(ChargingMode::standard);

    if (GetKeyValue(httpbuf, "canbusprotocol", &temp, urlEncoded))
    {
        mysettings.canbusprotocol = (CanBusProtocolEmulation)temp;
    }
    else
    {
        // Field not found/invalid, so disable
        mysettings.canbusprotocol = CanBusProtocolEmulation::CANBUS_DISABLED;
        mysettings.canbusinverter = CanBusInverter::INVERTER_GENERIC;
        mysettings.canbusbaud = 500;
    }

    // Default value
    mysettings.canbusinverter = CanBusInverter::INVERTER_GENERIC;
    if (GetKeyValue(httpbuf, "canbusinverter", &temp, urlEncoded))
    {
        mysettings.canbusinverter = (CanBusInverter)temp;
    }

    GetKeyValue(httpbuf, "canbusbaud", &mysettings.canbusbaud, urlEncoded);

    GetKeyValue(httpbuf, "nominalbatcap", &mysettings.nominalbatcap, urlEncoded);
    GetKeyValue(httpbuf, "cellminmv", &mysettings.cellminmv, urlEncoded);
    GetKeyValue(httpbuf, "cellmaxmv", &mysettings.cellmaxmv, urlEncoded);
    GetKeyValue(httpbuf, "kneemv", &mysettings.kneemv, urlEncoded);
    GetKeyValue(httpbuf, "cellmaxspikemv", &mysettings.cellmaxspikemv, urlEncoded);

    float temp_float;

    if (GetKeyValue(httpbuf, "cur_val1", &temp_float, urlEncoded))
    {
        mysettings.current_value1 = (uint16_t)(10 * temp_float);
    }

    if (GetKeyValue(httpbuf, "cur_val2", &temp_float, urlEncoded))
    {
        mysettings.current_value2 = (uint16_t)(10 * temp_float);
    }

    if (GetKeyValue(httpbuf, "sensitivity", &temp_float, urlEncoded))
    {
        mysettings.sensitivity = (int16_t)(10 * temp_float);
    }
    if (GetKeyValue(httpbuf, "chargevolt", &temp_float, urlEncoded))
    {
        mysettings.chargevolt = (uint16_t)(10 * temp_float);
    }
    if (GetKeyValue(httpbuf, "chargecurrent", &temp_float, urlEncoded))
    {
        mysettings.chargecurrent = (uint16_t)(10 * temp_float);
    }
    if (GetKeyValue(httpbuf, "dischargecurrent", &temp_float, urlEncoded))
    {
        mysettings.dischargecurrent = (uint16_t)(10 * temp_float);
    }
    if (GetKeyValue(httpbuf, "dischargevolt", &temp_float, urlEncoded))
    {
        mysettings.dischargevolt = (uint16_t)(10 * temp_float);
    }
    mysettings.stopchargebalance = false;
    GetKeyValue(httpbuf, "stopchargebalance", &mysettings.stopchargebalance, urlEncoded);

    mysettings.socoverride = false;
    GetKeyValue(httpbuf, "socoverride", &mysettings.socoverride, urlEncoded);

    mysettings.socforcelow = false;
    GetKeyValue(httpbuf, "socforcelow", &mysettings.socforcelow, urlEncoded);

    mysettings.dynamiccharge = false;
    GetKeyValue(httpbuf, "dynamiccharge", &mysettings.dynamiccharge, urlEncoded);

    mysettings.preventcharging = false;
    GetKeyValue(httpbuf, "preventcharging", &mysettings.preventcharging, urlEncoded);

    mysettings.preventdischarge = false;
    GetKeyValue(httpbuf, "preventdischarge", &mysettings.preventdischarge, urlEncoded);

    GetKeyValue(httpbuf, "chargetemplow", &mysettings.chargetemplow, urlEncoded);
    GetKeyValue(httpbuf, "chargetemphigh", &mysettings.chargetemphigh, urlEncoded);
    GetKeyValue(httpbuf, "dischargetemplow", &mysettings.dischargetemplow, urlEncoded);
    GetKeyValue(httpbuf, "dischargetemphigh", &mysettings.dischargetemphigh, urlEncoded);

    GetKeyValue(httpbuf, "absorptimer", &mysettings.absorptiontimer, urlEncoded);

    if (GetKeyValue(httpbuf, "floatvolt", &temp_float, urlEncoded))
    {
        mysettings.floatvoltage = (uint16_t)(10 * temp_float);
    }
    GetKeyValue(httpbuf, "floattimer", &mysettings.floatvoltagetimer, urlEncoded);
    GetKeyValue(httpbuf, "socresume", &mysettings.stateofchargeresumevalue, urlEncoded);

    if (mysettings.canbusprotocol == CanBusProtocolEmulation::CANBUS_DISABLED)
    {
        // Reset CAN counters if its disabled.
        canbus_messages_received = 0;
        canbus_messages_received_error = 0;
        canbus_messages_sent = 0;
        canbus_messages_failed_sent = 0;
    }

    // Default GENERIC inverter for VICTRON integration
    if (mysettings.canbusprotocol == CanBusProtocolEmulation::CANBUS_VICTRON)
    {
        mysettings.canbusinverter = CanBusInverter::INVERTER_GENERIC;
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

    if (mysettings.currentMonitoringEnabled)
    {
        if (mysettings.currentMonitoringDevice == CurrentMonitorDevice::DIYBMS_CURRENT_MON_MODBUS)
        {
            CurrentMonitorSetRelaySettingsExternal(newvalues);
        }
        if (mysettings.currentMonitoringDevice == CurrentMonitorDevice::DIYBMS_CURRENT_MON_INTERNAL)
        {
            CurrentMonitorSetRelaySettingsInternal(newvalues);
        }
    }

    return SendSuccess(req);
}

/// @brief Generates new home assistant API key and stored into flash
/// @param req 
/// @param urlEncoded 
/// @return 
esp_err_t post_homeassistant_apikey_json_handler(httpd_req_t *req, bool urlEncoded)
{
    char buffer[32];

    // Compare existing key to stored value, if they match allow generation of new key
    if (GetTextFromKeyValue(httpbuf, "haAPI", buffer, sizeof(buffer), urlEncoded))
    {
        if (strncmp(mysettings.homeassist_apikey, buffer, strlen(mysettings.homeassist_apikey)) != 0)
        {
            ESP_LOGE(TAG, "Incorrect ApiKey in form variable %s", buffer);
            return SendFailure(req);
        }

        memset(&mysettings.homeassist_apikey, 0, sizeof(mysettings.homeassist_apikey));
        randomCharacters(mysettings.homeassist_apikey, sizeof(mysettings.homeassist_apikey) - 1);
        saveConfiguration();

        ESP_LOGI(TAG, "new ha apikey=%s", mysettings.homeassist_apikey);

        return SendSuccess(req);
    }

    return SendFailure(req);
}

esp_err_t post_savenetconfig_json_handler(httpd_req_t *req, bool urlEncoded)
{
    char buffer[32];

    uint32_t new_ip = 0;
    uint32_t new_netmask = 0;
    uint32_t new_gw = 0;
    uint32_t new_dns1 = 0;
    uint32_t new_dns2 = 0;

    ip4_addr_t ipadd;

    if (GetTextFromKeyValue(httpbuf, "new_ip", buffer, sizeof(buffer), urlEncoded))
    {
        if (ip4addr_aton(buffer, &ipadd))
        {
            new_ip = ipadd.addr;
        }
    }
    if (GetTextFromKeyValue(httpbuf, "new_netmask", buffer, sizeof(buffer), urlEncoded))
    {
        if (ip4addr_aton(buffer, &ipadd))
        {
            new_netmask = ipadd.addr;
        }
    }
    if (GetTextFromKeyValue(httpbuf, "new_gw", buffer, sizeof(buffer), urlEncoded))
    {
        if (ip4addr_aton(buffer, &ipadd))
        {
            new_gw = ipadd.addr;
        }
    }
    if (GetTextFromKeyValue(httpbuf, "new_dns1", buffer, sizeof(buffer), urlEncoded))
    {
        if (ip4addr_aton(buffer, &ipadd))
        {
            new_dns1 = ipadd.addr;
        }
    }
    if (GetTextFromKeyValue(httpbuf, "new_dns2", buffer, sizeof(buffer), urlEncoded))
    {
        if (ip4addr_aton(buffer, &ipadd))
        {
            new_dns2 = ipadd.addr;
        }
    }

    if (new_ip == 0)
    {
        // Default back to DHCP, clear all the manual settings
        _wificonfig.manualConfig = 0;
        _wificonfig.wifi_ip = 0;
        _wificonfig.wifi_netmask = 0;
        _wificonfig.wifi_gateway = 0;
        _wificonfig.wifi_dns1 = 0;
        _wificonfig.wifi_dns2 = 0;
    }
    else
    {
        // Basic validation for invalid address details
        if ((new_netmask == 0) || (new_gw == 0) || (new_dns1 == 0))
        {
            ESP_LOGE(TAG, "Invalid manual network values - rejected");
            return SendFailure(req);
        }

        _wificonfig.manualConfig = 1;
        _wificonfig.wifi_ip = new_ip;
        _wificonfig.wifi_netmask = new_netmask;
        _wificonfig.wifi_gateway = new_gw;
        _wificonfig.wifi_dns1 = new_dns1;
        _wificonfig.wifi_dns2 = new_dns2;
    }

    // Save WIFI config
    SaveWIFI(&_wificonfig);

    // Attempt to save to SD card - but this may not be installed
    SaveWIFIJson(&_wificonfig);

    return SendSuccess(req);
}

esp_err_t post_setsoc_json_handler(httpd_req_t *req, bool urlEncoded)
{
    float new_soc = 0;
    if (GetKeyValue(httpbuf, "setsoc", &new_soc, urlEncoded))
    {
        if (CurrentMonitorSetSOC(new_soc))
        {
            return SendSuccess(req);
        }
    }
    return SendFailure(req);
}

esp_err_t post_resetdailyahcount_json_handler(httpd_req_t *req, bool)
{
    if (CurrentMonitorResetDailyAmpHourCounters())
    {
        return SendSuccess(req);
    }
    return SendFailure(req);
}

esp_err_t post_savecmbasic_json_handler(httpd_req_t *req, bool urlEncoded)
{
    uint16_t shuntmaxcur = 0;
    if (GetKeyValue(httpbuf, "shuntmaxcur", &shuntmaxcur, urlEncoded))
    {
        uint16_t shuntmv = 0;
        if (GetKeyValue(httpbuf, "shuntmv", &shuntmv, urlEncoded))
        {
            uint16_t batterycapacity = 0;
            if (GetKeyValue(httpbuf, "cmbatterycapacity", &batterycapacity, urlEncoded))
            {
                float fullchargevolt = 0;
                if (GetKeyValue(httpbuf, "cmfullchargevolt", &fullchargevolt, urlEncoded))
                {
                    float tailcurrent = 0;
                    if (GetKeyValue(httpbuf, "cmtailcurrent", &tailcurrent, urlEncoded))
                    {
                        float chargeefficiency = 0;
                        if (GetKeyValue(httpbuf, "cmchargeefficiency", &chargeefficiency, urlEncoded))
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

    if (!_avrsettings.programmingModeEnabled)
    {
        httpd_resp_set_type(req, "application/json");
        setNoStoreCacheControl(req);

        doc["message"] = "Failed: Programming mode not enabled";
        bufferused += serializeJson(doc, httpbuf, BUFSIZE);

        return httpd_resp_send(req, httpbuf, bufferused);
    }

    auto manifestfilename = String("/avr/manifest.json");

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

            _avrsettings.efuse = (uint8_t)strtoul(x["efuse"].as<String>().c_str(), nullptr, 16);
            _avrsettings.hfuse = (uint8_t)strtoul(x["hfuse"].as<String>().c_str(), nullptr, 16);
            _avrsettings.lfuse = (uint8_t)strtoul(x["lfuse"].as<String>().c_str(), nullptr, 16);
            _avrsettings.mcu = (uint32_t)strtoul(x["mcu"].as<String>().c_str(), nullptr, 16);

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
        mysettings.currentMonitoringDevice = CurrentMonitorDevice::DIYBMS_CURRENT_MON_MODBUS;
        mysettings.currentMonitoringModBusAddress = 90;
    }

    saveConfiguration();

    return SendSuccess(req);
}
esp_err_t post_saverules_json_handler(httpd_req_t *req, bool urlEncoded)
{
    char textBuffer[32];

    char keyBuffer[32];

    // relaytype
    for (int i = 0; i < RELAY_TOTAL; i++)
    {
        snprintf(keyBuffer, sizeof(keyBuffer), "relaytype%i", (i + 1));

        if (GetTextFromKeyValue(httpbuf, keyBuffer, textBuffer, sizeof(textBuffer), urlEncoded))
        {
            ESP_LOGD(TAG, "%s=%s", keyBuffer, textBuffer);

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
        snprintf(keyBuffer, sizeof(keyBuffer), "defaultrelay%i", (i + 1));

        if (GetTextFromKeyValue(httpbuf, keyBuffer, textBuffer, sizeof(textBuffer), urlEncoded))
        {
            ESP_LOGD(TAG, "%s=%s", keyBuffer, textBuffer);

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
        snprintf(keyBuffer, sizeof(keyBuffer), "rule%ivalue", rule);

        int32_t tempint32;
        if (GetKeyValue(httpbuf, keyBuffer, &tempint32, urlEncoded))
        {
            ESP_LOGD(TAG, "%s=%u", keyBuffer, tempint32);

            mysettings.rulevalue[rule] = tempint32;
        }

        snprintf(keyBuffer, sizeof(keyBuffer), "rule%ihyst", rule);
        if (GetKeyValue(httpbuf, keyBuffer, &tempint32, urlEncoded))
        {
            ESP_LOGD(TAG, "%s=%u", keyBuffer, tempint32);

            mysettings.rulehysteresis[rule] = tempint32;
        }

        // Rule/relay processing
        for (int i = 0; i < RELAY_TOTAL; i++)
        {
            snprintf(keyBuffer, sizeof(keyBuffer), "rule%irelay%i", rule, (i + 1));

            if (GetTextFromKeyValue(httpbuf, keyBuffer, textBuffer, sizeof(textBuffer), urlEncoded))
            {
                ESP_LOGD(TAG, "%s=%s", keyBuffer, textBuffer);
                mysettings.rulerelaystate[rule][i] = strcmp(textBuffer, "X") == 0 ? RELAY_X : strcmp(textBuffer, "On") == 0 ? RelayState::RELAY_ON
                                                                                                                            : RelayState::RELAY_OFF;
            }
        }

        // Reset state of rules after updating the new values
        rules.resetAllRules();
    }

    saveConfiguration();

    return SendSuccess(req);
}

esp_err_t post_restoreconfig_json_handler(httpd_req_t *req, bool urlEncoded)
{
    bool success = false;

    if (_avrsettings.programmingModeEnabled)
    {
        return SendFailure(req);
    }

    char filename[128];
    // Prepend "/"
    strcpy(filename, "/");

    if (!GetTextFromKeyValue(httpbuf, "filename", &filename[1], sizeof(filename) - 1, urlEncoded))
    {
        ESP_LOGE(TAG, "Unable to decode filename");
        return SendFailure(req);
    }

    uint16_t flashram = 0;
    if (GetKeyValue(httpbuf, "flashram", &flashram, urlEncoded))
    {
    }

    if (flashram == 0)
    {
        if (!_sd_card_installed)
        {
            return SendFailure(req);
        }

        if (hal.GetVSPIMutex())
        {
            if (SD.exists(filename))
            {
                ESP_LOGI(TAG, "Restore SD config from %s", filename);

                // Needs to be large enough to de-serialize the JSON file
                DynamicJsonDocument doc(5000);

                File file = SD.open(filename, "r");

                // Deserialize the JSON document
                DeserializationError error = deserializeJson(doc, file);
                if (error)
                {
                    ESP_LOGE(TAG, "Deserialization Error");
                }
                else
                {
                    // Restore the config...
                    diybms_eeprom_settings myset;

                    JSONToSettings(doc, &myset);
                    // Repair any bad values
                    ValidateConfiguration(&myset);

                    // Copy the new settings over top of old
                    memcpy(&mysettings, &myset, sizeof(mysettings));

                    saveConfiguration();

                    success = true;
                }

                file.close();
            }
            else
            {
                ESP_LOGE(TAG, "File does not exist %s", filename);
            }

            hal.ReleaseVSPIMutex();
        }
    }

    if (flashram == 1)
    {
        if (LittleFS.exists(filename))
        {
            ESP_LOGI(TAG, "Restore LittleFS config from %s", filename);

            // Needs to be large enough to de-serialize the JSON file
            DynamicJsonDocument doc(5500);

            File file = LittleFS.open(filename, "r");

            // Deserialize the JSON document
            DeserializationError error = deserializeJson(doc, file);
            if (error)
            {
                ESP_LOGE(TAG, "Deserialization Error");
            }
            else
            {
                // Restore the config...
                diybms_eeprom_settings myset;
                JSONToSettings(doc, &myset);
                // Free up the memory
                doc.clear();
                // Repair any bad values
                ValidateConfiguration(&myset);

                // Copy the new settings over top of old
                memcpy(&mysettings, &myset, sizeof(mysettings));

                saveConfiguration();

                success = true;
            }

            file.close();
        }
        else
        {
            ESP_LOGE(TAG, "File does not exist %s", filename);
        }
    }

    if (success)
    {
        return SendSuccess(req);
    }

    return SendFailure(req);
}

esp_err_t save_data_handler(httpd_req_t *req)
{
    // ESP_LOGI(TAG, "JSON call");

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

    std::array<std::string, 30> uri_array = {
        "savebankconfig", "saventp", "saveglobalsetting",
        "savemqtt", "saveinfluxdb",
        "saveconfigtofile", "wificonfigtofile",
        "savesetting", "restartcontroller", "saverules",
        "savedisplaysetting", "savestorage", "resetcounters",
        "sdmount", "sdunmount", "enableavrprog",
        "disableavrprog", "avrprog", "savers485settings",
        "savecurrentmon", "savecmbasic", "savecmadvanced",
        "savecmrelay", "restoreconfig", "savechargeconfig",
        "visibletiles", "dailyahreset", "setsoc",
        "savenetconfig", "newhaapikey"};

    std::array<std::function<esp_err_t(httpd_req_t * req, bool urlEncoded)>, 30> func_ptr = {
        post_savebankconfig_json_handler, post_saventp_json_handler, post_saveglobalsetting_json_handler,
        post_savemqtt_json_handler, post_saveinfluxdbsetting_json_handler,
        post_saveconfigurationtoflash_json_handler, post_savewificonfigtosdcard_json_handler,
        post_savesetting_json_handler, post_restartcontroller_json_handler, post_saverules_json_handler,
        post_savedisplaysetting_json_handler, post_savestorage_json_handler, post_resetcounters_json_handler,
        post_sdmount_json_handler, post_sdunmount_json_handler, post_enableavrprog_json_handler,
        post_disableavrprog_json_handler, post_avrprog_json_handler, post_savers485settings_json_handler,
        post_savecurrentmon_json_handler, post_savecmbasic_json_handler, post_savecmadvanced_json_handler,
        post_savecmrelay_json_handler, post_restoreconfig_json_handler, post_savechargeconfig_json_handler,
        post_visibletiles_json_handler, post_resetdailyahcount_json_handler, post_setsoc_json_handler,
        post_savenetconfig_json_handler, post_homeassistant_apikey_json_handler};

    auto name = std::string(req->uri);

    if (name.rfind("/post/", 0) == 0)
    {
        // skip over first 6 characters "/post/" characters
        name = name.substr(6);
    }

    for (size_t i = 0; i < uri_array.size(); i++)
    {
        if (name.compare(uri_array.at(i)) == 0)
        {
            // Found it
            ESP_LOGI(TAG, "API post: %s", name.c_str());
            return func_ptr.at(i)(req, urlEncoded);
        }
    }

    ESP_LOGE(TAG, "No API post match: %s", name.c_str());

    return httpd_resp_send_500(req);
}