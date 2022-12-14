#define USE_ESP_IDF_LOG 1
static constexpr const char *const TAG = "diybms-set";

#include "settings.h"

bool ValidateGetSetting(esp_err_t err, const char *key)
{
    switch (err)
    {
    case ESP_OK:
        ESP_LOGI(TAG, "Read key (%s)", key);
        return true;
        break;
    case ESP_ERR_NVS_NOT_FOUND:
        ESP_LOGW(TAG, "Key not initialized (%s)", key);
        break;
    default:
        ESP_LOGE(TAG, "Error (%s) reading (%s)", esp_err_to_name(err), key);
    }
    return false;
}

bool getString(nvs_handle_t handle, const char *key, char *out_value, size_t size)
{
    size_t length = size;
    return ValidateGetSetting(nvs_get_str(handle, key, out_value, &length), key);
}
bool getSetting(nvs_handle_t handle, const char *key, float *out_value)
{
    size_t required_size = sizeof(float);
    return ValidateGetSetting(nvs_get_blob(handle, key, (void *)out_value, &required_size), key);
}
bool getSetting(nvs_handle_t handle, const char *key, uint8_t *out_value)
{
    return ValidateGetSetting(nvs_get_u8(handle, key, out_value), key);
}
bool getSetting(nvs_handle_t handle, const char *key, int32_t *out_value)
{
    return ValidateGetSetting(nvs_get_i32(handle, key, out_value), key);
}
/*
bool getSetting(nvs_handle_t handle, const char *key, uint32_t *out_value)
{
    return ValidateGetSetting(nvs_get_u32(handle, key, out_value), key);
}
*/
bool getSetting(nvs_handle_t handle, const char *key, int8_t *out_value)
{
    return ValidateGetSetting(nvs_get_i8(handle, key, out_value), key);
}

bool getSettingBlob(nvs_handle_t handle, const char *key, void *out_value, size_t size)
{
    size_t stored_size = 0;
    esp_err_t err = nvs_get_blob(handle, key, NULL, &stored_size);
    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGW(TAG, "Key not initialized (%s)", key);
    }
    if (err != ESP_OK)
    {
        return false;
    }

    // Size is different
    if (stored_size != size)
    {
        ESP_LOGW(TAG, "Rejecting (%s) blob size changed, old=%d, new=%d", key, stored_size, size);
        return false;
    }

    size_t required_size = size;
    return ValidateGetSetting(nvs_get_blob(handle, key, out_value, &required_size), key);
}

bool getSetting(nvs_handle_t handle, const char *key, uint16_t *out_value)
{
    return ValidateGetSetting(nvs_get_u16(handle, key, out_value), key);
}
bool getSetting(nvs_handle_t handle, const char *key, bool *out_value)
{
    return ValidateGetSetting(nvs_get_u8(handle, key, (uint8_t *)out_value), key);
}

void InitializeNVS()
{
    // Initialize NVS
    ESP_LOGI(TAG, "nvs_flash_init");
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

void writeSetting(nvs_handle_t handle, const char *key, int8_t value)
{
    ESP_LOGD(TAG, "Writing (%s)=%i", key, value);
    ESP_ERROR_CHECK(nvs_set_i8(handle, key, value));
}
void writeSetting(nvs_handle_t handle, const char *key, uint8_t value)
{
    ESP_LOGD(TAG, "Writing (%s)=%u", key, value);
    ESP_ERROR_CHECK(nvs_set_u8(handle, key, value));
}
void writeSetting(nvs_handle_t handle, const char *key, int32_t value)
{
    ESP_LOGD(TAG, "Writing (%s)=%i", key, value);
    ESP_ERROR_CHECK(nvs_set_i32(handle, key, value));
}
void writeSetting(nvs_handle_t handle, const char *key, uint16_t value)
{
    ESP_LOGD(TAG, "Writing (%s)=%u", key, value);
    ESP_ERROR_CHECK(nvs_set_u16(handle, key, value));
}
void writeSetting(nvs_handle_t handle, const char *key, const char *value)
{
    ESP_LOGD(TAG, "Writing (%s)=%s", key, value);
    ESP_ERROR_CHECK(nvs_set_str(handle, key, value));
}
void writeSettingBlob(nvs_handle_t handle, const char *key, const void *value, size_t length)
{
    ESP_LOGD(TAG, "Writing (%s), length=%u", key, length);
    ESP_ERROR_CHECK(nvs_set_blob(handle, key, value, length));
}

void SaveConfiguration(diybms_eeprom_settings *settings)
{
    const char *partname = "diybms-ctrl";
    ESP_LOGI(TAG, "Write config");

    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(partname, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error %s opening NVS handle!", esp_err_to_name(err));
    }
    else
    {
        // Save settings
        writeSetting(nvs_handle, "totalBanks", settings->totalNumberOfBanks);
        writeSetting(nvs_handle, "totalSeriesMod", settings->totalNumberOfSeriesModules);
        writeSetting(nvs_handle, "baudRate", settings->baudRate);
        writeSetting(nvs_handle, "interpacketgap", settings->interpacketgap);
        writeSettingBlob(nvs_handle, "rulevalue", settings->rulevalue, sizeof(settings->rulevalue));
        writeSettingBlob(nvs_handle, "rulehysteresis", settings->rulehysteresis, sizeof(settings->rulehysteresis));
        writeSettingBlob(nvs_handle, "rulerelaystate", settings->rulerelaystate, sizeof(settings->rulerelaystate));
        writeSettingBlob(nvs_handle, "rulerelaydef", settings->rulerelaydefault, sizeof(settings->rulerelaydefault));
        writeSettingBlob(nvs_handle, "relaytype", settings->relaytype, sizeof(settings->relaytype));
      
        writeSetting(nvs_handle, "g_voltagehigh", settings->graph_voltagehigh);
        writeSetting(nvs_handle, "g_voltagelow", settings->graph_voltagelow);

        writeSetting(nvs_handle, "BypassOverTemp", settings->BypassOverTempShutdown);
        writeSetting(nvs_handle, "BypassThresmV", settings->BypassThresholdmV);
        writeSetting(nvs_handle, "TZ", settings->timeZone);
        writeSetting(nvs_handle, "minutesTZ", settings->minutesTimeZone);
        writeSetting(nvs_handle, "daylight", settings->daylight);

        writeSetting(nvs_handle, "logEnabled", (uint8_t)settings->loggingEnabled);
        writeSetting(nvs_handle, "logFreqSec", settings->loggingFrequencySeconds);

        writeSetting(nvs_handle, "curMonEnabled", (uint8_t)settings->currentMonitoringEnabled);
        writeSetting(nvs_handle, "curMonMBAddress", (uint8_t)settings->currentMonitoringModBusAddress);
        writeSetting(nvs_handle, "curMonDevice", (uint8_t)settings->currentMonitoringDevice);

        writeSetting(nvs_handle, "485baudrate", settings->rs485baudrate);
        writeSetting(nvs_handle, "485databits", (uint8_t)settings->rs485databits);
        writeSetting(nvs_handle, "485parity", (uint8_t)settings->rs485parity);
        writeSetting(nvs_handle, "485stopbits", (uint8_t)settings->rs485stopbits);

        writeSetting(nvs_handle, "canbusprotocol", (uint8_t)settings->canbusprotocol);
        writeSetting(nvs_handle, "nominalbatcap", settings->nominalbatcap);

        writeSetting(nvs_handle, "cha_volt", settings->chargevolt);
        writeSetting(nvs_handle, "cha_current", settings->chargecurrent);
        writeSetting(nvs_handle, "dis_current", settings->dischargecurrent);
        writeSetting(nvs_handle, "dis_volt", settings->dischargevolt);
        writeSetting(nvs_handle, "cellminmv", settings->cellminmv);
        writeSetting(nvs_handle, "cellmaxmv", settings->cellmaxmv);
        writeSetting(nvs_handle, "kneemv", settings->kneemv);
        writeSetting(nvs_handle, "cha_templow", settings->chargetemplow);
        writeSetting(nvs_handle, "cha_temphigh", settings->chargetemphigh);
        writeSetting(nvs_handle, "dis_templow", settings->dischargetemplow);
        writeSetting(nvs_handle, "dis_temphigh", settings->dischargetemphigh);
        writeSetting(nvs_handle, "stopchargebal", (uint8_t)settings->stopchargebalance);

        writeSetting(nvs_handle, "socoverride", (uint8_t)settings->socoverride);
        writeSetting(nvs_handle, "socforcelow", (uint8_t)settings->socforcelow);

        writeSetting(nvs_handle, "dynamiccharge", (uint8_t)settings->dynamiccharge);
        writeSetting(nvs_handle, "preventchar", (uint8_t)settings->preventcharging);
        writeSetting(nvs_handle, "preventdis", (uint8_t)settings->preventdischarge);

        writeSetting(nvs_handle, "mqttenable", (uint8_t)settings->mqtt_enabled);
        writeSetting(nvs_handle, "infenabled", (uint8_t)settings->influxdb_enabled);
        writeSetting(nvs_handle, "inflogFreq", settings->influxdb_loggingFreqSeconds);

        writeSettingBlob(nvs_handle, "tileconfig", settings->tileconfig, sizeof(settings->tileconfig));

        writeSetting(nvs_handle, "ntpServer", &settings->ntpServer[0]);
        writeSetting(nvs_handle, "language", &settings->language[0]);
        writeSetting(nvs_handle, "mqtt_uri", &settings->mqtt_uri[0]);
        writeSetting(nvs_handle, "mqtt_topic", &settings->mqtt_topic[0]);
        writeSetting(nvs_handle, "mqtt_usern", &settings->mqtt_username[0]);
        writeSetting(nvs_handle, "mqtt_pword", &settings->mqtt_password[0]);

        writeSetting(nvs_handle, "inf_serverurl", &settings->influxdb_serverurl[0]);
        writeSetting(nvs_handle, "inf_bucket", &settings->influxdb_databasebucket[0]);
        writeSetting(nvs_handle, "inf_apitoken", &settings->influxdb_apitoken[0]);
        writeSetting(nvs_handle, "inf_orgid", &settings->influxdb_orgid[0]);

        ESP_ERROR_CHECK(nvs_commit(nvs_handle));
        nvs_close(nvs_handle);
    }
}

void LoadConfiguration(diybms_eeprom_settings *settings)
{
    const char *partname = "diybms-ctrl";
    ESP_LOGI(TAG, "Load config");

    // Set all settings in the STRUCT to be the defaults
    DefaultConfiguration(settings);

    nvs_stats_t nvs_stats;
    esp_err_t err = nvs_get_stats(NULL, &nvs_stats);
    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "Count: UsedEntries = (%d), FreeEntries = (%d), AllEntries = (%d)", nvs_stats.used_entries, nvs_stats.free_entries, nvs_stats.total_entries);
    }

    nvs_handle_t nvs_handle;
    err = nvs_open(partname, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle", esp_err_to_name(err));
    }
    else
    {
        // Open
        getSetting(nvs_handle, "totalBanks", &settings->totalNumberOfBanks);
        getSetting(nvs_handle, "totalSeriesMod", &settings->totalNumberOfSeriesModules);
        getSetting(nvs_handle, "baudRate", &settings->baudRate);
        getSetting(nvs_handle, "interpacketgap", &settings->interpacketgap);

        getSettingBlob(nvs_handle, "rulevalue", &settings->rulevalue, sizeof(settings->rulevalue));
        getSettingBlob(nvs_handle, "rulehysteresis", &settings->rulehysteresis, sizeof(settings->rulehysteresis));

        getSettingBlob(nvs_handle, "rulerelaystate", &settings->rulerelaystate, sizeof(settings->rulerelaystate));
        getSettingBlob(nvs_handle, "rulerelaydef", &settings->rulerelaydefault, sizeof(settings->rulerelaydefault));
        getSettingBlob(nvs_handle, "relaytype", &settings->relaytype, sizeof(settings->relaytype));

        getSetting(nvs_handle, "g_voltagehigh", &settings->graph_voltagehigh);
        getSetting(nvs_handle, "g_voltagelow", &settings->graph_voltagelow);

        getSetting(nvs_handle, "BypassOverTemp", &settings->BypassOverTempShutdown);
        getSetting(nvs_handle, "BypassThresmV", &settings->BypassThresholdmV);

        getSetting(nvs_handle, "TZ", &settings->timeZone);
        getSetting(nvs_handle, "minutesTZ", &settings->minutesTimeZone);
        getSetting(nvs_handle, "daylight", &settings->daylight);

        getSetting(nvs_handle, "logEnabled", &settings->loggingEnabled);
        getSetting(nvs_handle, "logFreqSec", &settings->loggingFrequencySeconds);

        getSetting(nvs_handle, "curMonEnabled", &settings->currentMonitoringEnabled);
        getSetting(nvs_handle, "curMonMBAddress", &settings->currentMonitoringModBusAddress);
        getSetting(nvs_handle, "curMonDevice", (uint8_t *)&settings->currentMonitoringDevice);

        getSetting(nvs_handle, "485baudrate", &settings->rs485baudrate);
        getSetting(nvs_handle, "485databits", (uint8_t *)&settings->rs485databits);
        getSetting(nvs_handle, "485parity", (uint8_t *)&settings->rs485parity);
        getSetting(nvs_handle, "485stopbits", (uint8_t *)&settings->rs485stopbits);

        getSetting(nvs_handle, "canbusprotocol", (uint8_t *)&settings->canbusprotocol);
        getSetting(nvs_handle, "nominalbatcap", &settings->nominalbatcap);
        getSetting(nvs_handle, "cha_volt", &settings->chargevolt);
        getSetting(nvs_handle, "cha_current", &settings->chargecurrent);
        getSetting(nvs_handle, "dis_current", &settings->dischargecurrent);
        getSetting(nvs_handle, "dis_volt", &settings->dischargevolt);
        getSetting(nvs_handle, "cellminmv", &settings->cellminmv);
        getSetting(nvs_handle, "cellmaxmv", &settings->cellmaxmv);
        getSetting(nvs_handle, "kneemv", &settings->kneemv);
        getSetting(nvs_handle, "cha_templow", &settings->chargetemplow);
        getSetting(nvs_handle, "cha_temphigh", &settings->chargetemphigh);
        getSetting(nvs_handle, "dis_templow", &settings->dischargetemplow);
        getSetting(nvs_handle, "dis_temphigh", &settings->dischargetemphigh);
        getSetting(nvs_handle, "stopchargebal", &settings->stopchargebalance);

        getSetting(nvs_handle, "socoverride", &settings->socoverride);
        getSetting(nvs_handle, "socforcelow", &settings->socforcelow);

        getSetting(nvs_handle, "dynamiccharge", &settings->dynamiccharge);
        getSetting(nvs_handle, "preventchar", &settings->preventcharging);
        getSetting(nvs_handle, "preventdis", &settings->preventdischarge);

        getSetting(nvs_handle, "mqttenable", &settings->mqtt_enabled);
        getSetting(nvs_handle, "infenabled", &settings->influxdb_enabled);
        getSetting(nvs_handle, "inflogFreq", &settings->influxdb_loggingFreqSeconds);

        getSettingBlob(nvs_handle, "tileconfig", &settings->tileconfig, sizeof(settings->tileconfig));

        getString(nvs_handle, "ntpServer", &settings->ntpServer[0], sizeof(settings->ntpServer));
        getString(nvs_handle, "language", &settings->language[0], sizeof(settings->language));
        getString(nvs_handle, "mqtt_uri", &settings->mqtt_uri[0], sizeof(settings->mqtt_uri));
        getString(nvs_handle, "mqtt_topic", &settings->mqtt_topic[0], sizeof(settings->mqtt_topic));
        getString(nvs_handle, "mqtt_usern", &settings->mqtt_username[0], sizeof(settings->mqtt_username));
        getString(nvs_handle, "mqtt_pword", &settings->mqtt_password[0], sizeof(settings->mqtt_password));

        getString(nvs_handle, "inf_serverurl", &settings->influxdb_serverurl[0], sizeof(settings->influxdb_serverurl));
        getString(nvs_handle, "inf_bucket", &settings->influxdb_databasebucket[0], sizeof(settings->influxdb_databasebucket));
        getString(nvs_handle, "inf_apitoken", &settings->influxdb_apitoken[0], sizeof(settings->influxdb_apitoken));
        getString(nvs_handle, "inf_orgid", &settings->influxdb_orgid[0], sizeof(settings->influxdb_orgid));

        nvs_close(nvs_handle);
    }
}

void DefaultConfiguration(diybms_eeprom_settings *_myset)
{
    ESP_LOGD(TAG, "Apply default config");

    // Zero all the bytes
    memset(_myset, 0, sizeof(diybms_eeprom_settings));

    // Default to a single module
    _myset->totalNumberOfBanks = 1;
    _myset->totalNumberOfSeriesModules = 1;
    // Default serial port speed
    _myset->baudRate = COMMS_BAUD_RATE;
    _myset->BypassOverTempShutdown = 65;
    _myset->interpacketgap = 6000;
    // 4.10V bypass
    _myset->BypassThresholdmV = 4100;
    _myset->graph_voltagehigh = 4500;
    _myset->graph_voltagelow = 2750;

    // EEPROM settings are invalid so default configuration
    _myset->mqtt_enabled = false;

    _myset->canbusprotocol = CanBusProtocolEmulation::CANBUS_DISABLED;
    _myset->nominalbatcap = 280;
    _myset->chargevolt = 565;       // Scale 0.1
    _myset->chargecurrent = 650;    // Scale 0.1
    _myset->dischargecurrent = 650; // Scale 0.1
    _myset->dischargevolt = 488;    // Scale 0.1
    _myset->chargetemplow = 0;
    _myset->chargetemphigh = 50;
    _myset->dischargetemplow = -30;
    _myset->dischargetemphigh = 55;
    // Just outside the ranges of 56.0V and 49.6V
    _myset->cellminmv = 3050;
    _myset->cellmaxmv = 3450;
    _myset->kneemv = 3320;
    _myset->stopchargebalance = true;
    _myset->socoverride = false;
    _myset->socforcelow = false;
    _myset->dynamiccharge = true;
    _myset->preventcharging = false;
    _myset->preventdischarge = false;

    _myset->loggingEnabled = false;
    _myset->loggingFrequencySeconds = 15;

    _myset->currentMonitoringEnabled = false;
    _myset->currentMonitoringModBusAddress = 90;
    _myset->currentMonitoringDevice = CurrentMonitorDevice::DIYBMS_CURRENT_MON;

    _myset->rs485baudrate = 19200;
    _myset->rs485databits = uart_word_length_t::UART_DATA_8_BITS;
    _myset->rs485parity = uart_parity_t::UART_PARITY_DISABLE;
    _myset->rs485stopbits = uart_stop_bits_t::UART_STOP_BITS_1;

    _myset->currentMonitoringEnabled = false;

    strncpy(_myset->language, "en", sizeof(_myset->language));

    // Default to EMONPI default MQTT settings
    strncpy(_myset->mqtt_topic, "emon/diybms", sizeof(_myset->mqtt_topic));
    strncpy(_myset->mqtt_uri, "mqtt://192.168.0.26:1883", sizeof(_myset->mqtt_uri));
    strncpy(_myset->mqtt_username, "emonpi", sizeof(_myset->mqtt_username));
    strncpy(_myset->mqtt_password, "emonpimqtt2016", sizeof(_myset->mqtt_password));

    _myset->influxdb_enabled = false;
    strncpy(_myset->influxdb_serverurl, "http://192.168.0.49:8086/api/v2/write", sizeof(_myset->influxdb_serverurl));
    strncpy(_myset->influxdb_databasebucket, "bucketname", sizeof(_myset->influxdb_databasebucket));
    strncpy(_myset->influxdb_orgid, "organisation", sizeof(_myset->influxdb_orgid));
    _myset->influxdb_loggingFreqSeconds = 15;

    _myset->timeZone = 0;
    _myset->minutesTimeZone = 0;
    _myset->daylight = false;
    strncpy(_myset->ntpServer, "time.google.com", sizeof(_myset->ntpServer));

    for (size_t x = 0; x < RELAY_TOTAL; x++)
    {
        _myset->rulerelaydefault[x] = RELAY_OFF;
    }

    // Emergency stop
    _myset->rulevalue[Rule::EmergencyStop] = 0;
    // Internal BMS error (communication issues, fault readings from modules etc)
    _myset->rulevalue[Rule::BMSError] = 0;
    // Current monitoring maximum AMPS
    _myset->rulevalue[Rule::CurrentMonitorOverCurrentAmps] = 100;
    // Individual cell over voltage
    _myset->rulevalue[Rule::ModuleOverVoltage] = 4150;
    // Individual cell under voltage
    _myset->rulevalue[Rule::ModuleUnderVoltage] = 3000;
    // Individual cell over temperature (external probe)
    _myset->rulevalue[Rule::ModuleOverTemperatureExternal] = 55;
    // Pack over voltage (mV)
    _myset->rulevalue[Rule::ModuleUnderTemperatureExternal] = 5;
    // Pack under voltage (mV)
    _myset->rulevalue[Rule::BankOverVoltage] = 4200 * 8;
    // RULE_PackUnderVoltage
    _myset->rulevalue[Rule::BankUnderVoltage] = 3000 * 8;
    _myset->rulevalue[Rule::Timer1] = 60 * 8;  // 8am
    _myset->rulevalue[Rule::Timer2] = 60 * 17; // 5pm

    _myset->rulevalue[Rule::ModuleOverTemperatureInternal] = 60;
    _myset->rulevalue[Rule::ModuleUnderTemperatureInternal] = 5;

    _myset->rulevalue[Rule::CurrentMonitorOverVoltage] = 4200 * 8;
    _myset->rulevalue[Rule::CurrentMonitorUnderVoltage] = 3000 * 8;

    for (size_t i = 0; i < RELAY_RULES; i++)
    {
        _myset->rulehysteresis[i] = _myset->rulevalue[i];

        // Set all relays to don't care
        for (size_t x = 0; x < RELAY_TOTAL; x++)
        {
            _myset->rulerelaystate[i][x] = RELAY_X;
        }
    }

    for (size_t x = 0; x < RELAY_TOTAL; x++)
    {
        _myset->relaytype[x] = RELAY_STANDARD;
    }

    // Default which "tiles" are visible on the web gui
    // For the meaning, look at array "TILE_IDS" in pagecode.js
    _myset->tileconfig[0] = 49152;
    _myset->tileconfig[1] = 0;
    _myset->tileconfig[2] = 62209;
    _myset->tileconfig[3] = 0;
    _myset->tileconfig[4] = 0;
}

void SaveWIFI(wifi_eeprom_settings *wifi)
{
    const char *partname = "diybms-wifi";
    ESP_LOGI(TAG, "Save WIFI config");

    wifi_eeprom_settings x;
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(partname, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle", esp_err_to_name(err));
    }
    else
    {
        // Write values
        ESP_ERROR_CHECK(nvs_set_str(nvs_handle, "SSID", &wifi->wifi_ssid[0]));
        ESP_ERROR_CHECK(nvs_set_str(nvs_handle, "PASS", &wifi->wifi_passphrase[0]));
        nvs_close(nvs_handle);
    }
}

bool LoadWIFI(wifi_eeprom_settings *wifi)
{
    const char *partname = "diybms-wifi";
    ESP_LOGI(TAG, "Load WIFI config");

    bool result = false;
    wifi_eeprom_settings x;
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(partname, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle", esp_err_to_name(err));
    }
    else
    {
        if (getString(nvs_handle, "SSID", &x.wifi_ssid[0], sizeof(x.wifi_ssid)))
        {
            if (getString(nvs_handle, "PASS", &x.wifi_passphrase[0], sizeof(x.wifi_passphrase)))
            {
                // Only return success if both values are retrieved, don't corrupt
                // external copy of wifi settings otherwise.
                memcpy(wifi, &x, sizeof(x));
                result = true;
            }
        }
        nvs_close(nvs_handle);
    }

    return result;
}

// Validate configuration and force correction if needed.
void ValidateConfiguration(diybms_eeprom_settings *settings)
{
    diybms_eeprom_settings defaults;
    DefaultConfiguration(&defaults);

    // Check its not zero
    if (settings->influxdb_loggingFreqSeconds < 5)
    {
        settings->influxdb_loggingFreqSeconds = defaults.influxdb_loggingFreqSeconds;
    }

    if (settings->rs485baudrate < 300)
    {
        settings->rs485baudrate = defaults.rs485baudrate;
    }

    if (settings->baudRate < 300)
    {
        settings->baudRate = defaults.baudRate;
    }

    if (settings->graph_voltagehigh > 5000 || settings->graph_voltagehigh <2000 || settings->graph_voltagehigh<0)
    {
        settings->graph_voltagehigh = defaults.graph_voltagehigh;
    }

    if (settings->graph_voltagelow > settings->graph_voltagehigh || settings->graph_voltagelow<0)
    {
        settings->graph_voltagelow = 0;
    }
}

// Builds up a JSON document which mirrors the parameters inside "diybms_eeprom_settings"
void GenerateSettingsJSONDocument(DynamicJsonDocument *doc, diybms_eeprom_settings *settings)
{
    JsonObject root = doc->createNestedObject("diybms_settings");

    root["totalNumberOfBanks"] = settings->totalNumberOfBanks;
    root["totalNumberOfSeriesModules"] = settings->totalNumberOfSeriesModules;
    root["baudRate"] = settings->baudRate;
    root["interpacketgap"] = settings->interpacketgap;

    root["graph_voltagehigh"] = settings->graph_voltagehigh;
    root["graph_voltagelow"] = settings->graph_voltagelow;

    root["BypassOverTempShutdown"] = settings->BypassOverTempShutdown;
    root["BypassThresholdmV"] = settings->BypassThresholdmV;

    root["timeZone"] = settings->timeZone;
    root["minutesTimeZone"] = settings->minutesTimeZone;
    root["daylight"] = settings->daylight;
    root["ntpServer"] = settings->ntpServer;

    root["loggingEnabled"] = settings->loggingEnabled;
    root["loggingFrequencySeconds"] = settings->loggingFrequencySeconds;

    root["currentMonitoringEnabled"] = settings->currentMonitoringEnabled;
    root["currentMonitoringModBusAddress"] = settings->currentMonitoringModBusAddress;

    root["rs485baudrate"] = settings->rs485baudrate;
    root["rs485databits"] = settings->rs485databits;
    root["rs485parity"] = settings->rs485parity;
    root["rs485stopbits"] = settings->rs485stopbits;

    root["language"] = settings->language;

    JsonObject mqtt = root.createNestedObject("mqtt");
    mqtt["enabled"] = settings->mqtt_enabled;
    mqtt["uri"] = settings->mqtt_uri;
    mqtt["topic"] = settings->mqtt_topic;
    mqtt["username"] = settings->mqtt_username;
    mqtt["password"] = settings->mqtt_password;

    JsonObject influxdb = root.createNestedObject("influxdb");
    influxdb["enabled"] = settings->influxdb_enabled;
    influxdb["apitoken"] = settings->influxdb_apitoken;
    influxdb["bucket"] = settings->influxdb_databasebucket;
    influxdb["org"] = settings->influxdb_orgid;
    influxdb["url"] = settings->influxdb_serverurl;
    influxdb["logfreq"] = settings->influxdb_loggingFreqSeconds;

    JsonObject outputs = root.createNestedObject("outputs");

    JsonArray d = outputs.createNestedArray("default");
    JsonArray t = outputs.createNestedArray("type");
    for (uint8_t i = 0; i < RELAY_TOTAL; i++)
    {
        d.add(settings->rulerelaydefault[i]);
        t.add(settings->relaytype[i]);
    }

    JsonObject rules = root.createNestedObject("rules");
    for (uint8_t rr = 0; rr < RELAY_RULES; rr++)
    {
        // This is a default "catch all"
        String elementName = String("rule") + String(rr);

        if (rr >= 0 && rr <= MAXIMUM_RuleNumber)
        {
            // Map enum to string so when this file is re-imported we are not locked to specific index offsets
            // which may no longer map to the correct rule
            elementName = String(RuleTextDescription[rr]);
        }
        else
        {
            ESP_LOGE(TAG, "Loop outside bounds of MAXIMUM_RuleNumber");
        }

        JsonObject state = rules.createNestedObject(elementName);

        state["value"] = settings->rulevalue[rr];
        state["hysteresis"] = settings->rulehysteresis[rr];

        JsonArray relaystate = state.createNestedArray("state");
        for (uint8_t rt = 0; rt < RELAY_TOTAL; rt++)
        {
            relaystate.add(settings->rulerelaystate[rr][rt]);
        }
    } // end for

    root["canbusprotocol"] = (uint8_t)settings->canbusprotocol;
    root["nominalbatcap"] = settings->nominalbatcap;

    root["chargevolt"] = settings->chargevolt;
    root["chargecurrent"] = settings->chargecurrent;
    root["dischargecurrent"] = settings->dischargecurrent;
    root["dischargevolt"] = settings->dischargevolt;

    root["chargetemplow"] = settings->chargetemplow;
    root["chargetemphigh"] = settings->chargetemphigh;
    root["dischargetemplow"] = settings->dischargetemplow;
    root["dischargetemphigh"] = settings->dischargetemphigh;
    root["stopchargebalance"] = settings->stopchargebalance;
    root["socoverride"] = settings->socoverride;
    root["socforcelow"] = settings->socforcelow;
    root["dynamiccharge"] = settings->dynamiccharge;
    root["preventdischarge"] = settings->preventdischarge;
    root["preventcharging"] = settings->preventcharging;
    root["cellminmv"] = settings->cellminmv;
    root["cellmaxmv"] = settings->cellmaxmv;
    root["kneemv"] = settings->kneemv;

    JsonArray tv = root.createNestedArray("tilevisibility");
    for (uint8_t i = 0; i < sizeof(settings->tileconfig) / sizeof(uint16_t); i++)
    {
        tv.add(settings->tileconfig[i]);
    }

    // wifi["password"] = DIYBMSSoftAP::Config().wifi_passphrase;
}

void JSONToSettings(DynamicJsonDocument &doc, diybms_eeprom_settings *settings)
{
    // Use defaults to populate the settings, just in case we are missing values from the JSON
    DefaultConfiguration(settings);

    if (!doc.containsKey("diybms_settings"))
    {
        // Wrong document type - quit...
        return;
    }

    JsonObject root = doc["diybms_settings"];

    settings->totalNumberOfBanks = root["totalNumberOfBanks"];
    settings->totalNumberOfSeriesModules = root["totalNumberOfSeriesModules"];
    settings->baudRate = root["baudRate"];
    settings->interpacketgap = root["interpacketgap"];

    settings->graph_voltagehigh = root["graph_voltagehigh"];
    settings->graph_voltagelow = root["graph_voltagelow"];

    settings->BypassOverTempShutdown = root["BypassOverTempShutdown"];
    settings->BypassThresholdmV = root["BypassThresholdmV"];

    settings->timeZone = root["timeZone"];
    settings->minutesTimeZone = root["minutesTimeZone"];
    settings->daylight = root["daylight"];
    strncpy(settings->ntpServer, root["ntpServer"].as<String>().c_str(), sizeof(settings->ntpServer));

    settings->loggingEnabled = root["loggingEnabled"];
    settings->loggingFrequencySeconds = root["loggingFrequencySeconds"];

    settings->currentMonitoringEnabled = root["currentMonitoringEnabled"];
    settings->currentMonitoringModBusAddress = root["currentMonitoringModBusAddress"];

    settings->rs485baudrate = root["rs485baudrate"];
    settings->rs485databits = root["rs485databits"];
    settings->rs485parity = root["rs485parity"];
    settings->rs485stopbits = root["rs485stopbits"];

    strncpy(settings->language, root["language"].as<String>().c_str(), sizeof(settings->language));

    settings->canbusprotocol = (CanBusProtocolEmulation)root["canbusprotocol"];
    settings->nominalbatcap = root["nominalbatcap"];
    settings->chargevolt = root["chargevolt"];
    settings->chargecurrent = root["chargecurrent"];
    settings->dischargecurrent = root["dischargecurrent"];
    settings->dischargevolt = root["dischargevolt"];
    settings->chargetemplow = root["chargetemplow"];
    settings->chargetemphigh = root["chargetemphigh"];
    settings->dischargetemplow = root["dischargetemplow"];
    settings->dischargetemphigh = root["dischargetemphigh"];
    settings->stopchargebalance = root["stopchargebalance"];
    settings->socoverride = root["socoverride"];
    settings->socforcelow = root["socforcelow"];
    settings->dynamiccharge = root["dynamiccharge"];
    settings->preventdischarge = root["preventdischarge"];
    settings->preventcharging = root["preventcharging"];
    settings->cellminmv = root["cellminmv"];
    settings->cellmaxmv = root["cellmaxmv"];
    settings->kneemv = root["kneemv"];

    JsonObject mqtt = root["mqtt"];
    if (!mqtt.isNull())
    {
        settings->mqtt_enabled = mqtt["enabled"];
        strncpy(settings->mqtt_uri, mqtt["uri"].as<String>().c_str(), sizeof(settings->mqtt_uri));
        strncpy(settings->mqtt_topic, mqtt["topic"].as<String>().c_str(), sizeof(settings->mqtt_topic));
        strncpy(settings->mqtt_username, mqtt["username"].as<String>().c_str(), sizeof(settings->mqtt_username));
        strncpy(settings->mqtt_password, mqtt["password"].as<String>().c_str(), sizeof(settings->mqtt_password));
    }

    JsonObject influxdb = root["influxdb"];
    if (!influxdb.isNull())
    {
        settings->influxdb_enabled = influxdb["enabled"];
        strncpy(settings->influxdb_apitoken, influxdb["apitoken"].as<String>().c_str(), sizeof(settings->influxdb_apitoken));
        strncpy(settings->influxdb_databasebucket, influxdb["bucket"].as<String>().c_str(), sizeof(settings->influxdb_databasebucket));
        strncpy(settings->influxdb_orgid, influxdb["org"].as<String>().c_str(), sizeof(settings->influxdb_orgid));
        strncpy(settings->influxdb_serverurl, influxdb["url"].as<String>().c_str(), sizeof(settings->influxdb_serverurl));
        settings->influxdb_loggingFreqSeconds = influxdb["logfreq"];
    }

    JsonObject outputs = root["outputs"];
    if (!outputs.isNull())
    {
        JsonArray d = outputs["default"].as<JsonArray>();

        uint8_t i = 0;
        for (JsonVariant v : d)
        {
            settings->rulerelaydefault[i] = (RelayState)v.as<uint8_t>();
            // ESP_LOGI(TAG, "relay default %u=%u", i, myset.rulerelaydefault[i]);
            i++;

            if (i > RELAY_TOTAL)
            {
                break;
            }
        }

        JsonArray t = outputs["type"].as<JsonArray>();
        i = 0;
        for (JsonVariant v : t)
        {
            settings->relaytype[i] = (RelayType)v.as<uint8_t>();
            // ESP_LOGI(TAG, "relay type %u=%u", i, myset.relaytype[i]);
            i++;
            if (i > RELAY_TOTAL)
            {
                break;
            }
        }
    }

    JsonObject rules = root["rules"];
    if (!rules.isNull())
    {
        for (JsonPair kv : rules)
        {
            char key[64];
            strncpy(key, kv.key().c_str(), sizeof(key));
            ESP_LOGI(TAG, "rule %s", key);

            for (size_t rulenumber = 0; rulenumber <= MAXIMUM_RuleNumber; rulenumber++)
            {
                if (strcmp(RuleTextDescription[rulenumber], key) == 0)
                {
                    ESP_LOGI(TAG, "Matched to rule %u", rulenumber);
                    JsonVariant v = kv.value();
                    settings->rulevalue[rulenumber] = v["value"].as<uint32_t>();
                    // ESP_LOGI(TAG, "value=%u", myset.rulevalue[rulenumber]);
                    settings->rulehysteresis[rulenumber] = v["hysteresis"].as<uint32_t>();
                    // ESP_LOGI(TAG, "hysteresis=%u", myset.rulehysteresis[rulenumber]);
                    JsonArray states = v["state"].as<JsonArray>();

                    uint8_t i = 0;
                    for (JsonVariant v : states)
                    {
                        settings->rulerelaystate[rulenumber][i] = (RelayState)v.as<uint8_t>();
                        // ESP_LOGI(TAG, "rulerelaystate %u", myset.rulerelaystate[rulenumber][i]);
                        i++;
                        if (i > RELAY_TOTAL)
                        {
                            break;
                        }
                    }

                    break;
                }
            }
        }
    }

    uint8_t i = 0;
    for (JsonVariant v : root["tilevisibility"].as<JsonArray>())
    {
        // Need to check for over flow of tileconfig array
        settings->tileconfig[i] = v.as<uint16_t>();
    }

    // Victron
    /*
    JsonObject victron = root["victron"];
    if (!victron.isNull())
    {

        JsonArray cvl = victron["cvl"].as<JsonArray>();

        uint8_t i = 0;
        for (JsonVariant v : cvl)
        {
            myset.cvl[i] = v.as<uint16_t>();
            ESP_LOGI(TAG, "cvl %u %u", i, myset.cvl[i]);
            i++;
            if (i > 3)
            {
                break;
            }
        }

        JsonArray ccl = victron["ccl"].as<JsonArray>();

        i = 0;
        for (JsonVariant v : ccl)
        {
            myset.ccl[i] = v.as<uint16_t>();
            ESP_LOGI(TAG, "ccl %u %u", i, myset.ccl[i]);
            i++;
            if (i > 3)
            {
                break;
            }
        }

        JsonArray dcl = victron["dcl"].as<JsonArray>();

        i = 0;
        for (JsonVariant v : dcl)
        {
            myset.dcl[i] = v.as<uint16_t>();
            ESP_LOGI(TAG, "dcl %u %u", i, myset.dcl[i]);
            i++;
            if (i > 3)
            {
                break;
            }
        }
    }
    */
}