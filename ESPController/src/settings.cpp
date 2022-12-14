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
bool getSetting(nvs_handle_t handle, const char *key, uint32_t *out_value)
{
    return ValidateGetSetting(nvs_get_u32(handle, key, out_value), key);
}
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

void SaveConfiguration(diybms_eeprom_settings *settings)
{
    const char *partname = "diybms-ctrl";
    ESP_LOGI(TAG, "Write config");

    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(partname, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
    }
    else
    {
        // Save settings
        ESP_ERROR_CHECK(nvs_set_u8(nvs_handle, "totalBanks", settings->totalNumberOfBanks));
        ESP_ERROR_CHECK(nvs_set_u8(nvs_handle, "totalSeriesMod", settings->totalNumberOfSeriesModules));
        ESP_ERROR_CHECK(nvs_set_u16(nvs_handle, "baudRate", settings->baudRate));
        ESP_ERROR_CHECK(nvs_set_u16(nvs_handle, "interpacketgap", settings->interpacketgap));
        ESP_ERROR_CHECK(nvs_set_blob(nvs_handle, "rulevalue", settings->rulevalue, sizeof(settings->rulevalue)));
        ESP_ERROR_CHECK(nvs_set_blob(nvs_handle, "rulehysteresis", settings->rulehysteresis, sizeof(settings->rulehysteresis)));
        ESP_ERROR_CHECK(nvs_set_blob(nvs_handle, "rulerelaystate", settings->rulerelaystate, sizeof(settings->rulerelaystate)));
        ESP_ERROR_CHECK(nvs_set_blob(nvs_handle, "rulerelaydef", settings->rulerelaydefault, sizeof(settings->rulerelaydefault)));
        ESP_ERROR_CHECK(nvs_set_blob(nvs_handle, "relaytype", settings->relaytype, sizeof(settings->relaytype)));
        ESP_ERROR_CHECK(nvs_set_blob(nvs_handle, "g_voltagehigh", (void *)(&settings->graph_voltagehigh), sizeof(settings->graph_voltagehigh)));
        ESP_ERROR_CHECK(nvs_set_blob(nvs_handle, "g_voltagelow", (void *)(&settings->graph_voltagelow), sizeof(settings->graph_voltagelow)));
        ESP_ERROR_CHECK(nvs_set_u8(nvs_handle, "BypassOverTemp", settings->BypassOverTempShutdown));
        ESP_ERROR_CHECK(nvs_set_u16(nvs_handle, "BypassThresmV", settings->BypassThresholdmV));
        ESP_ERROR_CHECK(nvs_set_i8(nvs_handle, "TZ", settings->timeZone));
        ESP_ERROR_CHECK(nvs_set_i8(nvs_handle, "minutesTZ", settings->minutesTimeZone));
        ESP_ERROR_CHECK(nvs_set_u8(nvs_handle, "daylight", settings->daylight));

        ESP_ERROR_CHECK(nvs_set_u8(nvs_handle, "logEnabled", (uint8_t)settings->loggingEnabled));
        ESP_ERROR_CHECK(nvs_set_u16(nvs_handle, "logFreqSec", settings->loggingFrequencySeconds));

        ESP_ERROR_CHECK(nvs_set_u8(nvs_handle, "curMonEnabled", (uint8_t)settings->currentMonitoringEnabled));
        ESP_ERROR_CHECK(nvs_set_u8(nvs_handle, "curMonMBAddress", (uint8_t)settings->currentMonitoringModBusAddress));
        ESP_ERROR_CHECK(nvs_set_u8(nvs_handle, "curMonDevice", (uint8_t)settings->currentMonitoringDevice));

        ESP_ERROR_CHECK(nvs_set_i32(nvs_handle, "485baudrate", settings->rs485baudrate));
        ESP_ERROR_CHECK(nvs_set_u8(nvs_handle, "485databits", (uint8_t)settings->rs485databits));
        ESP_ERROR_CHECK(nvs_set_u8(nvs_handle, "485parity", (uint8_t)settings->rs485parity));
        ESP_ERROR_CHECK(nvs_set_u8(nvs_handle, "485stopbits", (uint8_t)settings->rs485stopbits));

        ESP_ERROR_CHECK(nvs_set_u8(nvs_handle, "canbusprotocol", (uint8_t)settings->canbusprotocol));
        ESP_ERROR_CHECK(nvs_set_u16(nvs_handle, "nominalbatcap", settings->nominalbatcap));

        ESP_ERROR_CHECK(nvs_set_u16(nvs_handle, "cha_volt", settings->chargevolt));
        ESP_ERROR_CHECK(nvs_set_u16(nvs_handle, "cha_current", settings->chargecurrent));
        ESP_ERROR_CHECK(nvs_set_u16(nvs_handle, "dis_current", settings->dischargecurrent));
        ESP_ERROR_CHECK(nvs_set_u16(nvs_handle, "dis_volt", settings->dischargevolt));
        ESP_ERROR_CHECK(nvs_set_u16(nvs_handle, "cellminmv", settings->cellminmv));
        ESP_ERROR_CHECK(nvs_set_u16(nvs_handle, "cellmaxmv", settings->cellmaxmv));
        ESP_ERROR_CHECK(nvs_set_u16(nvs_handle, "kneemv", settings->kneemv));
        ESP_ERROR_CHECK(nvs_set_i8(nvs_handle, "cha_templow", settings->chargetemplow));
        ESP_ERROR_CHECK(nvs_set_i8(nvs_handle, "cha_temphigh", settings->chargetemphigh));
        ESP_ERROR_CHECK(nvs_set_i8(nvs_handle, "dis_templow", settings->dischargetemplow));
        ESP_ERROR_CHECK(nvs_set_i8(nvs_handle, "dis_temphigh", settings->dischargetemphigh));
        ESP_ERROR_CHECK(nvs_set_u8(nvs_handle, "stopchargebal", (uint8_t)settings->stopchargebalance));

        ESP_ERROR_CHECK(nvs_set_u8(nvs_handle, "socoverride", (uint8_t)settings->socoverride));
        ESP_ERROR_CHECK(nvs_set_u8(nvs_handle, "socforcelow", (uint8_t)settings->socforcelow));

        ESP_ERROR_CHECK(nvs_set_u8(nvs_handle, "dynamiccharge", (uint8_t)settings->dynamiccharge));
        ESP_ERROR_CHECK(nvs_set_u8(nvs_handle, "preventchar", (uint8_t)settings->preventcharging));
        ESP_ERROR_CHECK(nvs_set_u8(nvs_handle, "preventdis", (uint8_t)settings->preventdischarge));

        ESP_ERROR_CHECK(nvs_set_u8(nvs_handle, "mqttenable", (uint8_t)settings->mqtt_enabled));
        ESP_ERROR_CHECK(nvs_set_u8(nvs_handle, "infenabled", (uint8_t)settings->influxdb_enabled));
        ESP_ERROR_CHECK(nvs_set_u8(nvs_handle, "inflogFreq", settings->influxdb_loggingFreqSeconds));

        ESP_ERROR_CHECK(nvs_set_blob(nvs_handle, "tileconfig", settings->tileconfig, sizeof(settings->tileconfig)));

        ESP_ERROR_CHECK(nvs_set_str(nvs_handle, "ntpServer", &settings->ntpServer[0]));
        ESP_ERROR_CHECK(nvs_set_str(nvs_handle, "language", &settings->language[0]));
        ESP_ERROR_CHECK(nvs_set_str(nvs_handle, "mqtt_uri", &settings->mqtt_uri[0]));
        ESP_ERROR_CHECK(nvs_set_str(nvs_handle, "mqtt_topic", &settings->mqtt_topic[0]));
        ESP_ERROR_CHECK(nvs_set_str(nvs_handle, "mqtt_usern", &settings->mqtt_username[0]));
        ESP_ERROR_CHECK(nvs_set_str(nvs_handle, "mqtt_pword", &settings->mqtt_password[0]));

        ESP_ERROR_CHECK(nvs_set_str(nvs_handle, "inf_serverurl", &settings->influxdb_serverurl[0]));
        ESP_ERROR_CHECK(nvs_set_str(nvs_handle, "inf_bucket", &settings->influxdb_databasebucket[0]));
        ESP_ERROR_CHECK(nvs_set_str(nvs_handle, "inf_apitoken", &settings->influxdb_apitoken[0]));
        ESP_ERROR_CHECK(nvs_set_str(nvs_handle, "inf_orgid", &settings->influxdb_orgid[0]));

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
    _myset->graph_voltagehigh = 4.5;
    _myset->graph_voltagelow = 2.75;

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
}