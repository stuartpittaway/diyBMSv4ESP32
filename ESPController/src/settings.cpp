#define USE_ESP_IDF_LOG 1
static constexpr const char *const TAG = "diybms-set";

#include "settings.h"

static const char totalNumberOfBanks_JSONKEY[] = "totalNumberOfBanks";
static const char totalNumberOfSeriesModules_JSONKEY[] = "totalNumberOfSeriesModules";

/* NVS KEYS
THESE STRINGS ARE USED TO HOLD THE PARAMETER IN NVS FLASH, MAXIMUM LENGTH OF 16 CHARACTERS
*/
static const char totalNumberOfBanks_NVSKEY[] = "totalBanks";
static const char totalNumberOfSeriesModules_NVSKEY[] = "totalSeriesMod";
static const char baudRate_NVSKEY[] = "baudRate";
static const char interpacketgap_NVSKEY[] = "interpacketgap";
static const char rulevalue_NVSKEY[] = "rulevalue";
static const char rulehysteresis_NVSKEY[] = "rulehysteresis";
static const char rulerelaystate_NVSKEY[] = "rulerelaystate";
static const char rulerelaydefault_NVSKEY[] = "rulerelaydef";
static const char relaytype_NVSKEY[] = "relaytype";
static const char graph_voltagehigh_NVSKEY[] = "g_voltagehigh";
static const char graph_voltagelow_NVSKEY[] = "g_voltagelow";
static const char BypassOverTempShutdown_NVSKEY[] = "BypassOverTemp";
static const char BypassThresholdmV_NVSKEY[] = "BypassThresmV";
static const char timeZone_NVSKEY[] = "TZ";
static const char minutesTimeZone_NVSKEY[] = "minutesTZ";
static const char daylight_NVSKEY[] = "daylight";
static const char loggingEnabled_NVSKEY[] = "logEnabled";
static const char loggingFrequencySeconds_NVSKEY[] = "logFreqSec";
static const char currentMonitoringEnabled_NVSKEY[] = "curMonEnabled";
static const char currentMonitoringModBusAddress_NVSKEY[] = "curMonMBAddress";
static const char currentMonitoringDevice_NVSKEY[] = "curMonDevice";
static const char rs485baudrate_NVSKEY[] = "485baudrate";
static const char rs485databits_NVSKEY[] = "485databits";
static const char rs485parity_NVSKEY[] = "485parity";
static const char rs485stopbits_NVSKEY[] = "485stopbits";
static const char canbusprotocol_NVSKEY[] = "canbusprotocol";
static const char nominalbatcap_NVSKEY[] = "nominalbatcap";
static const char chargevolt_NVSKEY[] = "cha_volt";
static const char chargecurrent_NVSKEY[] = "cha_current";
static const char dischargecurrent_NVSKEY[] = "dis_current";
static const char dischargevolt_NVSKEY[] = "dis_volt";
static const char cellminmv_NVSKEY[] = "cellminmv";
static const char cellmaxmv_NVSKEY[] = "cellmaxmv";
static const char kneemv_NVSKEY[] = "kneemv";
static const char sensitivity_NVSKEY[] = "sensitivity";
static const char current_value1_NVSKEY[] = "cur_val1";
static const char current_value2_NVSKEY[] = "cur_val2";
static const char cellmaxspikemv_NVSKEY[] = "cellmaxspikemv";
static const char chargetemplow_NVSKEY[] = "cha_templow";
static const char chargetemphigh_NVSKEY[] = "cha_temphigh";
static const char dischargetemplow_NVSKEY[] = "dis_templow";
static const char dischargetemphigh_NVSKEY[] = "dis_temphigh";
static const char stopchargebalance_NVSKEY[] = "stopchargebal";
static const char socoverride_NVSKEY[] = "socoverride";
static const char socforcelow_NVSKEY[] = "socforcelow";
static const char dynamiccharge_NVSKEY[] = "dynamiccharge";
static const char preventcharging_NVSKEY[] = "preventchar";
static const char preventdischarge_NVSKEY[] = "preventdis";
static const char mqtt_enabled_NVSKEY[] = "mqttenable";
static const char influxdb_enabled_NVSKEY[] = "infenabled";
static const char influxdb_loggingFreqSeconds_NVSKEY[] = "inflogFreq";
static const char tileconfig_NVSKEY[] = "tileconfig";
static const char ntpServer_NVSKEY[] = "ntpServer";
static const char language_NVSKEY[] = "language";
static const char mqtt_uri_NVSKEY[] = "mqtt_uri";
static const char mqtt_topic_NVSKEY[] = "mqtt_topic";
static const char mqtt_username_NVSKEY[] = "mqtt_usern";
static const char mqtt_password_NVSKEY[] = "mqtt_pword";
static const char influxdb_serverurl_NVSKEY[] = "inf_serverurl";
static const char influxdb_databasebucket_NVSKEY[] = "inf_bucket";
static const char influxdb_apitoken_NVSKEY[] = "inf_apitoken";
static const char influxdb_orgid_NVSKEY[] = "inf_orgid";

#define MACRO_NVSWRITE(VARNAME) writeSetting(nvs_handle, VARNAME##_NVSKEY, settings->VARNAME);
#define MACRO_NVSWRITE_UINT8(VARNAME) writeSetting(nvs_handle, VARNAME##_NVSKEY, (uint8_t)settings->VARNAME);
#define MACRO_NVSWRITESTRING(VARNAME) writeSetting(nvs_handle, VARNAME##_NVSKEY, &settings->VARNAME[0]);
#define MACRO_NVSWRITEBLOB(VARNAME) writeSettingBlob(nvs_handle, VARNAME##_NVSKEY, settings->VARNAME, sizeof(settings->VARNAME));

// Macros to read NVS keys into variables
#define MACRO_NVSREAD(VARNAME) getSetting(nvs_handle, VARNAME##_NVSKEY, &settings->VARNAME);
#define MACRO_NVSREAD_UINT8(VARNAME) getSetting(nvs_handle, VARNAME##_NVSKEY, (uint8_t *)&settings->VARNAME);
#define MACRO_NVSREADSTRING(VARNAME) getString(nvs_handle, VARNAME##_NVSKEY, &settings->VARNAME[0], sizeof(settings->VARNAME));
#define MACRO_NVSREADBLOB(VARNAME) getSettingBlob(nvs_handle, VARNAME##_NVSKEY, &settings->VARNAME, sizeof(settings->VARNAME));

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
bool getSetting(nvs_handle_t handle, const char *key, int16_t *out_value)
{
    return ValidateGetSetting(nvs_get_i16(handle, key, out_value), key);
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

void writeSetting(nvs_handle_t handle, const char *key, bool value)
{
    writeSetting(handle, key, (uint8_t)value);
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
void writeSetting(nvs_handle_t handle, const char *key, int16_t value)
{
    ESP_LOGD(TAG, "Writing (%s)=%i", key, value);
    ESP_ERROR_CHECK(nvs_set_i16(handle, key, value));
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
        MACRO_NVSWRITE(totalNumberOfBanks)
        MACRO_NVSWRITE(totalNumberOfSeriesModules)
        MACRO_NVSWRITE(baudRate)
        MACRO_NVSWRITE(interpacketgap)

        MACRO_NVSWRITEBLOB(rulevalue);
        MACRO_NVSWRITEBLOB(rulehysteresis);
        MACRO_NVSWRITEBLOB(rulerelaystate);
        MACRO_NVSWRITEBLOB(rulerelaydefault);
        MACRO_NVSWRITEBLOB(relaytype);

        MACRO_NVSWRITE(graph_voltagehigh)
        MACRO_NVSWRITE(graph_voltagelow)
        MACRO_NVSWRITE(BypassOverTempShutdown)
        MACRO_NVSWRITE(BypassThresholdmV)
        MACRO_NVSWRITE(timeZone)
        MACRO_NVSWRITE(minutesTimeZone)
        MACRO_NVSWRITE(daylight)
        MACRO_NVSWRITE(loggingEnabled)
        MACRO_NVSWRITE(loggingFrequencySeconds)

        MACRO_NVSWRITE(currentMonitoringEnabled)
        MACRO_NVSWRITE(currentMonitoringModBusAddress)
        MACRO_NVSWRITE_UINT8(currentMonitoringDevice);
        MACRO_NVSWRITE(rs485baudrate)
        MACRO_NVSWRITE_UINT8(rs485databits);
        MACRO_NVSWRITE_UINT8(rs485parity);
        MACRO_NVSWRITE_UINT8(rs485stopbits);
        MACRO_NVSWRITE_UINT8(canbusprotocol);

        MACRO_NVSWRITE(nominalbatcap)

        MACRO_NVSWRITE(chargevolt)
        MACRO_NVSWRITE(chargecurrent)
        MACRO_NVSWRITE(dischargecurrent)
        MACRO_NVSWRITE(dischargevolt)
        MACRO_NVSWRITE(cellminmv)
        MACRO_NVSWRITE(cellmaxmv)
        MACRO_NVSWRITE(kneemv)
        MACRO_NVSWRITE(sensitivity);
        MACRO_NVSWRITE(current_value1);
        MACRO_NVSWRITE(current_value2);
        MACRO_NVSWRITE(cellmaxspikemv);
        MACRO_NVSWRITE(chargetemplow);
        MACRO_NVSWRITE(chargetemphigh);
        MACRO_NVSWRITE(dischargetemplow);
        MACRO_NVSWRITE(dischargetemphigh);
        MACRO_NVSWRITE(stopchargebalance);
        MACRO_NVSWRITE(socoverride);
        MACRO_NVSWRITE(socforcelow);

        MACRO_NVSWRITE(dynamiccharge);
        MACRO_NVSWRITE(preventcharging);
        MACRO_NVSWRITE(preventdischarge);
        MACRO_NVSWRITE(mqtt_enabled);
        MACRO_NVSWRITE(influxdb_enabled);
        MACRO_NVSWRITE(influxdb_loggingFreqSeconds);

        MACRO_NVSWRITEBLOB(tileconfig);

        MACRO_NVSWRITESTRING(ntpServer);
        MACRO_NVSWRITESTRING(language);
        MACRO_NVSWRITESTRING(mqtt_uri);
        MACRO_NVSWRITESTRING(mqtt_topic);
        MACRO_NVSWRITESTRING(mqtt_username);
        MACRO_NVSWRITESTRING(mqtt_password);
        MACRO_NVSWRITESTRING(influxdb_serverurl);
        MACRO_NVSWRITESTRING(influxdb_databasebucket);
        MACRO_NVSWRITESTRING(influxdb_apitoken);
        MACRO_NVSWRITESTRING(influxdb_orgid);

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
    // Apply validation rules, just to ensure the defaults make sense
    ValidateConfiguration(settings);

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
        MACRO_NVSREAD(totalNumberOfBanks);
        MACRO_NVSREAD(totalNumberOfSeriesModules);
        MACRO_NVSREAD(baudRate);
        MACRO_NVSREAD(interpacketgap);

        MACRO_NVSREADBLOB(rulevalue);
        MACRO_NVSREADBLOB(rulehysteresis);
        MACRO_NVSREADBLOB(rulerelaystate);
        MACRO_NVSREADBLOB(rulerelaydefault);
        MACRO_NVSREADBLOB(relaytype);

        MACRO_NVSREAD(graph_voltagehigh);
        MACRO_NVSREAD(graph_voltagelow);
        MACRO_NVSREAD(BypassOverTempShutdown);
        MACRO_NVSREAD(BypassThresholdmV);
        MACRO_NVSREAD(minutesTimeZone);
        MACRO_NVSREAD(daylight);
        MACRO_NVSREAD(loggingEnabled);
        MACRO_NVSREAD(loggingFrequencySeconds);
        MACRO_NVSREAD(currentMonitoringEnabled);
        MACRO_NVSREAD(currentMonitoringModBusAddress);
        MACRO_NVSREAD_UINT8(currentMonitoringModBusAddress);
        MACRO_NVSREAD(rs485baudrate);
        MACRO_NVSREAD_UINT8(rs485databits);
        MACRO_NVSREAD_UINT8(rs485parity);
        MACRO_NVSREAD_UINT8(rs485stopbits);
        MACRO_NVSREAD_UINT8(canbusprotocol);
        MACRO_NVSREAD(nominalbatcap);
        MACRO_NVSREAD(chargevolt);
        MACRO_NVSREAD(chargecurrent);
        MACRO_NVSREAD(dischargecurrent);
        MACRO_NVSREAD(dischargevolt);
        MACRO_NVSREAD(cellminmv);
        MACRO_NVSREAD(cellmaxmv);
        MACRO_NVSREAD(kneemv);
        MACRO_NVSREAD(sensitivity);
        MACRO_NVSREAD(current_value1);
        MACRO_NVSREAD(current_value2);
        MACRO_NVSREAD(cellmaxspikemv);
        MACRO_NVSREAD(chargetemplow);
        MACRO_NVSREAD(chargetemphigh);
        MACRO_NVSREAD(dischargetemplow);
        MACRO_NVSREAD(dischargetemphigh);
        MACRO_NVSREAD(stopchargebalance);
        MACRO_NVSREAD(socoverride);
        MACRO_NVSREAD(socforcelow);

        MACRO_NVSREAD(dynamiccharge);
        MACRO_NVSREAD(preventcharging);
        MACRO_NVSREAD(preventdischarge);

        MACRO_NVSREAD(mqtt_enabled);
        MACRO_NVSREAD(influxdb_enabled);
        MACRO_NVSREAD(influxdb_loggingFreqSeconds);

        MACRO_NVSREADBLOB(tileconfig);

        MACRO_NVSREADSTRING(ntpServer);
        MACRO_NVSREADSTRING(language);
        MACRO_NVSREADSTRING(mqtt_uri);
        MACRO_NVSREADSTRING(mqtt_topic);
        MACRO_NVSREADSTRING(mqtt_username);
        MACRO_NVSREADSTRING(mqtt_password);
        MACRO_NVSREADSTRING(influxdb_serverurl);
        MACRO_NVSREADSTRING(influxdb_databasebucket);
        MACRO_NVSREADSTRING(influxdb_apitoken);
        MACRO_NVSREADSTRING(influxdb_orgid);

        nvs_close(nvs_handle);
    }

    // Ensure values make sense
    ValidateConfiguration(settings);
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
    _myset->nominalbatcap = 280;    // Scale 1
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
    _myset->sensitivity = 30; // Scale 0.1

    _myset->current_value1 = 50; // 5.0
    _myset->current_value2 = 03; // 0.3

    // Allow this "safe" cell voltage to allow a bit of wiggle room/spike control
    _myset->cellmaxspikemv = 3550;
    _myset->stopchargebalance = false;
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
    _myset->rulevalue[Rule::ModuleOverTemperatureExternal] = 50;
    // Individual cell under temperature (external probe)
    _myset->rulevalue[Rule::ModuleUnderTemperatureExternal] = 2;
    // Bank Over voltage (mV)
    _myset->rulevalue[Rule::BankOverVoltage] = 4200 * 8;
    // RULE_BankUnderVoltage
    _myset->rulevalue[Rule::BankUnderVoltage] = 3000 * 8;
    // Bank range
    _myset->rulevalue[Rule::BankRange] = 30;

    _myset->rulevalue[Rule::Timer1] = 60 * 8;  // 8am
    _myset->rulevalue[Rule::Timer2] = 60 * 17; // 5pm

    // Temperature of the actual module/PCB attached to a cell (internal temperature)
    _myset->rulevalue[Rule::ModuleOverTemperatureInternal] = 75;
    _myset->rulevalue[Rule::ModuleUnderTemperatureInternal] = 5;

    _myset->rulevalue[Rule::CurrentMonitorOverVoltage] = 4200 * 8;
    _myset->rulevalue[Rule::CurrentMonitorUnderVoltage] = 3000 * 8;

    // Set rulehysteresis to match the rulevalue as the default
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

    // Override hysteresis values if needed
    _myset->rulehysteresis[Rule::BankRange] = 15;
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

    if (settings->graph_voltagehigh > 5000 || settings->graph_voltagehigh < 2000 || settings->graph_voltagehigh < 0)
    {
        settings->graph_voltagehigh = defaults.graph_voltagehigh;
    }

    if (settings->graph_voltagelow > settings->graph_voltagehigh || settings->graph_voltagelow < 0)
    {
        settings->graph_voltagelow = 0;
    }

    if (settings->cellmaxmv > settings->cellmaxspikemv)
    {
        settings->cellmaxmv = defaults.cellmaxmv;
    }
    if (settings->cellmaxmv < settings->cellminmv)
    {
        settings->cellmaxmv = settings->cellminmv;
    }
    if (settings->cellminmv > settings->cellmaxmv)
    {
        settings->cellminmv = settings->cellmaxmv;
    }
    if (settings->kneemv > settings->cellmaxmv || settings->kneemv > settings->cellmaxspikemv)
    {
        settings->kneemv = defaults.kneemv;
    }

    // Limit to 1
    if (settings->sensitivity < 1 * 10)
    {
        settings->sensitivity = 1 * 10;
    }
    // Limit to 100
    if (settings->sensitivity > 100 * 10)
    {
        settings->sensitivity = 100 * 10;
    }

    // Limit to 100
    if (settings->current_value1 > 100 * 10)
    {
        settings->current_value1 = 100 * 10;
    }
    if (settings->current_value2 > 100 * 10)
    {
        settings->current_value2 = 100 * 10;
    }

    // Ensure that all PULSE relays default to OFF (pulse will only pulse on/off not off/on)
    for (uint8_t i = 0; i < RELAY_TOTAL; i++)
    {
        if (settings->relaytype[i] == RelayType::RELAY_PULSE)
        {
            settings->rulerelaydefault[i] = RelayState::RELAY_OFF;
        }
    }

    // Ensure trigger and reset (rulevalue and rulehysteresis) values make sense and
    // the rulehysteresis value is either greater or lower than rulevalue as required.

    // These rules expect the hysteresis (reset) value to be LOWER than the trigger value
    const Rule hysteresis_lower[] = {Rule::CurrentMonitorOverCurrentAmps,
                                     Rule::ModuleOverVoltage,
                                     Rule::ModuleOverTemperatureInternal,
                                     Rule::ModuleOverTemperatureExternal,
                                     Rule::CurrentMonitorOverVoltage,
                                     Rule::BankOverVoltage,
                                     Rule::BankRange};

    for (size_t i = 0; i < sizeof(hysteresis_lower); i++)
    {
        Rule index = hysteresis_lower[i];

        if (settings->rulehysteresis[index] > settings->rulevalue[index])
        {
            ESP_LOGI(TAG, "Fixed LOWER hysteresis %u from %i to %i", (uint8_t)index, settings->rulehysteresis[index], settings->rulevalue[index]);
            settings->rulehysteresis[index] = settings->rulevalue[index];
        }
    }

    // These rules expect the hysteresis (reset) value to be GREATER than the trigger value
    const Rule hysteresis_greater[] = {Rule::ModuleUnderVoltage,
                                       Rule::ModuleUnderTemperatureInternal,
                                       Rule::ModuleUnderTemperatureExternal,
                                       Rule::CurrentMonitorUnderVoltage,
                                       Rule::BankUnderVoltage,
                                       Rule::Timer2,
                                       Rule::Timer1};

    for (size_t i = 0; i < sizeof(hysteresis_greater); i++)
    {
        Rule index = hysteresis_greater[i];

        if (settings->rulehysteresis[index] < settings->rulevalue[index])
        {
            ESP_LOGI(TAG, "Fixed GREATER hysteresis %u from %i to %i", (uint8_t)index, settings->rulehysteresis[index], settings->rulevalue[index]);
            settings->rulehysteresis[index] = settings->rulevalue[index];
        }
    }
}

// Builds up a JSON document which mirrors the parameters inside "diybms_eeprom_settings"
void GenerateSettingsJSONDocument(DynamicJsonDocument *doc, diybms_eeprom_settings *settings)
{
    JsonObject root = doc->createNestedObject("diybms_settings");

    root[totalNumberOfBanks_JSONKEY] = settings->totalNumberOfBanks;
    root[totalNumberOfSeriesModules_JSONKEY] = settings->totalNumberOfSeriesModules;
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

    root["cellmaxspikemv"] = settings->cellmaxspikemv;
    root["sensitivity"] = settings->sensitivity;
    root["cur_val1"] = settings->current_value1;
    root["cur_val2"] = settings->current_value2;

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

    settings->totalNumberOfBanks = root[totalNumberOfBanks_JSONKEY];
    settings->totalNumberOfSeriesModules = root[totalNumberOfSeriesModules_JSONKEY];
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

    settings->cellmaxspikemv = root["cellmaxspikemv"];
    settings->sensitivity = root["sensitivity"];

    settings->current_value1 = root["cur_val1"];
    settings->current_value2 = root["cur_val2"];

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
                    // ESP_LOGI(TAG, "value=%i", myset.rulevalue[rulenumber]);
                    settings->rulehysteresis[rulenumber] = v["hysteresis"].as<uint32_t>();
                    // ESP_LOGI(TAG, "hysteresis=%i", myset.rulehysteresis[rulenumber]);
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