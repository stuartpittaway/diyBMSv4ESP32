#define USE_ESP_IDF_LOG 1
static constexpr const char *const TAG = "diybms-set";

#include "settings.h"

/*
THESE STRINGS ARE USED AS KEYS IN THE JSON SETTINGS BACKUP FILES
DEFINED HERE (ONCE) TO ENSURE TYPOS ARE NOT MADE
*/
static const char totalNumberOfBanks_JSONKEY[] = "totalNumberOfBanks";
static const char totalNumberOfSeriesModules_JSONKEY[] = "totalNumberOfSeriesModules";
static const char baudRate_JSONKEY[] = "baudRate";
static const char interpacketgap_JSONKEY[] = "interpacketgap";
static const char graph_voltagehigh_JSONKEY[] = "graph_voltagehigh";
static const char graph_voltagelow_JSONKEY[] = "graph_voltagelow";
static const char BypassOverTempShutdown_JSONKEY[] = "BypassOverTempShutdown";
static const char BypassThresholdmV_JSONKEY[] = "BypassThresholdmV";
static const char timeZone_JSONKEY[] = "timeZone";
static const char minutesTimeZone_JSONKEY[] = "minutesTimeZone";
static const char daylight_JSONKEY[] = "daylight";
static const char ntpServer_JSONKEY[] = "ntpServer";
static const char loggingEnabled_JSONKEY[] = "loggingEnabled";
static const char loggingFrequencySeconds_JSONKEY[] = "loggingFrequencySeconds";
static const char currentMonitoringEnabled_JSONKEY[] = "currentMonitoringEnabled";
static const char currentMonitoringModBusAddress_JSONKEY[] = "currentMonitoringModBusAddress";
static const char rs485baudrate_JSONKEY[] = "rs485baudrate";
static const char rs485databits_JSONKEY[] = "rs485databits";
static const char rs485parity_JSONKEY[] = "rs485parity";
static const char rs485stopbits_JSONKEY[] = "rs485stopbits";
static const char language_JSONKEY[] = "language";
static const char mqtt_enabled_JSONKEY[] = "enabled";
static const char mqtt_basic_cell_reporting_JSONKEY[] = "basiccellrpt";
static const char mqtt_uri_JSONKEY[] = "uri";
static const char mqtt_topic_JSONKEY[] = "topic";
static const char mqtt_username_JSONKEY[] = "username";
static const char mqtt_password_JSONKEY[] = "password";
static const char influxdb_enabled_JSONKEY[] = "enabled";
static const char influxdb_apitoken_JSONKEY[] = "apitoken";
static const char influxdb_databasebucket_JSONKEY[] = "bucket";
static const char influxdb_orgid_JSONKEY[] = "org";
static const char influxdb_serverurl_JSONKEY[] = "url";
static const char influxdb_loggingFreqSeconds_JSONKEY[] = "logfreq";
static const char canbusprotocol_JSONKEY[] = "canbusprotocol";
static const char canbusinverter_JSONKEY[] = "canbusinverter";
static const char canbusbaud_JSONKEY[] = "canbusbaud";
static const char canbus_equipment_addr_JSONKEY[] = "canbusequip";
static const char nominalbatcap_JSONKEY[] = "nominalbatcap";
static const char chargevolt_JSONKEY[] = "chargevolt";
static const char chargecurrent_JSONKEY[] = "chargecurrent";
static const char dischargecurrent_JSONKEY[] = "dischargecurrent";
static const char dischargevolt_JSONKEY[] = "dischargevolt";
static const char chargetemplow_JSONKEY[] = "chargetemplow";
static const char chargetemphigh_JSONKEY[] = "chargetemphigh";
static const char dischargetemplow_JSONKEY[] = "dischargetemplow";
static const char dischargetemphigh_JSONKEY[] = "dischargetemphigh";
static const char stopchargebalance_JSONKEY[] = "stopchargebalance";
static const char socoverride_JSONKEY[] = "socoverride";
static const char socforcelow_JSONKEY[] = "socforcelow";
static const char dynamiccharge_JSONKEY[] = "dynamiccharge";
static const char preventdischarge_JSONKEY[] = "preventdischarge";
static const char preventcharging_JSONKEY[] = "preventcharging";
static const char cellminmv_JSONKEY[] = "cellminmv";
static const char cellmaxmv_JSONKEY[] = "cellmaxmv";
static const char kneemv_JSONKEY[] = "kneemv";
static const char cellmaxspikemv_JSONKEY[] = "cellmaxspikemv";
static const char sensitivity_JSONKEY[] = "sensitivity";
static const char current_value1_JSONKEY[] = "cur_val1";
static const char current_value2_JSONKEY[] = "cur_val2";

static const char currentMonitoringDevice_JSONKEY[] = "currentMonitoringDevice";
static const char currentMonitoring_shuntmv_JSONKEY[] = "currentMonitoringShuntmv";
static const char currentMonitoring_shuntmaxcur_JSONKEY[] = "currentMonitoringShuntMaxCur";
static const char currentMonitoring_batterycapacity_JSONKEY[] = "currentMonitoringBatteryCap";
static const char currentMonitoring_fullchargevolt_JSONKEY[] = "currentMonitoringFullChargeVolt";
static const char currentMonitoring_tailcurrent_JSONKEY[] = "currentMonitoringTailCurrent";
static const char currentMonitoring_chargeefficiency_JSONKEY[] = "currentMonitoringChargeEfficiency";

static const char currentMonitoring_shuntcal_JSONKEY[] = "currentMonitoringShuntCal";
static const char currentMonitoring_temperaturelimit_JSONKEY[] = "currentMonitoringTempLimit";
static const char currentMonitoring_overvoltagelimit_JSONKEY[] = "currentMonitoringOverVLimit";
static const char currentMonitoring_undervoltagelimit_JSONKEY[] = "currentMonitoringUnderVLimit";
static const char currentMonitoring_overcurrentlimit_JSONKEY[] = "currentMonitoringOverCurrent";
static const char currentMonitoring_undercurrentlimit_JSONKEY[] = "currentMonitoringUnderCurrent";
static const char currentMonitoring_overpowerlimit_JSONKEY[] = "currentMonitoringOverPower";
static const char currentMonitoring_shunttempcoefficient_JSONKEY[] = "currentMonitoringShuntTempCoeff";
static const char currentMonitoring_tempcompenabled_JSONKEY[] = "currentMonitoringTempCompEnable";

static const char absorptiontimer_JSONKEY[] = "absorptiontimer";
static const char floatvoltage_JSONKEY[] = "floatvoltage";
static const char floatvoltagetimer_JSONKEY[] = "floatvoltagetimer";
static const char stateofchargeresumevalue_JSONKEY[] = "stateofchargeresumevalue";
static const char homeassist_apikey_JSONKEY[] = "homeassistapikey";

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
static const char canbusinverter_NVSKEY[] = "canbusinverter";
static const char canbusbaud_NVSKEY[] = "canbusbaud";
static const char canbus_equipment_addr_NVSKEY[]="canbusequip";
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
static const char mqtt_basic_cell_reporting_NVSKEY[] = "basiccellrpt";
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

static const char currentMonitoring_shuntmv_NVSKEY[] = "curMonshuntmv";
static const char currentMonitoring_shuntmaxcur_NVSKEY[] = "curMonShtMaxCur";
static const char currentMonitoring_batterycapacity_NVSKEY[] = "curMonBatCap";
static const char currentMonitoring_fullchargevolt_NVSKEY[] = "curMonFullChgV";
static const char currentMonitoring_tailcurrent_NVSKEY[] = "curMonTailCur";
static const char currentMonitoring_chargeefficiency_NVSKEY[] = "curMonChargeEff";
static const char currentMonitoring_shuntcal_NVSKEY[] = "curMonShuntCal";
static const char currentMonitoring_temperaturelimit_NVSKEY[] = "curMontemplimit";
static const char currentMonitoring_overvoltagelimit_NVSKEY[] = "curMonovervolt";
static const char currentMonitoring_undervoltagelimit_NVSKEY[] = "curMonundervolt";
static const char currentMonitoring_overcurrentlimit_NVSKEY[] = "curMonovercur";
static const char currentMonitoring_undercurrentlimit_NVSKEY[] = "curMonundercur";
static const char currentMonitoring_overpowerlimit_NVSKEY[] = "curMonoverpower";
static const char currentMonitoring_shunttempcoefficient_NVSKEY[] = "curMontempcoef";
static const char currentMonitoring_tempcompenabled_NVSKEY[] = "curMonTempCompE";

static const char absorptiontimer_NVSKEY[] = "absorptimer";
static const char floatvoltage_NVSKEY[] = "floatV";
static const char floatvoltagetimer_NVSKEY[] = "floatVtimer";
static const char stateofchargeresumevalue_NVSKEY[] = "socresume";
static const char homeassist_apikey_NVSKEY[] = "haapikey";

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
        ESP_LOGD(TAG, "Read key (%s)", key);
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
    esp_err_t err = nvs_get_blob(handle, key, nullptr, &stored_size);
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
        MACRO_NVSWRITE_UINT8(canbusinverter);
        MACRO_NVSWRITE(canbusbaud);
        MACRO_NVSWRITE_UINT8(canbus_equipment_addr);

        MACRO_NVSWRITE(currentMonitoring_shuntmv);
        MACRO_NVSWRITE(currentMonitoring_shuntmaxcur);
        MACRO_NVSWRITE(currentMonitoring_batterycapacity);
        MACRO_NVSWRITE(currentMonitoring_fullchargevolt);
        MACRO_NVSWRITE(currentMonitoring_tailcurrent);
        MACRO_NVSWRITE(currentMonitoring_chargeefficiency);
        MACRO_NVSWRITE(currentMonitoring_shuntcal);
        MACRO_NVSWRITE(currentMonitoring_temperaturelimit);
        MACRO_NVSWRITE(currentMonitoring_overvoltagelimit);
        MACRO_NVSWRITE(currentMonitoring_undervoltagelimit);
        MACRO_NVSWRITE(currentMonitoring_overcurrentlimit);
        MACRO_NVSWRITE(currentMonitoring_undercurrentlimit);
        MACRO_NVSWRITE(currentMonitoring_overpowerlimit);
        MACRO_NVSWRITE(currentMonitoring_shunttempcoefficient);
        MACRO_NVSWRITE(currentMonitoring_tempcompenabled);

        MACRO_NVSWRITE(nominalbatcap);
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
        MACRO_NVSWRITE(mqtt_basic_cell_reporting);
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

        MACRO_NVSWRITE(absorptiontimer);
        MACRO_NVSWRITE(floatvoltage);
        MACRO_NVSWRITE(floatvoltagetimer);
        MACRO_NVSWRITE(stateofchargeresumevalue);

        MACRO_NVSWRITESTRING(homeassist_apikey);

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
        MACRO_NVSREAD(timeZone);
        MACRO_NVSREAD(minutesTimeZone);
        MACRO_NVSREAD(daylight);
        MACRO_NVSREAD(loggingEnabled);
        MACRO_NVSREAD(loggingFrequencySeconds);

        MACRO_NVSREAD(currentMonitoringEnabled);
        MACRO_NVSREAD(currentMonitoringModBusAddress);
        MACRO_NVSREAD_UINT8(currentMonitoringDevice);

        MACRO_NVSREAD(currentMonitoring_shuntmv);
        MACRO_NVSREAD(currentMonitoring_shuntmaxcur);
        MACRO_NVSREAD(currentMonitoring_batterycapacity);
        MACRO_NVSREAD(currentMonitoring_fullchargevolt);
        MACRO_NVSREAD(currentMonitoring_tailcurrent);
        MACRO_NVSREAD(currentMonitoring_chargeefficiency);
        MACRO_NVSREAD(currentMonitoring_shuntcal);
        MACRO_NVSREAD(currentMonitoring_temperaturelimit);
        MACRO_NVSREAD(currentMonitoring_overvoltagelimit);
        MACRO_NVSREAD(currentMonitoring_undervoltagelimit);
        MACRO_NVSREAD(currentMonitoring_overcurrentlimit);
        MACRO_NVSREAD(currentMonitoring_undercurrentlimit);
        MACRO_NVSREAD(currentMonitoring_overpowerlimit);
        MACRO_NVSREAD(currentMonitoring_shunttempcoefficient);
        MACRO_NVSREAD(currentMonitoring_tempcompenabled);

        MACRO_NVSREAD(rs485baudrate);
        MACRO_NVSREAD_UINT8(rs485databits);
        MACRO_NVSREAD_UINT8(rs485parity);
        MACRO_NVSREAD_UINT8(rs485stopbits);
        MACRO_NVSREAD_UINT8(canbusprotocol);
        MACRO_NVSREAD_UINT8(canbusinverter);
        MACRO_NVSREAD(canbusbaud);
        MACRO_NVSREAD_UINT8(canbus_equipment_addr)
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
        MACRO_NVSREAD(mqtt_basic_cell_reporting);
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

        MACRO_NVSREAD(absorptiontimer);
        MACRO_NVSREAD(floatvoltage);
        MACRO_NVSREAD(floatvoltagetimer);
        MACRO_NVSREAD_UINT8(stateofchargeresumevalue);

        MACRO_NVSREADSTRING(homeassist_apikey);

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
    _myset->mqtt_basic_cell_reporting = false;

    _myset->canbusprotocol = CanBusProtocolEmulation::CANBUS_DISABLED;
    _myset->canbusinverter = CanBusInverter::INVERTER_GENERIC;

    _myset->canbus_equipment_addr = 0;
    _myset->canbusbaud=500;
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
    _myset->currentMonitoringDevice = CurrentMonitorDevice::DIYBMS_CURRENT_MON_MODBUS;

    _myset->currentMonitoring_shuntmv = 50;
    _myset->currentMonitoring_shuntmaxcur = 150;
    _myset->currentMonitoring_batterycapacity = 280;
    _myset->currentMonitoring_fullchargevolt = 5600;   // 56.00V
    _myset->currentMonitoring_tailcurrent = 1400;      // 14.00A
    _myset->currentMonitoring_chargeefficiency = 9950; // 99.50%
    _myset->currentMonitoring_shuntcal = 0;
    _myset->currentMonitoring_temperaturelimit = 80;
    _myset->currentMonitoring_overvoltagelimit = 5840;    // 58.4V
    _myset->currentMonitoring_undervoltagelimit = 4000;   // 40V
    _myset->currentMonitoring_overcurrentlimit = 15000;   // 150.00A
    _myset->currentMonitoring_undercurrentlimit = -15000; //-150.00A
    _myset->currentMonitoring_overpowerlimit = 5000;      // 5000W
    _myset->currentMonitoring_shunttempcoefficient = 15;
    _myset->currentMonitoring_tempcompenabled = false; // Disabled

    _myset->rs485baudrate = 19200;
    _myset->rs485databits = uart_word_length_t::UART_DATA_8_BITS;
    _myset->rs485parity = uart_parity_t::UART_PARITY_DISABLE;
    _myset->rs485stopbits = uart_stop_bits_t::UART_STOP_BITS_1;

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

    // Below makes reference to "float" this doesn't really exist in Lithium world
    // Once state of charge exceeds 99%, wait this many minutes until switching to float mode
    _myset->absorptiontimer = 60;
    // Voltage to drop to when in float mode. Scale 0.1 (3.3V * 16)
    _myset->floatvoltage = 528;
    // Wait this many minutes in float mode before disabling charge completely (6 hours)
    _myset->floatvoltagetimer = 6 * 60;
    // Once battery SoC drops below this value, resume normal charging operation
    _myset->stateofchargeresumevalue = 96;
}

/// @brief Save WIFI settings into FLASH NVS
/// @param wifi
void SaveWIFI(const wifi_eeprom_settings *wifi)
{
    const char *partname = "diybms-wifi";
    ESP_LOGI(TAG, "Save WIFI config");

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

        ESP_ERROR_CHECK(nvs_set_u32(nvs_handle, "IP", wifi->wifi_ip));
        ESP_ERROR_CHECK(nvs_set_u32(nvs_handle, "GATEWAY", wifi->wifi_gateway));
        ESP_ERROR_CHECK(nvs_set_u32(nvs_handle, "NETMASK", wifi->wifi_netmask));
        ESP_ERROR_CHECK(nvs_set_u32(nvs_handle, "DNS1", wifi->wifi_dns1));
        ESP_ERROR_CHECK(nvs_set_u32(nvs_handle, "DNS2", wifi->wifi_dns2));
        ESP_ERROR_CHECK(nvs_set_u8(nvs_handle, "MANUALCONFIG", wifi->manualConfig));
        nvs_close(nvs_handle);
    }
}

bool LoadWIFI(wifi_eeprom_settings *wifi)
{
    const char *partname = "diybms-wifi";

    bool result = false;
    wifi_eeprom_settings x;
    memset(&x, 0, sizeof(x));
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(partname, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle", esp_err_to_name(err));
        return false;
    }

    if (
        getString(nvs_handle, "SSID", &x.wifi_ssid[0], sizeof(x.wifi_ssid)) &&
        getString(nvs_handle, "PASS", &x.wifi_passphrase[0], sizeof(x.wifi_passphrase)) &&
        getSetting(nvs_handle, "IP", &x.wifi_ip) &&
        getSetting(nvs_handle, "GATEWAY", &x.wifi_gateway) &&
        getSetting(nvs_handle, "NETMASK", &x.wifi_netmask) &&
        getSetting(nvs_handle, "DNS1", &x.wifi_dns1) &&
        getSetting(nvs_handle, "DNS2", &x.wifi_dns2) &&
        getSetting(nvs_handle, "MANUALCONFIG", &x.manualConfig))
    {
        // Only return success if all values are retrieved, don't corrupt
        // external copy of wifi settings otherwise.
        memcpy(wifi, &x, sizeof(x));
        result = true;
    }

    nvs_close(nvs_handle);

    ESP_LOGI(TAG, "Load WIFI config from FLASH - return %u", result);

    ESP_LOGI(TAG, "IP=%u,GW=%u", x.wifi_ip, x.wifi_gateway);

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

    // 24hr max
    if (settings->absorptiontimer > 60 * 24)
    {
        settings->absorptiontimer = settings->absorptiontimer;
    }
    if (settings->floatvoltagetimer > 60 * 24)
    {
        settings->floatvoltagetimer = settings->floatvoltagetimer;
    }

    // Float voltage must be equal or below charge voltage
    if (settings->floatvoltage > settings->chargevolt)
    {
        settings->floatvoltage = settings->chargevolt;
    }

    // SoC cannot be over 99%
    if (settings->stateofchargeresumevalue > 99)
    {
        settings->stateofchargeresumevalue = settings->stateofchargeresumevalue;
    }
}

// Builds up a JSON document which mirrors the parameters inside "diybms_eeprom_settings"
void GenerateSettingsJSONDocument(DynamicJsonDocument *doc, diybms_eeprom_settings *settings)
{
    JsonObject root = doc->createNestedObject("diybms_settings");

    root[totalNumberOfBanks_JSONKEY] = settings->totalNumberOfBanks;
    root[totalNumberOfSeriesModules_JSONKEY] = settings->totalNumberOfSeriesModules;
    root[baudRate_JSONKEY] = settings->baudRate;
    root[interpacketgap_JSONKEY] = settings->interpacketgap;
    root[graph_voltagehigh_JSONKEY] = settings->graph_voltagehigh;
    root[graph_voltagelow_JSONKEY] = settings->graph_voltagelow;
    root[BypassOverTempShutdown_JSONKEY] = settings->BypassOverTempShutdown;
    root[BypassThresholdmV_JSONKEY] = settings->BypassThresholdmV;
    root[timeZone_JSONKEY] = settings->timeZone;
    root[minutesTimeZone_JSONKEY] = settings->minutesTimeZone;
    root[daylight_JSONKEY] = settings->daylight;
    root[ntpServer_JSONKEY] = settings->ntpServer;
    root[loggingEnabled_JSONKEY] = settings->loggingEnabled;
    root[loggingFrequencySeconds_JSONKEY] = settings->loggingFrequencySeconds;
    root[currentMonitoringEnabled_JSONKEY] = settings->currentMonitoringEnabled;
    root[currentMonitoringModBusAddress_JSONKEY] = settings->currentMonitoringModBusAddress;

    root[currentMonitoringDevice_JSONKEY] = (uint8_t)settings->currentMonitoringDevice;
    root[currentMonitoring_shuntmv_JSONKEY] = settings->currentMonitoring_shuntmv;
    root[currentMonitoring_shuntmaxcur_JSONKEY] = settings->currentMonitoring_shuntmaxcur;
    root[currentMonitoring_batterycapacity_JSONKEY] = settings->currentMonitoring_batterycapacity;
    root[currentMonitoring_fullchargevolt_JSONKEY] = settings->currentMonitoring_fullchargevolt;
    root[currentMonitoring_tailcurrent_JSONKEY] = settings->currentMonitoring_tailcurrent;
    root[currentMonitoring_chargeefficiency_JSONKEY] = settings->currentMonitoring_chargeefficiency;

    root[currentMonitoring_shuntcal_JSONKEY] = settings->currentMonitoring_shuntcal;
    root[currentMonitoring_temperaturelimit_JSONKEY] = settings->currentMonitoring_temperaturelimit;
    root[currentMonitoring_overvoltagelimit_JSONKEY] = settings->currentMonitoring_overvoltagelimit;
    root[currentMonitoring_undervoltagelimit_JSONKEY] = settings->currentMonitoring_undervoltagelimit;
    root[currentMonitoring_overcurrentlimit_JSONKEY] = settings->currentMonitoring_overcurrentlimit;
    root[currentMonitoring_undercurrentlimit_JSONKEY] = settings->currentMonitoring_undercurrentlimit;
    root[currentMonitoring_overpowerlimit_JSONKEY] = settings->currentMonitoring_overpowerlimit;
    root[currentMonitoring_shunttempcoefficient_JSONKEY] = settings->currentMonitoring_shunttempcoefficient;
    root[currentMonitoring_tempcompenabled_JSONKEY] = settings->currentMonitoring_tempcompenabled;

    root[rs485baudrate_JSONKEY] = settings->rs485baudrate;
    root[rs485databits_JSONKEY] = settings->rs485databits;
    root[rs485parity_JSONKEY] = settings->rs485parity;
    root[rs485stopbits_JSONKEY] = settings->rs485stopbits;
    root[language_JSONKEY] = settings->language;

    root[homeassist_apikey_JSONKEY]=settings->homeassist_apikey;

    JsonObject mqtt = root.createNestedObject("mqtt");
    mqtt[mqtt_enabled_JSONKEY] = settings->mqtt_enabled;
    mqtt[mqtt_basic_cell_reporting_JSONKEY] = settings->mqtt_basic_cell_reporting;
    mqtt[mqtt_uri_JSONKEY] = settings->mqtt_uri;
    mqtt[mqtt_topic_JSONKEY] = settings->mqtt_topic;
    mqtt[mqtt_username_JSONKEY] = settings->mqtt_username;
    mqtt[mqtt_password_JSONKEY] = settings->mqtt_password;

    JsonObject influxdb = root.createNestedObject("influxdb");
    influxdb[influxdb_enabled_JSONKEY] = settings->influxdb_enabled;
    influxdb[influxdb_apitoken_JSONKEY] = settings->influxdb_apitoken;
    influxdb[influxdb_databasebucket_JSONKEY] = settings->influxdb_databasebucket;
    influxdb[influxdb_orgid_JSONKEY] = settings->influxdb_orgid;
    influxdb[influxdb_serverurl_JSONKEY] = settings->influxdb_serverurl;
    influxdb[influxdb_loggingFreqSeconds_JSONKEY] = settings->influxdb_loggingFreqSeconds;

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

        if (rr <= MAXIMUM_RuleNumber)
        {
            // Map enum to string so when this file is re-imported we are not locked to specific index offsets
            // which may no longer map to the correct rule
            elementName = String(Rules::RuleTextDescription.at(rr).c_str());
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

    root[canbusprotocol_JSONKEY] = (uint8_t)settings->canbusprotocol;
    root[canbusinverter_JSONKEY] = (uint8_t)settings->canbusinverter;
    root[canbusbaud_JSONKEY] = settings->canbusbaud;
    root[canbus_equipment_addr_JSONKEY]=settings->canbus_equipment_addr;
    root[nominalbatcap_JSONKEY] = settings->nominalbatcap;

    root[chargevolt_JSONKEY] = settings->chargevolt;
    root[chargecurrent_JSONKEY] = settings->chargecurrent;
    root[dischargecurrent_JSONKEY] = settings->dischargecurrent;
    root[dischargevolt_JSONKEY] = settings->dischargevolt;

    root[chargetemplow_JSONKEY] = settings->chargetemplow;
    root[chargetemphigh_JSONKEY] = settings->chargetemphigh;
    root[dischargetemplow_JSONKEY] = settings->dischargetemplow;
    root[dischargetemphigh_JSONKEY] = settings->dischargetemphigh;
    root[stopchargebalance_JSONKEY] = settings->stopchargebalance;
    root[socoverride_JSONKEY] = settings->socoverride;
    root[socforcelow_JSONKEY] = settings->socforcelow;
    root[dynamiccharge_JSONKEY] = settings->dynamiccharge;
    root[preventdischarge_JSONKEY] = settings->preventdischarge;
    root[preventcharging_JSONKEY] = settings->preventcharging;
    root[cellminmv_JSONKEY] = settings->cellminmv;
    root[cellmaxmv_JSONKEY] = settings->cellmaxmv;
    root[kneemv_JSONKEY] = settings->kneemv;

    root[cellmaxspikemv_JSONKEY] = settings->cellmaxspikemv;
    root[sensitivity_JSONKEY] = settings->sensitivity;
    root[current_value1_JSONKEY] = settings->current_value1;
    root[current_value2_JSONKEY] = settings->current_value2;

    root[absorptiontimer_JSONKEY] = settings->absorptiontimer;
    root[floatvoltage_JSONKEY] = settings->floatvoltage;
    root[floatvoltagetimer_JSONKEY] = settings->floatvoltagetimer;
    root[stateofchargeresumevalue_JSONKEY] = settings->stateofchargeresumevalue;

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
    settings->baudRate = root[baudRate_JSONKEY];
    settings->interpacketgap = root[interpacketgap_JSONKEY];

    settings->graph_voltagehigh = root[graph_voltagehigh_JSONKEY];
    settings->graph_voltagelow = root[graph_voltagelow_JSONKEY];

    settings->BypassOverTempShutdown = root[BypassOverTempShutdown_JSONKEY];
    settings->BypassThresholdmV = root[BypassThresholdmV_JSONKEY];

    settings->timeZone = root[timeZone_JSONKEY];
    settings->minutesTimeZone = root[minutesTimeZone_JSONKEY];
    settings->daylight = root[daylight_JSONKEY];
    strncpy(settings->ntpServer, root[ntpServer_JSONKEY].as<String>().c_str(), sizeof(settings->ntpServer));

    settings->loggingEnabled = root[loggingEnabled_JSONKEY];
    settings->loggingFrequencySeconds = root[loggingFrequencySeconds_JSONKEY];

    settings->currentMonitoringEnabled = root[currentMonitoringEnabled_JSONKEY];
    settings->currentMonitoringModBusAddress = root[currentMonitoringModBusAddress_JSONKEY];

    settings->currentMonitoringDevice = (CurrentMonitorDevice)(uint8_t)root[currentMonitoringDevice_JSONKEY];
    settings->currentMonitoring_shuntmv = root[currentMonitoring_shuntmv_JSONKEY];
    settings->currentMonitoring_shuntmaxcur = root[currentMonitoring_shuntmaxcur_JSONKEY];
    settings->currentMonitoring_batterycapacity = root[currentMonitoring_batterycapacity_JSONKEY];
    settings->currentMonitoring_fullchargevolt = root[currentMonitoring_fullchargevolt_JSONKEY];
    settings->currentMonitoring_tailcurrent = root[currentMonitoring_tailcurrent_JSONKEY];
    settings->currentMonitoring_chargeefficiency = root[currentMonitoring_chargeefficiency_JSONKEY];
    settings->currentMonitoring_shuntcal = root[currentMonitoring_shuntcal_JSONKEY];
    settings->currentMonitoring_temperaturelimit = root[currentMonitoring_temperaturelimit_JSONKEY];
    settings->currentMonitoring_overvoltagelimit = root[currentMonitoring_overvoltagelimit_JSONKEY];
    settings->currentMonitoring_undervoltagelimit = root[currentMonitoring_undervoltagelimit_JSONKEY];
    settings->currentMonitoring_overcurrentlimit = root[currentMonitoring_overcurrentlimit_JSONKEY];
    settings->currentMonitoring_undercurrentlimit = root[currentMonitoring_undercurrentlimit_JSONKEY];
    settings->currentMonitoring_overpowerlimit = root[currentMonitoring_overpowerlimit_JSONKEY];
    settings->currentMonitoring_shunttempcoefficient = root[currentMonitoring_shunttempcoefficient_JSONKEY];
    settings->currentMonitoring_tempcompenabled = root[currentMonitoring_tempcompenabled_JSONKEY];

    settings->rs485baudrate = root[rs485baudrate_JSONKEY];
    settings->rs485databits = root[rs485databits_JSONKEY];
    settings->rs485parity = root[rs485parity_JSONKEY];
    settings->rs485stopbits = root[rs485stopbits_JSONKEY];

    strncpy(settings->language, root[language_JSONKEY].as<String>().c_str(), sizeof(settings->language));

    settings->canbusprotocol = (CanBusProtocolEmulation)root[canbusprotocol_JSONKEY];
    settings->canbusinverter = (CanBusInverter)root[canbusinverter_JSONKEY];
    settings->canbusbaud = root[canbusbaud_JSONKEY];
    settings->canbus_equipment_addr=root[canbus_equipment_addr_JSONKEY];
    settings->nominalbatcap = root[nominalbatcap_JSONKEY];
    settings->chargevolt = root[chargevolt_JSONKEY];
    settings->chargecurrent = root[chargecurrent_JSONKEY];
    settings->dischargecurrent = root[dischargecurrent_JSONKEY];
    settings->dischargevolt = root[dischargevolt_JSONKEY];
    settings->chargetemplow = root[chargetemplow_JSONKEY];
    settings->chargetemphigh = root[chargetemphigh_JSONKEY];
    settings->dischargetemplow = root[dischargetemplow_JSONKEY];
    settings->dischargetemphigh = root[dischargetemphigh_JSONKEY];
    settings->stopchargebalance = root[stopchargebalance_JSONKEY];
    settings->socoverride = root[socoverride_JSONKEY];
    settings->socforcelow = root[socforcelow_JSONKEY];
    settings->dynamiccharge = root[dynamiccharge_JSONKEY];
    settings->preventdischarge = root[preventdischarge_JSONKEY];
    settings->preventcharging = root[preventcharging_JSONKEY];
    settings->cellminmv = root[cellminmv_JSONKEY];
    settings->cellmaxmv = root[cellmaxmv_JSONKEY];
    settings->kneemv = root[kneemv_JSONKEY];
    settings->cellmaxspikemv = root[cellmaxspikemv_JSONKEY];
    settings->sensitivity = root[sensitivity_JSONKEY];
    settings->current_value1 = root[current_value1_JSONKEY];
    settings->current_value2 = root[current_value2_JSONKEY];

    settings->absorptiontimer=root[absorptiontimer_JSONKEY];
    settings->floatvoltage=root[floatvoltage_JSONKEY];
    settings->floatvoltagetimer=root[floatvoltagetimer_JSONKEY];
    settings->stateofchargeresumevalue=root[stateofchargeresumevalue_JSONKEY];

    strncpy(settings->homeassist_apikey, root[homeassist_apikey_JSONKEY].as<String>().c_str(), sizeof(settings->homeassist_apikey));

    JsonObject mqtt = root["mqtt"];
    if (!mqtt.isNull())
    {
        settings->mqtt_enabled = mqtt[mqtt_enabled_JSONKEY];
        settings->mqtt_basic_cell_reporting=mqtt[mqtt_basic_cell_reporting_JSONKEY];
        strncpy(settings->mqtt_uri, mqtt[mqtt_uri_JSONKEY].as<String>().c_str(), sizeof(settings->mqtt_uri));
        strncpy(settings->mqtt_topic, mqtt[mqtt_topic_JSONKEY].as<String>().c_str(), sizeof(settings->mqtt_topic));
        strncpy(settings->mqtt_username, mqtt[mqtt_username_JSONKEY].as<String>().c_str(), sizeof(settings->mqtt_username));
        strncpy(settings->mqtt_password, mqtt[mqtt_password_JSONKEY].as<String>().c_str(), sizeof(settings->mqtt_password));
    }

    JsonObject influxdb = root["influxdb"];
    if (!influxdb.isNull())
    {
        settings->influxdb_enabled = influxdb[influxdb_enabled_JSONKEY];
        strncpy(settings->influxdb_apitoken, influxdb[influxdb_apitoken_JSONKEY].as<String>().c_str(), sizeof(settings->influxdb_apitoken));
        strncpy(settings->influxdb_databasebucket, influxdb[influxdb_databasebucket_JSONKEY].as<String>().c_str(), sizeof(settings->influxdb_databasebucket));
        strncpy(settings->influxdb_orgid, influxdb[influxdb_orgid_JSONKEY].as<String>().c_str(), sizeof(settings->influxdb_orgid));
        strncpy(settings->influxdb_serverurl, influxdb[influxdb_serverurl_JSONKEY].as<String>().c_str(), sizeof(settings->influxdb_serverurl));
        settings->influxdb_loggingFreqSeconds = influxdb[influxdb_loggingFreqSeconds_JSONKEY];
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
            for (size_t rulenumber = 0; rulenumber <= MAXIMUM_RuleNumber; rulenumber++)
            {
                if (Rules::RuleTextDescription.at(rulenumber).compare(kv.key().c_str()) == 0)
                {
                    JsonVariant v = kv.value();
                    settings->rulevalue[rulenumber] = v["value"].as<int32_t>();
                    settings->rulehysteresis[rulenumber] = v["hysteresis"].as<int32_t>();
                    JsonArray states = v["state"].as<JsonArray>();

                    ESP_LOGI(TAG, "Matched to rule %u:%s, value=%i,hysteresis=%i", rulenumber, Rules::RuleTextDescription.at(rulenumber).c_str(), settings->rulevalue[rulenumber], settings->rulehysteresis[rulenumber]);

                    uint8_t i = 0;
                    for (JsonVariant x : states)
                    {
                        settings->rulerelaystate[rulenumber][i] = (RelayState)x.as<uint8_t>();
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
}