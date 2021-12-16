#include "webserver.h"
#include "webserver_json_post.h"
#include "webserver_helper_funcs.h"

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
    else
    {
        return SendFailure(req);
    }
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
    uint32_t tempVariable;

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