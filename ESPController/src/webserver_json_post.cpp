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

    // Need to validate POST variable XSS....
    if (!validateXSSWithPOST(req, httpbuf))
    {
        return ESP_FAIL;
    }

    uint8_t totalSeriesModules = 1;
    uint8_t totalBanks = 1;
    uint16_t baudrate = COMMS_BAUD_RATE;

    uint32_t tempVariable;

    if (GetUint32FromKeyValue(httpbuf, "totalSeriesModules", &tempVariable))
    {
        // Obviously could overflow
        totalSeriesModules = (uint8_t)tempVariable;

        if (GetUint32FromKeyValue(httpbuf, "totalBanks", &tempVariable))
        {
            // Obviously could overflow
            totalBanks = (uint8_t)tempVariable;

            if (GetUint32FromKeyValue(httpbuf, "baudrate", &tempVariable))
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

    // Need to validate POST variable XSS....
    if (!validateXSSWithPOST(req, httpbuf))
    {
        return ESP_FAIL;
    }

    uint32_t tempVariable;
    if (GetUint32FromKeyValue(httpbuf, "NTPZoneHour", &tempVariable))
    {
        _mysettings->timeZone = (uint8_t)tempVariable;
    }
    if (GetUint32FromKeyValue(httpbuf, "NTPZoneMin", &tempVariable))
    {
        _mysettings->minutesTimeZone = (uint8_t)tempVariable;
    }

    if (GetTextFromKeyValue(httpbuf, "NTPServer", _mysettings->ntpServer, sizeof(_mysettings->ntpServer)))
    {
    }

    // HTML Boolean value, so element is not POST'ed if FALSE/OFF
    if (GetBoolFromKeyValue(httpbuf, "NTPDST", &_mysettings->daylight))
    {
    }
    else
    {
        _mysettings->daylight = false;
    }

    saveConfiguration();

    return SendSuccess(req);
}
