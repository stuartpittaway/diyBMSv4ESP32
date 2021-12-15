#include "webserver.h"
#include "webserver_json_post.h"

bool getPostDataIntoBuffer(httpd_req_t *req)
{
    /* Destination buffer for content of HTTP POST request.
     * httpd_req_recv() accepts char* only, but content could
     * as well be any binary data (needs type casting).
     * In case of string data, null termination will be absent, and
     * content length would give length of string */

    if (req->content_len > sizeof(httpbuf))
    {
        ESP_LOGD(TAG, "Buffer not large enough %u", req->content_len);
        return false;
    }

    /* Truncate if content length larger than the buffer */
    // size_t recv_size = min(req->content_len, sizeof(httpbuf));

    int ret = httpd_req_recv(req, httpbuf, req->content_len);
    if (ret <= 0)
    { /* 0 return value indicates connection closed */
        /* Check if timeout occurred */
        // if (ret == HTTPD_SOCK_ERR_TIMEOUT)
        //{
        /* In case of timeout one can choose to retry calling
         * httpd_req_recv(), but to keep it simple, here we
         * respond with an HTTP 408 (Request Timeout) error */
        httpd_resp_send_408(req);
        //}
        /* In case of error, returning ESP_FAIL will
         * ensure that the underlying socket is closed */
        return false;
    }

    // Ensure null terminated
    httpbuf[ret] = 0;

    ESP_LOGD(TAG, "Post data %i bytes, %s", ret, httpbuf);

    return true;
}

// Gets an unsigned long value from a character buffer (as returned in HTTP request, query string etc)
bool GetUint32FromKeyValue(const char *buffer, const char *key, bool urlEncoded, uint32_t *value)
{
    char param[32];

    if (httpd_query_key_value(httpbuf, key, param, sizeof(param)) == ESP_OK)
    {
        //ESP_LOGD(TAG, "Found: %s=%s", key, param);

        if (urlEncoded)
        {
            // TODO: Implement URLDECODE function
        }

        // String to number conversion
        char **endptr;
        unsigned long v = strtoul(param, endptr, 10);

        *value = v;
        return true;
    }

    return false;
}

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

    bool urlencoded = HasURLEncodedHeader(req);

    uint8_t totalSeriesModules = 1;
    uint8_t totalBanks = 1;
    uint16_t baudrate = COMMS_BAUD_RATE;

    uint32_t tempVariable;

    if (GetUint32FromKeyValue(httpbuf, "totalSeriesModules", urlencoded, &tempVariable))
    {
        // Obviously could overflow
        totalSeriesModules = (uint8_t)tempVariable;

        if (GetUint32FromKeyValue(httpbuf, "totalBanks", urlencoded, &tempVariable))
        {
            // Obviously could overflow
            totalBanks = (uint8_t)tempVariable;

            if (GetUint32FromKeyValue(httpbuf, "baudrate", urlencoded, &tempVariable))
            {
                // Obviously could overflow
                baudrate = (uint8_t)tempVariable;

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
