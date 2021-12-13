#include "webserver.h"
#include "webserver_json_post.h"

esp_err_t post_savebankconfig_json_handler(httpd_req_t *req)
{
    //Need to validate POST variable as well...
    if (!validateXSS(req))
    {
        return ESP_FAIL;
    }

    /* Destination buffer for content of HTTP POST request.
     * httpd_req_recv() accepts char* only, but content could
     * as well be any binary data (needs type casting).
     * In case of string data, null termination will be absent, and
     * content length would give length of string */
    char content[100];

    /* Truncate if content length larger than the buffer */
    size_t recv_size = min(req->content_len, sizeof(content));

    int ret = httpd_req_recv(req, content, recv_size);
    if (ret <= 0)
    { /* 0 return value indicates connection closed */
        /* Check if timeout occurred */
        if (ret == HTTPD_SOCK_ERR_TIMEOUT)
        {
            /* In case of timeout one can choose to retry calling
             * httpd_req_recv(), but to keep it simple, here we
             * respond with an HTTP 408 (Request Timeout) error */
            httpd_resp_send_408(req);
        }
        /* In case of error, returning ESP_FAIL will
         * ensure that the underlying socket is closed */
        return ESP_FAIL;
    }

    //Ensure null terminated
    content[ret]=0;

    ESP_LOGD(TAG,"Post %s",content);

    return SendSuccess(req);

    uint8_t totalSeriesModules = 1;
    uint8_t totalBanks = 1;
    uint16_t baudrate = COMMS_BAUD_RATE;

    /*
        if (request->hasParam("totalSeriesModules", true))
        {
            AsyncWebParameter *p1 = request->getParam("totalSeriesModules", true);
            totalSeriesModules = p1->value().toInt();
        }

        if (request->hasParam("totalBanks", true))
        {
            AsyncWebParameter *p1 = request->getParam("totalBanks", true);
            totalBanks = p1->value().toInt();
        }

        if (request->hasParam("baudrate", true))
        {
            AsyncWebParameter *p1 = request->getParam("baudrate", true);
            baudrate = p1->value().toInt();
        }

        if (totalSeriesModules * totalBanks <= maximum_controller_cell_modules)
        {
            _mysettings->totalNumberOfSeriesModules = totalSeriesModules;
            _mysettings->totalNumberOfBanks = totalBanks;
            _mysettings->baudRate = baudrate;
            saveConfiguration();

            SendSuccess(req);
        }
        else
        {
            // Error
            // httpd_resp_send_408(req);
        }
        */
}
