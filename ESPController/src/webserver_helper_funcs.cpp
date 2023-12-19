#define USE_ESP_IDF_LOG 1
static constexpr const char *const TAG = "diybms-webfuncs";

#include "webserver_helper_funcs.h"

char CookieValue[20 + 1];
// DIYBMS=a&zp!b4okcj$2$Dg*zUC; path=/; HttpOnly; SameSite=Strict
char cookie[45 + sizeof(CookieValue)];

void setCookie(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Set-Cookie", cookie);
}

void randomCharacters(char* value, int length) {
    // Pick random characters from this string (we could just use ASCII offset instead of this)
    // but this also avoids javascript escape characters like backslash and cookie escape chars like ; and %

    auto alphabet=std::string("!$*#@ABCDEFGHIJKLMNOPQRSTUVWXYZ1234567890abcdefghijklmnopqrstuvwxyz");
    // Leave NULL terminator on char array
    for (uint8_t x = 0; x < length; x++)
    {        
        // Random number between 0 and array length (minus null char)
        value[x] = alphabet.at(random(0, alphabet.length()));
    }
}

void setCookieValue()
{
    // We generate a unique number which is used in all following JSON requests
    // we use this as a simple method to avoid cross site scripting attacks
    // This MUST be done once the WIFI is switched on otherwise only PSEUDO random data is generated!!

    // ESP32 has inbuilt random number generator
    // https://techtutorialsx.com/2017/12/22/esp32-arduino-random-number-generation/

    memset(&CookieValue, 0, sizeof(CookieValue));
    randomCharacters(CookieValue,sizeof(CookieValue)-1);

    // Generate the full cookie string, as a HTTPONLY cookie, valid for this session only
    snprintf(cookie, sizeof(cookie), "DIYBMS=%s; path=/; HttpOnly; SameSite=Strict", CookieValue);
}

bool getPostDataIntoBuffer(httpd_req_t *req)
{
    /* Destination buffer for content of HTTP POST request.
     * httpd_req_recv() accepts char* only, but content could
     * as well be any binary data (needs type casting).
     * In case of string data, null termination will be absent, and
     * content length would give length of string */

    if (req->content_len > sizeof(httpbuf))
    {
        ESP_LOGE(TAG, "Buffer not large enough %u", req->content_len);
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
        // httpd_resp_send_408(req);
        // }
        return false;
    }

    // Ensure null terminated
    httpbuf[ret] = 0;

    // ESP_LOGD(TAG, "Post data %s", httpbuf);

    return true;
}

// Gets text value from a character buffer (as returned in HTTP request, query string etc)
bool GetTextFromKeyValue(const char *buffer, const char *key, char *text, size_t textLength, bool urlEncoded)
{
    if (httpd_query_key_value(buffer, key, text, textLength) == ESP_OK)
    {
        // ESP_LOGD(TAG, "Found: %s=%s", key, param);

        if (urlEncoded)
        {
            // Decode the incoming char array
            auto buf = (char *)malloc(textLength + 1);
            if (buf == nullptr)
            {
                ESP_LOGE(TAG, "Unable to malloc");
                return false;
            }

            strncpy(buf, text, textLength);
            // Overwrite existing char array
            url_decode(buf, text);
            free(buf);
        }

        return true;
    }

    ESP_LOGW(TAG, "Missing key named '%s'", key);

    return false;
}

bool GetKeyValue(const char *buffer, const char *key, uint8_t *value, bool urlEncoded)
{
    uint32_t uint32Variable;
    bool reply = GetKeyValue(buffer, key, &uint32Variable, urlEncoded);
    if (reply)
    {
        // Truncate down to uint8
        if (uint32Variable > 0xFF)
        {
            ESP_LOGW(TAG, "Overflow %i", uint32Variable);
        }
        *value = (uint8_t)uint32Variable;
    }
    return reply;
}

bool GetKeyValue(const char *buffer, const char *key, uint16_t *value, bool urlEncoded)
{
    uint32_t uint32Variable;
    bool reply = GetKeyValue(buffer, key, &uint32Variable, urlEncoded);
    if (reply)
    {
        // Truncate down to uint16
        if (uint32Variable > 0xFFFF)
        {
            ESP_LOGW(TAG, "Overflow %i", uint32Variable);
        }
        *value = (uint16_t)uint32Variable;
    }
    return reply;
}

bool GetKeyValue(const char *buffer, const char *key, int8_t *value, bool urlEncoded)
{
    int32_t int32Variable;
    bool reply = GetKeyValue(buffer, key, &int32Variable, urlEncoded);
    if (reply)
    {
        // Truncate down to int8
        // Check for overflow?
        if (int32Variable > 0xFF)
        {
            ESP_LOGW(TAG, "Overflow %i", int32Variable);
        }

        *value = (int8_t)int32Variable;
    }
    return reply;
}

bool GetKeyValue(const char *buffer, const char *key, int16_t *value, bool urlEncoded)
{
    int32_t int32Variable;
    bool reply = GetKeyValue(buffer, key, &int32Variable, urlEncoded);
    if (reply)
    {
        // Truncate down to int16
        *value = (int16_t)int32Variable;
    }
    return reply;
}

// Gets an SIGNED long value from a character buffer (as returned in HTTP request, query string etc)
bool GetKeyValue(const char *buffer, const char *key, int32_t *value, bool urlEncoded)
{
    char param[32];

    if (GetTextFromKeyValue(buffer, key, param, sizeof(param), urlEncoded))
    {
        // String to number conversion
        char **endptr = nullptr;
        long v = strtol(param, endptr, 10);

        *value = (int32_t)v;
        return true;
    }

    return false;
}

// Gets a FLOAT value from a character buffer (as returned in HTTP request, query string etc)
bool GetKeyValue(const char *buffer, const char *key, float *value, bool urlEncoded)
{
    char param[32];

    if (GetTextFromKeyValue(buffer, key, param, sizeof(param), urlEncoded))
    {
        // String to number conversion
        char **endptr = nullptr;
        float v = strtof(param, endptr);

        *value = v;
        return true;
    }

    return false;
}

// Gets an unsigned long value from a character buffer (as returned in HTTP request, query string etc)
bool GetKeyValue(const char *buffer, const char *key, uint32_t *value, bool urlEncoded)
{
    char param[32];

    if (GetTextFromKeyValue(buffer, key, param, sizeof(param), urlEncoded))
    {
        // String to number conversion
        char **endptr = nullptr;
        unsigned long v = strtoul(param, endptr, 10);

        *value = (uint32_t)v;
        return true;
    }

    return false;
}

bool GetKeyValue(const char *buffer, const char *key, bool *value, bool urlEncoded)
{
    char param[32];

    if (GetTextFromKeyValue(buffer, key, param, sizeof(param), urlEncoded))
    {
        // Compare strings
        bool v = false;

        if (strncmp(param, "on", 2) == 0)
        {
            v = true;
        }

        *value = v;
        return true;
    }

    return false;
}

/* Converts a hex character to its integer value */
char from_hex(char ch)
{
    return isdigit(ch) ? ch - '0' : tolower(ch) - 'a' + 10;
}

/* Returns a url-decoded version of str */
void url_decode(char *str, char *buf)
{
    // ESP_LOGD(TAG, "Encoded: %s", str);
    char *pstr = str;
    char *pbuf = buf;
    while (*pstr)
    {
        if (*pstr == '%')
        {
            if (pstr[1] && pstr[2])
            {
                *pbuf++ = from_hex(pstr[1]) << 4 | from_hex(pstr[2]);
                pstr += 2;
            }
        }
        else if (*pstr == '+')
        {
            *pbuf++ = ' ';
        }
        else
        {
            *pbuf++ = *pstr;
        }
        pstr++;
    }
    *pbuf = '\0';

    ESP_LOGD(TAG, "Decoded: %s", buf);
}

bool validateXSS(httpd_req_t *req)
{
    char requestcookie[sizeof(CookieValue)];

    const char *invalidcookie=R"({"error":"Invalid cookie"})";

    size_t cookielength = sizeof(CookieValue);

    esp_err_t result = httpd_req_get_cookie_val(req, "DIYBMS", requestcookie, &cookielength);

    if (result == ESP_OK)
    {
        // Compare received cookie to our expected cookie
        if (strncmp(CookieValue, requestcookie, sizeof(CookieValue)) == 0)
        {
            // All good, everything ok.
            return true;
        }

        // Cookie found and returned correctly (not truncated etc)
        ESP_LOGW(TAG, "Incorrect cookie rec %s", requestcookie);


        httpd_resp_send_err(req, httpd_err_code_t::HTTPD_400_BAD_REQUEST, invalidcookie);
        return false;
    }

    ESP_LOGE(TAG, "httpd_req_get_cookie_val (%s)", esp_err_to_name(result));

    // Fail - wrong cookie or not supplied etc.
    httpd_resp_send_err(req, httpd_err_code_t::HTTPD_400_BAD_REQUEST, invalidcookie);
    return false;
}

// Determine if the HTTPD request has a Content-Type of x-www-form-urlencoded
bool HasURLEncodedHeader(httpd_req_t *req)
{
    const char value[] = "application/x-www-form-urlencoded";
    char buffer[128];

    esp_err_t result = httpd_req_get_hdr_value_str(req, "Content-Type", buffer, sizeof(buffer));
    // ESP_LOGD(TAG, "esp_err_t=%i", result);

    if (result == ESP_OK)
    {
        // ESP_LOGD(TAG, "URLEncode=%s", buffer);
        //  Compare received value to our expected value (just the start of the string, minus null char)
        if (strncmp(value, buffer, sizeof(value) - 1) == 0)
        {
            // ESP_LOGD(TAG, "Header found and matches x-www-form-urlencoded");
            return true;
        }

        // Didn't match, so fall through to FALSE handler
    }

    return false;
}

bool validateXSSWithPOST(httpd_req_t *req, const char *postbuffer, bool urlencoded)
{
    // Need to validate POST variable as well...
    if (validateXSS(req))
    {
        // Must be larger than CookieValue as it could be URLEncoded
        char param[2 * sizeof(CookieValue)];
        if (httpd_query_key_value(postbuffer, "xss", param, sizeof(param)) == ESP_OK)
        {
            if (urlencoded)
            {
                // Decode the incoming char array
                char param_encoded[sizeof(param)];
                strncpy(param_encoded, param, sizeof(param));
                // Overwrite existing char array
                url_decode(param_encoded, param);
            }

            // Compare received cookie to our expected cookie
            if (strncmp(CookieValue, param, sizeof(CookieValue)) == 0)
            {
                // All good, everything ok.
                return true;
            }

            // Cookie found and returned correctly (not truncated etc)
            ESP_LOGW(TAG, "Incorrect POST cookie %s", param);

            // Failed POST XSS check
            httpd_resp_send_err(req, httpd_err_code_t::HTTPD_400_BAD_REQUEST, "Invalid cookie 3");
            return false;
        }
        else
        {
            ESP_LOGW(TAG, "xss query key returned not OK");
            // Failed POST XSS check
            httpd_resp_send_err(req, httpd_err_code_t::HTTPD_400_BAD_REQUEST, "Invalid cookie 4");
            return false;
        }
        // Failed POST XSS check
        httpd_resp_send_err(req, httpd_err_code_t::HTTPD_400_BAD_REQUEST, "Invalid cookie 5");
        return false;
    }

    // validateXSS has already sent httpd_resp_send_err...
    return false;
}