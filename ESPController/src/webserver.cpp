
#include "webserver.h"
#include "webserver_json_requests.h"


httpd_handle_t _myserver;

// Pointers to other classes (not always a good idea in static classes)
// static sdcard_info (*_sdcardcallback)();
fs::SDFS *_sdcard;
void (*_sdcardaction_callback)(uint8_t action);
PacketRequestGenerator *_prg;
PacketReceiveProcessor *_receiveProc;
diybms_eeprom_settings *_mysettings;
Rules *_rules;
ControllerState *_controlState;
HAL_ESP32 *_hal;

char CookieValue[20 + 1];


// Shared buffer for all HTTP generated replies
char httpbuf[BUFSIZE];

// DIYBMS=a&zp!b4okcj$2$Dg*zUC; path=/; HttpOnly; SameSite=Strict
char cookie[45 + sizeof(CookieValue)];

void setCacheControl(httpd_req_t *req)
{
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
}

String TemplateProcessor(const String &var)
{
  if (var == "XSS_KEY")
    return String(CookieValue);

#if defined(ESP32)
  if (var == "PLATFORM")
    return String("ESP32");
#endif

  if (var == "LANGUAGE")
    return String(_mysettings->language);

  if (var == "GIT_VERSION")
    return String(GIT_VERSION);

  if (var == "COMPILE_DATE_TIME")
    return String(COMPILE_DATE_TIME);

  if (var == "graph_voltagehigh")
    return String(_mysettings->graph_voltagehigh);

  if (var == "graph_voltagelow")
    return String(_mysettings->graph_voltagelow);

  if (var == "integrity_file_jquery_js")
    return String(integrity_file_jquery_js);

  if (var == "noofseriesmodules")
    return String(maximum_controller_cell_modules);

  if (var == "maxnumberofbanks")
    return String(maximum_number_of_banks);

  return String();
}

int printBoolean(char *buffer, size_t bufferLen, const char *fieldName, boolean value, boolean addComma)
{
  return snprintf(buffer, bufferLen,
                  "\"%s\":%s%s",
                  fieldName,
                  value ? "true" : "false",
                  addComma ? "," : "");
}

int printBoolean(char *buffer, size_t bufferLen, const char *fieldName, boolean value)
{
  return printBoolean(buffer, bufferLen, fieldName, value, true);
}


esp_err_t SendSuccess(httpd_req_t *req)
{
  StaticJsonDocument<100> doc;
  doc["success"] = true;
  int bufferused = 0;
  bufferused += serializeJson(doc, httpbuf, BUFSIZE);
  return httpd_resp_send(req, httpbuf, bufferused);
}


// The main home page
esp_err_t default_htm_handler(httpd_req_t *req)
{
  httpd_resp_set_type(req, "text/html");
  setCacheControl(req);
  setCookie(req);

  char *file_pointer = (char *)file_default_htm;
  size_t max_len = size_file_default_htm;
  char *end_pointer = file_pointer + max_len;

  ESP_LOGI(TAG, "default.htm");

  char *p = file_pointer;

  while (p < end_pointer)
  {
    const char *templateCharacter = "%";

    // Find the next escape character (start of a template keyword)
    char *start_ptr = (char *)memchr(p, '%', end_pointer - p);

    if (start_ptr != NULL)
    {
      // Output to client up to the start of the template string
      httpd_resp_send_chunk(req, p, start_ptr - p);

      // Skip over template character %
      p = start_ptr + 1;

      // Find the end of the template
      char *end_ptr = (char *)memchr(p, '%', end_pointer - p);

      if (end_ptr != NULL)
      {
        // Length of the template keyword
        int paramNameLength = end_ptr - p;

        if (paramNameLength > 0)
        {
          char buf[50 + 1];
          memcpy(buf, p, paramNameLength);
          buf[paramNameLength] = 0;
          String paramName = String(buf);
          String templateValue = TemplateProcessor(paramName);

          ESP_LOGD(TAG, "Template %s = '%s'", paramName.c_str(), templateValue.c_str());

          // Output the template answer
          httpd_resp_sendstr_chunk(req, templateValue.c_str());
        }
        else
        {
          // Its an empty keyword %% - so just output a % character
          httpd_resp_sendstr_chunk(req, templateCharacter);
        }

        end_ptr++;
        p = end_ptr;
      }
      else
      {
        break;
      }
    }
    else
    {
      break;
    }
  }

  // output the last chunk
  if (p < end_pointer)
  {
    httpd_resp_send_chunk(req, p, end_pointer - p);
  }

  // Indicate last chunk (zero byte length)
  return httpd_resp_send_chunk(req, p, 0);
}

void SetCacheAndETag(httpd_req_t *req, const char *ETag)
{
  httpd_resp_set_hdr(req, "ETag", ETag);
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache, max-age=86400");
}

typedef struct
{
  const uint8_t *resp;
  size_t resp_len;
  const char *etag;
  const char *mimetype;
} WEBKIT_RESPONSE_ARGS;

const char *const text_css = "text/css";
const char *const application_javascript = "application/javascript";
const char *const image_png = "image/png";
const char *const image_x_icon = "image/x-icon";

void setCookieValue()
{
  // We generate a unique number which is used in all following JSON requests
  // we use this as a simple method to avoid cross site scripting attacks
  // This MUST be done once the WIFI is switched on otherwise only PSEUDO random data is generated!!

  // ESP32 has inbuilt random number generator
  // https://techtutorialsx.com/2017/12/22/esp32-arduino-random-number-generation/

  // Pick random characters from this string (we could just use ASCII offset instead of this)
  // but this also avoids javascript escape characters like backslash and cookie escape chars like ; and %
  char alphabet[] = "!$&*#@ABCDEFGHIJKLMNOPQRSTUVWXYZ1234567890abcdefghijklmnopqrstuvwxyz";

  memset(CookieValue, 0, sizeof(CookieValue));

  for (uint8_t x = 0; x < sizeof(CookieValue) - 1; x++)
  {
    CookieValue[x] = alphabet[random(sizeof(alphabet))];
  }

  // Generate the full cookie string, as a HTTPONLY cookie, valid for this session only
  snprintf(cookie, sizeof(cookie), "DIYBMS=%s; path=/; HttpOnly; SameSite=Strict", CookieValue);
}

void setCookie(httpd_req_t *req)
{
  httpd_resp_set_hdr(req, "Set-Cookie", cookie);
}

// Handle static files which are NOT GZIP compressed
esp_err_t static_content_handler(httpd_req_t *req)
{
  WEBKIT_RESPONSE_ARGS *args = (WEBKIT_RESPONSE_ARGS *)(req->user_ctx);

  httpd_resp_set_type(req, args->mimetype);

  // TODO: GET ETAGS WORKING AGAIN!!
  char buffer[50];

  if (httpd_req_get_hdr_value_str(req, "If-None-Match", buffer, sizeof(buffer)) == ESP_OK)
  {
    // We have a value in the eTag header

    if (strncmp(buffer, args->etag, strlen(args->etag)) == 0)
    {
      ESP_LOGD(TAG, "Cached response for %s", req->uri);
      // Matched
      httpd_resp_set_status(req, "304 Not Modified");
      httpd_resp_send(req, NULL, 0);
      return ESP_OK;
    }
  }

  ESP_LOGD(TAG, "Web serve %s", req->uri);
  SetCacheAndETag(req, args->etag);
  return httpd_resp_send(req, (const char *)args->resp, args->resp_len);
}

// Handle static files which are already GZIP compressed
esp_err_t static_content_handler_gzipped(httpd_req_t *req)
{
  WEBKIT_RESPONSE_ARGS *args = (WEBKIT_RESPONSE_ARGS *)(req->user_ctx);

  httpd_resp_set_type(req, args->mimetype);

  // TODO: GET ETAGS WORKING AGAIN!!
  char buffer[50];

  if (httpd_req_get_hdr_value_str(req, "If-None-Match", buffer, sizeof(buffer)) == ESP_OK)
  {
    // We have a value in the eTag header

    if (strncmp(buffer, args->etag, strlen(args->etag)) == 0)
    {
      ESP_LOGD(TAG, "Cached response for %s", req->uri);
      // Matched
      httpd_resp_set_status(req, "304 Not Modified");
      httpd_resp_send(req, NULL, 0);
      return ESP_OK;
    }
  }

  ESP_LOGD(TAG, "Web serve %s", req->uri);
  httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
  SetCacheAndETag(req, args->etag);
  return httpd_resp_send(req, (const char *)args->resp, args->resp_len);
}

/* Our URI handler function to be called during GET /uri request */
esp_err_t get_root_handler(httpd_req_t *req)
{
  httpd_resp_set_type(req, "text/html");
  httpd_resp_set_status(req, "301 Moved Permanently");
  httpd_resp_set_hdr(req, "Location", "/default.htm");
  httpd_resp_send(req, NULL, 0);
  return ESP_OK;
}

/* Our URI handler function to be called during POST /uri request */
esp_err_t post_handler(httpd_req_t *req)
{
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

  /* Send a simple response */
  const char resp[] = "URI POST Response";
  httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

/* URI handler structure for GET /uri */
httpd_uri_t uri_root_get = {.uri = "/", .method = HTTP_GET, .handler = get_root_handler, .user_ctx = NULL};

httpd_uri_t uri_defaulthtm_get = {.uri = "/default.htm", .method = HTTP_GET, .handler = default_htm_handler, .user_ctx = NULL};

WEBKIT_RESPONSE_ARGS webkit_style_css_args = {file_style_css_gz, size_file_style_css_gz, etag_file_style_css_gz, text_css};
httpd_uri_t uri_stylecss_get = {.uri = "/style.css", .method = HTTP_GET, .handler = static_content_handler_gzipped, .user_ctx = (void *)&webkit_style_css_args};

WEBKIT_RESPONSE_ARGS webkit_pagecode_js_args = {file_pagecode_js_gz, size_file_pagecode_js_gz, etag_file_pagecode_js_gz, application_javascript};
httpd_uri_t uri_pagecode_js_get = {.uri = "/pagecode.js", .method = HTTP_GET, .handler = static_content_handler_gzipped, .user_ctx = (void *)&webkit_pagecode_js_args};

WEBKIT_RESPONSE_ARGS webkit_jquery_js_args = {file_jquery_js_gz, size_file_jquery_js_gz, etag_file_jquery_js_gz, application_javascript};
httpd_uri_t uri_jquery_js_get = {.uri = "/jquery.js", .method = HTTP_GET, .handler = static_content_handler_gzipped, .user_ctx = (void *)&webkit_jquery_js_args};

WEBKIT_RESPONSE_ARGS webkit_notify_min_js_args = {file_notify_min_js_gz, size_file_notify_min_js_gz, etag_file_notify_min_js_gz, application_javascript};
httpd_uri_t uri_notify_min_js_get = {.uri = "/notify.min.js", .method = HTTP_GET, .handler = static_content_handler_gzipped, .user_ctx = (void *)&webkit_notify_min_js_args};

WEBKIT_RESPONSE_ARGS webkit_echarts_min_js_args = {file_echarts_min_js_gz, size_file_echarts_min_js_gz, etag_file_echarts_min_js_gz, application_javascript};
httpd_uri_t uri_echarts_min_js_get = {.uri = "/echarts.min.js", .method = HTTP_GET, .handler = static_content_handler_gzipped, .user_ctx = (void *)&webkit_echarts_min_js_args};

WEBKIT_RESPONSE_ARGS webkit_lang_ru_js_args = {file_lang_ru_js_gz, size_file_lang_ru_js_gz, etag_file_lang_ru_js_gz, application_javascript};
httpd_uri_t uri_lang_ru_js_get = {.uri = "/lang_ru.js", .method = HTTP_GET, .handler = static_content_handler_gzipped, .user_ctx = (void *)&webkit_lang_ru_js_args};

WEBKIT_RESPONSE_ARGS webkit_lang_hr_js_args = {file_lang_hr_js_gz, size_file_lang_hr_js_gz, etag_file_lang_hr_js_gz, application_javascript};
httpd_uri_t uri_lang_hr_js_get = {.uri = "/lang_hr.js", .method = HTTP_GET, .handler = static_content_handler_gzipped, .user_ctx = (void *)&webkit_lang_hr_js_args};

WEBKIT_RESPONSE_ARGS webkit_lang_nl_js_args = {file_lang_nl_js_gz, size_file_lang_nl_js_gz, etag_file_lang_nl_js_gz, application_javascript};
httpd_uri_t uri_lang_nl_js_get = {.uri = "/lang_nl.js", .method = HTTP_GET, .handler = static_content_handler_gzipped, .user_ctx = (void *)&webkit_lang_nl_js_args};

WEBKIT_RESPONSE_ARGS webkit_lang_pt_js_args = {file_lang_pt_js_gz, size_file_lang_pt_js_gz, etag_file_lang_pt_js_gz, application_javascript};
httpd_uri_t uri_lang_pt_js_get = {.uri = "/lang_pt.js", .method = HTTP_GET, .handler = static_content_handler_gzipped, .user_ctx = (void *)&webkit_lang_pt_js_args};

WEBKIT_RESPONSE_ARGS webkit_lang_de_js_args = {file_lang_de_js_gz, size_file_lang_de_js_gz, etag_file_lang_de_js_gz, application_javascript};
httpd_uri_t uri_lang_de_js_get = {.uri = "/lang_de.js", .method = HTTP_GET, .handler = static_content_handler_gzipped, .user_ctx = (void *)&webkit_lang_de_js_args};

WEBKIT_RESPONSE_ARGS webkit_lang_es_js_args = {file_lang_es_js_gz, size_file_lang_es_js_gz, etag_file_lang_es_js_gz, application_javascript};
httpd_uri_t uri_lang_es_js_get = {.uri = "/lang_es.js", .method = HTTP_GET, .handler = static_content_handler_gzipped, .user_ctx = (void *)&webkit_lang_es_js_args};

WEBKIT_RESPONSE_ARGS webkit_lang_en_js_args = {file_lang_en_js_gz, size_file_lang_en_js_gz, etag_file_lang_en_js_gz, application_javascript};
httpd_uri_t uri_lang_en_js_get = {.uri = "/lang_en.js", .method = HTTP_GET, .handler = static_content_handler_gzipped, .user_ctx = (void *)&webkit_lang_en_js_args};

WEBKIT_RESPONSE_ARGS webkit_favicon_ico_args = {file_favicon_ico_gz, size_file_favicon_ico_gz, etag_file_favicon_ico_gz, image_x_icon};
httpd_uri_t uri_favicon_ico_get = {.uri = "/favicon.ico", .method = HTTP_GET, .handler = static_content_handler_gzipped, .user_ctx = (void *)&webkit_favicon_ico_args};

WEBKIT_RESPONSE_ARGS webkit_logo_png_args = {file_logo_png, size_file_logo_png, etag_file_logo_png, image_png};
httpd_uri_t uri_logo_png_get = {.uri = "/logo.png", .method = HTTP_GET, .handler = static_content_handler, .user_ctx = (void *)&webkit_logo_png_args};

WEBKIT_RESPONSE_ARGS webkit_wait_png_args = {file_wait_png, size_file_wait_png, etag_file_wait_png, image_png};
httpd_uri_t uri_wait_png_get = {.uri = "/wait.png", .method = HTTP_GET, .handler = static_content_handler, .user_ctx = (void *)&webkit_wait_png_args};

WEBKIT_RESPONSE_ARGS webkit_patron_png_args = {file_patron_png, size_file_patron_png, etag_file_patron_png, image_png};
httpd_uri_t uri_patron_png_get = {.uri = "/patron.png", .method = HTTP_GET, .handler = static_content_handler, .user_ctx = (void *)&webkit_patron_png_args};

WEBKIT_RESPONSE_ARGS webkit_warning_png_args = {file_warning_png, size_file_warning_png, etag_file_warning_png, image_png};
httpd_uri_t uri_warning_png_get = {.uri = "/warning.png", .method = HTTP_GET, .handler = static_content_handler, .user_ctx = (void *)&webkit_warning_png_args};

httpd_uri_t uri_monitor2_json_get = {.uri = "/monitor2.json", .method = HTTP_GET, .handler = content_handler_monitor2, .user_ctx = NULL};
httpd_uri_t uri_monitor3_json_get = {.uri = "/monitor3.json", .method = HTTP_GET, .handler = content_handler_monitor3, .user_ctx = NULL};
httpd_uri_t uri_integration_json_get = {.uri = "/integration.json", .method = HTTP_GET, .handler = content_handler_integration, .user_ctx = NULL};
httpd_uri_t uri_settings_json_get = {.uri = "/settings.json", .method = HTTP_GET, .handler = content_handler_settings, .user_ctx = NULL};
httpd_uri_t uri_rules_json_get = {.uri = "/rules.json", .method = HTTP_GET, .handler = content_handler_rules, .user_ctx = NULL};
httpd_uri_t uri_getvictron_json_get = {.uri = "/getvictron.json", .method = HTTP_GET, .handler = content_handler_getvictron, .user_ctx = NULL};
httpd_uri_t uri_rs485settings_json_get = {.uri = "/rs485settings.json", .method = HTTP_GET, .handler = content_handler_rs485settings, .user_ctx = NULL};
httpd_uri_t uri_currentmonitor_json_get = {.uri = "/currentmonitor.json", .method = HTTP_GET, .handler = content_handler_currentmonitor, .user_ctx = NULL};
httpd_uri_t uri_avrstatus_json_get = {.uri = "/avrstatus.json", .method = HTTP_GET, .handler = content_handler_avrstatus, .user_ctx = NULL};
httpd_uri_t uri_modules_json_get = {.uri = "/modules.json", .method = HTTP_GET, .handler = content_handler_modules, .user_ctx = NULL};
httpd_uri_t uri_identifymodule_json_get = {.uri = "/identifyModule.json", .method = HTTP_GET, .handler = content_handler_identifymodule, .user_ctx = NULL};
httpd_uri_t uri_storage_json_get = {.uri = "/storage.json", .method = HTTP_GET, .handler = content_handler_storage, .user_ctx = NULL};
httpd_uri_t uri_avrstorage_json_get = {.uri = "/avrstorage.json", .method = HTTP_GET, .handler = content_handler_avrstorage, .user_ctx = NULL};

httpd_uri_t uri_download_get = {.uri = "/download", .method = HTTP_GET, .handler = content_handler_downloadfile, .user_ctx = NULL};

/* URI handler structure for POST /uri */
httpd_uri_t uri_post = {.uri = "/uri", .method = HTTP_POST, .handler = post_handler, .user_ctx = NULL};

void clearModuleValues(uint8_t module)
{
  cmi[module].valid = false;
  cmi[module].voltagemV = 0;
  cmi[module].voltagemVMin = 6000;
  cmi[module].voltagemVMax = 0;
  cmi[module].badPacketCount = 0;
  cmi[module].inBypass = false;
  cmi[module].bypassOverTemp = false;
  cmi[module].internalTemp = -40;
  cmi[module].externalTemp = -40;
}

/* Function for starting the webserver */
httpd_handle_t start_webserver(void)
{
  /* Generate default configuration */
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();

  config.max_uri_handlers = 48;
  config.max_open_sockets = 5;
  config.max_resp_headers = 16;
  config.stack_size = 4096;

  /* Empty handle to esp_http_server */
  httpd_handle_t server = NULL;

  /* Start the httpd server */
  if (httpd_start(&server, &config) == ESP_OK)
  {

    /* Register URI handlers */
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_root_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_defaulthtm_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_stylecss_get));
    // Images
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_favicon_ico_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_warning_png_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_patron_png_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_logo_png_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_wait_png_get));

    // Javascript libs...
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_pagecode_js_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_jquery_js_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_notify_min_js_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_echarts_min_js_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_lang_ru_js_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_lang_hr_js_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_lang_nl_js_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_lang_pt_js_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_lang_de_js_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_lang_es_js_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_lang_en_js_get));

    // Web services/API
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_monitor2_json_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_monitor3_json_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_integration_json_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_settings_json_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_rules_json_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_getvictron_json_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_rs485settings_json_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_currentmonitor_json_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_avrstatus_json_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_modules_json_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_identifymodule_json_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_storage_json_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_avrstorage_json_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_download_get));

    // httpd_register_uri_handler(server, &uri_post);
  }
  /* If server failed to start, handle will be NULL */
  return server;
}

/* Function for stopping the webserver */
void stop_webserver(httpd_handle_t server)
{
  if (server)
  {
    /* Stop the httpd server */
    httpd_stop(server);
  }
}

// Start Web Server (crazy amount of pointer params!)
void StartServer(diybms_eeprom_settings *mysettings,
                 fs::SDFS *sdcard,
                 PacketRequestGenerator *prg,
                 PacketReceiveProcessor *pktreceiveproc,
                 ControllerState *controlState,
                 Rules *rules,
                 void (*sdcardaction_callback)(uint8_t action),
                 HAL_ESP32 *hal)
{
  // Generate the cookie value
  setCookieValue();

  _myserver = start_webserver();
  _hal = hal;
  _prg = prg;
  _controlState = controlState;
  _rules = rules;
  _sdcard = sdcard;
  _mysettings = mysettings;
  _receiveProc = pktreceiveproc;
  _sdcardaction_callback = sdcardaction_callback;
}

/* Helper function to get a cookie value from a cookie string of the type "cookie1=val1; cookie2=val2" */
esp_err_t httpd_cookie_key_value(const char *cookie_str, const char *key, char *val, size_t *val_size)
{
  if (cookie_str == NULL || key == NULL || val == NULL)
  {
    return ESP_ERR_INVALID_ARG;
  }

  const char *cookie_ptr = cookie_str;
  const size_t buf_len = *val_size;
  size_t _val_size = *val_size;

  while (strlen(cookie_ptr))
  {
    /* Search for the '=' character. Else, it would mean
     * that the parameter is invalid */
    const char *val_ptr = strchr(cookie_ptr, '=');
    if (!val_ptr)
    {
      break;
    }
    size_t offset = val_ptr - cookie_ptr;

    /* If the key, does not match, continue searching.
     * Compare lengths first as key from cookie string is not
     * null terminated (has '=' in the end) */
    if ((offset != strlen(key)) || (strncasecmp(cookie_ptr, key, offset) != 0))
    {
      /* Get the name=val string. Multiple name=value pairs
       * are separated by '; ' */
      cookie_ptr = strchr(val_ptr, ' ');
      if (!cookie_ptr)
      {
        break;
      }
      cookie_ptr++;
      continue;
    }

    /* Locate start of next query */
    cookie_ptr = strchr(++val_ptr, ';');
    /* Or this could be the last query, in which
     * case get to the end of query string */
    if (!cookie_ptr)
    {
      cookie_ptr = val_ptr + strlen(val_ptr);
    }

    /* Update value length, including one byte for null */
    _val_size = cookie_ptr - val_ptr + 1;

    /* Copy value to the caller's buffer. */
    strlcpy(val, val_ptr, min(_val_size, buf_len));

    /* If buffer length is smaller than needed, return truncation error */
    if (buf_len < _val_size)
    {
      *val_size = _val_size;
      return ESP_ERR_HTTPD_RESULT_TRUNC;
    }
    /* Save amount of bytes copied to caller's buffer */
    *val_size = min(_val_size, buf_len);
    return ESP_OK;
  }
  ESP_LOGD(TAG, "cookie %s not found", key);
  return ESP_ERR_NOT_FOUND;
}

/* Get the value of a cookie from the request headers */
esp_err_t httpd_req_get_cookie_val(httpd_req_t *req, const char *cookie_name, char *val, size_t *val_size)
{
  esp_err_t ret;
  size_t hdr_len_cookie = httpd_req_get_hdr_value_len(req, "Cookie");
  char *cookie_str = NULL;

  if (hdr_len_cookie <= 0)
  {
    return ESP_ERR_NOT_FOUND;
  }
  cookie_str = (char *)malloc(hdr_len_cookie + 1);
  if (cookie_str == NULL)
  {
    ESP_LOGE(TAG, "Failed to allocate memory");
    return ESP_ERR_NO_MEM;
  }

  if (httpd_req_get_hdr_value_str(req, "Cookie", cookie_str, hdr_len_cookie + 1) != ESP_OK)
  {
    ESP_LOGW(TAG, "Cookie not found");
    free(cookie_str);
    return ESP_ERR_NOT_FOUND;
  }

  ret = httpd_cookie_key_value(cookie_str, cookie_name, val, val_size);
  free(cookie_str);
  return ret;
}

boolean validateXSS(httpd_req_t *req)
{
  char requestcookie[sizeof(CookieValue)];

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
    ESP_LOGW(TAG, "Incorrect cookie received %s", requestcookie);
  }

  // Fail - wrong cookie or not supplied etc.
  httpd_resp_send_err(req, httpd_err_code_t::HTTPD_400_BAD_REQUEST, "Invalid cookie");
  return false;
}
