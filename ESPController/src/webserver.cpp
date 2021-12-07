
#include "webserver.h"

httpd_handle_t _myserver;
String UUIDString;
String UUIDStringLast2Chars;

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

void generateUUID()
{
  uint8_t uuidNumber[16]; // UUIDs in binary form are 16 bytes long

  // ESP32 has inbuilt random number generator
  // https://techtutorialsx.com/2017/12/22/esp32-arduino-random-number-generation/
  for (uint8_t x = 0; x < 16; x++)
  {
    uuidNumber[x] = random(0xFF);
  }

  UUIDString = uuidToString(uuidNumber);

  // 481efb3f-0400-0000-101f-fb3fd01efb3f
  UUIDStringLast2Chars = UUIDString.substring(34);
}

String uuidToString(uint8_t *uuidLocation)
{
  const char hexchars[] = "0123456789abcdef";
  String string = "";
  int i;
  for (i = 0; i < 16; i++)
  {
    if (i == 4)
      string += "-";
    if (i == 6)
      string += "-";
    if (i == 8)
      string += "-";
    if (i == 10)
      string += "-";
    uint8_t topDigit = uuidLocation[i] >> 4;
    uint8_t bottomDigit = uuidLocation[i] & 0x0f;
    // High hex digit
    string += hexchars[topDigit];
    // Low hex digit
    string += hexchars[bottomDigit];
  }

  return string;
}

void setCacheControl(httpd_req_t *req)
{
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
}

// The main home page
esp_err_t default_htm_handler(httpd_req_t *req)
{
  httpd_resp_set_type(req, "text/html");
  setCacheControl(req);
  return httpd_resp_send(req, (const char *)file_default_htm, size_file_default_htm);
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

// Handle static files which are NOT GZIP compressed
esp_err_t static_content_handler(httpd_req_t *req)
{
  WEBKIT_RESPONSE_ARGS *args = (WEBKIT_RESPONSE_ARGS *)(req->user_ctx);
  ESP_LOGD(TAG, "Web serve %s", req->uri);

  /*
    _myserver->on("/style.css", HTTP_GET,
                  [](AsyncWebServerRequest *request)
                  {
                    if (request->header("If-None-Match").equals(String(etag_file_style_css_gz)))
                    {
                      request->send(304);
                    }
                    else
                    {
                      AsyncWebServerResponse *response = request->beginResponse_P(200, "text/css", file_style_css_gz, size_file_style_css_gz);
                      SetCacheAndETagGzip(response, String(etag_file_style_css_gz));
                      request->send(response);
                    }
                  });
  */

  httpd_resp_set_type(req, args->mimetype);
  SetCacheAndETag(req, args->etag);
  return httpd_resp_send(req, (const char *)args->resp, args->resp_len);
}

// Handle static files which are already GZIP compressed
esp_err_t static_content_handler_gzipped(httpd_req_t *req)
{

  WEBKIT_RESPONSE_ARGS *args = (WEBKIT_RESPONSE_ARGS *)(req->user_ctx);

  ESP_LOGD(TAG, "Web serve %s", req->uri);

  /*
    _myserver->on("/style.css", HTTP_GET,
                  [](AsyncWebServerRequest *request)
                  {
                    if (request->header("If-None-Match").equals(String(etag_file_style_css_gz)))
                    {
                      request->send(304);
                    }
                    else
                    {
                      AsyncWebServerResponse *response = request->beginResponse_P(200, "text/css", file_style_css_gz, size_file_style_css_gz);
                      SetCacheAndETagGzip(response, String(etag_file_style_css_gz));
                      request->send(response);
                    }
                  });
  */

  httpd_resp_set_type(req, args->mimetype);
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
httpd_uri_t uri_root_get = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = get_root_handler,
    .user_ctx = NULL};

httpd_uri_t uri_defaulthtm_get = {
    .uri = "/default.htm",
    .method = HTTP_GET,
    .handler = default_htm_handler,
    .user_ctx = NULL};

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

/* URI handler structure for POST /uri */
httpd_uri_t uri_post = {
    .uri = "/uri",
    .method = HTTP_POST,
    .handler = post_handler,
    .user_ctx = NULL};

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

  /* Empty handle to esp_http_server */
  httpd_handle_t server = NULL;

  /* Start the httpd server */
  if (httpd_start(&server, &config) == ESP_OK)
  {
    /* Register URI handlers */
    httpd_register_uri_handler(server, &uri_root_get);
    httpd_register_uri_handler(server, &uri_defaulthtm_get);
    httpd_register_uri_handler(server, &uri_stylecss_get);
    // Images
    httpd_register_uri_handler(server, &uri_favicon_ico_get);
    httpd_register_uri_handler(server, &uri_warning_png_get);
    httpd_register_uri_handler(server, &uri_patron_png_get);
    httpd_register_uri_handler(server, &uri_logo_png_get);
    httpd_register_uri_handler(server, &uri_wait_png_get);

    // Javascript libs...
    httpd_register_uri_handler(server, &uri_pagecode_js_get);
    httpd_register_uri_handler(server, &uri_jquery_js_get);
    httpd_register_uri_handler(server, &uri_notify_min_js_get);
    httpd_register_uri_handler(server, &uri_echarts_min_js_get);
    httpd_register_uri_handler(server, &uri_lang_ru_js_get);
    httpd_register_uri_handler(server, &uri_lang_hr_js_get);
    httpd_register_uri_handler(server, &uri_lang_nl_js_get);
    httpd_register_uri_handler(server, &uri_lang_pt_js_get);
    httpd_register_uri_handler(server, &uri_lang_de_js_get);
    httpd_register_uri_handler(server, &uri_lang_es_js_get);
    httpd_register_uri_handler(server, &uri_lang_en_js_get);

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
