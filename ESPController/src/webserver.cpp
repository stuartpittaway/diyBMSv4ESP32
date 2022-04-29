
#define USE_ESP_IDF_LOG 1
static constexpr const char * const TAG = "diybms-web";

//Enable USE_WEBSOCKET_DEBUG_LOG to redirect console/debug serial port output
//to a websocket stream, viewable in the browser DEBUG window/console.
//Experimental feature
//#define USE_WEBSOCKET_DEBUG_LOG

#include "webserver.h"
#include "webserver_helper_funcs.h"
#include "webserver_json_requests.h"
#include "webserver_json_post.h"

#include <esp_log.h>
#include <stdarg.h>

httpd_handle_t _myserver;

// Shared buffer for all HTTP generated replies
char httpbuf[BUFSIZE];

void setNoStoreCacheControl(httpd_req_t *req)
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
    return String(mysettings.language);

  if (var == "GIT_VERSION")
    return String(GIT_VERSION);

  if (var == "COMPILE_DATE_TIME")
    return String(COMPILE_DATE_TIME);

  if (var == "graph_voltagehigh")
    return String(mysettings.graph_voltagehigh);

  if (var == "graph_voltagelow")
    return String(mysettings.graph_voltagelow);

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

esp_err_t SendFailure(httpd_req_t *req)
{
  return httpd_resp_send_500(req);
}

esp_err_t SendSuccess(httpd_req_t *req)
{
  httpd_resp_set_type(req, "application/json");
  setNoStoreCacheControl(req);

  StaticJsonDocument<100> doc;
  doc["success"] = true;
  int bufferused = 0;
  bufferused += serializeJson(doc, httpbuf, BUFSIZE);
  return httpd_resp_send(req, httpbuf, bufferused);
}

void saveConfiguration()
{
  Settings::WriteConfig("diybms", (char *)(&mysettings), sizeof(diybms_eeprom_settings));
}

// The main home page
esp_err_t default_htm_handler(httpd_req_t *req)
{
  httpd_resp_set_type(req, "text/html");
  setNoStoreCacheControl(req);
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

// Handle static files (images, javascript etc)
esp_err_t static_content_handler(httpd_req_t *req)
{

  const char *const mime_text_css = "text/css";
  const char *const mime_application_javascript = "application/javascript";
  const char *const mime_image_png = "image/png";
  const char *const mime_image_x_icon = "image/x-icon";

  enum enum_mimetype : uint8_t
  {
    text_css ,
    application_javascript ,
    image_x_icon ,
    image_png 
  };

  typedef struct
  {
    const uint8_t *resp;
    size_t resp_len;
    const char *etag;
    enum_mimetype mimetype;
  } WEBKIT_RESPONSE_ARGS;

  WEBKIT_RESPONSE_ARGS webkit_style_css_args = {file_style_css_gz, size_file_style_css_gz, etag_file_style_css_gz, text_css};
  WEBKIT_RESPONSE_ARGS webkit_pagecode_js_args = {file_pagecode_js_gz, size_file_pagecode_js_gz, etag_file_pagecode_js_gz, application_javascript};
  WEBKIT_RESPONSE_ARGS webkit_jquery_js_args = {file_jquery_js_gz, size_file_jquery_js_gz, etag_file_jquery_js_gz, application_javascript};
  WEBKIT_RESPONSE_ARGS webkit_notify_min_js_args = {file_notify_min_js_gz, size_file_notify_min_js_gz, etag_file_notify_min_js_gz, application_javascript};
  WEBKIT_RESPONSE_ARGS webkit_echarts_min_js_args = {file_echarts_min_js_gz, size_file_echarts_min_js_gz, etag_file_echarts_min_js_gz, application_javascript};
  WEBKIT_RESPONSE_ARGS webkit_lang_ru_js_args = {file_lang_ru_js_gz, size_file_lang_ru_js_gz, etag_file_lang_ru_js_gz, application_javascript};
  WEBKIT_RESPONSE_ARGS webkit_lang_hr_js_args = {file_lang_hr_js_gz, size_file_lang_hr_js_gz, etag_file_lang_hr_js_gz, application_javascript};
  WEBKIT_RESPONSE_ARGS webkit_lang_nl_js_args = {file_lang_nl_js_gz, size_file_lang_nl_js_gz, etag_file_lang_nl_js_gz, application_javascript};
  WEBKIT_RESPONSE_ARGS webkit_lang_pt_js_args = {file_lang_pt_js_gz, size_file_lang_pt_js_gz, etag_file_lang_pt_js_gz, application_javascript};
  WEBKIT_RESPONSE_ARGS webkit_lang_de_js_args = {file_lang_de_js_gz, size_file_lang_de_js_gz, etag_file_lang_de_js_gz, application_javascript};
  WEBKIT_RESPONSE_ARGS webkit_lang_es_js_args = {file_lang_es_js_gz, size_file_lang_es_js_gz, etag_file_lang_es_js_gz, application_javascript};
  WEBKIT_RESPONSE_ARGS webkit_lang_en_js_args = {file_lang_en_js_gz, size_file_lang_en_js_gz, etag_file_lang_en_js_gz, application_javascript};
  WEBKIT_RESPONSE_ARGS webkit_favicon_ico_args = {file_favicon_ico_gz, size_file_favicon_ico_gz, etag_file_favicon_ico_gz, image_x_icon};
  WEBKIT_RESPONSE_ARGS webkit_logo_png_args = {file_logo_png, size_file_logo_png, etag_file_logo_png, image_png};
  WEBKIT_RESPONSE_ARGS webkit_wait_png_args = {file_wait_png, size_file_wait_png, etag_file_wait_png, image_png};
  WEBKIT_RESPONSE_ARGS webkit_patron_png_args = {file_patron_png, size_file_patron_png, etag_file_patron_png, image_png};
  WEBKIT_RESPONSE_ARGS webkit_warning_png_args = {file_warning_png, size_file_warning_png, etag_file_warning_png, image_png};

  const char *uri_array[] = {
      "/style.css", "/pagecode.js", "/jquery.js", "/notify.min.js", "/echarts.min.js",
      "/lang_ru.js", "/lang_hr.js", "/lang_nl.js", "/lang_pt.js", "/lang_de.js", "/lang_es.js", "/lang_en.js",
      "/favicon.ico", "/logo.png", "/wait.png", "/patron.png", "/warning.png"};

  WEBKIT_RESPONSE_ARGS arguments[] = {
      webkit_style_css_args, webkit_pagecode_js_args, webkit_jquery_js_args, webkit_notify_min_js_args, webkit_echarts_min_js_args,
      webkit_lang_ru_js_args, webkit_lang_hr_js_args, webkit_lang_nl_js_args, webkit_lang_pt_js_args, webkit_lang_de_js_args, webkit_lang_es_js_args, webkit_lang_en_js_args, webkit_favicon_ico_args, webkit_logo_png_args, webkit_wait_png_args,
      webkit_patron_png_args, webkit_warning_png_args};

  // Sanity check arrays are the same size
  ESP_ERROR_CHECK(sizeof(arguments) / sizeof(WEBKIT_RESPONSE_ARGS) == sizeof(uri_array) / sizeof(unsigned int) ? ESP_OK : ESP_FAIL);

  for (size_t i = 0; i < sizeof(uri_array) / sizeof(unsigned int); i++)
  {
    if (strncmp(req->uri, uri_array[i], strlen(uri_array[i])) == 0)
    {
      WEBKIT_RESPONSE_ARGS args = arguments[i];

      switch (args.mimetype)
      {
      case application_javascript:
        httpd_resp_set_type(req, mime_application_javascript);
        break;
      case text_css:
        httpd_resp_set_type(req, mime_text_css);
        break;
      case image_x_icon:
        httpd_resp_set_type(req, mime_image_x_icon);
        break;
      case image_png:
        httpd_resp_set_type(req, mime_image_png);
        break;
      }

      char buffer[50];

      if (httpd_req_get_hdr_value_str(req, "If-None-Match", buffer, sizeof(buffer)) == ESP_OK)
      {
        // We have a value in the eTag header
        if (strncmp(buffer, args.etag, strlen(args.etag)) == 0)
        {
          ESP_LOGD(TAG, "Cached: %s", req->uri);
          // Matched
          httpd_resp_set_status(req, "304 Not Modified");
          httpd_resp_send(req, NULL, 0);
          return ESP_OK;
        }
      }

      if (args.mimetype != image_png)
      {
        // Everything is GZIP compressed except PNG images
        httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
      }

      ESP_LOGD(TAG, "Serve: %s", req->uri);
      SetCacheAndETag(req, args.etag);
      return httpd_resp_send(req, (const char *)args.resp, args.resp_len);
    }
  }

  ESP_LOGE(TAG, "Not found: %s", req->uri);

  return httpd_resp_send_404(req);
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

#ifdef USE_WEBSOCKET_DEBUG_LOG
// web socket handler
static esp_err_t ws_handler(httpd_req_t *req)
{
  // NO-OP
  return ESP_OK;
}
static const httpd_uri_t uri_ws_get = {.uri = "/ws", .method = HTTP_GET, .handler = ws_handler, .user_ctx = NULL, .is_websocket = true};
#endif

/* URI handler structure for GET /uri */
static const httpd_uri_t uri_root_get = {.uri = "/", .method = HTTP_GET, .handler = get_root_handler, .user_ctx = NULL};
static const httpd_uri_t uri_defaulthtm_get = {.uri = "/default.htm", .method = HTTP_GET, .handler = default_htm_handler, .user_ctx = NULL};
static const httpd_uri_t uri_api_get = {.uri = "/api/*", .method = HTTP_GET, .handler = api_handler, .user_ctx = NULL};
static const httpd_uri_t uri_download_get = {.uri = "/download", .method = HTTP_GET, .handler = content_handler_downloadfile, .user_ctx = NULL};
static const httpd_uri_t uri_save_data_post = {.uri = "/post/*", .method = HTTP_POST, .handler = save_data_handler, .user_ctx = NULL};
static const httpd_uri_t uri_static_content_get = {.uri = "*", .method = HTTP_GET, .handler = static_content_handler, .user_ctx = NULL};

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

#ifdef USE_WEBSOCKET_DEBUG_LOG
extern "C" int log_output_redirector(const char * format, va_list args)
{
  size_t fd_count = CONFIG_LWIP_MAX_LISTENING_TCP;
  int client_fds[CONFIG_LWIP_MAX_LISTENING_TCP] = {0};
  httpd_ws_frame_t ws_pkt = {};
  char log_buffer[64];
  char * temp = &log_buffer[0];
  va_list copy;

  va_copy(copy, args);
  int format_len = vsnprintf(temp, sizeof(log_buffer), format, copy);
  va_end(copy);
  if (format_len < 0)
  {
    va_end(args);
    return 0;
  }
  if (format_len >= sizeof(log_buffer))
  {
    temp = (char *)calloc(1, format_len + 1);
    if (temp == NULL)
    {
      va_end(args);
      return 0;
    }
    format_len = vsnprintf(temp, format_len + 1, format, args);
  }
  va_end(args);
  //Don't use printf - uses lots of stack space and causes task stack crash/growth.
  //printf(temp);
  fputs(temp, stdout);
  ws_pkt.len = format_len;
  ws_pkt.type = HTTPD_WS_TYPE_TEXT;
  ws_pkt.payload = (uint8_t *)(temp);

  // blast the message out to any connected websocket clients
  httpd_get_client_list(_myserver, &fd_count, client_fds);
  for (int idx = 0; idx < fd_count ; idx++)
  {
    if (httpd_ws_get_fd_info(_myserver, client_fds[idx]) == HTTPD_WS_CLIENT_WEBSOCKET)
    {
      httpd_ws_send_frame_async(_myserver, client_fds[idx], &ws_pkt);
    }
  }
  if (temp != log_buffer)
  {
    free(temp);
  }
  return format_len;
}
#endif


/* Function for starting the webserver */
httpd_handle_t start_webserver(void)
{
  /* Generate default configuration */
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();

  config.max_uri_handlers = 8;
  config.max_open_sockets = 5;
  config.max_resp_headers = 16;
  config.stack_size = 5000;
  config.uri_match_fn = httpd_uri_match_wildcard;

  /* Empty handle to esp_http_server */
  httpd_handle_t server = NULL;

  /* Start the httpd server */
  if (httpd_start(&server, &config) == ESP_OK)
  {
    /* Register URI handlers */
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_root_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_defaulthtm_get));

    // Web services/API
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_api_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_download_get));

    // Post services
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_save_data_post));

#ifdef USE_WEBSOCKET_DEBUG_LOG
    // Websocket
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_ws_get));
    esp_log_set_vprintf(log_output_redirector);
#endif

    // Catch all - this must be last in the list
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_static_content_get));
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
void StartServer()
{
  // Generate the cookie value
  setCookieValue();

  _myserver = start_webserver();
}
