
#include "webserver.h"
#include "webserver_helper_funcs.h"
#include "webserver_json_requests.h"
#include "webserver_json_post.h"

const char *const text_css = "text/css";
const char *const application_javascript = "application/javascript";
const char *const image_png = "image/png";
const char *const image_x_icon = "image/x-icon";

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

typedef struct
{
  const uint8_t *resp;
  size_t resp_len;
  const char *etag;
  const char *mimetype;
} WEBKIT_RESPONSE_ARGS;

// Handle static files which are NOT GZIP compressed
esp_err_t static_content_handler(httpd_req_t *req)
{
  WEBKIT_RESPONSE_ARGS *args = (WEBKIT_RESPONSE_ARGS *)(req->user_ctx);

  httpd_resp_set_type(req, args->mimetype);

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

/* URI handler structure for GET /uri */
httpd_uri_t uri_root_get = {.uri = "/", .method = HTTP_GET, .handler = get_root_handler, .user_ctx = NULL};


httpd_uri_t uri_api_get = {.uri = "/api/*", .method = HTTP_GET, .handler = api_handler, .user_ctx = NULL};

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
httpd_uri_t uri_victron_json_get = {.uri = "/victron.json", .method = HTTP_GET, .handler = content_handler_victron, .user_ctx = NULL};
httpd_uri_t uri_rs485settings_json_get = {.uri = "/rs485settings.json", .method = HTTP_GET, .handler = content_handler_rs485settings, .user_ctx = NULL};
httpd_uri_t uri_currentmonitor_json_get = {.uri = "/currentmonitor.json", .method = HTTP_GET, .handler = content_handler_currentmonitor, .user_ctx = NULL};
httpd_uri_t uri_avrstatus_json_get = {.uri = "/avrstatus.json", .method = HTTP_GET, .handler = content_handler_avrstatus, .user_ctx = NULL};
httpd_uri_t uri_modules_json_get = {.uri = "/modules.json", .method = HTTP_GET, .handler = content_handler_modules, .user_ctx = NULL};
httpd_uri_t uri_identifymodule_json_get = {.uri = "/identifyModule.json", .method = HTTP_GET, .handler = content_handler_identifymodule, .user_ctx = NULL};
httpd_uri_t uri_storage_json_get = {.uri = "/storage.json", .method = HTTP_GET, .handler = content_handler_storage, .user_ctx = NULL};
httpd_uri_t uri_avrstorage_json_get = {.uri = "/avrstorage.json", .method = HTTP_GET, .handler = content_handler_avrstorage, .user_ctx = NULL};

httpd_uri_t uri_download_get = {.uri = "/download", .method = HTTP_GET, .handler = content_handler_downloadfile, .user_ctx = NULL};

/* URI handler structure for POST /uri */
httpd_uri_t uri_savebankconfig_json_post = {.uri = "/savebankconfig.json", .method = HTTP_POST, .handler = post_savebankconfig_json_handler, .user_ctx = NULL};
httpd_uri_t uri_saventp_json_post = {.uri = "/saventp.json", .method = HTTP_POST, .handler = post_saventp_json_handler, .user_ctx = NULL};
httpd_uri_t uri_saveglobalsetting_json_post = {.uri = "/saveglobalsetting.json", .method = HTTP_POST, .handler = post_saveglobalsetting_json_handler, .user_ctx = NULL};
httpd_uri_t uri_savemqtt_json_post = {.uri = "/savemqtt.json", .method = HTTP_POST, .handler = post_savemqtt_json_handler, .user_ctx = NULL};
httpd_uri_t uri_saveinfluxdbsetting_json_post = {.uri = "/saveinfluxdb.json", .method = HTTP_POST, .handler = post_saveinfluxdbsetting_json_handler, .user_ctx = NULL};
httpd_uri_t uri_saveconfigurationtosdcard_json_post = {.uri = "/saveconfigtofile.json", .method = HTTP_POST, .handler = post_saveconfigurationtosdcard_json_handler, .user_ctx = NULL};
httpd_uri_t uri_savewificonfigtosdcard_json_post = {.uri = "/wificonfigtofile.json", .method = HTTP_POST, .handler = post_savewificonfigtosdcard_json_handler, .user_ctx = NULL};
httpd_uri_t uri_savesetting_json_post = {.uri = "/savesetting.json", .method = HTTP_POST, .handler = post_savesetting_json_handler, .user_ctx = NULL};
httpd_uri_t uri_restartcontroller_json_post = {.uri = "/restartcontroller.json", .method = HTTP_POST, .handler = post_restartcontroller_json_handler, .user_ctx = NULL};

httpd_uri_t uri_saverules_json_post = {.uri = "/saverules.json", .method = HTTP_POST, .handler = post_saverules_json_handler, .user_ctx = NULL};
httpd_uri_t uri_savedisplaysetting_json_post = {.uri = "/savedisplaysetting.json", .method = HTTP_POST, .handler = post_savedisplaysetting_json_handler, .user_ctx = NULL};
httpd_uri_t uri_savestorage_json_post = {.uri = "/savestorage.json", .method = HTTP_POST, .handler = post_savestorage_json_handler, .user_ctx = NULL};
httpd_uri_t uri_resetcounters_json_post = {.uri = "/resetcounters.json", .method = HTTP_POST, .handler = post_resetcounters_json_handler, .user_ctx = NULL};
httpd_uri_t uri_sdmount_json_post = {.uri = "/sdmount.json", .method = HTTP_POST, .handler = post_sdmount_json_handler, .user_ctx = NULL};
httpd_uri_t uri_sdunmount_json_post = {.uri = "/sdunmount.json", .method = HTTP_POST, .handler = post_sdunmount_json_handler, .user_ctx = NULL};
httpd_uri_t uri_enableavrprog_json_post = {.uri = "/enableavrprog.json", .method = HTTP_POST, .handler = post_enableavrprog_json_handler, .user_ctx = NULL};
httpd_uri_t uri_disableavrprog_json_post = {.uri = "/disableavrprog.json", .method = HTTP_POST, .handler = post_disableavrprog_json_handler, .user_ctx = NULL};
httpd_uri_t uri_avrprog_json_post = {.uri = "/avrprog.json", .method = HTTP_POST, .handler = post_avrprog_json_handler, .user_ctx = NULL};
httpd_uri_t uri_savers485settings_json_post = {.uri = "/savers485settings.json", .method = HTTP_POST, .handler = post_savers485settings_json_handler, .user_ctx = NULL};
httpd_uri_t uri_savecurrentmon_json_post = {.uri = "/savecurrentmon.json", .method = HTTP_POST, .handler = post_savecurrentmon_json_handler, .user_ctx = NULL};
httpd_uri_t uri_savecmbasic_json_post = {.uri = "/savecmbasic.json", .method = HTTP_POST, .handler = post_savecmbasic_json_handler, .user_ctx = NULL};
httpd_uri_t uri_savecmadvanced_json_post = {.uri = "/savecmadvanced.json", .method = HTTP_POST, .handler = post_savecmadvanced_json_handler, .user_ctx = NULL};
httpd_uri_t uri_savecmrelay_json_post = {.uri = "/savecmrelay.json", .method = HTTP_POST, .handler = post_savecmrelay_json_handler, .user_ctx = NULL};
httpd_uri_t uri_savevictron_json_post = {.uri = "/savevictron.json", .method = HTTP_POST, .handler = post_savevictron_json_handler, .user_ctx = NULL};

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

  config.max_uri_handlers = 58;
  config.max_open_sockets = 5;
  config.max_resp_headers = 16;
  config.stack_size = 4096;
  config.uri_match_fn = httpd_uri_match_wildcard;

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
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_api_get));

    //httpd_uri_match_wildcard()
    
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_monitor2_json_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_monitor3_json_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_integration_json_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_settings_json_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_rules_json_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_victron_json_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_rs485settings_json_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_currentmonitor_json_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_avrstatus_json_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_modules_json_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_identifymodule_json_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_storage_json_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_avrstorage_json_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_download_get));

    // Post services
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_savebankconfig_json_post));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_saventp_json_post));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_saveglobalsetting_json_post));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_savemqtt_json_post));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_saveinfluxdbsetting_json_post));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_saveconfigurationtosdcard_json_post));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_savewificonfigtosdcard_json_post));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_savesetting_json_post));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_restartcontroller_json_post));

    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_saverules_json_post));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_savedisplaysetting_json_post));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_savestorage_json_post));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_resetcounters_json_post));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_sdmount_json_post));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_sdunmount_json_post));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_enableavrprog_json_post));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_disableavrprog_json_post));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_avrprog_json_post));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_savers485settings_json_post));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_savecurrentmon_json_post));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_savecmbasic_json_post));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_savecmadvanced_json_post));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_savecmrelay_json_post));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri_savevictron_json_post));
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
