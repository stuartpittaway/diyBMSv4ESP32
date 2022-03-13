/*

 ____  ____  _  _  ____  __  __  ___    _  _  __
(  _ \(_  _)( \/ )(  _ \(  \/  )/ __)  ( \/ )/. |
 )(_) )_)(_  \  /  ) _ < )    ( \__ \   \  /(_  _)
(____/(____) (__) (____/(_/\/\_)(___/    \/   (_)

  (c) 2017 to 2022 Stuart Pittaway

  This is the code for the ESP32 controller - it talks to the V4.X cell modules over isolated serial bus

  This code runs on ESP32 DEVKIT-C and compiles with VS CODE and PLATFORM IO environment.

  Unless you are making code changes, please use the pre-compiled version from GITHUB instead.
*/

#define USE_ESP_IDF_LOG 1
static constexpr const char * const TAG = "diybms-influxdb";

#include "influxdb.h"
#include <esp_http_client.h>
#include <string>

/// Helper which encodes an integer type to a hex string.
///
/// @param w integer to convert
/// @param hex_len size of hex encoded string (automatically sized).
///
/// @return hex encoded string.
template <typename I> std::string to_hex(I w, size_t hex_len = sizeof(I) << 1)
{
    static const char* digits = "0123456789ABCDEF";
    std::string rc(hex_len,'0');
    for (size_t i = 0, j = (hex_len - 1) * 4 ; i < hex_len; ++i , j -= 4)
    {
        rc[i] = digits[(w >> j) & 0x0F];
    }
    return rc;
}

/// Helper which URL encodes a string as described in RFC-1738 sec. 2.2.
///
/// @param source is the string to be encoded.
///
/// @return the encoded string.
///
/// RFC: https://www.ietf.org/rfc/rfc1738.txt
static inline std::string url_encode(const std::string source)
{
  const std::string reserved_characters = "?#/:;+@&=";
  const std::string illegal_characters = "%<>{}|\\\"^`!*'()$,[]";
  std::string encoded = "";

  // reserve the size of the source string plus 25%, this space will be
  // reclaimed if the final string length is shorter.
  encoded.reserve(source.length() + (source.length() / 4));

  // process the source string character by character checking for any that
  // are outside the ASCII printable character range, in the reserve character
  // list or illegal character list. For accepted characters it will be added
  // to the encoded string directly, any that require encoding will be hex
  // encoded before being added to the encoded string.
  for (auto ch : source)
  {
    if (ch <= 0x20 || ch >= 0x7F ||
        reserved_characters.find(ch) != std::string::npos ||
        illegal_characters.find(ch) != std::string::npos)
    {
      // if it is outside the printable ASCII character *OR* is in either the
      // reserved or illegal characters we need to encode it as %HH.
      // NOTE: space will be encoded as "%20" and not as "+", either is an
      // acceptable option per the RFC.
      encoded.append("%").append(to_hex(ch));
    }
    else
    {
      encoded += ch;
    }
  }
  // shrink the buffer to the actual length
  encoded.shrink_to_fit();
  return encoded;
}

/// HTTP Client event handler
///
/// @param evt HTTP client event structure.
///
/// @return Always returns ESP_OK.
static esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    switch(evt->event_id)
    {
        case HTTP_EVENT_ERROR:
            ESP_LOGE(TAG, "HTTP Client Error encountered");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGI(TAG, "HTTP Client Connected!");
            break;
        case HTTP_EVENT_HEADERS_SENT:
            ESP_LOGV(TAG, "HTTP Client sent all request headers");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGV(TAG, "Header: key=%s, value=%s", evt->header_key, evt->header_value);
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGV(TAG, "HTTP Client data recevied: len=%d", evt->data_len);
            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGI(TAG, "HTTP Client finished");
            break;
         case HTTP_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "HTTP Client Disconnected!");
            break;
    }
    return ESP_OK;
}

/// Maximum number of modules to generate data for in a single invocation.
static constexpr uint8_t MAX_MODULES_PER_CALL = 16;

/// Generates and send module data to InfluxDB.
void influx_task_action()
{
    // Index of the first module to send data for, this is static to allow sending
    // modules in smaller batches (16)
    static uint8_t moduleIndex = 0;
    esp_http_client_config_t config = {};
    esp_http_client_handle_t http_client;
    std::string authtoken;
    std::string url;
    std::string module_data;
    module_data.reserve(768);

    // Generate data to send to InfluxDB.
    for (uint8_t remainingModules = MAX_MODULES_PER_CALL;
         moduleIndex < TotalNumberOfCells() && remainingModules > 0;
         remainingModules--, moduleIndex++)
    {
        // Only generate data for the module if it is valid.
        if (cmi[moduleIndex].valid)
        {
            char module_voltage[16] = {};
            uint8_t bank = moduleIndex / mysettings.totalNumberOfSeriesModules;
            uint8_t module_in_bank = moduleIndex - (bank * mysettings.totalNumberOfSeriesModules);
            std::string module_id = std::to_string(bank).append("_").append(std::to_string(module_in_bank));
            std::string module_internal_temp = std::to_string(cmi[moduleIndex].internalTemp);
            std::string module_external_temp = std::to_string(cmi[moduleIndex].externalTemp);
            std::string module_bypass = cmi[moduleIndex].inBypass ? "true" : "false";
            // NOTE: this is not using std::to_string since we want to truncate to 2 decimal places.
            snprintf(module_voltage, sizeof(module_voltage), "%.2f", cmi[moduleIndex].voltagemV / 1000.0f);

            ESP_LOGV(TAG, "Index:%d, bank:%d, module:%d, id:%s, voltage:%s, int-temp:%d, ext-temp:%s, bypass:%s",
                moduleIndex, bank, module_in_bank, module_id.c_str(), module_voltage,
                module_internal_temp.c_str(), module_external_temp.c_str(), module_bypass.c_str());

            //Data in LINE PROTOCOL format https://docs.influxdata.com/influxdb/v2.0/reference/syntax/line-protocol/
            module_data.append("cells, cell=").append(module_id);
            module_data.append(" v=").append(module_voltage);
            module_data.append(",i=").append(module_internal_temp).append("i");
            module_data.append(",e=").append(module_external_temp).append("i");
            module_data.append(",b=").append(module_bypass);
            module_data.append("\n");
        }
    }

    // Ensure moduleIndex remains within bounds.
    if (moduleIndex > (TotalNumberOfCells() - 1))
    {
        moduleIndex = 0;
    }

    module_data.shrink_to_fit();

    // If we did not generate any module data we can exit early.
    if (module_data.empty() || module_data.length() == 0)
    {
        ESP_LOGI(TAG, "No module data to send to InfluxDB");
        return;
    }

    authtoken.reserve(128);
    authtoken.append("Token ").append(mysettings.influxdb_apitoken);
    authtoken.shrink_to_fit();

    url.reserve(348);
    url.append(mysettings.influxdb_serverurl);
    url.append("?org=").append(url_encode(mysettings.influxdb_orgid));
    url.append("&bucker=").append(url_encode(mysettings.influxdb_databasebucket));
    url.shrink_to_fit();

    config.event_handler = http_event_handler;
    config.method = HTTP_METHOD_POST;
    config.url = url.c_str();

    // Initialize http client and prepare to process the request.
    http_client = esp_http_client_init(&config);

    // Set authorization header
    esp_http_client_set_header(http_client, "Authorization", authtoken.c_str());

    // Set Content-Encoding for the post payload
    esp_http_client_set_header(http_client, "Content-Type", "text/plain");

    // Add post data to the client.
    ESP_ERROR_CHECK(
        esp_http_client_set_post_field(http_client, module_data.c_str(), module_data.length()));

    // Process the http request
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_http_client_perform(http_client));
}