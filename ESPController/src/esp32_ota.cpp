/*
 * DIYBMS ESP32 - controller firmware update from the web browser (OTA)
 *
 * POST /ota?xss=<key>   body: controller firmware image (app partition .bin)
 *
 * Replaces the previous simple /ota handler. Checks the session, the image size and the
 * image header, replies with JSON for the progress display in the web page and then
 * restarts the controller.
 *
 * Note: as before, this is not a secure update mechanism - anyone on the local network
 * with a valid web session can send new firmware to the controller.
 *
 * The new firmware is written to the free OTA partition (app0/app1). esp_ota_end()
 * validates the image (header, checksum, SHA256). Only if this succeeds is the new
 * partition set as boot partition. On a transfer or validation error the running
 * firmware stays active. Settings (NVS), WiFi and the file system are not touched.
 */
#define USE_ESP_IDF_LOG 1
static constexpr const char *const TAG = "diybms-ota";

#include <Arduino.h>
#include <ArduinoJson.h>
#include <esp_http_server.h>
#include <esp_ota_ops.h>
#include <esp_timer.h>
#include "esp32_ota.h"
#include "EmbeddedFiles_Defines.h"

extern char CookieValue[20 + 1];
extern void url_decode(char *encoded, char *decoded);
extern void suspendTasksDuringFirmwareUpdate();
extern void resumeTasksAfterFirmwareUpdateFailure();

// ESP32 application images start with this magic byte
static constexpr uint8_t ESP_IMAGE_MAGIC = 0xE9;

static volatile bool ota_running = false;

// Default: nothing blocks the update (can be overridden, see esp32_ota.h)
__attribute__((weak)) const char *ESP32Ota_BlockedReason()
{
    return nullptr;
}

// Uptime, running partition, version and maximum image size.
// The web page uses the uptime to detect that the controller has restarted.
esp_err_t ESP32Ota_StatusHandler(httpd_req_t *req)
{
    char buf[192];
    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_t *next = esp_ota_get_next_update_partition(nullptr);
    int n = snprintf(buf, sizeof(buf), "{\"uptime\":%lu,\"part\":\"%s\",\"ver\":\"%s\",\"max\":%lu}",
                     (unsigned long)(esp_timer_get_time() / 1000000ULL), running ? running->label : "?", GIT_VERSION,
                     (unsigned long)(next ? next->size : 0));
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, buf, n);
}

static esp_err_t ota_reply(httpd_req_t *req, bool ok, const char *msg, size_t bodyRead)
{
    // Discard the rest of the request body, otherwise the browser may not receive the reply
    char tmp[256];
    size_t left = req->content_len > bodyRead ? req->content_len - bodyRead : 0;
    int timeouts = 0;
    while (left > 0)
    {
        int r = httpd_req_recv(req, tmp, left < sizeof(tmp) ? left : sizeof(tmp));
        if (r == HTTPD_SOCK_ERR_TIMEOUT && ++timeouts < 5)
            continue;
        if (r <= 0)
            break;
        left -= r;
    }

    char buf[220];
    JsonDocument d;
    d["ok"] = ok ? 1 : 0;
    d["message"] = msg;
    size_t n = serializeJson(d, buf, sizeof(buf));
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "Connection", "close");
    return httpd_resp_send(req, buf, n);
}

// Restart directly in the calling handler (not from a separate task, which may never
// get scheduled once the other tasks have been suspended)
void ESP32Ota_RestartAfterReply(const char *reason)
{
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGW(TAG, "Restart (%s)", reason);
    esp_restart();
}

esp_err_t ESP32Ota_UploadHandler(httpd_req_t *req)
{
    // Check the session: cookie and key in the URL (the key can contain $ # @, so decode it)
    char cookie[sizeof(CookieValue) + 2] = {0};
    size_t cookielen = sizeof(cookie);
    char query[160] = {0};
    char enc[3 * sizeof(CookieValue) + 4] = {0};
    char xss[3 * sizeof(CookieValue) + 4] = {0};
    if (httpd_req_get_cookie_val(req, "DIYBMS", cookie, &cookielen) != ESP_OK ||
        strncmp(cookie, CookieValue, sizeof(CookieValue)) != 0 ||
        httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "xss", enc, sizeof(enc)) != ESP_OK)
    {
        return ota_reply(req, false, "Web page is out of date (controller has restarted) - refresh the page (F5) and select the file again", 0);
    }
    url_decode(enc, xss);
    if (strncmp(xss, CookieValue, sizeof(CookieValue)) != 0)
        return ota_reply(req, false, "Web page is out of date (controller has restarted) - refresh the page (F5) and select the file again", 0);

    const char *blocked = ESP32Ota_BlockedReason();
    if (blocked != nullptr)
        return ota_reply(req, false, blocked, 0);
    if (ota_running)
        return ota_reply(req, false, "A controller update is already running", 0);

    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_t *target = esp_ota_get_next_update_partition(nullptr);
    if (target == nullptr)
        return ota_reply(req, false, "No OTA partition found", 0);

    size_t len = req->content_len;
    if (len == 0 || len > target->size)
    {
        char m[140];
        snprintf(m, sizeof(m), "File size %u bytes does not fit the OTA partition (max. %u) - use the app image, not the full flash image",
                 (unsigned)len, (unsigned)target->size);
        return ota_reply(req, false, m, 0);
    }

    ota_running = true;
    ESP_LOGI(TAG, "OTA: %u bytes, running %s, writing %s", (unsigned)len, running ? running->label : "?", target->label);

    const size_t CHUNK = 4096;
    uint8_t *buf = (uint8_t *)malloc(CHUNK);
    if (buf == nullptr)
    {
        ota_running = false;
        return ota_reply(req, false, "Out of memory", 0);
    }

    esp_ota_handle_t h = 0;
    bool begun = false;
    bool suspended = false;
    size_t got = 0;
    int timeouts = 0;
    const char *err = nullptr;

    while (got < len)
    {
        int r = httpd_req_recv(req, (char *)buf, (len - got) < CHUNK ? (len - got) : CHUNK);
        if (r == HTTPD_SOCK_ERR_TIMEOUT && ++timeouts < 10)
            continue;
        if (r <= 0)
        {
            err = "Transfer aborted";
            break;
        }
        timeouts = 0;

        if (!begun)
        {
            if (buf[0] != ESP_IMAGE_MAGIC)
            {
                err = "Not an ESP32 application image (use the app image, not the full flash image)";
                got += r;
                break;
            }
            // Erase sector by sector while writing instead of erasing the whole partition up
            // front (that takes several seconds during which WiFi and the web server stall)
            if (esp_ota_begin(target, OTA_WITH_SEQUENTIAL_WRITES, &h) != ESP_OK)
            {
                err = "esp_ota_begin failed";
                got += r;
                break;
            }
            begun = true;
            // As before: stop display, SD logging, RS485/CAN and module polling during the update
            suspendTasksDuringFirmwareUpdate();
            suspended = true;
        }

        if (esp_ota_write(h, buf, r) != ESP_OK)
        {
            err = "Writing to flash failed";
            got += r;
            break;
        }
        got += r;
        // Give other tasks (WiFi, web server) some time
        vTaskDelay(1);
    }
    free(buf);

    if (err == nullptr && begun)
    {
        esp_err_t e = esp_ota_end(h);
        begun = false;
        if (e == ESP_ERR_OTA_VALIDATE_FAILED)
            err = "Firmware image validation failed (corrupt file?) - previous firmware stays active";
        else if (e != ESP_OK)
            err = "esp_ota_end failed - previous firmware stays active";
        else if (esp_ota_set_boot_partition(target) != ESP_OK)
            err = "Setting the boot partition failed - previous firmware stays active";
    }
    if (begun)
        esp_ota_abort(h);

    if (err != nullptr)
    {
        ota_running = false;
        if (suspended)
            resumeTasksAfterFirmwareUpdateFailure();
        ESP_LOGE(TAG, "OTA: %s", err);
        return ota_reply(req, false, err, got);
    }

    char m[120];
    snprintf(m, sizeof(m), "Firmware written (%u bytes, %s). Controller is restarting ...", (unsigned)len, target->label);
    ESP_LOGI(TAG, "%s", m);
    ota_reply(req, true, m, got);
    ESP32Ota_RestartAfterReply("new firmware");
}
