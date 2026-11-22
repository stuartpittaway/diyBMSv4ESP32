/*
 * DIYBMS ESP32 - controller firmware update from the web browser (OTA)
 *
 * POST /ota?xss=<key>      body: controller firmware image (app partition .bin)
 * GET  /api/otastatus      {"uptime":s,"part":"app0|app1","ver":"...","max":bytes}
 */
#ifndef DIYBMS_ESP32_OTA_H
#define DIYBMS_ESP32_OTA_H

#include <esp_http_server.h>

esp_err_t ESP32Ota_UploadHandler(httpd_req_t *req);
esp_err_t ESP32Ota_StatusHandler(httpd_req_t *req);

// Weak hook: other features (e.g. a running module firmware update) can override this
// to refuse a controller update while they are busy.
// Returns nullptr if the update may start, otherwise a message for the browser.
const char *ESP32Ota_BlockedReason();

// Call after the HTTP reply has been sent: wait a moment so the browser receives it,
// then restart. Shared by the firmware update and "Restart Controller".
[[noreturn]] void ESP32Ota_RestartAfterReply(const char *reason);

#endif
