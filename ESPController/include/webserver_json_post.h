#ifndef DIYBMSWebServer_Json_Post_H_
#define DIYBMSWebServer_Json_Post_H_

#include "settings.h"
#include "LittleFS.h"
#include "defines.h"
#include "PacketRequestGenerator.h"
#include "PacketReceiveProcessor.h"

esp_err_t save_data_handler(httpd_req_t *req);

extern diybms_eeprom_settings mysettings;
extern PacketRequestGenerator prg;
extern PacketReceiveProcessor receiveProc;
extern HAL_ESP32 hal;
extern fs::SDFS SD;

extern TaskHandle_t avrprog_task_handle;
extern uint32_t canbus_messages_received;
extern uint32_t canbus_messages_sent;
extern uint32_t canbus_messages_failed_sent;
extern uint32_t canbus_messages_received_error;
extern Rules rules;
extern CardAction card_action;

extern avrprogramsettings _avrsettings;
extern wifi_eeprom_settings _wificonfig;

extern void stopMqtt();
extern void ConfigureRS485();
extern bool CurrentMonitorSetSOC(float value);
extern bool CurrentMonitorResetDailyAmpHourCounters();
extern void CurrentMonitorSetBasicSettings(uint16_t shuntmv, uint16_t shuntmaxcur, uint16_t batterycapacity, float fullchargevolt, float tailcurrent, float chargeefficiency);
extern void CurrentMonitorSetAdvancedSettings(currentmonitoring_struct newvalues);
extern void CurrentMonitorSetRelaySettingsInternal(currentmonitoring_struct newvalues);
extern void CurrentMonitorSetRelaySettingsExternal(currentmonitoring_struct newvalues);
extern void setCacheControl(httpd_req_t *req);
extern void configureSNTP(long gmtOffset_sec, int daylightOffset_sec, const char *server1);
extern void DefaultConfiguration(diybms_eeprom_settings *_myset);
extern bool SaveWIFIJson(const wifi_eeprom_settings* setting);
extern void randomCharacters(char *value, int length);

esp_err_t post_savebankconfig_json_handler(httpd_req_t *req, bool urlEncoded);
esp_err_t post_saventp_json_handler(httpd_req_t *req, bool urlEncoded);
esp_err_t post_savemqtt_json_handler(httpd_req_t *req, bool urlEncoded);
esp_err_t post_saveglobalsetting_json_handler(httpd_req_t *req, bool urlEncoded);
esp_err_t post_restartcontroller_json_handler(httpd_req_t *req, bool urlEncoded);
esp_err_t post_saveinfluxdbsetting_json_handler(httpd_req_t *req, bool urlEncoded);
esp_err_t post_saveconfigurationtoflash_json_handler(httpd_req_t *req, bool urlEncoded);
esp_err_t post_savewificonfigtosdcard_json_handler(httpd_req_t *req, bool);
esp_err_t post_savesetting_json_handler(httpd_req_t *req, bool urlEncoded);
esp_err_t post_savestorage_json_handler(httpd_req_t *req, bool urlEncoded);
esp_err_t post_visibletiles_json_handler(httpd_req_t *req, bool urlEncoded);
esp_err_t post_savedisplaysetting_json_handler(httpd_req_t *req, bool urlEncoded);
esp_err_t post_resetcounters_json_handler(httpd_req_t *req, bool);
esp_err_t post_sdmount_json_handler(httpd_req_t *req, bool);
esp_err_t post_sdunmount_json_handler(httpd_req_t *req, bool);
esp_err_t post_enableavrprog_json_handler(httpd_req_t *req, bool);
esp_err_t post_disableavrprog_json_handler(httpd_req_t *req, bool);
esp_err_t post_savers485settings_json_handler(httpd_req_t *req, bool urlEncoded);
esp_err_t post_savechargeconfig_json_handler(httpd_req_t *req, bool urlEncoded);
esp_err_t post_savecmrelay_json_handler(httpd_req_t *req, bool urlEncoded);
esp_err_t post_setsoc_json_handler(httpd_req_t *req, bool urlEncoded);
esp_err_t post_resetdailyahcount_json_handler(httpd_req_t *req, bool);
esp_err_t post_savecmbasic_json_handler(httpd_req_t *req, bool urlEncoded);
esp_err_t post_savecmadvanced_json_handler(httpd_req_t *req, bool urlEncoded);
esp_err_t post_avrprog_json_handler(httpd_req_t *req, bool urlEncoded);
esp_err_t post_savecurrentmon_json_handler(httpd_req_t *req, bool urlEncoded);
esp_err_t post_saverules_json_handler(httpd_req_t *req, bool urlEncoded);
esp_err_t post_restoreconfig_json_handler(httpd_req_t *req, bool urlEncoded);
esp_err_t post_homeassistant_apikey_json_handler(httpd_req_t *req, bool urlEncoded);
esp_err_t save_data_handler(httpd_req_t *req);

#endif
