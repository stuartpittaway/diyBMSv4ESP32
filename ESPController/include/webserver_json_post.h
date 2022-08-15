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
extern Rules rules;
extern CardAction card_action;

extern avrprogramsettings _avrsettings;

extern void stopMqtt();
extern void ConfigureRS485();
extern void CurrentMonitorSetBasicSettings(uint16_t shuntmv, uint16_t shuntmaxcur, uint16_t batterycapacity, float fullchargevolt, float tailcurrent, float chargeefficiency);
extern void CurrentMonitorSetAdvancedSettings(currentmonitoring_struct newvalues);
extern void CurrentMonitorSetRelaySettings(currentmonitoring_struct newvalues);
extern void setCacheControl(httpd_req_t *req);
extern wifi_eeprom_settings _wificonfig;
extern void configureSNTP(long gmtOffset_sec, int daylightOffset_sec, const char *server1);
extern const char *RuleTextDescription[];
extern void DefaultConfiguration(diybms_eeprom_settings *_myset);
#endif
