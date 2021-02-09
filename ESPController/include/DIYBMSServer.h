
#ifndef DIYBMSServer_H_
#define DIYBMSServer_H_

#include <Arduino.h>

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

#include <EEPROM.h>

#include "defines.h"
#include "Rules.h"
#include "settings.h"
#include "ArduinoJson.h"
#include "PacketRequestGenerator.h"
#include "PacketReceiveProcessor.h"

#include "Modbus.h"

#include "FS.h"
#include <LITTLEFS.h>
#include "SD.h"

#include "HAL_ESP32.h"
class DIYBMSServer
{
public:
    static void StartServer(AsyncWebServer *webserver,
                            diybms_eeprom_settings *mysettings,
                            fs::SDFS *sdcard,
                            PacketRequestGenerator *prg,
                            PacketReceiveProcessor *pktreceiveproc,
                            ControllerState *controlState,
                            Rules *rules,
                            ModbusInfo (*ModBus)[MODBUS_NUM],
                            ModbusVal (*ModBusVal)[MODBUS_NUM],
                            void (*sdcardaction_callback)(uint8_t action),
                            HAL_ESP32 *hal
                            );

    static void generateUUID();
    static void clearModuleValues(uint8_t module);

private:
    static AsyncWebServer *_myserver;
    static String UUIDString;

    //Pointers to other classes (not always a good idea in static classes)
    //static sdcard_info (*_sdcardcallback)();
    static fs::SDFS *_sdcard;
    static void (*_sdcardaction_callback)(uint8_t action);
    static PacketRequestGenerator *_prg;
    static PacketReceiveProcessor *_receiveProc;
    static diybms_eeprom_settings *_mysettings;
    static Rules *_rules;
    static ControllerState *_controlState;
    static HAL_ESP32 *_hal;

    static ModbusInfo (*_ModBus)[MODBUS_NUM];
    static ModbusVal (*_ModBusVal)[MODBUS_NUM];

    static void saveConfiguration()
    {
        Settings::WriteConfig("diybms", (char *)_mysettings, sizeof(diybms_eeprom_settings));
    }
    static void PrintStreamComma(AsyncResponseStream *response,const char *text, uint32_t value);
    static void PrintStream(AsyncResponseStream *response,const char *text, uint32_t value);
    static void PrintStreamCommaBoolean(AsyncResponseStream *response,const char *text, bool value);
    static void fileSystemListDirectory(AsyncResponseStream *response,fs::FS &fs, const char * dirname, uint8_t levels);

    static void handleNotFound(AsyncWebServerRequest *request);
    static void monitor2(AsyncWebServerRequest *request);
    static void monitor3(AsyncWebServerRequest *request);
    //static void monitor(AsyncWebServerRequest *request);
    static void modules(AsyncWebServerRequest *request);
    static void integration(AsyncWebServerRequest *request);
    static void identifyModule(AsyncWebServerRequest *request);
    static void GetRules(AsyncWebServerRequest *request);
    static String TemplateProcessor(const String &var);
    static bool validateXSS(AsyncWebServerRequest *request);
    static void SendSuccess(AsyncWebServerRequest *request);
    static void SendFailure(AsyncWebServerRequest *request);
    static void settings(AsyncWebServerRequest *request);
    static void resetCounters(AsyncWebServerRequest *request);
    static void handleRestartController(AsyncWebServerRequest *request);
    static void storage(AsyncWebServerRequest *request);
    static void avrstorage(AsyncWebServerRequest *request);

    static void modbus(AsyncWebServerRequest *request);
    static void modbusVal(AsyncWebServerRequest *request);
    static void saveModbus(AsyncWebServerRequest *request);

    static void downloadFile(AsyncWebServerRequest *request);
    static void saveSetting(AsyncWebServerRequest *request);
    static void saveInfluxDBSetting(AsyncWebServerRequest *request);
    static void saveMQTTSetting(AsyncWebServerRequest *request);
    static void saveGlobalSetting(AsyncWebServerRequest *request);
    static void saveBankConfiguration(AsyncWebServerRequest *request);
    static void saveRuleConfiguration(AsyncWebServerRequest *request);
    static void saveNTP(AsyncWebServerRequest *request);
    static void saveStorage(AsyncWebServerRequest *request);

    static void saveDisplaySetting(AsyncWebServerRequest *request);

    static void sdMount(AsyncWebServerRequest *request);
    static void sdUnmount(AsyncWebServerRequest *request);
    static void avrProgrammer(AsyncWebServerRequest *request);

    static String uuidToString(uint8_t *uuidLocation);
    static void SetCacheAndETagGzip(AsyncWebServerResponse *response, String ETag);
    static void SetCacheAndETag(AsyncWebServerResponse *response, String ETag);
};

//TODO: Remove this
extern bool _sd_card_installed;

#endif

