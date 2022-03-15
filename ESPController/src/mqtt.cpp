/*
 ____  ____  _  _  ____  __  __  ___    _  _  __
(  _ \(_  _)( \/ )(  _ \(  \/  )/ __)  ( \/ )/. |
 )(_) )_)(_  \  /  ) _ < )    ( \__ \   \  /(_  _)
(____/(____) (__) (____/(_/\/\_)(___/    \/   (_)

  (c) 2017 to 2022 Stuart Pittaway
*/

#define USE_ESP_IDF_LOG 1
static constexpr const char *const TAG = "diybms-mqtt";

#include "mqtt.h"

//#define MQTT_LOGGING

bool mqttClient_connected = false;
esp_mqtt_client_handle_t mqtt_client = nullptr;

static void mqtt_connected_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
    mqttClient_connected = true;
}

static void mqtt_disconnected_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
    mqttClient_connected = false;
}

static void mqtt_error_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    // ESP_LOGD(TAG, "Event base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;

    ESP_LOGE(TAG, "MQTT_EVENT_ERROR");
    if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT)
    {
        // log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
        // log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
        // log_error_if_nonzero("captured as transport's socket errno", event->error_handle->esp_transport_sock_errno);
        ESP_LOGE(TAG, "Last err no string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
    }
}
void stopMqtt()
{
    if (mqtt_client != nullptr)
    {
        ESP_LOGI(TAG, "Stopping MQTT client");
        mqttClient_connected = false;
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_mqtt_client_disconnect(mqtt_client));
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_mqtt_client_stop(mqtt_client));
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_mqtt_client_destroy(mqtt_client));
        mqtt_client = nullptr;
    }
}

// Connects to MQTT if required
void connectToMqtt()
{
    if (mysettings.mqtt_enabled && mqttClient_connected)
    {
        // Already connected and enabled
        return;
    }

    if (mysettings.mqtt_enabled)
    {
        stopMqtt();

        ESP_LOGI(TAG, "Connect MQTT");

        // Need to preset variables in esp_mqtt_client_config_t otherwise LoadProhibited errors
        esp_mqtt_client_config_t mqtt_cfg{
            .event_handle = NULL, .host = "", .uri = mysettings.mqtt_uri, .disable_auto_reconnect = false};

        mqtt_cfg.username = mysettings.mqtt_username;
        mqtt_cfg.password = mysettings.mqtt_password;

        mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
        if (mqtt_client != NULL)
        {
            ESP_ERROR_CHECK_WITHOUT_ABORT(esp_mqtt_client_register_event(mqtt_client, esp_mqtt_event_id_t::MQTT_EVENT_CONNECTED, mqtt_connected_handler, NULL));
            ESP_ERROR_CHECK_WITHOUT_ABORT(esp_mqtt_client_register_event(mqtt_client, esp_mqtt_event_id_t::MQTT_EVENT_DISCONNECTED, mqtt_disconnected_handler, NULL));
            ESP_ERROR_CHECK_WITHOUT_ABORT(esp_mqtt_client_register_event(mqtt_client, esp_mqtt_event_id_t::MQTT_EVENT_ERROR, mqtt_error_handler, NULL));
            if (esp_mqtt_client_start(mqtt_client) != ESP_OK)
            {
                ESP_LOGE(TAG, "esp_mqtt_client_start failed");
            }
        }
        else
        {
            ESP_LOGE(TAG, "mqtt_client returned NULL");
        }
    }
    else
    {
        stopMqtt();
    }
}

void mqtt2(PacketReceiveProcessor *receiveProc,
           PacketRequestGenerator *prg,
           uint16_t requestq_count,
           Rules *rules,
           RelayState *previousRelayState)
{

#define jsonbuffer_size 400
    if (mysettings.mqtt_enabled && mqttClient_connected)
    {
        ESP_LOGI(TAG, "MQTT 2");

        void *jsonbuffer = malloc(jsonbuffer_size);
        if (jsonbuffer == NULL)
        {
            ESP_LOGE(TAG, "Failed to malloc");
            return;
        }

        char topic[60];

        // DynamicJsonDocument doc(400);
        DynamicJsonDocument doc(jsonbuffer_size);
        JsonObject root = doc.to<JsonObject>();

        root["banks"] = mysettings.totalNumberOfBanks;
        root["cells"] = mysettings.totalNumberOfSeriesModules;
        root["uptime"] = millis() / 1000;
        root["commserr"] = receiveProc->HasCommsTimedOut() ? 1 : 0;
        root["sent"] = prg->packetsGenerated;
        root["received"] = receiveProc->packetsReceived;
        root["badcrc"] = receiveProc->totalCRCErrors;
        root["ignored"] = receiveProc->totalNotProcessedErrors;
        root["oos"] = receiveProc->totalOutofSequenceErrors;
        root["sendqlvl"] = requestq_count;
        root["roundtrip"] = receiveProc->packetTimerMillisecond;

        size_t len = serializeJson(doc, jsonbuffer, jsonbuffer_size);
        snprintf(topic, sizeof(topic), "%s/status", mysettings.mqtt_topic);

        int msg_id1 = esp_mqtt_client_publish(mqtt_client, topic, (char *)jsonbuffer, len, 1, 0);
        // MQTT_SKIP_PUBLISH_IF_DISCONNECTED
        ESP_LOGD(TAG, "mqtt msg_id=%d", msg_id1);

#if defined(MQTT_LOGGING)
        ESP_LOGD(TAG, "MQTT %s %s", topic, jsonbuffer);
#endif

        // Output bank level information (just voltage for now)
        for (int8_t bank = 0; bank < mysettings.totalNumberOfBanks; bank++)
        {
            doc.clear();
            doc["voltage"] = (float)rules->packvoltage[bank] / (float)1000.0;
            snprintf(topic, sizeof(topic), "%s/bank/%d", mysettings.mqtt_topic, bank);

            len = serializeJson(doc, jsonbuffer, jsonbuffer_size);

            int msg_id2 = esp_mqtt_client_publish(mqtt_client, topic, (char *)jsonbuffer, len, 1, 0);
            ESP_LOGD(TAG, "mqtt msg_id=%d", msg_id2);
#if defined(MQTT_LOGGING)
            ESP_LOGD(TAG, "MQTT %s %s", topic, jsonbuffer);
#endif
        }

        // Using Json for below reduced MQTT messages from 14 to 2. Could be combined into same json object too. But even better is status + event driven.
        doc.clear(); // Need to clear the json object for next message
        snprintf(topic, sizeof(topic), "%s/rule", mysettings.mqtt_topic);
        for (uint8_t i = 0; i < RELAY_RULES; i++)
        {
            doc[(String)i] = rules->rule_outcome[i] ? 1 : 0; // String conversion should be removed but just quick to get json format nice
        }
        len = serializeJson(doc, jsonbuffer, jsonbuffer_size);
#if defined(MQTT_LOGGING)
        ESP_LOGD(TAG, "MQTT %s %s", topic, jsonbuffer);
#endif
        int msg_id3 = esp_mqtt_client_publish(mqtt_client, topic, (char *)jsonbuffer, len, 1, 0);
        ESP_LOGD(TAG, "mqtt msg_id=%d", msg_id3);

        doc.clear(); // Need to clear the json object for next message
        snprintf(topic, sizeof(topic), "%s/output", mysettings.mqtt_topic);
        for (uint8_t i = 0; i < RELAY_TOTAL; i++)
        {
            doc[(String)i] = (previousRelayState[i] == RelayState::RELAY_ON) ? 1 : 0;
        }

        len = serializeJson(doc, jsonbuffer, jsonbuffer_size);
#if defined(MQTT_LOGGING)
        ESP_LOGD(TAG, "MQTT %s %s", topic, jsonbuffer);
#endif
        int msg_id4 = esp_mqtt_client_publish(mqtt_client, topic, (char *)jsonbuffer, len, 1, 0);
        ESP_LOGD(TAG, "mqtt msg_id=%d", msg_id4);

        // Unallocate the buffer
        free(jsonbuffer);

    } // end if
}

void mqtt1(currentmonitoring_struct *currentMonitor, Rules *rules)
{
    // Send a few MQTT packets and keep track so we send the next batch on following calls
    static uint8_t mqttStartModule = 0;
    static int64_t lastcurrentMonitortimestamp = 0;

#define jsonbuffer_size 400
    if (mysettings.mqtt_enabled && mqttClient_connected == false)
    {
        ESP_LOGE(TAG, "MQTT enabled, but not connected");
    }

    if (mysettings.mqtt_enabled && mqttClient_connected)
    {
        ESP_LOGI(TAG, "MQTT 1");
        char topic[60];
        // StaticJsonDocument<300> doc;
        DynamicJsonDocument doc(jsonbuffer_size);

        void *jsonbuffer = malloc(jsonbuffer_size);
        if (jsonbuffer == NULL)
        {
            ESP_LOGE(TAG, "Failed to malloc");
            return;
        }

        // If the BMS is in error, stop sending MQTT packets for the data
        if (!rules->rule_outcome[Rule::BMSError])
        {
            if (mqttStartModule > (TotalNumberOfCells() - 1))
            {
                mqttStartModule = 0;
            }

            uint8_t counter = 0;
            uint8_t i = mqttStartModule;

            while (i < TotalNumberOfCells() && counter < 8)
            {
                // ESP_LOGI(TAG, "Send MQTT for module %u", i);
                // Only send valid module data
                if (cmi[i].valid)
                {
                    uint8_t bank = i / mysettings.totalNumberOfSeriesModules;
                    uint8_t module = i - (bank * mysettings.totalNumberOfSeriesModules);

                    doc.clear();
                    doc["voltage"] = (float)cmi[i].voltagemV / (float)1000.0;
                    doc["vMax"] = (float)cmi[i].voltagemVMax / (float)1000.0;
                    doc["vMin"] = (float)cmi[i].voltagemVMin / (float)1000.0;
                    doc["inttemp"] = cmi[i].internalTemp;
                    doc["exttemp"] = cmi[i].externalTemp;
                    doc["bypass"] = cmi[i].inBypass ? 1 : 0;
                    doc["PWM"] = (int)((float)cmi[i].PWMValue / (float)255.0 * 100);
                    doc["bypassT"] = cmi[i].bypassOverTemp ? 1 : 0;
                    doc["bpc"] = cmi[i].badPacketCount;
                    doc["mAh"] = cmi[i].BalanceCurrentCount;
                    size_t len = serializeJson(doc, jsonbuffer, jsonbuffer_size);

                    snprintf(topic, sizeof(topic), "%s/%d/%d", mysettings.mqtt_topic, bank, module);

                    int msg_id1 = esp_mqtt_client_publish(mqtt_client, topic, (char *)jsonbuffer, len, 1, 0);
                    ESP_LOGD(TAG, "mqtt msg_id=%d", msg_id1);

#if defined(MQTT_LOGGING)
                    ESP_LOGI(TAG, "MQTT %s %s", topic, jsonbuffer);
#endif
                }

                counter++;

                i++;
            }

            // After transmitting this many packets over MQTT, store our current state and exit the function.
            // this prevents flooding the ESP controllers wifi stack and potentially causing reboots/fatal exceptions
            mqttStartModule = i;
        }

        if (mysettings.currentMonitoringEnabled)
        {
            // Send current monitor data
            doc.clear(); // Need to clear the json object for next message
            snprintf(topic, sizeof(topic), "%s/modbus/A%u", mysettings.mqtt_topic, mysettings.currentMonitoringModBusAddress);

            doc["valid"] = currentMonitor->validReadings ? 1 : 0;

            if (currentMonitor->validReadings && currentMonitor->timestamp != lastcurrentMonitortimestamp)
            {
                // Send current monitor data if its valid and not sent before
                doc["voltage"] = currentMonitor->modbus.voltage;
                doc["current"] = currentMonitor->modbus.current;
                doc["power"] = currentMonitor->modbus.power;
                if (mysettings.currentMonitoringDevice == CurrentMonitorDevice::DIYBMS_CURRENT_MON)
                {
                    doc["mAhIn"] = currentMonitor->modbus.milliamphour_in;
                    doc["mAhOut"] = currentMonitor->modbus.milliamphour_out;
                    doc["temperature"] = currentMonitor->modbus.temperature;
                    doc["shuntmV"] = currentMonitor->modbus.shuntmV;
                    doc["relayState"] = currentMonitor->RelayState ? 1 : 0;
                    doc["soc"] = currentMonitor->stateofcharge;
                }
            }

            lastcurrentMonitortimestamp = currentMonitor->timestamp;

            size_t len2 = serializeJson(doc, jsonbuffer, jsonbuffer_size);
#if defined(MQTT_LOGGING)
            ESP_LOGD(TAG, "MQTT %s %s", topic, jsonbuffer);
#endif
            int msg_id2 = esp_mqtt_client_publish(mqtt_client, topic, (char *)jsonbuffer, len2, 1, 0);
            ESP_LOGD(TAG, "mqtt msg_id=%d", msg_id2);
        }

        free(jsonbuffer);
    }
}
