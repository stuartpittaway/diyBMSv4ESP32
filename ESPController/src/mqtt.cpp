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
#include "string_utils.h"
#include <string>

bool mqttClient_connected = false;
esp_mqtt_client_handle_t mqtt_client = nullptr;

/// Utility function for publishing an MQTT message.
///
/// @param topic Topic to publish the message to.
/// @param payload Message payload to be published.
/// @param clear_payload When true @param payload will be cleared upon sending.
static inline void publish_message(std::string &topic, std::string &payload, bool clear_payload = true)
{
    static constexpr int MQTT_QUALITY_OF_SERVICE = 1;
    static constexpr int MQTT_RETAIN_MESSAGE = 0;

    if (mqtt_client && mqttClient_connected)
    {
        int id = esp_mqtt_client_publish(
            mqtt_client, topic.c_str(), payload.c_str(), payload.length(),
            MQTT_QUALITY_OF_SERVICE, MQTT_RETAIN_MESSAGE);
        ESP_LOGD(TAG, "Topic:%s, ID:%d, Length:%i", topic.c_str(), id, payload.length());
        ESP_LOGV(TAG, "Payload:%s", payload.c_str());
    }

    if (clear_payload)
    {
        payload.clear();
        payload.shrink_to_fit();
    }
}

/// Utility function returning the uptime of the ESP32 in seconds.
///
/// @return The uptime of the ESP32 in seconds.
static inline uint32_t uptime_in_seconds()
{
    uint64_t uptime_time_msec = esp_timer_get_time();
    uint64_t uptime_time_millis = uptime_time_msec / 1000;
    uint32_t uptime_time_seconds = uptime_time_millis / 1000;
    return uptime_time_seconds;
}

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
            if (ESP_ERROR_CHECK_WITHOUT_ABORT(esp_mqtt_client_start(mqtt_client)) != ESP_OK)
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

void GeneralStatusPayload(PacketRequestGenerator *prg, PacketReceiveProcessor *receiveProc, uint16_t requestq_count)
{
    ESP_LOGI(TAG, "General status payload");
    std::string status;
    status.reserve(400);
    status.append("{\"banks\":").append(std::to_string(mysettings.totalNumberOfBanks));
    status.append(",\"cells\":").append(std::to_string(mysettings.totalNumberOfSeriesModules));
    status.append(",\"uptime\":").append(std::to_string(uptime_in_seconds()));
    status.append(",\"commserr\":").append(std::to_string(receiveProc->HasCommsTimedOut() ? 1 : 0));
    status.append(",\"sent\":").append(std::to_string(prg->packetsGenerated));
    status.append(",\"received\":").append(std::to_string(receiveProc->packetsReceived));
    status.append(",\"badcrc\":").append(std::to_string(receiveProc->totalCRCErrors));
    status.append(",\"ignored\":").append(std::to_string(receiveProc->totalNotProcessedErrors));
    status.append(",\"oos\":").append(std::to_string(receiveProc->totalOutofSequenceErrors));
    status.append(",\"sendqlvl\":").append(std::to_string(requestq_count));
    status.append(",\"roundtrip\":").append(std::to_string(receiveProc->packetTimerMillisecond));
    status.append("}");

    std::string topic = mysettings.mqtt_topic;
    topic.append("/status");

    publish_message(topic, status);
}

void BankLevelInformation(Rules *rules)
{
    // Output bank level information (just voltage for now)
    for (int8_t bank = 0; bank < mysettings.totalNumberOfBanks; bank++)
    {
        ESP_LOGI(TAG, "Bank(%d) status payload", bank);
        std::string bank_status;
        bank_status.reserve(128);
        bank_status.append("{\"voltage\":").append(float_to_string(rules->packvoltage[bank] / 1000.0f)).append("}");

        std::string topic = mysettings.mqtt_topic;
        topic.append("/bank/").append(std::to_string(bank));
        publish_message(topic, bank_status);
    }
}

void RuleStatus(Rules *rules)
{
    ESP_LOGI(TAG, "Rule status payload");
    std::string rule_status;
    rule_status.reserve(128);
    rule_status.append("{");
    for (uint8_t i = 0; i < RELAY_RULES; i++)
    {
        rule_status.append("\"").append(std::to_string(i)).append("\":").append(std::to_string(rules->rule_outcome[i] ? 1 : 0));
        if (i < (RELAY_RULES - 1))
        {
            rule_status.append(",");
        }
    }
    rule_status.append("}");
    std::string topic = mysettings.mqtt_topic;
    topic.append("/rule");
    publish_message(topic, rule_status);
}

void OutputStatus(RelayState *previousRelayState)
{
    ESP_LOGI(TAG, "Outputs status payload");
    std::string relay_status;
    relay_status.reserve(128);
    relay_status.append("{");
    for (uint8_t i = 0; i < RELAY_TOTAL; i++)
    {
        relay_status.append("\"").append(std::to_string(i)).append("\":");
        relay_status.append(std::to_string((previousRelayState[i] == RelayState::RELAY_ON) ? 1 : 0));
        if (i < (RELAY_TOTAL - 1))
        {
            relay_status.append(",");
        }
    }
    relay_status.append("}");
    std::string topic = mysettings.mqtt_topic;
    topic.append("/output");
    publish_message(topic, relay_status);
}

void MQTTCurrentMonitoring(currentmonitoring_struct *currentMonitor)
{
    static int64_t lastcurrentMonitortimestamp = 0;

    ESP_LOGI(TAG, "MQTT Payload for current data");
    std::string status;
    status.reserve(256);
    status.append("{\"valid\":").append(std::to_string(currentMonitor->validReadings ? 1 : 0));

    if (currentMonitor->validReadings && currentMonitor->timestamp != lastcurrentMonitortimestamp)
    {
        // Send current monitor data if its valid and not sent before
        status.append(",\"voltage\":").append(float_to_string(currentMonitor->modbus.voltage));
        status.append(",\"current\":").append(float_to_string(currentMonitor->modbus.current));
        status.append(",\"power\":").append(float_to_string(currentMonitor->modbus.power));

        if (mysettings.currentMonitoringDevice == CurrentMonitorDevice::DIYBMS_CURRENT_MON)
        {
            status.append(",\"mAhIn\":").append(std::to_string(currentMonitor->modbus.milliamphour_in));
            status.append(",\"mAhOut\":").append(std::to_string(currentMonitor->modbus.milliamphour_out));
            status.append(",\"temperature\":").append(std::to_string(currentMonitor->modbus.temperature));
            status.append(",\"shuntmV\":").append(std::to_string(currentMonitor->modbus.shuntmV));
            status.append(",\"relayState\":").append(std::to_string(currentMonitor->RelayState ? 1 : 0));
            status.append(",\"soc\":").append(float_to_string(currentMonitor->stateofcharge));
        }
    }
    status.append("}");

    lastcurrentMonitortimestamp = currentMonitor->timestamp;

    std::string topic = mysettings.mqtt_topic;
    topic.append("/modbus_A").append(std::to_string(mysettings.currentMonitoringModBusAddress));
    publish_message(topic, status);
}

void MQTTCellData()
{
    // Send a few MQTT packets and keep track so we send the next batch on following calls
    static uint8_t mqttStartModule = 0;
    static constexpr uint8_t MAX_MODULES_PER_ITERATION = 8;

    if (mqttStartModule > (TotalNumberOfCells() - 1))
    {
        mqttStartModule = 0;
    }

    uint8_t counter = 0;
    uint8_t i = mqttStartModule;

    ESP_LOGI(TAG, "MQTT Payload for cell data");

    while (i < TotalNumberOfCells() && counter < MAX_MODULES_PER_ITERATION)
    {
        // Only send valid module data
        if (cmi[i].valid)
        {

            std::string status;
            std::string topic = mysettings.mqtt_topic;
            status.reserve(128);

            uint8_t bank = i / mysettings.totalNumberOfSeriesModules;
            uint8_t module = i - (bank * mysettings.totalNumberOfSeriesModules);

            status.append("{\"voltage\":").append(float_to_string(cmi[i].voltagemV / 1000.0f));
            status.append(",\"vMax\":").append(float_to_string(cmi[i].voltagemVMax / 1000.0f));
            status.append(",\"vMin\":").append(float_to_string(cmi[i].voltagemVMin / 1000.0f));
            status.append(",\"inttemp\":").append(std::to_string(cmi[i].internalTemp));
            status.append(",\"exttemp\":").append(std::to_string(cmi[i].externalTemp));
            status.append(",\"bypass\":").append(std::to_string(cmi[i].inBypass ? 1 : 0));
            status.append(",\"PWM\":").append(std::to_string((int)((float)cmi[i].PWMValue / (float)255.0 * 100)));
            status.append(",\"bypassT\":").append(std::to_string(cmi[i].bypassOverTemp ? 1 : 0));
            status.append(",\"bpc\":").append(std::to_string(cmi[i].badPacketCount));
            status.append(",\"mAh\":").append(std::to_string(cmi[i].BalanceCurrentCount));
            status.append("}");

            topic.append("/").append(std::to_string(bank)).append("/").append(std::to_string(module));
            publish_message(topic, status);
        }

        counter++;

        i++;
    }

    // After transmitting this many packets over MQTT, store our current state and exit the function.
    // this prevents flooding the ESP controllers wifi stack and potentially causing reboots/fatal exceptions
    mqttStartModule = i;
}

void mqtt1(currentmonitoring_struct *currentMonitor, Rules *rules)
{
    if (!mysettings.mqtt_enabled)
    {
        return;
    }

    if (!wifi_isconnected)
    {
        ESP_LOGE(TAG, "MQTT enabled, but WIFI not connected");
        return;
    }

    if (mqttClient_connected == false)
    {
        ESP_LOGE(TAG, "MQTT enabled, but not connected to broker");
        return;
    }

    // If the BMS is in error, stop sending MQTT packets for the data
    if (!rules->rule_outcome[Rule::BMSError])
    {
        MQTTCellData();
    }

    if (mysettings.currentMonitoringEnabled)
    {
        MQTTCurrentMonitoring(currentMonitor);
    }
}

void mqtt2(PacketReceiveProcessor *receiveProc,
           PacketRequestGenerator *prg,
           uint16_t requestq_count,
           Rules *rules,
           RelayState *previousRelayState)
{
    if (!mysettings.mqtt_enabled)
    {
        return;
    }
    if (!wifi_isconnected)
    {
        ESP_LOGE(TAG, "MQTT enabled, but WIFI not connected");
        return;
    }

    if (mqttClient_connected == false)
    {
        ESP_LOGE(TAG, "MQTT enabled, but not connected");
        return;
    }

    GeneralStatusPayload(prg, receiveProc, requestq_count);
    BankLevelInformation(rules);
    RuleStatus(rules);
    OutputStatus(previousRelayState);
}
