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
uint16_t mqtt_error_connection_count = 0;
uint16_t mqtt_error_transport_count = 0;
uint16_t mqtt_connection_count = 0;
uint16_t mqtt_disconnection_count = 0;

bool checkMQTTReady()
{
    if (!mysettings.mqtt_enabled)
    {
        return false;
    }

    if (mqtt_client == nullptr)
    {
        ESP_LOGW(TAG, "MQTT enabled, but not yet init");
        return false;
    }
    if (!wifi_isconnected)
    {
        ESP_LOGW(TAG, "MQTT enabled, WIFI not connected");
        return false;
    }
    if (mqttClient_connected == false)
    {
        ESP_LOGW(TAG, "MQTT enabled, but not connected");
        return false;
    }

    return true;
}

/// Utility function for publishing an MQTT message.
///
/// @param topic Topic to publish the message to.
/// @param payload Message payload to be published.
/// @param clear_payload When true @param payload will be cleared upon sending.
static inline void publish_message(std::string const &topic, std::string &payload, bool clear_payload = true)
{
    static constexpr int MQTT_QUALITY_OF_SERVICE = 0;
    static constexpr int MQTT_RETAIN_MESSAGE = 1;

    if (mqtt_client != nullptr && mqttClient_connected)
    {
        int id = esp_mqtt_client_publish(mqtt_client, topic.c_str(),
                                         payload.c_str(), payload.length(), MQTT_QUALITY_OF_SERVICE, MQTT_RETAIN_MESSAGE);

        /*int id = esp_mqtt_client_enqueue(mqtt_client, topic.c_str(),
                                         payload.c_str(), payload.length(),
                                         MQTT_QUALITY_OF_SERVICE, MQTT_RETAIN_MESSAGE, true);
*/
        if (id < 0)
        {
            ESP_LOGE(TAG, "Topic:%s, failed publish", topic.c_str());
        }

        ESP_LOGD(TAG, "Topic:%s, ID:%d, Length:%i", topic.c_str(), id, payload.length());
        // ESP_LOGV(TAG, "Payload:%s", payload.c_str());
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
    return (uint32_t)(esp_timer_get_time() / (uint64_t)1000 / (uint64_t)1000);
}

static void mqtt_connected_handler(void *, esp_event_base_t, int32_t, void *)
{
    ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
    mqttClient_connected = true;
    mqtt_connection_count++;
}

static void mqtt_disconnected_handler(void *, esp_event_base_t, int32_t, void *)
{
    ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
    mqttClient_connected = false;
    mqtt_disconnection_count++;
}

static void mqtt_error_handler(void *, esp_event_base_t, int32_t, void *event_data)
{
    auto event = (esp_mqtt_event_handle_t)event_data;

    // ESP_LOGE(TAG, "MQTT_EVENT_ERROR type=%i",event->error_handle->error_type);
    if (event->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED)
    {
        mqtt_error_connection_count++;
        // esp_mqtt_connect_return_code_t reason for failure
        ESP_LOGE(TAG, "MQTT_ERROR_TYPE_CONNECTION_REFUSED code=%i", event->error_handle->connect_return_code);
    }

    if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT)
    {
        mqtt_error_transport_count++;
        // log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
        // log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
        // log_error_if_nonzero("captured as transport's socket errno", event->error_handle->esp_transport_sock_errno);
        ESP_LOGE(TAG, "ERROR_TYPE_TCP (%s)", strerror(event->error_handle->esp_transport_sock_errno));
    }
}

void stopMqtt()
{
    if (mqtt_client != nullptr)
    {
        ESP_LOGI(TAG, "Stopping MQTT client");
        mqttClient_connected = false;

        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_mqtt_client_stop(mqtt_client));
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_mqtt_client_destroy(mqtt_client));
        mqtt_client = nullptr;

        // Reset stats
        mqtt_error_connection_count = 0;
        mqtt_error_transport_count = 0;
        mqtt_connection_count = 0;
        mqtt_disconnection_count = 0;
    }
}



// Connects to MQTT if required
void connectToMqtt()
{
    ESP_LOGI(TAG, "MQTT counters: Err_Con=%u,Err_Trans=%u,Conn=%u,Disc=%u", mqtt_error_connection_count,
             mqtt_error_transport_count, mqtt_connection_count, mqtt_disconnection_count);

    if (mysettings.mqtt_enabled && mqtt_client == nullptr)
    {
        ESP_LOGI(TAG, "esp_mqtt_client_init");

        auto lwt = std::string(mysettings.mqtt_topic).append("/status");
        auto lwt_msg= std::string("{\"alive\":0}");

        // Need to preset variables in esp_mqtt_client_config_t otherwise LoadProhibited errors
        esp_mqtt_client_config_t mqtt_cfg{
            .event_handle = nullptr,
            .host = "",
            .uri = mysettings.mqtt_uri,
            .username = mysettings.mqtt_username,
            .password = mysettings.mqtt_password,
            .lwt_topic = lwt.c_str(),
            .lwt_msg = lwt_msg.c_str(), /* LWT message */
            .lwt_qos = 1,         /* LWT message qos */
            .lwt_retain = 1,      /* LWT retained message flag */
            .lwt_msg_len = (int)lwt_msg.length(),     /* LWT message length */

            // Reconnect if there server has a problem (or wrong IP/password etc.)
            .disable_auto_reconnect = false,
            .buffer_size = 512,
            // 30 seconds
            .reconnect_timeout_ms = 30000,
            .out_buffer_size = 1024,
            // 4 seconds
            .network_timeout_ms = 4000

        };

        mqtt_client = esp_mqtt_client_init(&mqtt_cfg);

        if (mqtt_client != nullptr)
        {
            ESP_ERROR_CHECK_WITHOUT_ABORT(esp_mqtt_client_register_event(mqtt_client, esp_mqtt_event_id_t::MQTT_EVENT_CONNECTED, mqtt_connected_handler, nullptr));
            ESP_ERROR_CHECK_WITHOUT_ABORT(esp_mqtt_client_register_event(mqtt_client, esp_mqtt_event_id_t::MQTT_EVENT_DISCONNECTED, mqtt_disconnected_handler, nullptr));
            ESP_ERROR_CHECK_WITHOUT_ABORT(esp_mqtt_client_register_event(mqtt_client, esp_mqtt_event_id_t::MQTT_EVENT_ERROR, mqtt_error_handler, nullptr));
            ESP_LOGI(TAG, "esp_mqtt_client_start");
            if (ESP_ERROR_CHECK_WITHOUT_ABORT(esp_mqtt_client_start(mqtt_client)) != ESP_OK)
            {
                ESP_LOGE(TAG, "esp_mqtt_client_start failed");
            }
        }
        else
        {
            ESP_LOGE(TAG, "esp_mqtt_client_init returned NULL");
        }
    }
}

void GeneralStatusPayload(const PacketRequestGenerator *prg, const PacketReceiveProcessor *receiveProc, uint16_t requestq_count, const Rules *rules)
{
    ESP_LOGI(TAG, "General status payload");
    std::string status;
    status.reserve(400);
    status.append("{\"banks\":")
        .append(std::to_string(mysettings.totalNumberOfBanks))
        .append(",\"cells\":")
        .append(std::to_string(mysettings.totalNumberOfSeriesModules))
        .append(",\"uptime\":")
        .append(std::to_string(uptime_in_seconds()))
        .append(",\"commserr\":")
        .append(std::to_string(receiveProc->HasCommsTimedOut() ? 1 : 0))
        .append(",\"sent\":")
        .append(std::to_string(prg->packetsGenerated))
        .append(",\"received\":")
        .append(std::to_string(receiveProc->packetsReceived))
        .append(",\"badcrc\":")
        .append(std::to_string(receiveProc->totalCRCErrors))
        .append(",\"ignored\":")
        .append(std::to_string(receiveProc->totalNotProcessedErrors))
        .append(",\"oos\":")
        .append(std::to_string(receiveProc->totalOutofSequenceErrors))
        .append(",\"sendqlvl\":")
        .append(std::to_string(requestq_count))
        .append(",\"roundtrip\":")
        .append(std::to_string(receiveProc->packetTimerMillisecond))
        .append(",\"alive\":1");

    if (mysettings.dynamiccharge)
    {
        status.append(",\"dynchargev\":")
            .append(float_to_string(((float)rules->DynamicChargeVoltage()) / 10.0F))
            .append(",\"dynchargec\":")
            .append(float_to_string(((float)rules->DynamicChargeCurrent()) / 10.0F));
    }

    status.append(",\"chgmode\":")
        .append(std::to_string((unsigned int)rules->getChargingMode()))
        .append(",\"chgtimer\":")
        .append(std::to_string(rules->getChargingTimerSecondsRemaining()));

    status.append("}");

    std::string topic = mysettings.mqtt_topic;
    topic.append("/status");

    publish_message(topic, status);
}

void BankLevelInformation(const Rules *rules)
{
    std::string bank_status;
    bank_status.reserve(64);
    // Output bank level information (just voltage for now)
    for (int8_t bank = 0; bank < mysettings.totalNumberOfBanks; bank++)
    {
        ESP_LOGI(TAG, "Bank %d status payload", bank);
        bank_status.clear();
        bank_status.append("{\"voltage\":")
            .append(float_to_string((float)(rules->bankvoltage.at(bank)) / 1000.0f))
            .append(",\"range\":")
            .append(std::to_string(rules->VoltageRangeInBank(bank)))
            .append("}");
        std::string topic = mysettings.mqtt_topic;
        topic.append("/bank/").append(std::to_string(bank));
        publish_message(topic, bank_status);
    }
}

void RuleStatus(const Rules *rules)
{
    ESP_LOGI(TAG, "Rule status payload");
    std::string rule_status;
    rule_status.reserve(128);
    rule_status.append("{");
    for (uint8_t i = 0; i < RELAY_RULES; i++)
    {
        rule_status.append("\"")
            .append(std::to_string(i))
            .append("\":")
            .append(std::to_string(rules->ruleOutcome((Rule)i) ? 1 : 0));
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

void OutputStatus(const RelayState *previousRelayState)
{
    ESP_LOGI(TAG, "Outputs status payload");
    std::string relay_status;
    relay_status.reserve(128);
    relay_status.append("{");
    for (uint8_t i = 0; i < RELAY_TOTAL; i++)
    {
        relay_status.append("\"")
            .append(std::to_string(i))
            .append("\":")
            .append(std::to_string((previousRelayState[i] == RelayState::RELAY_ON) ? 1 : 0));

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

void MQTTCurrentMonitoring(const currentmonitoring_struct *currentMonitor)
{
    static int64_t lastcurrentMonitortimestamp = 0;

    ESP_LOGI(TAG, "MQTT Payload for current data");
    std::string status;
    status.reserve(256);
    status.append("{\"valid\":").append(std::to_string(currentMonitor->validReadings ? 1 : 0));

    if (currentMonitor->validReadings && currentMonitor->timestamp != lastcurrentMonitortimestamp)
    {
        // Send current monitor data if its valid and not sent before
        status.append(",\"voltage\":").append(float_to_string(currentMonitor->modbus.voltage)).append(",\"current\":").append(float_to_string(currentMonitor->modbus.current)).append(",\"power\":").append(float_to_string(currentMonitor->modbus.power));

        if (mysettings.currentMonitoringDevice == CurrentMonitorDevice::DIYBMS_CURRENT_MON_MODBUS || mysettings.currentMonitoringDevice == CurrentMonitorDevice::DIYBMS_CURRENT_MON_INTERNAL)
        {
            status.append(",\"mAhIn\":").append(std::to_string(currentMonitor->modbus.milliamphour_in)).append(",\"mAhOut\":").append(std::to_string(currentMonitor->modbus.milliamphour_out)).append(",\"DailymAhIn\":").append(std::to_string(currentMonitor->modbus.daily_milliamphour_in)).append(",\"DailymAhOut\":").append(std::to_string(currentMonitor->modbus.daily_milliamphour_out)).append(",\"temperature\":").append(std::to_string(currentMonitor->modbus.temperature)).append(",\"relayState\":").append(std::to_string(currentMonitor->RelayState ? 1 : 0)).append(",\"soc\":").append(float_to_string(currentMonitor->stateofcharge));
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

    std::string status;
    status.reserve(128);

    while (i < TotalNumberOfCells() && counter < MAX_MODULES_PER_ITERATION)
    {
        // Only send valid module data
        if (cmi[i].valid)
        {

            uint8_t bank = i / mysettings.totalNumberOfSeriesModules;
            uint8_t m = i - (bank * mysettings.totalNumberOfSeriesModules);

            status.clear();
            status.append("{\"voltage\":").append(float_to_string(cmi[i].voltagemV / 1000.0f)).append(",\"exttemp\":").append(std::to_string(cmi[i].externalTemp));

            if (mysettings.mqtt_basic_cell_reporting == false)
            {
                status.append(",\"vMax\":").append(float_to_string(cmi[i].voltagemVMax / 1000.0f)).append(",\"vMin\":").append(float_to_string(cmi[i].voltagemVMin / 1000.0f)).append(",\"inttemp\":").append(std::to_string(cmi[i].internalTemp)).append(",\"bypass\":").append(std::to_string(cmi[i].inBypass ? 1 : 0)).append(",\"PWM\":").append(std::to_string((int)((float)cmi[i].PWMValue / (float)255.0 * 100))).append(",\"bypassT\":").append(std::to_string(cmi[i].bypassOverTemp ? 1 : 0)).append(",\"bpc\":").append(std::to_string(cmi[i].badPacketCount)).append(",\"mAh\":").append(std::to_string(cmi[i].BalanceCurrentCount));
            }

            status.append("}");

            std::string topic = mysettings.mqtt_topic;
            topic.append("/").append(std::to_string(bank)).append("/").append(std::to_string(m));
            publish_message(topic, status);
        }

        counter++;

        i++;
    }

    // After transmitting this many packets over MQTT, store our current state and exit the function.
    // this prevents flooding the ESP controllers wifi stack and potentially causing reboots/fatal exceptions
    mqttStartModule = i;
}

void mqtt1(const currentmonitoring_struct *currentMonitor, const Rules *rules)
{
    if (!checkMQTTReady())
    {
        return;
    }

    // If the BMS is in error, stop sending MQTT packets for the cell data
    if (!rules->ruleOutcome(Rule::BMSError))
    {
        MQTTCellData();
    }

    if (mysettings.currentMonitoringEnabled)
    {
        MQTTCurrentMonitoring(currentMonitor);
    }
}

void mqtt2(const PacketReceiveProcessor *receiveProc,
           const PacketRequestGenerator *prg,
           uint16_t requestq_count,
           const Rules *rules)
{
    if (!checkMQTTReady())
    {
        return;
    }

    GeneralStatusPayload(prg, receiveProc, requestq_count, rules);
    BankLevelInformation(rules);
}

void mqtt3(const Rules *rules, const RelayState *previousRelayState)
{
    if (!checkMQTTReady())
    {
        return;
    }

    RuleStatus(rules);
    OutputStatus(previousRelayState);
}
