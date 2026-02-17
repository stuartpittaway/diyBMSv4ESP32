#define USE_ESP_IDF_LOG 1
static constexpr const char *const TAG = "diybms-mppt";

#include "mppt_manager.h"
#include "thingset_objects.h"
#include <esp_log.h>

// Global instance
MPPTManager mppt_manager;

// External reference to send function (from main.cpp)
extern void send_ext_canbus_message(const uint32_t identifier, const uint8_t *buffer, const uint8_t length);

MPPTManager::MPPTManager() : device_count_(0), last_timeout_check_(0) {
    memset(devices_, 0, sizeof(devices_));
}

void MPPTManager::begin() {
    ESP_LOGI(TAG, "MPPT Manager initialized");
    device_count_ = 0;
    last_timeout_check_ = millis();
}

void MPPTManager::update() {
    // Check for device timeouts periodically
    if (millis() - last_timeout_check_ > 1000) {
        checkTimeouts();
        last_timeout_check_ = millis();
    }
}

void MPPTManager::handleCANMessage(const twai_message_t* msg) {
    if (!msg) {
        return;
    }
    
    // Parse ThingSet message
    ThingSetCANMessage ts_msg;
    if (!thingset_can_parse(msg, &ts_msg)) {
        return;
    }
    
    processThingSetMessage(ts_msg);
}

void MPPTManager::processThingSetMessage(const ThingSetCANMessage& msg) {
    // Only process publication messages for now (Phase 1)
    if (msg.msg_type == THINGSET_CAN_TYPE_PUBSUB) {
        // This is telemetry from an MPPT device
        updateDeviceFromPubSub(msg.source_addr, msg.data, msg.data_len);
    } else if (msg.msg_type == THINGSET_CAN_TYPE_RESPONSE) {
        // Response to our request
        if (thingset_decode_response(msg.data, msg.data_len)) {
            ESP_LOGD(TAG, "Received successful response from node %d", msg.source_addr);
        } else {
            ESP_LOGW(TAG, "Received error response from node %d", msg.source_addr);
        }
    }
}

void MPPTManager::updateDeviceFromPubSub(uint8_t node_id, const uint8_t* data, uint8_t len) {
    // Decode CBOR publication
    ThingSetPubSubData pubsub_data;
    if (!thingset_decode_pubsub(data, len, &pubsub_data)) {
        ESP_LOGW(TAG, "Failed to decode publication from node %d", node_id);
        return;
    }
    
    // Find or create device
    MPPTDevice* device = findOrCreateDevice(node_id);
    if (!device) {
        ESP_LOGW(TAG, "Cannot add device node %d (max devices reached)", node_id);
        return;
    }
    
    // Update telemetry
    MPPTTelemetry telem = {};
    telem.bat_voltage = pubsub_data.has_bat_voltage ? pubsub_data.bat_voltage : device->telemetry.bat_voltage;
    telem.bat_current = pubsub_data.has_bat_current ? pubsub_data.bat_current : device->telemetry.bat_current;
    telem.solar_voltage = pubsub_data.has_solar_voltage ? pubsub_data.solar_voltage : device->telemetry.solar_voltage;
    telem.solar_current = pubsub_data.has_solar_current ? pubsub_data.solar_current : device->telemetry.solar_current;
    telem.bat_temperature = pubsub_data.has_bat_temp ? pubsub_data.bat_temp : device->telemetry.bat_temperature;
    
    // Map charge state
    if (pubsub_data.has_chg_state) {
        switch (pubsub_data.chg_state) {
            case 0:
                telem.state = MPPT_STATE_STANDBY;
                break;
            case 1:
            case 2:
                telem.state = MPPT_STATE_CC_CHARGE;
                break;
            case 3:
                telem.state = MPPT_STATE_CV_CHARGE;
                break;
            case 4:
                telem.state = MPPT_STATE_TRICKLE;
                break;
            default:
                telem.state = MPPT_STATE_UNKNOWN;
                break;
        }
    } else {
        telem.state = device->telemetry.state;
    }
    
    device->updateTelemetry(telem);
    
    if (!device->discovered) {
        ESP_LOGI(TAG, "Discovered MPPT device: node_id=%d", node_id);
    }
    
    ESP_LOGD(TAG, "Node %d: Bat=%.2fV %.2fA, Solar=%.2fV %.2fA, State=%d",
             node_id, telem.bat_voltage, telem.bat_current,
             telem.solar_voltage, telem.solar_current, telem.state);
}

MPPTDevice* MPPTManager::findOrCreateDevice(uint8_t node_id) {
    // First, try to find existing device
    for (uint8_t i = 0; i < device_count_; i++) {
        if (devices_[i].node_id == node_id) {
            return &devices_[i];
        }
    }
    
    // Not found, create new device if space available
    if (device_count_ < MPPT_MAX_DEVICES) {
        devices_[device_count_].node_id = node_id;
        devices_[device_count_].discovered = false;
        devices_[device_count_].online = false;
        devices_[device_count_].control_enabled = false;
        device_count_++;
        return &devices_[device_count_ - 1];
    }
    
    return nullptr;
}

void MPPTManager::checkTimeouts() {
    for (uint8_t i = 0; i < device_count_; i++) {
        if (devices_[i].online && !devices_[i].isOnline()) {
            ESP_LOGW(TAG, "Node %d offline (timeout)", devices_[i].node_id);
            devices_[i].markOffline();
        }
    }
}

uint8_t MPPTManager::getOnlineCount() const {
    uint8_t count = 0;
    for (uint8_t i = 0; i < device_count_; i++) {
        if (devices_[i].isOnline()) {
            count++;
        }
    }
    return count;
}

const MPPTDevice* MPPTManager::getDevice(uint8_t index) const {
    if (index >= device_count_) {
        return nullptr;
    }
    return &devices_[index];
}

const MPPTDevice* MPPTManager::getDeviceByNodeId(uint8_t node_id) const {
    for (uint8_t i = 0; i < device_count_; i++) {
        if (devices_[i].node_id == node_id) {
            return &devices_[i];
        }
    }
    return nullptr;
}

bool MPPTManager::sendThingSetRequest(uint8_t node_id, const uint8_t* data, uint8_t len) {
    if (!data || len == 0 || len > 8) {
        return false;
    }
    
    // Build ThingSet request message
    ThingSetCANMessage ts_msg;
    ts_msg.source_addr = 0x00;  // Broadcast address (we are the master)
    ts_msg.target_addr = node_id;
    ts_msg.msg_type = THINGSET_CAN_TYPE_REQUEST;
    ts_msg.data_len = len;
    memcpy(ts_msg.data, data, len);
    
    // Encode to CAN message
    twai_message_t can_msg;
    if (!thingset_can_encode(&ts_msg, &can_msg)) {
        ESP_LOGE(TAG, "Failed to encode ThingSet request");
        return false;
    }
    
    // Send via CAN bus
    send_ext_canbus_message(can_msg.identifier, can_msg.data, can_msg.data_length_code);
    
    ESP_LOGD(TAG, "Sent request to node %d, len=%d", node_id, len);
    return true;
}

bool MPPTManager::sendEnableCommand(uint8_t node_id, bool enable) {
    uint8_t buffer[16];
    uint8_t len = thingset_encode_write_bool_request(TS_OBJ_ENABLE, enable, buffer, sizeof(buffer));
    
    if (len == 0) {
        ESP_LOGE(TAG, "Failed to encode enable command");
        return false;
    }
    
    ESP_LOGI(TAG, "Sending enable=%d to node %d", enable, node_id);
    return sendThingSetRequest(node_id, buffer, len);
}

bool MPPTManager::sendTargetVoltage(uint8_t node_id, float voltage) {
    uint8_t buffer[16];
    uint8_t len = thingset_encode_write_request(TS_OBJ_TARGET_VOLTAGE, voltage, buffer, sizeof(buffer));
    
    if (len == 0) {
        ESP_LOGE(TAG, "Failed to encode target voltage command");
        return false;
    }
    
    ESP_LOGI(TAG, "Sending target voltage=%.2fV to node %d", voltage, node_id);
    return sendThingSetRequest(node_id, buffer, len);
}

bool MPPTManager::sendTargetCurrent(uint8_t node_id, float current) {
    uint8_t buffer[16];
    uint8_t len = thingset_encode_write_request(TS_OBJ_TARGET_CURRENT, current, buffer, sizeof(buffer));
    
    if (len == 0) {
        ESP_LOGE(TAG, "Failed to encode target current command");
        return false;
    }
    
    ESP_LOGI(TAG, "Sending target current=%.2fA to node %d", current, node_id);
    return sendThingSetRequest(node_id, buffer, len);
}
