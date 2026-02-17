#define USE_ESP_IDF_LOG 1
static constexpr const char *const TAG = "diybms-thingset-can";

#include "thingset_can.h"
#include <esp_log.h>

bool thingset_can_parse(const twai_message_t* can_msg, ThingSetCANMessage* ts_msg) {
    if (!can_msg || !ts_msg) {
        return false;
    }
    
    // ThingSet uses extended 29-bit CAN IDs
    if (!(can_msg->flags & TWAI_MSG_FLAG_EXTD)) {
        return false;
    }
    
    // Extract fields from 29-bit CAN ID
    // Bit 28-24: Priority (ignore for now)
    // Bit 23-16: Target address
    // Bit 15-8:  Source address
    // Bit 7-3:   Reserved
    // Bit 2-0:   Message type
    
    uint32_t can_id = can_msg->identifier;
    
    ts_msg->target_addr = (can_id >> 16) & 0xFF;
    ts_msg->source_addr = (can_id >> 8) & 0xFF;
    ts_msg->msg_type = can_id & 0x07;
    
    // Validate message type
    if (ts_msg->msg_type > THINGSET_CAN_TYPE_RESPONSE) {
        return false;
    }
    
    // Copy data payload
    ts_msg->data_len = can_msg->data_length_code;
    if (ts_msg->data_len > sizeof(ts_msg->data)) {
        ts_msg->data_len = sizeof(ts_msg->data);
    }
    
    memcpy(ts_msg->data, can_msg->data, ts_msg->data_len);
    
    ESP_LOGD(TAG, "Parsed ThingSet: src=0x%02X, tgt=0x%02X, type=%d, len=%d",
             ts_msg->source_addr, ts_msg->target_addr, ts_msg->msg_type, ts_msg->data_len);
    
    return true;
}

bool thingset_can_encode(const ThingSetCANMessage* ts_msg, twai_message_t* can_msg) {
    if (!ts_msg || !can_msg) {
        return false;
    }
    
    // Validate message type
    if (ts_msg->msg_type > THINGSET_CAN_TYPE_RESPONSE) {
        return false;
    }
    
    // Validate data length
    // Note: Currently limiting to standard CAN (8 bytes). CAN-FD support (up to 64 bytes)
    // requires TWAI_MODE_FD flag and is not currently enabled in this implementation.
    if (ts_msg->data_len > 8) {
        ESP_LOGE(TAG, "ThingSet data too long: %d bytes", ts_msg->data_len);
        return false;
    }
    
    // Build 29-bit extended CAN ID
    // Priority = 6 (default for ThingSet)
    uint32_t can_id = 0;
    can_id |= (6 << 24);                      // Priority
    can_id |= (ts_msg->target_addr << 16);    // Target address
    can_id |= (ts_msg->source_addr << 8);     // Source address
    can_id |= (ts_msg->msg_type & 0x07);      // Message type
    
    can_msg->identifier = can_id;
    can_msg->flags = TWAI_MSG_FLAG_EXTD;  // Extended frame
    can_msg->data_length_code = ts_msg->data_len;
    
    // Copy data
    memcpy(can_msg->data, ts_msg->data, ts_msg->data_len);
    
    ESP_LOGD(TAG, "Encoded ThingSet: ID=0x%08X, len=%d", can_id, ts_msg->data_len);
    
    return true;
}
