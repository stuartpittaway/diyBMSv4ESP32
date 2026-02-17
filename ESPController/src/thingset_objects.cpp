#define USE_ESP_IDF_LOG 1
static constexpr const char *const TAG = "diybms-thingset-obj";

#include "thingset_objects.h"
#include <esp_log.h>
#include <cbor.h>

bool thingset_decode_pubsub(const uint8_t* data, uint8_t len, ThingSetPubSubData* output) {
    if (!data || !output || len == 0) {
        return false;
    }
    
    // Initialize output structure
    memset(output, 0, sizeof(ThingSetPubSubData));
    
    // ThingSet publications are CBOR maps
    CborParser parser;
    CborValue map;
    
    CborError err = cbor_parser_init(data, len, 0, &parser, &map);
    if (err != CborNoError) {
        ESP_LOGE(TAG, "CBOR parser init failed: %d", err);
        return false;
    }
    
    // Check if it's a map
    if (!cbor_value_is_map(&map)) {
        ESP_LOGE(TAG, "Expected CBOR map");
        return false;
    }
    
    CborValue element;
    err = cbor_value_enter_container(&map, &element);
    if (err != CborNoError) {
        ESP_LOGE(TAG, "Failed to enter map: %d", err);
        return false;
    }
    
    // Iterate through map entries
    while (!cbor_value_at_end(&element)) {
        // Get key (should be integer)
        if (!cbor_value_is_integer(&element)) {
            cbor_value_advance(&element);
            continue;
        }
        
        int64_t key;
        cbor_value_get_int64(&element, &key);
        cbor_value_advance(&element);
        
        // Get value based on key
        switch (key) {
            case TS_OBJ_BAT_VOLTAGE:
                if (cbor_value_is_float(&element)) {
                    float val;
                    cbor_value_get_float(&element, &val);
                    output->bat_voltage = val;
                    output->has_bat_voltage = true;
                } else if (cbor_value_is_double(&element)) {
                    double val;
                    cbor_value_get_double(&element, &val);
                    output->bat_voltage = (float)val;
                    output->has_bat_voltage = true;
                }
                break;
                
            case TS_OBJ_BAT_CURRENT:
                if (cbor_value_is_float(&element)) {
                    float val;
                    cbor_value_get_float(&element, &val);
                    output->bat_current = val;
                    output->has_bat_current = true;
                } else if (cbor_value_is_double(&element)) {
                    double val;
                    cbor_value_get_double(&element, &val);
                    output->bat_current = (float)val;
                    output->has_bat_current = true;
                }
                break;
                
            case TS_OBJ_BAT_TEMP:
                if (cbor_value_is_integer(&element)) {
                    int64_t val;
                    cbor_value_get_int64(&element, &val);
                    output->bat_temp = (int16_t)val;
                    output->has_bat_temp = true;
                }
                break;
                
            case TS_OBJ_SOLAR_VOLTAGE:
                if (cbor_value_is_float(&element)) {
                    float val;
                    cbor_value_get_float(&element, &val);
                    output->solar_voltage = val;
                    output->has_solar_voltage = true;
                } else if (cbor_value_is_double(&element)) {
                    double val;
                    cbor_value_get_double(&element, &val);
                    output->solar_voltage = (float)val;
                    output->has_solar_voltage = true;
                }
                break;
                
            case TS_OBJ_SOLAR_CURRENT:
                if (cbor_value_is_float(&element)) {
                    float val;
                    cbor_value_get_float(&element, &val);
                    output->solar_current = val;
                    output->has_solar_current = true;
                } else if (cbor_value_is_double(&element)) {
                    double val;
                    cbor_value_get_double(&element, &val);
                    output->solar_current = (float)val;
                    output->has_solar_current = true;
                }
                break;
                
            case TS_OBJ_CHG_STATE:
                if (cbor_value_is_integer(&element)) {
                    int64_t val;
                    cbor_value_get_int64(&element, &val);
                    output->chg_state = (uint8_t)val;
                    output->has_chg_state = true;
                }
                break;
                
            default:
                // Unknown object ID, skip
                break;
        }
        
        cbor_value_advance(&element);
    }
    
    return true;
}

bool thingset_decode_response(const uint8_t* data, uint8_t len) {
    if (!data || len == 0) {
        return false;
    }
    
    // ThingSet response: first byte is status code
    // 0x80 = Success (Created)
    // 0x81 = Success (Deleted)
    // 0x82 = Success (Valid)
    // 0x83 = Success (Changed)
    // 0x84 = Success (Content)
    // 0xA0+ = Error codes
    
    uint8_t status = data[0];
    
    if (status >= 0x80 && status < 0xA0) {
        ESP_LOGD(TAG, "ThingSet response OK: 0x%02X", status);
        return true;
    }
    
    ESP_LOGE(TAG, "ThingSet response error: 0x%02X", status);
    return false;
}

uint8_t thingset_encode_write_request(uint8_t obj_id, float value, uint8_t* output, uint8_t max_len) {
    if (!output || max_len < 16) {
        return 0;
    }
    
    // ThingSet write request format:
    // Byte 0: Function code (0x07 for PATCH/write)
    // Remaining: CBOR map with object ID -> value
    
    output[0] = 0x07;  // PATCH function code
    
    CborEncoder encoder, mapEncoder;
    cbor_encoder_init(&encoder, output + 1, max_len - 1, 0);
    
    // Create map with 1 entry
    CborError err = cbor_encoder_create_map(&encoder, &mapEncoder, 1);
    if (err != CborNoError) {
        return 0;
    }
    
    // Add key (object ID)
    err = cbor_encode_int(&mapEncoder, obj_id);
    if (err != CborNoError) {
        return 0;
    }
    
    // Add value (float)
    err = cbor_encode_float(&mapEncoder, value);
    if (err != CborNoError) {
        return 0;
    }
    
    err = cbor_encoder_close_container(&encoder, &mapEncoder);
    if (err != CborNoError) {
        return 0;
    }
    
    size_t encoded_len = cbor_encoder_get_buffer_size(&encoder, output + 1);
    return 1 + encoded_len;  // +1 for function code
}

uint8_t thingset_encode_write_bool_request(uint8_t obj_id, bool value, uint8_t* output, uint8_t max_len) {
    if (!output || max_len < 16) {
        return 0;
    }
    
    output[0] = 0x07;  // PATCH function code
    
    CborEncoder encoder, mapEncoder;
    cbor_encoder_init(&encoder, output + 1, max_len - 1, 0);
    
    CborError err = cbor_encoder_create_map(&encoder, &mapEncoder, 1);
    if (err != CborNoError) {
        return 0;
    }
    
    err = cbor_encode_int(&mapEncoder, obj_id);
    if (err != CborNoError) {
        return 0;
    }
    
    err = cbor_encode_boolean(&mapEncoder, value);
    if (err != CborNoError) {
        return 0;
    }
    
    err = cbor_encoder_close_container(&encoder, &mapEncoder);
    if (err != CborNoError) {
        return 0;
    }
    
    size_t encoded_len = cbor_encoder_get_buffer_size(&encoder, output + 1);
    return 1 + encoded_len;
}
