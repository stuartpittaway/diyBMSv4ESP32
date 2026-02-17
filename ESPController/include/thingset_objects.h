#ifndef THINGSET_OBJECTS_H
#define THINGSET_OBJECTS_H

#include <Arduino.h>

// ThingSet object IDs used by LibreMPPT (from data_objects.cpp)
// These IDs correspond to the CBOR map keys in ThingSet publications
#define TS_OBJ_BAT_VOLTAGE      0x01  // "Bat_V" - Battery voltage
#define TS_OBJ_BAT_CURRENT      0x02  // "Bat_A" - Battery current
#define TS_OBJ_BAT_TEMP         0x04  // "Bat_degC" - Battery temperature
#define TS_OBJ_SOLAR_VOLTAGE    0x10  // "Solar_V" - Solar panel voltage
#define TS_OBJ_SOLAR_CURRENT    0x11  // "Solar_A" - Solar panel current
#define TS_OBJ_CHG_STATE        0x30  // "ChgState" - Charger state
#define TS_OBJ_ENABLE           0x60  // "wEnable" - Enable/disable charging
#define TS_OBJ_TARGET_VOLTAGE   0x61  // "rBatTarget_V" - Target voltage
#define TS_OBJ_TARGET_CURRENT   0x62  // "rBatTarget_A" - Target current

// Structure to hold decoded publication data
struct ThingSetPubSubData {
    bool has_bat_voltage;
    float bat_voltage;
    
    bool has_bat_current;
    float bat_current;
    
    bool has_bat_temp;
    int16_t bat_temp;
    
    bool has_solar_voltage;
    float solar_voltage;
    
    bool has_solar_current;
    float solar_current;
    
    bool has_chg_state;
    uint8_t chg_state;
};

// Decode ThingSet CBOR publication message
// Returns true if decoding was successful
bool thingset_decode_pubsub(const uint8_t* data, uint8_t len, ThingSetPubSubData* output);

// Decode ThingSet CBOR response message
// Returns true if decoding was successful (response OK)
bool thingset_decode_response(const uint8_t* data, uint8_t len);

// Encode ThingSet CBOR request to write a single value
// Returns encoded length, or 0 on error
uint8_t thingset_encode_write_request(uint8_t obj_id, float value, uint8_t* output, uint8_t max_len);

// Encode ThingSet CBOR request to write a boolean value
// Returns encoded length, or 0 on error
uint8_t thingset_encode_write_bool_request(uint8_t obj_id, bool value, uint8_t* output, uint8_t max_len);

#endif // THINGSET_OBJECTS_H
