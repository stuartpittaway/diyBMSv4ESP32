#ifndef MPPT_DEVICE_H
#define MPPT_DEVICE_H

#include <Arduino.h>

enum MPPTState : uint8_t {
    MPPT_STATE_UNKNOWN = 0,
    MPPT_STATE_STANDBY = 1,
    MPPT_STATE_CC_CHARGE = 2,
    MPPT_STATE_CV_CHARGE = 3,
    MPPT_STATE_TRICKLE = 4,
    MPPT_STATE_OFFLINE = 5
};

struct MPPTTelemetry {
    float bat_voltage;      // Battery voltage (V)
    float bat_current;      // Battery current (A)
    float bat_power;        // Calculated (W)
    float solar_voltage;    // Solar panel voltage (V)
    float solar_current;    // Solar panel current (A)
    float solar_power;      // Calculated (W)
    int16_t bat_temperature; // Battery temperature (°C)
    MPPTState state;
    uint32_t last_update;   // millis() timestamp
};

struct MPPTDevice {
    uint8_t node_id;        // ThingSet CAN node address (e.g., 10, 11, 12...)
    bool discovered;        // Has been seen on CAN bus
    bool online;            // Updated within timeout period
    MPPTTelemetry telemetry;
    
    // Control state
    bool control_enabled;   // Should we control this MPPT?
    float target_voltage;   // Commanded voltage (V)
    float target_current;   // Commanded current (A)
    
    // Check if device is online (last_update < timeout)
    bool isOnline() const {
        return online && (millis() - telemetry.last_update < 5000);
    }
    
    // Mark device as offline
    void markOffline() {
        online = false;
        telemetry.state = MPPT_STATE_OFFLINE;
    }
    
    // Update telemetry data
    void updateTelemetry(const MPPTTelemetry& data) {
        telemetry = data;
        telemetry.last_update = millis();
        // Calculate power
        telemetry.bat_power = telemetry.bat_voltage * telemetry.bat_current;
        telemetry.solar_power = telemetry.solar_voltage * telemetry.solar_current;
        online = true;
        discovered = true;
    }
};

#endif // MPPT_DEVICE_H
