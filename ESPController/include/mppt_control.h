#ifndef MPPT_CONTROL_H
#define MPPT_CONTROL_H

#include "mppt_manager.h"
#include "Rules.h"

enum ChargeState : uint8_t {
    CHARGE_STATE_IDLE = 0,
    CHARGE_STATE_BULK = 1,      // CC - constant current
    CHARGE_STATE_ABSORPTION = 2, // CV - constant voltage
    CHARGE_STATE_FLOAT = 3,
    CHARGE_STATE_STOPPED = 4
};

class MPPTControl {
public:
    MPPTControl();
    
    // Initialize with references to rules and settings
    void begin();
    
    // Main control loop - call periodically (e.g., every 1 second)
    void update();
    
    // Manual control
    void enableAllMPPTs(bool enable);
    void setManualVoltage(float voltage);
    void setManualCurrent(float current);
    
    // Get current state
    ChargeState getChargeState() const { return charge_state_; }
    float getTargetVoltage() const { return target_voltage_; }
    float getTargetCurrent() const { return target_current_; }
    
private:
    // Calculate optimal charge voltage based on battery state
    float calculateChargeVoltage();
    
    // Calculate optimal charge current based on battery state and rules
    float calculateChargeCurrent();
    
    // Distribute current across multiple MPPTs
    void distributeCurrentTargets();
    
    // Update charge state machine
    void updateChargeState();
    
    // Apply temperature compensation
    float applyTemperatureCompensation(float voltage);
    
    // Send targets to all online MPPTs
    void sendTargetsToMPPTs();
    
    ChargeState charge_state_;
    float target_voltage_;
    float target_current_;
    
    // State machine timing
    uint32_t last_update_ms_;
    uint32_t state_entry_time_ms_;
    
    // Manual control mode
    bool manual_mode_;
    float manual_voltage_;
    float manual_current_;
};

extern MPPTControl mppt_control;

#endif // MPPT_CONTROL_H
