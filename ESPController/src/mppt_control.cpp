#define USE_ESP_IDF_LOG 1
static constexpr const char *const TAG = "diybms-mppt-ctrl";

#include "mppt_control.h"
#include "settings.h"
#include <esp_log.h>

// Global instance
MPPTControl mppt_control;

// External references
extern Rules rules;
extern diybms_eeprom_settings mysettings;
extern uint16_t TotalNumberOfCells();

MPPTControl::MPPTControl() 
    : charge_state_(CHARGE_STATE_IDLE)
    , target_voltage_(0.0f)
    , target_current_(0.0f)
    , last_update_ms_(0)
    , state_entry_time_ms_(0)
    , manual_mode_(false)
    , manual_voltage_(0.0f)
    , manual_current_(0.0f)
{
}

void MPPTControl::begin() {
    ESP_LOGI(TAG, "MPPT Control initialized");
    charge_state_ = CHARGE_STATE_IDLE;
    last_update_ms_ = millis();
}

void MPPTControl::update() {
    // Only update if MPPT control is enabled
    if (!mysettings.mppt_control_enabled) {
        return;
    }
    
    // Rate limit updates (1 Hz)
    uint32_t now = millis();
    if (now - last_update_ms_ < 1000) {
        return;
    }
    last_update_ms_ = now;
    
    // Check if any MPPTs are online
    uint8_t online_count = mppt_manager.getOnlineCount();
    if (online_count == 0) {
        ESP_LOGD(TAG, "No MPPTs online");
        charge_state_ = CHARGE_STATE_IDLE;
        return;
    }
    
    // Manual mode override
    if (manual_mode_) {
        target_voltage_ = manual_voltage_;
        target_current_ = manual_current_;
        sendTargetsToMPPTs();
        return;
    }
    
    // Check BMS safety rules
    if (rules.ruleOutcome(Rule::BMSError)) {
        ESP_LOGW(TAG, "BMS Error - stopping charge");
        charge_state_ = CHARGE_STATE_STOPPED;
        enableAllMPPTs(false);
        return;
    }
    
    // Update charge state machine
    updateChargeState();
    
    // Calculate targets based on battery state
    target_voltage_ = calculateChargeVoltage();
    target_current_ = calculateChargeCurrent();
    
    // Apply to MPPTs
    distributeCurrentTargets();
    sendTargetsToMPPTs();
    
    ESP_LOGI(TAG, "State=%d, Target: %.2fV %.2fA, Online MPPTs: %d",
             charge_state_, target_voltage_, target_current_, online_count);
}

float MPPTControl::calculateChargeVoltage() {
    // Start with configured charge voltage (scale 0.1)
    float voltage = mysettings.chargevolt / 10.0f;
    
    // Apply temperature compensation if configured
    if (mysettings.mppt_temp_compensation_enabled) {
        voltage = applyTemperatureCompensation(voltage);
    }
    
    // Adjust based on charge state
    switch (charge_state_) {
        case CHARGE_STATE_BULK:
            // Use full charge voltage
            break;
            
        case CHARGE_STATE_ABSORPTION:
            // Stay at charge voltage
            break;
            
        case CHARGE_STATE_FLOAT:
            // Reduce to float voltage using configured offset (scale 0.1)
            voltage = mysettings.floatvoltage / 10.0f;
            break;
            
        case CHARGE_STATE_STOPPED:
            voltage = 0.0f;
            break;
            
        default:
            break;
    }
    
    // Safety bounds
    float min_voltage = TotalNumberOfCells() * 3.0f;  // Minimum safe voltage
    float max_voltage = TotalNumberOfCells() * 3.65f; // Maximum safe voltage
    voltage = constrain(voltage, min_voltage, max_voltage);
    
    return voltage;
}

float MPPTControl::calculateChargeCurrent() {
    // Start with configured charge current (scale 0.1)
    float current = mysettings.chargecurrent / 10.0f;
    
    // Check temperature limits - use Rules class variables
    if (rules.highestInternalTemp > mysettings.chargetemphigh) {
        ESP_LOGW(TAG, "High temperature - reducing charge current");
        current *= 0.5f; // Reduce to 50%
    }
    
    if (rules.lowestInternalTemp < mysettings.chargetemplow) {
        ESP_LOGW(TAG, "Low temperature - stopping charge");
        return 0.0f;
    }
    
    // Check for high cell voltage - use Rules class variables
    if (rules.highestCellVoltage > (mysettings.graph_voltagehigh - 50)) {
        ESP_LOGW(TAG, "Cell voltage high - reducing charge current");
        current *= 0.3f; // Reduce to 30%
    }
    
    // Adjust based on charge state
    switch (charge_state_) {
        case CHARGE_STATE_BULK:
            // Full current
            break;
            
        case CHARGE_STATE_ABSORPTION:
            // Taper current based on cell voltage
            if (rules.highestCellVoltage > (mysettings.graph_voltagehigh - 100)) {
                current *= 0.5f;
            }
            break;
            
        case CHARGE_STATE_FLOAT:
            // Very low current for float
            current *= 0.1f;
            break;
            
        case CHARGE_STATE_STOPPED:
            current = 0.0f;
            break;
            
        default:
            break;
    }
    
    // Safety bounds
    current = constrain(current, 0.0f, 200.0f); // Max 200A
    
    return current;
}

void MPPTControl::updateChargeState() {
    uint32_t time_in_state = millis() - state_entry_time_ms_;
    ChargeState new_state = charge_state_;
    
    // Get current cell voltage (mV) from Rules class
    uint16_t max_cell_mv = rules.highestCellVoltage;
    uint16_t min_cell_mv = rules.lowestCellVoltage;
    
    switch (charge_state_) {
        case CHARGE_STATE_IDLE:
            // Start charging if battery voltage is low enough
            if (max_cell_mv < (mysettings.graph_voltagehigh - 200)) {
                new_state = CHARGE_STATE_BULK;
                ESP_LOGI(TAG, "Starting BULK charge");
            }
            break;
            
        case CHARGE_STATE_BULK:
            // Transition to absorption when max cell reaches target
            if (max_cell_mv >= (mysettings.graph_voltagehigh - 50)) {
                new_state = CHARGE_STATE_ABSORPTION;
                ESP_LOGI(TAG, "Entering ABSORPTION phase");
            }
            break;
            
        case CHARGE_STATE_ABSORPTION:
            // Stay in absorption for configured time (convert to milliseconds)
            if (time_in_state > (mysettings.mppt_absorption_time_minutes * 60UL * 1000UL)) {
                // Check if current has tapered sufficiently
                // (Would need to monitor actual current from MPPTs)
                new_state = CHARGE_STATE_FLOAT;
                ESP_LOGI(TAG, "Entering FLOAT phase");
            }
            break;
            
        case CHARGE_STATE_FLOAT:
            // Stay in float unless voltage drops significantly
            if (min_cell_mv < (mysettings.graph_voltagehigh - 400)) {
                new_state = CHARGE_STATE_BULK;
                ESP_LOGI(TAG, "Restarting BULK charge from float");
            }
            break;
            
        case CHARGE_STATE_STOPPED:
            // Manual restart required
            break;
    }
    
    // Update state
    if (new_state != charge_state_) {
        charge_state_ = new_state;
        state_entry_time_ms_ = millis();
    }
}

float MPPTControl::applyTemperatureCompensation(float voltage) {
    // Temperature compensation: configurable mV per °C per cell
    // Default: -3mV/°C/cell above 25°C reference temperature
    int8_t avg_temp = (rules.highestInternalTemp + rules.lowestInternalTemp) / 2;
    float temp_offset = avg_temp - 25; // Degrees above reference
    
    // Apply compensation (convert mV to V)
    float compensation = temp_offset * TotalNumberOfCells() * (mysettings.mppt_temp_compensation_mv_per_c / 1000.0f);
    
    ESP_LOGD(TAG, "Temp compensation: %.2fV (temp=%d°C)", compensation, avg_temp);
    
    return voltage + compensation;
}

void MPPTControl::distributeCurrentTargets() {
    // For now, send same target to all MPPTs
    // Future enhancement: load balancing, MPPT-specific limits
    
    uint8_t online_count = mppt_manager.getOnlineCount();
    if (online_count == 0) {
        return;
    }
    
    // Equal distribution (could be enhanced with load balancing)
    // This will be applied per-MPPT in sendTargetsToMPPTs
    float current_per_mppt = target_current_ / online_count;
    
    ESP_LOGD(TAG, "Distributing %.2fA across %d MPPTs (%.2fA each)",
             target_current_, online_count, current_per_mppt);
}

void MPPTControl::sendTargetsToMPPTs() {
    uint8_t device_count = mppt_manager.getDeviceCount();
    uint8_t online_count = mppt_manager.getOnlineCount();
    
    if (online_count == 0) {
        return;
    }
    
    // Calculate per-MPPT current
    float current_per_mppt = target_current_ / online_count;
    
    for (uint8_t i = 0; i < device_count; i++) {
        const MPPTDevice* device = mppt_manager.getDevice(i);
        if (device && device->isOnline()) {
            // Send voltage target
            mppt_manager.sendTargetVoltage(device->node_id, target_voltage_);
            
            // Send distributed current target
            mppt_manager.sendTargetCurrent(device->node_id, current_per_mppt);
            
            // Enable charging if not stopped
            if (charge_state_ != CHARGE_STATE_STOPPED) {
                mppt_manager.sendEnableCommand(device->node_id, true);
            }
        }
    }
}

void MPPTControl::enableAllMPPTs(bool enable) {
    uint8_t device_count = mppt_manager.getDeviceCount();
    
    for (uint8_t i = 0; i < device_count; i++) {
        const MPPTDevice* device = mppt_manager.getDevice(i);
        if (device && device->isOnline()) {
            mppt_manager.sendEnableCommand(device->node_id, enable);
        }
    }
    
    ESP_LOGI(TAG, "All MPPTs %s", enable ? "ENABLED" : "DISABLED");
}

void MPPTControl::setManualVoltage(float voltage) {
    manual_voltage_ = voltage;
    manual_mode_ = true;
    ESP_LOGI(TAG, "Manual voltage set: %.2fV", voltage);
}

void MPPTControl::setManualCurrent(float current) {
    manual_current_ = current;
    manual_mode_ = true;
    ESP_LOGI(TAG, "Manual current set: %.2fA", current);
}
