#ifndef MPPT_MANAGER_H
#define MPPT_MANAGER_H

#include <Arduino.h>
#include <driver/twai.h>
#include "mppt_device.h"
#include "thingset_can.h"

#define MPPT_MAX_DEVICES 8
#define MPPT_TIMEOUT_MS 5000

class MPPTManager {
public:
    MPPTManager();
    
    // Lifecycle
    void begin();
    void update();  // Call from main loop to check timeouts
    
    // Message handling
    void handleCANMessage(const twai_message_t* msg);
    
    // Device access
    uint8_t getDeviceCount() const { return device_count_; }
    uint8_t getOnlineCount() const;
    const MPPTDevice* getDevice(uint8_t index) const;
    const MPPTDevice* getDeviceByNodeId(uint8_t node_id) const;
    
    // Control commands (basic implementation for Phase 1)
    bool sendEnableCommand(uint8_t node_id, bool enable);
    bool sendTargetVoltage(uint8_t node_id, float voltage);
    bool sendTargetCurrent(uint8_t node_id, float current);
    
private:
    MPPTDevice devices_[MPPT_MAX_DEVICES];
    uint8_t device_count_;
    uint32_t last_timeout_check_;
    
    void processThingSetMessage(const ThingSetCANMessage& msg);
    void updateDeviceFromPubSub(uint8_t node_id, const uint8_t* data, uint8_t len);
    MPPTDevice* findOrCreateDevice(uint8_t node_id);
    void checkTimeouts();
    bool sendThingSetRequest(uint8_t node_id, const uint8_t* data, uint8_t len);
};

// Global instance
extern MPPTManager mppt_manager;

#endif // MPPT_MANAGER_H
