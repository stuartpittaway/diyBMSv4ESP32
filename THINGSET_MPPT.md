# ThingSet CAN Protocol Support for MPPT Communication

## Overview

This implementation adds ThingSet CAN protocol support to diyBMS for bidirectional communication with LibreMPPT charge controllers. This is Phase 1 (MVP) focusing on telemetry reception and basic control commands.

## Features Implemented

### 1. ThingSet CAN Protocol Layer
- **Extended Frame Support**: 29-bit CAN addressing as per ThingSet spec
- **Message Types**: PubSub (0x00), Request (0x01), Response (0x02)
- **Address Parsing**: Source/target address extraction from CAN ID
- **CBOR Encoding/Decoding**: Using arduino-cbor library for ThingSet data objects

### 2. MPPT Device Management
- **Device Registry**: Track up to 8 MPPT devices simultaneously
- **Auto-Discovery**: Devices discovered via CAN message monitoring
- **Heartbeat/Timeout**: Mark devices offline after 5 seconds of no telemetry
- **Per-Device Telemetry**: Store voltage, current, temperature, state per MPPT

### 3. Telemetry Reception
Receive and decode ThingSet publications from MPPT `SUBSET_CAN`:
- Battery voltage (`Bat_V`)
- Battery current (`Bat_A`)
- Solar voltage (`Solar_V`)
- Solar current (`Solar_A`)
- Battery temperature (`Bat_degC`)
- Charger state (`ChgState`)

### 4. Basic Control Commands
- **Enable/Disable** charging (`wEnable`)
- **Set target voltage** (`rBatTarget_V`)
- **Set target current** (`rBatTarget_A`)

### 5. Integration
- Coexists with existing inverter protocols (Pylon, PylonForce, Victron)
- Configuration via `diybms_eeprom_settings`:
  - `mppt_control_enabled` - Enable MPPT control (default: false)
  - `mppt_base_node_id` - Base ThingSet node ID (default: 10)
  - `mppt_max_devices` - Max MPPTs to manage (default: 4)
  - `mppt_telemetry_interval_ms` - Telemetry request interval (default: 1000ms)

## File Structure

### New Header Files
- `ESPController/include/thingset_can.h` - ThingSet CAN protocol definitions
- `ESPController/include/thingset_objects.h` - ThingSet object IDs and CBOR encoding
- `ESPController/include/mppt_device.h` - MPPT device and telemetry structures
- `ESPController/include/mppt_manager.h` - MPPT manager class interface

### New Implementation Files
- `ESPController/src/thingset_can.cpp` - CAN frame parsing/encoding
- `ESPController/src/thingset_objects.cpp` - CBOR encoding/decoding
- `ESPController/src/mppt_manager.cpp` - Device management and message handling

### Modified Files
- `ESPController/platformio.ini` - Added arduino-cbor library dependency
- `ESPController/include/defines.h` - Added MPPT settings to eeprom_settings struct
- `ESPController/src/settings.cpp` - Added MPPT default configuration
- `ESPController/src/main.cpp`:
  - Added MPPT manager initialization in `setup()`
  - Added MPPT manager update in `loop()`
  - Modified `canbus_rx()` to handle ThingSet messages

## Usage

### Enable MPPT Control
1. Set `mppt_control_enabled = true` in settings
2. Configure `mppt_base_node_id` to match your MPPT node IDs
3. Reboot controller

### Monitor MPPT Telemetry
Check serial logs for messages like:
```
[MPPT] Discovered device: node_id=10
[MPPT] Node 10: Bat=57.2V 23.4A, Solar=82.1V 18.2A, State=CC_CHARGE
```

### Send Control Commands (API for future web UI)
```cpp
// Enable charging
mppt_manager.sendEnableCommand(10, true);

// Set target voltage
mppt_manager.sendTargetVoltage(10, 58.4);

// Set target current
mppt_manager.sendTargetCurrent(10, 25.0);
```

## ThingSet CAN Protocol Details

### CAN ID Format (29-bit Extended)
```
Bit 28-24: Priority (6 for ThingSet)
Bit 23-16: Target address (0x00 = broadcast)
Bit 15-8:  Source address (MPPT node ID)
Bit 7-3:   Reserved
Bit 2-0:   Message type (0=PubSub, 1=Request, 2=Response)
```

### Message Types
- **Publication (0x00)**: MPPT → diyBMS telemetry (1 Hz)
- **Request (0x01)**: diyBMS → MPPT control command
- **Response (0x02)**: MPPT → diyBMS confirmation

### CBOR Encoding
ThingSet uses CBOR (RFC 7049) for efficient data encoding:
- Publications: CBOR map `{object_id: value, ...}`
- Requests: Function code (0x07 for PATCH) + CBOR map
- Responses: Status code (0x80-0x8F = success)

## Backward Compatibility

This implementation maintains 100% compatibility with existing functionality:
- ✅ All inverter protocols work unchanged
- ✅ MPPT control is **opt-in** (disabled by default)
- ✅ No changes to existing web UI
- ✅ No changes to BMS rules or charge logic
- ✅ Extended CAN frames filtered by message type to avoid conflicts

## Testing

### Unit Tests
- ThingSet CAN message parsing (various address combinations)
- CBOR decoding of LibreMPPT data structures
- Device registry add/remove/timeout logic

### Integration Tests
1. **Single MPPT**: Connect one LibreMPPT, verify telemetry reception
2. **Multi-MPPT**: Connect 2-3 MPPTs, verify independent tracking
3. **Coexistence**: Run with Pylon protocol simultaneously
4. **Timeout**: Disconnect MPPT, verify offline detection after 5 seconds
5. **Control**: Send enable/voltage/current commands, verify MPPT response

## Future Phases

### Phase 2: Master Control Logic
- Charge algorithm using BMS rules
- Send voltage/current targets to MPPTs
- Coordinate multiple MPPTs

### Phase 3: Web Interface
- REST API for MPPT data
- Web UI for monitoring
- Web UI for manual control

### Phase 4: Advanced Features
- Load balancing between MPPTs
- MPPT-specific current limits
- Historical data logging

## References

- ThingSet Protocol: https://thingset.io/spec/v0.6/
- LibreMPPT: https://github.com/LibreSolar/MPPT-2420-LC
- Existing diyBMS CAN: `pylon_canbus.cpp`, `pylonforce_canbus.cpp`
- ESP32 TWAI Driver: `HAL_ESP32.cpp`
