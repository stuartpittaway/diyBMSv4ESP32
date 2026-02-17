#ifndef THINGSET_CAN_H
#define THINGSET_CAN_H

#include <Arduino.h>
#include <driver/twai.h>

// ThingSet CAN message types
#define THINGSET_CAN_TYPE_PUBSUB    0x00  // Publication message
#define THINGSET_CAN_TYPE_REQUEST   0x01  // Request message
#define THINGSET_CAN_TYPE_RESPONSE  0x02  // Response message

// ThingSet CAN address format (29-bit extended)
// Bit 28-24: Priority (typically 6)
// Bit 23-16: Target address (0x00 = broadcast)
// Bit 15-8:  Source address (MPPT node ID, e.g., 10, 11, 12)
// Bit 7-3:   Reserved
// Bit 2-0:   Message type (0=PubSub, 1=Request, 2=Response)

struct ThingSetCANMessage {
    uint8_t source_addr;
    uint8_t target_addr;
    uint8_t msg_type;
    uint8_t data[64];  // Max CAN-FD payload
    uint8_t data_len;
};

// Parse a TWAI CAN message into a ThingSet message structure
// Returns true if message was successfully parsed as ThingSet format
bool thingset_can_parse(const twai_message_t* can_msg, ThingSetCANMessage* ts_msg);

// Encode a ThingSet message into a TWAI CAN message
// Returns true if encoding was successful
bool thingset_can_encode(const ThingSetCANMessage* ts_msg, twai_message_t* can_msg);

#endif // THINGSET_CAN_H
