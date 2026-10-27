// PacketStruct for the PC simulation (identical to module and ESP32 firmware)
#pragma once
#include <stdint.h>

#define maximum_cell_modules 16

struct PacketStruct
{
    uint8_t start_address;
    uint8_t end_address;
    uint8_t command;
    uint8_t hops;
    uint16_t sequence;
    uint16_t moduledata[maximum_cell_modules];
    uint16_t crc;
} __attribute__((packed));

static_assert(sizeof(PacketStruct) == 40, "PacketStruct must be 40 bytes");
