// PacketStruct fuer die PC-Simulation (identisch zu Modul- und ESP32-Firmware)
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

static_assert(sizeof(PacketStruct) == 40, "PacketStruct muss 40 Byte haben");
