#pragma once

#include <Arduino.h>

class SerialEncoder
{
public:
    typedef void (*PacketHandlerFunction)();

    void begin(Stream *stream, PacketHandlerFunction onPacketFunction, uint8_t packetSize, uint8_t *receiveBufferPtr, uint8_t receiveBufferSize)
    {
        _stream = stream;
        _onPacketReceivedFunction = onPacketFunction;
        _packetSizeBytes = packetSize;
        _receiveBufferPtr = receiveBufferPtr;
        _receiveBufferSize = receiveBufferSize;
    }

    void update();
    void send(const uint8_t *buffer);
    void sendStartFrame()
    {
        Serial1.print("Encoder:Send:");
        _stream->write(FrameStart);
    }

    void debugByte(uint8_t data)
    {
        if (data <= 0x0F)
        {
            Serial1.write('0');
        }
        Serial1.print(data, HEX);
        Serial1.write(' ');
    }

private:
    const uint8_t FrameStart = B10101010;
    const uint8_t FrameEscape = B10000000;

    uint8_t _escapeNextBytes = 0;
    uint8_t _receiveBufferSize = 0;
    uint8_t *_receiveBufferPtr = nullptr;
    uint8_t _receiveBufferIndex = 0;

    Stream *_stream = nullptr;
    uint8_t _packetSizeBytes = 0;
    PacketHandlerFunction _onPacketReceivedFunction = nullptr;
};