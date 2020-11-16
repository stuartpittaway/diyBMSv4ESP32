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
    void processByte(uint8_t data);

    void sendByte(uint8_t data) {
        //debugByte(data);
        _stream->write(data);
    }
    void sendStartFrame()
    {
        sendByte(FrameStart);
    }

/*
    void debugByte(uint8_t data)
    {
        if (data <= 0x0F)
        {
            Serial1.write('0');
        }
        Serial1.print(data, HEX);
        Serial1.write(' ');
    }
*/

private:
//Important that these two bytes have the HIGHEST bit set, as this is cleared and added back in as part of the encoding
    const uint8_t FrameStart = B10101010;
    const uint8_t FrameEscape = B10001000;
    const uint8_t FrameMask = B01111111;
    const uint8_t FrameMaskInvert = B10000000;

    //Is the next received byte to be treated as encoded
    bool _escapeNextByte = false;
    //TRUE when we receive a valid start frame byte
    bool _startByteReceived=false;

    //Total size of the receive buffer
    uint8_t _receiveBufferSize = 0;
    //Pointer to start of receive buffer (byte array)
    uint8_t *_receiveBufferPtr = nullptr;
    //Index into receive buffer of current position
    uint8_t _receiveBufferIndex = 0;

    //Send/Receive stream
    Stream *_stream = nullptr;
    //Size of a valid fixed length packet of data
    uint8_t _packetSizeBytes = 0;

    //Call back function after _packetSizeBytes have been received
    PacketHandlerFunction _onPacketReceivedFunction = nullptr;
};