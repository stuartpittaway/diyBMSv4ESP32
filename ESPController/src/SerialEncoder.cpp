#include "SerialEncoder.h"

// Checks stream for new bytes to arrive and processes them as needed
void SerialEncoder::update()
{
    if (_stream == nullptr)
        return;
    if (_receiveBufferPtr == nullptr)
        return;

    while (_stream->available() > 0)
    {
        uint8_t data = _stream->read();

        if (data == FrameStart)
        {
            //Its a new frame so start at begining of buffer
            debugByte(data);
            Serial1.println("FrameStart");
            _receiveBufferIndex = 0;
        }
        else if (data == FrameEscape)
        {
            //Escape the next two bytes
            _escapeNextBytes = 2;
            Serial1.print("_");
            debugByte(data);
        }
        else
        {
            if (_escapeNextBytes == 2)
            {
                Serial1.print("_");
                debugByte(data);
                _receiveBufferPtr[_receiveBufferIndex] = data;
                _escapeNextBytes--;
            }
            else if (_escapeNextBytes == 1)
            {
                Serial1.print("_");
                debugByte(data);
                //Or the bytes together
                _receiveBufferPtr[_receiveBufferIndex] = _receiveBufferPtr[_receiveBufferIndex] | data;
                _receiveBufferIndex++;
                _escapeNextBytes--;
            }
            else if (_escapeNextBytes == 0)
            {
                //Simple raw byte
                debugByte(data);
                _receiveBufferPtr[_receiveBufferIndex] = data;
                _receiveBufferIndex++;
            }
        }

        if (_receiveBufferIndex == _packetSizeBytes)
        {
            Serial1.println("Callback");
            //We have received an entire packet, so callback
            _onPacketReceivedFunction();
            _receiveBufferIndex = 0;
        }
        else if (_receiveBufferIndex == _receiveBufferSize)
        {
            //About to over flow, so reset
            _receiveBufferIndex = 0;
            Serial1.print("Overflow:");
            Serial1.print(_receiveBufferIndex);
            Serial1.print("=");
            Serial1.println(_receiveBufferSize);
        }
    }
}

// Sends a buffer (fixed length)
void SerialEncoder::send(const uint8_t *buffer)
{

    if (_stream == nullptr || buffer == nullptr)
        return;

    Serial1.println();
    sendStartFrame();

    debugByte(FrameStart);

    for (size_t i = 0; i < _packetSizeBytes; i++)
    {
        uint8_t v = buffer[i];

        if (v == FrameStart || v == FrameEscape)
        {
            //We escape the byte turning 1 byte into 3 bytes :-(
            _stream->write(FrameEscape);
            //Turn the byte into two halves so they don't get detected as frame markers
            _stream->write(v & B11110000);
            _stream->write(v & B00001111);

            debugByte(FrameEscape);
            debugByte(v & B11110000);
            debugByte(v & B00001111);
        }
        else
        {
            //Send the raw byte unescaped/edited
            _stream->write(v);
            debugByte(v);
        }
    }

    Serial1.println(" done");
}
