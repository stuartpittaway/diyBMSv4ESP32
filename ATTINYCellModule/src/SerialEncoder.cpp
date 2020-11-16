#include "SerialEncoder.h"

//Process a single byte received from the stream
void SerialEncoder::processByte(uint8_t data)
{
    //debugByte(data);

    if (data == FrameStart)
    {
        //Serial1.print(" FrameStart ");
        //Its a new frame so start at begining of buffer
        reset();
        _startByteReceived = true;
        return;
    }

    if (!_startByteReceived)
    {
        //Ignore all bytes until the start frame is detected
        return;
    }

    if (data == FrameEscape)
    {
        //Serial1.print("_");
        //Escape the next byte, and ignore this byte
        _escapeNextByte = true;
        return;
    }

    if (_escapeNextByte)
    {
        //Serial1.print("_");
        //Add the BIT back on to the encoded data
        data = (data | FrameMaskInvert);
        //Serial1.print("[");        debugByte(data);        Serial1.print("]");
    }

    //Simple raw byte
    _receiveBufferPtr[_receiveBufferIndex] = data;
    _receiveBufferIndex++;
    _escapeNextByte = false;

    //Now check if we have received the whole packet?
    if (_receiveBufferIndex == _packetSizeBytes)
    {
        //We have received an entire packet, so callback
        _onPacketReceivedFunction();
        reset();
        return;
    }

    if (_receiveBufferIndex == _receiveBufferSize)
    {
        //About to over flow, so abort data packet, and reset
        reset();
    }
}

// Checks stream for new bytes to arrive and processes them as needed
void SerialEncoder::checkInputStream()
{
    if (_stream == nullptr || _receiveBufferPtr == nullptr)
        return;

    while (_stream->available() > 0)
    {
        processByte((uint8_t)_stream->read());
    }
}

// Sends a buffer (fixed length)
void SerialEncoder::sendBuffer(const uint8_t *buffer)
{
    //Serial1.println();    Serial1.print("Send:");

    if (_stream == nullptr || buffer == nullptr)
        return;

    sendStartFrame();

    for (size_t i = 0; i < _packetSizeBytes; i++)
    {
        //This is the raw byte to send
        uint8_t v = buffer[i];

        if (v == FrameStart || v == FrameEscape)
        {
            //We escape the byte turning 1 byte into 2 bytes :-(
            //Serial1.print("_");
            sendByte(FrameEscape);
            //Strip the mask bit out - this clears the highest BIT of the byte
            //Serial1.print("_");
            sendByte(v & FrameMask);
        }
        else
        {
            //Send the raw byte unescaped/edited
            sendByte(v);
        }
    }

    //Serial1.println();
}
