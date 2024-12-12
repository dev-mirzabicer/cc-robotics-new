// CommonLib/InterESPProtocol.cpp

#include "InterESPProtocol.h"
#include "config_common.h" // For retry parameters
#include <Arduino.h>

// Initialize Wire in the init function
void InterESPProtocol::init(uint32_t clockSpeed)
{
    Wire.begin();
    Wire.setClock(clockSpeed);
    Serial.print("InterESPProtocol: I2C initialized at ");
    Serial.print(clockSpeed / 1000);
    Serial.println(" kHz.");
}

bool InterESPProtocol::sendMessage(uint8_t slaveAddress, MessageType msgType, const void *payload, uint16_t payloadSize)
{
    uint8_t retries = 0;
    bool success = false;

    while (retries < I2C_MAX_RETRIES && !success)
    {
        Wire.beginTransmission(slaveAddress);
        Wire.write(static_cast<uint8_t>(msgType));
        Wire.write(static_cast<uint8_t>(payloadSize & 0xFF));        // Low byte
        Wire.write(static_cast<uint8_t>((payloadSize >> 8) & 0xFF)); // High byte

        if (payloadSize > 0 && payload != nullptr)
        {
            Wire.write(reinterpret_cast<const uint8_t *>(payload), payloadSize);
        }

        uint8_t error = Wire.endTransmission();

        if (error == 0)
        {
            success = true;
            Serial.print("InterESPProtocol: Message sent to 0x");
            Serial.print(slaveAddress, HEX);
            Serial.print(" | Type: ");
            Serial.println(static_cast<int>(msgType));
        }
        else
        {
            retries++;
            Serial.print("InterESPProtocol: Failed to send message to 0x");
            Serial.print(slaveAddress, HEX);
            Serial.print(". Error code: ");
            Serial.print(error);
            Serial.print(". Retry attempt: ");
            Serial.println(retries);
            delay(I2C_RETRY_DELAY_MS);
        }
    }

    if (!success)
    {
        Serial.print("InterESPProtocol: All retries failed for sending message to 0x");
        Serial.println(slaveAddress, HEX);
    }

    return success;
}

bool InterESPProtocol::requestMessage(uint8_t slaveAddress, MessageType requestMsgType, void *responseBuffer, uint16_t responseSize)
{
    uint8_t retries = 0;
    bool success = false;

    while (retries < I2C_MAX_RETRIES && !success)
    {
        // Send the request message
        success = sendMessage(slaveAddress, requestMsgType, nullptr, 0);
        if (!success)
        {
            retries++;
            Serial.print("InterESPProtocol: Retry sending request message (Attempt ");
            Serial.print(retries);
            Serial.println(").");
            delay(I2C_RETRY_DELAY_MS);
            continue;
        }

        // Wait briefly to allow the slave to prepare the response
        delay(I2C_RETRY_DELAY_MS); // Use retry delay as wait time

        // Request the response
        Wire.requestFrom(static_cast<int>(slaveAddress), static_cast<int>(3 + responseSize), static_cast<int>(true)); // block until data is received or timeout

        unsigned long start = millis();
        while (Wire.available() < (3 + responseSize) && (millis() - start) < 100)
        { // 100ms timeout
            delay(10);
        }

        if (Wire.available() < (3 + responseSize))
        {
            Serial.print("InterESPProtocol: Incomplete response received from 0x");
            Serial.print(slaveAddress, HEX);
            Serial.println(".");
            success = false;
            retries++;
            delay(I2C_RETRY_DELAY_MS);
            continue;
        }

        // Read the response message type
        uint8_t respMsgTypeVal = Wire.read();
        MessageType respMsgType = static_cast<MessageType>(respMsgTypeVal);

        // Read payload length
        uint8_t lenLow = Wire.read();
        uint8_t lenHigh = Wire.read();
        uint16_t respPayloadLen = static_cast<uint16_t>(lenLow) | (static_cast<uint16_t>(lenHigh) << 8);

        if (respPayloadLen != responseSize)
        {
            Serial.print("InterESPProtocol: Response payload size mismatch from 0x");
            Serial.print(slaveAddress, HEX);
            Serial.print(". Expected: ");
            Serial.print(responseSize);
            Serial.print(", Received: ");
            Serial.println(respPayloadLen);
            // Drain the remaining bytes
            while (Wire.available())
            {
                Wire.read();
            }
            success = false;
            retries++;
            delay(I2C_RETRY_DELAY_MS);
            continue;
        }

        // Read the payload into responseBuffer
        if (Wire.readBytes(reinterpret_cast<char *>(responseBuffer), responseSize) != responseSize)
        {
            Serial.print("InterESPProtocol: Failed to read complete response payload from 0x");
            Serial.print(slaveAddress, HEX);
            Serial.println(".");
            success = false;
            retries++;
            delay(I2C_RETRY_DELAY_MS);
            continue;
        }

        // Successful read
        Serial.print("InterESPProtocol: Received message from 0x");
        Serial.print(slaveAddress, HEX);
        Serial.print(" | Type: ");
        Serial.println(static_cast<int>(respMsgType));
        success = true;
    }

    if (!success)
    {
        Serial.print("InterESPProtocol: All retries failed for requesting message from 0x");
        Serial.println(slaveAddress, HEX);
    }

    return success;
}
