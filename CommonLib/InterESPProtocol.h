// CommonLib/InterESPProtocol.h

#ifndef INTER_ESP_PROTOCOL_H
#define INTER_ESP_PROTOCOL_H

#include <Wire.h>
#include "messages.h"

/**
 * @class InterESPProtocol
 * @brief Handles inter-ESP communication using I2C with a request/response mechanism.
 */
class InterESPProtocol
{
public:
    /**
     * @brief Initializes the I2C communication with the specified clock speed.
     * @param clockSpeed I2C clock speed in Hz (e.g., 400000 for 400kHz).
     */
    static void init(uint32_t clockSpeed = 400000);

    /**
     * @brief Sends a message to a slave device.
     * @param slaveAddress I2C address of the slave device.
     * @param msgType Type of the message.
     * @param payload Pointer to the message payload data.
     * @param payloadSize Size of the payload data in bytes.
     * @return True if the message was sent successfully, false otherwise.
     */
    static bool sendMessage(uint8_t slaveAddress, MessageType msgType, const void *payload, uint16_t payloadSize);

    /**
     * @brief Requests a message from a slave device.
     * @param slaveAddress I2C address of the slave device.
     * @param requestMsgType Type of the request message.
     * @param responseBuffer Pointer to the buffer where the response will be stored.
     * @param responseSize Size of the expected response payload in bytes.
     * @return True if the response was received successfully, false otherwise.
     */
    static bool requestMessage(uint8_t slaveAddress, MessageType requestMsgType, void *responseBuffer, uint16_t responseSize);
};

#endif // INTER_ESP_PROTOCOL_H
