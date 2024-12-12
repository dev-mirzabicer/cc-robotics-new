// MainController/tasks/CommunicationTask.cpp

#include "CommunicationTask.h"
#include "CommonLib/InterESPProtocol.h"
#include "config.h"
#include "CommonLib/messages.h"

// External queues defined in main.cpp
extern QueueHandle_t sensorDataQueue;
extern QueueHandle_t pathQueue;
extern QueueHandle_t occupancyGridQueue;
extern QueueHandle_t adjustedPathQueue;
extern QueueHandle_t healthStatusQueue;

extern QueueHandle_t motorVelocityCmdQueue; // Outgoing velocity commands to MotorController
extern QueueHandle_t pumpCmdQueue;          // Outgoing pump commands to MotorController

/**
 * @brief The main function that runs the CommunicationTask.
 *        It handles both incoming messages from slaves and outgoing messages to MotorController.
 * @param parameter Task parameters (unused).
 */
void CommunicationTask::run(void *parameter)
{
    Serial.println("CommunicationTask (MainController): Starting...");

    // Initialize Inter-ESP communication as master
    InterESPProtocol::init(INTER_ESP_BAUD_RATE);

    // Main loop
    while (true)
    {
        // First, handle any incoming messages from MotorController or SensorController
        handleIncomingMessages();

        // Next, handle any outgoing messages that need to be sent to MotorController
        handleOutgoingMessages();

        // Yield to allow other tasks to run
        taskYIELD();
    }
}

/**
 * @brief Handles incoming messages from slaves (SensorController/MotorController).
 */
void CommunicationTask::handleIncomingMessages()
{
    uint8_t senderAddress;
    MessageType msgType;
    uint8_t buffer[256]; // max message size

    // Attempt to receive an incoming message (non-blocking)
    // Assuming InterESPProtocol::receiveMessage is implemented to handle master receiving
    // However, in I2C master mode, receiving is via requestMessage, so incoming messages are handled differently
    // Therefore, we need to handle incoming data as part of the request-response protocol
    // In this design, main controller initiates requests, so we don't have unsolicited incoming messages

    // If your InterESPProtocol allows unsolicited messages via a separate mechanism, implement it here
    // For simplicity, we proceed with request-response only

    // Therefore, no action is taken here for incoming messages
}

/**
 * @brief Handles outgoing messages to slaves (MotorController).
 *        Specifically, sends VelocityCommandMsg and PumpCommandMsg to MotorController.
 */
void CommunicationTask::handleOutgoingMessages()
{
    // Check for a VelocityCommandMsg to send to MotorController
    VelocityCommandMsg velCmd;
    while (xQueueReceive(motorVelocityCmdQueue, &velCmd, 0) == pdPASS)
    {
        // Send velocity command to MotorController using InterESPProtocol
        bool success = InterESPProtocol::sendMessage(
            MOTOR_CONTROLLER_I2C_ADDRESS,
            MessageType::VELOCITY_COMMAND,
            &velCmd,
            sizeof(VelocityCommandMsg));

        if (!success)
        {
            Serial.println("CommunicationTask: Failed to send VelocityCommandMsg to MotorController.");
            // Optionally, implement retry logic or re-enqueue the message
        }
        else
        {
            Serial.print("CommunicationTask: Sent VelocityCommandMsg to MotorController: linear_vel_x=");
            Serial.print(velCmd.linear_vel_x);
            Serial.print(", angular_vel_z=");
            Serial.println(velCmd.angular_vel_z);
        }
    }

    // Check for a PumpCommandMsg to send to MotorController
    PumpCommandMsg pumpCmd;
    while (xQueueReceive(pumpCmdQueue, &pumpCmd, 0) == pdPASS)
    {
        // Send pump command to MotorController using InterESPProtocol
        bool success = InterESPProtocol::sendMessage(
            MOTOR_CONTROLLER_I2C_ADDRESS,
            MessageType::PUMP_COMMAND,
            &pumpCmd,
            sizeof(PumpCommandMsg));

        if (!success)
        {
            Serial.println("CommunicationTask: Failed to send PumpCommandMsg to MotorController.");
            // Optionally, implement retry logic or re-enqueue the message
        }
        else
        {
            Serial.print("CommunicationTask: Sent PumpCommandMsg to MotorController: Intake=");
            Serial.print(pumpCmd.pumpIntakeControl > 0 ? "ON" : "OFF");
            Serial.print(", Outflow=");
            Serial.println(pumpCmd.pumpOutflowControl > 0 ? "ON" : "OFF");
        }
    }

    // Optionally, handle other outgoing messages or command types here
}

/**
 * @brief Processes a received SensorDataMsg.
 * @param msg The received SensorDataMsg.
 */
void CommunicationTask::processSensorDataMessage(const SensorDataMsg &msg)
{
    if (xQueueSend(sensorDataQueue, &msg, 0) != pdPASS)
    {
        Serial.println("CommunicationTask: sensorDataQueue full, dropping SensorDataMsg.");
    }
}

/**
 * @brief Processes a received OccupancyGridMsg.
 * @param msg The received OccupancyGridMsg.
 */
void CommunicationTask::processOccupancyGridMessage(const OccupancyGridMsg &msg)
{
    if (xQueueSend(occupancyGridQueue, &msg, 0) != pdPASS)
    {
        Serial.println("CommunicationTask: occupancyGridQueue full, dropping OccupancyGridMsg.");
    }
}

/**
 * @brief Processes a received HealthStatusMsg.
 * @param msg The received HealthStatusMsg.
 */
void CommunicationTask::processHealthStatusMessage(const HealthStatusMsg &msg)
{
    if (xQueueSend(healthStatusQueue, &msg, 0) != pdPASS)
    {
        Serial.println("CommunicationTask: healthStatusQueue full, dropping HealthStatusMsg.");
    }
}

/**
 * @brief Processes a received VelocityCommandMsg from MotorController.
 *        (Not typically expected unless bidirectional commands are needed)
 * @param msg The received VelocityCommandMsg.
 */
void CommunicationTask::processVelocityCommandMessage(const VelocityCommandMsg &msg)
{
    // If MotorController sends VelocityCommandMsg back, handle it here
    Serial.print("CommunicationTask: Received VelocityCommandMsg from MotorController: linear_vel_x=");
    Serial.print(msg.linear_vel_x);
    Serial.print(", angular_vel_z=");
    Serial.println(msg.angular_vel_z);
    // Handle accordingly, e.g., update state or logs
}

/**
 * @brief Processes a received PumpCommandMsg from MotorController.
 *        (Not typically expected unless bidirectional commands are needed)
 * @param msg The received PumpCommandMsg.
 */
void CommunicationTask::processPumpCommandMessage(const PumpCommandMsg &msg)
{
    // If MotorController sends PumpCommandMsg back, handle it here
    Serial.print("CommunicationTask: Received PumpCommandMsg from MotorController: Intake=");
    Serial.print(msg.pumpIntakeControl > 0 ? "ON" : "OFF");
    Serial.print(", Outflow=");
    Serial.println(msg.pumpOutflowControl > 0 ? "ON" : "OFF");
    // Handle accordingly, e.g., update state or logs
}
