// MainController/tasks/CommunicationTask.h

#ifndef MAIN_COMMUNICATION_TASK_H
#define MAIN_COMMUNICATION_TASK_H

#include "CommonLib/messages.h"
#include "config.h"
#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>

/**
 * @class CommunicationTask
 * @brief Handles all inter-ESP communication, including both incoming and outgoing messages.
 */
class CommunicationTask
{
public:
    /**
     * @brief The main function that runs the CommunicationTask.
     * @param parameter Task parameters (unused).
     */
    static void run(void *parameter);

private:
    /**
     * @brief Handles incoming messages from slaves (SensorController/MotorController).
     */
    static void handleIncomingMessages();

    /**
     * @brief Handles outgoing messages to slaves (MotorController).
     */
    static void handleOutgoingMessages();

    /**
     * @brief Processes a received SensorDataMsg.
     * @param msg The received SensorDataMsg.
     */
    static void processSensorDataMessage(const SensorDataMsg &msg);

    /**
     * @brief Processes a received OccupancyGridMsg.
     * @param msg The received OccupancyGridMsg.
     */
    static void processOccupancyGridMessage(const OccupancyGridMsg &msg);

    /**
     * @brief Processes a received HealthStatusMsg.
     * @param msg The received HealthStatusMsg.
     */
    static void processHealthStatusMessage(const HealthStatusMsg &msg);

    /**
     * @brief Processes a received VelocityCommandMsg from MotorController.
     *        (If MotorController ever sends such messages back)
     * @param msg The received VelocityCommandMsg.
     */
    static void processVelocityCommandMessage(const VelocityCommandMsg &msg);

    /**
     * @brief Processes a received PumpCommandMsg from MotorController.
     *        (If MotorController ever sends such messages back)
     * @param msg The received PumpCommandMsg.
     */
    static void processPumpCommandMessage(const PumpCommandMsg &msg);
};

#endif // MAIN_COMMUNICATION_TASK_H
