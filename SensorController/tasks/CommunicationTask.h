// SensorController/tasks/CommunicationTask.h

#ifndef SENSOR_COMMUNICATION_TASK_H
#define SENSOR_COMMUNICATION_TASK_H

#include "CommonLib/messages.h"
#include "config.h"
#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>

/**
 * @class CommunicationTask
 * @brief Handles all inter-ESP communication, including responding to MainController's requests.
 */
class CommunicationTask
{
public:
    /**
     * @brief The main function that runs the CommunicationTask.
     *        It handles responding to requests from MainController.
     * @param parameter Task parameters (unused).
     */
    static void run(void *parameter);

private:
    /**
     * @brief Sends the latest SensorDataMsg to MainController.
     * @param msg The SensorDataMsg to send.
     */
    static void sendSensorData(const SensorDataMsg &msg);

    /**
     * @brief Sends the latest OccupancyGridMsg to MainController.
     * @param msg The OccupancyGridMsg to send.
     */
    static void sendOccupancyGrid(const OccupancyGridMsg &msg);

    /**
     * @brief Sends the latest HealthStatusMsg to MainController.
     * @param msg The HealthStatusMsg to send.
     */
    static void sendHealthStatus(const HealthStatusMsg &msg);
};

#endif // SENSOR_COMMUNICATION_TASK_H
