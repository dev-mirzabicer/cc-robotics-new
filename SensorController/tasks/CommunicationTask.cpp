// SensorController/tasks/CommunicationTask.cpp

#include "CommunicationTask.h"
#include "CommonLib/InterESPProtocol.h"
#include "CommonLib/messages.h"
#include "config.h"
#include <Arduino.h>

// External queues defined in main.cpp
extern QueueHandle_t sensorDataQueue;
extern QueueHandle_t occupancyGridQueue;
extern QueueHandle_t communicationDataQueue;
extern QueueHandle_t healthStatusQueue;

extern OccupancyGrid occupancyGrid;
extern SemaphoreHandle_t occupancyGridMutex;

/**
 * @brief The main function that runs the CommunicationTask.
 *        It periodically sends sensor data, occupancy grids, and health status to MainController.
 * @param parameter Task parameters (unused).
 */
void CommunicationTask::run(void *parameter)
{
    Serial.println("CommunicationTask (SensorController): Starting...");

    // Initialize Inter-ESP communication
    InterESPProtocol::init(INTER_ESP_BAUD_RATE);

    while (true)
    {
        // Periodically send SensorDataMsg
        SensorDataMsg sensorData;
        if (xQueueReceive(sensorDataQueue, &sensorData, pdMS_TO_TICKS(100)) == pdPASS)
        {
            sendSensorData(sensorData);
        }

        // Periodically send OccupancyGridMsg
        OccupancyGridMsg gridMsg;
        if (xQueueReceive(occupancyGridQueue, &gridMsg, pdMS_TO_TICKS(100)) == pdPASS)
        {
            sendOccupancyGrid(gridMsg);
        }

        // Periodically send HealthStatusMsg
        HealthStatusMsg healthMsg;
        if (xQueueReceive(healthStatusQueue, &healthMsg, pdMS_TO_TICKS(100)) == pdPASS)
        {
            sendHealthStatus(healthMsg);
        }

        // Yield to allow other tasks to run
        taskYIELD();
    }
}

/**
 * @brief Sends the latest SensorDataMsg to MainController.
 * @param msg The SensorDataMsg to send.
 */
void CommunicationTask::sendSensorData(const SensorDataMsg &msg)
{
    // MainController is master; SensorController does not send unsolicited messages
    // Thus, we rely on request/response via I2C, handled in onReceive() and onRequest()
    // Therefore, this function may be unnecessary unless implementing additional unsolicited communication

    // For completeness, if we decide to implement a buffer to send when requested, do nothing here
}

/**
 * @brief Sends the latest OccupancyGridMsg to MainController.
 * @param msg The OccupancyGridMsg to send.
 */
void CommunicationTask::sendOccupancyGrid(const OccupancyGridMsg &msg)
{
    // Similar to sendSensorData(), OccupancyGrid is sent upon MainController's request
    // Thus, no action is needed here unless implementing unsolicited communication
}

/**
 * @brief Sends the latest HealthStatusMsg to MainController.
 * @param msg The HealthStatusMsg to send.
 */
void CommunicationTask::sendHealthStatus(const HealthStatusMsg &msg)
{
    // Similar to sendSensorData(), HealthStatus is sent upon MainController's request
    // Thus, no action is needed here unless implementing unsolicited communication
}
