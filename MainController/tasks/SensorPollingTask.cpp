// MainController/tasks/SensorPollingTask.cpp

#include "SensorPollingTask.h"
#include "CommonLib/InterESPProtocol.h"
#include "CommonLib/messages.h"
#include "config.h"

// External queues defined in main.cpp
extern QueueHandle_t sensorDataQueue;    ///< Queue for SensorDataMsg
extern QueueHandle_t occupancyGridQueue; ///< Queue for OccupancyGridMsg
extern QueueHandle_t healthStatusQueue;  ///< Queue for HealthStatusMsg

/**
 * @brief The main function that runs the SensorPollingTask.
 *        It periodically requests sensor data, occupancy grid data, and health status from SensorController.
 * @param parameter Task parameters (unused).
 */
void SensorPollingTask::run(void *parameter)
{
    Serial.println("SensorPollingTask: Starting...");

    const TickType_t xDelay500ms = pdMS_TO_TICKS(500); // 500ms delay between polling
    const TickType_t xDelay1s = pdMS_TO_TICKS(1000);   // 1s delay for occupancy grid polling

    // Define the I2C address of SensorController
    const uint8_t SENSOR_CONTROLLER_I2C_ADDRESS = SENSOR_CONTROLLER_I2C_ADDRESS;

    while (true)
    {
        // -----------------------------
        // 1. Request Sensor Data
        // -----------------------------
        {
            SensorDataMsg sensorData;
            bool success = InterESPProtocol::requestMessage(
                SENSOR_CONTROLLER_I2C_ADDRESS,
                MessageType::REQUEST_SENSOR_DATA,
                &sensorData,
                sizeof(SensorDataMsg));

            if (success)
            {
                // Enqueue the received sensor data
                if (xQueueSend(sensorDataQueue, &sensorData, pdMS_TO_TICKS(100)) != pdPASS)
                {
                    Serial.println("SensorPollingTask: Failed to enqueue SensorDataMsg (Queue Full).");
                }
                else
                {
                    Serial.println("SensorPollingTask: Enqueued SensorDataMsg.");
                }
            }
            else
            {
                Serial.println("SensorPollingTask: Failed to request SensorDataMsg from SensorController.");
            }
        }

        // -----------------------------
        // 2. Request Occupancy Grid Data
        // -----------------------------
        {
            OccupancyGridMsg occupancyGrid;
            bool success = InterESPProtocol::requestMessage(
                SENSOR_CONTROLLER_I2C_ADDRESS,
                MessageType::REQUEST_OCCUPANCY_GRID,
                &occupancyGrid,
                sizeof(OccupancyGridMsg));

            if (success)
            {
                // Enqueue the received occupancy grid data
                if (xQueueSend(occupancyGridQueue, &occupancyGrid, pdMS_TO_TICKS(100)) != pdPASS)
                {
                    Serial.println("SensorPollingTask: Failed to enqueue OccupancyGridMsg (Queue Full).");
                }
                else
                {
                    Serial.println("SensorPollingTask: Enqueued OccupancyGridMsg.");
                }
            }
            else
            {
                Serial.println("SensorPollingTask: Failed to request OccupancyGridMsg from SensorController.");
            }
        }

        // -----------------------------
        // 3. Request Health Status
        // -----------------------------
        {
            HealthStatusMsg healthStatus;
            bool success = InterESPProtocol::requestMessage(
                SENSOR_CONTROLLER_I2C_ADDRESS,
                MessageType::REQUEST_HEALTH_STATUS,
                &healthStatus,
                sizeof(HealthStatusMsg));

            if (success)
            {
                // Enqueue the received health status
                if (xQueueSend(healthStatusQueue, &healthStatus, pdMS_TO_TICKS(100)) != pdPASS)
                {
                    Serial.println("SensorPollingTask: Failed to enqueue HealthStatusMsg (Queue Full).");
                }
                else
                {
                    Serial.println("SensorPollingTask: Enqueued HealthStatusMsg.");
                }
            }
            else
            {
                Serial.println("SensorPollingTask: Failed to request HealthStatusMsg from SensorController.");
            }
        }

        // Delay before the next polling cycle
        vTaskDelay(xDelay500ms);
    }
}
