// SensorController/tasks/PreProcessingTask.cpp

#include "PreProcessingTask.h"

extern QueueHandle_t sensorDataQueue;
extern QueueHandle_t preprocessedDataQueue;
extern QueueHandle_t communicationDataQueue;

void PreProcessingTask::run(void *parameter)
{
    Serial.println("PreProcessingTask: Starting...");

    SensorDataMsg rawData;

    while (true)
    {
        if (xQueueReceive(sensorDataQueue, &rawData, portMAX_DELAY) == pdPASS)
        {
            preprocessData(rawData);

            // Send to preprocessedDataQueue
            if (xQueueSend(preprocessedDataQueue, &rawData, 0) != pdPASS)
            {
                Serial.println("PreProcessingTask: preprocessedDataQueue full, dropping message.");
            }

            // Also send to communicationDataQueue
            if (xQueueSend(communicationDataQueue, &rawData, 0) != pdPASS)
            {
                Serial.println("PreProcessingTask: communicationDataQueue full, dropping message.");
            }
        }
        taskYIELD();
    }
}

void PreProcessingTask::preprocessData(SensorDataMsg &data)
{
    // Apply low-pass filter example:
    // Implemented as a simple exponential moving average
    static bool firstRun = true;
    static SensorDataMsg prev;
    if (firstRun)
    {
        prev = data;
        firstRun = false;
    }

    float alpha = 0.9f; // Low-pass filter coefficient

    // Apply filtering to acceleration
    data.acceleration_x = alpha * prev.acceleration_x + (1.0f - alpha) * data.acceleration_x;
    data.acceleration_y = alpha * prev.acceleration_y + (1.0f - alpha) * data.acceleration_y;
    data.acceleration_z = alpha * prev.acceleration_z + (1.0f - alpha) * data.acceleration_z;

    // Apply filtering to gyroscope
    data.gyro_x = alpha * prev.gyro_x + (1.0f - alpha) * data.gyro_x;
    data.gyro_y = alpha * prev.gyro_y + (1.0f - alpha) * data.gyro_y;
    data.gyro_z = alpha * prev.gyro_z + (1.0f - alpha) * data.gyro_z;

    // Magnetometer, pressure, and sonars are left unfiltered for simplicity
    // Additional filtering can be implemented similarly if desired

    prev = data; // Update previous data for next iteration
}
