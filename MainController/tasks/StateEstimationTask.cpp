// MainController/tasks/StateEstimationTask.cpp

#include "StateEstimationTask.h"
#include "CommonLib/config_manager.h"

// External global variables defined in main.cpp
extern State currentState;
extern SemaphoreHandle_t stateMutex;

extern QueueHandle_t sensorDataQueue;

// Constants for sensor fusion
const float dt = 0.1f;     // Time step (seconds), assuming 10 Hz update rate
const float alpha = 0.98f; // Complementary filter coefficient

void StateEstimationTask::run(void *parameter)
{
    Serial.println("StateEstimationTask: Starting...");

    while (1)
    {
        // Wait indefinitely for sensor data
        SensorDataMsg sensorData;
        if (xQueueReceive(sensorDataQueue, &sensorData, portMAX_DELAY) == pdPASS)
        {
            processSensorData(sensorData);
        }

        // Yield to allow other tasks to run
        taskYIELD();
    }
}

/**
 * @brief Processes incoming sensor data to update the submarine's state.
 * @param sensorData Sensor data received from SensorController.
 */
void StateEstimationTask::processSensorData(const SensorDataMsg &sensorData)
{
    // Fetch the current state in a thread-safe manner
    State newState;
    if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE)
    {
        newState = currentState; // Create a copy of the current state
        xSemaphoreGive(stateMutex);
    }
    else
    {
        Serial.println("StateEstimationTask: Failed to acquire state mutex.");
        return;
    }

    // =========================
    // Position Estimation
    // =========================

    // Integrate acceleration to update velocity
    newState.vx += sensorData.acceleration_x * dt;
    newState.vy += sensorData.acceleration_y * dt;
    newState.vz += sensorData.acceleration_z * dt;

    // Integrate velocity to update position
    newState.x += newState.vx * dt;
    newState.y += newState.vy * dt;
    newState.z += newState.vz * dt;

    // =========================
    // Orientation Estimation
    // =========================

    // Integrate gyroscope data to update orientation
    newState.roll += sensorData.gyro_x * dt;
    newState.pitch += sensorData.gyro_y * dt;
    newState.yaw += sensorData.gyro_z * dt;

    // Apply complementary filter to correct yaw using magnetometer data
    newState.yaw = alpha * newState.yaw + (1.0f - alpha) * sensorData.magnetometer_z;

    // Normalize yaw to [-PI, PI]
    if (newState.yaw > PI)
    {
        newState.yaw -= 2.0f * PI;
    }
    if (newState.yaw < -PI)
    {
        newState.yaw += 2.0f * PI;
    }

    // =========================
    // Update Global State
    // =========================

    updateGlobalState(newState);
}

/**
 * @brief Updates the global state with the newly estimated values.
 * @param newState The newly estimated state.
 */
void StateEstimationTask::updateGlobalState(const State &newState)
{
    if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE)
    {
        currentState = newState; // Update the global state
        xSemaphoreGive(stateMutex);
    }
    else
    {
        Serial.println("StateEstimationTask: Failed to acquire state mutex for update.");
    }
}
