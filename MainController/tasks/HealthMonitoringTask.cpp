// MainController/tasks/HealthMonitoringTask.cpp

#include "HealthMonitoringTask.h"

extern QueueHandle_t healthStatusQueue;

// Consider sensors unhealthy if no health updates for > 5 seconds
static const TickType_t HEALTH_TIMEOUT = pdMS_TO_TICKS(5000);

bool HealthMonitoringTask::allSensorsHealthy = false;
TickType_t HealthMonitoringTask::lastHealthUpdate = 0;

void HealthMonitoringTask::run(void *parameter)
{
    Serial.println("HealthMonitoringTask: Starting...");
    lastHealthUpdate = xTaskGetTickCount();

    while (true)
    {
        monitorHealth();
        vTaskDelay(pdMS_TO_TICKS(1000)); // Check health every 1s
    }
}

void HealthMonitoringTask::monitorHealth()
{
    // Check if we have a new HealthStatusMsg
    HealthStatusMsg status;
    if (xQueueReceive(healthStatusQueue, &status, 0) == pdPASS)
    {
        // Received a new health status
        lastHealthUpdate = xTaskGetTickCount();
        allSensorsHealthy = status.imu_health && status.magnetometer_health &&
                            status.pressure_sensor_health && status.sonar_forward_health && status.sonar_rotating_health;

        if (!allSensorsHealthy)
        {
            Serial.println("HealthMonitoringTask: One or more sensors reported unhealthy status.");
        }
        else
        {
            Serial.println("HealthMonitoringTask: All sensors healthy.");
        }
    }

    // Check if too long since last update
    TickType_t now = xTaskGetTickCount();
    if ((now - lastHealthUpdate) > HEALTH_TIMEOUT)
    {
        Serial.println("HealthMonitoringTask: No health updates received recently. Considering sensors unhealthy.");
        allSensorsHealthy = false;
    }
}
