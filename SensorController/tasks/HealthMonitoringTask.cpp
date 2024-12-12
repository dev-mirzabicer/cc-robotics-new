// SensorController/tasks/HealthMonitoringTask.cpp

#include "HealthMonitoringTask.h"

#include "SensorWrappers/IMUWrapper.h"
#include "SensorWrappers/MagnetometerWrapper.h"
#include "SensorWrappers/PressureSensorWrapper.h"
#include "SensorWrappers/SonarWrapper.h"

extern QueueHandle_t healthStatusQueue;

void HealthMonitoringTask::run(void *parameter)
{
    Serial.println("HealthMonitoringTask: Starting...");
    while (true)
    {
        monitorHealth();
        vTaskDelay(pdMS_TO_TICKS(1000)); // Check health every second
    }
}

void HealthMonitoringTask::monitorHealth()
{
    HealthStatusMsg status;
    status.imu_health = IMUWrapper::isHealthy();
    status.magnetometer_health = MagnetometerWrapper::isHealthy();
    status.pressure_sensor_health = PressureSensorWrapper::isHealthy();
    status.sonar_forward_health = SonarWrapper::isForwardSonarHealthy();
    status.sonar_rotating_health = SonarWrapper::isRotatingSonarHealthy();

    if (xQueueSend(healthStatusQueue, &status, 0) != pdPASS)
    {
        Serial.println("HealthMonitoringTask: healthStatusQueue full, dropping HealthStatusMsg.");
    }
    else
    {
        Serial.println("HealthMonitoringTask: Sent HealthStatusMsg to healthStatusQueue.");
    }
}
