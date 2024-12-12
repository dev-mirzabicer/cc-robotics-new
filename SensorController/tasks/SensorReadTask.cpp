// SensorController/tasks/SensorReadTask.cpp

#include "SensorReadTask.h"
#include "config.h"
#include "CommonLib/messages.h"

#include "SensorWrappers/IMUWrapper.h"
#include "SensorWrappers/MagnetometerWrapper.h"
#include "SensorWrappers/PressureSensorWrapper.h"
#include "SensorWrappers/SonarWrapper.h"

extern QueueHandle_t sensorDataQueue;
extern bool calibrationDone;
extern SemaphoreHandle_t calibrationMutex;

void SensorReadTask::run(void *parameter)
{
    Serial.println("SensorReadTask: Starting...");

    // Initialize sensors via wrappers
    if (!IMUWrapper::init())
    {
        Serial.println("SensorReadTask: IMU initialization failed.");
        while (1)
            ;
    }
    Serial.println("SensorReadTask: IMU initialized.");

    if (!MagnetometerWrapper::init())
    {
        Serial.println("SensorReadTask: Magnetometer initialization failed.");
        while (1)
            ;
    }
    Serial.println("SensorReadTask: Magnetometer initialized.");

    if (!PressureSensorWrapper::init())
    {
        Serial.println("SensorReadTask: Pressure sensor initialization failed.");
        while (1)
            ;
    }
    Serial.println("SensorReadTask: Pressure sensor initialized.");

    if (!SonarWrapper::initForwardSonar())
    {
        Serial.println("SensorReadTask: Forward sonar initialization failed.");
        while (1)
            ;
    }
    Serial.println("SensorReadTask: Forward sonar initialized.");

    if (!SonarWrapper::initRotatingSonar())
    {
        Serial.println("SensorReadTask: Rotating sonar initialization failed.");
        while (1)
            ;
    }
    Serial.println("SensorReadTask: Rotating sonar initialized.");

    // Wait for CalibrationTask to complete calibration
    Serial.println("SensorReadTask: Waiting for sensor calibration to complete...");
    while (true)
    {
        if (xSemaphoreTake(calibrationMutex, portMAX_DELAY) == pdTRUE)
        {
            bool done = calibrationDone;
            xSemaphoreGive(calibrationMutex);
            if (done)
                break; // Calibration complete, proceed to reading
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    Serial.println("SensorReadTask: Calibration done, starting sensor readings.");

    // Set reading frequency
    const TickType_t readInterval = pdMS_TO_TICKS(100); // 10 Hz reading
    SensorDataMsg sensorData;

    while (true)
    {
        // Read IMU
        sensorData.acceleration_x = IMUWrapper::readAccelerationX();
        sensorData.acceleration_y = IMUWrapper::readAccelerationY();
        sensorData.acceleration_z = IMUWrapper::readAccelerationZ();

        sensorData.gyro_x = IMUWrapper::readGyroX();
        sensorData.gyro_y = IMUWrapper::readGyroY();
        sensorData.gyro_z = IMUWrapper::readGyroZ();

        // Read Magnetometer
        sensorData.magnetometer_x = MagnetometerWrapper::readMagnetometerX();
        sensorData.magnetometer_y = MagnetometerWrapper::readMagnetometerY();
        sensorData.magnetometer_z = MagnetometerWrapper::readMagnetometerZ();

        // Read Pressure
        sensorData.pressure = PressureSensorWrapper::readPressure();

        // Read Forward Sonar
        {
            SonarData fwd = SonarWrapper::readForwardSonar();
            sensorData.sonar_distance_forward = fwd.distance;
            sensorData.sonar_angle_forward = fwd.angle; // Should be 0 as per specification
        }

        // Read Rotating Sonar
        {
            SonarData rot = SonarWrapper::readRotatingSonar();
            sensorData.sonar_distance_rotating = rot.distance;
            sensorData.sonar_angle_rotating = rot.angle;
        }

        // Send to sensorDataQueue
        if (xQueueSend(sensorDataQueue, &sensorData, 0) != pdPASS)
        {
            Serial.println("SensorReadTask: sensorDataQueue full, dropping SensorDataMsg.");
        }

        // Wait for next cycle
        vTaskDelay(readInterval);
    }
}
