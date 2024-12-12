// SensorController/tasks/CalibrationTask.cpp

#include "CalibrationTask.h"

#include "SensorWrappers/IMUWrapper.h"
#include "SensorWrappers/MagnetometerWrapper.h"
#include "SensorWrappers/PressureSensorWrapper.h"
#include "SensorWrappers/SonarWrapper.h"

extern bool calibrationDone;
extern SemaphoreHandle_t calibrationMutex;

void CalibrationTask::run(void *parameter)
{
    Serial.println("CalibrationTask: Starting sensor calibration.");

    // Calibrate IMU
    if (!IMUWrapper::calibrate())
    {
        Serial.println("CalibrationTask: IMU calibration failed.");
        while (1)
            ; // Halt on failure
    }

    // Calibrate Magnetometer
    if (!MagnetometerWrapper::calibrate())
    {
        Serial.println("CalibrationTask: Magnetometer calibration failed.");
        while (1)
            ; // Halt on failure
    }

    // Calibrate Pressure Sensor
    if (!PressureSensorWrapper::calibrate())
    {
        Serial.println("CalibrationTask: Pressure sensor calibration failed.");
        while (1)
            ; // Halt on failure
    }

    // Calibrate Forward Sonar
    if (!SonarWrapper::calibrateForwardSonar())
    {
        Serial.println("CalibrationTask: Forward sonar calibration failed.");
        while (1)
            ; // Halt on failure
    }

    // Calibrate Rotating Sonar
    if (!SonarWrapper::calibrateRotatingSonar())
    {
        Serial.println("CalibrationTask: Rotating sonar calibration failed.");
        while (1)
            ; // Halt on failure
    }

    Serial.println("CalibrationTask: All sensors calibrated successfully.");

    // Set calibrationDone flag
    if (xSemaphoreTake(calibrationMutex, portMAX_DELAY) == pdTRUE)
    {
        calibrationDone = true;
        xSemaphoreGive(calibrationMutex);
    }
    else
    {
        Serial.println("CalibrationTask: Failed to set calibrationDone flag.");
        while (1)
            ;
    }

    // Delete this task as calibration is complete
    Serial.println("CalibrationTask: Calibration complete. Deleting task.");
    vTaskDelete(NULL);
}
