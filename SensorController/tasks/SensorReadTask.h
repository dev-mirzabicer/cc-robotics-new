// SensorController/tasks/SensorReadTask.h

#ifndef SENSOR_READ_TASK_H
#define SENSOR_READ_TASK_H

#include "CommonLib/messages.h"
#include "config.h"
#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>

/**
 * @class SensorReadTask
 * @brief Task that reads raw sensor data from IMU, magnetometer, pressure sensor, and sonars
 *        using provided wrappers and sends them to sensorDataQueue.
 */
class SensorReadTask
{
public:
    static void run(void *parameter);
};

#endif // SENSOR_READ_TASK_H
