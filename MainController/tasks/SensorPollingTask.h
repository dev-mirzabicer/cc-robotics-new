// MainController/tasks/SensorPollingTask.h

#ifndef SENSOR_POLLING_TASK_H
#define SENSOR_POLLING_TASK_H

#include "CommonLib/messages.h"
#include "config.h"
#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>

/**
 * @class SensorPollingTask
 * @brief Task responsible for periodically requesting sensor data, occupancy grid data, and health status from SensorController.
 */
class SensorPollingTask
{
public:
    /**
     * @brief The main function that runs the SensorPollingTask.
     * @param parameter Task parameters (unused).
     */
    static void run(void *parameter);
};

#endif // SENSOR_POLLING_TASK_H
