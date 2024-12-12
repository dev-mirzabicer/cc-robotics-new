// SensorController/tasks/CalibrationTask.h

#ifndef CALIBRATION_TASK_H
#define CALIBRATION_TASK_H

#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>

/**
 * @class CalibrationTask
 * @brief Performs sensor calibration before other tasks proceed.
 */
class CalibrationTask
{
public:
    static void run(void *parameter);
};

#endif // CALIBRATION_TASK_H
