// SensorController/tasks/PreProcessingTask.h

#ifndef PRE_PROCESSING_TASK_H
#define PRE_PROCESSING_TASK_H

#include "CommonLib/messages.h"
#include "config.h"
#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>

/**
 * @class PreProcessingTask
 * @brief Preprocesses raw sensor data (calibration offsets, filtering) and sends to preprocessed and communication queues.
 */
class PreProcessingTask
{
public:
    static void run(void *parameter);

private:
    static void preprocessData(SensorDataMsg &data);
};

#endif // PRE_PROCESSING_TASK_H
