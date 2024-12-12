// SensorController/tasks/HealthMonitoringTask.h

#ifndef HEALTH_MONITORING_TASK_H
#define HEALTH_MONITORING_TASK_H

#include "CommonLib/messages.h"
#include "config.h"
#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>

/**
 * @class HealthMonitoringTask
 * @brief Task responsible for monitoring the health of sensors and system components.
 */
class HealthMonitoringTask
{
public:
    static void run(void *parameter);

private:
    static void monitorHealth();
};

#endif // HEALTH_MONITORING_TASK_H
