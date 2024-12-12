// SensorController/tasks/OccupancyAdjustTask.h

#ifndef OCCUPANCY_ADJUST_TASK_H
#define OCCUPANCY_ADJUST_TASK_H

#include "CommonLib/messages.h"
#include "config.h"
#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>

/**
 * @class OccupancyAdjustTask
 * @brief Task that adjusts the occupancy grid based on preprocessed sensor data.
 */
class OccupancyAdjustTask
{
public:
    static void run(void *parameter);

private:
    static void updateOccupancyGrid(const SensorDataMsg &data);
    static void mapToGrid(float x, float y, int &gx, int &gy);
    static void decayOccupancyGrid();
};

#endif // OCCUPANCY_ADJUST_TASK_H
