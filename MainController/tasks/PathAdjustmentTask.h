// MainController/tasks/PathAdjustmentTask.h

#ifndef PATH_ADJUSTMENT_TASK_H
#define PATH_ADJUSTMENT_TASK_H

#include "CommonLib/messages.h"
#include "config.h"
#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>

/**
 * @class PathAdjustmentTask
 * @brief Task responsible for adjusting incoming PathMsg, e.g., for obstacle avoidance, and enqueuing adjusted paths.
 */
class PathAdjustmentTask
{
public:
    /**
     * @brief The main function that runs the PathAdjustmentTask.
     *        It dequeues PathMsg from pathQueue, adjusts the path, and enqueues to adjustedPathQueue.
     * @param parameter Task parameters (unused).
     */
    static void run(void *parameter);

private:
    /**
     * @brief Adjusts the given PathMsg for obstacle avoidance or path smoothing.
     * @param originalPath The original PathMsg received.
     * @param adjustedPath The adjusted PathMsg to be enqueued.
     */
    static void adjustPath(const PathMsg &originalPath, PathMsg &adjustedPath);
};

#endif // PATH_ADJUSTMENT_TASK_H
