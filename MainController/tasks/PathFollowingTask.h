// MainController/tasks/PathFollowingTask.h

#ifndef PATH_FOLLOWING_TASK_H
#define PATH_FOLLOWING_TASK_H

#include "CommonLib/messages.h"
#include "config.h"
#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>

/**
 * @class PathFollowingTask
 * @brief Task responsible for following the submarine's path by computing velocity and pump commands.
 */
class PathFollowingTask
{
public:
    /**
     * @brief The main function that runs the PathFollowingTask.
     *        It consumes adjusted PathMsg and computes commands accordingly.
     * @param parameter Task parameters (unused).
     */
    static void run(void *parameter);

private:
    /**
     * @brief Sends a VelocityCommandMsg to the CommunicationTask.
     * @param linearVel Linear velocity in m/s.
     * @param angularVel Angular velocity in rad/s.
     */
    static void sendVelocityCommand(float linearVel, float angularVel);

    /**
     * @brief Sends a PumpCommandMsg to the CommunicationTask.
     * @param intakePump On/Off state for intake pump.
     * @param outflowPump On/Off state for outflow pump.
     */
    static void sendPumpCommand(bool intakePump, bool outflowPump);

    // PID Controller Variables for Depth
    static float depthIntegral;
    static float depthLastError;
};

#endif // PATH_FOLLOWING_TASK_H
