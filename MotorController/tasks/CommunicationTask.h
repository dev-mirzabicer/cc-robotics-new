// MotorController/tasks/CommunicationTask.h

#ifndef MOTOR_COMMUNICATION_TASK_H
#define MOTOR_COMMUNICATION_TASK_H

#include "CommonLib/messages.h"
#include "config.h"
#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>

/**
 * @class CommunicationTask
 * @brief Handles inter-ESP communication from MainController to MotorController.
 *        Specifically, receives VelocityCommandMsg and PumpCommandMsg.
 */
class CommunicationTask
{
public:
    /**
     * @brief The main function that runs the CommunicationTask.
     *        It handles receiving messages from MainController.
     * @param parameter Task parameters (unused).
     */
    static void run(void *parameter);

private:
    // No private members needed as CommunicationTask relies on I2C callbacks
};

#endif // MOTOR_COMMUNICATION_TASK_H
