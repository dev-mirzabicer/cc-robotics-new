// MainController/tasks/RemoteCommunicationTask.h

#ifndef REMOTE_COMMUNICATION_TASK_H
#define REMOTE_COMMUNICATION_TASK_H

#include "CommonLib/messages.h"
#include "config.h"
#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>

/**
 * @class RemoteCommunicationTask
 * @brief Task responsible for receiving remote commands, including PathMsg, and enqueuing them into pathQueue.
 */
class RemoteCommunicationTask
{
public:
    /**
     * @brief The main function that runs the RemoteCommunicationTask.
     *        It listens for incoming PathMsg and enqueues them into pathQueue.
     * @param parameter Task parameters (unused).
     */
    static void run(void *parameter);

private:
    /**
     * @brief Processes the received PathMsg and enqueues it into pathQueue.
     * @param pathMsg The received PathMsg.
     */
    static void processPathMessage(const PathMsg &pathMsg);
};

#endif // REMOTE_COMMUNICATION_TASK_H
