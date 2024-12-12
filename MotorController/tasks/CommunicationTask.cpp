// MotorController/tasks/CommunicationTask.cpp

#include "CommunicationTask.h"
#include "config.h"
#include "CommonLib/messages.h"
#include "CommonLib/InterESPProtocol.h"
#include <Arduino.h>

// External queues defined in main.cpp
extern QueueHandle_t mainVelocityCmdQueue;
extern QueueHandle_t mainPumpCmdQueue;

/**
 * @brief The main function that runs the CommunicationTask.
 *        In this I2C slave setup, all communication is handled via I2C callbacks.
 *        This task can remain empty or be used for additional communication handling if needed.
 * @param parameter Task parameters (unused).
 */
void CommunicationTask::run(void *parameter)
{
    Serial.println("CommunicationTask (MotorController): Running...");
    while (true)
    {
        // In I2C slave mode, receive messages via onReceive callback
        // No additional handling needed here unless implementing unsolicited communication
        taskYIELD();
    }
}
