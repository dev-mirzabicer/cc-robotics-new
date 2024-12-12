// MotorController/main.cpp

#include <Arduino.h>
#include "config.h"
#include "CommonLib/config_common.h"
#include "CommonLib/messages.h"
#include "CommonLib/config_manager.h"

#include <Wire.h>
#include <FreeRTOS.h>
#include <task.h>

// Include task headers
#include "tasks/MotorControlTask.h"
#include "tasks/PumpControlTask.h"
#include "tasks/CommunicationTask.h"

// Global Queues
QueueHandle_t mainVelocityCmdQueue;
QueueHandle_t mainPumpCmdQueue;

// FreeRTOS Stack Overflow Hook
extern "C"
{
    void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
    {
        Serial.print("Stack overflow in task: ");
        Serial.println(pcTaskName);
        ESP.restart();
    }
}

// I2C Callbacks for MotorController as a slave
extern "C"
{
    void onReceive(int numBytes); // Implemented elsewhere (Communication handling)
    void onRequest();             // Implemented elsewhere (Response if needed)
}

void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
        ;
    }
    Serial.println("MotorController: Initializing...");

    // Initialize ConfigManager
    ConfigManager::init();

    // Initialize I2C as slave
    Wire.begin(MOTOR_CONTROLLER_I2C_ADDRESS);
    // Set callbacks for receive/request
    Wire.onReceive(onReceive);
    Wire.onRequest(onRequest);
    Serial.print("MotorController: I2C initialized as slave at address 0x");
    Serial.println(MOTOR_CONTROLLER_I2C_ADDRESS, HEX);

    // Create queues
    mainVelocityCmdQueue = xQueueCreate(VELOCITY_CMD_QUEUE_SIZE, sizeof(VelocityCommandMsg));
    if (!mainVelocityCmdQueue)
    {
        Serial.println("MotorController: Failed to create mainVelocityCmdQueue.");
        while (1)
            ;
    }
    Serial.println("MotorController: mainVelocityCmdQueue created.");

    mainPumpCmdQueue = xQueueCreate(PUMP_CMD_QUEUE_SIZE, sizeof(PumpCommandMsg));
    if (!mainPumpCmdQueue)
    {
        Serial.println("MotorController: Failed to create mainPumpCmdQueue.");
        while (1)
            ;
    }
    Serial.println("MotorController: mainPumpCmdQueue created.");

    // Create tasks
    BaseType_t taskCreated;

    taskCreated = xTaskCreate(MotorControlTask::run, "MotorControl",
                              TASK_STACK_SIZE_MOTOR_CONTROL, NULL,
                              TASK_PRIORITY_MOTOR_CONTROL, NULL);
    if (taskCreated != pdPASS)
    {
        Serial.println("MotorController: Failed to create MotorControlTask.");
        while (1)
            ;
    }
    Serial.println("MotorController: MotorControlTask created.");

    taskCreated = xTaskCreate(PumpControlTask::run, "PumpControl",
                              TASK_STACK_SIZE_PUMP_CONTROL, NULL,
                              TASK_PRIORITY_PUMP_CONTROL, NULL);
    if (taskCreated != pdPASS)
    {
        Serial.println("MotorController: Failed to create PumpControlTask.");
        while (1)
            ;
    }
    Serial.println("MotorController: PumpControlTask created.");

    taskCreated = xTaskCreate(CommunicationTask::run, "Communication",
                              TASK_STACK_SIZE_COMMUNICATION, NULL,
                              TASK_PRIORITY_COMMUNICATION, NULL);
    if (taskCreated != pdPASS)
    {
        Serial.println("MotorController: Failed to create CommunicationTask.");
        while (1)
            ;
    }
    Serial.println("MotorController: CommunicationTask created.");

    Serial.println("MotorController: Starting FreeRTOS scheduler.");
    vTaskStartScheduler();
}

void loop()
{
    // Empty, FreeRTOS tasks manage everything
}
