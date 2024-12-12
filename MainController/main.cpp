// MainController/main.cpp

#include <Arduino.h>
#include "config.h"
#include "CommonLib/config_common.h"
#include "CommonLib/messages.h"
#include "CommonLib/config_manager.h"

#include <Wire.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

// Include task headers
#include "tasks/StateEstimationTask.h"
#include "tasks/PathAdjustmentTask.h"
#include "tasks/CommunicationTask.h"
#include "tasks/RemoteCommunicationTask.h"
#include "tasks/HealthMonitoringTask.h"
// YawControlTask removed since yaw is handled in PathFollowingTask
#include "tasks/PathFollowingTask.h"
#include "tasks/SensorPollingTask.h"

// Global Variables
State currentState = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
SemaphoreHandle_t stateMutex;

QueueHandle_t sensorDataQueue;
QueueHandle_t pathQueue;
QueueHandle_t occupancyGridQueue;
QueueHandle_t adjustedPathQueue;
QueueHandle_t healthStatusQueue;
QueueHandle_t motorVelocityCmdQueue;
QueueHandle_t pumpCmdQueue;

// Optional: I2C Mutex if needed for protection of I2C bus
SemaphoreHandle_t i2cMutex = NULL;

// FreeRTOS Stack Overflow Hook
extern "C"
{
    void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
    {
        Serial.print("Stack overflow in task: ");
        Serial.println(pcTaskName);
        // Take appropriate action, such as resetting the system
        ESP.restart();
    }
}

void setup()
{
    // Initialize Serial for debugging
    Serial.begin(115200);
    while (!Serial)
    {
        ;
    }
    Serial.println("MainController: Initializing...");

    // Initialize ConfigManager
    ConfigManager::init();

    // Initialize I2C as master
    Wire.begin(I2C_MASTER_SDA_PIN, I2C_MASTER_SCL_PIN);
    Wire.setClock(400000); // 400kHz for faster I2C if needed
    Serial.println("MainController: I2C initialized as master.");

    // Create mutexes
    stateMutex = xSemaphoreCreateMutex();
    if (stateMutex == NULL)
    {
        Serial.println("MainController: Failed to create state mutex.");
        while (1)
            ;
    }
    Serial.println("MainController: State mutex created.");

    i2cMutex = xSemaphoreCreateMutex();
    if (i2cMutex == NULL)
    {
        Serial.println("MainController: Failed to create i2cMutex.");
        while (1)
            ;
    }
    Serial.println("MainController: i2cMutex created.");

    // Create queues
    sensorDataQueue = xQueueCreate(SENSOR_DATA_QUEUE_SIZE, sizeof(SensorDataMsg));
    if (!sensorDataQueue)
    {
        Serial.println("Failed to create sensorDataQueue.");
        while (1)
            ;
    }
    Serial.println("MainController: sensorDataQueue created.");

    pathQueue = xQueueCreate(PATH_QUEUE_SIZE, sizeof(PathMsg));
    if (!pathQueue)
    {
        Serial.println("Failed to create pathQueue.");
        while (1)
            ;
    }
    Serial.println("MainController: pathQueue created.");

    occupancyGridQueue = xQueueCreate(OCCUPANCY_GRID_QUEUE_SIZE, sizeof(OccupancyGridMsg));
    if (!occupancyGridQueue)
    {
        Serial.println("Failed to create occupancyGridQueue.");
        while (1)
            ;
    }
    Serial.println("MainController: occupancyGridQueue created.");

    adjustedPathQueue = xQueueCreate(ADJUSTED_PATH_QUEUE_SIZE, sizeof(PathMsg));
    if (!adjustedPathQueue)
    {
        Serial.println("Failed to create adjustedPathQueue.");
        while (1)
            ;
    }
    Serial.println("MainController: adjustedPathQueue created.");

    healthStatusQueue = xQueueCreate(HEALTH_STATUS_QUEUE_SIZE, sizeof(HealthStatusMsg));
    if (!healthStatusQueue)
    {
        Serial.println("Failed to create healthStatusQueue.");
        while (1)
            ;
    }
    Serial.println("MainController: healthStatusQueue created.");

    motorVelocityCmdQueue = xQueueCreate(MOTOR_VELOCITY_CMD_QUEUE_SIZE, sizeof(VelocityCommandMsg));
    if (!motorVelocityCmdQueue)
    {
        Serial.println("Failed to create motorVelocityCmdQueue.");
        while (1)
            ;
    }
    Serial.println("MainController: motorVelocityCmdQueue created.");

    pumpCmdQueue = xQueueCreate(PUMP_CMD_QUEUE_SIZE, sizeof(PumpCommandMsg));
    if (!pumpCmdQueue)
    {
        Serial.println("Failed to create pumpCmdQueue.");
        while (1)
            ;
    }
    Serial.println("MainController: pumpCmdQueue created.");

    // Create tasks
    BaseType_t taskCreated;

    taskCreated = xTaskCreate(StateEstimationTask::run, "StateEstimation",
                              TASK_STACK_SIZE_STATE_ESTIMATION, NULL,
                              TASK_PRIORITY_STATE_ESTIMATION, NULL);
    if (taskCreated != pdPASS)
    {
        Serial.println("Failed to create StateEstimationTask.");
        while (1)
            ;
    }
    Serial.println("MainController: StateEstimationTask created.");

    taskCreated = xTaskCreate(PathAdjustmentTask::run, "PathAdjustment",
                              TASK_STACK_SIZE_PATH_ADJUSTMENT, NULL,
                              TASK_PRIORITY_PATH_ADJUSTMENT, NULL);
    if (taskCreated != pdPASS)
    {
        Serial.println("Failed to create PathAdjustmentTask.");
        while (1)
            ;
    }
    Serial.println("MainController: PathAdjustmentTask created.");

    taskCreated = xTaskCreate(CommunicationTask::run, "Communication",
                              TASK_STACK_SIZE_COMMUNICATION, NULL,
                              TASK_PRIORITY_COMMUNICATION, NULL);
    if (taskCreated != pdPASS)
    {
        Serial.println("Failed to create CommunicationTask.");
        while (1)
            ;
    }
    Serial.println("MainController: CommunicationTask created.");

    taskCreated = xTaskCreate(RemoteCommunicationTask::run, "RemoteCommunication",
                              TASK_STACK_SIZE_REMOTE_COMMUNICATION, NULL,
                              TASK_PRIORITY_REMOTE_COMMUNICATION, NULL);
    if (taskCreated != pdPASS)
    {
        Serial.println("Failed to create RemoteCommunicationTask.");
        while (1)
            ;
    }
    Serial.println("MainController: RemoteCommunicationTask created.");

    taskCreated = xTaskCreate(HealthMonitoringTask::run, "HealthMonitoring",
                              TASK_STACK_SIZE_HEALTH_MONITORING, NULL,
                              TASK_PRIORITY_HEALTH_MONITORING, NULL);
    if (taskCreated != pdPASS)
    {
        Serial.println("Failed to create HealthMonitoringTask.");
        while (1)
            ;
    }
    Serial.println("MainController: HealthMonitoringTask created.");

    taskCreated = xTaskCreate(PathFollowingTask::run, "PathFollowing",
                              TASK_STACK_SIZE_PATH_FOLLOWING, NULL,
                              TASK_PRIORITY_PATH_FOLLOWING, NULL);
    if (taskCreated != pdPASS)
    {
        Serial.println("Failed to create PathFollowingTask.");
        while (1)
            ;
    }
    Serial.println("MainController: PathFollowingTask created.");

    taskCreated = xTaskCreate(SensorPollingTask::run, "SensorPolling",
                              2048, NULL, 2, NULL);
    if (taskCreated != pdPASS)
    {
        Serial.println("Failed to create SensorPollingTask.");
        while (1)
            ;
    }
    Serial.println("MainController: SensorPollingTask created.");

    Serial.println("MainController: Starting FreeRTOS scheduler.");
    vTaskStartScheduler();
}

void loop()
{
    // Empty. All functionality is handled by FreeRTOS tasks.
}
