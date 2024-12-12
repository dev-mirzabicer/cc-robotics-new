// SensorController/main.cpp

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
#include "tasks/SensorReadTask.h"
#include "tasks/PreProcessingTask.h"
#include "tasks/CommunicationTask.h"
#include "tasks/OccupancyAdjustTask.h"
#include "tasks/HealthMonitoringTask.h"
#include "tasks/CalibrationTask.h"

// Global Queues
QueueHandle_t sensorDataQueue;        // From SensorReadTask to PreProcessingTask
QueueHandle_t preprocessedDataQueue;  // From PreProcessingTask to OccupancyAdjustTask
QueueHandle_t communicationDataQueue; // From PreProcessingTask to CommunicationTask
QueueHandle_t healthStatusQueue;      // From HealthMonitoringTask to CommunicationTask

// Occupancy Grid
struct OccupancyGrid
{
    uint16_t width;
    uint16_t height;
    float resolution;
    uint8_t grid_data[OCCUPANCY_GRID_WIDTH * OCCUPANCY_GRID_HEIGHT];
    int origin_x;
    int origin_y;
};

OccupancyGrid occupancyGrid;
SemaphoreHandle_t occupancyGridMutex;

bool calibrationDone = false;
SemaphoreHandle_t calibrationMutex = NULL;

// I2C Callbacks
extern "C"
{
    void onReceive(int numBytes); // For receiving requests
    void onRequest();             // For sending responses
}

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

void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
        ;
    }
    Serial.println("SensorController: Initializing...");

    // Initialize ConfigManager
    ConfigManager::init();

    // Initialize I2C as slave
    Wire.begin(SENSOR_CONTROLLER_I2C_ADDRESS);
    Wire.onReceive(onReceive);
    Wire.onRequest(onRequest);
    Serial.print("SensorController: I2C initialized as slave at address 0x");
    Serial.println(SENSOR_CONTROLLER_I2C_ADDRESS, HEX);

    // Create queues
    sensorDataQueue = xQueueCreate(SENSOR_DATA_QUEUE_SIZE, sizeof(SensorDataMsg));
    if (!sensorDataQueue)
    {
        Serial.println("SensorController: Failed to create sensorDataQueue.");
        while (1)
            ;
    }
    Serial.println("SensorController: sensorDataQueue created.");

    preprocessedDataQueue = xQueueCreate(PREPROCESSED_DATA_QUEUE_SIZE, sizeof(SensorDataMsg));
    if (!preprocessedDataQueue)
    {
        Serial.println("SensorController: Failed to create preprocessedDataQueue.");
        while (1)
            ;
    }
    Serial.println("SensorController: preprocessedDataQueue created.");

    communicationDataQueue = xQueueCreate(COMMUNICATION_DATA_QUEUE_SIZE, sizeof(SensorDataMsg));
    if (!communicationDataQueue)
    {
        Serial.println("SensorController: Failed to create communicationDataQueue.");
        while (1)
            ;
    }
    Serial.println("SensorController: communicationDataQueue created.");

    healthStatusQueue = xQueueCreate(HEALTH_STATUS_QUEUE_SIZE, sizeof(HealthStatusMsg));
    if (!healthStatusQueue)
    {
        Serial.println("SensorController: Failed to create healthStatusQueue.");
        while (1)
            ;
    }
    Serial.println("SensorController: healthStatusQueue created.");

    // Initialize occupancy grid
    occupancyGrid.width = OCCUPANCY_GRID_WIDTH;
    occupancyGrid.height = OCCUPANCY_GRID_HEIGHT;
    occupancyGrid.resolution = OCCUPANCY_GRID_RESOLUTION;
    for (int i = 0; i < (OCCUPANCY_GRID_WIDTH * OCCUPANCY_GRID_HEIGHT); i++)
    {
        occupancyGrid.grid_data[i] = 0;
    }
    occupancyGrid.origin_x = OCCUPANCY_GRID_WIDTH / 2;
    occupancyGrid.origin_y = OCCUPANCY_GRID_HEIGHT / 2;

    occupancyGridMutex = xSemaphoreCreateMutex();
    if (!occupancyGridMutex)
    {
        Serial.println("SensorController: Failed to create occupancyGridMutex.");
        while (1)
            ;
    }
    Serial.println("SensorController: occupancyGridMutex created.");

    calibrationMutex = xSemaphoreCreateMutex();
    if (!calibrationMutex)
    {
        Serial.println("SensorController: Failed to create calibrationMutex.");
        while (1)
            ;
    }
    calibrationDone = false;
    Serial.println("SensorController: calibrationMutex created.");

    // Create tasks
    BaseType_t taskCreated;

    taskCreated = xTaskCreate(CalibrationTask::run, "Calibration",
                              TASK_STACK_SIZE_CALIBRATION, NULL,
                              TASK_PRIORITY_CALIBRATION, NULL);
    if (taskCreated != pdPASS)
    {
        Serial.println("SensorController: Failed to create CalibrationTask.");
        while (1)
            ;
    }
    Serial.println("SensorController: CalibrationTask created.");

    taskCreated = xTaskCreate(SensorReadTask::run, "SensorRead",
                              TASK_STACK_SIZE_SENSOR_READ, NULL,
                              TASK_PRIORITY_SENSOR_READ, NULL);
    if (taskCreated != pdPASS)
    {
        Serial.println("SensorController: Failed to create SensorReadTask.");
        while (1)
            ;
    }
    Serial.println("SensorController: SensorReadTask created.");

    taskCreated = xTaskCreate(PreProcessingTask::run, "PreProcessing",
                              TASK_STACK_SIZE_PRE_PROCESSING, NULL,
                              TASK_PRIORITY_PRE_PROCESSING, NULL);
    if (taskCreated != pdPASS)
    {
        Serial.println("SensorController: Failed to create PreProcessingTask.");
        while (1)
            ;
    }
    Serial.println("SensorController: PreProcessingTask created.");

    taskCreated = xTaskCreate(OccupancyAdjustTask::run, "OccupancyAdjust",
                              TASK_STACK_SIZE_OCCUPANCY_ADJUST, NULL,
                              TASK_PRIORITY_OCCUPANCY_ADJUST, NULL);
    if (taskCreated != pdPASS)
    {
        Serial.println("SensorController: Failed to create OccupancyAdjustTask.");
        while (1)
            ;
    }
    Serial.println("SensorController: OccupancyAdjustTask created.");

    taskCreated = xTaskCreate(CommunicationTask::run, "Communication",
                              TASK_STACK_SIZE_COMMUNICATION, NULL,
                              TASK_PRIORITY_COMMUNICATION, NULL);
    if (taskCreated != pdPASS)
    {
        Serial.println("SensorController: Failed to create CommunicationTask.");
        while (1)
            ;
    }
    Serial.println("SensorController: CommunicationTask created.");

    taskCreated = xTaskCreate(HealthMonitoringTask::run, "HealthMonitoring",
                              TASK_STACK_SIZE_HEALTH_MONITORING, NULL,
                              TASK_PRIORITY_HEALTH_MONITORING, NULL);
    if (taskCreated != pdPASS)
    {
        Serial.println("SensorController: Failed to create HealthMonitoringTask.");
        while (1)
            ;
    }
    Serial.println("SensorController: HealthMonitoringTask created.");

    Serial.println("SensorController: All tasks created successfully.");
    Serial.println("SensorController: Starting FreeRTOS scheduler.");
    vTaskStartScheduler();
}

void loop()
{
    // Empty, all functionality is handled by FreeRTOS tasks.
}
