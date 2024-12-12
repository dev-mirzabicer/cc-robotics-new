// SensorController/config.h

#ifndef SENSOR_CONTROLLER_CONFIG_H
#define SENSOR_CONTROLLER_CONFIG_H

#include "CommonLib/config_common.h"

// =========================
// Task Stack Sizes & Priorities
// =========================

#define TASK_STACK_SIZE_SENSOR_READ 4096
#define TASK_PRIORITY_SENSOR_READ 3

#define TASK_STACK_SIZE_PRE_PROCESSING 4096
#define TASK_PRIORITY_PRE_PROCESSING 3

#define TASK_STACK_SIZE_COMMUNICATION 2048
#define TASK_PRIORITY_COMMUNICATION 2

#define TASK_STACK_SIZE_OCCUPANCY_ADJUST 2048
#define TASK_PRIORITY_OCCUPANCY_ADJUST 2

#define TASK_STACK_SIZE_HEALTH_MONITORING 2048
#define TASK_PRIORITY_HEALTH_MONITORING 1

#define TASK_STACK_SIZE_CALIBRATION 2048
#define TASK_PRIORITY_CALIBRATION 2

// =========================
// Queue Sizes for SensorController
// =========================

#define SENSOR_DATA_QUEUE_SIZE 10
#define PREPROCESSED_DATA_QUEUE_SIZE 10
#define COMMUNICATION_DATA_QUEUE_SIZE 10
#define HEALTH_STATUS_QUEUE_SIZE 5

// =========================
// Occupancy Grid Parameters
// =========================

#define OCCUPANCY_GRID_WIDTH 100
#define OCCUPANCY_GRID_HEIGHT 100
#define OCCUPANCY_GRID_RESOLUTION 0.1f // meters per cell
#define OBSTACLE_RADIUS 2

// =========================
// Sensor Configuration
// =========================

// Assuming sensor wrappers handle any pin assignments and initialization internally.
// No placeholders: rely entirely on sensor wrappers.

// =========================
// I2C Address
// =========================

#define SENSOR_CONTROLLER_I2C_ADDRESS 0x05 // Ensure this matches config_common.h

#endif // SENSOR_CONTROLLER_CONFIG_H
