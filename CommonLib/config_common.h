// CommonLib/config_common.h

#ifndef CONFIG_COMMON_H
#define CONFIG_COMMON_H

#include <Arduino.h>

// =========================
// I2C Addresses
// =========================

// Unique I2C addresses for each ESP32 module
#define MAIN_CONTROLLER_I2C_ADDRESS 0x10
#define MOTOR_CONTROLLER_I2C_ADDRESS 0x04
#define SENSOR_CONTROLLER_I2C_ADDRESS 0x05 // Example address; adjust as needed

// =========================
// Communication Parameters
// =========================

// Inter-ESP Communication (I2C) Baud Rates are managed by Wire library
// No separate baud rate for I2C, but you can set clock speed in InterESPProtocol::init()

// =========================
// Unit Definitions
// =========================

#define UNIT_LINEAR_VELOCITY "m/s"
#define UNIT_ANGULAR_VELOCITY "rad/s"
#define UNIT_DISTANCE "meters"
#define UNIT_ANGLE "radians"
#define UNIT_PRESSURE "kPa"
#define UNIT_ACCELERATION "m/s^2"
#define UNIT_GYRO "rad/s"
#define UNIT_MAGNETOMETER "µT"

// =========================
// Task Stack Sizes and Priorities
// =========================

// Define stack sizes (in words) and priorities for FreeRTOS tasks
// Adjust these values based on actual task requirements and available memory

// MainController Task Configurations
#define TASK_STACK_SIZE_STATE_ESTIMATION 4096
#define TASK_PRIORITY_STATE_ESTIMATION 3

#define TASK_STACK_SIZE_PATH_ADJUSTMENT 4096
#define TASK_PRIORITY_PATH_ADJUSTMENT 3

#define TASK_STACK_SIZE_COMMUNICATION 2048
#define TASK_PRIORITY_COMMUNICATION 2

#define TASK_STACK_SIZE_REMOTE_COMMUNICATION 2048
#define TASK_PRIORITY_REMOTE_COMMUNICATION 2

#define TASK_STACK_SIZE_HEALTH_MONITORING 2048
#define TASK_PRIORITY_HEALTH_MONITORING 1

#define TASK_STACK_SIZE_YAW_CONTROL 2048
#define TASK_PRIORITY_YAW_CONTROL 3

#define TASK_STACK_SIZE_PATH_FOLLOWING 4096
#define TASK_PRIORITY_PATH_FOLLOWING 3

// MotorController Task Configurations
#define TASK_STACK_SIZE_MOTOR_CONTROL 4096
#define TASK_PRIORITY_MOTOR_CONTROL 3

#define TASK_STACK_SIZE_PUMP_CONTROL 2048
#define TASK_PRIORITY_PUMP_CONTROL 2

#define TASK_STACK_SIZE_MOTOR_COMMUNICATION 2048
#define TASK_PRIORITY_MOTOR_COMMUNICATION 2

// SensorController Task Configurations
#define TASK_STACK_SIZE_SENSOR_READ 4096
#define TASK_PRIORITY_SENSOR_READ 3

#define TASK_STACK_SIZE_PRE_PROCESSING 4096
#define TASK_PRIORITY_PRE_PROCESSING 3

#define TASK_STACK_SIZE_OCCUPANCY_ADJUST 2048
#define TASK_PRIORITY_OCCUPANCY_ADJUST 2

#define TASK_STACK_SIZE_COMMUNICATION 2048
#define TASK_PRIORITY_COMMUNICATION 2

#define TASK_STACK_SIZE_HEALTH_MONITORING 2048
#define TASK_PRIORITY_HEALTH_MONITORING 1

#define TASK_STACK_SIZE_CALIBRATION 2048
#define TASK_PRIORITY_CALIBRATION 2

// =========================
// Occupancy Grid Parameters
// =========================

#define OCCUPANCY_GRID_WIDTH 100       // Number of cells in X-direction
#define OCCUPANCY_GRID_HEIGHT 100      // Number of cells in Y-direction
#define OCCUPANCY_GRID_RESOLUTION 0.1f // Meters per cell

#define OBSTACLE_RADIUS 2 // Cells to mark around an obstacle

// =========================
// PID Parameters (Defaults)
// =========================

// Default PID coefficients (can be overridden via ConfigManager)
#define DEFAULT_KP_MOTOR 1.0
#define DEFAULT_KI_MOTOR 0.1
#define DEFAULT_KD_MOTOR 0.05

#define DEFAULT_KP_YAW 1.0
#define DEFAULT_KI_YAW 0.1
#define DEFAULT_KD_YAW 0.05

// =========================
// Additional Configurations
// =========================

// Add any other shared configurations below

#endif // CONFIG_COMMON_H
