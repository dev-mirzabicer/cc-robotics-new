// MainController/config.h

#ifndef MAIN_CONTROLLER_CONFIG_H
#define MAIN_CONTROLLER_CONFIG_H

#include "CommonLib/config_common.h"

// =========================
// I2C Configuration
// =========================

#define I2C_MASTER_SDA_PIN 21 ///< I2C SDA pin number
#define I2C_MASTER_SCL_PIN 22 ///< I2C SCL pin number

// =========================
// Queue Sizes
// =========================

#define SENSOR_DATA_QUEUE_SIZE 10   ///< Maximum number of SensorDataMsg in the queue
#define PATH_QUEUE_SIZE 10          ///< Maximum number of PathMsg in the queue
#define OCCUPANCY_GRID_QUEUE_SIZE 5 ///< Maximum number of OccupancyGridMsg in the queue
#define HEALTH_STATUS_QUEUE_SIZE 5  ///< Maximum number of HealthStatusMsg in the queue

#define MOTOR_VELOCITY_CMD_QUEUE_SIZE 10 ///< Maximum number of VelocityCommandMsg from MotorController
#define PUMP_CMD_QUEUE_SIZE 10           ///< Maximum number of PumpCommandMsg from MotorController

#define ADJUSTED_PATH_QUEUE_SIZE 10 ///< Maximum number of adjusted PathMsg

// =========================
// State Structure
// =========================

struct State
{
    float x;     ///< Position in X-axis (meters)
    float y;     ///< Position in Y-axis (meters)
    float z;     ///< Position in Z-axis (meters)
    float roll;  ///< Roll angle (radians)
    float pitch; ///< Pitch angle (radians)
    float yaw;   ///< Yaw angle (radians)
    float vx;    ///< Velocity in X-axis (m/s)
    float vy;    ///< Velocity in Y-axis (m/s)
    float vz;    ///< Velocity in Z-axis (m/s)
    // Additional state variables can be added here
};

// =========================
// PID Constants for Depth Control
// =========================

#define Kp_DEPTH 1.0f  ///< Proportional gain for depth control
#define Ki_DEPTH 0.1f  ///< Integral gain for depth control
#define Kd_DEPTH 0.05f ///< Derivative gain for depth control

#define MAX_DEPTH_INTEGRAL 10.0f ///< Maximum integral term to prevent windup
#define DEPTH_CONTROL_DT 0.1f    ///< Time step for PID calculations (seconds)

// =========================
// State Following Parameters
// =========================

#define MAX_FORWARD_SPEED 1.0f  ///< Maximum forward speed (m/s)
#define LINEAR_GAIN 0.5f        ///< Proportional gain for linear speed
#define ANGULAR_GAIN 1.0f       ///< Proportional gain for angular speed
#define DISTANCE_THRESHOLD 0.5f ///< Threshold distance to consider waypoint reached (meters)
#define ANGLE_TOLERANCE 0.1f    ///< Yaw angle tolerance (radians)

// =========================
// Additional Configurations
// =========================

// Define stack sizes (in words) and priorities for FreeRTOS tasks
// Adjust these values based on actual task requirements and available memory

// StateEstimationTask
#define TASK_STACK_SIZE_STATE_ESTIMATION 4096
#define TASK_PRIORITY_STATE_ESTIMATION 3

// PathAdjustmentTask
#define TASK_STACK_SIZE_PATH_ADJUSTMENT 4096
#define TASK_PRIORITY_PATH_ADJUSTMENT 3

// CommunicationTask
#define TASK_STACK_SIZE_COMMUNICATION 2048
#define TASK_PRIORITY_COMMUNICATION 2

// RemoteCommunicationTask
#define TASK_STACK_SIZE_REMOTE_COMMUNICATION 2048
#define TASK_PRIORITY_REMOTE_COMMUNICATION 2

// HealthMonitoringTask
#define TASK_STACK_SIZE_HEALTH_MONITORING 2048
#define TASK_PRIORITY_HEALTH_MONITORING 1

// YawControlTask (to be removed)
#define TASK_STACK_SIZE_YAW_CONTROL 2048
#define TASK_PRIORITY_YAW_CONTROL 3

// PathFollowingTask
#define TASK_STACK_SIZE_PATH_FOLLOWING 4096
#define TASK_PRIORITY_PATH_FOLLOWING 3

#endif // MAIN_CONTROLLER_CONFIG_H
