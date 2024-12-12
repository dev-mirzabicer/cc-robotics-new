// CommonLib/messages.h

#ifndef MESSAGES_H
#define MESSAGES_H

#include <stdint.h>

/**
 * @enum MessageType
 * @brief Enum defining the type of message for robust decoding.
 */
enum class MessageType : uint8_t
{
    VELOCITY_COMMAND = 1,
    PATH = 2,
    OCCUPANCY_GRID = 3,
    SENSOR_DATA = 4,
    HEALTH_STATUS = 5,
    PUMP_COMMAND = 6,
    REQUEST_SENSOR_DATA = 10,
    REQUEST_OCCUPANCY_GRID = 11,
    REQUEST_HEALTH_STATUS = 12
    // Add more if needed
};

// =========================
// Velocity Command Message
// =========================

/**
 * @struct VelocityCommandMsg
 * @brief Message structure for sending velocity commands.
 */
typedef struct
{
    float linear_vel_x;  ///< Linear velocity in X-direction (m/s)
    float angular_vel_z; ///< Angular velocity around Z-axis (rad/s)
} VelocityCommandMsg;

// =========================
// Path Message
// =========================

/**
 * @struct PathMsg
 * @brief Message structure for sending a global path.
 */
typedef struct
{
    uint8_t num_waypoints;  ///< Number of waypoints in the path
    float waypoints_x[100]; ///< X-coordinates of waypoints (meters)
    float waypoints_y[100]; ///< Y-coordinates of waypoints (meters)
    float waypoints_z[100]; ///< Z-coordinates of waypoints (meters)
} PathMsg;

// =========================
// Occupancy Grid Message
// =========================

/**
 * @struct OccupancyGridMsg
 * @brief Message structure for sending occupancy grid data.
 */
typedef struct
{
    uint16_t grid_width;      ///< Width of the occupancy grid (number of cells)
    uint16_t grid_height;     ///< Height of the occupancy grid (number of cells)
    uint8_t grid_data[10000]; ///< Flattened occupancy grid data (0: free, 1: occupied)
} OccupancyGridMsg;

// =========================
// Sensor Data Message
// =========================

/**
 * @struct SensorDataMsg
 * @brief Message structure for sending sensor data.
 */
typedef struct
{
    // IMU Data
    float acceleration_x; ///< Acceleration in X-axis (m/s^2)
    float acceleration_y; ///< Acceleration in Y-axis (m/s^2)
    float acceleration_z; ///< Acceleration in Z-axis (m/s^2)
    float gyro_x;         ///< Gyroscope data around X-axis (rad/s)
    float gyro_y;         ///< Gyroscope data around Y-axis (rad/s)
    float gyro_z;         ///< Gyroscope data around Z-axis (rad/s)

    // Magnetometer Data
    float magnetometer_x; ///< Magnetic field in X-axis (µT)
    float magnetometer_y; ///< Magnetic field in Y-axis (µT)
    float magnetometer_z; ///< Magnetic field in Z-axis (µT)

    // Pressure Sensor Data
    float pressure; ///< Pressure measurement (kPa)

    // Sonar Data
    float sonar_distance_forward;  ///< Distance from forward sonar (meters)
    float sonar_angle_forward;     ///< Angle of forward sonar (radians) [Always 0]
    float sonar_distance_rotating; ///< Distance from rotating sonar (meters)
    float sonar_angle_rotating;    ///< Angle of rotating sonar (radians)
} SensorDataMsg;

// =========================
// Health Status Message
// =========================

/**
 * @struct HealthStatusMsg
 * @brief Message structure for sending health status of sensors and modules.
 */
typedef struct
{
    bool imu_health;             ///< Health status of IMU
    bool magnetometer_health;    ///< Health status of Magnetometer
    bool pressure_sensor_health; ///< Health status of Pressure Sensor
    bool sonar_forward_health;   ///< Health status of Forward Sonar
    bool sonar_rotating_health;  ///< Health status of Rotating Sonar
    // Add additional health statuses as needed
} HealthStatusMsg;

// =========================
// Pump Command Message
// =========================

/**
 * @struct PumpCommandMsg
 * @brief Message structure for controlling pumps.
 */
typedef struct
{
    uint8_t pumpIntakeControl;  ///< Control flag for intake pump (0: OFF, 1: ON)
    uint8_t pumpOutflowControl; ///< Control flag for outflow pump (0: OFF, 1: ON)
} PumpCommandMsg;

// =========================
// Additional Messages
// =========================

// Define additional message structures below as needed

#endif // MESSAGES_H
