// MainController/tasks/PathFollowingTask.cpp

#include "PathFollowingTask.h"
#include "CommonLib/InterESPProtocol.h"
#include "config.h"
#include "CommonLib/messages.h"
#include "CommonLib/utils.h"

#include <math.h>

// External queues and variables
extern QueueHandle_t adjustedPathQueue; ///< Queue for adjusted PathMsg
extern SemaphoreHandle_t stateMutex;
extern State currentState;

extern QueueHandle_t motorVelocityCmdQueue; ///< Queue for VelocityCommandMsg to MotorController
extern QueueHandle_t pumpCmdQueue;          ///< Queue for PumpCommandMsg to MotorController

// Initialize static PID variables
float PathFollowingTask::depthIntegral = 0.0f;
float PathFollowingTask::depthLastError = 0.0f;

/**
 * @brief The main function that runs the PathFollowingTask.
 *        It consumes adjusted PathMsg and computes VelocityCommandMsg and PumpCommandMsg.
 * @param parameter Task parameters (unused).
 */
void PathFollowingTask::run(void *parameter)
{
    Serial.println("PathFollowingTask: Starting...");

    PathMsg currentPath;
    bool havePath = false;
    uint8_t currentWaypointIndex = 0;

    while (true)
    {
        // If we don't have a path or we've completed the current path, try to get a new one
        if (!havePath || currentWaypointIndex >= currentPath.num_waypoints)
        {
            PathMsg newPath;
            // Now read from adjustedPathQueue, not pathQueue
            if (xQueueReceive(adjustedPathQueue, &newPath, pdMS_TO_TICKS(100)) == pdPASS)
            {
                currentPath = newPath;
                currentWaypointIndex = 0;
                havePath = true;
                Serial.println("PathFollowingTask: Received new ADJUSTED path.");
            }
            else
            {
                // No new adjusted path, idle by sending zero commands
                sendVelocityCommand(0.0f, 0.0f);
                sendPumpCommand(false, false);
                taskYIELD();
                continue;
            }
        }

        // Follow the current waypoint
        if (currentWaypointIndex < currentPath.num_waypoints)
        {
            // Lock the state to get current position and orientation
            if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE)
            {
                float currentX = currentState.x;
                float currentY = currentState.y;
                float currentZ = currentState.z;
                float currentYaw = currentState.yaw;
                float actualVx = currentState.vx;
                xSemaphoreGive(stateMutex);

                // Get the current waypoint
                float targetX = currentPath.waypoints_x[currentWaypointIndex];
                float targetY = currentPath.waypoints_y[currentWaypointIndex];
                float targetZ = currentPath.waypoints_z[currentWaypointIndex];

                // Compute distance and angle to the waypoint
                float dx = targetX - currentX;
                float dy = targetY - currentY;
                float dz = targetZ - currentZ;

                float distance = sqrt(dx * dx + dy * dy + dz * dz);
                float targetYaw = atan2(dy, dx);
                float yawError = targetYaw - currentYaw;

                // Normalize yaw error to [-PI, PI]
                while (yawError > PI)
                    yawError -= 2 * PI;
                while (yawError < -PI)
                    yawError += 2 * PI;

                // Compute velocity commands using proportional control
                float linearVel = LINEAR_GAIN * distance;
                if (linearVel > MAX_FORWARD_SPEED)
                    linearVel = MAX_FORWARD_SPEED;

                float angularVel = ANGULAR_GAIN * yawError;
                // Clamp angularVel to reasonable limits if necessary

                // Compute Depth Control using PID
                float depthError = targetZ - currentZ;
                // PID calculations
                depthIntegral += depthError * DEPTH_CONTROL_DT;
                // Clamp integral to prevent windup
                if (depthIntegral > MAX_DEPTH_INTEGRAL)
                    depthIntegral = MAX_DEPTH_INTEGRAL;
                if (depthIntegral < -MAX_DEPTH_INTEGRAL)
                    depthIntegral = -MAX_DEPTH_INTEGRAL;

                float depthDerivative = (depthError - depthLastError) / DEPTH_CONTROL_DT;
                float depthControlSignal = (Kp_DEPTH * depthError) + (Ki_DEPTH * depthIntegral) + (Kd_DEPTH * depthDerivative);
                depthLastError = depthError;

                // Decide pump states based on depthControlSignal
                bool intakePump = false;
                bool outflowPump = false;

                const float PUMP_THRESHOLD = 0.05f; // Threshold to activate pumps

                if (depthControlSignal > PUMP_THRESHOLD)
                {
                    // Need to increase depth: Turn on intake pump
                    intakePump = true;
                }
                else if (depthControlSignal < -PUMP_THRESHOLD)
                {
                    // Need to decrease depth: Turn on outflow pump
                    outflowPump = true;
                }
                // Else, within acceptable range: keep pumps off

                // Send velocity and pump commands
                sendVelocityCommand(linearVel, angularVel);
                sendPumpCommand(intakePump, outflowPump);

                // Log commanded vs. actual velocities
                Serial.print("PathFollowingTask: Commanded Vx=");
                Serial.print(linearVel);
                Serial.print(" m/s, Actual Vx=");
                Serial.print(actualVx);
                Serial.println(" m/s");

                // Check if waypoint is reached
                if (distance < DISTANCE_THRESHOLD && abs(yawError) < ANGLE_TOLERANCE)
                {
                    Serial.print("PathFollowingTask: Waypoint ");
                    Serial.print(currentWaypointIndex);
                    Serial.println(" reached.");
                    currentWaypointIndex++;
                }
            }
            else
            {
                // Failed to take stateMutex, log and continue
                Serial.println("PathFollowingTask: Failed to acquire stateMutex.");
            }
        }

        // Yield to allow other tasks to run
        taskYIELD();
    }

    /**
     * @brief Sends a VelocityCommandMsg to the CommunicationTask.
     * @param linearVel Linear velocity in m/s.
     * @param angularVel Angular velocity in rad/s.
     */
    void PathFollowingTask::sendVelocityCommand(float linearVel, float angularVel)
    {
        VelocityCommandMsg cmd;
        cmd.linear_vel_x = linearVel;
        cmd.angular_vel_z = angularVel;

        if (xQueueSend(motorVelocityCmdQueue, &cmd, pdMS_TO_TICKS(10)) != pdPASS)
        {
            Serial.println("PathFollowingTask: motorVelocityCmdQueue full, dropping VelocityCommandMsg.");
        }
        else
        {
            Serial.print("PathFollowingTask: Enqueued VelocityCommandMsg: linear=");
            Serial.print(cmd.linear_vel_x);
            Serial.print(", angular=");
            Serial.println(cmd.angular_vel_z);
        }
    }

    /**
     * @brief Sends a PumpCommandMsg to the CommunicationTask.
     * @param intakePump On/Off state for intake pump.
     * @param outflowPump On/Off state for outflow pump.
     */
    void PathFollowingTask::sendPumpCommand(bool intakePump, bool outflowPump)
    {
        PumpCommandMsg cmd;
        cmd.pumpIntakeControl = intakePump ? 1 : 0;
        cmd.pumpOutflowControl = outflowPump ? 1 : 0;

        if (xQueueSend(pumpCmdQueue, &cmd, pdMS_TO_TICKS(10)) != pdPASS)
        {
            Serial.println("PathFollowingTask: pumpCmdQueue full, dropping PumpCommandMsg.");
        }
        else
        {
            Serial.print("PathFollowingTask: Enqueued PumpCommandMsg: Intake=");
            Serial.print(cmd.pumpIntakeControl > 0 ? "ON" : "OFF");
            Serial.print(", Outflow=");
            Serial.println(cmd.pumpOutflowControl > 0 ? "ON" : "OFF");
        }
    }
