// MotorController/tasks/MotorControlTask.cpp

#include "MotorControlTask.h"
#include "config.h"
#include "CommonLib/utils.h"

// External queue defined in main.cpp
extern QueueHandle_t mainVelocityCmdQueue;

// LEDC configuration parameters are defined in config.h
#define TURN_FACTOR 0.1f // Define as needed to influence angular velocity effect

/**
 * @brief The main function that runs the MotorControlTask.
 *        It continuously waits for VelocityCommandMsg and updates motor PWM accordingly.
 * @param parameter Task parameters (unused).
 */
void MotorControlTask::run(void *parameter)
{
    Serial.println("MotorControlTask: Starting...");

    // Initialize PWM channels
    initPWM();

    VelocityCommandMsg cmd;
    // Default: stop motors
    cmd.linear_vel_x = 0.0f;
    cmd.angular_vel_z = 0.0f;
    updateMotors(cmd);

    while (true)
    {
        // Wait indefinitely for a velocity command
        if (xQueueReceive(mainVelocityCmdQueue, &cmd, portMAX_DELAY) == pdPASS)
        {
            updateMotors(cmd);
        }

        // Yield to allow other tasks to run
        taskYIELD();
    }
}

/**
 * @brief Initializes the PWM channels for motor control using LEDC.
 */
void MotorControlTask::initPWM()
{
    // Configure LEDC timer for each channel
    ledcSetup(MOTOR_LEFT_CHANNEL, MOTOR_PWM_FREQUENCY_HZ, MOTOR_PWM_RESOLUTION_BITS);
    ledcSetup(MOTOR_RIGHT_CHANNEL, MOTOR_PWM_FREQUENCY_HZ, MOTOR_PWM_RESOLUTION_BITS);

    // Attach PWM channels to respective pins
    ledcAttachPin(PWM_PIN_MOTOR_LEFT, MOTOR_LEFT_CHANNEL);
    ledcAttachPin(PWM_PIN_MOTOR_RIGHT, MOTOR_RIGHT_CHANNEL);

    // Initialize motors to stop
    ledcWrite(MOTOR_LEFT_CHANNEL, MOTOR_PWM_STOP_DUTY);
    ledcWrite(MOTOR_RIGHT_CHANNEL, MOTOR_PWM_STOP_DUTY);

    Serial.println("MotorControlTask: PWM initialized and motors stopped.");
}

/**
 * @brief Sets a specific motor channel to a given duty cycle.
 * @param channel LEDC channel.
 * @param duty Duty cycle (0 - 65535 for 16-bit resolution).
 */
void MotorControlTask::setMotorPWM(int channel, uint32_t duty)
{
    if (duty > ((1 << MOTOR_PWM_RESOLUTION_BITS) - 1))
        duty = ((1 << MOTOR_PWM_RESOLUTION_BITS) - 1);
    ledcWrite(channel, duty);
}

/**
 * @brief Updates motor speeds based on the received velocity command.
 *        Implements differential thrust:
 *        - linear_vel_x controls forward/backward speed
 *        - angular_vel_z controls turning by making left and right thrust differ
 *
 * @param cmd VelocityCommandMsg containing linear and angular velocities.
 */
void MotorControlTask::updateMotors(const VelocityCommandMsg &cmd)
{
    float leftVel = cmd.linear_vel_x + (cmd.angular_vel_z * TURN_FACTOR);
    float rightVel = cmd.linear_vel_x - (cmd.angular_vel_z * TURN_FACTOR);

    float leftMs = velocityToPulseWidth(leftVel);
    float rightMs = velocityToPulseWidth(rightVel);

    uint32_t leftDuty = msToDuty(leftMs);
    uint32_t rightDuty = msToDuty(rightMs);

    setMotorPWM(MOTOR_LEFT_CHANNEL, leftDuty);
    setMotorPWM(MOTOR_RIGHT_CHANNEL, rightDuty);

    // Logging occasionally for debugging
    static int count = 0;
    if (count++ % 50 == 0)
    {
        Serial.print("MotorControlTask: cmd linear=");
        Serial.print(cmd.linear_vel_x);
        Serial.print(", angular=");
        Serial.print(cmd.angular_vel_z);
        Serial.print(" => Left PWM=");
        Serial.print(leftMs, 3);
        Serial.print("ms Right PWM=");
        Serial.print(rightMs, 3);
        Serial.println("ms");
    }
}
