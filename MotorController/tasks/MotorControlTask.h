// MotorController/tasks/MotorControlTask.h

#ifndef MOTOR_CONTROL_TASK_H
#define MOTOR_CONTROL_TASK_H

#include "CommonLib/messages.h"
#include "config.h"
#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>

/**
 * @class MotorControlTask
 * @brief Controls motors by converting velocity commands into ESC signals.
 *
 * Without feedback, this is open-loop control: commanded velocity -> PWM duty cycle.
 */
class MotorControlTask
{
public:
    /**
     * @brief The main function that runs the MotorControlTask.
     * @param parameter Task parameters (unused).
     */
    static void run(void *parameter);

private:
    /**
     * @brief Initializes the PWM channels for motor control.
     */
    static void initPWM();

    /**
     * @brief Sets a specific motor channel to a given duty cycle.
     * @param channel LEDC channel.
     * @param duty Duty cycle (0 - 65535 for 16-bit resolution).
     */
    static void setMotorPWM(int channel, uint32_t duty);

    /**
     * @brief Updates motor speeds based on the received velocity command.
     * @param cmd The received VelocityCommandMsg.
     */
    static void updateMotors(const VelocityCommandMsg &cmd);
};

#endif // MOTOR_CONTROL_TASK_H
