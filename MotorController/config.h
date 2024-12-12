// MotorController/config.h

#ifndef MOTOR_CONTROLLER_CONFIG_H
#define MOTOR_CONTROLLER_CONFIG_H

#include "CommonLib/config_common.h"

// =========================
// Task Stack Sizes & Priorities
// =========================

#define TASK_STACK_SIZE_MOTOR_CONTROL 4096
#define TASK_PRIORITY_MOTOR_CONTROL 3

#define TASK_STACK_SIZE_PUMP_CONTROL 2048
#define TASK_PRIORITY_PUMP_CONTROL 2

#define TASK_STACK_SIZE_COMMUNICATION 2048
#define TASK_PRIORITY_COMMUNICATION 2

// =========================
// Motor & ESC Configuration
// =========================

// PWM pins for left and right thrusters
#define PWM_PIN_MOTOR_LEFT 18
#define PWM_PIN_MOTOR_RIGHT 19

// Standard RC servo signals: 50Hz frequency (20ms period), pulse width 1ms-2ms
#define MOTOR_PWM_FREQUENCY_HZ 50
#define MOTOR_PWM_RESOLUTION_BITS 16

// LEDC Channels
#define MOTOR_LEFT_CHANNEL 0
#define MOTOR_RIGHT_CHANNEL 1

// PWM parameters
#define MOTOR_PERIOD_MS 20.0f
#define MOTOR_MIN_PW_MS 1.0f
#define MOTOR_MAX_PW_MS 2.0f
#define MOTOR_STOP_PW_MS 1.5f

// Convert ms to duty steps
static inline uint32_t msToDuty(float ms)
{
    return (uint32_t)((ms / MOTOR_PERIOD_MS) * ((1 << MOTOR_PWM_RESOLUTION_BITS) - 1));
}

#define MOTOR_PWM_MIN_DUTY msToDuty(MOTOR_MIN_PW_MS)
#define MOTOR_PWM_STOP_DUTY msToDuty(MOTOR_STOP_PW_MS)
#define MOTOR_PWM_MAX_DUTY msToDuty(MOTOR_MAX_PW_MS)

// =========================
// Velocity to PWM Mapping
// =========================

// Define the maximum forward and reverse velocities
#define MOTOR_MAX_FORWARD_VELOCITY 1.0f  // m/s at 2ms
#define MOTOR_MAX_REVERSE_VELOCITY -1.0f // m/s at 1ms

/**
 * @brief Converts a linear velocity to a PWM pulse width.
 * @param v Linear velocity in m/s.
 * @return Pulse width in milliseconds.
 */
static inline float velocityToPulseWidth(float v)
{
    if (v > MOTOR_MAX_FORWARD_VELOCITY)
        v = MOTOR_MAX_FORWARD_VELOCITY;
    if (v < MOTOR_MAX_REVERSE_VELOCITY)
        v = MOTOR_MAX_REVERSE_VELOCITY;

    // Map: -1.0 m/s -> 1ms, 0 m/s -> 1.5ms, 1.0 m/s -> 2ms
    // Slope = 0.5ms per (m/s)
    float ms = MOTOR_STOP_PW_MS + (v * 0.5f);
    return ms;
}

// =========================
// Pump Configuration
// =========================

// Define pins for intake and outflow pumps.
#define PUMP_INTAKE_PIN 23
#define PUMP_OUTFLOW_PIN 22

#endif // MOTOR_CONTROLLER_CONFIG_H
