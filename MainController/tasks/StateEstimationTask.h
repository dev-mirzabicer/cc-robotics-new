// MainController/tasks/StateEstimationTask.h

#ifndef STATE_ESTIMATION_TASK_H
#define STATE_ESTIMATION_TASK_H

#include "CommonLib/messages.h"
#include "config.h"
#include "CommonLib/utils.h"

#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>

/**
 * @class StateEstimationTask
 * @brief Task responsible for estimating the submarine's current state using sensor data.
 */
class StateEstimationTask
{
public:
    /**
     * @brief Entry point for the StateEstimationTask.
     * @param parameter Task parameter (unused).
     */
    static void run(void *parameter);

private:
    /**
     * @brief Processes incoming sensor data to update the submarine's state.
     * @param sensorData Sensor data received from SensorController.
     */
    static void processSensorData(const SensorDataMsg &sensorData);

    /**
     * @brief Updates the global state with the newly estimated values.
     * @param newState The newly estimated state.
     */
    static void updateGlobalState(const State &newState);
};

#endif // STATE_ESTIMATION_TASK_H
