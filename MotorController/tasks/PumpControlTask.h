// MotorController/tasks/PumpControlTask.h

#ifndef PUMP_CONTROL_TASK_H
#define PUMP_CONTROL_TASK_H

#include "CommonLib/messages.h"
#include "config.h"
#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>

/**
 * @class PumpControlTask
 * @brief Controls intake and outflow pumps based on PumpCommandMsg.
 */
class PumpControlTask
{
public:
    /**
     * @brief The main function that runs the PumpControlTask.
     *        It continuously waits for PumpCommandMsg and updates pump states accordingly.
     * @param parameter Task parameters (unused).
     */
    static void run(void *parameter);

private:
    /**
     * @brief Initializes the pump control pins.
     */
    static void initPumps();

    /**
     * @brief Sets the intake pump state.
     * @param on True to turn ON, False to turn OFF.
     */
    static void setPumpIntake(bool on);

    /**
     * @brief Sets the outflow pump state.
     * @param on True to turn ON, False to turn OFF.
     */
    static void setPumpOutflow(bool on);
};

#endif // PUMP_CONTROL_TASK_H
