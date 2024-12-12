// MotorController/tasks/PumpControlTask.cpp

#include "PumpControlTask.h"
#include "config.h"
#include "CommonLib/utils.h"

// External queue defined in main.cpp
extern QueueHandle_t mainPumpCmdQueue;

/**
 * @brief The main function that runs the PumpControlTask.
 *        It continuously waits for PumpCommandMsg and updates pump states accordingly.
 * @param parameter Task parameters (unused).
 */
void PumpControlTask::run(void *parameter)
{
    Serial.println("PumpControlTask: Starting...");

    // Initialize pump control pins
    initPumps();

    PumpCommandMsg cmd;
    // Default: pumps OFF
    setPumpIntake(false);
    setPumpOutflow(false);

    while (true)
    {
        // Wait indefinitely for a pump command
        if (xQueueReceive(mainPumpCmdQueue, &cmd, portMAX_DELAY) == pdPASS)
        {
            setPumpIntake(cmd.pumpIntakeControl > 0);
            setPumpOutflow(cmd.pumpOutflowControl > 0);
            Serial.print("PumpControlTask: Intake=");
            Serial.print(cmd.pumpIntakeControl > 0 ? "ON" : "OFF");
            Serial.print(" Outflow=");
            Serial.println(cmd.pumpOutflowControl > 0 ? "ON" : "OFF");
        }

        // Yield to allow other tasks to run
        taskYIELD();
    }
}

/**
 * @brief Initializes the pump control pins.
 */
void PumpControlTask::initPumps()
{
    pinMode(PUMP_INTAKE_PIN, OUTPUT);
    pinMode(PUMP_OUTFLOW_PIN, OUTPUT);
    digitalWrite(PUMP_INTAKE_PIN, LOW);
    digitalWrite(PUMP_OUTFLOW_PIN, LOW);
    Serial.println("PumpControlTask: Pumps initialized and OFF.");
}

/**
 * @brief Sets the intake pump state.
 * @param on True to turn ON, False to turn OFF.
 */
void PumpControlTask::setPumpIntake(bool on)
{
    digitalWrite(PUMP_INTAKE_PIN, on ? HIGH : LOW);
}

/**
 * @brief Sets the outflow pump state.
 * @param on True to turn ON, False to turn OFF.
 */
void PumpControlTask::setPumpOutflow(bool on)
{
    digitalWrite(PUMP_OUTFLOW_PIN, on ? HIGH : LOW);
}
