// MainController/tasks/RemoteCommunicationTask.cpp

#include "RemoteCommunicationTask.h"
#include "CommonLib/InterESPProtocol.h"
#include "config.h"
#include "CommonLib/messages.h"

// External queues defined in main.cpp
extern QueueHandle_t pathQueue; ///< Queue for PathMsg

/**
 * @brief The main function that runs the RemoteCommunicationTask.
 *        It listens for incoming PathMsg and enqueues them into pathQueue.
 * @param parameter Task parameters (unused).
 */
void RemoteCommunicationTask::run(void *parameter)
{
    Serial.println("RemoteCommunicationTask: Starting...");

    // Define the I2C address for the remote controller if applicable
    // For this implementation, assume that the remote sends PathMsg via I2C as master
    // and this task acts as the I2C slave to receive it.

    // However, in most cases, ESP32 cannot act as both I2C master and slave simultaneously
    // without careful management. Alternatively, the remote could send PathMsg via another
    // communication interface like UART or wireless (Wi-Fi/Bluetooth).
    //
    // For the sake of this implementation, we'll assume that RemoteCommunicationTask
    // receives PathMsg via a serial interface (e.g., UART), which is common for remote commands.

    // Initialize Serial for remote communication (e.g., Bluetooth Serial or USB Serial)
    // Here, we'll use Serial2 (pins 16 and 17 on ESP32) for demonstration
    Serial2.begin(115200, SERIAL_8N1, 16, 17); // RX = GPIO16, TX = GPIO17
    Serial.println("RemoteCommunicationTask: Serial2 initialized for remote communication.");

    while (true)
    {
        // Check if data is available on Serial2
        if (Serial2.available() >= sizeof(PathMsg))
        {
            PathMsg receivedPath;
            // Read the PathMsg
            Serial2.readBytes(reinterpret_cast<char*>(&receivedPath), sizeof(PathMsg));

            // Optional: Validate the received PathMsg (e.g., checksum, magic number)
            // For simplicity, we'll assume the message is well-formed

            Serial.println("RemoteCommunicationTask: Received PathMsg from remote.");

            // Process the received PathMsg
            processPathMessage(receivedPath);
        }

        // Yield to allow other tasks to run
        taskYIELD();
    }
}

/**
 * @brief Processes the received PathMsg and enqueues it into pathQueue.
 * @param pathMsg The received PathMsg.
 */
void RemoteCommunicationTask::processPathMessage(const PathMsg &pathMsg)
{
    // Enqueue the PathMsg into pathQueue
    if (xQueueSend(pathQueue, &pathMsg, pdMS_TO_TICKS(100)) != pdPASS)
    {
        Serial.println("RemoteCommunicationTask: Failed to enqueue PathMsg (Queue Full).");
    }
    else
    {
        Serial.println("RemoteCommunicationTask: Enqueued PathMsg into pathQueue.");
    }
}
