// SensorController/tasks/OccupancyAdjustTask.cpp

#include "OccupancyAdjustTask.h"
#include <math.h>

extern QueueHandle_t preprocessedDataQueue;
extern SemaphoreHandle_t occupancyGridMutex;
extern struct OccupancyGrid occupancyGrid;

void OccupancyAdjustTask::run(void *parameter)
{
    Serial.println("OccupancyAdjustTask: Starting...");

    const TickType_t decayInterval = pdMS_TO_TICKS(5000); // every 5s decay
    TickType_t lastDecay = xTaskGetTickCount();

    SensorDataMsg data;
    while (true)
    {
        // Check if time for decay
        TickType_t now = xTaskGetTickCount();
        if ((now - lastDecay) > decayInterval)
        {
            decayOccupancyGrid();
            lastDecay = now;
        }

        if (xQueueReceive(preprocessedDataQueue, &data, 50 / portTICK_PERIOD_MS) == pdPASS)
        {
            updateOccupancyGrid(data);
        }

        taskYIELD();
    }
}

void OccupancyAdjustTask::updateOccupancyGrid(const SensorDataMsg &data)
{
    if (xSemaphoreTake(occupancyGridMutex, portMAX_DELAY) == pdTRUE)
    {
        // Update with forward sonar
        if (data.sonar_distance_forward > 0)
        {
            float dist = data.sonar_distance_forward;
            float angle = data.sonar_angle_forward; // should be 0
            float x = dist * cos(angle);
            float y = dist * sin(angle);

            int gx, gy;
            mapToGrid(x, y, gx, gy);
            if (gx >= 0 && gx < (int)occupancyGrid.width && gy >= 0 && gy < (int)occupancyGrid.height)
            {
                occupancyGrid.grid_data[gy * occupancyGrid.width + gx] = 1;
                // Mark surrounding cells
                for (int dx = -OBSTACLE_RADIUS; dx <= OBSTACLE_RADIUS; dx++)
                {
                    for (int dy = -OBSTACLE_RADIUS; dy <= OBSTACLE_RADIUS; dy++)
                    {
                        int adjx = gx + dx;
                        int adjy = gy + dy;
                        if (adjx >= 0 && adjx < (int)occupancyGrid.width && adjy >= 0 && adjy < (int)occupancyGrid.height)
                        {
                            occupancyGrid.grid_data[adjy * occupancyGrid.width + adjx] = 1;
                        }
                    }
                }
            }
        }

        // Update with rotating sonar
        if (data.sonar_distance_rotating > 0)
        {
            float dist = data.sonar_distance_rotating;
            float angle = data.sonar_angle_rotating;
            float x = dist * cos(angle);
            float y = dist * sin(angle);

            int gx, gy;
            mapToGrid(x, y, gx, gy);
            if (gx >= 0 && gx < (int)occupancyGrid.width && gy >= 0 && gy < (int)occupancyGrid.height)
            {
                occupancyGrid.grid_data[gy * occupancyGrid.width + gx] = 1;
                for (int dx = -OBSTACLE_RADIUS; dx <= OBSTACLE_RADIUS; dx++)
                {
                    for (int dy = -OBSTACLE_RADIUS; dy <= OBSTACLE_RADIUS; dy++)
                    {
                        int adjx = gx + dx;
                        int adjy = gy + dy;
                        if (adjx >= 0 && adjx < (int)occupancyGrid.width && adjy >= 0 && adjy < (int)occupancyGrid.height)
                        {
                            occupancyGrid.grid_data[adjy * occupancyGrid.width + adjx] = 1;
                        }
                    }
                }
            }
        }

        xSemaphoreGive(occupancyGridMutex);
    }
    else
    {
        Serial.println("OccupancyAdjustTask: Failed to get occupancyGridMutex.");
    }
}

void OccupancyAdjustTask::mapToGrid(float x, float y, int &gx, int &gy)
{
    // Convert (x,y) in meters to grid coordinates
    // Assuming (0,0) in world coords is at grid origin_x, origin_y
    gx = (int)(occupancyGrid.origin_x + x / occupancyGrid.resolution);
    gy = (int)(occupancyGrid.origin_y + y / occupancyGrid.resolution);
}

void OccupancyAdjustTask::decayOccupancyGrid()
{
    if (xSemaphoreTake(occupancyGridMutex, portMAX_DELAY) == pdTRUE)
    {
        // Simple decay: Reset all cells to 0
        for (int i = 0; i < (int)(occupancyGrid.width * occupancyGrid.height); i++)
        {
            occupancyGrid.grid_data[i] = 0;
        }
        Serial.println("OccupancyAdjustTask: Occupancy grid decayed.");
        xSemaphoreGive(occupancyGridMutex);
    }
    else
    {
        Serial.println("OccupancyAdjustTask: Failed to acquire occupancyGridMutex for decay.");
    }
}
