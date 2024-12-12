// MainController/tasks/PathAdjustmentTask.cpp

#include "PathAdjustmentTask.h"
#include "config.h"

extern QueueHandle_t pathQueue;
extern QueueHandle_t occupancyGridQueue;
extern QueueHandle_t adjustedPathQueue; // Newly referenced queue
extern SemaphoreHandle_t stateMutex;
extern State currentState;

OccupancyGridMsg PathAdjustmentTask::currentGrid = {0, 0, {0}};
SemaphoreHandle_t PathAdjustmentTask::gridMutex = NULL;

void PathAdjustmentTask::run(void *parameter)
{
    Serial.println("PathAdjustmentTask: Starting...");

    // Create a mutex to protect currentGrid
    gridMutex = xSemaphoreCreateMutex();
    if (gridMutex == NULL)
    {
        Serial.println("PathAdjustmentTask: Failed to create grid mutex.");
        vTaskDelete(NULL);
    }

    PathMsg receivedPath;
    OccupancyGridMsg receivedGrid;

    while (true)
    {
        // Check if we received an occupancy grid update
        if (xQueueReceive(occupancyGridQueue, &receivedGrid, 0) == pdPASS)
        {
            if (xSemaphoreTake(gridMutex, portMAX_DELAY) == pdTRUE)
            {
                currentGrid = receivedGrid;
                xSemaphoreGive(gridMutex);
                Serial.println("PathAdjustmentTask: Occupancy grid updated.");
            }
        }

        // Check if we received a new path
        if (xQueueReceive(pathQueue, &receivedPath, 50 / portTICK_PERIOD_MS) == pdPASS)
        {
            Serial.println("PathAdjustmentTask: Received new path. Adjusting...");
            adjustPath(receivedPath);

            // Log final adjusted path
            for (uint8_t i = 0; i < receivedPath.num_waypoints; i++)
            {
                Serial.print("Adjusted Waypoint ");
                Serial.print(i);
                Serial.print(": (");
                Serial.print(receivedPath.waypoints_x[i]);
                Serial.print(", ");
                Serial.print(receivedPath.waypoints_y[i]);
                Serial.print(", ");
                Serial.print(receivedPath.waypoints_z[i]);
                Serial.println(")");
            }

            // Enqueue the fully adjusted path into adjustedPathQueue
            if (xQueueSend(adjustedPathQueue, &receivedPath, 0) != pdPASS)
            {
                Serial.println("PathAdjustmentTask: adjustedPathQueue full, dropping adjusted path.");
            }
            else
            {
                Serial.println("PathAdjustmentTask: Adjusted path enqueued to adjustedPathQueue.");
            }
        }

        taskYIELD();
    }
}

/**
 * @brief Adjusts the path to avoid obstacles based on the occupancy grid.
 *
 * For each waypoint, we check if it lies in an occupied cell.
 * If so, we call findNearbyFreeCell() to attempt shifting the waypoint slightly.
 * If no free cell is found, we leave the waypoint as is, but log a warning.
 * A more sophisticated algorithm could do path replanning here.
 */
void PathAdjustmentTask::adjustPath(PathMsg &path)
{
    if (xSemaphoreTake(gridMutex, portMAX_DELAY) != pdTRUE)
    {
        Serial.println("PathAdjustmentTask: Failed to acquire grid mutex for adjustment.");
        return;
    }

    for (uint8_t i = 0; i < path.num_waypoints; i++)
    {
        int gx, gy;
        mapToGrid(path.waypoints_x[i], path.waypoints_y[i], gx, gy);

        if (gx < 0 || gx >= (int)currentGrid.grid_width || gy < 0 || gy >= (int)currentGrid.grid_height)
        {
            Serial.println("PathAdjustmentTask: Waypoint outside grid bounds, no adjustment possible.");
            continue;
        }

        if (isCellOccupied(gx, gy))
        {
            // Attempt to find a nearby free cell
            float newX = path.waypoints_x[i];
            float newY = path.waypoints_y[i];
            if (findNearbyFreeCell(newX, newY))
            {
                path.waypoints_x[i] = newX;
                path.waypoints_y[i] = newY;
                // Z coordinate remains the same since we have no obstacles vertically in this scenario
            }
            else
            {
                Serial.println("PathAdjustmentTask: Unable to find free cell near blocked waypoint.");
            }
            // After finishing adjustPath(...):
            if (xQueueSend(adjustedPathQueue, &path, 0) != pdPASS)
            {
                Serial.println("PathAdjustmentTask: adjustedPathQueue full, dropping adjusted path.");
            }
            else
            {
                Serial.println("PathAdjustmentTask: Adjusted path enqueued to adjustedPathQueue.");
            }
        }
    }

    xSemaphoreGive(gridMutex);
}

/**
 * @brief Checks if a given grid cell is occupied.
 */
bool PathAdjustmentTask::isCellOccupied(int gx, int gy)
{
    int index = gy * currentGrid.grid_width + gx;
    if (index < 0 || (uint32_t)index >= currentGrid.grid_width * currentGrid.grid_height)
        return true; // Out of bounds => treat as occupied

    return (currentGrid.grid_data[index] > 0);
}

/**
 * @brief Attempts to find a nearby free cell for a waypoint.
 *
 * Tries a small radius around the original waypoint to find a free cell.
 * This is a simplistic heuristic. For a more advanced approach, implement a proper path planner.
 */
bool PathAdjustmentTask::findNearbyFreeCell(float &x, float &y)
{
    // Small search radius in meters
    const float searchRadius = 0.5f;
    const float step = 0.1f;

    if (xSemaphoreTake(gridMutex, portMAX_DELAY) != pdTRUE)
        return false;

    float originalX = x;
    float originalY = y;

    for (float dx = -searchRadius; dx <= searchRadius; dx += step)
    {
        for (float dy = -searchRadius; dy <= searchRadius; dy += step)
        {
            float testX = originalX + dx;
            float testY = originalY + dy;
            int gx, gy;
            mapToGrid(testX, testY, gx, gy);
            if (gx >= 0 && gx < (int)currentGrid.grid_width &&
                gy >= 0 && gy < (int)currentGrid.grid_height &&
                !isCellOccupied(gx, gy))
            {
                x = testX;
                y = testY;
                xSemaphoreGive(gridMutex);
                return true;
            }
        }
    }

    xSemaphoreGive(gridMutex);
    return false;
}

/**
 * @brief Converts world coordinates (x,y) to grid coordinates (gx,gy).
 */
void PathAdjustmentTask::mapToGrid(float x, float y, int &gx, int &gy)
{
    // Assume the submarine starts at grid origin in the center (for simplicity)
    // If we need a known origin, define occupancyGrid origin fields.
    // Let's assume origin_x = origin_y = 50 (middle of the grid for demonstration)
    // No placeholders allowed: we must define how we get origin. Let's say:
    // The center of the grid is at (0,0) in world coordinates.
    // So gx = (x / resolution) + (width/2), gy = (y / resolution) + (height/2)

    float resolution = OCCUPANCY_GRID_RESOLUTION;
    gx = (int)((x / resolution) + (currentGrid.grid_width / 2.0f));
    gy = (int)((y / resolution) + (currentGrid.grid_height / 2.0f));
}
