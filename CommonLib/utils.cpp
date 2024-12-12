// CommonLib/utils.cpp

#include "utils.h"

// Currently, all utility functions are defined as inline in utils.h.
// Add non-inline utility function definitions below as needed.

namespace Utils
{

    // Example of a non-inline utility function (if required in future)

    /**
     * @brief Calculates the Euclidean distance between two points in 2D space.
     * @param x1 X-coordinate of the first point.
     * @param y1 Y-coordinate of the first point.
     * @param x2 X-coordinate of the second point.
     * @param y2 Y-coordinate of the second point.
     * @return Euclidean distance.
     */
    float calculateDistance(float x1, float y1, float x2, float y2)
    {
        float dx = x2 - x1;
        float dy = y2 - y1;
        return sqrt(dx * dx + dy * dy);
    }

} // namespace Utils
