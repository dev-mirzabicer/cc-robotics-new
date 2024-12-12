// CommonLib/utils.h

#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>

namespace Utils
{

    /**
     * @brief Converts degrees to radians.
     * @param degrees Angle in degrees.
     * @return Angle in radians.
     */
    inline float degreesToRadians(float degrees)
    {
        return degrees * (PI / 180.0f);
    }

    /**
     * @brief Converts radians to degrees.
     * @param radians Angle in radians.
     * @return Angle in degrees.
     */
    inline float radiansToDegrees(float radians)
    {
        return radians * (180.0f / PI);
    }

    /**
     * @brief Calculates a simple checksum by summing all bytes.
     * @param data Pointer to the data buffer.
     * @param length Number of bytes to include in the checksum.
     * @return Calculated checksum as an 8-bit unsigned integer.
     */
    inline uint8_t calculateChecksum(const uint8_t *data, size_t length)
    {
        uint8_t checksum = 0;
        for (size_t i = 0; i < length; ++i)
        {
            checksum += data[i];
        }
        return checksum;
    }

    // Add other utility function declarations below as needed

} // namespace Utils

#endif // UTILS_H
