/**
 * @file size_conversion.h
 * @brief Utility functions for converting between size_t and byte arrays.
 *
 * This header provides two functions:
 * - `size_t_to_bytes`: Converts a `size_t` value to a dynamically allocated byte array.
 * - `bytes_to_size_t`: Converts a byte array back to a `size_t` value.
 */

#ifndef SIZE_CONVERSION_H
#define SIZE_CONVERSION_H

#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

/*Statistics struct data representation*/
typedef struct
{
    size_t bytes_read;          // Number of bytes read before any destuffing
    unsigned int nFrames;       // Number of good frames sent/received
    unsigned int errorFrames;
    unsigned int frames_size;   // Size of good frames sent
    double time_send_control;   // Time spent on sending control frames
    double time_send_data;      // Time spent on sending data frames
    struct timeval start;       // When program starts
} Statistics;

/**
 * @brief Converts a `size_t` integer to a byte array representation.
 *
 * @param value The `size_t` value to be converted to a byte array.
 * @param size  Pointer to an `unsigned char` where the function will store
 *              the number of bytes in the returned array.
 *              This parameter must not be NULL.
 *
 * @return A pointer to a dynamically allocated array of `unsigned char` bytes
 *         representing the `value`. The caller is responsible for freeing this memory.
 *         Returns NULL if memory allocation fails or if `size` is NULL.
 *
 * @note The byte array represents `value` in little-endian order.
 */
unsigned char* size_t_to_bytes(size_t value, unsigned char *size);

/**
 * @brief Converts a byte array back to a `size_t` integer.
 *
 * @param n      The number of bytes in the `bytes` array.
 * @param bytes  Pointer to an array of `unsigned char` representing the byte array.
 *               This parameter must not be NULL.
 *
 * @return The `size_t` integer reconstructed from the byte array.
 *         If `bytes` is NULL, returns 0.
 *
 * @note Assumes the byte array is in little-endian order.
 */
size_t bytes_to_size_t(unsigned char n, const unsigned char *bytes);


/**
 * @brief Gets the diference between two dates
 *
 * @param ti  inicial time
 * @param tf  end state time
 *
 * @return double value represent the diff of the times
 *
 */
double get_time_difference(struct timeval ti, struct timeval tf);

#endif // SIZE_CONVERSION_H
