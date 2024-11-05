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
#include "macros.h"

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
 * @brief Perform byte stuffing on the input buffer.
 * 
 * This function takes an input buffer and applies byte stuffing by replacing specific 
 * control characters (like FRAME_FLAG and FRAME_ESCAPE) with escape sequences. This 
 * allows the transmitted data to include special bytes without being misinterpreted by 
 * the receiving end as control flags.
 *
 * @param buffer The input buffer containing the original data to be stuffed.
 * @param bufSize The size of the input buffer.
 * @param newSize Pointer to an integer where the function will store the size of 
 *                the stuffed data in bytes.
 * 
 * @return A pointer to the newly allocated buffer containing the stuffed data, or 
 *         NULL if there is an error. The caller is responsible for freeing the 
 *         returned buffer.
 */
const unsigned char * byteStuffing(const unsigned char *buffer, int bufSize, int *newSize);


/**
 * @brief Perform byte destuffing on a stuffed input buffer.
 * 
 * This function removes byte stuffing from a buffer, restoring the original data by 
 * interpreting escape sequences and control flags. It writes the destuffed data back 
 * in-place and calculates the BCC (Block Check Character) of the received data.
 *
 * @param buffer The input buffer containing stuffed data. The destuffed data will be 
 *               written back to this buffer.
 * @param bufferSize The size of the input buffer in bytes.
 * @param processedSize Pointer to an integer where the function will store the size 
 *                      of the destuffed data in bytes.
 * @param bcc2Received Pointer to an unsigned char where the function will store the 
 *                     BCC2 (error-checking code) calculated from the destuffed data.
 * 
 * @return Returns 0 on success, or -1 on error.
 */
int byteDestuffing(unsigned char *buffer, int bufferSize, int *processedSize, unsigned char *bcc2Received);


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
