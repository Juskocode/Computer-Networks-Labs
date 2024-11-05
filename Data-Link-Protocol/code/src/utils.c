#include "utils.h"

unsigned char* size_t_to_bytes(size_t value, unsigned char *size) {
    if (size == NULL) return NULL;

    unsigned char static_bytes[sizeof(size_t)];
    unsigned char length = 0;

    // Extract bytes from value
    do {
        static_bytes[length++] = value & 0xFF;
        value >>= 8;
    } while (value);

    // Allocate memory for the result
    unsigned char *bytes = malloc(length);
    if (bytes == NULL) return NULL;

    // Copy the bytes to the allocated memory
    memcpy(bytes, static_bytes, length);

    *size = length;
    return bytes;
}

size_t bytes_to_size_t(unsigned char n, const unsigned char *bytes) {
    if (bytes == NULL) return 0;

    size_t value = 0;
    size_t multiplier = 1;

    for (unsigned char i = 0; i < n; i++) {
        value += bytes[i] * multiplier;
        multiplier *= 256;
    }
    return value;
}

/* Function for byte stuffing */
const unsigned char * byteStuffing(const unsigned char *buffer, int bufSize, int *newSize) {
    if (buffer == NULL || newSize == NULL) return NULL;

    // Allocate buffer with exact size needed, using an overestimated max size.
    unsigned char *result = (unsigned char *) malloc(bufSize * 2); // Avoid +1 as bufSize * 2 already covers escape cases
    if (result == NULL) return NULL;

    size_t j = 0;

    // Process each byte in buffer
    for (size_t i = 0; i < bufSize; ++i) {
        if (buffer[i] == FRAME_FLAG) {
            result[j++] = FRAME_ESCAPE;
            result[j++] = FRAME_ESCAPED_FLAG;
        } else if (buffer[i] == FRAME_ESCAPE) {
            result[j++] = FRAME_ESCAPE;
            result[j++] = FRAME_ESCAPED_ESCAPE;
        } else {
            result[j++] = buffer[i];
        }
    }

    *newSize = (int)j;

    // Resize only if there's a difference between used size and allocated size
    if (j < bufSize * 2) {
        unsigned char *shrunkResult = realloc(result, j);
        if (shrunkResult != NULL) {
            result = shrunkResult; // Only update result if realloc was successful
        }
    }

    return result;
}

/* Function to perform byte destuffing */
int byteDestuffing(unsigned char *buffer, int bufferSize, int *processedSize, unsigned char *bcc2Received) {
    if (buffer == NULL || processedSize == NULL || bufferSize < 1) return -1;

    unsigned char *writePtr = buffer;  // Pointer to write in-place to buffer
    unsigned char *readPtr = buffer;   // Pointer to read from buffer
    unsigned char *endPtr = buffer + bufferSize; // Pointer to end of buffer

    while (readPtr < endPtr) {
        if (*readPtr != FRAME_ESCAPE) {
            *writePtr++ = *readPtr++;
        } else if (readPtr + 1 < endPtr) {  // Check we don't go out of bounds
            ++readPtr;
            if (*readPtr == FRAME_ESCAPED_FLAG) {
                *writePtr++ = FRAME_FLAG;
            } else if (*readPtr == FRAME_ESCAPED_ESCAPE) {
                *writePtr++ = FRAME_ESCAPE;
            } else {
                *writePtr++ = FRAME_ESCAPE; // If invalid escape, copy escape as is
                *writePtr++ = *readPtr; // Copy the following byte
            }
            ++readPtr;
        } else { // Handle edge case if FRAME_ESCAPE is last byte
            *writePtr++ = FRAME_ESCAPE;
            break;
        }
    }

    // Update BCC2 and processed size, assuming the last byte is the BCC2 value
    *bcc2Received = *(writePtr - 1);
    *processedSize = (int)(writePtr - buffer - 1);

    return 0;
}

// Retrieves the current time difference in seconds
double get_time_difference(struct timeval ti, struct timeval tf) {
    return (tf.tv_sec - ti.tv_sec) + (tf.tv_usec - ti.tv_usec) / 1e6;
}
