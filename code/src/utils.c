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

// Retrieves the current time difference in seconds
double get_time_difference(struct timeval ti, struct timeval tf) {
    return (tf.tv_sec - ti.tv_sec) + (tf.tv_usec - ti.tv_usec) / 1e6;
}
