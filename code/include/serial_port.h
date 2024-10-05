// Serial port header.
// NOTE: This file must not be changed.

#ifndef _SERIAL_PORT_H_
#define _SERIAL_PORT_H_

// Open and configure the serial port.
// Returns -1 on error.
int openSerialPort(const char *serialPort, int baudRate);

// Restore original port settings and close the serial port.
// Returns -1 on error.
int closeSerialPort();

// Wait for a byte received from the serial port and read it (must
// check whether a byte was actually received from the return value).
// Returns -1 on error, 0 if no byte was received, 1 if a byte was received.
int readByte(char *byte);

// Write up to numBytes to the serial port (must check how many were actually
// written in the return value).
// Returns -1 on error, otherwise the number of bytes written.
int writeBytes(const char *bytes, int numBytes);

#endif // _SERIAL_PORT_H_
