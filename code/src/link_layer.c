// Link layer protocol implementation

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include <signal.h>
#include <sys/time.h>
#include <time.h>

#include "link_layer.h"
#include "serial_port.h"  // Include the serial port helper functions

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

#define FALSE 0
#define TRUE 1

#define FRAME_SIZE 5

/*[F][A][C][BCC1][BCC2]*/

/* Frame Boundary Flags */
#define FRAME_FLAG             0x7E
#define FRAME_ESCAPE           0x7D
#define FRAME_ESCAPED_FLAG     0x5E
#define FRAME_ESCAPED_ESCAPE   0x5D

/* Address Field Identifiers */
#define ADDR_SENDER            0x03
#define ADDR_RECEIVER          0x01

/* Control Field Commands */
#define CTRL_SET               0x03
#define CTRL_UA                0x07
#define CTRL_DISCONNECT        0x0B

/* Information Field Identifiers */
#define INFO_CTRL_0            0x00
#define INFO_CTRL_1            0x40


/*ACK's and NACK's*/
#define RR0         0xAA
#define RR1         0xAB
#define REJ0        0x54
#define REJ1        0x55

// NOTE: only for report statistics
#define FAKE_BCC1_ERR   0.0             // %
#define FAKE_BCC2_ERR   0.0             // %
#define TPROP           0               // ms
#define FILE_SIZE       10968

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

/* Enum to represent communication states */
enum CommState {
    STATE_START, 
    STATE_FLAG_RECEIVED,
    STATE_ADDRESS_RECEIVED,
    STATE_CONTROL_RECEIVED,
    STATE_BCC_OK,
    STATE_DATA,
    STATE_STOP
};

LinkLayer connectionParameters;
Statistics statistics = {0, 0, 0, 0, 0.0, 0.0};
int alarmEnabled = FALSE;
int alarmCount = 0;
unsigned char C_Ns = 0; // Ns
unsigned char C_Nr = 0; // Nr (o valor que ele espera de receber)

// Retrieves the current time difference in seconds
double get_time_difference(struct timeval ti, struct timeval tf) {
    return (tf.tv_sec - ti.tv_sec) + (tf.tv_usec - ti.tv_usec) / 1e6;
}

// Opens the serial port and configures it using the openSerialPort function
int connectFD(LinkLayer connectionParametersApp) {
    if (connectionParametersApp.serialPort[0] == '\0') return -1;

    // Use the openSerialPort function from serial_port.c
    fd = openSerialPort(connectionParametersApp.serialPort, connectionParametersApp.baudRate);
    if (fd < 0) {
        fprintf(stderr, "Failed to open serial port %s\n", connectionParametersApp.serialPort);
        return -1;
    }

    printf("Serial port connection established\n");
    return 0;
}

// Closes the serial port using the closeSerialPort function
int disconnectFD() {
    // Use the closeSerialPort function from serial_port.c
    if (closeSerialPort() == -1) {
        perror("Failed to restore serial port settings");
        return -1;
    }
    printf("Serial port disconnected\n");
    return 0;
}

// Reads a byte from the serial port using the readByte function
int readByteFromPort(char *byte) {
    int result = readByte(byte);
    if (result == -1) {
        perror("Error reading byte from serial port");
    }
    return result;
}

// Writes bytes to the serial port using the writeBytes function
int writeBytesToPort(const char *bytes, int numBytes) {
    int result = writeBytes(bytes, numBytes);
    if (result == -1) {
        perror("Error writing bytes to serial port");
    }
    return result;
}

void alarmHandler(int signal)
{
    alarmCount++;
    alarmEnabled = TRUE;
    printf("Alarm count: %d\n", alarmCount);
}

void alarmDisable()
{
    alarm(0);
    alarmEnabled = FALSE;
    alarmCount = 0;
}

/* Function to send a command packet */
int sendCommandPacket(unsigned char address, unsigned char control) {
    unsigned char buffer[FRAME_SIZE] = {FRAME_FLAG, address, control, 0, FRAME_FLAG};
    buffer[3] = buffer[1] ^ buffer[2];
    
    if(write(fd, buffer, FRAME_SIZE) < 0) {
        perror("Error writing send command");
        return -1;
    }
    return 0;
}

/* Function to receive a packet */
int receivePacket(unsigned char expectedAddress, unsigned char expectedControl) {
    enum CommState currentState = STATE_START;
    
    while (currentState != STATE_STOP) {
        unsigned char byte = 0;
        int bytesRead;
        if((bytesRead = read(fd, &byte, sizeof(byte))) < 0) {
            perror("Error reading DISC command");
            return -1;
        }
        if(bytesRead > 0) {
            switch (currentState) {
            case STATE_START:
                if(byte == FRAME_FLAG) currentState = STATE_FLAG_RECEIVED;
                break;
            case STATE_FLAG_RECEIVED:
                if(byte == FRAME_FLAG) continue;
                if(byte == expectedAddress) currentState = STATE_ADDRESS_RECEIVED;
                else currentState = STATE_START;
                break;
            case STATE_ADDRESS_RECEIVED:
                if(byte == expectedControl) currentState = STATE_CONTROL_RECEIVED;
                else if(byte == FRAME_FLAG) currentState = STATE_FLAG_RECEIVED;
                else currentState = STATE_START;
                break;
            case STATE_CONTROL_RECEIVED:
                if(byte == (expectedControl ^ expectedAddress)) currentState = STATE_BCC_OK;
                else if(byte == FRAME_FLAG) currentState = STATE_FLAG_RECEIVED;
                else currentState = STATE_START;
                break;
            case STATE_BCC_OK:
                if(byte == FRAME_FLAG) currentState = STATE_STOP;
                else currentState = STATE_START;
                break;
            default:
                currentState = STATE_START;
            }
        }
    }
    
    return 0; 
}

/* Function to receive a packet with retransmission */
int receivePacketWithRetransmission(unsigned char expectedAddress, unsigned char expectedControl, unsigned char sendAddress, unsigned char sendControl) {
    enum CommState currentState = STATE_START;
    (void)signal(SIGALRM, alarmHandler);
    if(sendCommandPacket(sendAddress, sendControl)) return -1;
    alarm(connectionParameters.timeout);
    printf("Alarm count: %d\n", alarmCount);
    printf("Connection retransmissions: %d\n", connectionParameters.nRetransmissions);

    while (currentState != STATE_STOP && alarmCount <= connectionParameters.nRetransmissions) {
        unsigned char byte = 0;
        int bytesRead;
        if((bytesRead = read(fd, &byte, sizeof(byte))) < 0) {
            perror("Error reading UA command");
            return -1;
        }
        if(bytesRead > 0) {
            switch (currentState) {
            case STATE_START:
                if(byte == FRAME_FLAG) currentState = STATE_FLAG_RECEIVED;
                break;
            case STATE_FLAG_RECEIVED:
                if(byte == FRAME_FLAG) continue;
                if(byte == expectedAddress) currentState = STATE_ADDRESS_RECEIVED;
                else currentState = STATE_START;
                break;
            case STATE_ADDRESS_RECEIVED:
                if(byte == expectedControl) currentState = STATE_CONTROL_RECEIVED;
                else if(byte == FRAME_FLAG) currentState = STATE_FLAG_RECEIVED;
                else currentState = STATE_START;
                break;
            case STATE_CONTROL_RECEIVED:
                if(byte == (expectedControl ^ expectedAddress)) currentState = STATE_BCC_OK;
                else if(byte == FRAME_FLAG) currentState = STATE_FLAG_RECEIVED;
                else currentState = STATE_START;
                break;
            case STATE_BCC_OK:
                if(byte == FRAME_FLAG) currentState = STATE_STOP;
                else currentState = STATE_START;
                break;
            default:
                currentState = STATE_START;
            }
        }

        if(currentState == STATE_STOP) {
            alarmDisable();
            return 0;
        }
        
        if(alarmEnabled) {
            alarmEnabled = FALSE;
            if (alarmCount <= connectionParameters.nRetransmissions) {
                if(sendCommandPacket(sendAddress, sendControl)) return -1;
                alarm(connectionParameters.timeout);
            }
            currentState = STATE_START;
        }
    }

    alarmDisable();
    return -1;
}

/* Function to open a link layer connection */
int llopen(LinkLayer connectionParams) {
    gettimeofday(&statistics.start, NULL);
    memcpy(&connectionParameters, &connectionParams, sizeof(connectionParams));

    if (connectFD(connectionParameters) == -1)
        return -1;

    if(connectionParameters.role == LlTx) {
        struct timeval start, end;
        gettimeofday(&start, NULL);

        if(receivePacketWithRetransmission(ADDR_SENDER, CTRL_UA, ADDR_SENDER, CTRL_SET)) return -1;
        printf("Received transmission successfully!\n");

        statistics.nFrames++;
        gettimeofday(&end, NULL);
        statistics.time_send_control += get_time_difference(start, end);

        printf("Connection established\n");
    }
    if(connectionParameters.role == LlRx) {
        srand(time(NULL));
        if(receivePacket(ADDR_SENDER, CTRL_SET)) return -1;
        statistics.nFrames++;
        statistics.bytes_read += FRAME_SIZE;
        if(sendCommandPacket(ADDR_SENDER, CTRL_UA)) return -1;
        printf("Connection established\n");
    }
    return 0;
}

/* Function for byte stuffing */
const unsigned char * performByteStuffing(const unsigned char *buffer, int bufSize, int *newSize) {
    if(buffer == NULL || newSize == NULL) return NULL;

    unsigned char *result = (unsigned char *) malloc(bufSize * 2 + 1);
    if(result == NULL) return NULL;
    size_t j = 0;

    for(size_t i = 0; i < bufSize; i++, j++) {
        if(buffer[i] == FRAME_FLAG) {
            result[j++] = FRAME_ESCAPE;
            result[j] = FRAME_ESCAPED_FLAG;
        }
        else if (buffer[i] == FRAME_ESCAPE) {
            result[j++] = FRAME_ESCAPE;
            result[j] = FRAME_ESCAPED_ESCAPE;
        }
        else
            result[j] = buffer[i];
    }

    *newSize = (int) j;
    result = realloc(result, j);
    return result;
}

/* Function to write to the link layer */
int llwrite(const unsigned char *buffer, int bufSize) {
    if(buffer == NULL) return -1;

    int stuffedSize;
    const unsigned char *stuffedBuffer = performByteStuffing(buffer, bufSize, &stuffedSize);
    if(stuffedBuffer == NULL) return -1;
    printf("Bytes sent: %d\n", stuffedSize);
    
    unsigned char *frame = (unsigned char *) malloc(stuffedSize + 6);
    if(frame == NULL) {
        free((unsigned char *) stuffedBuffer);
        return -1;
    }

    frame[0] = FRAME_FLAG;
    frame[1] = ADDR_SENDER;
    frame[2] = (C_Ns) ? INFO_CTRL_1 : INFO_CTRL_0;
    frame[3] = frame[1] ^ frame[2];
    memcpy(frame + 4, stuffedBuffer, stuffedSize);

    unsigned char bcc2 = 0x00;
    for(size_t i = 0; i < bufSize; i++) bcc2 ^= buffer[i];
    frame[stuffedSize + 4] = bcc2;

    if(bcc2 == FRAME_FLAG) {
        frame[stuffedSize + 4] = FRAME_ESCAPE;
        stuffedSize++;
        frame[stuffedSize + 4] = FRAME_ESCAPED_FLAG;
        frame = realloc(frame, stuffedSize + 6);
        frame[stuffedSize + 5] = FRAME_FLAG;
    } else {
        frame[stuffedSize + 5] = FRAME_FLAG;
    }

    enum CommState currentState = STATE_START;
    (void)signal(SIGALRM, alarmHandler);

    struct timeval start;
    gettimeofday(&start, NULL);

    if(write(fd, frame, (stuffedSize + 6)) < 0) {
        free(frame);
        perror("Error writing send command");
        return -1;
    }
    alarm(connectionParameters.timeout);
    unsigned char receivedControl = 0, receivedAddress = 0;

    while (currentState != STATE_STOP && alarmCount <= connectionParameters.nRetransmissions) {
        unsigned char byte = 0;
        int bytesRead;
        if((bytesRead = read(fd, &byte, sizeof(byte))) < 0) {
            free(frame);
            perror("Error reading command");
            return -1;
        }
        if(bytesRead > 0) {
            switch (currentState) {
            case STATE_START:
                receivedControl = 0;
                receivedAddress = 0;
                if(byte == FRAME_FLAG) currentState = STATE_FLAG_RECEIVED;
                break;
            case STATE_FLAG_RECEIVED:
                if(byte == FRAME_FLAG) continue;
                if(byte == ADDR_SENDER || byte == ADDR_RECEIVER) {
                    currentState = STATE_ADDRESS_RECEIVED;
                    receivedAddress = byte;
                } else currentState = STATE_START;
                break;
            case STATE_ADDRESS_RECEIVED:
                if(byte == RR0 || byte == RR1 || byte == REJ0 || byte == REJ1) {
                    currentState = STATE_CONTROL_RECEIVED;
                    receivedControl = byte;
                } else if(byte == FRAME_FLAG) currentState = STATE_FLAG_RECEIVED;
                else currentState = STATE_START;
                break;
            case STATE_CONTROL_RECEIVED:
                if(byte == (receivedControl ^ receivedAddress)) currentState = STATE_BCC_OK;
                else if(byte == FRAME_FLAG) currentState = STATE_FLAG_RECEIVED;
                else currentState = STATE_START;
                break;
            case STATE_BCC_OK:
                if(byte == FRAME_FLAG) currentState = STATE_STOP;
                else currentState = STATE_START;
                break;
            default:
                currentState = STATE_START;
            }
        }

        if(currentState == STATE_STOP) {
            if(receivedControl == REJ0 || receivedControl == REJ1) {
                alarmEnabled = TRUE;
                alarmCount = 0;
                printf("Received reject; Retrying.\n");
            }
            if(receivedControl == RR0 || receivedControl == RR1) {
                struct timeval end;
                gettimeofday(&end, NULL);

                statistics.time_send_data += get_time_difference(start, end);

                alarmDisable();
                C_Ns = 1 - C_Ns;
                statistics.nFrames++;
                free(frame);
                return bufSize;
            }
        }
        
        if(alarmEnabled) {
            alarmEnabled = FALSE;
            if (alarmCount <= connectionParameters.nRetransmissions) {
                if(write(fd, frame, (stuffedSize + 6)) < 0) {
                    perror("Error writing send command");
                    return -1;
                }
                alarm(connectionParameters.timeout);
            }
            currentState = STATE_START;
        }
    }

    alarmDisable();
    free(frame);
    return -1;
}

/* Function to perform byte destuffing */
int byteDestuffing(unsigned char *buffer, int bufferSize, int *processedSize, unsigned char *bcc2Received) {
    if (buffer == NULL || processedSize == NULL) return -1;
    if (bufferSize < 1) return 0;

    unsigned char *readPtr = buffer;  // Pointer to read from buffer
    unsigned char *writePtr = buffer; // Pointer to write in-place to buffer

    while (readPtr < buffer + bufferSize) {
        if (*readPtr != FRAME_ESCAPE) {
            *writePtr++ = *readPtr++;
        } else {
            if (*(readPtr + 1) == FRAME_ESCAPED_FLAG) {
                *writePtr++ = FRAME_FLAG;
            } else if (*(readPtr + 1) == FRAME_ESCAPED_ESCAPE) {
                *writePtr++ = FRAME_ESCAPE;
            }
            readPtr += 2;
        }
    }

    *bcc2Received = *(writePtr - 1);
    *processedSize = writePtr - buffer - 1;
    return 0;
}

/* Function to read data from the link layer */
int llread(unsigned char *packet) {
    usleep(TPROP * 1000);

    enum CommState currentState = STATE_START;
    unsigned char controlByteReceived = 0;
    size_t packetIndex = 0;

    while (currentState != STATE_STOP) {
        unsigned char byte = 0;
        int bytesRead;
        if ((bytesRead = read(fd, &byte, sizeof(byte))) < 0) {
            perror("Error reading DISC command");
            return -1;
        }

        if (bytesRead > 0) {
            switch (currentState) {
            case STATE_START:
                controlByteReceived = 0;
                packetIndex = 0;
                if (byte == FRAME_FLAG) currentState = STATE_FLAG_RECEIVED;
                break;
            case STATE_FLAG_RECEIVED:
                if (byte == FRAME_FLAG) continue;
                if (byte == ADDR_SENDER) currentState = STATE_ADDRESS_RECEIVED;
                else currentState = STATE_START;
                break;
            case STATE_ADDRESS_RECEIVED:
                if (byte == INFO_CTRL_0 || byte == INFO_CTRL_1) {
                    currentState = STATE_CONTROL_RECEIVED;
                    controlByteReceived = byte;
                } else if (byte == FRAME_FLAG) currentState = STATE_FLAG_RECEIVED;
                else currentState = STATE_START;
                break;
            case STATE_CONTROL_RECEIVED:
                if (byte == (controlByteReceived ^ ADDR_SENDER)) currentState = STATE_DATA;
                else {
                    statistics.errorFrames++;
                    if (byte == FRAME_FLAG) currentState = STATE_FLAG_RECEIVED;
                    else currentState = STATE_START;
                }
                break;
            case STATE_DATA:
                if (byte == FRAME_FLAG) {
                    int processedSize = 0;
                    unsigned char bcc2Received = 0;

                    if (byteDestuffing(packet, packetIndex, &processedSize, &bcc2Received)) return -1;

                    unsigned char bcc2Calculated = 0x00;
                    for (size_t i = 0; i < processedSize; i++) bcc2Calculated ^= packet[i];

                    unsigned char responseControl, responseAddress = ADDR_SENDER;

                    if (bcc2Calculated == bcc2Received) {
                        responseControl = (controlByteReceived == INFO_CTRL_0) ? RR1 : RR0;
                    } else {
                        if ((C_Nr == 0 && controlByteReceived == INFO_CTRL_1) || (C_Nr == 1 && controlByteReceived == INFO_CTRL_0)) {
                            responseControl = (controlByteReceived == INFO_CTRL_0) ? RR1 : RR0;
                        } else {
                            responseControl = (controlByteReceived == INFO_CTRL_0) ? REJ0 : REJ1;
                        }
                    }

                    currentState = STATE_START;

                    int errorBcc1 = rand() % 100;
                    int errorBcc2 = rand() % 100;

                    // Simulation for report purposes
                    if ((C_Nr == 0 && controlByteReceived == INFO_CTRL_0) || (C_Nr == 1 && controlByteReceived == INFO_CTRL_1)) {
                        if (errorBcc1 < FAKE_BCC1_ERR) {
                            statistics.errorFrames++;
                            break;
                        }

                        if (errorBcc2 < FAKE_BCC2_ERR) {
                            responseControl = (controlByteReceived == INFO_CTRL_0) ? REJ0 : REJ1;
                        }
                    }

                    usleep(TPROP * 1000);

                    if (sendCommandPacket(responseAddress, responseControl)) return -1;

                    if (responseControl == REJ0 || responseControl == REJ1) {
                        statistics.errorFrames++;
                        break;
                    }

                    if ((C_Nr == 0 && controlByteReceived == INFO_CTRL_0) || (C_Nr == 1 && controlByteReceived == INFO_CTRL_1)) {
                        C_Nr = 1 - C_Nr;
                        printf("Bytes received: %d\n", processedSize);
                        statistics.bytes_read += processedSize + 6;
                        statistics.nFrames++;
                        return processedSize;
                    }
                    printf("Received duplicate packet\n");
                } else {
                    packet[packetIndex++] = byte;
                }
                break;
            default:
                currentState = STATE_START;
            }
        }
    }

    return -1;
}


void printStatistics() {
    printf("\n############ Communication Statistics ############\n");


    struct timeval end;
    gettimeofday(&end, NULL);
    
    // Calculate time taken for file transfer
    float timeTaken = get_time_difference(statistics.start, end);
    printf("Total time taken for file transfer: %.2f seconds\n", timeTaken);

    if (connectionParameters.role == LlRx) {
        // Receiver statistics
        printf("\n-- Receiver Statistics --\n");
        
        // Calculate theoretical efficiency
        float a = ((float) TPROP / 1000.0) / ((float) MAX_PAYLOAD_SIZE * 8.0 / connectionParameters.baudRate);
        float expectedFER = FAKE_BCC1_ERR / 100.0 + ((100.0 - FAKE_BCC1_ERR) / 100.0) * (FAKE_BCC2_ERR / 100.0);
        float theoreticalEfficiency = (1.0 - expectedFER) / (1 + 2 * a);
        
        printf("Total bytes received (post-destuffing): %lu bytes\n", statistics.bytes_read);
        printf("Number of valid frames received: %d frames\n", statistics.nFrames);
        printf("Average frame size: %.2f bytes\n", (float) statistics.bytes_read / statistics.nFrames);
        
        printf("\nData transfer rate: %.2f bits per second\n", (float) statistics.bytes_read * 8.0 / timeTaken);
        printf("Theoretical communication efficiency: %.2f\n", theoreticalEfficiency);
        printf("Targeted efficiency: %.2f\n", 
            ((float) (FILE_SIZE * 8.0) / timeTaken) / (float) connectionParameters.baudRate);
    } 
    else {
        // Transmitter statistics
        printf("\n-- Transmitter Statistics --\n");
        
        printf("Number of frames successfully sent: %d frames\n", statistics.nFrames);
        printf("Total time for sending control frames (with acknowledgment): %.2f seconds\n", statistics.time_send_control);
        printf("Total time for sending data frames (with acknowledgment): %.2f seconds\n", statistics.time_send_data);
        
        printf("\nAverage time per frame sent: %.2f seconds\n", 
            (statistics.time_send_data + statistics.time_send_control) / statistics.nFrames);
    }
    printf("\n##################################################\n");

}


int llclose(int showStatistics)
{
    if(connectionParameters.role == LlTx)
    {
        struct timeval temp_start, temp_end;

        gettimeofday(&temp_start, NULL);

        if(receivePacketWithRetransmission(ADDR_RECEIVER, CTRL_DISCONNECT, ADDR_SENDER, CTRL_DISCONNECT))
            return disconnectFD();

        statistics.nFrames++;
        gettimeofday(&temp_end, NULL);

        statistics.time_send_control += get_time_difference(temp_start, temp_end);
        statistics.nFrames++;

        if(sendCommandPacket(ADDR_RECEIVER, CTRL_UA)) return disconnectFD();
        printf("Disconnected\n");
    }
    if(connectionParameters.role == LlRx)
    {
        if(receivePacket(ADDR_SENDER, CTRL_DISCONNECT)) 
            return disconnectFD();
        statistics.bytes_read += FRAME_SIZE;
        statistics.nFrames++;

        if(receivePacketWithRetransmission(ADDR_RECEIVER, CTRL_UA, ADDR_RECEIVER, CTRL_DISCONNECT)) 
            return disconnectFD();
        statistics.bytes_read += FRAME_SIZE;
        statistics.nFrames++;

        printf("Disconnected\n");
    }

    if(showStatistics) printStatistics();
    return disconnectFD();
}