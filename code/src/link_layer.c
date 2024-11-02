// Link layer protocol implementation

#include <fcntl.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>

#include "link_layer.h"
#include "serial_port.h"  // Include the serial port helper functions
#include "utils.h"
#include "state_machine.h"

LinkLayer connectionParameters;
Statistics stats = {0, 0, 0, 0, 0.0, 0.0}; // Holds frame count, byte count, and timing data
int isAlarmEnabled = FALSE;                // Tracks if an alarm is currently active
int alarmRetryCount = 0;                   // Counts the number of alarm retries

unsigned char sendSeqNum = 0;              // Send sequence number (Ns)
unsigned char recvSeqNum = 0;              // Receive sequence number (Nr)

// Opens the serial port and configures it using the openSerialPort function
int establishSerialConnection(LinkLayer connectionParametersApp) {
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
int terminateSerialConnection() {
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
    alarmRetryCount++;
    isAlarmEnabled = TRUE;
    printf("Alarm count: %d\n", alarmRetryCount);
}

void alarmDisable()
{
    alarm(0);
    isAlarmEnabled = FALSE;
    alarmRetryCount = 0;
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
        if(bytesRead > 0) 
            currentState = processByte(currentState, byte, expectedAddress, expectedControl);
    }
    
    return 0; 
}

/* Function to receive a packet with retransmission */
int receivePacketWithRetransmission(unsigned char expectedAddress, unsigned char expectedControl, unsigned char sendAddress, unsigned char sendControl) {
    enum CommState currentState = STATE_START;
    (void)signal(SIGALRM, alarmHandler);
    if(sendCommandPacket(sendAddress, sendControl)) return -1;
    alarm(connectionParameters.timeout);

    while (currentState != STATE_STOP && alarmRetryCount <= connectionParameters.nRetransmissions) {
        unsigned char byte = 0;
        int bytesRead;
        if((bytesRead = read(fd, &byte, sizeof(byte))) < 0) {
            perror("Error reading UA command");
            return -1;
        }
        if(bytesRead > 0) 
            currentState = processByte(currentState, byte, expectedAddress, expectedControl);

        if(currentState == STATE_STOP) {
            alarmDisable();
            return 0;
        }
        
        if(isAlarmEnabled) {
            isAlarmEnabled = FALSE;
            if (alarmRetryCount <= connectionParameters.nRetransmissions) {
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
int llopen(LinkLayer connectionParamsAppLayer) {
    gettimeofday(&stats.start, NULL);
    memcpy(&connectionParameters, &connectionParamsAppLayer, sizeof(connectionParamsAppLayer));

    if (establishSerialConnection(connectionParamsAppLayer) == -1)
        return -1;

    if(connectionParamsAppLayer.role == LlTx) {
        struct timeval start, end;
        gettimeofday(&start, NULL);

        if(receivePacketWithRetransmission(ADDR_SENDER, CTRL_UA, ADDR_SENDER, CTRL_SET)) return -1;
        printf("Received transmission successfully!\n");

        stats.nFrames++;
        gettimeofday(&end, NULL);
        stats.time_send_control += get_time_difference(start, end);

        printf("Connection established\n");
    }
    if(connectionParamsAppLayer.role == LlRx) {
        srand(time(NULL));
        if(receivePacket(ADDR_SENDER, CTRL_SET)) return -1;
        stats.nFrames++;
        stats.bytes_read += FRAME_SIZE;
        if(sendCommandPacket(ADDR_SENDER, CTRL_UA)) return -1;
        printf("Connection established\n");
    }
    return 0;
}

/* Function to write to the link layer */
int llwrite(const unsigned char *buffer, int bufSize) {
    if(buffer == NULL) return -1;

    int stuffedSize;
    const unsigned char *stuffedBuffer = byteStuffing(buffer, bufSize, &stuffedSize);
    if(stuffedBuffer == NULL) return -1;
    printf("Bytes sent: %d\n", stuffedSize);
    
    unsigned char *frame = (unsigned char *) malloc(stuffedSize + 6);
    if(frame == NULL) {
        free((unsigned char *) stuffedBuffer);
        return -1;
    }

    frame[0] = FRAME_FLAG;
    frame[1] = ADDR_SENDER;
    frame[2] = (sendSeqNum) ? INFO_CTRL_1 : INFO_CTRL_0;
    frame[3] = frame[1] ^ frame[2];

    memcpy(frame + FRAME_SIZE - 1, stuffedBuffer, stuffedSize);

    unsigned char bcc2 = 0x00;
    for(size_t i = 0; i < bufSize; i++) bcc2 ^= buffer[i];
    frame[stuffedSize + FRAME_SIZE - 1] = bcc2;

    if(bcc2 == FRAME_FLAG) {
        frame[stuffedSize + FRAME_SIZE - 1] = FRAME_ESCAPE;
        stuffedSize++;
        frame[stuffedSize + FRAME_SIZE - 1] = FRAME_ESCAPED_FLAG;
        frame = realloc(frame, stuffedSize + FRAME_SIZE + 1);
        frame[stuffedSize + FRAME_SIZE] = FRAME_FLAG;
    } else {
        frame[stuffedSize + FRAME_SIZE] = FRAME_FLAG;
    }

    enum CommState currentState = STATE_START;
    (void)signal(SIGALRM, alarmHandler);

    struct timeval start;
    gettimeofday(&start, NULL);

    if(write(fd, frame, (stuffedSize + FRAME_SIZE + 1)) < 0) {
        free(frame);
        perror("Error writing send command");
        return -1;
    }
    alarm(connectionParameters.timeout);
    unsigned char receivedControl = 0, receivedAddress = 0;

    while (currentState != STATE_STOP && alarmRetryCount <= connectionParameters.nRetransmissions) {
        unsigned char byte = 0;
        int bytesRead = read(fd, &byte, sizeof(byte));
        if (bytesRead < 0) {
            free(frame);
            perror("Error reading command");
            return -1;
        }

        if (bytesRead > 0) {
            switch (currentState) {
                case STATE_START:
                    if (byte == FRAME_FLAG) currentState = STATE_FLAG_RECEIVED;
                    break;

                case STATE_FLAG_RECEIVED:
                    if (byte == FRAME_FLAG) continue;
                    if (byte == ADDR_SENDER || byte == ADDR_RECEIVER) {
                        receivedAddress = byte;
                        currentState = STATE_ADDRESS_RECEIVED;
                    } else {
                        currentState = STATE_START;
                    }
                    break;

                case STATE_ADDRESS_RECEIVED:
                    if (byte == RR0 || byte == RR1 || byte == REJ0 || byte == REJ1) {
                        receivedControl = byte;
                        currentState = STATE_CONTROL_RECEIVED;
                    } else if (byte == FRAME_FLAG) {
                        currentState = STATE_FLAG_RECEIVED;
                    } else {
                        currentState = STATE_START;
                    }
                    break;

                case STATE_CONTROL_RECEIVED:
                    if (byte == (receivedControl ^ receivedAddress)) {
                        currentState = STATE_BCC_OK;
                    } else if (byte == FRAME_FLAG) {
                        currentState = STATE_FLAG_RECEIVED;
                    } else {
                        currentState = STATE_START;
                    }
                    break;

                case STATE_BCC_OK:
                    if (byte == FRAME_FLAG) currentState = STATE_STOP;
                    else currentState = STATE_START;
                    break;
            }
        }

        if (currentState == STATE_STOP) {
            if (receivedControl == REJ0 || receivedControl == REJ1) {
                isAlarmEnabled = TRUE;
                alarmRetryCount = 0;
                printf("Received reject; Retrying.\n");
            } else if (receivedControl == RR0 || receivedControl == RR1) {
                struct timeval end;
                gettimeofday(&end, NULL);

                stats.time_send_data += get_time_difference(start, end);
                alarmDisable();
                sendSeqNum ^= 1;
                stats.nFrames++;
                free(frame);
                return bufSize;
            }
        }

        if (isAlarmEnabled && alarmRetryCount <= connectionParameters.nRetransmissions) {
            isAlarmEnabled = FALSE;
            if (write(fd, frame, stuffedSize + FRAME_SIZE + 1) < 0) {
                perror("Error writing send command");
                return -1;
            }
            alarm(connectionParameters.timeout);
            currentState = STATE_START;
        }
    }

    alarmDisable();
    free(frame);
    return -1;
}

/* Function to read data from the link layer */
int llread(unsigned char *packet) {
    usleep(TPROP * 1000);

    enum CommState currentState = STATE_START;
    unsigned char controlByteReceived = 0;
    size_t packetIndex = 0;

    while (currentState != STATE_STOP) {
        unsigned char byte = 0;
        int bytesRead = read(fd, &byte, sizeof(byte));
        if (bytesRead < 0) {
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
                    currentState = (byte == ADDR_SENDER) ? STATE_ADDRESS_RECEIVED : STATE_START;
                    break;

                case STATE_ADDRESS_RECEIVED:
                    if (byte == INFO_CTRL_0 || byte == INFO_CTRL_1) {
                        controlByteReceived = byte;
                        currentState = STATE_CONTROL_RECEIVED;
                    } else {
                        currentState = (byte == FRAME_FLAG) ? STATE_FLAG_RECEIVED : STATE_START;
                    }
                    break;

                case STATE_CONTROL_RECEIVED:
                    if (byte == (controlByteReceived ^ ADDR_SENDER)) {
                        currentState = STATE_DATA;
                    } else {
                        stats.errorFrames++;
                        currentState = (byte == FRAME_FLAG) ? STATE_FLAG_RECEIVED : STATE_START;
                    }
                    break;

                case STATE_DATA:
                    if (byte == FRAME_FLAG) {
                        int processedSize = 0;
                        unsigned char bcc2Received = 0;

                        if (byteDestuffing(packet, packetIndex, &processedSize, &bcc2Received)) return -1;

                        unsigned char bcc2Calculated = 0x00;
                        for (size_t i = 0; i < processedSize; i++) bcc2Calculated ^= packet[i];

                        unsigned char responseControl = (bcc2Calculated == bcc2Received) ? 
                            ((controlByteReceived == INFO_CTRL_0) ? RR1 : RR0) : 
                            ((recvSeqNum == 0 && controlByteReceived == INFO_CTRL_1) || 
                            (recvSeqNum == 1 && controlByteReceived == INFO_CTRL_0) ? 
                            ((controlByteReceived == INFO_CTRL_0) ? RR1 : RR0) : 
                            ((controlByteReceived == INFO_CTRL_0) ? REJ0 : REJ1));

                        currentState = STATE_START;

                        if ((recvSeqNum == 0 && controlByteReceived == INFO_CTRL_0) || 
                            (recvSeqNum == 1 && controlByteReceived == INFO_CTRL_1)) {
                            if (rand() % 100 < FAKE_BCC1_ERR) {
                                stats.errorFrames++;
                                break;
                            }
                            if (rand() % 100 < FAKE_BCC2_ERR) responseControl = (controlByteReceived == INFO_CTRL_0) ? REJ0 : REJ1;
                        }

                        usleep(TPROP * 1000);

                        if (sendCommandPacket(ADDR_SENDER, responseControl)) return -1;

                        if (responseControl == REJ0 || responseControl == REJ1) {
                            stats.errorFrames++;
                            break;
                        }

                        if ((recvSeqNum == 0 && controlByteReceived == INFO_CTRL_0) || 
                            (recvSeqNum == 1 && controlByteReceived == INFO_CTRL_1)) {
                            recvSeqNum = 1 - recvSeqNum;
                            printf("Bytes received: %d\n", processedSize);
                            stats.bytes_read += processedSize + FRAME_SIZE + 1;
                            stats.nFrames++;
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
    float timeTaken = get_time_difference(stats.start, end);
    printf("Total time taken for file transfer: %.2f seconds\n", timeTaken);

    if (connectionParameters.role == LlRx) {
        // Receiver stats
        printf("\n-- Receiver Statistics --\n");
        
        // Calculate theoretical efficiency
        float a = ((float) TPROP / 1000.0) / ((float) MAX_PAYLOAD_SIZE * 8.0 / connectionParameters.baudRate);
        float expectedFER = FAKE_BCC1_ERR / 100.0 + ((100.0 - FAKE_BCC1_ERR) / 100.0) * (FAKE_BCC2_ERR / 100.0);
        float theoreticalEfficiency = (1.0 - expectedFER) / (1 + 2 * a);
        
        printf("Total bytes received (post-destuffing): %lu bytes\n", stats.bytes_read);
        printf("Number of valid frames received: %d frames\n", stats.nFrames);
        printf("Average frame size: %.2f bytes\n", (float) stats.bytes_read / stats.nFrames);
        
        printf("\nData transfer rate: %.2f bits per second\n", (float) stats.bytes_read * 8.0 / timeTaken);
        printf("Theoretical communication efficiency: %.2f\n", theoreticalEfficiency);
        printf("Targeted efficiency: %.2f\n", 
            ((float) (FILE_SIZE * 8.0) / timeTaken) / (float) connectionParameters.baudRate);
    } 
    else {
        // Transmitter stats
        printf("\n-- Transmitter Statistics --\n");
        
        printf("Number of frames successfully sent: %d frames\n", stats.nFrames);
        printf("Total time for sending control frames (with acknowledgment): %.2f seconds\n", stats.time_send_control);
        printf("Total time for sending data frames (with acknowledgment): %.2f seconds\n", stats.time_send_data);
        
        printf("\nAverage time per frame sent: %.2f seconds\n", 
            (stats.time_send_data + stats.time_send_control) / stats.nFrames);
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
            return terminateSerialConnection();

        stats.nFrames++;
        gettimeofday(&temp_end, NULL);

        stats.time_send_control += get_time_difference(temp_start, temp_end);
        stats.nFrames++;

        if(sendCommandPacket(ADDR_RECEIVER, CTRL_UA)) return terminateSerialConnection();
        printf("Disconnected\n");
    }
    if(connectionParameters.role == LlRx)
    {
        if(receivePacket(ADDR_SENDER, CTRL_DISCONNECT)) 
            return terminateSerialConnection();
        stats.bytes_read += FRAME_SIZE;
        stats.nFrames++;

        if(receivePacketWithRetransmission(ADDR_RECEIVER, CTRL_UA, ADDR_RECEIVER, CTRL_DISCONNECT)) 
            return terminateSerialConnection();
        stats.bytes_read += FRAME_SIZE;
        stats.nFrames++;

        printf("Disconnected\n");
    }

    if(showStatistics) printStatistics();
    return terminateSerialConnection();
}