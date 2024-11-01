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

/*FLAGS*/
#define FLAG        0x7E
#define ESC         0x7D
#define ESC_FLAG    0x5E
#define ESC_ESC     0x5D

/*Adress Field*/
#define A_SEND      0x03
#define A_RECV      0x01

/*Control Field*/
#define C_SET       0x03
#define C_UA        0x07
#define C_DISC      0x0B

/*Information Field*/
#define C_INF0      0x00
#define C_INF1      0x40

#define RR0         0x05
#define RR1         0x85
#define REJ0        0x01
#define REJ1        0x81

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

enum state{
    START, 
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_OK,
    DATA,
    STOP
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

int send_packet_command( unsigned char A, unsigned char C)
{
    unsigned char buf[FRAME_SIZE] = {FLAG, A, C, 0, FLAG};
    buf[3] = buf[1] ^ buf[2];
    
    if(write(fd, buf, FRAME_SIZE) < 0)
    {
        perror("Error write send command");
        return -1;
    }
    return 0;
}

int receivePacket(unsigned char A_EXPECTED, unsigned char C_EXPECTED) 
{
    enum state enum_state = START;
    
    while (enum_state != STOP)
    {
        unsigned char byte = 0;
        int bytes;
        if((bytes = read(fd, &byte, sizeof(byte))) < 0)
        {
            perror("Error read DISC command");
            return -1;
        }
        if(bytes > 0){
            switch (enum_state)
            {
            case START:
                if(byte == FLAG) enum_state = FLAG_RCV;
                break;
            case FLAG_RCV:
                if(byte == FLAG) continue;
                if(byte == A_EXPECTED) enum_state = A_RCV;
                else enum_state = START;
                break;
            case A_RCV:
                if(byte == C_EXPECTED) enum_state = C_RCV;
                else if(byte == FLAG) enum_state = FLAG_RCV;
                else enum_state = START;
                break;
            case C_RCV:
                if(byte == (C_EXPECTED ^ A_EXPECTED)) enum_state = BCC_OK;
                else if(byte == FLAG) enum_state = FLAG_RCV;
                else enum_state = START;
                break;
            case BCC_OK:
                if(byte == FLAG) enum_state = STOP;
                else enum_state = START;
                break;      
            default:
                enum_state = START;
            }
        }
    }
    
    return 0; 
}

int receivePacketRetransmission(unsigned char A_EXPECTED, unsigned char C_EXPECTED, unsigned char A_TO_SEND, unsigned char C_TO_SEND)
{
    enum state enum_state = START;
    (void)signal(SIGALRM, alarmHandler);
    if(send_packet_command(A_TO_SEND, C_TO_SEND)) return -1;
    alarm(connectionParameters.timeout);
    printf("Alarm count: alarmCount is %d\n", alarmCount);
    printf("Connection retansmissions:  %d\n", connectionParameters.nRetransmissions);

    while (enum_state != STOP && alarmCount <= connectionParameters.nRetransmissions)
    {
        unsigned char byte = 0;
        int bytes;
        if((bytes = read(fd, &byte, sizeof(byte))) < 0)
        {
            perror("Error read UA command");
            return -1;
        }
        if(bytes > 0){
            switch (enum_state)
            {
            case START:
                if(byte == FLAG) enum_state = FLAG_RCV;
                break;
            case FLAG_RCV:
                if(byte == FLAG) continue;
                if(byte == A_EXPECTED) enum_state = A_RCV;
                else enum_state = START;
                break;
            case A_RCV:
                if(byte == C_EXPECTED) enum_state = C_RCV;
                else if(byte == FLAG) enum_state = FLAG_RCV;
                else enum_state = START;
                break;
            case C_RCV:
                if(byte == (C_EXPECTED ^ A_EXPECTED)) enum_state = BCC_OK;
                else if(byte == FLAG) enum_state = FLAG_RCV;
                else enum_state = START;
                break;
            case BCC_OK:
                if(byte == FLAG) enum_state = STOP;
                else enum_state = START;
                break;      
            default:
                enum_state = START;
            }
        }

        if(enum_state == STOP) 
        {
            alarmDisable();
            return 0;
        }
        
        if(alarmEnabled)
        {
            alarmEnabled = FALSE;
            if (alarmCount <= connectionParameters.nRetransmissions) {
                if(send_packet_command(A_TO_SEND, C_TO_SEND)) return -1;
                alarm(connectionParameters.timeout);
            }
            enum_state = START;
        }
    }

    alarmDisable();
    
    return -1;
}

int llopen(LinkLayer connectionParametersApp)
{
    gettimeofday(&statistics.start, NULL);

    memcpy(&connectionParameters, &connectionParametersApp, sizeof(connectionParametersApp));

    if (connectFD(connectionParameters) == -1)
        return -1;

    if(connectionParameters.role == LlTx){
        struct timeval temp_start, temp_end;
        gettimeofday(&temp_start, NULL);

        if(receivePacketRetransmission(A_SEND, C_UA, A_SEND, C_SET)) return -1;
        printf("receive transmission with sucess!\n");

        statistics.nFrames++;

        gettimeofday(&temp_end, NULL);

        statistics.time_send_control += get_time_difference(temp_start, temp_end);

        printf("Connection established\n");
    }
    if(connectionParameters.role == LlRx) {
        srand(time(NULL));
        if(receivePacket(A_SEND, C_SET)) return -1;
        statistics.nFrames++;
        statistics.bytes_read += FRAME_SIZE;
        if(send_packet_command(A_SEND, C_UA)) return -1;
        printf("Connection established\n");
    }
    return 0;
}

const unsigned char * byteStuffing(const unsigned char *buf, int bufSize, int *newSize)
{
    if(buf == NULL || newSize == NULL) return NULL;

    unsigned char *result = (unsigned char *) malloc(bufSize * 2 + 1);
    if(result == NULL) return NULL;
    size_t j = 0;

    for(size_t i = 0; i < bufSize; i++, j++){
        if(buf[i] == FLAG){
            result[j++] = ESC;
            result[j] = ESC_FLAG;
        }
        else if (buf[i] == ESC) {
            result[j++] = ESC;
            result[j] = ESC_ESC;
        }
        else
            result[j] = buf[i];
    }

    *newSize = (int) j;
    result = realloc(result, j);

    if (result == NULL) return NULL;

    return result;
}

int llwrite(const unsigned char *buf, int bufSize)
{
    if(buf == NULL) return -1;

    int newSize;
    const unsigned char *newBuf = byteStuffing(buf, bufSize, &newSize);
    if(newBuf == NULL) return -1;
    printf("Bytes sent: %d\n", newSize);
    unsigned char *trama = (unsigned char *) malloc(newSize + 6);
    if(trama == NULL){
        free((unsigned char *) newBuf);
        return -1;
    } 

    trama[0] = FLAG;
    trama[1] = A_SEND;
    trama[2] = (C_Ns) ? C_INF1 : C_INF0;
    trama[3] = trama[1] ^ trama[2];
    memcpy(trama + 4, newBuf, newSize);


    unsigned char bcc2 = 0x00;
    for(size_t i = 0; i < bufSize; i++) bcc2 ^=  buf[i];
    trama[newSize + 4] = bcc2; 
    if(bcc2 == FLAG){
        trama[newSize + 4] = ESC;
        newSize++;
        trama[newSize + 4] = ESC_FLAG;
        trama = realloc(trama, newSize + 6);
        trama[newSize + 5] = FLAG;
    }else{
        trama[newSize + 5] = FLAG;
    }
    

    enum state enum_state = START;
    (void)signal(SIGALRM, alarmHandler);

    struct timeval temp_start;
    gettimeofday(&temp_start, NULL);

    if(write(fd, trama, (newSize + 6)) < 0)
    {
        free(trama);
        perror("Error write send command");
        return -1;
    }
    alarm(connectionParameters.timeout); 
    unsigned char C_received = 0, A_received = 0;

    while (enum_state != STOP && alarmCount <= connectionParameters.nRetransmissions)
    {
        unsigned char byte = 0;
        int bytes;
        if((bytes = read(fd, &byte, sizeof(byte))) < 0)
        {
            free(trama);
            perror("Error read command");
            return -1;
        }
        if(bytes > 0){
            switch (enum_state)
            {
            case START:
                C_received = 0;
                A_received = 0;
                if(byte == FLAG) enum_state = FLAG_RCV;
                break;
            case FLAG_RCV:
                if(byte == FLAG) continue;
                if(byte == A_SEND || byte == A_RECV) {
                    enum_state = A_RCV;
                    A_received = byte;
                }
                else enum_state = START;
                break;
            case A_RCV:
                if(byte == RR0 || byte == RR1 || byte == REJ0 || byte == REJ1) {
                    enum_state = C_RCV;
                    C_received = byte;
                }
                else if(byte == FLAG) enum_state = FLAG_RCV;
                else enum_state = START;
                break;
            case C_RCV:
                if(byte == (C_received ^ A_received)) enum_state = BCC_OK;
                else if(byte == FLAG) enum_state = FLAG_RCV;
                else enum_state = START;
                break;
            case BCC_OK:
                if(byte == FLAG) enum_state = STOP;
                else enum_state = START;
                break;
            default:
                enum_state = START;
            }
        }

        if(enum_state == STOP) 
        {
            if(C_received == REJ0 || C_received == REJ1){
                alarmEnabled = TRUE;
                alarmCount = 0; 
                printf("Received reject; Second try.\n");
            }
            if(C_received == RR0 || C_received == RR1) {
                struct timeval temp_end;
                gettimeofday(&temp_end, NULL);

                statistics.time_send_data += get_time_difference(temp_start, temp_end);

                alarmDisable();
                C_Ns = 1 - C_Ns;
                statistics.nFrames++;
                free(trama);
                return bufSize;
            }
        }
        
        if(alarmEnabled)
        {
            alarmEnabled = FALSE;

            if (alarmCount <= connectionParameters.nRetransmissions) {
                if(write(fd, trama, (newSize + 6)) < 0)
                {
                    perror("Error write send command");
                    return -1;
                }
                alarm(connectionParameters.timeout);
            }

            enum_state = START;
        }
    }

    alarmDisable();
    free(trama);

    return -1;
}

int byteDestuffing(unsigned char *buf, int bufSize, int *newSize, unsigned char *bcc2_received)
{
    if (buf == NULL || newSize == NULL) return -1;
    if (bufSize < 1) return 0;

    unsigned char *read = buf;           // Pointer for reading from buf
    unsigned char *write = buf;          // Pointer for writing to buf

    while (read < buf + bufSize) {
        if (*read != ESC) {
            *write++ = *read++;
        } else {
            if (*(read + 1) == ESC_FLAG) {
                *write++ = FLAG;
            } else if (*(read + 1) == ESC_ESC) {
                *write++ = ESC;
            }
            read += 2;
        }
    }

    *bcc2_received = *(write - 1);
    *newSize = write - buf - 1;
    return 0;
}

int llread(unsigned char *packet)
{
    usleep(TPROP * 1000);

    enum state enum_state = START;
    unsigned char C_received = 0;
    size_t pkt_indx = 0;
    while (enum_state != STOP)
    {
        unsigned char byte = 0;
        int bytes;
        if((bytes = read(fd,&byte, sizeof(byte))) < 0)
        {
            perror("Error read DISC command");
            return -1;
        }
        if(bytes > 0){
            switch (enum_state)
            {
            case START:
                C_received = 0;
                pkt_indx = 0;
                if(byte == FLAG) enum_state = FLAG_RCV;
                break;
            case FLAG_RCV:
                if(byte == FLAG) continue;
                if(byte == A_SEND) enum_state = A_RCV;
                else enum_state = START;
                break;
            case A_RCV:
                if(byte == C_INF0 || byte == C_INF1){
                    enum_state = C_RCV;
                    C_received = byte;
                }
                else if(byte == FLAG) enum_state = FLAG_RCV;
                else enum_state = START;
                break;
            case C_RCV:
                if (byte == (C_received ^ A_SEND)) enum_state = DATA; 
                else {
                    statistics.errorFrames++;
                    if(byte == FLAG) enum_state = FLAG_RCV;
                    else enum_state = START;
                }
                break;
            case DATA: 
                if(byte == FLAG) {
                    int newSize = 0;
                    unsigned char bcc2_received = 0;
                    
                    if (byteDestuffing(packet, pkt_indx, &newSize, &bcc2_received)) return -1;

                    unsigned char bcc2 = 0x00;
                    for (size_t i = 0; i < newSize; i++) bcc2 ^= packet[i];

                    unsigned char C_respons, A_respons;

                    if (bcc2 == bcc2_received) {
                        C_respons = (C_received == C_INF0)? RR1 : RR0;
                        A_respons = A_SEND;
                    }
                    else {
                        if ((C_Nr == 0 && C_received == C_INF1) || (C_Nr == 1 && C_received == C_INF0)) {
                            C_respons = (C_received == C_INF0)? RR1 : RR0;
                            A_respons = A_SEND;
                        }
                        else {
                            C_respons = (C_received == C_INF0) ? REJ0 : REJ1;
                            A_respons = A_SEND;
                        }
                    }

                    enum_state = START;

                    int error_in_bcc1 = rand() % 100;
                    int error_in_bcc2 = rand() % 100;

                    // NOTE: code only for report
                    if ((C_Nr == 0 && C_received == C_INF0) || (C_Nr == 1 && C_received == C_INF1)) {
                        if (error_in_bcc1 <= FAKE_BCC1_ERR - 1) {
                            statistics.errorFrames++;
                            break;
                        }

                        if (error_in_bcc2 <= FAKE_BCC2_ERR - 1) {
                            C_respons = (C_received == C_INF0) ? REJ0 : REJ1;
                            A_respons = A_SEND;
                        }
                    }

                    usleep(TPROP * 1000);

                    if (send_packet_command(A_respons, C_respons)) return -1;

                    if (C_respons == REJ0 || C_respons == REJ1) {
                        statistics.errorFrames++;
                        break;
                    }

                    if ((C_Nr == 0 && C_received == C_INF0) || (C_Nr == 1 && C_received == C_INF1)){
                        C_Nr = 1 - C_Nr;
                        printf("Bytes received: %d\n", newSize);
                        statistics.bytes_read += newSize + 6;
                        statistics.nFrames++;
                        return newSize;
                    }
                    printf("Received duplicate\n");
                } else packet[pkt_indx++] = byte;
                
                break;
            default:
                enum_state = START;
            }
        }
    }
    return -1;
}

void printStatistics()
{
    printf("\n======== Statistics ========\n");

    if (connectionParameters.role == LlRx) {

        float a = (float) ((float) TPROP / 1000) / (float) ((float) MAX_PAYLOAD_SIZE * 8.0 / (float) connectionParameters.baudRate);

        float EXPECTED_FER = FAKE_BCC1_ERR/100.0 + (100.0 - FAKE_BCC1_ERR)/100.0 * FAKE_BCC2_ERR/100.0;

        printf("\nNumber of bytes received (after destuffing): %lu\n", statistics.bytes_read);

        printf("\nNumber of good frames received: %d frames\n", statistics.nFrames);

        struct timeval end;
        gettimeofday(&end, NULL);

        printf("\nTime taken to download file: %f seconds\n", get_time_difference(statistics.start, end));

        printf("\nAverage size of a frame: %ld bytes per frame\n", statistics.bytes_read / statistics.nFrames);

        printf("\nDébito recebido (bits/s): %f\n", (float) statistics.bytes_read * 8.0 / get_time_difference(statistics.start, end));

        printf("\nEficiencia teorica: %f", (1.0 - EXPECTED_FER) / (1 + 2*a));

        printf("\nEficiencia pedida %f\n", ((float) ((float) FILE_SIZE * 8.0) / get_time_difference(statistics.start, end)) / (float) connectionParameters.baudRate);
    }

    else {
        printf("\nNumber of good frames sent: %d frames\n", statistics.nFrames);

        printf("\nTime taken to send and receive confirmation of receival of control frames: %f seconds\n", statistics.time_send_control);

        printf("\nTime taken to send and receive confirmation of receival of data frames: %f seconds\n", statistics.time_send_data);

        printf("\nAverage time taken to send a frame: %f seconds\n", (statistics.time_send_data + statistics.time_send_control) / statistics.nFrames);
    }

    printf("\n============================\n");
}

int llclose(int showStatistics)
{
    if(connectionParameters.role == LlTx)
    {
        struct timeval temp_start, temp_end;

        gettimeofday(&temp_start, NULL);

        if(receivePacketRetransmission(A_RECV, C_DISC, A_SEND, C_DISC))
            return disconnectFD();

        statistics.nFrames++;
        gettimeofday(&temp_end, NULL);

        statistics.time_send_control += get_time_difference(temp_start, temp_end);
        statistics.nFrames++;

        if(send_packet_command(A_RECV, C_UA)) return disconnectFD();
        printf("Disconnected\n");
    }
    if(connectionParameters.role == LlRx)
    {
        if(receivePacket(A_SEND, C_DISC)) return disconnectFD();
        statistics.nFrames++;
        statistics.bytes_read += FRAME_SIZE;

        if(receivePacketRetransmission(A_RECV, C_UA, A_RECV, C_DISC)) return disconnectFD();
        statistics.nFrames++;           // Retirar este linha se se mudar em cima para send_packet_command
        statistics.bytes_read += FRAME_SIZE;   // Retirar este linha se se mudar em cima para send_packet_command

        printf("Disconnected\n");
        // if(send_packet_command(A_RECV, C_DISC)) return disconnectFD();
        
    }

    if(showStatistics) printStatistics();
    return disconnectFD();
}