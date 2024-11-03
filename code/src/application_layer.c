#include "application_layer.h"
#include "link_layer.h"
#include "utils.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>

#define DATA 1
#define C_START 2
#define C_END 3

#define T_FILESIZE 0
#define T_FILENAME 1

#define MAX_FILENAME 50
#define PROGRESS_BAR_WIDTH 50

enum state {
    RECV_START,
    RECV_CONT,
    RECV_END
};

typedef struct {
    size_t file_size;
    char * file_name;
    size_t bytesRead;
} FileProps;

enum state stateReceive = RECV_START;
FileProps fileProps = {0, "", 0};

void showProgress(size_t bytesRead, size_t fileSize, double timeElapsed, unsigned char role) {
    double progress = (double) bytesRead / fileSize;
    int barWidth = (int)(progress * PROGRESS_BAR_WIDTH);
    double speed = (bytesRead / 1024.0) / timeElapsed;

    if (role == LlRx) {
        printf("\r[");
        for (int i = 0; i < PROGRESS_BAR_WIDTH; i++) {
            if (i < barWidth) printf("\033[0;32m#\033[0m");
            else printf(" ");
        }

        printf("] " COLOR_POSITIVE"%.2f%%, %.2f KB/s, "COLOR_RESET, progress * 100, speed);
    } else {
        printf(COLOR_POSITIVE "%.2f%%" "%.2f" "KB/s, "COLOR_RESET, progress * 100, speed);
    }

    fflush(stdout);
}

int sendPacketData(size_t nBytes, unsigned char *data) {
    if (data == NULL) return -1;
    
    unsigned char *packet = (unsigned char *) malloc(nBytes + 3);
    if (packet == NULL) return -1;
    
    packet[0] = DATA;
    packet[1] = nBytes >> 8;
    packet[2] = nBytes & 0xFF;
    memcpy(packet + 3, data, nBytes);

    int result = llwrite(packet, nBytes + 3);

    free(packet);
    return result;
}

int sendPacketControl(unsigned char C, const char * filename, size_t file_size) {
    if (filename == NULL) return -1;
    
    unsigned char L1 = 0;
    unsigned char * V1 = size_t_to_bytes(file_size, &L1);
    if (V1 == NULL) return -1;

    unsigned char L2 = (unsigned char) strlen(filename);
    unsigned char *packet = (unsigned char *) malloc(5 + L1 + L2);
    if (packet == NULL) {
        free(V1);
        return -1;
    }

    size_t indx = 0;
    packet[indx++] = C;
    packet[indx++] = T_FILESIZE;
    packet[indx++] = L1;
    memcpy(packet + indx, V1, L1); indx += L1;
    packet[indx++] = T_FILENAME;
    packet[indx++] = L2;
    memcpy(packet + indx, filename, L2); indx += L2;  

    free(V1);
    int res = llwrite(packet, (int) indx);

    free(packet);
    return res;
}

unsigned char * readPacketData(unsigned char *buff, size_t *newSize) {
    if (buff == NULL) return NULL;
    if (buff[0] != DATA) return NULL;

    *newSize = buff[1] * 256 + buff[2];
    return buff + 3;
}

int readPacketControl(unsigned char * buff) {
    if (buff == NULL) return -1;
    size_t indx = 0;

    char * file_name = malloc(MAX_FILENAME);
    if (file_name == NULL) return -1;

    if (buff[indx] == C_START) stateReceive = RECV_CONT;
    else if (buff[indx] == C_END) stateReceive = RECV_END;
    else {
        free(file_name);
        return -1;
    }

    indx++;
    if (buff[indx++] != T_FILESIZE) return -1;
    unsigned char L1 = buff[indx++];
    unsigned char * V1 = malloc(L1);
    if (V1 == NULL) return -1;
    memcpy(V1, buff + indx, L1); indx += L1;
    size_t file_size = bytes_to_size_t(L1, V1);
    free(V1);

    if (buff[indx++] != T_FILENAME) return -1;
    unsigned char L2 = buff[indx++];
    memcpy(file_name, buff + indx, L2);
    file_name[L2] = '\0';

    if (buff[0] == C_START) {
        fileProps.file_size = file_size;
        fileProps.file_name = file_name;
        printf("[INFO] Started receiving file: '%s'\n", file_name);
    }
    if (buff[0] == C_END) {
        if (fileProps.file_size != fileProps.bytesRead) {
            perror("Number of bytes read doesn't match size of file\n");
        }
        if (strcmp(fileProps.file_name, file_name)) {
            perror("Names of file given in the start and end packets don't match\n");
        }
        printf("[INFO] Finished receiving file: '%s'\n", file_name);
    }
    
    free(file_name);
    return 0;
}

void applicationLayer(const char *serialPort, const char *role, int baudRate, int nTries, int timeout, const char *filename) {
    if (serialPort == NULL || role == NULL || filename == NULL) {
        perror("Initialization error: One or more required arguments are NULL.");
        return;
    }

    if (strlen(filename) > MAX_FILENAME) {
        printf("The length of the given file name is greater than what is supported: %d characters\n", MAX_FILENAME);
        return;
    }
    
    LinkLayer connectionParametersApp;
    strncpy(connectionParametersApp.serialPort, serialPort, sizeof(connectionParametersApp.serialPort) - 1);
    connectionParametersApp.role = strcmp(role, "tx") ? LlRx : LlTx;
    connectionParametersApp.baudRate = baudRate;
    connectionParametersApp.nRetransmissions = nTries;
    connectionParametersApp.timeout = timeout;

    if (llopen(connectionParametersApp) == -1) {
        perror("Link layer error: Failed to open the connection.");
        llclose(FALSE);
        return;
    }
    
    if (connectionParametersApp.role == LlTx) {
        size_t bytesRead = 0;
        unsigned char *buffer = (unsigned char *) malloc(MAX_PAYLOAD_SIZE + 20);
        if (buffer == NULL) {
            perror("Memory allocation error at buffer creation.");
            llclose(FALSE);
            return;
        }

        FILE* file = fopen(filename, "rb");
        if (file == NULL) {
            perror("File error: Unable to open the file for reading.");
            fclose(file);
            free(buffer);
            llclose(FALSE);
            return;
        }

        fseek(file, 0, SEEK_END);
        size_t file_size = ftell(file);
        rewind(file);

        if (sendPacketControl(C_START, filename, file_size) == -1) {
            perror("Transmission error: Failed to send the START packet control.");
            fclose(file);
            llclose(FALSE);
            return;
        }

        struct timeval start, current;
        gettimeofday(&start, NULL);

        while ((bytesRead = fread(buffer, 1, MAX_PAYLOAD_SIZE, file)) > 0) {
            size_t sended_bytes = sendPacketData(bytesRead, buffer);
            
            if (sended_bytes == -1) {
                perror("Transmission error: Failed to send the DATA packet control.");
                fclose(file);
                llclose(FALSE);
                return;
            }

            gettimeofday(&current, NULL);
            double timeElapsed = get_time_difference(start, current);
            showProgress(ftell(file), file_size, timeElapsed, LlTx);
        }

        if (sendPacketControl(C_END, filename, file_size) == -1) {
            perror("Transmission error: Failed to send the END packet control.");
            fclose(file);
            llclose(FALSE);
            return;
        }

        fclose(file);
        printf("\n");
    } 
    
    if (connectionParametersApp.role == LlRx) {
        unsigned char * buf = malloc(MAX_PAYLOAD_SIZE + 20);
        unsigned char * packet = malloc(MAX_PAYLOAD_SIZE + 20);
        if (buf == NULL || packet == NULL) {
            perror("Initialization error: One or more buffers pointers are NULL.");
            llclose(FALSE);
            return;
        }

        FILE *file = fopen(filename, "wb");
        
        if (file == NULL) {
            perror("File error: Unable to open the file for writing.");
            fclose(file);
            llclose(FALSE);
            return;
        }

        size_t bytes_readed = 0;
        struct timeval start, current;
        gettimeofday(&start, NULL);

        while (stateReceive != RECV_END) {
            bytes_readed = llread(buf);

            if (bytes_readed == -1) {
                perror("Link layer error: Failed to read from the link.");
                fclose(file);
                llclose(FALSE);
                return;
            }
            
            if (buf[0] == C_START || buf[0] == C_END) {
                if (readPacketControl(buf) == -1) {
                    perror("Packet error: Failed to read control packet.");
                    fclose(file);
                    llclose(FALSE);
                    return;
                }
            } else if (buf[0] == DATA) {
                packet = readPacketData(buf, &bytes_readed);
                if (packet == NULL) {
                    perror("Packet error: Failed to read data packet.");
                    fclose(file);
                    llclose(FALSE);
                    return;
                }
                fwrite(packet, 1, bytes_readed, file);
                fileProps.bytesRead += bytes_readed;

                gettimeofday(&current, NULL);
                double timeElapsed = get_time_difference(start, current);
                showProgress(fileProps.bytesRead, fileProps.file_size, timeElapsed, LlRx);
            }
        }

        fclose(file);
        printf("\n");
    }

    if (llclose(TRUE) == -1) {
        perror("Link layer error: Failed to close the connection.");
        return;
    }
}
