// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#define DATA 1
#define C_START 2
#define C_END 3

#define T_FILESIZE 0
#define T_FILENAME 1

#define MAX_FILENAME 50

enum state{
    RECV_START,
    RECV_CONT,
    RECV_END
};

typedef struct
{
    size_t file_size;
    char * file_name;
    size_t bytesRead;
} FileProps;

enum state stateReceive = RECV_START;
FileProps fileProps = {0, "", 0};


int sendPacketData(size_t nBytes, unsigned char *data) 
{
    if(data == NULL) return -1;
    
    unsigned char *packet = (unsigned char *) malloc(nBytes + 3);
    if(packet == NULL) return -1;
    
    packet[0] = DATA;
    packet[1] = nBytes >> 8;
    packet[2] = nBytes & 0xFF;

    memcpy(packet + 3, data, nBytes);

    int result = llwrite(packet, nBytes + 3);

    free(packet);

    return result;
}

unsigned char * itouchar(size_t value, unsigned char *size)
{
    if (size == NULL) return NULL; 
    
    size_t tmp_value = value;
    size_t length = 0;
    do {
        length++;
        tmp_value >>= 8;
    } while (tmp_value);

    unsigned char *bytes = malloc(length);
    if (bytes == NULL) return NULL;
    

    for (size_t i = 0; i < length; i++, value >>= 8)
        bytes[i] = value & 0xFF;

    *size = length;
    return bytes;
}

size_t uchartoi (unsigned char n, unsigned char * numbers)
{
    if(numbers == NULL) return 0;
    size_t value = 0;
    size_t power = 1;
    for(int i = 0; i < n; i++, power <<= 8){
        value += numbers[i] * power;
    }
    return value;
}

int sendPacketControl(unsigned char C, const char * filename, size_t file_size)
{
    if(filename == NULL) return -1;
    
    unsigned char L1 = 0;
    unsigned char * V1 = itouchar(file_size, &L1);
    if(V1 == NULL) return -1;

    unsigned char L2 = (unsigned char) strlen(filename);
    unsigned char *packet = (unsigned char *) malloc(5 + L1 + L2);
    if(packet == NULL) {
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

unsigned char * readPacketData(unsigned char *buff, size_t *newSize)
{
    if (buff == NULL) return NULL;
    if (buff[0] != DATA) return NULL;

    *newSize = buff[1] * 256 + buff[2];

    return buff + 3;
}

int readPacketControl(unsigned char * buff)
{   
    if (buff == NULL) return -1;
    size_t indx = 0;

    char * file_name = malloc(MAX_FILENAME);
    if(file_name == NULL) return -1;

    if(buff[indx] == C_START) stateReceive = RECV_CONT;
    else if(buff[indx] == C_END) stateReceive = RECV_END;
    else {
        free(file_name);
        return -1;
    }

    indx++;
    if (buff[indx++] != T_FILESIZE) return -1;
    unsigned char L1 = buff[indx++];
    unsigned char * V1 = malloc(L1);
    if(V1 == NULL) return -1;
    memcpy(V1, buff + indx, L1); indx += L1;
    size_t file_size = uchartoi(L1, V1);
    free(V1);

    if(buff[indx++] != T_FILENAME) return -1;
    unsigned char L2 = buff[indx++];
    memcpy(file_name, buff + indx, L2);
    file_name[L2] = '\0';

    if(buff[0] == C_START){
        fileProps.file_size = file_size;
        fileProps.file_name = file_name;
        printf("[INFO] Started receiving file: '%s'\n", file_name);
    }
    if(buff[0] == C_END){
        if (fileProps.file_size != fileProps.bytesRead) {
            perror("Number of bytes read doesn't match size of file\n");
        }
        if(strcmp(fileProps.file_name, file_name)){
            perror("Names of file given in the start and end packets don't match\n");
        }
        printf("[INFO] Finished receiving file: '%s'\n", file_name);
    }
    
    free(file_name);
    return 0;
}

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    if(serialPort == NULL || role == NULL || filename == NULL){
        perror("Initialization error: One or more required arguments are NULL.");
        return;
    }

    if (strlen(filename) > MAX_FILENAME) {
        printf("The lenght of the given file name is greater than what is supported: %d characters'\n", MAX_FILENAME);
        return;
    }
        
    LinkLayer connectionParametersApp;
    strncpy(connectionParametersApp.serialPort, serialPort, sizeof(connectionParametersApp.serialPort)-1);
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
        if(buffer == NULL) {
            perror("Memory allocation error at buffer creation.");
            llclose(FALSE);
            return;
        }

        FILE* file = fopen(filename, "rb");
        if(file == NULL) {
            perror("File error: Unable to open the file for reading.");
            fclose(file);
            free(buffer);
            llclose(FALSE);
            return;
        }

        fseek(file, 0, SEEK_END);
        size_t file_size = ftell(file);
        rewind(file);

        if(sendPacketControl(C_START, filename, file_size) == -1) {
            perror("Transmission error: Failed to send the START packet control.");
            fclose(file);
            llclose(FALSE);
            return;
        }

        while ((bytesRead = fread(buffer, 1, MAX_PAYLOAD_SIZE, file)) > 0) {
            size_t sended_bytes = 0;
            sended_bytes = sendPacketData(bytesRead, buffer);
            
            if(sended_bytes == -1){
                perror("Transmission error: Failed to send the DATA packet control.");
                fclose(file);
                llclose(FALSE);
                return;
            }
        }

        if(sendPacketControl(C_END, filename, file_size) == -1){
            perror("Transmission error: Failed to send the END packet control.");
            fclose(file);
            llclose(FALSE);
            return;
        }

        fclose(file);
    } 
    
    if (connectionParametersApp.role == LlRx) {
        
        unsigned char * buf = malloc(MAX_PAYLOAD_SIZE + 20);
        unsigned char * packet = malloc(MAX_PAYLOAD_SIZE + 20);
        if(buf == NULL || packet == NULL){
            perror("Initialization error: One or more buffers pointers are NULL.");
            llclose(FALSE);
            return;
        }

        
        FILE *file = fopen(filename, "wb");
        // To save the file with the same name as the one sent, use:
        // FILE *file = fopen(fileProps.file_name, "wb");
        
        if(file == NULL) {
            perror("File error: Unable to open the file for writing.");
            fclose(file);
            llclose(FALSE);
            return;
        }

        size_t bytes_readed = 0;
        
        while(stateReceive != RECV_END){
            bytes_readed = llread(buf);

            if(bytes_readed == -1) {
                perror("Link layer error: Failed to read from the link.");
                fclose(file);
                llclose(FALSE);
                return;
            }
            
            if(buf[0] == C_START || buf[0] == C_END){
                if(readPacketControl(buf) == -1) {
                    perror("Packet error: Failed to read control packet.");
                    fclose(file);
                    llclose(FALSE);
                    return;
                }
            }else if(buf[0] == DATA){
                packet = readPacketData(buf, &bytes_readed);
                if(packet == NULL) {
                    perror("Packet error: Failed to read data packet.");
                    fclose(file);
                    llclose(FALSE);
                    return;
                }
                fwrite(packet, 1, bytes_readed, file);
                fileProps.bytesRead += bytes_readed;
            }
        }

        fclose(file);
    }


    if (llclose(TRUE) == -1) {
        perror("Link layer error: Failed to close the connection.");
        return;
    }
}
