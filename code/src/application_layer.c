
// Application layer protocol implementation

#include "application_layer.h"

#include "link_layer.h"
#include <fcntl.h>

void send_file(const char *filename)
{
    int fd = open(filename, O_RDONLY);
    if (fd == -1)
    {
        printf("Error opening file\n");
        exit(-1);
    }
    unsigned char buf[MAX_PAYLOAD_SIZE];
    int bytes_read = 0;
    int packet_number = 1;
    while ((bytes_read = read(fd, buf, MAX_PAYLOAD_SIZE)) > 0)
    {
        llwrite(buf, bytes_read);
        packet_number++;
    }
    close(fd);
}
void receive_file(const char *filename)
{
    int fd = open(filename, O_WRONLY);
    if (fd == -1)
    {
        printf("Error opening file\n");
        exit(-1);
    }
    unsigned char buf[MAX_PAYLOAD_SIZE];
    int bytes_read = 0;
    int packet_number = 1;
    while ((bytes_read = llread(buf)) > 0)
    {
        write(fd, buf, bytes_read);
        packet_number++;
    }
    close(fd);
}

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayerRole rol = strcmp(role, "tx") == 0 ? LlTx : LlRx;
    LinkLayer link_layer;
    link_layer.baudRate = baudRate;
    link_layer.nRetransmissions = nTries;
    link_layer.role = role;
    link_layer.timeout = timeout;
    strcpy(link_layer.serialPort, serialPort);
    if (llopen(link_layer) == -1)
    {
        printf("Error opening link layer\n");
        exit(-1);
    }
    if (rol == LlTx)
    {
        send_file(filename);
    }
    else
    {
        receive_file(filename);
    }
    if (llclose(1) == -1)
    {
        printf("Error closing link layer\n");
        exit(-1);
    }
    printf("Application layer finished\n");
}