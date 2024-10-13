// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

int fd;
struct termios oldtio;
struct termios newtio;
LinkLayer parameters;
int alarmEnabled = FALSE;
int alarmCount = 0;

void alarmHandler(int signal) {
    alarmEnabled = FALSE;
    alarmCount++;
    printf("Alarm #%d\n", alarmCount);
}
void send_ua_frame(int fd) {
    char ack[5];
    ack[0] = FLAG;
    ack[1] = RX_ADDR;
    ack[2] = TX_ADDR;
    ack[3] = ack[1] ^ ack[2];
    ack[4] = FLAG;
    write(fd, ack, 5);  // no need to check if it was sent correctly
}
int write_to_serial_port(int fd, const char *frame, int frame_size) {
    char response[5];
    int STOP = FALSE;
    alarmCount = 0;
    alarmEnabled = FALSE;
    while (alarmCount < parameters.nRetransmissions && !STOP) {
        if (!alarmEnabled) {
            int bytes = write(fd, frame, frame_size);
            printf("%d bytes written\n", bytes);
            alarm(parameters.timeout); // Set alarm to be triggered in X seconds
            alarmEnabled = TRUE;
        }
        int bytes = read(fd, response, 5);
        printf("%d bytes read\n", bytes);
        if (TRUE) { // process bytes with a state machine (for now we assume they are correct)
            // e.g. set -> ua; i -> rr
            STOP = TRUE;
            alarm(0);
        }
        printf("%d bytes read\n", bytes);
    }
}


////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    if (openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate) < 0)
    {
        return -1;

    }

    if (connectionParameters.role == LlTx) {
        char buf[5];
        buf[0] = FLAG;
        buf[1] = TX_ADDR;
        buf[2] = RX_ADDR;
        buf[3] = buf[1] ^ buf[2];
        buf[4] = FLAG;
        write_to_serial_port(buf, 5, fd);
    }
    else {
        char buf[5];
        read_from_serial_port(fd, buf, 5);
        // or llread(NULL), we expect no data;
    }

    return 1;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    // builds frame with data given by buf, and writes it to the serial port, using STOP & WAIT
    int frame_size = bufSize + 6;
    unsigned char frame[frame_size];
    frame[0] = FLAG;
    frame[1] = TX_ADDR;
    frame[2] = RX_ADDR;
    frame[3] = frame[1] ^ frame[2];
    int data_bcc_index = frame_size - 2;
    for (int i = 0; i < bufSize; i++) {
        frame[i + 4] = buf[i];
        frame[data_bcc_index] ^= buf[i];
    }
    frame[frame_size - 1] = FLAG;
    write_to_serial_port(fd, frame, frame_size);    // make sure it gets sent correctly

    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    int max_frame_size = MAX_PAYLOAD_SIZE + 6;
    char frame[max_frame_size];
    int bytes_read = read(fd, frame, max_frame_size);
    if (bytes_read == -1) {
        perror("read");
        exit(-1);
    }
    printf("%d bytes read\n", bytes_read);
    // process bytes_read with a state machine, then send response accordingly
    // for now, we just send an acknowledgment for the set frame (UA)
    send_ua_frame(fd);

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{
    // TODO

    int clstat = closeSerialPort();
    return clstat;
}
