// Virtual cable program to test serial port.
// Creates a pair of virtual Tx / Rx serial ports using "socat".
//
// Author: Manuel Ricardo [mricardo@fe.up.pt]
// Modified by: Eduardo Nuno Almeida [enalmeida@fe.up.pt]
// Modified by: Rui Prior [rcprior@fc.up.pt]

#include <fcntl.h>
#include <math.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#define TXDEV "/dev/ttyS10"
#define RXDEV "/dev/ttyS11"
// Baudrate settings are defined in <asm/termbits.h>, which is
// included by <termios.h>
#define BAUDRATE B9600         // For struct termios
#define DEFAULT_BAUDRATE 9600  // For the delaying transmissions
#define _POSIX_SOURCE 1        // POSIX compliant source
#define FALSE 0
#define TRUE 1

#define BUF_SIZE 2048

// Current running parameters
struct Parameters {
    int cableOn;
    double byteER;   // Byte error rate
    struct timespec byteDelay;
    unsigned long propDelay;   // Desired propagation delay in usec
    int bufSize;  // Dimensioned to enforce the propagation delay
    char *tx2rx;
    char *tx2rxValid;  // TRUE if corresponding entry holds a byte
    long tx2rxIdx;     // Input index for the tx2rx buffer
    char *rx2tx;
    char *rx2txValid;  // TRUE if corresponding entry holds a byte
    long rx2txIdx;     // Input index for the tx2rx buffer
    FILE *logfile;
};

struct Parameters par = {
    .cableOn = TRUE,
    .byteER = 0.0,
    .propDelay = 0,
    .tx2rx = NULL,
    .tx2rxValid = NULL,
    .rx2tx = NULL,
    .rx2txValid = NULL,
    .logfile = NULL};

// Returns: serial port file descriptor (fd).
int openSerialPort(const char *serialPort, struct termios *oldtio, struct termios *newtio)
{
    int fd = open(serialPort, O_RDWR | O_NONBLOCK | O_NOCTTY);

    if (fd < 0)
        return -1;

    // Save current port settings
    if (tcgetattr(fd, oldtio) == -1)
        return -1;

    memset(newtio, 0, sizeof(*newtio));
    newtio->c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio->c_iflag = IGNPAR;
    newtio->c_oflag = 0;
    newtio->c_lflag = 0;
    newtio->c_cc[VTIME] = 0; // Inter-character timer unused (polling mode)
    newtio->c_cc[VMIN] = 0;  // Read without blocking
    tcflush(fd, TCIOFLUSH);

    if (tcsetattr(fd, TCSANOW, newtio) == -1)
        return -1;

    return fd;
}


// Add noise to a buffer, by flipping the byte in the "errorIndex" position.
void addNoiseToBuffer(unsigned char *buf, size_t errorIndex)
{
    buf[errorIndex] ^= 0xFF;
}


// Initialize the ring buffers that implement the propagation delay
// Returns 0 on success, -1 on failure
int init_ring_buffers(void)
{
    long nsecPropDelay = 1000 * par.propDelay;
    long bytesInFlight = nsecPropDelay / par.byteDelay.tv_nsec;
    // Round instead of truncating
    if (nsecPropDelay % par.byteDelay.tv_nsec > par.byteDelay.tv_nsec / 2)
    {
        ++bytesInFlight;
    }
    long actualPropDelay = bytesInFlight * par.byteDelay.tv_nsec / 1000; // usec
    par.bufSize = bytesInFlight + 1;
    par.tx2rx = realloc(par.tx2rx, par.bufSize);
    par.tx2rxValid = realloc(par.tx2rxValid, par.bufSize);
    par.rx2tx = realloc(par.rx2tx, par.bufSize);
    par.rx2txValid = realloc(par.rx2txValid, par.bufSize);
    if (par.tx2rx == NULL || par.tx2rxValid == NULL || par.rx2tx == NULL || par.rx2txValid == NULL)
    {
        return -1;
    }
    bzero(par.tx2rxValid, par.bufSize);
    bzero(par.rx2txValid, par.bufSize);
    par.tx2rxIdx = 0;
    par.rx2txIdx = 0;
    printf("PROPAGATION DELAY SET TO %ld usec (DESIRED = %lu usec)\n", actualPropDelay, par.propDelay);
    return 0;
}


// Set the byte delay corresponding to the selected baud rate
void set_baud_rate(unsigned long baud)
{
    // 10 bit times per byte; delay in nanoseconds
    double delay = 1.0e10 / baud;
    par.byteDelay.tv_sec = 0;
    par.byteDelay.tv_nsec = (long) delay;
    printf("BAUD RATE: %lu\n", baud);
    init_ring_buffers();
}


// Make the program use RT priority to improve precision in timing
void set_rt_priority(void) {
    struct sched_param sp = { .sched_priority = 50 };
    if (sched_setscheduler(0, SCHED_FIFO, &sp) == -1) {
      perror("Could not set realtime priority");
    }
}


// Compute the difference between two timespecs
struct timespec timespec_diff(const struct timespec *t2, const struct timespec *t1)
{
    struct timespec diff = { .tv_sec = t2->tv_sec - t1->tv_sec,
                             .tv_nsec = t2->tv_nsec - t1->tv_nsec };
    if (diff.tv_nsec < 0) {
        diff.tv_nsec += 1000000000;
        --diff.tv_sec;
    }
    return diff;
}


// Compute the sum of two timespecs
struct timespec timespec_sum(const struct timespec *t1, const struct timespec *t2)
{
    struct timespec sum = { .tv_sec = t1->tv_sec + t2->tv_sec,
                             .tv_nsec = t1->tv_nsec + t2->tv_nsec };
    if (sum.tv_nsec >= 1000000000) {
        sum.tv_nsec -= 1000000000;
        ++sum.tv_sec;
    }
    return sum;
}


// Compare two timespecs returning -1, 0 or 1 if t1 is less than, equal or
// greater than t2, respectively
int timespec_comp(const struct timespec *t1, const struct timespec *t2)
{
    if (t1->tv_sec < t2->tv_sec) {
        return -1;
    }
    else if (t1->tv_sec > t2->tv_sec) {
        return 1;
    }
    else if (t1->tv_nsec < t2->tv_nsec) {
        return -1;
    }
    else if (t1->tv_nsec > t2->tv_nsec) {
        return 1;
    }
    return 0;
}


int timespec_is_negative(const struct timespec *t)
{
    if (t->tv_sec < 0 || t->tv_nsec < 0)
    {
        return TRUE;
    }
    return FALSE;
}


void endlog(void)
{
    if (par.logfile != NULL)
    {
        fclose(par.logfile);
        par.logfile = NULL;
    }
}


void startlog(const char *filename)
{
    endlog();
    par.logfile = fopen(filename, "w");
    if (par.logfile != NULL)
    {
        fprintf(par.logfile, "Tx->Rx | Rx->Tx\n");
        printf("LOGGING TO FILE %s\n", filename);
    }
    else
    {
        printf("ERROR OPENING FILE %s, NOT LOGGING\n", filename);
    }
}


// Show help
void help()
{
    printf("\n\n"
           "Transmitter must open " TXDEV "\n"
           "Receiver must open " RXDEV "\n"
           "\n"
           "The cable program is sensible to the following interactive commands:\n"
           "--- help         : show this help\n"
           "--- on           : connect the cable and data is exchanged (default state)\n"
           "--- off          : disconnect the cable disabling data to be exchanged\n"
           "--- ber <ber>    : add noise to data bits at a specified BER (default=0)\n"
           "--- baud <rate>  : set baud rate, between 1200 and 115200 (default=9600)\n"
           "                   note that 10 bits are sent per byte (8-N-1)\n"
           "--- prop <delay> : set the propagation delay in usec (0-1000000, default=0)\n"
           "                   will be approximated to an integer multiple of the byte\n"
           "                   delay (10 / baud_rate)\n"
           "--- log <file>   : log transmitted data to file\n"
           "--- endlog       : stop logging transmitted data\n"
           "--- quit         : terminate the program\n"
           "\n"
           "IMPORTANT: Changing the baud rate or propagation delay while a transmission is\n"
           "           ongoing will result in losses.\n"
           "\n");
}

int main(int argc, char *argv[])
{
    printf("\n");

    system("socat -dd PTY,link=" TXDEV ",mode=777,raw,echo=0 PTY,link=/dev/emulatorTx,mode=777,raw,echo=0 &");
    sleep(1);
    printf("\n");

    system("socat -dd PTY,link=" RXDEV ",mode=777,raw,echo=0 PTY,link=/dev/emulatorRx,mode=777,raw,echo=0 &");
    sleep(1);

    help();

    // Configure serial ports
    struct termios oldtioTx;
    struct termios newtioTx;

    int fdTx = openSerialPort("/dev/emulatorTx", &oldtioTx, &newtioTx);

    if (fdTx < 0)
    {
        perror("Opening Tx emulator serial port");
        exit(-1);
    }

    struct termios oldtioRx;
    struct termios newtioRx;

    int fdRx = openSerialPort("/dev/emulatorRx", &oldtioRx, &newtioRx);

    if (fdRx < 0)
    {
        perror("Opening Rx emulator serial port");
        exit(-1);
    }

    // Configure stdin to receive commands to this program
    int oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    char rxStdin[BUF_SIZE] = {0};

    int STOP = FALSE;

    set_baud_rate(DEFAULT_BAUDRATE);

    set_rt_priority();

    // For logging
    char tx2rxTx[3], tx2rxRx[3], rx2txTx[3], rx2txRx[3];
    int cableIdle = FALSE;

    printf("\nCable ready\n\n");

    // To compensate for deviations in byte transmission time
    struct timespec currentTime, nextTxTime, timeDiff, nextWait;
    int skipWait = FALSE;
    int unreliableRate = FALSE;
    clock_gettime(CLOCK_MONOTONIC, &nextTxTime);

    while (STOP == FALSE)
    {
        // Check how much waiting time we should have (if any)
        clock_gettime(CLOCK_MONOTONIC, &currentTime);
        timeDiff = timespec_diff(&currentTime, &nextTxTime);
        nextTxTime = timespec_sum(&nextTxTime, &par.byteDelay);
        if (timeDiff.tv_sec >= 1)
        {
            if (unreliableRate == FALSE)
            {
                printf("UNRELIABLE RATE: Could not keep up, timeDiff exceeded 1s\n"
                       "No further warnings will be issued\n");
                unreliableRate = TRUE;
            }
        }
        nextWait = timespec_diff(&nextTxTime, &currentTime);
        if (timespec_is_negative(&nextWait))
        {
            skipWait = TRUE;
        }
        else
        {
            skipWait = FALSE;
        }

        // Read from Tx
        int bytesFromTx = read(fdTx, par.tx2rx + par.tx2rxIdx, 1);
        par.tx2rxValid[par.tx2rxIdx] = bytesFromTx > 0;

        // Read from Rx
        int bytesFromRx = read(fdRx, par.rx2tx + par.rx2txIdx, 1);
        par.rx2txValid[par.rx2txIdx] = bytesFromRx > 0;

        if (!par.cableOn)
        {
            // Ignore what was read
            par.tx2rxValid[par.tx2rxIdx] = 0;
            par.rx2txValid[par.rx2txIdx] = 0;
        }

        if (par.logfile != NULL)  // Currently logging
        {
            if (par.tx2rxValid[par.tx2rxIdx])
            {
                sprintf(tx2rxTx, "%02hhX", par.tx2rx[par.tx2rxIdx]);
            }
            else
            {
                memcpy(tx2rxTx, "  ", 3);
            }
            if (par.rx2txValid[par.rx2txIdx])
            {
                sprintf(rx2txTx, "%02hhX", par.rx2tx[par.rx2txIdx]);
            }
            else
            {
                memcpy(rx2txTx, "  ", 3);
            }
        }

        // Advance indices to next position
        par.tx2rxIdx = (par.tx2rxIdx + 1) % par.bufSize;
        par.rx2txIdx = (par.rx2txIdx + 1) % par.bufSize;

        if (par.cableOn)
        {
            if (par.tx2rxValid[par.tx2rxIdx])
            {
                // Add error, if applicable
                if (par.byteER != 0.0 && (double) rand() / (double) RAND_MAX < par.byteER)
                {
                    // At most one wrong bit per byte, good enough if ber < 0.02
                    par.tx2rx[par.tx2rxIdx] ^= (char) 1 << rand() % 8;
                }
                write(fdRx, par.tx2rx + par.tx2rxIdx, 1);
            }

            if (par.rx2txValid[par.rx2txIdx])
            {
                // Add error, if applicable
                if (par.byteER != 0.0 && (double) rand() / (double) RAND_MAX < par.byteER)
                {
                    // At most one wrong bit per byte, good enough if ber < 0.02
                    par.rx2tx[par.rx2txIdx] ^= (char) 1 << rand() % 8;
                }
                write(fdTx, par.rx2tx + par.rx2txIdx, 1);
            }
        }

        if (par.logfile != NULL)  // Currently logging
        {
            if (par.tx2rxValid[par.tx2rxIdx])
            {
                sprintf(tx2rxRx, "%02hhX", par.tx2rx[par.tx2rxIdx]);
            }
            else
            {
                memcpy(tx2rxRx, "  ", 3);
            }
            if (par.rx2txValid[par.rx2txIdx])
            {
                sprintf(rx2txRx, "%02hhX", par.rx2tx[par.rx2txIdx]);
            }
            else
            {
                memcpy(rx2txRx, "  ", 3);
            }

            if (*tx2rxTx == ' ' && *rx2txTx == ' ' && *tx2rxRx == ' ' && *rx2txRx == ' ')
            {
                if (cableIdle == FALSE)
                {
                    fputs("---------------\n", par.logfile);
                    cableIdle = TRUE;
                }
            }
            else
            {
                fprintf(par.logfile, "%s  %s | %s  %s\n", tx2rxTx, tx2rxRx, rx2txTx, rx2txRx);
                cableIdle = FALSE;
            }
        }

        // Read commands from STDIN to control the cable mode
        int fromStdin = read(STDIN_FILENO, rxStdin, BUF_SIZE);
        if (fromStdin > 0)
        {
            rxStdin[fromStdin - 1] = '\0';

            if (strcmp(rxStdin, "off") == 0)
            {
                printf("CONNECTION OFF\n");
                if (par.cableOn && par.logfile != NULL)
                {
                    fputs("CABLE OFF\n", par.logfile);
                }
                par.cableOn = FALSE;
            }
            else if (strcmp(rxStdin, "on") == 0)
            {
                printf("CONNECTION ON\n");
                par.cableOn = TRUE;
            }
            else if (strncmp(rxStdin, "ber ", 4) == 0)
            {
                double ber;
                sscanf(rxStdin + 4, "%lf", &ber);
                // Compute pow(1 - ber, 8) without libm
                double acc = 1 - ber;
                acc *= acc;   // Squared
                acc *= acc;   // To the fourth
                acc *= acc;   // To the eightth
                par.byteER = 1.0 - acc;
                //printf("Byte Error Rate is %lf\n", par.byteER);
                if (ber >= 0.0 && ber < 1.0)
                {
                    printf("BER SET TO %lf\n", ber);
                    if (ber > 0.01)
                    {
                        printf("   ACTUAL BER WILL BE LOWER THAN DEFINED FOR VALUES ABOVE 0.01\n");
                    }
                }
                else
                {
                    printf("BAD BER VALUE %lf (MUST BE 0 <= BER < 1.0)", ber);
                }
            }
            else if (strncmp(rxStdin, "baud ", 5) == 0)
            {
                unsigned long baud = 0;
                sscanf(rxStdin + 5, "%lu", &baud);
                switch (baud) {
                    case 1200:
                    case 1800:
                    case 2400:
                    case 4800:
                    case 9600:
                    case 19200:
                    case 38400:
                    case 57600:
                    case 115200:
                        set_baud_rate(baud);
                        break;
                    default:
                        printf("UNSUPPORTED BAUD RATE: must be one of 1200, 1800, 2400, 4800, 9600, 19200, 38400, 57600 or 115200\n");
                }
            }
            else if (strncmp(rxStdin, "prop ", 5) == 0)
            {
                unsigned long propDelay;
                if (sscanf(rxStdin + 5, "%lu", &propDelay) < 1 || propDelay > 1000000)
                {
                    printf("BAD OR OUT OF RANGE PROPAGATION DELAY\n");
                }
                else
                {
                    par.propDelay = propDelay;
                    init_ring_buffers();
                }
            }
            else if (strncmp(rxStdin, "log ", 4) == 0)
            {
                startlog(rxStdin + 4);
            }
            else if (strcmp(rxStdin, "endlog") == 0)
            {
                endlog();
                printf("NOT LOGGING\n");
            }
            else if (strcmp(rxStdin, "quit") == 0)
            {
                printf("END OF THE PROGRAM\n");
                STOP = TRUE;
            }
            else if (strcmp(rxStdin, "help") == 0) {
                help();
            }
            else {
                printf("BAD COMMAND OR MISSING PARAMETERS\n");
            }
        }

        if (skipWait == FALSE) {
            nanosleep(&nextWait, NULL);
        }
    }

    // Restore the old port settings
    if (tcsetattr(fdRx, TCSANOW, &oldtioRx) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    if (tcsetattr(fdTx, TCSANOW, &oldtioTx) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    close(fdTx);
    close(fdRx);

    system("killall socat");

    return 0;
}
