// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

#define TRUE 1
#define FALSE 0

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
#define RR0                    0xAA
#define RR1                    0xAB
#define REJ0                   0x54
#define REJ1                   0x55

// Define color codes with backgrounds
#define COLOR_RESET "\033[0m"
#define COLOR_HEADER "\033[1;35m"         // Bright Magenta Text
#define COLOR_HEADER_BG "\033[1;35;45m"   // Magenta Text with Magenta Background
#define COLOR_NEGATIVE "\033[1;31m"       // Bright Red Text
#define COLOR_NEGATIVE_BG "\033[1;31;41m" // Red Text with Red Background
#define COLOR_POSITIVE "\033[1;32m"       // Bright Green Text
#define COLOR_POSITIVE_BG "\033[1;32;42m" // Green Text with Green Background
#define COLOR_STAT "\033[1;33m"           // Bright Yellow for stats
#define COLOR_TEXT "\033[0;37m"           // Light Gray for regular text
#define COLOR_BLACK "\033[1;30m"          // Bold Black Text

/*PrintStatists macros*/
#define FAKE_BCC1_ERR   0.0             // %
#define FAKE_BCC2_ERR   0.0             // %
#define TPROP           0               // ms
#define FILE_SIZE       10968
