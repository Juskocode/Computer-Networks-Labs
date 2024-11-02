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

// NOTE: only for report stats
#define FAKE_BCC1_ERR   0.0             // %
#define FAKE_BCC2_ERR   0.0             // %
#define TPROP           0               // ms
#define FILE_SIZE       10968
