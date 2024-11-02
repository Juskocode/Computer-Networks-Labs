#include "state_machine.h"

/* State machine function to process each byte */
enum CommState processByte(enum CommState currentState, unsigned char byte, 
                           unsigned char expectedAddress, unsigned char expectedControl) {
    switch (currentState) {
    case STATE_START:
        if (byte == FRAME_FLAG) return STATE_FLAG_RECEIVED;
        break;
    case STATE_FLAG_RECEIVED:
        if (byte == FRAME_FLAG) return STATE_FLAG_RECEIVED;
        if (byte == expectedAddress) return STATE_ADDRESS_RECEIVED;
        return STATE_START;
    case STATE_ADDRESS_RECEIVED:
        if (byte == expectedControl) return STATE_CONTROL_RECEIVED;
        if (byte == FRAME_FLAG) return STATE_FLAG_RECEIVED;
        return STATE_START;
    case STATE_CONTROL_RECEIVED:
        if (byte == (expectedControl ^ expectedAddress)) return STATE_BCC_OK;
        if (byte == FRAME_FLAG) return STATE_FLAG_RECEIVED;
        return STATE_START;
    case STATE_BCC_OK:
        if (byte == FRAME_FLAG) return STATE_STOP;
        return STATE_START;
    default:
        return STATE_START;
    }
    return currentState;
}