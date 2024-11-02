#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <stdio.h>  // For printf and perror
#include <signal.h> // For signal handling

/**
 * @file state_machine.h
 * @brief Header file defining the state machine for packet reception.
 *
 * This file provides an enumeration for the communication states involved in
 * packet reception and a function prototype for processing each byte according
 * to the state machine's logic.
 *
 * The state machine is used to receive and validate packets by progressing through
 * several states (e.g., recognizing flags, addresses, control bytes, and checksum).
 */

/* Constants */

/**
 * @brief Flag byte used to indicate the start and end of a packet frame.
 */
#define FRAME_FLAG 0x7E

/* Enumeration Types */

/**
 * @enum CommState
 * @brief States representing the progress of packet reception in the state machine.
 *
 * These states are used to track the current position within the packet reception process:
 * - STATE_START: Initial state, awaiting the start flag.
 * - STATE_FLAG_RECEIVED: Start flag received; expecting the address byte.
 * - STATE_ADDRESS_RECEIVED: Address byte matched; expecting the control byte.
 * - STATE_CONTROL_RECEIVED: Control byte matched; expecting the checksum (BCC).
 * - STATE_BCC_OK: Checksum (BCC) is correct; expecting the end flag.
 * - STATE_STOP: End flag received, packet is successfully received.
 */
enum CommState {
    STATE_START,            /**< Initial state, waiting for the start flag. */
    STATE_FLAG_RECEIVED,    /**< Start flag received, waiting for address byte. */
    STATE_ADDRESS_RECEIVED, /**< Address byte received, waiting for control byte. */
    STATE_CONTROL_RECEIVED, /**< Control byte received, waiting for BCC. */
    STATE_BCC_OK,           /**< BCC is correct, waiting for end flag. */
    STATE_DATA,             /**< DATA byte are being received*/
    STATE_STOP              /**< End flag received, packet is successfully processed. */
};

/* Function Prototypes */

/**
 * @brief Processes a single byte within the state machine for packet reception.
 *
 * This function takes the current state, the incoming byte, and the expected address
 * and control bytes. It evaluates the byte according to the state machine logic and
 * updates the current state accordingly.
 *
 * @param currentState The current state of the packet reception process.
 * @param byte The incoming byte to process.
 * @param expectedAddress The expected address byte for the packet.
 * @param expectedControl The expected control byte for the packet.
 * @return The updated state after processing the byte.
 *
 * @note This function is used by packet receiving functions to handle
 *       byte-by-byte packet validation.
 */
enum CommState processByte(enum CommState currentState, unsigned char byte, 
                           unsigned char expectedAddress, unsigned char expectedControl);

#endif // STATE_MACHINE_H
