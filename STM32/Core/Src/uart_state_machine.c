/*
 * uart_state_machine.c
 *
 *  Created on: Sep 7, 2022
 *      Author: Jiarui.Ng
 */

#include "uart_state_machine.h"

/*
 * Ingests a message from UART
 * Returns a response message to be sent back
 * Actuates any motors/etc if necessary
 * This function is used for testing
 */
uint8_t state_machine_interpret_simple(uint8_t *uart_msg, uint16_t msg_size) {

	if (msg_size == 0) return 0;

	switch (uart_msg[0]) {
	case StateMachineForward:
		return StateMachineFullAck;
		break;

	case StateMachineBackward:
		return StateMachineFullAck;
		break;

	case StateMachineWheelsLeft:
		return StateMachineFullAck;
		break;

	case StateMachineWheelsRight:
		return StateMachineFullAck;
		break;

	case StateMachineWheelsCenter:
		return StateMachineFullAck;
		break;

	default:
		return StateMachineUnknown;
		break;
	}
}
