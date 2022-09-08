/*
 * uart_state_machine.c
 *
 *  Created on: Sep 7, 2022
 *      Author: Jiarui.Ng
 */

#include "uart_state_machine.h"
#include "move.h"

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
		forward(MotorSpeed2);
		return StateMachineFullAck;
		break;

	case StateMachineBackward:
		backward(MotorSpeed2);
		return StateMachineFullAck;
		break;

	case StateMachineWheelsLeft:
		servo_point_left_full();
		return StateMachineFullAck;
		break;

	case StateMachineWheelsRight:
		servo_point_right_full();
		return StateMachineFullAck;
		break;

	case StateMachineReset:
		servo_point_center();
		motor_stop();
		return StateMachineFullAck;
		break;

	default:
		return StateMachineUnknown;
		break;
	}
}
