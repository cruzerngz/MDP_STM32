/*
 * uart_state_machine.h
 *
 *  Created on: Sep 7, 2022
 *      Author: Jiarui.Ng
 *
 * This module takes in and interprets UART data to control the car.
 */

#ifndef INC_UART_STATE_MACHINE_H_
#define INC_UART_STATE_MACHINE_H_

// Mode, set first
typedef enum {

} StateMachineMode;

typedef struct {

} StateMachineState;

// Testing out the state machine and movement
typedef enum {
	StateMachineTestForward,
	StateMachineTestBackward,
	StateMachineTestStop,
	StateMachineTestRight90,
	StateMachineTestLeft90,
	StateMachineTestRight180,
	StateMachineTestLeft180

} StateMachineTestMoves;

uint8_t *state_machine_interpret(uint8_t *uart_msg, uint16_t msg_size);

#endif /* INC_UART_STATE_MACHINE_H_ */
