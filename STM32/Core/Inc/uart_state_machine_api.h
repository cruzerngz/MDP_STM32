/*
 * uart_state_machine_api.h
 *
 * This header file is meant to be included together with
 *
 *  Created on: Sep 7, 2022
 *      Author: Jiarui.Ng
 */

#ifndef INC_UART_STATE_MACHINE_API_H_
#define INC_UART_STATE_MACHINE_API_H_

// state machine valid responses
typedef enum {
	StateMachinePartialAck = '.',
	StateMachineFullAck = ';',
	StateMachineUnknown = '?',
} StateMachineResponse;

// state machine valid drive modes
typedef enum {
	StateMachineFineControl = 'f',
	StateMachineToyCar = 't',
	StateMachineSlave = 's'
} StateMachineDriveMode;

typedef enum {
	StateMachineForward = 'w',
	StateMachineBackward = 'x',
	StateMachineWheelsLeft = 'a',
	StateMachineWheelsRight = 'd',
	StateMachineReset = 's'
} StateMachineDriveSlave;


#endif /* INC_UART_STATE_MACHINE_API_H_ */
