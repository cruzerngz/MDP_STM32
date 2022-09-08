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

/**
 * Global commands and responses
 */

// state machine valid responses
typedef enum {
	StateMachinePartialAck = '.',
	StateMachineFullAck = ';',
	StateMachineUnknown = '?',
} StateMachineResponse;

// state machine valid drive mode (and its commands)
typedef enum {
	StateMachineFineControl = 'f',
	StateMachineToyCar = 't',
	StateMachineModeSelect = '\\' // this is a single backslash
} StateMachineDriveMode;

/**
 * Fine control mode commands and modifiers
 */
// TODO!()


/**
 * Toy car states, commands and modifiers
 */

typedef enum {
	ToyCarDrive,
	ToyCarSetting,
	ToyCarMagnitude
} ToyCarStates;

// valid commands to toy car
typedef enum {
	ToyCarMoveForward = 'w',
	ToyCarMoveBackward = 's',
	ToyCarMoveStop = 'z',
	ToyCarMoveWheelsLeft = 'a',
	ToyCarMoveWheelsRight = 'd',
	ToyCarMoveWheelsCenter = 'x',

	ToyCarModifyMotorSpeed = 'm',
	ToyCarModifyServoMag = 'n',
} ToyCarMoveCommands;


#endif /* INC_UART_STATE_MACHINE_API_H_ */
