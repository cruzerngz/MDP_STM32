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
	StateMachineInputField = '_',
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
 * Fine control mode states
 */
typedef enum {
	FineControlIdle,
	FineControlMove,
	FineControlTurn,
	FineControlInPlaceTurn
} FineControlStates;

/**
 * @brief Fine control mode commands + subcommands
 *
 */
typedef enum {
	FineControlReset = 'q',
	FineControlMovement = 'm',
	FineControlTurning = 't',
	FineControlInPlaceTurning = 'c',
	FineControlApproachObstacle = 'o',

	FineControlMoveForward = 'f',
	FineControlMoveBackward = 'b',
	FineControlTurnLeft = 'l',
	FineControlTurnRight = 'r'
} FineControlCommands;


/**
 * Toy car states, commands and modifiers
 */
typedef enum {
	ToyCarDrive,
	ToyCarSetting,
	ToyCarCustom
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

	ToyCarCustomMove = 'c'
} ToyCarMoveCommands;


#endif /* INC_UART_STATE_MACHINE_API_H_ */
