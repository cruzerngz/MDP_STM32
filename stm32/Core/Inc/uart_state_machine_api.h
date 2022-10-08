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
    StateMachineSpeed = 's', // high speed mode
    StateMachineConfig = 'c',
	StateMachineFineControl = 'f',
	StateMachineToyCar = 't',
	StateMachineModeSelect = '\\' // this is a single backslash
} StateMachineDriveMode;

/**
 * High speed mode states
 */
typedef enum {
    SpeedIdle,
    SpeedLaneChange,
    SpeedBigLaneChange,
    SpeedUTurn
} SpeedStates;

/**
 * High speed mode commands + subcommands
 */
typedef enum {
    SpeedLane = 'l',
    SpeedBigLane = 'b',
    SpeedUTurning = 'u',

    SpeedSwitchLeft = 'l',
    SpeedSwitchRight = 'r',
} SpeedCommands;

/**
 * Config mode states + substates
 */
typedef enum {
	ConfigIdle,
	ConfigPID,
	ConfigDefaultSpeed,
    ConfigLowGripFlag,

	ConfigPIDKp,
	ConfigPIDKi,
	ConfigPIDKd,
    ConfigDefaultSpeedStraight,
    ConfigDefaultSpeedTurn,
} ConfigStates;

/**
 * @brief Config mode commands + subcommands
 *
 */
typedef enum {
    ConfigReset = 'q',
    ConfigK = 'k',
    ConfigSpeed = 's',
    ConfigLowGrip = 'g',

    ConfigKp = 'p',
    ConfigKi = 'i',
    ConfigKd = 'd',
    ConfigSpeedStraight = 's',
    ConfigSpeedTurn = 't',
    ConfigLowGripTrue = 't',
    ConfigLowGripFalse = 'f',
} ConfigCommands;

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
