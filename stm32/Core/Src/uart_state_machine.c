/*
 * uart_state_machine.c
 *
 *  Created on: Sep 7, 2022
 *      Author: Jiarui.Ng
 */

#include "stdbool.h"

#include "uart_state_machine.h"
#include "uart_state_machine_api.h"

// disable when unit testing
#ifndef UNITTEST
#include "stm32f4xx_hal.h" // temp include, for debugging
#include "move.h"
#endif

// Returns the number of elements in the array
#define ARRAY_LEN(_array_) (sizeof(_array_) / sizeof(_array_[0]))

// Offset to convert an ascii char (as uint8_t) to an int val
#define ASCII_OFFSET 48


/* EXTERN GLOBALS */

// flags polled by main to check for any command to execute
// reset by main task, no need to be reset here
volatile uint16_t 						FLAG_MOVEMENT_DISTANCE = 0;
volatile uint16_t 						FLAG_TURN_ANGLE = 0;

// additional flags used by move/turn, requires reset
volatile int8_t 						FLAG_MOVE_DIR = 0; // forward/backward, +1 or -1 only
volatile int8_t 						FLAG_TURN_DIR = 0; // right/left, +1 or -1 only


/* STATIC GLOBALS */
// Some globals are not static for extern unit tests to access them

// default drive mode
static volatile StateMachineDriveMode 	GLOBAL_DRIVE_MODE = StateMachineModeSelect;

// default starting mode for fine control
static volatile FineControlStates 		GLOBAL_FINE_CONTROL_MODE = FineControlIdle;
// static
volatile int16_t 						GLOBAL_FINE_CONTROL_MAGNITUDE = 0; // param that sets the flags after interrupt
// +1 for right, -1 for left
volatile int8_t  						GLOBAL_FINE_CONTROL_TURN_DIR = 0;
// +1 for forward, -1 for backward
volatile int8_t 						GLOBAL_FINE_CONTROL_MOVE_DIR = 0;
static volatile bool 					GLOBAL_FINE_CONTROL_ACCEPT_TURN_MOVE = false;
static volatile bool 					GLOBAL_FINE_CONTROL_INGEST = false;

// default starting modes for toy car
static volatile ToyCarStates 			GLOBAL_TOY_CAR_MODE = ToyCarDrive;
static volatile ToyCarMoveCommands 		GLOBAL_TOY_CAR_MODE_MODIFIER;

#ifndef UNITTEST
static MotorSpeed 						VALID_SPEEDS[] = {MotorSpeed1, MotorSpeed2, MotorSpeed3};
static ServoMagnitude 					VALID_DIRS[] = {ServoMag1, ServoMag2, ServoMag3, ServoMag4, ServoMag5};

static volatile MotorSpeed 				GLOBAL_TOY_CAR_SPEED = MotorSpeed1;
static volatile ServoMagnitude 			GLOBAL_TOY_CAR_DIR = ServoMag1;
#endif

// sub-states and their respective interpreters

uint8_t _state_machine_mode_interpreter(uint8_t command);

uint8_t _fine_control_interpreter(uint8_t command);
uint8_t _fine_control_interpreter_idle(uint8_t command);
uint8_t _fine_control_interpreter_move(uint8_t command);
uint8_t _fine_control_interpreter_turn(uint8_t command);
uint8_t _fine_control_interpreter_turn_dir(uint8_t command);
uint8_t _fine_control_interpreter_ingest_magnitude(uint8_t command);

void _fine_control_reset_intern_flags(void);
void _fine_control_reset_extern_flags(void);
uint8_t _fine_control_set_flags(void);


uint8_t _toy_car_interpreter(uint8_t command);
uint8_t _toy_car_interpreter_drive(uint8_t command);
uint8_t _toy_car_interpreter_setting(uint8_t command);
uint8_t _toy_car_interpreter_custom(uint8_t command);
//uint8_t _toy_car_interpreter_modifier(uint8_t command);


/**
 * Main entry point to the state machine.
 * Ingests a single char from UART
 * and modifies its internal state if command is valid
 * State change -> changes in car behavior
 */
uint8_t state_machine_interpreter(uint8_t command) {
	if(command == StateMachineModeSelect) {
		GLOBAL_DRIVE_MODE = StateMachineModeSelect;
		return StateMachineFullAck;
	}

	switch(GLOBAL_DRIVE_MODE) {
	case StateMachineFineControl:
		return _fine_control_interpreter(command);
		break;

	case StateMachineToyCar:
		#ifndef UNITTEST
			return _toy_car_interpreter(command);
		#else
			return StateMachineUnknown;
		#endif
		break;

	case StateMachineModeSelect:
		return _state_machine_mode_interpreter(command);
		break;

	default:
		GLOBAL_DRIVE_MODE = StateMachineToyCar;
		return StateMachineUnknown;
	}

	return StateMachineUnknown;
}

uint8_t _state_machine_mode_interpreter(uint8_t command) {
	switch(command) {
	case StateMachineFineControl:
		GLOBAL_DRIVE_MODE = StateMachineFineControl;
		return StateMachineFullAck;
		break;

	case StateMachineToyCar:
		GLOBAL_DRIVE_MODE = StateMachineToyCar;
		return StateMachineFullAck;
		break;

	case StateMachineModeSelect:
		GLOBAL_DRIVE_MODE = StateMachineModeSelect;
		return StateMachineFullAck;
		break;

	default:
		return StateMachineUnknown;
		break;
	}

	return StateMachineUnknown;
}

/**
 * Main entry point to the fine control state machine
 */
uint8_t _fine_control_interpreter(uint8_t command) {
	switch(GLOBAL_FINE_CONTROL_MODE) {
	case FineControlIdle:
		return _fine_control_interpreter_idle(command);
		break;

	case FineControlMove:
		return _fine_control_interpreter_move(command);
		break;

	case FineControlTurn:
		return _fine_control_interpreter_turn(command);
		break;

	default:
		GLOBAL_FINE_CONTROL_MODE = FineControlIdle;
		return StateMachineUnknown;
	}
}

uint8_t _fine_control_interpreter_idle(uint8_t command) {
	switch(command) {
	case FineControlReset:
		GLOBAL_FINE_CONTROL_MODE = FineControlIdle;
		GLOBAL_FINE_CONTROL_INGEST = false;
		GLOBAL_FINE_CONTROL_MAGNITUDE = 0; 		// reset
		GLOBAL_FINE_CONTROL_TURN_DIR = 0; // reset
		return StateMachineFullAck;
		break;

	case FineControlMovement:
		GLOBAL_FINE_CONTROL_MODE = FineControlMove;
		// GLOBAL_FINE_CONTROL_INGEST = true;
		// GLOBAL_FINE_CONTROL_MAGNITUDE = 0; // reset the value
		return StateMachinePartialAck;
		break;

	case FineControlTurning:
		GLOBAL_FINE_CONTROL_MODE = FineControlTurn;
		// GLOBAL_FINE_CONTROL_INGEST = true;
		// GLOBAL_FINE_CONTROL_MAGNITUDE = 0; // reset the value
		return StateMachinePartialAck;
		break;

	default:
		GLOBAL_FINE_CONTROL_MODE = FineControlIdle;
		return StateMachineUnknown;
		break;
	}
}
uint8_t _fine_control_interpreter_move(uint8_t command) {
	if(GLOBAL_FINE_CONTROL_INGEST) { // set move dir, continue
		return _fine_control_interpreter_ingest_magnitude(command);

	} else { // move dir not set, to be set here
		if(command == FineControlMoveForward) GLOBAL_FINE_CONTROL_MOVE_DIR = 1;
		else if(command == FineControlMoveBackward) GLOBAL_FINE_CONTROL_MOVE_DIR = -1;
		else {
			GLOBAL_FINE_CONTROL_MODE = FineControlIdle;
			GLOBAL_FINE_CONTROL_INGEST = false;
			return StateMachineUnknown;
		}

		GLOBAL_FINE_CONTROL_INGEST = true;
		GLOBAL_FINE_CONTROL_MAGNITUDE = 0; // reset the value
		return StateMachineInputField;
	}
}

/**
 * @brief Handles setting of flags for turning
 *
 *
 * @param command
 * @return uint8_t
 */
uint8_t _fine_control_interpreter_turn(uint8_t command) {
	if(GLOBAL_FINE_CONTROL_INGEST) { // if ingesting numbers
		return _fine_control_interpreter_ingest_magnitude(command);
	}
	else if(GLOBAL_FINE_CONTROL_ACCEPT_TURN_MOVE) { // if taking in move dirrection
		return _fine_control_interpreter_turn_dir(command);
	}
	else {
		if(command == FineControlTurnRight) {
			GLOBAL_FINE_CONTROL_TURN_DIR = 1;
			GLOBAL_FINE_CONTROL_ACCEPT_TURN_MOVE = true;
			return StateMachinePartialAck;
		}
		else if(command == FineControlTurnLeft) {
			GLOBAL_FINE_CONTROL_TURN_DIR = -1;
			GLOBAL_FINE_CONTROL_ACCEPT_TURN_MOVE = true;
			return StateMachinePartialAck;
		}
		else {
			_fine_control_reset_intern_flags();
			// GLOBAL_FINE_CONTROL_MODE = FineControlIdle;
			// GLOBAL_FINE_CONTROL_INGEST = false;
			// GLOBAL_FINE_CONTROL_ACCEPT_TURN_MOVE = true;
			return StateMachineUnknown;
		}
	}
}

/**
 * @brief Set the movement dir when turning (forward/backward)
 *
 * @param command
 * @return uint8_t
 */
uint8_t _fine_control_interpreter_turn_dir(uint8_t command) {
	switch(command) {
		case FineControlMoveForward:
			GLOBAL_FINE_CONTROL_MOVE_DIR = 1;
			GLOBAL_FINE_CONTROL_INGEST = true;
			return StateMachineInputField;
			break;

		case FineControlMoveBackward:
			GLOBAL_FINE_CONTROL_MOVE_DIR = -1;
			GLOBAL_FINE_CONTROL_INGEST = true;
			return StateMachineInputField;
			break;

		default:
			_fine_control_reset_intern_flags();
			return StateMachineUnknown;
			break;
	}
}

uint8_t _fine_control_interpreter_ingest_magnitude(uint8_t command) {
	uint8_t command_as_int = command - ASCII_OFFSET;
	uint8_t reply;

	if(command == StateMachineFullAck) {
		reply = _fine_control_set_flags();
		_fine_control_reset_intern_flags();

		return reply;
// 		if(GLOBAL_FINE_CONTROL_MODE == FineControlMove) {
// 			FLAG_MOVEMENT_DISTANCE = GLOBAL_FINE_CONTROL_MAGNITUDE;
// 			FLAG_TURN_DIR = GLOBAL_FINE_CONTROL_MAGNITUDE_SIGN;
// 			GLOBAL_FINE_CONTROL_INGEST = false;
// 			GLOBAL_FINE_CONTROL_MODE = FineControlIdle;
// //			HAL_UART_Transmit(&huart3, (uint8_t *)"set\r\n", 10, HAL_MAX_DELAY);
// 			return StateMachineFullAck;

// 		} else if (GLOBAL_FINE_CONTROL_MODE == FineControlTurn) {
// 			FLAG_TURN_ANGLE = GLOBAL_FINE_CONTROL_MAGNITUDE;
// 			FLAG_TURN_DIR = GLOBAL_FINE_CONTROL_MAGNITUDE_SIGN;
// 			GLOBAL_FINE_CONTROL_INGEST = false;
// 			GLOBAL_FINE_CONTROL_MODE = FineControlIdle;
// //			HAL_UART_Transmit(&huart3, (uint8_t *)"set\r\n", 10, HAL_MAX_DELAY);
// 			return StateMachineFullAck;

// 		} else {
// 			GLOBAL_FINE_CONTROL_INGEST = false;
// 			GLOBAL_FINE_CONTROL_MODE = FineControlIdle;
// 			return StateMachineUnknown;
// 		}
	}

	else if(command_as_int >= 0 && command_as_int <= 9 ) {
		GLOBAL_FINE_CONTROL_MAGNITUDE = (GLOBAL_FINE_CONTROL_MAGNITUDE * 10) + command_as_int;
		return StateMachineInputField;
		// continue with ingest

	} else { // fallback to idle state
		_fine_control_reset_intern_flags();
		// GLOBAL_FINE_CONTROL_INGEST = false;
		// GLOBAL_FINE_CONTROL_MODE = FineControlIdle;
		return StateMachineUnknown;
	}

	return StateMachineUnknown;
}

/**
 * @brief Sets the various flags in fine control mode
 * Passes the values from statically scoped variables to extern variables
 *
 */
uint8_t _fine_control_set_flags(void) {
	switch(GLOBAL_FINE_CONTROL_MODE) {
		case FineControlMove:
			FLAG_MOVEMENT_DISTANCE = GLOBAL_FINE_CONTROL_MAGNITUDE;
			FLAG_MOVE_DIR = GLOBAL_FINE_CONTROL_MOVE_DIR;
			return StateMachineFullAck;
			break;

		case FineControlTurn:
			FLAG_TURN_ANGLE = GLOBAL_FINE_CONTROL_MAGNITUDE;
			FLAG_TURN_DIR = GLOBAL_FINE_CONTROL_TURN_DIR;
			FLAG_MOVE_DIR = GLOBAL_FINE_CONTROL_MOVE_DIR;
			return StateMachineFullAck;
			break;

		default: // do nothing
			return StateMachineUnknown;
			break;

	}
}

/**
 * @brief Resets all flags internal to defaults
 * Resets the fine control mode to idle
 *
 */
void _fine_control_reset_intern_flags(void) {
	GLOBAL_FINE_CONTROL_MAGNITUDE = 0;
	GLOBAL_FINE_CONTROL_TURN_DIR = 0;
	GLOBAL_FINE_CONTROL_MOVE_DIR = 0;
	GLOBAL_FINE_CONTROL_ACCEPT_TURN_MOVE = false;
	GLOBAL_FINE_CONTROL_INGEST = false;

	GLOBAL_FINE_CONTROL_MODE = FineControlIdle;
}

/**
 * @brief Resets all extern flags (FLAG_*) to defaults
 *
 */
void _fine_control_reset_extern_flags(void) {
	FLAG_MOVE_DIR = 0;
	FLAG_TURN_DIR = 0;
	FLAG_MOVEMENT_DISTANCE = 0;
	FLAG_TURN_ANGLE = 0;
}

#ifndef UNITTEST // disable this block if unit testing

/**
 * Main entry point to the toy car state machine
 */
uint8_t _toy_car_interpreter(uint8_t command) {
	switch(GLOBAL_TOY_CAR_MODE) {
	case ToyCarDrive:
		return _toy_car_interpreter_drive(command);
		break;

	case ToyCarSetting:
		return _toy_car_interpreter_setting(command);
		break;

	case ToyCarCustom:
		return _toy_car_interpreter_custom(command);
		break;

	default:
		GLOBAL_TOY_CAR_MODE = ToyCarDrive;
		return StateMachineUnknown;
		break;
	}

	return StateMachineUnknown;
}

/**
 * Handles the toy car movement
 */
uint8_t _toy_car_interpreter_drive(uint8_t command) {
	switch(command) {
	case ToyCarMoveForward:
		forward(GLOBAL_TOY_CAR_SPEED);
		return StateMachineFullAck;
		break;

	case ToyCarMoveBackward:
		backward(GLOBAL_TOY_CAR_SPEED);
		return StateMachineFullAck;
		break;

	case ToyCarMoveStop:
		stop();
		return StateMachineFullAck;
		break;

	case ToyCarMoveWheelsLeft:
		servo_point(ServoDirLeft, GLOBAL_TOY_CAR_DIR);
		return StateMachineFullAck;
		break;

	case ToyCarMoveWheelsRight:
		servo_point(ServoDirRight, GLOBAL_TOY_CAR_DIR);
		return StateMachineFullAck;
		break;

	case ToyCarMoveWheelsCenter:
		steer_straight();
		return StateMachineFullAck;
		break;

	case ToyCarModifyMotorSpeed:
		GLOBAL_TOY_CAR_MODE = ToyCarSetting;
		GLOBAL_TOY_CAR_MODE_MODIFIER = ToyCarModifyMotorSpeed;
		return StateMachinePartialAck;
		break;

	case ToyCarModifyServoMag:
		GLOBAL_TOY_CAR_MODE = ToyCarSetting;
		GLOBAL_TOY_CAR_MODE_MODIFIER = ToyCarModifyServoMag;
		return StateMachinePartialAck;
		break;

	case ToyCarCustomMove:
		GLOBAL_TOY_CAR_MODE = ToyCarCustom;
		return StateMachinePartialAck;
		break;

	default:
		return StateMachineUnknown;
		break;
	}

	return StateMachineUnknown;
}

uint8_t _toy_car_interpreter_setting(uint8_t command) {
	uint8_t command_as_int = command - ASCII_OFFSET;

	switch(GLOBAL_TOY_CAR_MODE_MODIFIER) {
	case ToyCarModifyMotorSpeed:
		if(command_as_int >= 0 && command_as_int < ARRAY_LEN(VALID_SPEEDS)) {
			GLOBAL_TOY_CAR_SPEED = VALID_SPEEDS[command_as_int];
			GLOBAL_TOY_CAR_MODE = ToyCarDrive;
			return StateMachineFullAck;
		} else {
			GLOBAL_TOY_CAR_MODE = ToyCarDrive;
			return StateMachineUnknown;
		}
		break;

	case ToyCarModifyServoMag:
		if(command_as_int >= 0 && command_as_int < ARRAY_LEN(VALID_DIRS)) {
			GLOBAL_TOY_CAR_DIR = VALID_DIRS[command_as_int];
			GLOBAL_TOY_CAR_MODE = ToyCarDrive;
			return StateMachineFullAck;
		} else {
			GLOBAL_TOY_CAR_MODE = ToyCarDrive;
			return StateMachineUnknown;
		}
		break;

	default:
		GLOBAL_TOY_CAR_MODE = ToyCarDrive;
		return StateMachineUnknown;
		break;
	}

	return StateMachineUnknown;
}

uint8_t _toy_car_interpreter_custom(uint8_t command) {
	uint8_t command_as_int = command - ASCII_OFFSET;


//
//	if(command_as_int >= 0 && command_as_int < ARRAY_LEN(CUSTOM_DIRECTIONS)) {
////		(CUSTOM_MOVEMENTS[command_as_int])();
//		HARDCODE_DIRECTION = (void *)CUSTOM_DIRECTIONS[command_as_int];
//		GLOBAL_TOY_CAR_MODE = ToyCarDrive;
//		return StateMachineFullAck;
//	} else {
//		GLOBAL_TOY_CAR_MODE = ToyCarDrive;
////		move_hard_left_45();
//		return StateMachineUnknown;
//	}
}

/*
 * Entry point to state machine
 * Ingests a message from UART
 * Returns a response message to be sent back
 * Actuates any motors/etc if necessary
 * This function is used for testing
 */
uint8_t state_machine_interpret_simple(uint8_t *uart_msg, uint16_t msg_size) {

	if (msg_size == 0) return 0;

	switch (uart_msg[0]) {
	case ToyCarMoveForward:
		forward(MotorSpeed2);
		return StateMachineFullAck;
		break;

	case ToyCarMoveBackward:
		backward(MotorSpeed2);
		return StateMachineFullAck;
		break;

	case ToyCarMoveStop:
		motor_stop();
		return StateMachineFullAck;

	case ToyCarMoveWheelsLeft:
		servo_point_left_full();
		return StateMachineFullAck;
		break;

	case ToyCarMoveWheelsRight:
		servo_point_right_full();
		return StateMachineFullAck;
		break;

	case ToyCarMoveWheelsCenter:
		servo_point_center();
		return StateMachineFullAck;
		break;

	default:
		return StateMachineUnknown;
		break;
	}
}
#endif // UNITTEST