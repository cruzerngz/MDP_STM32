/*
 * uart_state_machine.c
 *
 *  Created on: Sep 7, 2022
 *      Author: Jiarui.Ng
 */

#include "uart_state_machine.h"
#include "uart_state_machine_api.h"
#include "move.h"

// Returns the number of elements in the array
#define ARRAY_LEN(_array_) (sizeof(_array_) / sizeof(_array_[0]))

// Offset to convert an ascii char (as uint8_t) to an int val
#define ASCII_OFFSET 48


// private variables

// current global state for state machine

// I prefer to define static global vars here, so all can be
// seen and accounted for.
// Rather than define them inside functions

// default drive mode
static volatile StateMachineDriveMode GLOBAL_DRIVE_MODE = StateMachineModeSelect;

// default starting modes for toy car
static volatile ToyCarStates GLOBAL_TOY_CAR_MODE = ToyCarDrive;
static volatile ToyCarMoveCommands GLOBAL_TOY_CAR_MODE_MODIFIER;

static MotorSpeed VALID_SPEEDS[] = {MotorSpeed1, MotorSpeed2, MotorSpeed3};
static ServoMagnitude VALID_DIRS[] = {ServoMag1, ServoMag2, ServoMag3, ServoMag4};

static volatile MotorSpeed GLOBAL_TOY_CAR_SPEED = MotorSpeed1;
static volatile ServoMagnitude GLOBAL_TOY_CAR_DIR = ServoMag1;

// sub-states and their respective interpreters

uint8_t _state_machine_mode_interpreter(uint8_t command);

uint8_t _fine_control_interpreter(uint8_t command);


uint8_t _toy_car_interpreter(uint8_t command);
uint8_t _toy_car_interpreter_drive(uint8_t command);
uint8_t _toy_car_interpreter_setting(uint8_t command);
uint8_t _toy_car_interpreter_modifier(uint8_t command);


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
		return StateMachineUnknown;
		break;

	case StateMachineToyCar:
		return _toy_car_interpreter(command);
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
