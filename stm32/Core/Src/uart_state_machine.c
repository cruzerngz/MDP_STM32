/*
 * uart_state_machine.c
 *
 *  Created on: Sep 7, 2022
 *      Author: Jiarui.Ng
 */

#include "stdbool.h"

#include "uart_state_machine.h"
#include "uart_state_machine_api.h"
#include "globals.h"

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
volatile uint8_t 						FLAG_IN_PLACE_CARDINAL = 0;
volatile int8_t 						FLAG_APPROACH_OBSTACLE =  0;

// additional flags used by move/turn, requires reset
volatile int8_t 						FLAG_MOVE_DIR = 0; // forward/backward, +1 or -1 only
volatile int8_t 						FLAG_TURN_DIR = 0; // right/left, +1 or -1 only

// flags used by config, initialised to be the values defined in move.h
// used to store the defaults if a reset is needed
// volatile int16_t                        FLAG_KP = MOVE_KP;
// volatile int16_t                        FLAG_KI = MOVE_KI;
// volatile int16_t                        FLAG_KD = MOVE_KD;
// volatile int16_t                        FLAG_DEFAULT_SPEED_STRAIGHT = MOVE_DEFAULT_SPEED_STRAIGHT_MM_S;
// volatile int16_t                        FLAG_DEFAULT_SPEED_TURN = MOVE_DEFAULT_SPEED_TURN_MM_S;

// flags used for high speed
volatile bool                           FLAG_CHANGE_LANE = 0;
volatile bool                           FLAG_CHANGE_BIG_LANE = 0;
volatile bool                           FLAG_U_TURN = 0;
volatile int8_t                         FLAG_SWITCH_DIR = 0; // right/left, +1 or -1 only
volatile int16_t                        FLAG_LANE_DISTANCE = 0;


/* STATIC GLOBALS */
// Some globals are not static for extern unit tests to access them

// default drive mode
static volatile StateMachineDriveMode 	GLOBAL_DRIVE_MODE = StateMachineModeSelect;


// default starting mode for high speed
static volatile SpeedStates             GLOBAL_SPEED_MODE = SpeedIdle;
static volatile int16_t                 GLOBAL_SPEED_MAGNITUDE = 0;
static volatile int8_t                  GLOBAL_SPEED_SWITCH_DIR = 0;
static volatile bool                    GLOBAL_SPEED_INGEST = false;
// static volatile bool                   GLOBAL_SPEED_MODE_IS_RESET = 1; // 1 if input contains '\s', to distinguish between substate and command with same input
// static volatile bool                   GLOBAL_SPEED_MODE_IS_PREFIX_SET = 0; // whether \s was sent previously

// default starting mode for fine control
static volatile ConfigStates            GLOBAL_CONFIG_MODE = ConfigIdle;
static volatile ConfigStates            GLOBAL_CONFIG_SUB_MODE = ConfigIdle;

// static
volatile int16_t 						GLOBAL_CONFIG_MAGNITUDE = 0; // param that sets the flags after interrupt
static volatile bool 					GLOBAL_CONFIG_INGEST = false;

// set to 0 or a value from move.h ?
// volatile int16_t                        GLOBAL_CONFIG_KP = MOVE_KP;
// volatile int16_t                        GLOBAL_CONFIG_KI = MOVE_KI;
// volatile int16_t                        GLOBAL_CONFIG_KD = MOVE_KD;

// volatile int16_t                        GLOBAL_CONFIG_DEFAULT_SPEED_STRAIGHT = MOVE_DEFAULT_SPEED_STRAIGHT_MM_S;
// volatile int16_t                        GLOBAL_CONFIG_DEFAULT_SPEED_TURN = MOVE_DEFAULT_SPEED_TURN_MM_S;

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
static ServoMagnitude 					VALID_DIRS[] = {ServoMag1, ServoMag2, ServoMag3, ServoMag4, ServoMag5, ServoMag6};

static volatile MotorSpeed 				GLOBAL_TOY_CAR_SPEED = MotorSpeed1;
static volatile ServoMagnitude 			GLOBAL_TOY_CAR_DIR = ServoMag1;
#endif

// sub-states and their respective interpreters

uint8_t _state_machine_mode_interpreter(uint8_t command);

uint8_t _high_speed_interpreter(uint8_t command);
uint8_t _high_speed_interpreter_idle(uint8_t command);
uint8_t _high_speed_interpreter_lane_change(uint8_t command);
uint8_t _high_speed_interpreter_boolean(uint8_t command);
uint8_t _high_speed_interpreter_ingest_magnitude(uint8_t command);

uint8_t _high_speed_lane_change_set_flags(void);
void _high_speed_reset_intern_flags(void);
void _high_speed_reset_extern_flags(void);

uint8_t _config_interpreter(uint8_t command);
uint8_t _config_interpreter_idle(uint8_t command);
uint8_t _config_interpreter_magnitude(uint8_t command);
uint8_t _config_interpreter_boolean(uint8_t command);
uint8_t _config_interpreter_ingest_magnitude(uint8_t command);

uint8_t _config_set_flags(void);
void _config_reset_intern_flags(void);
// void _config_reset_extern_flags(void);


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
    case StateMachineSpeed:
        return _high_speed_interpreter(command);
        break;

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

    case StateMachineConfig:
		return _config_interpreter(command);
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
    case StateMachineSpeed:
		GLOBAL_DRIVE_MODE = StateMachineSpeed;
        // GLOBAL_SPEED_MODE_IS_RESET = 1; // dont keep track of GLOBAL_SPEED_MODE state
		return StateMachineFullAck;
		break;

    case StateMachineConfig:
		GLOBAL_DRIVE_MODE = StateMachineConfig;
		return StateMachineFullAck;
		break;

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
 * Main entry point to the high speed state machine
 */
uint8_t _high_speed_interpreter(uint8_t command) {
    // if (GLOBAL_SPEED_MODE_IS_RESET) { // refresh GLOBAL_SPEED_MODE state
    //     GLOBAL_SPEED_MODE = SpeedIdle; 
    // }

    switch(GLOBAL_SPEED_MODE) {
	case SpeedIdle:
		return _high_speed_interpreter_idle(command);
		break;

    case SpeedLaneChange:
        return _high_speed_interpreter_lane_change(command);
        break;

    case SpeedBigLaneChange:
    case SpeedUTurn:
        return _high_speed_interpreter_boolean(command);
        break;

	default:
		GLOBAL_SPEED_MODE = SpeedIdle;
		return StateMachineUnknown;
	}
}

uint8_t _high_speed_interpreter_idle(uint8_t command) {
	switch(command) {
	case SpeedLane:
        // at this point, all input are valid and at end of command
        // GLOBAL_SPEED_MODE_IS_RESET = 0; // now keep track of GLOBAL_SPEED_MODE state
		GLOBAL_SPEED_MODE = SpeedLaneChange;
		return StateMachinePartialAck;
		break;

	case SpeedBigLane:
        // GLOBAL_SPEED_MODE_IS_RESET = 0; 
		GLOBAL_SPEED_MODE = SpeedBigLaneChange;
		return StateMachinePartialAck;
		break;

	case SpeedUTurning:
        // GLOBAL_SPEED_MODE_IS_RESET = 0; 
		GLOBAL_SPEED_MODE = SpeedUTurn;
		return StateMachinePartialAck;
		break;

	default:
		GLOBAL_SPEED_MODE = SpeedIdle;
		return StateMachineUnknown;
		break;
	}
}


uint8_t _high_speed_interpreter_lane_change(uint8_t command) {
    if (GLOBAL_SPEED_INGEST) {
        return _high_speed_interpreter_ingest_magnitude(command);

    } else {
        // l or r is sent in
        if (command == SpeedSwitchRight) GLOBAL_SPEED_SWITCH_DIR = 1;
        else if (command == SpeedSwitchLeft) GLOBAL_SPEED_SWITCH_DIR = -1;
        else {
            _high_speed_reset_intern_flags();
            return StateMachineUnknown;
        }

        GLOBAL_SPEED_INGEST = true;
        GLOBAL_SPEED_MAGNITUDE = 0;
        return StateMachineInputField;
    }
}

uint8_t _high_speed_interpreter_boolean(uint8_t command) {
    if (command != SpeedSwitchRight && command != SpeedSwitchLeft) {
        _high_speed_reset_intern_flags();
        return StateMachineUnknown;
    }

    switch(GLOBAL_SPEED_MODE) {
    // case SpeedLaneChange:
    //     FLAG_CHANGE_LANE = 1;
    //     FLAG_SWITCH_DIR = (command == SpeedSwitchRight) ? 1 : -1;
    //     _high_speed_reset_intern_flags();
    //     return StateMachineFullAck;
    //     break;

    case SpeedBigLaneChange:
        FLAG_CHANGE_BIG_LANE = 1;
        FLAG_SWITCH_DIR = (command == SpeedSwitchRight) ? 1 : -1;
        _high_speed_reset_intern_flags();
        return StateMachineFullAck;
        break;

    case SpeedUTurn:
        FLAG_U_TURN = 1;
        FLAG_SWITCH_DIR = (command == SpeedSwitchRight) ? 1 : -1;
        _high_speed_reset_intern_flags();
        return StateMachineFullAck;
        break;

    default: // fallback to idle state
        _high_speed_reset_intern_flags();
        return StateMachineUnknown;
        break;
	}
}


uint8_t _high_speed_interpreter_ingest_magnitude(uint8_t command) {
	uint8_t command_as_int = command - ASCII_OFFSET;
	uint8_t reply;

    // done giving value input
	if(command == StateMachineFullAck) {
		reply = _high_speed_lane_change_set_flags();
		_high_speed_reset_intern_flags(); 

		return reply;
	}

	else if(command_as_int >= 0 && command_as_int <= 9 ) {
		GLOBAL_SPEED_MAGNITUDE = (GLOBAL_SPEED_MAGNITUDE * 10) + command_as_int;
		return StateMachineInputField;
		// continue with ingest

	} else { // fallback to idle state
		_config_reset_intern_flags();
		return StateMachineUnknown;
	}

	return StateMachineUnknown;
}


uint8_t _high_speed_lane_change_set_flags(void) {
    FLAG_CHANGE_LANE = 1;
    FLAG_SWITCH_DIR = GLOBAL_SPEED_SWITCH_DIR;
    FLAG_LANE_DISTANCE = GLOBAL_SPEED_MAGNITUDE;
    return StateMachineFullAck;
}


/**
 * @brief Resets all flags internal to defaults
 * Resets the high speed mode to idle
 *
 */
void _high_speed_reset_intern_flags(void) {
	GLOBAL_SPEED_MODE = SpeedIdle;
    GLOBAL_SPEED_MAGNITUDE = 0;
    GLOBAL_SPEED_INGEST = false;
    GLOBAL_SPEED_SWITCH_DIR = 0;
}

/**
 * @brief Resets all extern flags (FLAG_*) to defaults
 *
 */
void _high_speed_reset_extern_flags(void) {
    FLAG_CHANGE_LANE = 0;
    FLAG_CHANGE_BIG_LANE = 0;
    FLAG_U_TURN = 0;
    FLAG_SWITCH_DIR = 0;
    FLAG_LANE_DISTANCE = 0;
}


/**
 * Main entry point to the config state machine
 */
uint8_t _config_interpreter(uint8_t command) {
	switch(GLOBAL_CONFIG_MODE) {
	case ConfigIdle:
		return _config_interpreter_idle(command);
		break;

	case ConfigPID: // go to substate
    case ConfigDefaultSpeed:
		return _config_interpreter_magnitude(command);
		break;

    case ConfigLowGripFlag:
		return _config_interpreter_boolean(command);
		break;

	default:
		GLOBAL_FINE_CONTROL_MODE = ConfigIdle;
		return StateMachineUnknown;
	}
}

uint8_t _config_interpreter_idle(uint8_t command) {
	switch(command) {
	case ConfigReset:
		GLOBAL_CONFIG_MODE = ConfigIdle;
        GLOBAL_CONFIG_SUB_MODE = ConfigIdle;
		GLOBAL_CONFIG_INGEST = false;
		GLOBAL_CONFIG_MAGNITUDE = 0; 		// reset
		return StateMachineFullAck;
		break;

	case ConfigK:
		GLOBAL_CONFIG_MODE = ConfigPID;
		return StateMachinePartialAck;
		break;

	case ConfigSpeed:
		GLOBAL_CONFIG_MODE = ConfigDefaultSpeed;
		return StateMachinePartialAck;
		break;

    case ConfigLowGrip:
		GLOBAL_CONFIG_MODE = ConfigLowGripFlag;
		return StateMachinePartialAck;
		break;

	default:
		GLOBAL_CONFIG_MODE = ConfigIdle;
		return StateMachineUnknown;
		break;
	}
}

uint8_t _config_interpreter_magnitude(uint8_t command) {
	if(GLOBAL_CONFIG_INGEST) {
		return _config_interpreter_ingest_magnitude(command);

	} else {
        // p, i, d, s, or t is sent in
        // need another global var to rmb if we are changing p, i, d, s, or t
        if (command == ConfigKp) GLOBAL_CONFIG_SUB_MODE = ConfigPIDKp;
        else if (command == ConfigKi) GLOBAL_CONFIG_SUB_MODE = ConfigPIDKi;
        else if (command == ConfigKd) GLOBAL_CONFIG_SUB_MODE = ConfigPIDKd;
        else if (command == ConfigSpeedStraight) GLOBAL_CONFIG_SUB_MODE = ConfigDefaultSpeedStraight;
        else if (command == ConfigSpeedTurn) GLOBAL_CONFIG_SUB_MODE = ConfigDefaultSpeedTurn;
        else {
            _config_reset_intern_flags();
            return StateMachineUnknown;
        }

        GLOBAL_CONFIG_INGEST = true;
        GLOBAL_CONFIG_MAGNITUDE = 0;
        return StateMachineInputField;
	}
}

uint8_t _config_interpreter_boolean(uint8_t command) {
    if (command == ConfigLowGripTrue) {
        _GLOBAL_MOVE_SURFACE_LOW_GRIP = true;
        return StateMachineFullAck;
    } 
    else if (command == ConfigLowGripFalse) {
        _GLOBAL_MOVE_SURFACE_LOW_GRIP = false;
        return StateMachineFullAck;
    } else { // fallback to idle state
		_config_reset_intern_flags();
		return StateMachineUnknown;
	}
}

uint8_t _config_interpreter_ingest_magnitude(uint8_t command) {
	uint8_t command_as_int = command - ASCII_OFFSET;
	uint8_t reply;

    // done giving value input
	if(command == StateMachineFullAck) {
		reply = _config_set_flags();
		_config_reset_intern_flags(); 

		return reply;
	}

	else if(command_as_int >= 0 && command_as_int <= 9 ) {
		GLOBAL_FINE_CONTROL_MAGNITUDE = (GLOBAL_FINE_CONTROL_MAGNITUDE * 10) + command_as_int;
		return StateMachineInputField;
		// continue with ingest

	} else { // fallback to idle state
		_config_reset_intern_flags();
		return StateMachineUnknown;
	}

	return StateMachineUnknown;
}

uint8_t _config_set_flags(void) {
	switch(GLOBAL_CONFIG_SUB_MODE) {
		case ConfigPIDKp:
            _GLOBAL_MOVE_KP = GLOBAL_CONFIG_MAGNITUDE;
			return StateMachineFullAck;
			break;

		case ConfigPIDKi:
            _GLOBAL_MOVE_KI = GLOBAL_CONFIG_MAGNITUDE;
			return StateMachineFullAck;
			break;

        case ConfigPIDKd:
            _GLOBAL_MOVE_KD = GLOBAL_CONFIG_MAGNITUDE;
			return StateMachineFullAck;
			break;

		case ConfigDefaultSpeedStraight:
            _GLOBAL_MOVE_DEFAULT_SPEED_STRAIGHT_MM_S = GLOBAL_CONFIG_MAGNITUDE;
			return StateMachineFullAck;
			break;

		case ConfigDefaultSpeedTurn:
            _GLOBAL_MOVE_DEFAULT_SPEED_TURN_MM_S = GLOBAL_CONFIG_MAGNITUDE;
			return StateMachineFullAck;
			break;

		default: // do nothing
			return StateMachineUnknown;
			break;
	}
}

/**
 * @brief Resets all flags internal to defaults
 * Resets the config mode and submode to idle
 *
 */
void _config_reset_intern_flags(void) {
	GLOBAL_CONFIG_MAGNITUDE = 0;
	GLOBAL_CONFIG_INGEST = false;

	GLOBAL_CONFIG_MODE = ConfigIdle;
	GLOBAL_CONFIG_SUB_MODE = ConfigIdle;
}

/**
 * @brief Resets all extern flags (FLAG_*) to defaults
 *
 */
// void _config_reset_extern_flags(void) {
// 	// FLAG_MOVE_DIR = 0;
// 	// FLAG_TURN_DIR = 0;
// 	// FLAG_MOVEMENT_DISTANCE = 0;
// 	// FLAG_TURN_ANGLE = 0;
// 	// FLAG_IN_PLACE_CARDINAL = 0;
// 	// FLAG_APPROACH_OBSTACLE = 0;
//     _GLOBAL_MOVE_KP = MOVE_KP;
//     _GLOBAL_MOVE_KI = MOVE_KI;
//     _GLOBAL_MOVE_KD = MOVE_KD;
//     _GLOBAL_MOVE_DEFAULT_SPEED_STRAIGHT_MM_S = MOVE_DEFAULT_SPEED_STRAIGHT_MM_S;
//     _GLOBAL_MOVE_DEFAULT_SPEED_TURN_MM_S = MOVE_DEFAULT_SPEED_TURN_MM_S;
// }



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

	case FineControlInPlaceTurn:
		return _fine_control_interpreter_ingest_magnitude(command);
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
		return StateMachinePartialAck;
		break;

	case FineControlTurning:
		GLOBAL_FINE_CONTROL_MODE = FineControlTurn;
		return StateMachinePartialAck;
		break;

	case FineControlInPlaceTurning:
		GLOBAL_FINE_CONTROL_MODE = FineControlInPlaceTurn;
		GLOBAL_FINE_CONTROL_INGEST = true;
		return StateMachineInputField;
		break;

	case FineControlApproachObstacle:
		FLAG_APPROACH_OBSTACLE = 1;
		return StateMachineFullAck;
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
	}

	else if(command_as_int >= 0 && command_as_int <= 9 ) {
		GLOBAL_FINE_CONTROL_MAGNITUDE = (GLOBAL_FINE_CONTROL_MAGNITUDE * 10) + command_as_int;
		return StateMachineInputField;
		// continue with ingest

	} else { // fallback to idle state
		_fine_control_reset_intern_flags();
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

		case FineControlInPlaceTurn:
			FLAG_IN_PLACE_CARDINAL = GLOBAL_FINE_CONTROL_MAGNITUDE;
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
	FLAG_IN_PLACE_CARDINAL = 0;
	FLAG_APPROACH_OBSTACLE = 0;
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
