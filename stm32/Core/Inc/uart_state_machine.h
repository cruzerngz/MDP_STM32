/*
 * uart_state_machine.h
 *
 *  Created on: Sep 7, 2022
 *      Author: Jiarui.Ng
 *
 * This module takes in and interprets UART data to control the car.
 */

#include "stdint.h"
#include "stdio.h"

#include "globals.h"

#ifndef INC_UART_STATE_MACHINE_H_
#define INC_UART_STATE_MACHINE_H_

// Used by main to exec any predefined movement
//void (*HARDCODE_DIRECTION)(void);
//HARDCODE_DIRECTION = NULL;


// Exported to main.c for use as interupt flags
// Distance to move forward in centimeters
extern volatile uint16_t FLAG_MOVEMENT_DISTANCE;

// move f/b when turning, Forward is +1, Backward is -1
extern volatile int8_t FLAG_MOVE_DIR;

// approach obstacle routine
extern volatile int8_t FLAG_APPROACH_OBSTACLE;

// Angle to turn in degrees
extern volatile uint16_t FLAG_TURN_ANGLE;
// Direction (front/back, left/right) the car takes, -1, 0, +1
extern volatile int8_t FLAG_TURN_DIR;

// Cardinal directions the car can do for in-place turn
// 0-indexed, max 15, steps of 22.5 degrees clockwise
extern volatile uint8_t FLAG_IN_PLACE_CARDINAL;

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

// UART response return type (for main to interpret)
typedef struct {
	uint8_t message[20];
	uint16_t len;
} UartResponseMessage;

uint8_t state_machine_interpreter(uint8_t command);//uint8_t *uart_msg, uint16_t msg_size);
uint8_t state_machine_interpret_simple(uint8_t *uart_msg, uint16_t msg_size);

#endif /* INC_UART_STATE_MACHINE_H_ */
