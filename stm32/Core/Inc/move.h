/*
 * move.h
 *
 *  Created on: Sep 1, 2022
 *      Author: jimmysqqr
 */
#include "motor.h"
#include "servo.h"

#ifndef INC_MOVE_H_
#define INC_MOVE_H_

// Move direction inherited from servo.h
// Exposed here as another enum
typedef enum {
	MoveDirLeft = ServoDirLeft,
	MoveDirRight = ServoDirRight
} MoveDirection;

// init and showcase functions
void move_test_startup();
void move_showcase();

// isolated moveset functions
void forward(MotorSpeed speed); // assume needs calibration
void backward(MotorSpeed speed); // assume needs calibration
void steer(ServoDirection dir, ServoMagnitude mag);
void steer_straight();
void stop();

// combined moveset functions
void forward_left(MotorSpeed speed, ServoDirection dir, ServoMagnitude mag);
void forward_right(MotorSpeed speed, ServoDirection dir, ServoMagnitude mag);

void backward_left(MotorSpeed speed, ServoDirection dir, ServoMagnitude mag);
void backward_right(MotorSpeed speed, ServoDirection dir, ServoMagnitude mag);

// dynamic movement functions, require the use of the encoder
void move_forward_calc(uint32_t centimeters);
void move_backward_calc(uint32_t centimeters);

void move_forward_cm(uint32_t centimeters);
void move_backward_cm(uint32_t centimeters);

void move_turn_forward_by(MoveDirection direction, uint16_t degrees);
void move_turn_backward_by(MoveDirection direction, uint16_t degrees);

void move_in_place_turn_by(MoveDirection direction, uint16_t degrees);

void move_in_place_turn_cardinal(uint8_t cardinal_direction);



// Hardcoded, known movements (requires further calibration)

void move_hard_left_45();
void move_hard_left_90();
void move_hard_left_135();
void move_hard_left_180();

void move_hard_right_45();
void move_hard_right_90();
void move_hard_right_135();
void move_hard_right_180();

// calibration functions for forward() and backward()
// maybe make these private functions?
//void adjust_forward(MotorSpeed speed, ServoDirection dir, ServoMagnitude mag);
//void adjust_backward(MotorSpeed speed, ServoDirection dir, ServoMagnitude mag);


#endif /* INC_MOVE_H_ */
