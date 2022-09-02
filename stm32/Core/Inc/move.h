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

// init and showcase functions
void move_test_startup();
void move_showcase();

// isolated moveset functions
void forward(MotorSpeed speed); // assume needs calibration
void backward(MotorSpeed speed); // assume needs calibration
void steer(ServoDirection dir, ServoMagnitude mag);
void steer_straight();
void stop();

void forward_by(uint16_t dist);
void backward_by(uint16_t dist);

// combined moveset functions
void forward_left(MotorSpeed speed, ServoDirection dir, ServoMagnitude mag);
void forward_right(MotorSpeed speed, ServoDirection dir, ServoMagnitude mag);

void backward_left(MotorSpeed speed, ServoDirection dir, ServoMagnitude mag);
void backward_right(MotorSpeed speed, ServoDirection dir, ServoMagnitude mag);

// calibration functions for forward() and backward()
// maybe make these private functions?
//void adjust_forward(MotorSpeed speed, ServoDirection dir, ServoMagnitude mag);
//void adjust_backward(MotorSpeed speed, ServoDirection dir, ServoMagnitude mag);


#endif /* INC_MOVE_H_ */
