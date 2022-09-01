/*
 * move.c
 *
 *  Created on: Sep 1, 2022
 *      Author: jimmysqqr
 */

#include "move.h"


// private macros
#define MOVE_DELAY_TICKS 100
//TODO!
// need a way to read instructions to stop
// RPi command or boolean isStop() ??
#define TEST_MOVE_TIMER 800
#define MOTOR_ADJUST_SPEED 300 // TODO: to be tested


// private function prototypes
void _adjust_forward(MotorSpeed speed);
void _adjust_backward(MotorSpeed speed);


/**
 * (Calibration) Adjust forward after moving backward
 */
void _adjust_forward(MotorSpeed speed) {
//	steer(dir, mag);
//	HAL_Delay(MOVE_DELAY_TICKS);
	forward(speed);
}


/**
 * (Calibration) Adjust backward after moving forward
 */
void _adjust_backward(MotorSpeed speed) {
//	steer(dir, mag);
//	HAL_Delay(MOVE_DELAY_TICKS);
	backward(speed);
}


/**
 * Sequence of moves to visually check that the motors and servo are functioning together
 */
void move_test_startup() {

	// isolated moves
	forward(MotorSpeed1);
	stop();
	_adjust_backward(MOTOR_ADJUST_SPEED);
	stop();
	HAL_Delay(100);

	backward(MotorSpeed1);
	stop();
	_adjust_forward(MOTOR_ADJUST_SPEED);
	stop();
	HAL_Delay(100);

	steer(ServoDirLeft, ServoMag1);
	HAL_Delay(100);

	steer(ServoDirRight, ServoMag1);
	HAL_Delay(100);

	steer_straight();
	HAL_Delay(300);

	// combined moves
	forward_left(MotorSpeed1, ServoDirLeft, ServoMag1);
	HAL_Delay(300);

	backward_left(MotorSpeed1, ServoDirLeft, ServoMag1);
	HAL_Delay(300);

	forward_right(MotorSpeed1, ServoDirRight, ServoMag1);
	HAL_Delay(300);

	backward_right(MotorSpeed1, ServoDirRight, ServoMag1);
	HAL_Delay(300);
}

/**
 * Go through all motor & servo combined movements
 */
void move_showcase() {
	//TODO!
}


/** Isolated moveset functions */

/**
 * (Motor only) Move forward at a specified speed
 */
void forward(MotorSpeed speed) { // assume needs calibration
	motor_forward(speed);
}


/**
 * (Motor only) Move backward at a specified speed
 */
void backward(MotorSpeed speed) { // assume needs calibration
	motor_backward(speed);
}


/**
 * (Motor only) Stop moving
 */
void stop() {
	motor_stop();
}

/**
 * (Servo only) Turn the wheels in a certain direction and magnitude
 */
void steer(ServoDirection dir, ServoMagnitude mag) {
	servo_point(dir, mag);
}


/**
 * (Servo only) Turn the wheels straight
 */
void steer_straight() {
	servo_point_center();
}


/** Combined moveset functions */

/**
 * (Motor & servo) Turn the wheels to an angled left and move forward
 */
void forward_left(MotorSpeed speed, ServoDirection dir, ServoMagnitude mag) {
	steer(dir, mag);
	HAL_Delay(MOVE_DELAY_TICKS);
	forward(speed);
}


/**
 * (Motor & servo) Turn the wheels to an angled right and move forward
 */
void forward_right(MotorSpeed speed, ServoDirection dir, ServoMagnitude mag) {
	steer(dir, mag);
	HAL_Delay(MOVE_DELAY_TICKS);
	forward(speed);
}


/**
 * (Motor & servo) Turn the wheels to an angled left and move backward
 */
void backward_left(MotorSpeed speed, ServoDirection dir, ServoMagnitude mag) {
	steer(dir, mag);
	HAL_Delay(MOVE_DELAY_TICKS);
	backward(speed);
}


/**
 * (Motor & servo) Turn the wheels to an angled right and move backward
 */
void backward_right(MotorSpeed speed, ServoDirection dir, ServoMagnitude mag) {
	steer(dir, mag);
	HAL_Delay(MOVE_DELAY_TICKS);
	backward(speed);
}

