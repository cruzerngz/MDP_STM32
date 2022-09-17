/*
 * move.c
 *
 *  Created on: Sep 1, 2022
 *      Author: jimmysqqr
 */

#include "stm32f4xx_hal.h"

#include "move.h"
#include "servo.h"
#include "motor.h"


// private macros
#define MOVE_DELAY_TICKS 100
//TODO!
// need a way to read instructions to stop
// RPi command or boolean isStop() ??
#define TEST_MOVE_TIMER 800
#define MOTOR_ADJUST_SPEED 300 // TODO: to be tested


// private globals

/*
 * Global struct containing the current effector states
 */
static struct global_state {
	uint32_t servo;
	uint32_t motor_left;
	uint32_t motor_right;
} GLOBAL_EFFECTOR_STATE = {
		.servo = SERVO_CENTER, // move SERVO_CENTER out of private namespace and use it here
		.motor_left = 0,
		.motor_right = 0
};


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

#define SERVO_FULL_LOCK_DELAY 350
#define DELAY_45 2650
#define DELAY_90 2650
#define DELAY_135 4500
#define DELAY_180 6000

/*
 * Start dynamic movement functions
 * Primary means of movement for car
 */


// Move the car forward by a specified distance
void move_forward_by(uint32_t centimeters) {
	servo_point_center();
	HAL_Delay(100);
	motor_forward(MotorSpeed2);
	HAL_Delay(60 * centimeters);

	motor_stop();
	servo_point_center();
}

// Move the car back by a specified distance
void move_backward_by(uint32_t centimeters) {
	servo_point_center();
	HAL_Delay(100);
	motor_backward(MotorSpeed2);
	HAL_Delay(60 * centimeters);

	motor_stop();
	servo_point_center();
}

// Move the car forward AND turn it to a specified angle
void move_turn_forward_by(MoveDirection direction, uint8_t degrees) {
	servo_point(direction, ServoMag4);
	HAL_Delay(SERVO_FULL_LOCK_DELAY);
	motor_forward(MotorSpeed2);
	HAL_Delay(44 * degrees);
	motor_stop();
	servo_point_center();
}

// Move the car forward AND turn it to a specified angle
void move_turn_backward_by(MoveDirection direction, uint8_t degrees) {

}

/*
 * Start of hardcoded movement functions
 * Note that car movement will vary depending on surface conditions.
 */


void move_hard_left_45() {
	servo_point_left_full();
	HAL_Delay(SERVO_FULL_LOCK_DELAY);
	motor_forward(MotorSpeed1);
	HAL_Delay(DELAY_45);
	motor_stop();
	servo_point_center();
}

void move_hard_right_45() {
	servo_point_right_full();
	HAL_Delay(SERVO_FULL_LOCK_DELAY);
	motor_forward(MotorSpeed2);
	HAL_Delay(DELAY_45);
	motor_stop();
	servo_point_center();
}

// Move and turn 90 degrees to the left
void move_hard_left_90() {
	servo_point_left_full();
	HAL_Delay(SERVO_FULL_LOCK_DELAY);
	motor_forward(MotorSpeed2);
	HAL_Delay(DELAY_90);
	motor_stop();
	servo_point_center();
}
void move_hard_right_90() {
	servo_point_right_full();
	HAL_Delay(SERVO_FULL_LOCK_DELAY);
	motor_forward(MotorSpeed2);
	HAL_Delay(DELAY_90);
	motor_stop();
	servo_point_center();
}
void move_hard_left_135() {
	servo_point_left_full();
	HAL_Delay(SERVO_FULL_LOCK_DELAY);
	motor_forward(MotorSpeed2);
	HAL_Delay(DELAY_135);
	motor_stop();
	servo_point_center();
}
void move_hard_right_135() {
	servo_point_right_full();
	HAL_Delay(SERVO_FULL_LOCK_DELAY);
	motor_forward(MotorSpeed2);
	HAL_Delay(DELAY_135);
	motor_stop();
	servo_point_center();
}
void move_hard_left_180() {
	servo_point_left_full();
	HAL_Delay(SERVO_FULL_LOCK_DELAY);
	motor_forward(MotorSpeed2);
	HAL_Delay(DELAY_180);
	motor_stop();
	servo_point_center();
}
void move_hard_right_180() {
	servo_point_right_full();
	HAL_Delay(SERVO_FULL_LOCK_DELAY);
	motor_forward(MotorSpeed2);
	HAL_Delay(DELAY_180);
	motor_stop();
	servo_point_center();
}
