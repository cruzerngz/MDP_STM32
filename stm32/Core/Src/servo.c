/*
 * servo.c
 *
 *  Created on: 27 Aug 2022
 *      Author: cruzerng
 */

#include "servo.h"

// private macros
// SERVO_CENTER moved into header file for move.c to use

// private static variables, initialized with servo_init();

static uint16_t SERVO_LEFT_MAGS[] = {
	8,
	16,
	24,
	32,
	40,
	SERVO_LEFT_LIMIT
};

static uint16_t SERVO_RIGHT_MAGS[] = {
	8,
	16,
	31,
	41,
	60,
	SERVO_RIGHT_LIMIT
};

// Pointer to the timer handler initialized in main
static TIM_HandleTypeDef *SERVO_PWM_TIMER;

// channel is (int) (0-indexed) * 4
// channel 1: 0,
// channel 2: 4, etc.
static uint32_t SERVO_CHANNEL;

// Address to the register responsible for controlling the servo's direction
// Must be declared as volatile or the compiler will optimize stuff and cause problems
// This is taken care of by prefixing "__IO" before type declaration
static __IO uint32_t *SERVO_PWM_REGISTER;

// Global containing the current direction set in servo
static ServoDirection SERVO_CURR_DIR;

// private function prototypes

/**
 * Bring the relavant servo control variables into scope.
 * Same params as HAL_TIM_PWM_Start()
 */
void servo_init(TIM_HandleTypeDef *htim, uint32_t Channel) {
	// assign private globals
	SERVO_PWM_TIMER = htim;
	SERVO_CHANNEL = Channel;

	// set the servo register once here
	switch (SERVO_CHANNEL) {
	case TIM_CHANNEL_1:
		SERVO_PWM_REGISTER = &(SERVO_PWM_TIMER->Instance->CCR1);
		break;
	case TIM_CHANNEL_2:
		SERVO_PWM_REGISTER = &(SERVO_PWM_TIMER->Instance->CCR2);
		break;
	case TIM_CHANNEL_3:
		SERVO_PWM_REGISTER = &(SERVO_PWM_TIMER->Instance->CCR3);
		break;
	case TIM_CHANNEL_4:
		SERVO_PWM_REGISTER = &(SERVO_PWM_TIMER->Instance->CCR4);
		break;
	default:
		SERVO_PWM_REGISTER = NULL;
		break;
	}

	HAL_TIM_PWM_Start(SERVO_PWM_TIMER, SERVO_CHANNEL);
	*SERVO_PWM_REGISTER = SERVO_CENTER;
	SERVO_CURR_DIR = ServoDirCenter;
}

/**
 * Sequence of moves to visually check that the servo is functioning
 */
void servo_test_startup() {
	servo_point_center();
	HAL_Delay(SERVO_MIN_DELAY_TICKS << 2);
	servo_point(ServoDirLeft, ServoMag1);
	HAL_Delay(SERVO_MIN_DELAY_TICKS);
	servo_point(ServoDirLeft, ServoMag2);
	HAL_Delay(SERVO_MIN_DELAY_TICKS);
	servo_point(ServoDirLeft, ServoMag3);
	HAL_Delay(SERVO_MIN_DELAY_TICKS);
	servo_point(ServoDirLeft, ServoMag4);
	HAL_Delay(SERVO_MIN_DELAY_TICKS);
	servo_point(ServoDirLeft, ServoMag3);
	HAL_Delay(SERVO_MIN_DELAY_TICKS);
	servo_point(ServoDirLeft, ServoMag2);
	HAL_Delay(SERVO_MIN_DELAY_TICKS);
	servo_point(ServoDirLeft, ServoMag1);
	HAL_Delay(SERVO_MIN_DELAY_TICKS);
	servo_point_center();
	HAL_Delay(SERVO_MIN_DELAY_TICKS);
	servo_point(ServoDirRight, ServoMag1);
	HAL_Delay(SERVO_MIN_DELAY_TICKS);
	servo_point(ServoDirRight, ServoMag2);
	HAL_Delay(SERVO_MIN_DELAY_TICKS);
	servo_point(ServoDirRight, ServoMag3);
	HAL_Delay(SERVO_MIN_DELAY_TICKS);
	servo_point(ServoDirRight, ServoMag4);
	HAL_Delay(SERVO_MIN_DELAY_TICKS);
	servo_point(ServoDirRight, ServoMag3);
	HAL_Delay(SERVO_MIN_DELAY_TICKS);
	servo_point(ServoDirRight, ServoMag2);
	HAL_Delay(SERVO_MIN_DELAY_TICKS);
	servo_point(ServoDirRight, ServoMag1);
	HAL_Delay(SERVO_MIN_DELAY_TICKS);
	servo_point_center();
	HAL_Delay(SERVO_MIN_DELAY_TICKS << 2);
	servo_point_left_full();
	HAL_Delay(SERVO_MIN_DELAY_TICKS << 3);
	servo_point_right_full();
	HAL_Delay(SERVO_MIN_DELAY_TICKS << 3);
	servo_point_center();
	HAL_Delay(SERVO_MIN_DELAY_TICKS << 3);
}

/**
 * Go through all servo movements
 */
void servo_showcase() {
	//TODO!
}

// directions

/**
 * Point the wheels at a certain direction with a specific magnitude
 */
void servo_point(ServoDirection dir, ServoMagnitude mag) {
//	int16_t offset = ((uint16_t) mag + 1) << 3;
//	switch (dir) {
//	case ServoDirLeft:
//		offset = 0 - offset;
//		SERVO_CURR_DIR = ServoDirLeft;
//		break;
//
//	case ServoDirRight:
//		SERVO_CURR_DIR = ServoDirRight;
//		break;
//
//	default:
//		SERVO_CURR_DIR = ServoDirCenter;
//		break;
//	}

	if(dir == ServoDirLeft) {
		*SERVO_PWM_REGISTER = SERVO_CENTER - SERVO_LEFT_MAGS[mag];
	}
	else if(dir == ServoDirRight) {
		*SERVO_PWM_REGISTER = SERVO_CENTER + SERVO_RIGHT_MAGS[mag]; // scaling factor
	}

//	*SERVO_PWM_REGISTER = SERVO_CENTER + offset;
}

/**
 * Point the wheels all the way to the left
 */
void servo_point_left_full(){
	*SERVO_PWM_REGISTER = SERVO_LEFT_MAX;
	SERVO_CURR_DIR = ServoDirLeft;
}
/**
 * Point the wheels all the way to the right
 */
void servo_point_right_full(){
	*SERVO_PWM_REGISTER = SERVO_RIGHT_MAX;
	SERVO_CURR_DIR = ServoDirRight;
}

/**
 * Center the wheels
 */
void servo_point_center() {
	switch(SERVO_CURR_DIR) {
	case ServoDirLeft:
		*SERVO_PWM_REGISTER = SERVO_CENTER + SERVO_CENTERING_OFFSET;
		break;
	case ServoDirRight:
		*SERVO_PWM_REGISTER = SERVO_CENTER - SERVO_CENTERING_OFFSET;
		break;
	case ServoDirCenter:
		*SERVO_PWM_REGISTER = SERVO_CENTER;
	}

	// // testing
	// HAL_Delay(25);
	// *SERVO_PWM_REGISTER = SERVO_CENTER + 5;
	// HAL_Delay(25);
	// *SERVO_PWM_REGISTER = SERVO_CENTER - 5;
	// HAL_Delay(25);
	// *SERVO_PWM_REGISTER = SERVO_CENTER;

	SERVO_CURR_DIR = ServoDirCenter;
//	*SERVO_PWM_REGISTER = SERVO_CENTER;
}

/**
 * Points the servo to a specified angle
 */
void servo_point_degrees(ServoDirection dir, uint8_t degrees) {

}

