/*
 * motor.c
 *
 *  Created on: 30 Aug 2022
 *      Author: jimmysqqr
 */

#include "motor.h"


// private macros

#define MOTOR_MIN_DELAY_TICKS 300

// need a way to read instructions to stop
// RPi command or boolean isStop() ??
#define TEST_MOTOR_TIMER 800


// private static variables, initialized with motor_init();

static TIM_HandleTypeDef *MOTOR_PWM_TIMER;
static uint32_t MOTOR_CHANNEL_1;
static uint32_t MOTOR_CHANNEL_2;


// private function prototypes

/**
 * Bring the relevant motor control variables into scope.
 * Same params as HAL_TIM_PWM_Start()
 */
void motor_init(TIM_HandleTypeDef *htim, uint32_t Channel1, uint32_t Channel2) {
	MOTOR_PWM_TIMER = htim;
	MOTOR_CHANNEL_1 = Channel1;
	MOTOR_CHANNEL_2 = Channel2;
	HAL_TIM_PWM_Start(MOTOR_PWM_TIMER, MOTOR_CHANNEL_1);
	HAL_TIM_PWM_Start(MOTOR_PWM_TIMER, MOTOR_CHANNEL_2);
}


/**
 * Go through all motor movements
 */
void motor_showcase() {
	//TODO!
}


/**
 * Sequence of moves to visually check that the motors are functioning
 */
void motor_test_startup() { // 500, 1000, 1500
	motor_stop();

	// forward with diff speed
	motor_forward(MotorSpeed1);
	motor_stop();
	HAL_DELAY(MOTOR_MIN_DELAY_TICKS);
	motor_forward(MotorSpeed2);
	motor_stop();
	HAL_DELAY(MOTOR_MIN_DELAY_TICKS);
	motor_forward(MotorSpeed3);
	motor_stop();
	HAL_DELAY(MOTOR_MIN_DELAY_TICKS);

	// backward with diff speed
	motor_backward(MotorSpeed1);
	motor_stop();
	HAL_DELAY(MOTOR_MIN_DELAY_TICKS);
	motor_backward(MotorSpeed2);
	motor_stop();
	HAL_DELAY(MOTOR_MIN_DELAY_TICKS);
	motor_backward(MotorSpeed3);
	motor_stop();
	HAL_DELAY(MOTOR_MIN_DELAY_TICKS);

	// forward with increasing speed
	motor_forward(MotorSpeed1);
	motor_forward(MotorSpeed2);
	motor_forward(MotorSpeed3);
	motor_stop();
	HAL_DELAY(MOTOR_MIN_DELAY_TICKS);

	// backward with increasing speed
	motor_backward(MotorSpeed1);
	motor_backward(MotorSpeed2);
	motor_backward(MotorSpeed3);
	motor_stop();
	HAL_DELAY(MOTOR_MIN_DELAY_TICKS);
}


/**
 * Move forward at a specified speed
 */
void motor_forward(MotorSpeed speed) {

//	uint16_t pwmVal = 100;
	uint16_t timer = TEST_MOTOR_TIMER;

	while (timer > 0) { // RPi command or boolean isStop() ??

		// left wheel clockwise
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);

		// right wheel anti-clockwise
		HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);

		// modify comparison value for the duty cycle
		__HAL_TIM_SetCompare(MOTOR_PWM_TIMER, MOTOR_CHANNEL_1, speed);
		__HAL_TIM_SetCompare(MOTOR_PWM_TIMER, MOTOR_CHANNEL_2, speed);

		timer--;
	}
//	motor_adjust_forward();
}


/**
 * Reverse at a specified speed
 */
void motor_reverse(MotorSpeed speed) {

	uint16_t timer = TEST_MOTOR_TIMER;

	while (1) { // RPi command or boolean isStop() ??

		// left wheel clockwise
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);

		// right wheel anti-clockwise
		HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);

		// modify comparison value for the duty cycle
		__HAL_TIM_SetCompare(MOTOR_PWM_TIMER, MOTOR_CHANNEL_1, speed);
		__HAL_TIM_SetCompare(MOTOR_PWM_TIMER, MOTOR_CHANNEL_2, speed);

		timer--;
	}
//	motor_adjust_backward();
}


/**
 * Stop the motors
 */
void motor_stop() {

	__HAL_TIM_SetCompare(MOTOR_PWM_TIMER, MOTOR_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(MOTOR_PWM_TIMER, MOTOR_CHANNEL_2, 0);

}

