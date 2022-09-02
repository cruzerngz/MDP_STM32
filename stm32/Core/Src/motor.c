/*
 * motor.c
 *
 *  Created on: 30 Aug 2022
 *      Author: jimmysqqr
 */

#include "motor.h"
#include "stm32f4xx_hal.h"

// private macros

#define MOTOR_DELAY_TICKS 1000

//TODO!
// need a way to read instructions to stop
// RPi command or boolean isStop() ??
#define TEST_MOTOR_TIMER 800


// private static variables, initialized with the motor_init...() family;

// timer pointers/variables
static TIM_HandleTypeDef *MOTOR_PWM_TIMER;
static uint32_t MOTOR_CHANNEL_LEFT;
static uint32_t MOTOR_CHANNEL_RIGHT;

static __IO uint32_t *MOTOR_LEFT_PWM_REGISTER;
static __IO uint32_t *MOTOR_RIGHT_PWM_REGISTER;

// motor GPIO banks
static GPIO_TypeDef *MOTOR_LEFT_GPIO_BANK;
static GPIO_TypeDef *MOTOR_RIGHT_GPIO_BANK;

// motor GPIO pins
// pin assignment as follows:
// set GPIO high for POS pins and GPIO low for neg pins
// to spin the wheel forward
// do the opposite to spin the wheel backwards

static uint16_t MOTOR_LEFT_GPIO_PIN_POS;
static uint16_t MOTOR_LEFT_GPIO_PIN_NEG;

static uint16_t MOTOR_RIGHT_GPIO_PIN_POS;
static uint16_t MOTOR_RIGHT_GPIO_PIN_NEG;


// private function prototypes


void _motor_left_set_pwm(MotorDirection dir, uint16_t pwm_val);
void _motor_right_set_pwm(MotorDirection dir, uint16_t pwm_val);


/**
 * Bring the relevant motor control variables into scope.
 * Same params as HAL_TIM_PWM_Start()
 * @param htim Motor timer handlers, assumes the same timer is used for both motor
 * @param channel_left Channel controlling the left motor
 * @param channel_right Channel controlling the right motor
 */
void motor_init_timer(TIM_HandleTypeDef *htim, uint32_t channel_left, uint32_t channel_right) {
	MOTOR_PWM_TIMER = htim;
	MOTOR_CHANNEL_LEFT = channel_left;
	MOTOR_CHANNEL_RIGHT = channel_right;

	// set the motor register once here (left)
	switch (MOTOR_CHANNEL_LEFT) {
	case TIM_CHANNEL_1:
		MOTOR_LEFT_PWM_REGISTER = &(MOTOR_PWM_TIMER->Instance->CCR1);
		break;
	case TIM_CHANNEL_2:
		MOTOR_LEFT_PWM_REGISTER = &(MOTOR_PWM_TIMER->Instance->CCR2);
		break;
	case TIM_CHANNEL_3:
		MOTOR_LEFT_PWM_REGISTER = &(MOTOR_PWM_TIMER->Instance->CCR3);
		break;
	case TIM_CHANNEL_4:
		MOTOR_LEFT_PWM_REGISTER = &(MOTOR_PWM_TIMER->Instance->CCR4);
		break;
	default:
		MOTOR_LEFT_PWM_REGISTER = NULL;
		break;
	}

	// set the motor register once here (right)
	switch (MOTOR_CHANNEL_RIGHT) {
	case TIM_CHANNEL_1:
		MOTOR_RIGHT_PWM_REGISTER = &(MOTOR_PWM_TIMER->Instance->CCR1);
		break;
	case TIM_CHANNEL_2:
		MOTOR_RIGHT_PWM_REGISTER = &(MOTOR_PWM_TIMER->Instance->CCR2);
		break;
	case TIM_CHANNEL_3:
		MOTOR_RIGHT_PWM_REGISTER = &(MOTOR_PWM_TIMER->Instance->CCR3);
		break;
	case TIM_CHANNEL_4:
		MOTOR_RIGHT_PWM_REGISTER = &(MOTOR_PWM_TIMER->Instance->CCR4);
		break;
	default:
		MOTOR_RIGHT_PWM_REGISTER = NULL;
		break;
	}

	HAL_TIM_PWM_Start(MOTOR_PWM_TIMER, MOTOR_CHANNEL_LEFT);
	HAL_TIM_PWM_Start(MOTOR_PWM_TIMER, MOTOR_CHANNEL_RIGHT);
}

/**
 * Bring the GPIO pin variables into scope for the left motor
 * Note that the GPIO pins need to be passed in the correct order
 * @param gpio_bank Pin bank (A-E)
 * @param gpio_pin_pos [A-D]-IN-1 pin
 * @param gpio_pin_neg [A-D]-IN-2 pin
 */
void motor_init_gpio_left(GPIO_TypeDef *gpio_bank, uint16_t gpio_pin_pos, uint16_t gpio_pin_neg) {
	MOTOR_LEFT_GPIO_BANK = gpio_bank;
	MOTOR_LEFT_GPIO_PIN_POS = gpio_pin_pos;
	MOTOR_LEFT_GPIO_PIN_NEG = gpio_pin_neg;
}

/**
 * Bring the GPIO pin variables into scope for the left motor
 * Note that the GPIO pins need to be passed in the correct order
 * @param gpio_bank Pin bank (A-E)
 * @param gpio_pin_pos [A-D]-IN-1 pin
 * @param gpio_pin_neg [A-D]-IN-2 pin
 */
void motor_init_gpio_right(GPIO_TypeDef *gpio_bank, uint16_t gpio_pin_pos, uint16_t gpio_pin_neg) {
	MOTOR_RIGHT_GPIO_BANK = gpio_bank;
	MOTOR_RIGHT_GPIO_PIN_POS = gpio_pin_pos;
	MOTOR_RIGHT_GPIO_PIN_NEG = gpio_pin_neg;
}

// Set the left motor with a particular PWM value
// TODO: Safety checks on the pwm value
void _motor_left_set_pwm(MotorDirection dir, uint16_t pwm_val) {
	HAL_GPIO_WritePin(
			MOTOR_LEFT_GPIO_BANK,
			MOTOR_LEFT_GPIO_PIN_POS,
			(dir == MotorDirForward) ? GPIO_PIN_SET : GPIO_PIN_RESET
	);
	HAL_GPIO_WritePin(
			MOTOR_LEFT_GPIO_BANK,
			MOTOR_LEFT_GPIO_PIN_NEG,
			(dir == MotorDirForward) ? GPIO_PIN_RESET : GPIO_PIN_SET
	);
	*MOTOR_LEFT_PWM_REGISTER = pwm_val;
//	__HAL_TIM_SetCompare(MOTOR_PWM_TIMER, MOTOR_CHANNEL_LEFT, pwm_val);

}

// Set the right motor with a particular PWM value
// TODO: Safety checks on the pwm value
void _motor_right_set_pwm(MotorDirection dir, uint16_t pwm_val) {
	HAL_GPIO_WritePin(
			MOTOR_RIGHT_GPIO_BANK,
			MOTOR_RIGHT_GPIO_PIN_POS,
			(dir == MotorDirForward) ? GPIO_PIN_SET : GPIO_PIN_RESET
	);
	HAL_GPIO_WritePin(
			MOTOR_RIGHT_GPIO_BANK,
			MOTOR_RIGHT_GPIO_PIN_NEG,
			(dir == MotorDirForward) ? GPIO_PIN_RESET : GPIO_PIN_SET
	);
	*MOTOR_RIGHT_PWM_REGISTER = pwm_val;
//	__HAL_TIM_SetCompare(MOTOR_PWM_TIMER, MOTOR_CHANNEL_RIGHT, pwm_val);

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

	motor_forward(MotorSpeed1);
	HAL_Delay(1000);
//	motor_forward(MotorSpeed2);
//	HAL_Delay(1000);
//	motor_forward(MotorSpeed3);
//	HAL_Delay(1000);
	motor_stop();
	HAL_Delay(1000);
	motor_backward(MotorSpeed1);
	HAL_Delay(1000);
//	motor_backward(MotorSpeed2);
//	HAL_Delay(1000);
//	motor_backward(MotorSpeed3);
//	HAL_Delay(1000);
	motor_stop();

}


/**
 * Move forward at a specified speed
 */
void motor_forward(MotorSpeed speed) {

//	uint16_t pwmVal = 100;
//	uint16_t timer = TEST_MOTOR_TIMER;

	_motor_left_set_pwm(MotorDirForward, speed);
	_motor_right_set_pwm(MotorDirForward, speed);


//	while (timer > 0) { // RPi command or boolean isStop() ??
//
//		_motor_left_set_pwm(MotorDirForward, speed);
//		_motor_right_set_pwm(MotorDirForward, speed);
////		// left wheel clockwise
////		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
////		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
////
////		// right wheel anti-clockwise
////		HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
////		HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
////
////		// modify comparison value for the duty cycle
////		__HAL_TIM_SetCompare(MOTOR_PWM_TIMER, MOTOR_CHANNEL_LEFT, speed);
////		__HAL_TIM_SetCompare(MOTOR_PWM_TIMER, MOTOR_CHANNEL_RIGHT, speed);
//
//		timer--;
//	}
//	motor_adjust_forward();
}


/**
 * Reverse at a specified speed
 */
void motor_backward(MotorSpeed speed) {


	_motor_left_set_pwm(MotorDirBackward, speed);
	_motor_right_set_pwm(MotorDirBackward, speed);


//	uint16_t timer = TEST_MOTOR_TIMER;

//	while (1) { // RPi command or boolean isStop() ??
//
////		// left wheel clockwise
////		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
////		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
////
////		// right wheel anti-clockwise
////		HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
////		HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
////
////		// modify comparison value for the duty cycle
////		__HAL_TIM_SetCompare(MOTOR_PWM_TIMER, MOTOR_CHANNEL_LEFT, speed);
////		__HAL_TIM_SetCompare(MOTOR_PWM_TIMER, MOTOR_CHANNEL_RIGHT, speed);
//
//		timer--;
//	}
////	motor_adjust_backward();
}


/**
 * Stop the motors
 */
void motor_stop() {

	_motor_left_set_pwm(MotorDirForward, 0);
	_motor_right_set_pwm(MotorDirForward, 0);

}

