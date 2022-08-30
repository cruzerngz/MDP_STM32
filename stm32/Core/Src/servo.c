/*
 * servo.c
 *
 *  Created on: 27 Aug 2022
 *      Author: cruzerng
 */

#include "servo.h"

// private static variables, initialized with servo_init();

static TIM_HandleTypeDef *SERVO_PWM_TIMER;

// channel is (0-indexed) * 4
// channel 1: 0,
// channel 2: 4, etc.
static uint32_t SERVO_CHANNEL;

static TIM_TypeDef *SERVO_TIM_DEF;


// private function prototypes

//TIM_TypeDef *get_timer();

// init and showcase functions



/**
 * Bring the relavant servo control variables into scope.
 * Same params as HAL_TIM_PWM_Start()
 */
void servo_init(TIM_HandleTypeDef *htim, uint32_t Channel) {
	SERVO_PWM_TIMER = htim;
	SERVO_CHANNEL = Channel;

	// set the channel once here
//	switch (SERVO_CHANNEL) {
//	case 0:
//		SERVO_TIM_DEF = SERVO_PWM_TIMER->Instance->CCR1;
//		break;
//	case 4:
//		SERVO_TIM_DEF = SERVO_PWM_TIMER->Instance->CCR2;
//		break;
//	case 8:
//		SERVO_TIM_DEF = SERVO_PWM_TIMER->Instance->CCR3;
//		break;
//	case 12:
//		SERVO_TIM_DEF = SERVO_PWM_TIMER->Instance->CCR4;
//		break;
//	default:
//		SERVO_TIM_DEF = NULL;
//	}

	HAL_TIM_PWM_Start(SERVO_PWM_TIMER, SERVO_CHANNEL);
}

/**
 *
 */
void servo_test_startup() {
	servo_point_center();
	HAL_Delay(300);
	servo_point_right_full();
	HAL_Delay(300);
	servo_point_left_full();
	HAL_Delay(300);
	servo_point_center();
}

/**
 * Go through all servo movements
 */
void servo_showcase() {

}

// directions

/**
 * Point the wheels at a certain direction with a specified magnitude
 */
void servo_point() {

}

/**
 * Point the wheels all the way to the left
 */
void servo_point_left_full(){
	switch (SERVO_CHANNEL) {
	case 0:
		SERVO_PWM_TIMER->Instance->CCR1 = SERVO_LEFT_MAX;
		break;
	case 4:
		SERVO_PWM_TIMER->Instance->CCR2 = SERVO_LEFT_MAX;
		break;
	case 8:
		SERVO_PWM_TIMER->Instance->CCR3 = SERVO_LEFT_MAX;
		break;
	case 12:
		SERVO_PWM_TIMER->Instance->CCR4 = SERVO_LEFT_MAX;
		break;
	default:
		break;
	}

}
/**
 * Point the wheels all the way to the right
 */
void servo_point_right_full(){
	switch (SERVO_CHANNEL) {
	case 0:
		SERVO_PWM_TIMER->Instance->CCR1 = SERVO_RIGHT_MAX;
		break;
	case 4:
		SERVO_PWM_TIMER->Instance->CCR2 = SERVO_RIGHT_MAX;
		break;
	case 8:
		SERVO_PWM_TIMER->Instance->CCR3 = SERVO_RIGHT_MAX;
		break;
	case 12:
		SERVO_PWM_TIMER->Instance->CCR4 = SERVO_RIGHT_MAX;
		break;
	default:
		break;
	}

}

/**
 * Center the wheels
 */
void servo_point_center() {


	switch (SERVO_CHANNEL) {
	case 0:
		SERVO_PWM_TIMER->Instance->CCR1 = SERVO_CENTER;
		break;
	case 4:
		SERVO_PWM_TIMER->Instance->CCR2 = SERVO_CENTER;
		break;
	case 8:
		SERVO_PWM_TIMER->Instance->CCR3 = SERVO_CENTER;
		break;
	case 12:
		SERVO_PWM_TIMER->Instance->CCR4 = SERVO_CENTER;
		break;
	default:
		break;
	}
}
