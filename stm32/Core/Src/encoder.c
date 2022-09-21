/*
 * encoder.c
 *
 *  Created on: Sep 7, 2022
 *      Author: jimmysqqr
 */

#include <encoder.h>
#include <string.h>
#include <stdio.h>

#include "oled.h"

#include "cmsis_os.h"

// private macros


// private static variables
// timer pointers/variables
static TIM_HandleTypeDef *MOTOR_ENCODER_TIMER_LEFT;
static TIM_HandleTypeDef *MOTOR_ENCODER_TIMER_RIGHT;
static uint32_t MOTOR_CHANNEL_LEFT;
static uint32_t MOTOR_CHANNEL_RIGHT;

static volatile uint32_t *MOTOR_ENCODER_READOUT_LEFT = NULL;
static volatile uint32_t *MOTOR_ENCODER_READOUT_RIGHT = NULL;


// private function prototypes
uint32_t _read_encoder_left();
uint32_t _read_encoder_right();

// read encoder for a side. Left = -1, Right = 1
uint32_t _read_encoder_side(int8_t side);

// same as above, use HAL_Delay to read the encoder
int _read_encoder_side_delay(int8_t side);



/**
 * Bring the relevant encoder control variables into scope.
 * Same params as HAL_TIM_PWM_Start()
 */
void encoder_init(TIM_HandleTypeDef *htim_left, TIM_HandleTypeDef *htim_right, uint32_t channel_left, uint32_t channel_right) {


	MOTOR_ENCODER_TIMER_LEFT = htim_left;
	MOTOR_ENCODER_TIMER_RIGHT = htim_right;
	MOTOR_CHANNEL_LEFT = channel_left;
	MOTOR_CHANNEL_RIGHT = channel_right;

	MOTOR_ENCODER_READOUT_LEFT = &(htim_left->Instance->CNT);
	MOTOR_ENCODER_READOUT_RIGHT = &(htim_right->Instance->CNT);

	HAL_TIM_Encoder_Start(MOTOR_ENCODER_TIMER_LEFT, MOTOR_CHANNEL_LEFT);
	HAL_TIM_Encoder_Start(MOTOR_ENCODER_TIMER_RIGHT, MOTOR_CHANNEL_RIGHT);


//	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
//	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
}


uint32_t _read_encoder_left() {
//	return _read_encoder_side_delay(-1);
	return _read_encoder_side(-1);
}


uint32_t _read_encoder_right() {
//	return _read_encoder_side_delay(1);
	return _read_encoder_side(1);
}

// Reads the absolute positions of the encoders
// Note that a positive delta (rising value) indicates clockwise motor rotation
// Negative delta (falling value) indicates anticlockwise motor rotation
uint32_t _read_encoder_side(int8_t side) {
	volatile uint32_t *encoder_side;

	if(side == 1) encoder_side = MOTOR_ENCODER_READOUT_RIGHT;
	else if(side == -1) encoder_side = MOTOR_ENCODER_READOUT_LEFT;
	else return 0;


	return (uint32_t)(*encoder_side * MOTOR_ENCODER_CONSTANT);
}

int _read_encoder_side_delay(int8_t side) {
	int cnt1, cnt2, diff;
	TIM_HandleTypeDef *encoder_side;
	if(side == 1) encoder_side = MOTOR_ENCODER_TIMER_RIGHT;
	else if(side == -1) encoder_side = MOTOR_ENCODER_TIMER_LEFT;
	else return 0;

	cnt1 = __HAL_TIM_GET_COUNTER(encoder_side);
	HAL_Delay(500); // delay 0.5s
	cnt2 = __HAL_TIM_GET_COUNTER(encoder_side);
	if(__HAL_TIM_IS_TIM_COUNTING_DOWN(encoder_side)) { // deceleration
		if(cnt2 < cnt1) {
			diff = cnt1 - cnt2;
		} else {
			diff = (INT16_MAX - cnt2) + cnt1;
		}
	} else { // acceleration
		if(cnt2 > cnt1) {
			diff = cnt2 - cnt1;
		} else {
			diff = (INT16_MAX - cnt1) + cnt2;
		}
	}
	return diff;
}


// Main polling function to the encoders.
//
void encoder_poll(void) {

}


