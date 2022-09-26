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

// Left is [0], Right is [1]
volatile uint32_t ENCODER_POS[2] = {0}; // raw readout from timer
volatile uint32_t ENCODER_POS_DIRECTIONAL[2] = {0}; // timer readout, compensated for opp rotations
volatile int16_t ENCODER_SPEED[2] = {0}; // speed in mm/s, pos for clockwise
volatile int16_t ENCODER_SPEED_DIRECTIONAL[2] = {0}; // speed in mm/s, pos for forward

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

// read encoder for a side. Left = 0, Right = 1
uint32_t _read_encoder_side_mm(uint8_t side);

// calculate the encoder speed, in millimeters per sec
void _calc_encoder_speed_mm_s();

// same as above, use HAL_Delay to read the encoder
int _read_encoder_side_delay(uint8_t side);



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
	return _read_encoder_side_mm(0);
}


uint32_t _read_encoder_right() {
//	return _read_encoder_side_delay(1);
	return _read_encoder_side_mm(1);
}

// Reads the absolute positions of the encoders, in mm
// Note that a positive delta (rising value) indicates clockwise motor rotation
// Negative delta (falling value) indicates anticlockwise motor rotation
uint32_t _read_encoder_side_mm(uint8_t side) {
	volatile uint32_t *encoder_side;

	if(side == 1) {
		encoder_side = MOTOR_ENCODER_READOUT_RIGHT;
		ENCODER_POS[side] = *encoder_side;
		ENCODER_POS_DIRECTIONAL[side] = *encoder_side;
	}
	else if(side == 0) {
		encoder_side = MOTOR_ENCODER_READOUT_LEFT;
		ENCODER_POS[side] = *encoder_side;
		ENCODER_POS_DIRECTIONAL[side] = MOTOR_ENCODER_MAX_POS - *encoder_side;
	}
	else return 0;


	return (uint32_t)(*encoder_side * MOTOR_ENCODER_CONSTANT);
}

//void _read_encoder_raw(uint8_t side) {
//
//}

// old function
int _read_encoder_side_delay(uint8_t side) {
	int cnt1, cnt2, diff;
	TIM_HandleTypeDef *encoder_side;
	if(side == 1) encoder_side = MOTOR_ENCODER_TIMER_RIGHT;
	else if(side == 0) encoder_side = MOTOR_ENCODER_TIMER_LEFT;
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

// calculate the encoder speed, in millimeters per sec
// Positive values indicate clockwise rotation
// negative values indicate anticlockwise rotation
// Assumes the polling rate defined in header file is used
void _calc_encoder_speed_mm_s() {
	// initial values
	static uint32_t r_start = 0;
	static uint32_t l_start = 0;
	// compared against these
	static uint32_t r_end = 0;
	static uint32_t l_end = 0;

	// read and calc delta
	r_end = ENCODER_POS[1];
	l_end = ENCODER_POS[0];

	ENCODER_SPEED[0] = (int16_t)(((int32_t)l_end - (int32_t)l_start) * MOTOR_ENCODER_CONSTANT * MOTOR_ENCODER_REFRESH_INTERVAL_FREQ);
	ENCODER_SPEED[1] = (int16_t)(((int32_t)r_end - (int32_t)r_start) * MOTOR_ENCODER_CONSTANT * MOTOR_ENCODER_REFRESH_INTERVAL_FREQ);
	ENCODER_SPEED_DIRECTIONAL[1] = ENCODER_SPEED[1];
	ENCODER_SPEED_DIRECTIONAL[0] = 0 - ENCODER_SPEED[0];

	r_start = r_end;
	l_start = l_end;
}

void encoder_reset_counters(void) {

}


// Main polling function to the encoders.
//
void encoder_poll(void) {
	ENCODER_POS[0] = *MOTOR_ENCODER_READOUT_LEFT;
	ENCODER_POS[1] = *MOTOR_ENCODER_READOUT_RIGHT;

	ENCODER_POS_DIRECTIONAL[0] = (MOTOR_ENCODER_MAX_POS - *MOTOR_ENCODER_READOUT_LEFT) % MOTOR_ENCODER_MAX_POS;
	ENCODER_POS_DIRECTIONAL[1] = *MOTOR_ENCODER_READOUT_RIGHT % MOTOR_ENCODER_MAX_POS;

	_calc_encoder_speed_mm_s();
}


