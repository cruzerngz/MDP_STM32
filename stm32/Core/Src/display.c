/*
 * display.c
 *
 *  Created on: Sep 7, 2022
 *      Author: jimmysqqr
 */

#include <string.h>
#include <stdio.h>

#include "display.h"
#include "oled.h"


// private macros


// private static variables
// timer pointers/variables
static TIM_HandleTypeDef *MOTOR_ENCODER_TIMER_LEFT;
static TIM_HandleTypeDef *MOTOR_ENCODER_TIMER_RIGHT;
static uint32_t MOTOR_CHANNEL_LEFT;
static uint32_t MOTOR_CHANNEL_RIGHT;


// private function prototypes
int _read_encoder_left();
int _read_encoder_right();


/**
 * Bring the relevant encoder control variables into scope.
 * Same params as HAL_TIM_PWM_Start()
 */
void encoder_init(TIM_HandleTypeDef *htim_left, TIM_HandleTypeDef *htim_right, uint32_t channel_left, uint32_t channel_right) {


	MOTOR_ENCODER_TIMER_LEFT = htim_left;
	MOTOR_ENCODER_TIMER_RIGHT = htim_right;
	MOTOR_CHANNEL_LEFT = channel_left;
	MOTOR_CHANNEL_RIGHT = channel_right;


	HAL_TIM_Encoder_Start(MOTOR_ENCODER_TIMER_LEFT, MOTOR_CHANNEL_LEFT);
	HAL_TIM_Encoder_Start(MOTOR_ENCODER_TIMER_RIGHT, MOTOR_CHANNEL_RIGHT);


//	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
//	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
}

void oled_init() {
	OLED_Display_On();
	HAL_Delay(2000);
}



int _read_encoder_left() {
	int cnt1, cnt2, diff;
	cnt1 = __HAL_TIM_GET_COUNTER(MOTOR_ENCODER_TIMER_LEFT);
	HAL_Delay(500); // delay 0.5s
	cnt2 = __HAL_TIM_GET_COUNTER(MOTOR_ENCODER_TIMER_LEFT);
	if(__HAL_TIM_IS_TIM_COUNTING_DOWN(MOTOR_ENCODER_TIMER_LEFT)) { // deceleration
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


int _read_encoder_right() {
	int cnt1, cnt2, diff;
	cnt1 = __HAL_TIM_GET_COUNTER(MOTOR_ENCODER_TIMER_RIGHT);
	HAL_Delay(500); // delay 0.5s
	cnt2 = __HAL_TIM_GET_COUNTER(MOTOR_ENCODER_TIMER_RIGHT);
	if(__HAL_TIM_IS_TIM_COUNTING_DOWN(MOTOR_ENCODER_TIMER_RIGHT)) { // deceleration
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



void display_speed() {
	int speed_left, speed_right;
	uint8_t buffer_left[20], buffer_right[20];
	for(;;)
	{
		speed_left = _read_encoder_left();
		sprintf(buffer_left, "L Speed: %5d", speed_left);
		OLED_ShowString(10, 20, buffer_left);
		OLED_Refresh_Gram();

		speed_right = _read_encoder_right();
		sprintf(buffer_right, "R Speed: %5d", speed_right);
		OLED_ShowString(10, 35, buffer_right);
		OLED_Refresh_Gram();
	}
}










