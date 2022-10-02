/*
 * encoder.h
 *
 *  Created on: Sep 7, 2022
 *      Author: jimmysqqr
 *
 * Motor encoder module
 */

#include "stm32f4xx_hal.h"


#ifndef INC_DISPLAY_H_
#define INC_DISPLAY_H_

#define MOTOR_ENCODER_CONSTANT 0.2757859901f
#define MOTOR_ENCODER_MAX_POS 65535 // uint16_max, timer rollover

// encoder speed in mm/s
extern volatile int16_t ENCODER_SPEED[2];

extern volatile int16_t ENCODER_SPEED_DIRECTIONAL[2];

// encoder pos in mm
extern volatile uint32_t ENCODER_POS[2];

// encoder pos in mm,
extern volatile uint32_t ENCODER_POS_DIRECTIONAL[2];

// 100Hz polling freq, for accuracy
#define MOTOR_ENCODER_REFRESH_INTERVAL_TICKS 10 // in ticks, 1 tick = 1ms
#define MOTOR_ENCODER_REFRESH_INTERVAL_FREQ 100 // In hertz

void encoder_init(TIM_HandleTypeDef *htim_left, TIM_HandleTypeDef *htim_right, uint32_t channel_left, uint32_t channel_right);
void encoder_poll(void); // polling with freq set in main (timer loop delay)
void encoder_reset_counters_backward(void); // reset internal bit counters
void encoder_reset_counters_forward(void); // reset internal bit counters

uint32_t _read_encoder_left();
uint32_t _read_encoder_right();
//void display_speed();

#endif /* INC_DISPLAY_H_ */
