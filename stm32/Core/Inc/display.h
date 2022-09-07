/*
 * display.h
 *
 *  Created on: Sep 7, 2022
 *      Author: jimmysqqr
 */

#include "stm32f4xx_hal.h"


#ifndef INC_DISPLAY_H_
#define INC_DISPLAY_H_

void encoder_init(TIM_HandleTypeDef *htim_left, TIM_HandleTypeDef *htim_right, uint32_t channel_left, uint32_t channel_right);
void oled_init();

//int read_encoder_left();
//int read_encoder_right();
void display_speed();

#endif /* INC_DISPLAY_H_ */
