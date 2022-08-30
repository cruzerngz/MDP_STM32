/*
 * servo.h
 *
 *  Created on: 27 Aug 2022
 *      Author: cruzerng
 */

#include "stm32f4xx_hal.h"


#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#define SERVO_CENTER 144
#define SERVO_LEFT_MAX 130
#define SERVO_RIGHT_MAX 160

// init and showcase functions
void servo_init(TIM_HandleTypeDef *htim, uint32_t Channel);
void servo_test_startup();
void servo_showcase();

// directions
void servo_point();
void servo_point_left_full();
void servo_point_right_full();
void servo_point_center();

#endif /* INC_SERVO_H_ */
