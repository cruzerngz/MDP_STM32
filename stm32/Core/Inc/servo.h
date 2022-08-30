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
#define SERVO_OFFSET 40 // Center-to-side offset. Still in trial and err phase

// left and right offsets are not the same
#define SERVO_LEFT_LIMIT 50
#define SERVO_RIGHT_LIMIT 74

#define SERVO_LEFT_MAX (SERVO_CENTER - SERVO_LEFT_LIMIT)
#define SERVO_RIGHT_MAX (SERVO_CENTER + SERVO_RIGHT_LIMIT)

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
