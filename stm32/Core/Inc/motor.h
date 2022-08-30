/*
 * motor.h
 *
 *  Created on: 31 Aug 2022
 *      Author: jimmysqqr
 */

#include "stm32f4xx_hal.h"

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

// Motor speed control variable
typedef enum { // to be tested
	MotorSpeed1 = 500,
	MotorSpeed2 = 1000,
	MotorSpeed3 = 1500,
} MotorSpeed;

// init and showcase functions
void motor_init(TIM_HandleTypeDef *htim, uint32_t Channel1, uint32_t Channel2);
void motor_test_startup();
void motor_showcase();

// moveset
void motor_forward(MotorSpeed speed);
void motor_reverse(MotorSpeed speed);
void motor_stop();

#endif /* INC_MOTOR_H_ */
