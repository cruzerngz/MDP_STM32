/*
 * servo.h
 *
 *  Created on: 27 Aug 2022
 *      Author: cruzerng
 */

#include "stm32f4xx_hal.h"


#ifndef INC_SERVO_H_
#define INC_SERVO_H_

// Servo direction control variable
typedef enum {
	ServoDirLeft,
	ServoDirRight
} ServoDirection;

// Servo magnitude control variable
typedef enum {
	ServoMag1,
	ServoMag2,
	ServoMag3,
	ServoMag4
} ServoMagnitude;

// init and showcase functions
void servo_init(TIM_HandleTypeDef *htim, uint32_t Channel);
void servo_test_startup();
void servo_showcase();

// directions
void servo_point(ServoDirection dir, ServoMagnitude mag);
void servo_point_center();
void servo_point_left_full();
void servo_point_right_full();

#endif /* INC_SERVO_H_ */
