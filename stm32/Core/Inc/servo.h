/*
 * servo.h
 *
 *  Created on: 27 Aug 2022
 *      Author: cruzerng
 */

#include "stm32f4xx_hal.h"


#ifndef INC_SERVO_H_
#define INC_SERVO_H_

// ++ adjusts to the right
// -- adjusts to the left
#define SERVO_CENTER 143 // locked

// amount to offset when centering from (left/right)
#define SERVO_CENTERING_OFFSET 1

// left and right offsets are not the same
#define SERVO_LEFT_LIMIT 50
#define SERVO_RIGHT_LIMIT 90

#define SERVO_LEFT_MAX (SERVO_CENTER - SERVO_LEFT_LIMIT)
#define SERVO_RIGHT_MAX (SERVO_CENTER + SERVO_RIGHT_LIMIT)

#define SERVO_MIN_DELAY_TICKS 100

#define ARRAY_LEN(_array_) (sizeof(_array_) / sizeof(_array_[0]))


// Servo direction control variable
typedef enum {
	ServoDirLeft,
	ServoDirRight,
	ServoDirCenter
} ServoDirection;

// Servo magnitude control variable
typedef enum {
	ServoMag1,
	ServoMag2,
	ServoMag3,
	ServoMag4,
	ServoMag5
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

// variable directions
void servo_point_degrees(ServoDirection dir, uint8_t degrees);

#endif /* INC_SERVO_H_ */
