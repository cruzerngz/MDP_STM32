/*
 * motor.h
 *
 *  Created on: 31 Aug 2022
 *      Author: jimmysqqr
 */

#include "stm32f4xx_hal.h"

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

// Motor select enum
typedef enum {
	MotorLeft,
	MotorRight
} MotorSide;

// Motor direction control
typedef enum {
	MotorDirForward,
	MotorDirBackward
} MotorDirection;

// Motor speed control variable
typedef enum { // TODO: to be tested
	MotorSpeed1 = 750,
	MotorSpeed2 = 1250,
	MotorSpeed3 = 3000,
} MotorSpeed;

// init and showcase functions
void motor_init_timer(TIM_HandleTypeDef *htim, uint32_t Channel1, uint32_t Channel2);
void motor_init_gpio_left(GPIO_TypeDef *gpio_bank, uint16_t gpio_pin_pos, uint16_t gpio_pin_neg);
void motor_init_gpio_right(GPIO_TypeDef *gpio_bank, uint16_t gpio_pin_pos, uint16_t gpio_pin_neg);


void motor_test_startup();
void motor_showcase();

// pwm set
void _motor_set_pwm(MotorDirection dir, MotorSide side, uint16_t pwm_val);
// void _motor_left_set_pwm(MotorDirection dir, uint16_t pwm_val);
// void _motor_right_set_pwm(MotorDirection dir, uint16_t pwm_val);


// moveset
void motor_forward(MotorSpeed speed);
void motor_backward(MotorSpeed speed);
void motor_stop();
void motor_stop_backward();

#endif /* INC_MOTOR_H_ */
