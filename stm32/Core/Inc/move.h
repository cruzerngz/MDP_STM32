/*
 * move.h
 *
 *  Created on: Sep 1, 2022
 *      Author: jimmysqqr
 */
#include "motor.h"
#include "servo.h"

#ifndef INC_MOVE_H_
#define INC_MOVE_H_

#define MOVE_LOW_GRIP_SURFACE // define this to enable slippery surface constants in computation

// 5.5, 16.0, 0.13 slight overshoot (no resistance on wheels)

#ifdef MOVE_LOW_GRIP_SURFACE // low grip surface mode, AKA the SCSE wing brick floor

#define MOVE_KP 7.0f//4.0f//6.0f //4.0f
#define MOVE_KI 24.0f//1.0f//6.0f //4.5f //40.0f
#define MOVE_KD 0.0f//0.3f //0.2f //0.13f

// default speeds
#define MOVE_DEFAULT_SPEED_STRAIGHT_MM_S 175
#define MOVE_DEFAULT_SPEED_TURN_MM_S 115
#define MOVE_PID_LOOP_PERIOD_TICKS 50 // pid refresh ticks

// difference in motor speed between outside and inside wheels
// when turining with servo magnitude 5
#define MOVE_PID_TURN_REDUCTION_FACTOR 0.584097859f
#define MOVE_PID_TURN_TICKS_PER_DEGREE 34.07755365f
#define MOVE_PID_TURN_OUTER_MM_PER_DEGREE 7.06666666666f

// multiplier for turning backwards, to account for wheel slip
#define MOVE_PID_SLIP_MULTIPLIER 1.0275f

#else

#define MOVE_KP 9.0f//4.0f//6.0f //4.0f
#define MOVE_KI 30.0f//1.0f//6.0f //4.5f //40.0f
#define MOVE_KD 0.0f//0.3f //0.2f //0.13f

// default speeds
#define MOVE_DEFAULT_SPEED_STRAIGHT_MM_S 400
#define MOVE_DEFAULT_SPEED_TURN_MM_S 250
#define MOVE_PID_LOOP_PERIOD_TICKS 50 // pid refresh ticks

// difference in motor speed between outside and inside wheels
// when turining with servo magnitude 5
#define MOVE_PID_TURN_REDUCTION_FACTOR 0.584097859f
#define MOVE_PID_TURN_TICKS_PER_DEGREE 34.07755365f
#define MOVE_PID_TURN_OUTER_MM_PER_DEGREE 7.06666666666f

// multiplier for turning backwards, to account for wheel slip
#define MOVE_PID_SLIP_MULTIPLIER 1.0275f

#endif

// #define MOVE_CAR_OUTER_WHEEL_RADIUS 39.05f
// #define MOVE_CAR_INNER_WHEEL_RADIUS 22.75f

// Move direction inherited from servo.h
// Exposed here as another enum
typedef enum {
	MoveDirLeft = ServoDirLeft,
	MoveDirRight = ServoDirRight
} MoveDirection;

// temp
extern float MOTOR_INTEGRATION_SUM[2];

// init and showcase functions
void move_test_startup();

// isolated moveset functions
void forward(MotorSpeed speed); // assume needs calibration
void backward(MotorSpeed speed); // assume needs calibration
void steer(ServoDirection dir, ServoMagnitude mag);
void steer_straight();
void stop();

// dynamic movement functions, require the use of the encoder
void move_forward_calc(uint32_t centimeters);
void move_backward_calc(uint32_t centimeters);

void move_forward_pid_cm(uint32_t centimeters);
void move_backward_pid_cm(uint32_t centimeters);

void move_turn_forward_by(MoveDirection direction, uint16_t degrees);
void move_turn_backward_by(MoveDirection direction, uint16_t degrees);

void move_turn_forward_pid_degrees(MoveDirection direction, uint16_t degrees);
void move_turn_backward_pid_degrees(MoveDirection direction, uint16_t degrees);

void move_in_place_turn_by(MoveDirection direction, uint16_t degrees);
void move_in_place_turn_cardinal(uint8_t cardinal_direction);

void move_to_obstacle(void);

void _set_motor_first_pwm_val(MotorDirection dir, MotorSide side, uint16_t speed_mm_s);
void _set_motor_speed_pid(MotorDirection dir, MotorSide side, uint16_t speed_mm_s);

// Hardcoded, known movements (requires further calibration)

void move_hard_left_45();
void move_hard_left_90();
void move_hard_left_135();
void move_hard_left_180();

void move_hard_right_45();
void move_hard_right_90();
void move_hard_right_135();
void move_hard_right_180();

#endif /* INC_MOVE_H_ */
