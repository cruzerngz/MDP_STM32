/*
 * move.h
 *
 *  Created on: Sep 1, 2022
 *      Author: jimmysqqr
 */
#include "motor.h"
#include "servo.h"
#include "stdbool.h"

#ifndef INC_MOVE_H_
#define INC_MOVE_H_

// #define MOVE_LOW_GRIP_SURFACE // define this to enable slippery surface constants in computation
// #define MOVE_MAX_TURN // define this to enable tighter turns (as tight as the servo allows) TODO
// #define MOVE_TURN_OVERDRIVE // define this to enable even tighter turns, uses the outer wheel TODO

// small overdrive for left motor
#define MOVE_LEFT_MOTOR_MULTIPLIER 1.025f

#ifdef MOVE_TURN_OVERDRIVE
#define MOVE_OUTER_OVERDRIVE_RATIO 1.2f
#else
#define MOVE_OUTER_OVERDRIVE_RATIO 1
#endif

#ifdef MOVE_MAX_TURN
#define MOVE_PID_TURN_REDUCTION_FACTOR 0.525f
// #define MOVE_PID_TURN_TICKS_PER_DEGREE 34.07755365f
#define MOVE_PID_TURN_OUTER_MM_PER_DEGREE 6.75f
#else
// difference in motor speed between outside and inside wheels
// when turining with servo magnitude 5
#define MOVE_PID_TURN_REDUCTION_FACTOR 0.58897f
#define MOVE_PID_TURN_TICKS_PER_DEGREE 34.07755365f
#define MOVE_PID_TURN_OUTER_MM_PER_DEGREE 7.07f
#define MOVE_PID_INNER_WHEEL_UNDERDRIVE 0.9f
#define MOVE_PID_OUTER_WHEEL_OVERDRIVE 1.11f
#endif

// 5.5, 16.0, 0.13 slight overshoot (no resistance on wheels)

#ifdef MOVE_LOW_GRIP_SURFACE // low grip surface mode, AKA the SCSE wing brick floor

#define MOVE_KP 7.0f//4.0f//6.0f //4.0f
#define MOVE_KI 18.0f//1.0f//6.0f //4.5f //40.0f
#define MOVE_KD 0.0f//0.3f //0.2f //0.13f

// default speeds
#define MOVE_DEFAULT_SPEED_STRAIGHT_MM_S 200
#define MOVE_DEFAULT_SPEED_TURN_MM_S 150
#define MOVE_PID_LOOP_PERIOD_TICKS 50 // pid refresh ticks

// multiplier for turning backwards, to account for wheel slip
#define MOVE_PID_SLIP_MULTIPLIER 1.0f //1.0255f
#define MOVE_REVERSE_MULTIPLIER 1.1f //1.01f

#else

#define MOVE_KP 7.0f//4.0f//6.0f //4.0f
#define MOVE_KI 20.0f//1.0f//6.0f //4.5f //40.0f
#define MOVE_KD 0.0f//0.3f //0.2f //0.13f

// default speeds
#define MOVE_DEFAULT_SPEED_STRAIGHT_MM_S 1100 // prev 350
#define MOVE_DEFAULT_SPEED_TURN_MM_S 740 // prev 300
#define MOVE_PID_LOOP_PERIOD_TICKS 50 // pid refresh ticks

// multiplier for turning backwards, to account for wheel slip
#define MOVE_PID_SLIP_MULTIPLIER 1.0275f

#endif

// unused, use move_default speeds for now
#define MOVE_HIGH_SPEED_STRAIGHT_MM_S 1000
#define MOVE_HIGH_SPEED_TURN_MM_S 750
#define MOVE_HIGH_SPEED_SLOW_STRAIGHT_MM_S 750
#define MOVE_HIGH_SPEED_SLOW_TURN_MM_S 500

// from r to l, straight less, turn right less
// from r to r, turn tighter?
// from l to l,
// l to r, straught more, turn left less


// fastest car macros
#define MOVE_F_OP1_FAST_TURN_LEFT_A1 42
#define MOVE_F_OP1_FAST_TURN_LEFT_A2 45

#define MOVE_F_OP1_FAST_TURN_RIGHT_A1 38
#define MOVE_F_OP1_FAST_TURN_RIGHT_A2 53

#define MOVE_F_OP2_FAST_TURN_LEFT_A1 42
#define MOVE_F_OP2_FAST_TURN_LEFT_A2 44
#define MOVE_F_OP2_FAST_TURN_LDISP 450
#define MOVE_F_OP2_FAST_TURN_LEFT_DISP 60

#define MOVE_F_OP2_FAST_TURN_RIGHT_A1 55 // to calibrate
#define MOVE_F_OP2_FAST_TURN_RIGHT_A2 65
#define MOVE_F_OP2_FAST_TURN_RDISP 450
#define MOVE_F_OP2_FAST_TURN_RIGHT_DISP 70

#define MOVE_F_UTURN_FAST_LEFT 200
#define MOVE_F_UTURN_FAST_RIGHT 195

// fastest car macros - slower version
#define MOVE_F_OP1_TURN_LEFT_A1 35
#define MOVE_F_OP1_TURN_LEFT_A2 38

#define MOVE_F_OP1_TURN_RIGHT_A1 35
#define MOVE_F_OP1_TURN_RIGHT_A2 45

#define MOVE_F_OP2_TURN_LEFT_A1 55
#define MOVE_F_OP2_TURN_LEFT_A2 70
#define MOVE_F_OP2_TURN_DISP 450
#define MOVE_F_OP2_TURN_LEFT_DISP 60

#define MOVE_F_OP2_TURN_RIGHT_A1 55 // to calibrate
#define MOVE_F_OP2_TURN_RIGHT_A2 70
#define MOVE_F_OP2_TURN_RIGHT_DISP 60

#define MOVE_F_UTURN_LEFT 180
#define MOVE_F_UTURN_RIGHT 180



// #define MOVE_CAR_OUTER_WHEEL_RADIUS 39.05f
// #define MOVE_CAR_INNER_WHEEL_RADIUS 22.75f

// Move direction inherited from servo.h
// Exposed here as another enum
typedef enum {
	MoveDirLeft = ServoDirLeft,
	MoveDirRight = ServoDirRight,
	MoveDirCenter
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

void move_forward_pid_cm(uint32_t centimeters, bool no_check);
void move_backward_pid_cm(uint32_t centimeters, bool no_check);

void move_turn_forward_by(MoveDirection direction, uint16_t degrees);
void move_turn_backward_by(MoveDirection direction, uint16_t degrees);

void move_turn_forward_pid_degrees(MoveDirection direction, uint16_t degrees, bool no_check);
void move_turn_backward_pid_degrees(MoveDirection direction, uint16_t degrees, bool no_check);

void move_in_place_turn_by(MoveDirection direction, uint16_t degrees);
void move_in_place_turn_cardinal(uint8_t cardinal_direction);

void move_to_obstacle(void);
void move_f_to_obstacle(void);
void move_f_to_carpark(void);

// higher speed code
void move_f_operation_1_fast(uint16_t displacement, MoveDirection dir);
void move_f_operation_2_fast(uint16_t displacement, MoveDirection dir);
void move_f_operation_u_turn_fast(MoveDirection dir);
void move_f_operation_3_fast();

// lower speed code
void move_f_operation_1(uint16_t displacement, MoveDirection dir);
void move_f_operation_2(uint16_t displacement, MoveDirection dir);
void move_f_operation_u_turn(MoveDirection dir);
void move_f_operation_3();

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
