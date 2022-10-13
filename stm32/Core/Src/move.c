/*
 * move.c
 *
 *  Created on: Sep 1, 2022
 *      Author: jimmysqqr
 */

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "math.h"
#include "stdlib.h"

#include "move.h"
#include "servo.h"
#include "motor.h"
#include "ir_adc.h"
#include "encoder.h"
#include "IMU_smooth.h"

#include "globals.h"

// private macros
#define MOVE_DELAY_TICKS 100
// TODO!
//  need a way to read instructions to stop
//  RPi command or boolean isStop() ??
#define TEST_MOVE_TIMER 800
#define MOTOR_ADJUST_SPEED 300 // TODO: to be tested

#define STRAIGHT_FORWARD_CONSTANT 57
#define STRAIGHT_BACKWARD_CONSTANT 55
#define TURN_FORWARD_CONSTANT 44
#define TURN_BACKWARD_CONSTANT 44

// private globals

//static
float MOTOR_INTEGRATION_SUM[2] = {0.0f}; // pid global
static float MOTOR_PREV_ERROR[2] = {0.0f}; // pid global
static float MOTOR_PREV_TICKS[2] = {0.0f}; // pid global

// internal state kept of the current outbound lane
// for the fastest car task
// so that the return trip is fully automated
static MoveDirection MOVE_F_OUTBOUND_LANE = MoveDirCenter;
static uint16_t MOVE_F_OBSTACLE_1_DISPLACEMENT = 0;
static uint16_t MOVE_F_OBSTACLE_2_DISPLACEMENT = 0;

// private function prototypes
void _pid_reset(uint32_t time_ticks);
void _pid_stop(MotorDirection dir, uint32_t speed_mm_s, uint32_t dist_mm);
void _move_in_direction_speed(MotorDirection dir, uint32_t speed_mm_s, uint32_t dist_mm, bool no_stop);

void _move_turn(MoveDirection move_dir, MotorDirection dir, uint32_t speed_mm_s, uint32_t degrees, bool no_stop);
void _move_turn_wide(MoveDirection move_dir, MotorDirection dir, uint32_t speed_mm_s, uint32_t degrees, bool no_stop); //wider turn ver

#define SERVO_FULL_LOCK_DELAY 450
#define DELAY_45 2650
#define DELAY_90 2650
#define DELAY_135 4500
#define DELAY_180 6000

/*
 * Start dynamic movement functions
 * Primary means of movement for car
 */

// Move the car forward by a specified distance, calculated
void move_forward_calc(uint32_t centimeters)
{
	servo_point_center();
	osDelay(100);
	motor_forward(MotorSpeed2);
	osDelay(57 * centimeters);

	motor_stop();
	servo_point_center();
}

// Move the car back by a specified distance, calculated
void move_backward_calc(uint32_t centimeters)
{
	servo_point_center();
	osDelay(100);
	motor_backward(MotorSpeed2);
	osDelay(55 * centimeters);

	motor_stop();
	servo_point_center();
}

/**
 * @brief Reset PID globals and sets the time to current
 *
 * @param time_ticks
 */
void _pid_reset(uint32_t time_ticks) {
	MOTOR_INTEGRATION_SUM[0] = 0.0f;
	MOTOR_INTEGRATION_SUM[1] = 0.0f;
	MOTOR_PREV_ERROR[0] = 0.0f;
	MOTOR_PREV_ERROR[1] = 0.0f;
	MOTOR_PREV_TICKS[0] = time_ticks;
	MOTOR_PREV_TICKS[1] = time_ticks;
}

// gradually bring the car to a stop - in (or less than) 640ms
// Reduces target speed linear
// pid reload freq up to 100 hz
void _pid_stop(MotorDirection dir, uint32_t speed_mm_s, uint32_t dist_mm) {
	uint32_t start_ticks = osKernelGetTickCount();
	uint32_t curr_ticks = 0;
	uint32_t iteration = 0; // out of 25
	uint32_t speed_subtract_amount = speed_mm_s >> 5; // div 32

	volatile uint32_t *ENCODER_LEFT;
	volatile uint32_t *ENCODER_RIGHT;
	uint32_t total_dist = 0;

	// _pid_reset(start_ticks);
	if(dir == MotorDirForward) {
		encoder_reset_counters_forward();
		ENCODER_LEFT = ENCODER_POS_DIRECTIONAL_FORWARD;
		ENCODER_RIGHT = ENCODER_POS_DIRECTIONAL_FORWARD + 1;
	}
	if(dir == MotorDirBackward) {
		encoder_reset_counters_backward();
		ENCODER_LEFT = ENCODER_POS_DIRECTIONAL_BACKWARD;
		ENCODER_RIGHT = ENCODER_POS_DIRECTIONAL_BACKWARD + 1;
	}

	do {
		curr_ticks = osKernelGetTickCount();

		taskENTER_CRITICAL();
		_set_motor_speed_pid(dir, MotorLeft, (uint32_t)speed_mm_s * MOVE_LEFT_MOTOR_MULTIPLIER * 1.025f);
		_set_motor_speed_pid(dir, MotorRight, speed_mm_s);
		total_dist = (*ENCODER_LEFT + *ENCODER_RIGHT) >> 1;
		taskEXIT_CRITICAL();

		speed_mm_s -= speed_subtract_amount; // linear decrease in target speed

		osDelayUntil(curr_ticks + 10); // 64 pid iterations at 100hz
	} while(curr_ticks - start_ticks < 320 || total_dist < dist_mm); // double stop conditions

	motor_stop();
}

// Internal move function
void _move_in_direction_speed(MotorDirection dir, uint32_t speed_mm_s, uint32_t dist_mm, bool no_stop) {
	servo_point_center();
	uint32_t time_ticks = osKernelGetTickCount();
	if(dist_mm < 100) speed_mm_s = speed_mm_s >> 1; // reduce speed for slow runs

	#ifdef MOVE_LOW_GRIP_SURFACE
	uint32_t target = (uint32_t)dist_mm;
	#else
	uint32_t target = (uint32_t)dist_mm;
	#endif

	volatile uint32_t *ENCODER_LEFT;
	volatile uint32_t *ENCODER_RIGHT;
	uint32_t total_dist = 0;

	_pid_reset(time_ticks);
	if(dir == MotorDirForward) {
		encoder_reset_counters_forward();
		ENCODER_LEFT = ENCODER_POS_DIRECTIONAL_FORWARD;
		ENCODER_RIGHT = ENCODER_POS_DIRECTIONAL_FORWARD + 1;
	}
	if(dir == MotorDirBackward) {
		encoder_reset_counters_backward();
		ENCODER_LEFT = ENCODER_POS_DIRECTIONAL_BACKWARD;
		ENCODER_RIGHT = ENCODER_POS_DIRECTIONAL_BACKWARD + 1;
	}

	do {
		time_ticks = osKernelGetTickCount();

		// int wheels_delta = *ENCODER_LEFT;

		taskENTER_CRITICAL();
		_set_motor_speed_pid(dir, MotorLeft, (uint32_t)speed_mm_s * MOVE_LEFT_MOTOR_MULTIPLIER);
		_set_motor_speed_pid(dir, MotorRight, speed_mm_s);
		total_dist = (*ENCODER_LEFT + *ENCODER_RIGHT) >> 1;
		taskEXIT_CRITICAL();
		osDelayUntil(time_ticks + MOVE_PID_LOOP_PERIOD_TICKS);
	} while(total_dist < target); // while still in delay loop

	if(no_stop == false) {motor_stop();}
}

// Internal turn function
void _move_turn(MoveDirection move_dir, MotorDirection dir, uint32_t speed_mm_s, uint32_t degrees, bool no_stop) {
	servo_point(move_dir, ServoMag5);
	if(degrees < 10) speed_mm_s = speed_mm_s >> 1;
	uint32_t time_ticks = osKernelGetTickCount();
	#ifdef MOVE_LOW_GRIP_SURFACE
	uint32_t target_dist_mm = degrees * MOVE_PID_TURN_OUTER_MM_PER_DEGREE * (1 + MOVE_PID_TURN_REDUCTION_FACTOR) * MOVE_PID_SLIP_MULTIPLIER;
	#else
	uint32_t target_dist_mm = degrees * MOVE_PID_TURN_OUTER_MM_PER_DEGREE * (1 + MOVE_PID_TURN_REDUCTION_FACTOR);

	#endif
	uint16_t left_motor_speed = move_dir == MoveDirLeft ? speed_mm_s * MOVE_PID_TURN_REDUCTION_FACTOR : speed_mm_s;
	uint16_t right_motor_speed = move_dir == MoveDirRight ? speed_mm_s * MOVE_PID_TURN_REDUCTION_FACTOR : speed_mm_s;
	volatile uint32_t *ENCODER_LEFT;
	volatile uint32_t *ENCODER_RIGHT;
	uint32_t total_dist = 0;

	_pid_reset(time_ticks);
	if(dir == MotorDirForward) {
		encoder_reset_counters_forward();
		ENCODER_LEFT = ENCODER_POS_DIRECTIONAL_FORWARD;
		ENCODER_RIGHT = ENCODER_POS_DIRECTIONAL_FORWARD + 1;
	}
	if(dir == MotorDirBackward) {
		encoder_reset_counters_backward();
		ENCODER_LEFT = ENCODER_POS_DIRECTIONAL_BACKWARD;
		ENCODER_RIGHT = ENCODER_POS_DIRECTIONAL_BACKWARD + 1;
	}

	if(no_stop == false) {osDelay(SERVO_FULL_LOCK_DELAY);}

	do {
		time_ticks = osKernelGetTickCount();

		taskENTER_CRITICAL();
		_set_motor_speed_pid(dir, MotorLeft, dir == MoveDirRight ? left_motor_speed * MOVE_OUTER_OVERDRIVE_RATIO : left_motor_speed);
		_set_motor_speed_pid(dir, MotorRight, dir == MoveDirLeft ? right_motor_speed * MOVE_OUTER_OVERDRIVE_RATIO : right_motor_speed);
		total_dist = *ENCODER_LEFT + *ENCODER_RIGHT;
		taskEXIT_CRITICAL();

		osDelayUntil(time_ticks + MOVE_PID_LOOP_PERIOD_TICKS);
	} while(total_dist < target_dist_mm);

	if(no_stop == false) {motor_stop();}
}


// Move a specified distance using the encoder
void move_forward_pid_cm(uint32_t centimeters, bool no_check)
{
	_move_in_direction_speed(MotorDirForward, MOVE_DEFAULT_SPEED_STRAIGHT_MM_S, centimeters * 10, no_check);

	if(no_check == false) {
		osDelay(MOVE_PID_LOOP_PERIOD_TICKS << 2);
		int32_t delta_cm = centimeters - (ENCODER_POS_DIRECTIONAL_FORWARD[0] + ENCODER_POS_DIRECTIONAL_FORWARD[1]) / 20;
		if(delta_cm < 0) _move_in_direction_speed(MotorDirBackward, MOVE_DEFAULT_SPEED_STRAIGHT_MM_S >> 1, abs(delta_cm * 10), false); //move_adjust_backward_pos_cm(abs(delta_cm));
		else if(delta_cm > 0) _move_in_direction_speed(MotorDirForward, MOVE_DEFAULT_SPEED_STRAIGHT_MM_S >> 1, delta_cm * 10, false); //move_adjust_forward_pos_cm(delta_cm);
	}
}

// Move a specified distance using the encoder
void move_backward_pid_cm(uint32_t centimeters, bool no_check)
{
	_move_in_direction_speed(MotorDirBackward, MOVE_DEFAULT_SPEED_STRAIGHT_MM_S, (uint32_t)centimeters * 10, false);

	if(no_check == false) {
		osDelay(MOVE_PID_LOOP_PERIOD_TICKS << 2);

		int32_t delta_cm = centimeters - (ENCODER_POS_DIRECTIONAL_BACKWARD[0] + ENCODER_POS_DIRECTIONAL_BACKWARD[1]) / 20;
		if(delta_cm < 0) _move_in_direction_speed(MotorDirForward, MOVE_DEFAULT_SPEED_STRAIGHT_MM_S >> 1, abs(delta_cm * 10), false); //move_adjust_backward_pos_cm(abs(delta_cm));
		else if(delta_cm > 0) _move_in_direction_speed(MotorDirBackward, MOVE_DEFAULT_SPEED_STRAIGHT_MM_S >> 1, delta_cm * 10, false); //move_adjust_forward_pos_cm(delta_cm);
	}
}

/**
 * @brief Uses servo mag 5 for turns (slightly tighter than previous implementation)
 * Turning circle (inner wheel to inner wheel, 180 degrees) approx 46cm
 * @param direction
 * @param degrees
 */
void move_turn_forward_pid_degrees(MoveDirection direction, uint16_t degrees, bool no_check) {
	// IMU_reset_yaw();
	_move_turn(direction, MotorDirForward, MOVE_DEFAULT_SPEED_TURN_MM_S, degrees, no_check);

	if(no_check == false) {
		osDelay(MOVE_PID_LOOP_PERIOD_TICKS);

		uint32_t outer_dist_travelled = direction == MoveDirLeft ? ENCODER_POS_DIRECTIONAL_FORWARD[1] : ENCODER_POS_DIRECTIONAL_FORWARD[0];
		uint32_t target_dist = (uint32_t)MOVE_PID_TURN_OUTER_MM_PER_DEGREE * degrees;

		int32_t delta = target_dist - outer_dist_travelled;
		int32_t delta_deg = delta / MOVE_PID_TURN_OUTER_MM_PER_DEGREE;



		// int32_t delta = (uint32_t) degrees - fabs(IMU_yaw);
		if(delta > 0) {
			_move_turn(direction, MotorDirForward, MOVE_DEFAULT_SPEED_TURN_MM_S >> 1, delta_deg, false);
		}
		if(delta < 0) {
			_move_turn(direction, MotorDirBackward, MOVE_DEFAULT_SPEED_TURN_MM_S >> 1, abs(delta_deg), false);
		}
	}

	servo_point_center();
}

/**
 * @brief Uses servo mag 5 for turns (slightly tighter than previous implenmentation)
 * Turning circle (inner wheel to inner wheel, 180 degrees) approx 46cm
 * @param direction
 * @param degrees
 */
void move_turn_backward_pid_degrees(MoveDirection direction, uint16_t degrees, bool no_check) {
	_move_turn(direction, MotorDirBackward, MOVE_DEFAULT_SPEED_TURN_MM_S, degrees, no_check);

	if(no_check == false) {
		osDelay(MOVE_PID_LOOP_PERIOD_TICKS);
		uint32_t outer_dist_travelled = direction == MoveDirLeft ? ENCODER_POS_DIRECTIONAL_BACKWARD[1] : ENCODER_POS_DIRECTIONAL_BACKWARD[0];
		uint32_t target_dist = (uint32_t)MOVE_PID_TURN_OUTER_MM_PER_DEGREE * degrees * MOVE_PID_SLIP_MULTIPLIER;
		int32_t delta = target_dist - outer_dist_travelled;
		int32_t delta_deg = delta / MOVE_PID_TURN_OUTER_MM_PER_DEGREE;

		// int32_t delta = (uint32_t) degrees - fabs(IMU_yaw);
		if(delta > 0) {
			_move_turn(direction, MotorDirBackward, MOVE_DEFAULT_SPEED_TURN_MM_S >> 1, delta_deg, false);
		}
		if(delta < 0) {
			_move_turn(direction, MotorDirForward, MOVE_DEFAULT_SPEED_TURN_MM_S >> 1, abs(delta_deg), false);
		}
	}

	servo_point_center();
}

// Move the car forward AND turn it to a specified angle
void move_turn_forward_by(MoveDirection direction, uint16_t degrees)
{
	servo_point(direction, ServoMag4);
	osDelay(SERVO_FULL_LOCK_DELAY);
	motor_forward(MotorSpeed2);
	osDelay(46 * degrees);
	motor_stop();
	servo_point_center();
}

// Move the car backward AND turn it to a specified angle
void move_turn_backward_by(MoveDirection direction, uint16_t degrees)
{
	servo_point(direction, ServoMag4);
	osDelay(SERVO_FULL_LOCK_DELAY);
	motor_backward(MotorSpeed2);
	osDelay(46 * degrees);
	motor_stop();
	servo_point_center();
}

// Perform an in-place turn, starting with a direction (forward/backward)
//
void move_in_place_turn_by(MoveDirection direction, uint16_t degrees)
{
}

// Perform an in place turn by a multiple of 22.5 degrees, 0-indexed
// Starts by moving backwards first
// E.g. cardinal_direction=4 performs a 90 degree in-place turn, clockwise
// E.g. cardinal_direction=12 performs a 90 degree in-place turn, anticlockwise
void move_in_place_turn_cardinal(uint8_t cardinal_direction)
{
	// convert clockwise multiples of 22.5 degrees
	// to pos-neg multiples of 22.5 degrees (-8 to +7)
	cardinal_direction = cardinal_direction % 16; // wrap to 0-15
	// actual turn action, -8 to +7
	int8_t num_turns = cardinal_direction > 7 ? 16 - cardinal_direction : cardinal_direction;
	// exec clockwise (1) or anticlockwise (0)s
	uint8_t clockwise = cardinal_direction <= 7 ? 1 : 0; // clockwise if less than 7

	MoveDirection rev_dir = clockwise ? MoveDirLeft : MoveDirRight;
	MoveDirection for_dir = clockwise ? MoveDirRight : MoveDirLeft;

	if(cardinal_direction == 4 || cardinal_direction == 12) {
        move_forward_pid_cm(25, false);
        // move_forward_pid_cm(5);
		osDelay(MOVE_PID_LOOP_PERIOD_TICKS);

        for (uint8_t i = 0; i < 2; i++)
        {
            move_turn_backward_pid_degrees(rev_dir, 22, false);
            osDelay(MOVE_PID_LOOP_PERIOD_TICKS);
            move_turn_forward_pid_degrees(for_dir, 22, false);
            osDelay(MOVE_PID_LOOP_PERIOD_TICKS);
        }

        // move_forward_pid_cm(10);
		// osDelay(MOVE_PID_LOOP_PERIOD_TICKS);

        // for (uint8_t i = 0; i < 2; i++)
        // {
        //     move_turn_backward_pid_degrees(rev_dir, 11);
        //     osDelay(MOVE_PID_LOOP_PERIOD_TICKS);
        //     move_turn_forward_pid_degrees(for_dir, 11);
        //     osDelay(MOVE_PID_LOOP_PERIOD_TICKS);
        // }

        move_backward_pid_cm(20, false);
		// osDelay(MOVE_PID_LOOP_PERIOD_TICKS);
        return;
    }

	for (uint8_t i = 0; i < num_turns; i++)
	{
		move_turn_backward_pid_degrees(rev_dir, 11, false);
		osDelay(MOVE_PID_LOOP_PERIOD_TICKS);
		move_turn_forward_pid_degrees(for_dir, 11, false);
		osDelay(MOVE_PID_LOOP_PERIOD_TICKS);
	}

	// if(cardinal_direction == 4 || cardinal_direction == 12) move_backward_pid_cm(10);
}

// Move to minimum forward clearance from obstacle
// Calculated to be 25cm, approx 1 car length away
void move_to_obstacle(void)
{
	int32_t ir_diff = 0;
	int32_t ticks = 0;
	MotorDirection dir;
	do
	{
		ticks = osKernelGetTickCount();
		taskENTER_CRITICAL();
		ir_diff = (int32_t)(1250 - IR_ADC_AVERAGE_READOUT);
		taskEXIT_CRITICAL();

		// dir = ir_diff > 0 ? MotorDirForward : MotorDirBackward;
		// _set_motor_speed_pid(dir, MotorLeft, MOVE_DEFAULT_SPEED_STRAIGHT_MM_S >> 1);
		// _set_motor_speed_pid(dir, MotorRight, MOVE_DEFAULT_SPEED_STRAIGHT_MM_S >> 1);

		if (ir_diff > 0)
		{
			motor_forward(MotorSpeed1);
		}
		else
		{
			motor_backward(MotorSpeed1);
		}
		// osDelayUntil(ticks + 25);
		osDelayUntil(ticks + MOVE_PID_LOOP_PERIOD_TICKS);
	} while (abs(ir_diff) > 20);

	motor_stop();
}

// sets the motor speed for one motor for one PID iteration
void _set_motor_speed_pid(MotorDirection dir, MotorSide side, uint16_t speed_mm_s) {
	float current_error = (float)(speed_mm_s - abs(ENCODER_SPEED_DIRECTIONAL[side])); // target - current
	uint32_t current_time = osKernelGetTickCount();
	uint32_t iteration_ticks = 0;
	uint32_t new_pwm_val = 0;

	iteration_ticks = (uint32_t)(current_time - MOTOR_PREV_TICKS[side]);

	MOTOR_INTEGRATION_SUM[side] += (float)(current_error * iteration_ticks / 1000);

	new_pwm_val = (uint32_t)(
			MOVE_KP * current_error +
			MOVE_KI * MOTOR_INTEGRATION_SUM[side] +
			MOVE_KD * 1000 * (current_error - MOTOR_PREV_ERROR[side]) / iteration_ticks
	);

	MOTOR_PREV_ERROR[side] = current_error;
	MOTOR_PREV_TICKS[side] = current_time;

	_motor_set_pwm(dir, side, new_pwm_val);
}

// _set_motor_first_pwm_val(MotorDirection dir, MotorSide side, uint16_t speed_mm_s) {

// }

#define MOVE_PID_TURN_OUTER_MM_PER_DEGREE_WIDE 10.3f
#define MOVE_PID_TURN_REDUCTION_FACTOR_WIDE 0.751f



// move fastest task stuff
void _move_turn_wide(MoveDirection move_dir, MotorDirection dir, uint32_t speed_mm_s, uint32_t degrees, bool no_stop) {
	servo_point(move_dir, ServoMag3); // check the turn radius for this
	if(degrees < 10) speed_mm_s = speed_mm_s >> 1;
	uint32_t time_ticks = osKernelGetTickCount();
	uint32_t target_dist_mm = degrees * MOVE_PID_TURN_OUTER_MM_PER_DEGREE_WIDE * (1 + MOVE_PID_TURN_REDUCTION_FACTOR_WIDE);
	uint16_t left_motor_speed = move_dir == MoveDirLeft ? speed_mm_s * MOVE_PID_TURN_REDUCTION_FACTOR_WIDE : speed_mm_s;
	uint16_t right_motor_speed = move_dir == MoveDirRight ? speed_mm_s * MOVE_PID_TURN_REDUCTION_FACTOR_WIDE : speed_mm_s;
	volatile uint32_t *ENCODER_LEFT;
	volatile uint32_t *ENCODER_RIGHT;
	uint32_t total_dist = 0;

	_pid_reset(time_ticks);
	if(dir == MotorDirForward) {
		encoder_reset_counters_forward();
		ENCODER_LEFT = ENCODER_POS_DIRECTIONAL_FORWARD;
		ENCODER_RIGHT = ENCODER_POS_DIRECTIONAL_FORWARD + 1;
	}
	if(dir == MotorDirBackward) {
		encoder_reset_counters_backward();
		ENCODER_LEFT = ENCODER_POS_DIRECTIONAL_BACKWARD;
		ENCODER_RIGHT = ENCODER_POS_DIRECTIONAL_BACKWARD + 1;
	}

	if(no_stop == false) {osDelay(SERVO_FULL_LOCK_DELAY);}

	do {
		time_ticks = osKernelGetTickCount();

		taskENTER_CRITICAL();
		_set_motor_speed_pid(dir, MotorLeft, dir == MoveDirRight ? left_motor_speed * MOVE_OUTER_OVERDRIVE_RATIO : left_motor_speed);
		_set_motor_speed_pid(dir, MotorRight, dir == MoveDirLeft ? right_motor_speed * MOVE_OUTER_OVERDRIVE_RATIO : right_motor_speed);
		total_dist = *ENCODER_LEFT + *ENCODER_RIGHT;
		taskEXIT_CRITICAL();

		osDelayUntil(time_ticks + MOVE_PID_LOOP_PERIOD_TICKS);
	} while(total_dist < target_dist_mm);

	if(no_stop == false) {motor_stop();}
}


// Moves to the obstacle, 50cm away
void move_f_to_obstacle(void) {
	servo_point_center();

	uint16_t TARGET_IR_READOUT = 750; // ir output at 50cm - calibrate!

	uint32_t time_ticks = osKernelGetTickCount();
	// uint32_t target = (uint32_t)dist_mm;
	volatile uint32_t *ENCODER_LEFT;
	volatile uint32_t *ENCODER_RIGHT;
	uint32_t total_dist = 0;
	int32_t ir_diff = 0;

	_pid_reset(time_ticks);
	encoder_reset_counters_forward();
	ENCODER_LEFT = ENCODER_POS_DIRECTIONAL_FORWARD;
	ENCODER_RIGHT = ENCODER_POS_DIRECTIONAL_FORWARD + 1;

	do {
		time_ticks = osKernelGetTickCount();

		ir_diff = (int32_t)(TARGET_IR_READOUT - IR_ADC_AVERAGE_READOUT);

		if(ir_diff < 0) {
			taskENTER_CRITICAL();
			_set_motor_speed_pid(MotorDirBackward, MotorLeft, (uint32_t)MOVE_DEFAULT_SPEED_STRAIGHT_MM_S * MOVE_LEFT_MOTOR_MULTIPLIER);
			_set_motor_speed_pid(MotorDirBackward, MotorRight, MOVE_DEFAULT_SPEED_STRAIGHT_MM_S);
			// total_dist = (*ENCODER_LEFT + *ENCODER_RIGHT) >> 1;
			taskEXIT_CRITICAL();

		} else if(ir_diff > 0) {
			taskENTER_CRITICAL();
			_set_motor_speed_pid(MotorDirForward, MotorLeft, (uint32_t)MOVE_DEFAULT_SPEED_STRAIGHT_MM_S * MOVE_LEFT_MOTOR_MULTIPLIER);
			_set_motor_speed_pid(MotorDirForward, MotorRight, MOVE_DEFAULT_SPEED_STRAIGHT_MM_S);
			// total_dist = (*ENCODER_LEFT + *ENCODER_RIGHT) >> 1;
			taskEXIT_CRITICAL();

		}

		osDelayUntil(time_ticks + MOVE_PID_LOOP_PERIOD_TICKS);
	} while(abs(ir_diff) > 20); // while still in delay loop

	motor_stop();
}

// move inside the carpark
void move_f_to_carpark(void) {
	uint32_t TARGET_IR_READOUT = 3000;
	int32_t ir_diff = 0;
	int32_t ticks = 0;
	MotorDirection dir;
	do
	{
		ticks = osKernelGetTickCount();
		taskENTER_CRITICAL();
		ir_diff = (int32_t)(TARGET_IR_READOUT - IR_ADC_AVERAGE_READOUT);
		taskEXIT_CRITICAL();

		// dir = ir_diff > 0 ? MotorDirForward : MotorDirBackward;
		// _set_motor_speed_pid(dir, MotorLeft, MOVE_DEFAULT_SPEED_STRAIGHT_MM_S >> 1);
		// _set_motor_speed_pid(dir, MotorRight, MOVE_DEFAULT_SPEED_STRAIGHT_MM_S >> 1);

		if (ir_diff > 0)
		{
			motor_forward(MotorSpeed1);
		}
		else
		{
			motor_backward(MotorSpeed1);
		}
		// osDelayUntil(ticks + 25);
		osDelayUntil(ticks + MOVE_PID_LOOP_PERIOD_TICKS);
	} while (abs(ir_diff) > 20);

	motor_stop();
}



// high speed version of week 9 challenge
// small lane change operation - faster version
// change lane 25cm to the left/right
void move_f_operation_1_fast(uint16_t displacement, MoveDirection dir) {
	if(dir != MoveDirCenter)  {
		MOVE_F_OUTBOUND_LANE = dir;
	}
	MOVE_F_OBSTACLE_1_DISPLACEMENT = displacement; // store the obstacle displacement

	uint32_t forward_dist = (uint32_t)displacement * 0.98f - 44;
	switch(dir) {
		case MoveDirLeft:
			// move_turn_forward_pid_degrees(dir, 42, true);
			_move_turn(dir, MotorDirForward, MOVE_HIGH_SPEED_TURN_MM_S, MOVE_F_OP1_FAST_TURN_LEFT_A1, true);
			// osDelay(50);
			// move_turn_forward_pid_degrees(dir == MoveDirLeft ? MoveDirRight : MoveDirLeft, 41, true);
			_move_turn(dir == MoveDirLeft ? MoveDirRight : MoveDirLeft, MotorDirForward, MOVE_HIGH_SPEED_TURN_MM_S, MOVE_F_OP1_FAST_TURN_LEFT_A2, true);

			// move_forward_pid_cm((uint32_t)forward_dist * 0.9f, true); // multiplier accounts for "wheel slip"
			_move_in_direction_speed(MotorDirForward, MOVE_HIGH_SPEED_STRAIGHT_MM_S, forward_dist * 9, true);
			_pid_stop(MotorDirForward, MOVE_DEFAULT_SPEED_STRAIGHT_MM_S, forward_dist);
			break;

		case MoveDirRight:
			// move_turn_forward_pid_degrees(dir, 35, true);
			_move_turn(dir, MotorDirForward, MOVE_HIGH_SPEED_TURN_MM_S, MOVE_F_OP1_FAST_TURN_RIGHT_A1, true);

			// move_turn_forward_pid_degrees(dir == MoveDirLeft ? MoveDirRight : MoveDirLeft, 45, true);
			_move_turn(dir == MoveDirLeft ? MoveDirRight : MoveDirLeft, MotorDirForward, MOVE_HIGH_SPEED_TURN_MM_S, MOVE_F_OP1_FAST_TURN_RIGHT_A2, true);

			// move_forward_pid_cm((uint32_t)forward_dist * 0.9f, true); // multiplier accounts for "wheel slip"
			_move_in_direction_speed(MotorDirForward, MOVE_HIGH_SPEED_STRAIGHT_MM_S, forward_dist * 9, true);
			_pid_stop(MotorDirForward, MOVE_DEFAULT_SPEED_STRAIGHT_MM_S, forward_dist);
			break;

		default: break;
	}
}

// big lane change operation - faster version
void move_f_operation_2_fast(uint16_t displacement, MoveDirection dir) {
	uint32_t forward_dist = displacement - 30;
	MOVE_F_OBSTACLE_2_DISPLACEMENT = displacement; // store the obstacle displacment

	if(dir == MOVE_F_OUTBOUND_LANE || dir == MoveDirCenter) { // if already in the lane, do a small lane change
		move_f_operation_1_fast(displacement, dir); // change lane again by 25cm
		return;

	} else { // if in the wrong lane, do a big lane change (75cm)

		switch(dir) {
			case MoveDirLeft: // 75cm left lane change
				// move_turn_forward_pid_degrees(dir, 55, true);
				_move_turn(dir, MotorDirForward, MOVE_HIGH_SPEED_TURN_MM_S, MOVE_F_OP2_FAST_TURN_LEFT_A1, true);

				// move_forward_pid_cm(35, true);
				_move_in_direction_speed(MotorDirForward, MOVE_HIGH_SPEED_TURN_MM_S, MOVE_F_OP2_FAST_TURN_DISP, true);

				// move_turn_forward_pid_degrees(dir == MoveDirLeft ? MoveDirRight : MoveDirLeft, 70, true);
				_move_turn(dir == MoveDirLeft ? MoveDirRight : MoveDirLeft, MotorDirForward, MOVE_HIGH_SPEED_TURN_MM_S, MOVE_F_OP2_FAST_TURN_LEFT_A2, true);

				// motor_stop();
				// servo_point_center();
				move_forward_pid_cm((uint16_t)(displacement - MOVE_F_OP2_FAST_TURN_LEFT_DISP), false);
				break;

			case MoveDirRight: // 75 cm right lane change
				// move_turn_forward_pid_degrees(dir, 55, true);
				_move_turn(dir, MotorDirForward, MOVE_HIGH_SPEED_TURN_MM_S, MOVE_F_OP2_FAST_TURN_RIGHT_A1, true);

				// move_forward_pid_cm(35, true);
				_move_in_direction_speed(MotorDirForward, MOVE_HIGH_SPEED_TURN_MM_S, 350, true);

				// move_turn_forward_pid_degrees(dir == MoveDirLeft ? MoveDirRight : MoveDirLeft, 70, true);
				_move_turn(dir == MoveDirLeft ? MoveDirRight : MoveDirLeft, MotorDirForward, MOVE_HIGH_SPEED_TURN_MM_S, MOVE_F_OP2_FAST_TURN_RIGHT_A2, true);

				// motor_stop();
				// servo_point_center();
				move_forward_pid_cm((uint16_t)(displacement - MOVE_F_OP2_FAST_TURN_RIGHT_DISP), false);
			break;

			default: break;
		}
	}

	MOVE_F_OUTBOUND_LANE = dir;
}

void move_f_operation_u_turn_fast(MoveDirection dir) {

	switch(dir) {
		case MoveDirLeft:
			_move_turn_wide(dir, MotorDirForward, MOVE_DEFAULT_SPEED_TURN_MM_S, MOVE_F_UTURN_FAST_LEFT, true);
			break;

		case MoveDirRight:
			_move_turn_wide(dir, MotorDirForward, MOVE_DEFAULT_SPEED_TURN_MM_S, MOVE_F_UTURN_FAST_RIGHT, true);
			break;
	}
}

// approach, u-turn and return trip - faster version
void move_f_operation_3_fast() {
	// homebound lane is opposite of the outbound lane, from start point perspective
	MoveDirection move_homebound_lane = MOVE_F_OUTBOUND_LANE == MoveDirLeft ? MoveDirRight : MoveDirLeft;
	// point back to center of carpark lot, and send it
	uint16_t carpark_angle = (uint16_t) tanh((float) 60/MOVE_F_OBSTACLE_1_DISPLACEMENT);
	uint16_t carpark_dist = (uint16_t) MOVE_F_OBSTACLE_1_DISPLACEMENT / cos(carpark_angle); // need to verify

	// u-turn
	move_f_operation_u_turn_fast(move_homebound_lane);
	// obstacle 1
	_move_in_direction_speed(MotorDirForward, MOVE_HIGH_SPEED_STRAIGHT_MM_S, MOVE_F_OBSTACLE_2_DISPLACEMENT * 5, true);
	_move_in_direction_speed(MotorDirForward, MOVE_DEFAULT_SPEED_TURN_MM_S, MOVE_F_OBSTACLE_2_DISPLACEMENT * 5, true);
	// go home
	move_turn_forward_pid_degrees(move_homebound_lane, carpark_angle << 1, true); // fixed, should be using carpark_angle
	move_forward_pid_cm((uint32_t)carpark_dist * 0.9f, true);
	_pid_stop(MotorDirForward, MOVE_DEFAULT_SPEED_STRAIGHT_MM_S, carpark_dist);

	// park in front (disabled for now)
	// move_f_to_obstacle();
}



// normal speed version of week 9 challenge
// small lane change operation - faster version


void move_f_operation_1(uint16_t displacement, MoveDirection dir) {
	if(dir != MoveDirCenter)  {
		MOVE_F_OUTBOUND_LANE = dir;
	}
	MOVE_F_OBSTACLE_1_DISPLACEMENT = displacement; // store the obstacle displacement

	uint32_t forward_dist = (uint32_t)displacement * 0.98f - 44;
	switch(dir) {
		case MoveDirLeft:
			// move_turn_forward_pid_degrees(dir, 42, true);
			_move_turn(dir, MotorDirForward, MOVE_HIGH_SPEED_SLOW_TURN_MM_S, MOVE_F_OP1_TURN_LEFT_A1, true);
			// osDelay(50);
			// move_turn_forward_pid_degrees(dir == MoveDirLeft ? MoveDirRight : MoveDirLeft, 41, true);
			_move_turn(dir == MoveDirLeft ? MoveDirRight : MoveDirLeft, MotorDirForward, MOVE_HIGH_SPEED_SLOW_TURN_MM_S, MOVE_F_OP1_TURN_LEFT_A2, true);

			// move_forward_pid_cm((uint32_t)forward_dist * 0.9f, true); // multiplier accounts for "wheel slip"
			_move_in_direction_speed(MotorDirForward, MOVE_HIGH_SPEED_SLOW_STRAIGHT_MM_S, forward_dist * 9, true);
			_pid_stop(MotorDirForward, MOVE_HIGH_SPEED_SLOW_STRAIGHT_MM_S, forward_dist);
			break;

		case MoveDirRight:
			// move_turn_forward_pid_degrees(dir, 35, true);
			_move_turn(dir, MotorDirForward, MOVE_HIGH_SPEED_SLOW_TURN_MM_S, MOVE_F_OP1_TURN_RIGHT_A1, true);

			// move_turn_forward_pid_degrees(dir == MoveDirLeft ? MoveDirRight : MoveDirLeft, 45, true);
			_move_turn(dir == MoveDirLeft ? MoveDirRight : MoveDirLeft, MotorDirForward, MOVE_HIGH_SPEED_SLOW_TURN_MM_S, MOVE_F_OP1_TURN_RIGHT_A2, true);

			// move_forward_pid_cm((uint32_t)forward_dist * 0.9f, true); // multiplier accounts for "wheel slip"
			_move_in_direction_speed(MotorDirForward, MOVE_HIGH_SPEED_SLOW_STRAIGHT_MM_S, forward_dist * 9, true);
			_pid_stop(MotorDirForward, MOVE_HIGH_SPEED_SLOW_STRAIGHT_MM_S, forward_dist);
			break;

		default: break;
	}
}

// big lane change operation - faster version
void move_f_operation_2(uint16_t displacement, MoveDirection dir) {
	uint32_t forward_dist = displacement - 30;
	MOVE_F_OBSTACLE_2_DISPLACEMENT = displacement; // store the obstacle displacment
	MOVE_F_OUTBOUND_LANE = dir;

	if(dir == MOVE_F_OUTBOUND_LANE || dir == MoveDirCenter) { // if already in the lane, do a small lane change
		move_f_operation_1(displacement, dir); // change lane again by 25cm
		return;

	} else { // if in the wrong lane, do a big lane change (75cm)

		switch(dir) {
			case MoveDirLeft: // 75cm left lane change
				// move_turn_forward_pid_degrees(dir, 55, true);
				_move_turn(dir, MotorDirForward, MOVE_HIGH_SPEED_SLOW_TURN_MM_S, MOVE_F_OP2_TURN_LEFT_A1, true);

				// move_forward_pid_cm(35, true);
				_move_in_direction_speed(MotorDirForward, MOVE_HIGH_SPEED_SLOW_TURN_MM_S, 350, true);

				// move_turn_forward_pid_degrees(dir == MoveDirLeft ? MoveDirRight : MoveDirLeft, 70, true);
				_move_turn(dir == MoveDirLeft ? MoveDirRight : MoveDirLeft, MotorDirForward, MOVE_HIGH_SPEED_SLOW_TURN_MM_S, MOVE_F_OP2_TURN_LEFT_A2, true);

				// motor_stop();
				// servo_point_center();
				move_forward_pid_cm((uint16_t)(displacement - MOVE_F_OP2_TURN_LEFT_DISP), false);
				// _move_in_direction_speed(MotorDirForward, MOVE_HIGH_SPEED_SLOW_TURN_MM_S, displacement - MOVE_F_OP2_TURN_LEFT_DISP, true);
				break;

			case MoveDirRight: // 75 cm right lane change
				// move_turn_forward_pid_degrees(dir, 55, true);
				_move_turn(dir, MotorDirForward, MOVE_HIGH_SPEED_SLOW_TURN_MM_S, MOVE_F_OP2_TURN_RIGHT_A1, true);

				// move_forward_pid_cm(35, true);
				_move_in_direction_speed(MotorDirForward, MOVE_HIGH_SPEED_SLOW_TURN_MM_S, 350, true);

				// move_turn_forward_pid_degrees(dir == MoveDirLeft ? MoveDirRight : MoveDirLeft, 70, true);
				_move_turn(dir == MoveDirLeft ? MoveDirRight : MoveDirLeft, MotorDirForward, MOVE_HIGH_SPEED_SLOW_TURN_MM_S, MOVE_F_OP2_TURN_RIGHT_A2, true);

				// motor_stop();
				// servo_point_center();
				move_forward_pid_cm((uint16_t)(displacement - MOVE_F_OP2_TURN_RIGHT_DISP), false);
			break;

			default: break;
		}
	}
}

void move_f_operation_u_turn(MoveDirection dir) {

	switch(dir) {
		case MoveDirLeft:
			_move_turn_wide(dir, MotorDirForward, MOVE_HIGH_SPEED_SLOW_TURN_MM_S, MOVE_F_UTURN_LEFT, true);
			break;

		case MoveDirRight:
			_move_turn_wide(dir, MotorDirForward, MOVE_HIGH_SPEED_SLOW_TURN_MM_S, MOVE_F_UTURN_RIGHT, true);
			break;
	}
}



// approach, u-turn and return trip - faster version
void move_f_operation_3() {
	// homebound lane is opposite of the outbound lane, from start point perspective
	MoveDirection move_homebound_lane = MOVE_F_OUTBOUND_LANE == MoveDirLeft ? MoveDirRight : MoveDirLeft;
	// point back to center of carpark lot, and send it
	uint16_t carpark_angle = (uint16_t) tanh((float) 60/MOVE_F_OBSTACLE_1_DISPLACEMENT);
	uint16_t carpark_dist = (uint16_t) MOVE_F_OBSTACLE_1_DISPLACEMENT / cos(carpark_angle); // need to verify

	// u-turn
	move_f_operation_u_turn(move_homebound_lane);
	// obstacle 1
	_move_in_direction_speed(MotorDirForward, MOVE_HIGH_SPEED_SLOW_STRAIGHT_MM_S, MOVE_F_OBSTACLE_2_DISPLACEMENT * 5, true);
	_move_in_direction_speed(MotorDirForward, MOVE_HIGH_SPEED_SLOW_TURN_MM_S, MOVE_F_OBSTACLE_2_DISPLACEMENT * 5, true);
	// go home
	move_turn_forward_pid_degrees(move_homebound_lane, carpark_angle << 1, true); // fixed, should be using carpark_angle
	move_forward_pid_cm((uint32_t)carpark_dist * 0.9f, true);
	_pid_stop(MotorDirForward, MOVE_HIGH_SPEED_SLOW_TURN_MM_S, carpark_dist);

	// park in front (disabled for now)
	// move_f_to_obstacle();
}


/*
 * Start of hardcoded movement functions
 * Note that car movement will vary depending on surface conditions.
 */

void move_hard_left_45()
{
	servo_point_left_full();
	osDelay(SERVO_FULL_LOCK_DELAY);
	motor_forward(MotorSpeed1);
	osDelay(DELAY_45);
	motor_stop();
	servo_point_center();
}

void move_hard_right_45()
{
	servo_point_right_full();
	osDelay(SERVO_FULL_LOCK_DELAY);
	motor_forward(MotorSpeed2);
	osDelay(DELAY_45);
	motor_stop();
	servo_point_center();
}

// Move and turn 90 degrees to the left
void move_hard_left_90()
{
	servo_point_left_full();
	osDelay(SERVO_FULL_LOCK_DELAY);
	motor_forward(MotorSpeed2);
	osDelay(DELAY_90);
	motor_stop();
	servo_point_center();
}
void move_hard_right_90()
{
	servo_point_right_full();
	osDelay(SERVO_FULL_LOCK_DELAY);
	motor_forward(MotorSpeed2);
	osDelay(DELAY_90);
	motor_stop();
	servo_point_center();
}
void move_hard_left_135()
{
	servo_point_left_full();
	osDelay(SERVO_FULL_LOCK_DELAY);
	motor_forward(MotorSpeed2);
	osDelay(DELAY_135);
	motor_stop();
	servo_point_center();
}
void move_hard_right_135()
{
	servo_point_right_full();
	osDelay(SERVO_FULL_LOCK_DELAY);
	motor_forward(MotorSpeed2);
	osDelay(DELAY_135);
	motor_stop();
	servo_point_center();
}
void move_hard_left_180()
{
	servo_point_left_full();
	osDelay(SERVO_FULL_LOCK_DELAY);
	motor_forward(MotorSpeed2);
	osDelay(DELAY_180);
	motor_stop();
	servo_point_center();
}
void move_hard_right_180()
{
	servo_point_right_full();
	osDelay(SERVO_FULL_LOCK_DELAY);
	motor_forward(MotorSpeed2);
	osDelay(DELAY_180);
	motor_stop();
	servo_point_center();
}
