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


// /*
//  * Global struct containing the current effector states
//  */
// static struct global_state
// {
//     uint32_t servo;
//     uint32_t motor_left;
//     uint32_t motor_right;
// } GLOBAL_EFFECTOR_STATE = {
//     .servo = SERVO_CENTER, // move SERVO_CENTER out of private namespace and use it here
//     .motor_left = 0,
//     .motor_right = 0};



// private function prototypes
void _pid_reset(uint32_t time_ticks);

void _adjust_forward(MotorSpeed speed);
void _adjust_backward(MotorSpeed speed);

/**
 * (Calibration) Adjust forward after moving backward
 */
void _adjust_forward(MotorSpeed speed)
{
	//	steer(dir, mag);
	//	osDelay(MOVE_DELAY_TICKS);
	forward(speed);
}

/**
 * (Calibration) Adjust backward after moving forward
 */
void _adjust_backward(MotorSpeed speed)
{
	//	steer(dir, mag);
	//	osDelay(MOVE_DELAY_TICKS);
	backward(speed);
}

/**
 * Sequence of moves to visually check that the motors and servo are functioning together
 */
void move_test_startup()
{

	// isolated moves
	forward(MotorSpeed1);
	stop();
	_adjust_backward(MOTOR_ADJUST_SPEED);
	stop();
	osDelay(100);

	backward(MotorSpeed1);
	stop();
	_adjust_forward(MOTOR_ADJUST_SPEED);
	stop();
	osDelay(100);

	steer(ServoDirLeft, ServoMag1);
	osDelay(100);

	steer(ServoDirRight, ServoMag1);
	osDelay(100);

	steer_straight();
	osDelay(300);

	// combined moves
	forward_left(MotorSpeed1, ServoDirLeft, ServoMag1);
	osDelay(300);

	backward_left(MotorSpeed1, ServoDirLeft, ServoMag1);
	osDelay(300);

	forward_right(MotorSpeed1, ServoDirRight, ServoMag1);
	osDelay(300);

	backward_right(MotorSpeed1, ServoDirRight, ServoMag1);
	osDelay(300);
}

/**
 * Go through all motor & servo combined movements
 */
void move_showcase()
{
	// TODO!
}

/** Isolated moveset functions */

/**
 * (Motor only) Move forward at a specified speed
 */
void forward(MotorSpeed speed)
{ // assume needs calibration
	motor_forward(speed);
}

/**
 * (Motor only) Move backward at a specified speed
 */
void backward(MotorSpeed speed)
{ // assume needs calibration
	motor_backward(speed);
}

/**
 * (Motor only) Stop moving
 */
void stop()
{
	motor_stop();
}

/**
 * (Servo only) Turn the wheels in a certain direction and magnitude
 */
void steer(ServoDirection dir, ServoMagnitude mag)
{
	servo_point(dir, mag);
}

/**
 * (Servo only) Turn the wheels straight
 */
void steer_straight()
{
	servo_point_center();
}

/** Combined moveset functions */

/**
 * (Motor & servo) Turn the wheels to an angled left and move forward
 */
void forward_left(MotorSpeed speed, ServoDirection dir, ServoMagnitude mag)
{
	steer(dir, mag);
	osDelay(MOVE_DELAY_TICKS);
	forward(speed);
}

/**
 * (Motor & servo) Turn the wheels to an angled right and move forward
 */
void forward_right(MotorSpeed speed, ServoDirection dir, ServoMagnitude mag)
{
	steer(dir, mag);
	osDelay(MOVE_DELAY_TICKS);
	forward(speed);
}

/**
 * (Motor & servo) Turn the wheels to an angled left and move backward
 */
void backward_left(MotorSpeed speed, ServoDirection dir, ServoMagnitude mag)
{
	steer(dir, mag);
	osDelay(MOVE_DELAY_TICKS);
	backward(speed);
}

/**
 * (Motor & servo) Turn the wheels to an angled right and move backward
 */
void backward_right(MotorSpeed speed, ServoDirection dir, ServoMagnitude mag)
{
	steer(dir, mag);
	osDelay(MOVE_DELAY_TICKS);
	backward(speed);
}

#define SERVO_FULL_LOCK_DELAY 350
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

// resets pid stuff
void _pid_reset(uint32_t time_ticks) {
	MOTOR_INTEGRATION_SUM[0] = 0.0f;
	MOTOR_INTEGRATION_SUM[1] = 0.0f;
	MOTOR_PREV_ERROR[0] = 0.0f;
	MOTOR_PREV_ERROR[1] = 0.0f;
	MOTOR_PREV_TICKS[0] = time_ticks;
	MOTOR_PREV_TICKS[1] = time_ticks;
}

// Move a specified distance using the encoder
void move_forward_pid_cm(uint32_t centimeters)
{
	uint32_t time_ticks = osKernelGetTickCount();
	uint32_t end_ticks = time_ticks + 10000 * centimeters / MOVE_DEFAULT_SPEED_MM_S;
	_pid_reset(time_ticks);
	// int16_t sum_dist = 0;

	encoder_reset_counters_forward();
	servo_point_center();

	do {
		time_ticks = osKernelGetTickCount();

		taskENTER_CRITICAL();
		_set_motor_speed_pid(MotorDirForward, MotorLeft, MOVE_DEFAULT_SPEED_MM_S);
		_set_motor_speed_pid(MotorDirForward, MotorRight, MOVE_DEFAULT_SPEED_MM_S);
		taskEXIT_CRITICAL();
		// sum_dist += (ENCODER_SPEED_DIRECTIONAL[0] + ENCODER_SPEED_DIRECTIONAL[1]) * MOVE_PID_LOOP_PERIOD_TICKS / 2000;
		osDelayUntil(time_ticks + MOVE_PID_LOOP_PERIOD_TICKS);
	} while(((ENCODER_POS_DIRECTIONAL[0] + ENCODER_POS_DIRECTIONAL[1]) / 20) < centimeters); // while still in delay loop

	motor_stop();
}

// Move a specified distance using the encoder
void move_backward_pid_cm(uint32_t centimeters)
{
	uint32_t time_ticks = osKernelGetTickCount();
	uint32_t end_ticks = time_ticks + 10000 * centimeters / MOVE_DEFAULT_SPEED_MM_S;
	_pid_reset(time_ticks);

	encoder_reset_counters_forward();
	servo_point_center();

	do {
		time_ticks = osKernelGetTickCount();

		taskENTER_CRITICAL();
		_set_motor_speed_pid(MotorDirBackward, MotorLeft, MOVE_DEFAULT_SPEED_MM_S);
		_set_motor_speed_pid(MotorDirBackward, MotorRight, MOVE_DEFAULT_SPEED_MM_S);
		taskEXIT_CRITICAL();

		osDelayUntil(time_ticks + MOVE_PID_LOOP_PERIOD_TICKS);
	} while((MOTOR_ENCODER_MAX_POS << 1 - ENCODER_POS_DIRECTIONAL[0] - ENCODER_POS_DIRECTIONAL[1]) / 20 < centimeters); // while still in delay loop

	motor_stop();
}

/**
 * @brief Uses servo mag 5 for turns (slightly tighter than previous implementation)
 *
 * @param direction
 * @param degrees
 */
void move_turn_forward_pid_degrees(MoveDirection direction, uint16_t degrees) {
	uint32_t time_ticks = osKernelGetTickCount();
	uint32_t start_ticks = time_ticks;
	uint32_t target_ticks = start_ticks + (uint32_t) degrees * MOVE_PID_TURN_TICKS_PER_DEGREE;
	uint16_t left_motor_speed = direction == MoveDirLeft ? MOVE_DEFAULT_SPEED_MM_S * MOVE_PID_TURN_REDUCTION_FACTOR : MOVE_DEFAULT_SPEED_MM_S;
	uint16_t right_motor_speed = direction == MoveDirRight ? MOVE_DEFAULT_SPEED_MM_S * MOVE_PID_TURN_REDUCTION_FACTOR : MOVE_DEFAULT_SPEED_MM_S;;
	_pid_reset(time_ticks);
	encoder_reset_counters_forward();

	servo_point(direction, ServoMag5);
	osDelay(SERVO_FULL_LOCK_DELAY);

	do {
		time_ticks = osKernelGetTickCount();

		taskENTER_CRITICAL();
		_set_motor_speed_pid(MotorDirForward, MotorLeft, left_motor_speed);
		_set_motor_speed_pid(MotorDirForward, MotorRight, right_motor_speed);
		taskEXIT_CRITICAL();

		osDelayUntil(time_ticks + MOVE_PID_LOOP_PERIOD_TICKS);
	} while(time_ticks < target_ticks);

	motor_stop();
	servo_point_center();
}

/**
 * @brief Uses servo mag 5 for turns (slightly tighter than previous implenmentation)
 *
 * @param direction
 * @param degrees
 */
void move_turn_backward_pid_degrees(MoveDirection direction, uint16_t degrees) {
	uint32_t time_ticks = osKernelGetTickCount();
	uint32_t start_ticks = time_ticks;
	uint32_t target_ticks = start_ticks + (uint32_t) degrees * MOVE_PID_TURN_TICKS_PER_DEGREE;
	uint16_t left_motor_speed = direction == MoveDirLeft ? MOVE_DEFAULT_SPEED_MM_S * MOVE_PID_TURN_REDUCTION_FACTOR : MOVE_DEFAULT_SPEED_MM_S;
	uint16_t right_motor_speed = direction == MoveDirRight ? MOVE_DEFAULT_SPEED_MM_S * MOVE_PID_TURN_REDUCTION_FACTOR : MOVE_DEFAULT_SPEED_MM_S;;
	_pid_reset(time_ticks);
	encoder_reset_counters_backward();

	servo_point(direction, ServoMag5);
	osDelay(SERVO_FULL_LOCK_DELAY);

	do {
		time_ticks = osKernelGetTickCount();

		taskENTER_CRITICAL();
		_set_motor_speed_pid(MotorDirBackward, MotorLeft, left_motor_speed);
		_set_motor_speed_pid(MotorDirBackward, MotorRight, right_motor_speed);
		taskEXIT_CRITICAL();

		osDelayUntil(time_ticks + MOVE_PID_LOOP_PERIOD_TICKS);
	} while(time_ticks < target_ticks);

	motor_stop();
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
	for (uint8_t i = 0; i < num_turns; i++)
	{
		move_turn_backward_by(rev_dir, 12);
		osDelay(100);
		move_turn_forward_by(for_dir, 11);
		osDelay(100);
	}
}

// Move to minimum forward clearance from obstacle
// Calculated to be 25cm, approx 1 car length away
void move_to_obstacle(void)
{
	int32_t ir_diff = 0;
	int32_t ticks = 0;
	do
	{
		ticks = osKernelGetTickCount();
		taskENTER_CRITICAL();
		ir_diff = (int32_t)(1250 - IR_ADC_AVERAGE_READOUT);
		taskEXIT_CRITICAL();

		if (ir_diff > 0)
		{
			forward(MotorSpeed1);
		}
		else
		{
			backward(MotorSpeed1);
		}
		osDelayUntil(ticks + 25);
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
