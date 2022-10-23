/*
 * IMU_smooth.h
 *
 *  Created on: Sep 11, 2022
 *      Author: cruzerng
 */

#include "IMU.h"

#ifndef IMU_INC_IMU_SMOOTH_H_
#define IMU_INC_IMU_SMOOTH_H_

// uncomment this to revert to normal IMU reads (no averaging of data)
//#define IMU_ENABLE_SMOOTHING


#define IMU_POLLING_RATE_TICKS 20 // 50 hz

// number of samples to take when calculating a rolling avg
#define IMU_NUM_SAMPLES_AVG 2
// Number of update intervals to wait before updating the rolling average
// set to 0 to calculate every update
// set to 1 to calculate every other update
// etc...
#define IMU_CALC_INTERVAL 1

#define IMU_JITTER_LOW_LIMIT 0.5f // IMU change low-end cut-off, in degrees/sec
#define IMU_JITTER_HIGH_LIMIT 90 // IMU change high-end cut-off, in degrees/sec (same as IMU output)

// ICM20948 struct stripped of UART and I2C pointers
typedef struct {
	float acc[3];
	float gyro[3];
	float temp_C;
} IMUData_t;

extern float IMU_yaw;

void IMU_S_Initialise(ICM20948 *dev, I2C_HandleTypeDef *i2cHandle, UART_HandleTypeDef *uart);
void IMU_reset_yaw();
void IMU_Poll(IMUData_t *data);


#endif /* IMU_INC_IMU_SMOOTH_H_ */
