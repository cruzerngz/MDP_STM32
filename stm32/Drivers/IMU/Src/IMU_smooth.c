/*
 * IMU_smooth.c
 *
 *  Created on: Sep 11, 2022
 *      Author: cruzerng
 */

#include "math.h"
#include "IMU_smooth.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

float IMU_yaw = 0.0f;
static uint32_t prev_ticks = 0;

static ICM20948 *IMU_INSTANCE = NULL;
IMUData_t IMU_QUEUE[IMU_NUM_SAMPLES_AVG] = {0};
static uint16_t IMU_QUEUE_POSITION = 0;
static uint16_t IMU_CALCS = 0;

// Current avg stored here
IMUData_t IMU_CURRENT_READOUT = {0};
// temp stored here
static IMUData_t IMU_TEMP_READOUT = {0};

// Private function declarations

void _icm_to_imu_data(ICM20948 *imu, IMUData_t *imu_data);
void _imu_filter_gyro(IMUData_t *imu_data);
void _add_to_imu_queue(IMUData_t *data);
void _imu_queue_average(IMUData_t *imu_data);


// Start private functions

// Convert the ICM to IMU
void _icm_to_imu_data(ICM20948 *imu, IMUData_t *imu_data) {
	imu_data->acc[0] = imu->acc[0];
	imu_data->acc[1] = imu->acc[1];
	imu_data->acc[2] = imu->acc[2];

	imu_data->gyro[0] = imu->gyro[0];
	imu_data->gyro[1] = imu->gyro[1];
	imu_data->gyro[2] = imu->gyro[2];

	imu_data->temp_C = imu->temp_C;
}

// Performs filtering of the IMU gyro data
// Low and high limits set in header file
void _imu_filter_data(IMUData_t *imu_data) {
	static float abs_jitter = 0;

	// custom offset for z-axis
//	if(imu_data->gyro[2] < 0.0f) {
//		imu_data->gyro[2] = imu_data->gyro[2] * 0.25f;
//	}

	for(int i=0; i<3; i++) {

		abs_jitter = fabs(imu_data->gyro[i]);
		if(
				abs_jitter < (float)IMU_JITTER_LOW_LIMIT ||
				abs_jitter > (float)IMU_JITTER_HIGH_LIMIT
		) {
			imu_data->gyro[i] = 0.0f;
		}
	}
}

/**
 * Adds new data to the IMU queue
 * Overrides any pre-existing data
 */
void _add_to_imu_queue(IMUData_t *data) {
	IMU_QUEUE[IMU_QUEUE_POSITION] = *data;
	if(IMU_QUEUE_POSITION >= IMU_NUM_SAMPLES_AVG) IMU_QUEUE_POSITION = 0;
	else IMU_QUEUE_POSITION++;
}

/**
 * Calculate average IMU readings
 */
void _imu_queue_average(IMUData_t *imu_data) {
	float avg_acc[3] = {0.0f};
	float avg_gyro[3] = {0.0f};
	float avg_temp_C = 0;

	for(int i=0; i<3; i++) {
		// accumulate
		for(int q_index=0; q_index<IMU_NUM_SAMPLES_AVG; q_index++) {
			// accumulate the temp amt
			if(i==0) {
				avg_temp_C += IMU_QUEUE[q_index].temp_C;
			}
			avg_acc[i] += IMU_QUEUE[q_index].acc[i];
			avg_gyro[i] += IMU_QUEUE[q_index].gyro[i];
		}
	}

	// divide
	avg_temp_C /= IMU_NUM_SAMPLES_AVG;

	for(int j=0; j<3; j++) {
		avg_acc[j] /= IMU_NUM_SAMPLES_AVG;
		avg_gyro[j] /= IMU_NUM_SAMPLES_AVG;
	}

	imu_data->acc[0] = avg_acc[0];
	imu_data->acc[1] = avg_acc[1];
	imu_data->acc[2] = avg_acc[2];
	imu_data->gyro[0] = avg_gyro[0];
	imu_data->gyro[1] = avg_gyro[1];
	imu_data->gyro[2] = avg_gyro[2];
	imu_data->temp_C = avg_temp_C;
}

// Start public functions

/**
 * Wrapper for IMU_Initialise()
 * This function sets some private pointers and
 * inits the private circular queue (for averaging IMU data)
 */
void IMU_S_Initialise(ICM20948 *dev, I2C_HandleTypeDef *i2cHandle, UART_HandleTypeDef *uart) {
	IMU_Initialise(dev, i2cHandle, uart); // pass params to actual init

	HAL_Delay(200);
//	Gyro_calibrate(dev); // perform calibration on init
	IMU_INSTANCE = dev;	 // assign the imu pointer to private static

	IMU_ReadAll(IMU_INSTANCE);
	_icm_to_imu_data(IMU_INSTANCE, &IMU_CURRENT_READOUT);

	// init all elements in queue to the same
	for(uint16_t q_index=0; q_index<IMU_NUM_SAMPLES_AVG; q_index++) {
		IMU_QUEUE[q_index] = IMU_CURRENT_READOUT;
	}
}

void IMU_reset_yaw() {
	IMU_yaw = 0.0f;
}

/**
 * Main function of this translation unit.
 * Returns the average IMU readings inside a custom struct
 */
void IMU_Poll(IMUData_t *data) {
	static curr_ticks = 0;
	curr_ticks = osKernelGetTickCount();
	IMU_ReadAll(IMU_INSTANCE);

#ifdef IMU_ENABLE_SMOOTHING
	_icm_to_imu_data(IMU_INSTANCE, &IMU_TEMP_READOUT);
	_imu_filter_data(&IMU_TEMP_READOUT);
	_add_to_imu_queue(&IMU_TEMP_READOUT);

//	*data = IMU_QUEUE[IMU_QUEUE_POSITION];

	if(IMU_CALCS < IMU_CALC_INTERVAL) {
		IMU_CALCS++;
		*data = IMU_CURRENT_READOUT;
		return;
	}
	else {
		IMU_CALCS = 0;
		_imu_queue_average(&IMU_CURRENT_READOUT);
		*data = IMU_CURRENT_READOUT;
		return;
	}

#else
	_icm_to_imu_data(IMU_INSTANCE, data);
	IMU_yaw += IMU_INSTANCE->gyro[2] * (float)((curr_ticks - prev_ticks) / 1000);
#endif
}
