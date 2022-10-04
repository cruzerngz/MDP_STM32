/*
 * IMU.h
 *
 *  Created on: Sep 11, 2022
 *      Author: cruzerng
 */

#ifndef IMU_INC_IMU_H_
#define IMU_INC_IMU_H_

#include "ICM20948.h"
#include <string.h>
#include <stdio.h>

uint8_t* IMU_Initialise(ICM20948 *dev, I2C_HandleTypeDef *i2cHandle, UART_HandleTypeDef *uart);

HAL_StatusTypeDef IMU_WriteOneByte(ICM20948 *dev, uint8_t reg, uint8_t data);
HAL_StatusTypeDef IMU_ReadOneByte(ICM20948 *dev, uint8_t reg, uint8_t *data);
HAL_StatusTypeDef Gyro_calibrate(ICM20948 *dev);

HAL_StatusTypeDef IMU_TempRead(ICM20948 *dev);
HAL_StatusTypeDef IMU_AccelRead(ICM20948 *dev);
HAL_StatusTypeDef IMU_GyroRead(ICM20948 *dev); // read change in pos
HAL_StatusTypeDef IMU_GyroReadAbs(ICM20948 *dev); // read absolute pos (not impl'd)
void IMU_ReadAll(ICM20948 *dev);

#endif /* IMU_INC_IMU_H_ */
