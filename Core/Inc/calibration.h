/*
 * calibration.h
 *
 *  Created on: May 20, 2025
 *      Author: ishanchitale
 */

#ifndef INC_CALIBRATION_H_
#define INC_CALIBRATION_H_

#include "lsm.h"
#include "usart.h"
#include "stdio.h"
#include "string.h"
#include "math.h"
#include "stm32l4xx_hal.h"

extern uint8_t accel_axis_status;
extern uint8_t gyro_axis_status;
extern float maxA[3];
extern float minA[3];
#define ACCEL_CRITERION 0.1
#define GYRO_CRITERION 0.1

int _write(int file, char *data, int len);

typedef struct {
	float accelOffsets[3];
	float accelSlopes[3];
	float gyroOffsets[3];
	float gyroSlopes[3];
} CALIBRATION_CONSTANTS;

#define FLASH_CALIBRATION_ADDR  ((uint32_t)0x0803F800)
#define FLASH_CALIBRATION_PAGE ((FLASH_CALIBRATION_ADDR - FLASH_BASE) / FLASH_PAGE_SIZE)
#define ACCEL_MAGIC_FLAG 0xACC3AABB
#define GYRO_MAGIC_FLAG 0xFAB0CCDD
#define ACCEL_CALIBRATION_FLAG_ADDR (FLASH_CALIBRATION_ADDR + sizeof(CALIBRATION_CONSTANTS))
#define GYRO_CALIBRATION_FLAG_ADDR (ACCEL_CALIBRATION_FLAG_ADDR + sizeof(uint64_t))


void saveCalibrationToFlash(CALIBRATION_CONSTANTS* data);
void loadCalibrationFromFlash(CALIBRATION_CONSTANTS* data);
void clearCalibrationFlash();

void getRawAverageAccel(uint16_t N, LSM* imu, float* ax, float* ay, float* az);
bool calibrateAccel(uint16_t N, LSM* imu, CALIBRATION_CONSTANTS* factors);
void accelCalibrationLoop(uint16_t N, LSM* imu, CALIBRATION_CONSTANTS* factors);

void getRawAverageGyro(uint16_t N, LSM* imu, float* roll, float* pitch, float* yaw);
bool calibrateGyroOffset(uint16_t N, LSM* imu, CALIBRATION_CONSTANTS* factors);
bool calibrateGyroSlopes(uint16_t N, LSM* imu, CALIBRATION_CONSTANTS* factors);
void gyroCalibrationLoop(uint16_t N, LSM* imu, CALIBRATION_CONSTANTS* factors);

#endif /* INC_CALIBRATION_H_ */
