/*
 * lms.h
 *
 *  Created on: Apr 5, 2025
 *      Author: ishanchitale
 */

#ifndef INC_LMS_H_
#define INC_LMS_H_

#include "stdint.h"
#include "stdbool.h"
#include "spi.h"

#define READ_FLAG 0x80
#define LSM_ID 0b01101000
#define ACCEL_SENS_2G  16384.0f
#define GYRO_SENS_245DPS 8.75f

typedef struct LSM {
	SPI_HandleTypeDef* spiInstance;
	GPIO_TypeDef* AG_CS_PORT;
	uint16_t AG_CS_PIN;

	uint8_t accelBytes[6];
	uint8_t gyroBytes[6];

	float accel[3];
	float gyro[3];
} LSM;

void initLSM(LSM* imu, SPI_HandleTypeDef* spi, GPIO_TypeDef* AG_PORT, uint16_t AG_PIN);
bool Enable_XL_G(LSM* imu);
bool IdCheck(LSM* imu);

bool reboot(LSM* imu);
bool AG_SingleByte_Write(LSM* imu, uint8_t reg, uint8_t txData);
bool AG_Read(LSM* imu, uint8_t reg, uint8_t* rxData, uint16_t numBytes);
void readXL(LSM* imu);
void readGyro(LSM* imu);

void computeAccel(LSM* imu);
void computeGyro(LSM* imu);

#endif /* INC_LMS_H_ */
