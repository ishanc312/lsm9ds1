/*
 * lms.h
 *
 *  Created on: Apr 5, 2025
 *      Author: ishanchitale
 */

#ifndef INC_LSM_H_
#define INC_LSM_H_

#include "stdint.h"
#include "stdbool.h"
#include "spi.h"
#include "can.h"

#define READ_FLAG 0x80
#define LSM_ID 0b01101000
#define ACCEL_SENS_2G  16384.0f
#define GYRO_SENS_245DPS 8.75f
#define IMU_NUMBER 1

typedef union ACCEL_DF {
	struct {
		int16_t accel_x;
		int16_t accel_y;
		int16_t accel_z;
	} data;
	uint8_t array[6];
} ACCEL_DF;

typedef union GYRO_DF {
	struct {
		int16_t roll;
		int16_t pitch;
		int16_t yaw;
	} data;
	uint8_t array[6];
} GYRO_DF;

typedef struct LSM {
	SPI_HandleTypeDef* spiInstance;
	GPIO_TypeDef* AG_CS_PORT;
	uint16_t AG_CS_PIN;

	uint8_t accelBytes[6];
	uint8_t gyroBytes[6];

	float rawAccel[3];
	float correctedAccel[3];
	float rawGyro[3];
	float correctedGyro[3];

	CAN_TxHeaderTypeDef ACCEL_CTXHeader;
	ACCEL_DF accel_df;

	CAN_TxHeaderTypeDef GYRO_CTXHeader;
	GYRO_DF gyro_df;
} LSM;

void initLSM(LSM* imu, SPI_HandleTypeDef* spi, GPIO_TypeDef* AG_PORT, uint16_t AG_PIN);
bool Enable_XL_G(LSM* imu);
bool IdCheck(LSM* imu);

bool reboot(LSM* imu);
bool AG_SingleByte_Write(LSM* imu, uint8_t reg, uint8_t txData);
bool AG_Read(LSM* imu, uint8_t reg, uint8_t* rxData, uint16_t numBytes);
void readXL(LSM* imu);
void readGyro(LSM* imu);

void computeRawAccel(LSM* imu);
void computeRawGyro(LSM* imu);

#endif /* INC_LSM_H_ */
