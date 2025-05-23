/*
 * lms.c
 *
 *  Created on: Apr 5, 2025
 *      Author: ishanchitale
 */

#include "lsm.h"

void initLSM(LSM* IMU, SPI_HandleTypeDef* spi, GPIO_TypeDef* AG_PORT, uint16_t AG_PIN) {
	IMU->spiInstance = spi;
	IMU->AG_CS_PORT = AG_PORT;
	IMU->AG_CS_PIN = AG_PIN;

	HAL_GPIO_WritePin(IMU->AG_CS_PORT, IMU->AG_CS_PIN, GPIO_PIN_SET);
}

bool Enable_XL_G(LSM* IMU) {
	// Enable XL and Gyroscope to be sampled at 238hz for now
	return (AG_SingleByte_Write(IMU, 0x10, 0b10000000) & AG_SingleByte_Write(IMU, 0x20, 0b10000000));
}

bool IdCheck(LSM* IMU) {
	uint8_t idCheck;
	AG_Read(IMU, 0x0F, &idCheck, 1);
	if (idCheck == LSM_ID) return 1;
	return 0;
	// Should output 0b01101000
}

bool AG_SingleByte_Write(LSM* IMU, uint8_t reg, uint8_t txData) {
	uint8_t write_msg[2] = {reg, txData};
	HAL_GPIO_WritePin(IMU->AG_CS_PORT, IMU->AG_CS_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit(IMU->spiInstance, write_msg, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(IMU->AG_CS_PORT, IMU->AG_CS_PIN, GPIO_PIN_SET);

	uint8_t data_check;
	AG_Read(IMU, reg, &data_check, 1);
	if (data_check != txData) return 0;

	return 1;
}

bool AG_Read(LSM* IMU, uint8_t reg, uint8_t* rxData, uint16_t numBytes) {
    reg |= READ_FLAG;

    HAL_GPIO_WritePin(IMU->AG_CS_PORT, IMU->AG_CS_PIN, GPIO_PIN_RESET);

    if (HAL_SPI_Transmit(IMU->spiInstance, &reg, 1, HAL_MAX_DELAY) != HAL_OK) {
        HAL_GPIO_WritePin(IMU->AG_CS_PORT, IMU->AG_CS_PIN, GPIO_PIN_SET);
        return false;
    }

    if (HAL_SPI_Receive(IMU->spiInstance, rxData, numBytes, HAL_MAX_DELAY) != HAL_OK) {
        HAL_GPIO_WritePin(IMU->AG_CS_PORT, IMU->AG_CS_PIN, GPIO_PIN_SET);
        return false;
    }

    HAL_GPIO_WritePin(IMU->AG_CS_PORT, IMU->AG_CS_PIN, GPIO_PIN_SET);
    return true;
}

bool reboot(LSM* IMU) {
	return AG_SingleByte_Write(IMU, 0x22, 0b10000000);
}

void readXL(LSM* IMU) {
	AG_Read(IMU, 0x28, IMU->accelBytes, 6);
}

void readGyro(LSM* IMU) {
	AG_Read(IMU, 0x18, IMU->gyroBytes, 6);
}

// Computation/Calibration Helper Functions

void computeRawAccel(LSM* IMU) {
	// Combine LSB & MSB and convert to signed int (via two's complement)
	int16_t raw_x = (int16_t) (IMU->accelBytes[1] << 8 | IMU->accelBytes[0]);
	int16_t raw_y = (int16_t) (IMU->accelBytes[3] << 8 | IMU->accelBytes[2]);
	int16_t raw_z = (int16_t) (IMU->accelBytes[5] << 8 | IMU->accelBytes[4]);

	// Divide by default sensitivity of the IMU
	IMU->rawAccel[0] = raw_x/ACCEL_SENS_2G;
	IMU->rawAccel[1] = raw_y/ACCEL_SENS_2G;
	IMU->rawAccel[2] = raw_z/ACCEL_SENS_2G;
}

void computeRawGyro(LSM* IMU) {
	// Combine LSB & MSB and convert to signed int (via two's complement)
	int16_t roll = (int16_t) (IMU->gyroBytes[1] << 8 | IMU->gyroBytes[0]);
	int16_t pitch = (int16_t) (IMU->gyroBytes[3] << 8 | IMU->gyroBytes[2]);
	int16_t yaw = (int16_t) (IMU->gyroBytes[5] << 8 | IMU->gyroBytes[4]);

	// Divide by default sensitivity of the gyroscope
	IMU->rawGyro[0] = roll/GYRO_SENS_245DPS;
	IMU->rawGyro[1] = pitch/GYRO_SENS_245DPS;
	IMU->rawGyro[2] = yaw/GYRO_SENS_245DPS;
}
