/*
 * calibration.c
 *
 *  Created on: May 20, 2025
 *      Author: ishanchitale
 */

#include "calibration.h"

float maxA[3] = {1.0, 1.0, 1.0};
float minA[3] = {-1.0, -1.0, -1.0};
uint8_t accel_axis_status = 0;
uint8_t gyro_axis_status = 0;

int _write(int file, char *data, int len) {
    HAL_UART_Transmit(&huart2, (uint8_t*)data, len, HAL_MAX_DELAY);
    return len;
}

// ACCEL CALIBRATION
void getRawAverageAccel(uint16_t N, LSM* imu, float* ax, float* ay, float* az) {
	float x = 0;
	float y = 0;
	float z = 0;
	for (size_t i = 0; i < N; i++) {
		readXL(imu);
		computeRawAccel(imu);
		x+=(imu->rawAccel[0]);
		y+=(imu->rawAccel[1]);
		z+=(imu->rawAccel[2]);
		HAL_Delay(100);
	}

	*ax = x/N;
	*ay = y/N;
	*az = z/N;
}

bool calibrateAccel(uint16_t N, LSM* imu, CALIBRATION_CONSTANTS* factors) {
	float ax = 0;
	float ay = 0;
	float az = 0;
	getRawAverageAccel(N, imu, &ax, &ay, &az);

	uint8_t valid = 0;
	if (fabs(ax) > fmax(fabs(ay), fabs(az)) && sqrt(ay*ay + az*az)/fabs(ax) < ACCEL_CRITERION) {
		valid = 1;
		if (ax > 0) {
			// x-axis pointing upward; ~1g
			maxA[0] = ax;
			accel_axis_status |= 1 << 0;
		} else {
			// x-axis pointing downward; ~ -1g
			minA[0] = ax;
			accel_axis_status |= 1 << 3;
		}
	}

	if (fabs(ay) > fmax(fabs(ax), fabs(az)) && sqrt(ax*ax + az*az)/fabs(ay) < ACCEL_CRITERION) {
		valid = 1;
		if (ay > 0) {
			// y-axis pointing upward; ~1g
			maxA[1] = ay;
			accel_axis_status |= 1 << 1;
		} else {
			// y-axis pointing downward; ~1g
			minA[1] = ay;
			accel_axis_status |= 1 << 4;
		}
	}

	if (fabs(az) > fmax(fabs(ax), fabs(ay)) && sqrt(ax*ax + ay*ay)/fabs(az) < ACCEL_CRITERION) {
		valid = 1;
		if (az > 0) {
			// z-axis pointing upward; ~1g
			maxA[2] = az;
			accel_axis_status |= 1 << 2;
		} else {
			// z-axis pointing downward; ~ -1g
			minA[2] = az;
			accel_axis_status |= 1 << 5;
		}
	}

	if (valid != 1) return 0;

	factors->accelOffsets[0] = (maxA[0] + minA[0])/2.0;
	factors->accelOffsets[1] = (maxA[1] + minA[1])/2.0;
	factors->accelOffsets[2] = (maxA[2] + minA[2])/2.0;

	factors->accelSlopes[0] = (maxA[0] - minA[0])/2.0;
	factors->accelSlopes[1] = (maxA[1] - minA[1])/2.0;
	factors->accelSlopes[2] = (maxA[2] - minA[2])/2.0;

	return 1;
}

void accelCalibrationLoop(uint16_t N, LSM* imu, CALIBRATION_CONSTANTS* factors) {
	while (accel_axis_status != 0b00111111) {
		printf("ALREADY CALIBRATED AXES: ");
		if (accel_axis_status & (1 << 0)) printf("+X ");
		if (accel_axis_status & (1 << 1)) printf("+Y ");
		if (accel_axis_status & (1 << 2)) printf("+Z ");
		if (accel_axis_status & (1 << 3)) printf("-X ");
		if (accel_axis_status & (1 << 4)) printf("-Y ");
		if (accel_axis_status & (1 << 5)) printf("-Z ");
		printf("\r\n");

		printf("PLACE BOARD IN ORIENTATION... YOU HAVE 7 SECONDS!\r\n");
		HAL_Delay(7000);
		printf("BEGINNING CALIBRATION PROCESS...\r\n");
		if (calibrateAccel(N, imu, factors) == 1) {
			printf("AXIS CALIBRATED.\r\n");
			printf("axis_status: 0x%02X\r\n", accel_axis_status);
		} else {
			printf("PLEASE RE-ALIGN BOARD AXIS....\r\n");
		}
	}

	printf("CALIBRATION COMPLETE: +X +Y +Z -X -Y -Z\r\n");
	printf("axis_status: 0x%02X\r\n", accel_axis_status);

	saveCalibrationToFlash(factors);
}

// GYRO CALIBRATION
void getRawAverageGyro(uint16_t N, LSM* imu, float* roll, float* pitch, float* yaw) {
	float ar = 0;
	float ap = 0;
	float ay = 0;
	for (size_t i = 0; i < N; i++) {
		readGyro(imu);
		computeRawGyro(imu);
		ar+=(imu->rawGyro[0]);
		ap+=(imu->rawGyro[1]);
		ay+=(imu->rawGyro[2]);
		HAL_Delay(100);
	}

	*roll = ar/N;
	*pitch = ap/N;
	*yaw = ay/N;
}

bool calibrateGyroOffset(uint16_t N, LSM* imu, CALIBRATION_CONSTANTS* factors) {
	float roll_o = 0;
	float pitch_o = 0;
	float yaw_o = 0;
	getRawAverageGyro(N, imu, &roll_o, &pitch_o, &yaw_o);
	factors->gyroOffsets[0] = roll_o;
	factors->gyroOffsets[1] = pitch_o;
	factors->gyroOffsets[2] = yaw_o;

	return 1;
}

bool calibrateGyroSlopes(uint16_t N, LSM* imu, CALIBRATION_CONSTANTS* factors) {
	bool validMmt = false;
	float dirX = 0;
	float dirY = 0;
	float dirZ = 0;

	float sigmaX2 = 0;
	float sigmaY2 = 0;
	float sigmaZ2 = 0;

	float maxXYZ = 0;

	for (size_t i = 0; i < N; i++) {
		readGyro(imu);
		computeRawGyro(imu);

		dirX += (imu->rawGyro[0] - factors->gyroOffsets[0])/238;
		dirY += (imu->rawGyro[1] - factors->gyroOffsets[1])/238;
		dirZ += (imu->rawGyro[2] - factors->gyroOffsets[2])/238;

		sigmaX2 += (imu->rawGyro[0]*imu->rawGyro[0]);
		sigmaY2 += (imu->rawGyro[1]*imu->rawGyro[1]);
		sigmaZ2 += (imu->rawGyro[2]*imu->rawGyro[2]);

		float absX = fabsf(imu->rawGyro[0]), absY = fabsf(imu->rawGyro[1]), absZ = fabsf(imu->rawGyro[2]);
		if (absX > maxXYZ) maxXYZ = absX;
		if (absY > maxXYZ) maxXYZ = absY;
		if (absZ > maxXYZ) maxXYZ = absZ;

		HAL_Delay(100);
	}

	sigmaX2 /= N;
	sigmaY2 /= N;
	sigmaZ2 /= N;

	dirX = fabsf(dirX);
	dirY = fabsf(dirY);
	dirZ = fabsf(dirZ);

	if (dirX > fmaxf(dirY, dirZ)) {
		if (sigmaY2 < GYRO_CRITERION && sigmaZ2 < GYRO_CRITERION) {
			validMmt = true;
            factors->gyroSlopes[0] = (float) (180.0/dirX);
			gyro_axis_status |= 1 << 0;
		}
	}

	if (dirY > fmaxf(dirX, dirZ)) {
		if (sigmaX2 < GYRO_CRITERION && sigmaZ2 < GYRO_CRITERION) {
			validMmt = true;
			factors->gyroSlopes[1] = (float) (180.0/dirY);
			gyro_axis_status |= 1 << 1;
		}
	}

	if (dirZ > fmaxf(dirX, dirY)) {
		if (sigmaX2 < GYRO_CRITERION && sigmaY2 < GYRO_CRITERION) {
			validMmt = true;
			factors->gyroSlopes[2] = (float) (180.0/dirZ);
			gyro_axis_status |= 1 << 2;
		}
	}

	if (validMmt == false) return false;

	return validMmt;
}

void gyroCalibrationLoop(uint16_t N, LSM* imu, CALIBRATION_CONSTANTS* factors) {
    if (!calibrateGyroOffset(N, imu, factors)) {
        printf("Gyro offset calibration failed.\r\n");
        return;
    }

	while (gyro_axis_status != 0b00000111) {
		printf("ALREADY CALIBRATED AXES: ");
		if (gyro_axis_status & (1 << 0)) printf("+X ");
		if (gyro_axis_status & (1 << 1)) printf(" Y");
		if (gyro_axis_status & (1 << 2)) printf(" Z");
		printf("\r\n");

		printf("PLACE BOARD IN ORIENTATION... YOU HAVE 5 SECONDS!\r\n");
		HAL_Delay(7000);
		printf("BEGINNING CALIBRATION PROCESS.... PLEASE ROTATE 180 DEGREES OVER 10 SECONDS.\r\n");

		// Change this rq
		if (calibrateGyroSlopes(N, imu, factors) == 1) {
			printf("AXIS CALIBRATED.\r\n");
			printf("axis_status: 0x%02X\r\n", gyro_axis_status);
		} else {
			printf("PLEASE RE-ALIGN BOARD AXIS...\r\n");
		}
	}

	printf("CALIBRATION COMPLETE: X Y Z");
	printf("axis_status: 0x%02X\r\n", gyro_axis_status);

	saveCalibrationToFlash(factors);
}


void saveCalibrationToFlash(CALIBRATION_CONSTANTS* data) {
	HAL_FLASH_Unlock(); // Unlock the flash

    // Clear any prior error flags
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR |
                           FLASH_FLAG_PGSERR | FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PROGERR);

	// Erase any current contents of the flash memory
	FLASH_EraseInitTypeDef eraseConfig = {
			.TypeErase = FLASH_TYPEERASE_PAGES,
	        .Page = FLASH_CALIBRATION_PAGE,
	        .NbPages = 1
	};
	uint32_t PageError;
	HAL_FLASHEx_Erase(&eraseConfig, &PageError);

	// Write double words to the flash memory
    uint64_t *src = (uint64_t *)data;
    for (uint32_t i = 0; i < sizeof(CALIBRATION_CONSTANTS) / 8; i++) {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FLASH_CALIBRATION_ADDR + i * 8, src[i]);
    }

    // Should probably add two flags: for ACCEL Calibration, and for GYRO Calibration
    if (accel_axis_status == 0b00111111) HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, ACCEL_CALIBRATION_FLAG_ADDR, ACCEL_MAGIC_FLAG);
    if (gyro_axis_status == 0b00000111) HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, GYRO_CALIBRATION_FLAG_ADDR, GYRO_MAGIC_FLAG);

    HAL_FLASH_Lock(); // Lock the flash once more
}

void loadCalibrationFromFlash(CALIBRATION_CONSTANTS* data) {
    memcpy(data, (void *)FLASH_CALIBRATION_ADDR, sizeof(CALIBRATION_CONSTANTS));
}

void clearCalibrationFlash() {
	HAL_FLASH_Unlock(); // Unlock the flash

    // Clear any prior error flags
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR |
                           FLASH_FLAG_PGSERR | FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PROGERR);

	// Erase any current contents of the flash memory
	FLASH_EraseInitTypeDef eraseConfig = {
			.TypeErase = FLASH_TYPEERASE_PAGES,
		    .Page = FLASH_CALIBRATION_PAGE,
		    .NbPages = 1
	};
	uint32_t PageError;
	HAL_FLASHEx_Erase(&eraseConfig, &PageError);

	HAL_FLASH_Lock();
}
