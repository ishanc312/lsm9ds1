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
#include "stm32l4xx_hal.h"

int _write(int file, char *data, int len);

typedef struct {
	float accelOffsets[3];
	float accelSlopes[3];
	float gyroOffsets[3];
	float gyroSlopes[3];
} CALIBRATION_CONSTANTS;

#define FLASH_CALIBRATION_ADDR  ((uint32_t)0x0803F800)

void saveCalibrationToFlash(CALIBRATION_CONSTANTS* data);
void loadCalibrationFromFlash(CALIBRATION_CONSTANTS* data);
void clearCalibrationFlash();

#endif /* INC_CALIBRATION_H_ */
