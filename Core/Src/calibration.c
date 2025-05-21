/*
 * calibration.c
 *
 *  Created on: May 20, 2025
 *      Author: ishanchitale
 */

#include "calibration.h"

int _write(int file, char *data, int len) {
    HAL_UART_Transmit(&huart2, (uint8_t*)data, len, HAL_MAX_DELAY);
    return len;
}

void saveCalibrationToFlash(CALIBRATION_CONSTANTS* data) {
	HAL_FLASH_Unlock(); // Unlock the flash

	// Erase any current contents of the flash memory
	FLASH_EraseInitTypeDef eraseConfig = {
			.TypeErase = FLASH_TYPEERASE_PAGES,
	        .Page = (FLASH_CALIBRATION_ADDR - FLASH_BASE) / FLASH_PAGE_SIZE,
	        .NbPages = 1
	};
	uint32_t PageError;
	HAL_FLASHEx_Erase(&eraseConfig, &PageError);

	// Write double words to the flash memory
    uint64_t *src = (uint64_t *)data;
    for (uint32_t i = 0; i < sizeof(CALIBRATION_CONSTANTS) / 8; i++) {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FLASH_CALIBRATION_ADDR + i * 8, src[i]);
    }

    HAL_FLASH_Lock(); // Lock the flash once more
}

void loadCalibrationFromFlash(CALIBRATION_CONSTANTS* data) {
    memcpy(data, (void *)FLASH_CALIBRATION_ADDR, sizeof(CALIBRATION_CONSTANTS));
}
