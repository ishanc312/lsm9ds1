/*
 * lms.c
 *
 *  Created on: Apr 5, 2025
 *      Author: ishanchitale
 */

#include "lms.h"
#include "spi.h"

bool AG_SingleByte_Write(uint8_t reg, uint8_t txData) {
	uint8_t write_msg[2] = {reg, txData};
	HAL_GPIO_WritePin(AG_CS_PORT, AG_CS_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, write_msg, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(AG_CS_PORT, AG_CS_PIN, GPIO_PIN_SET);

	uint8_t data_check;
	AG_Read(reg, &data_check, 1);
	if (data_check != txData) return 0;

	return 1;
}

void AG_Read(uint8_t reg, uint8_t* rxData, uint16_t numBytes) {
	/* NOTE: For this device, you don't need to set the second bit
	of the read request to 1 for a multibyte read */
	reg |= READ_FLAG;
	HAL_GPIO_WritePin(AG_CS_PORT, AG_CS_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, &reg, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi3, rxData, numBytes, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(AG_CS_PORT, AG_CS_PIN, GPIO_PIN_SET);
}

bool reboot() {
	return AG_SingleByte_Write(0x22, 0b10000000);
}

bool Enable_XL_G() {
	// Enable XL and Gyroscope to be sampled at 238hz for now
	return AG_SingleByte_Write(0x10, 0b10000000);
}

void AG_Who_Am_I(uint8_t* rxData) {
	AG_Read(0x0F, rxData, 1);
	// Should output 0b01101000
}

void readXL(uint8_t* rxData) {
	AG_Read(0x28, rxData, 6);
}

void readGyro(uint8_t* rxData) {
	AG_Read(0x18, rxData, 6);
}

