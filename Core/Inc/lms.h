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

#define READ_FLAG 0x80
#define AG_CS_PORT GPIOB
#define AG_CS_PIN GPIO_PIN_6
#define AG_G_PORT GPIOB
#define AG_G_PIN GPIO_PIN_7

bool AG_SingleByte_Write(uint8_t reg, uint8_t txData);
void AG_Read(uint8_t reg, uint8_t* rxData, uint16_t numBytes);

bool reboot();
bool Enable_XL_G();
void AG_Who_Am_I(uint8_t* rxData);
void readXL(uint8_t* rxData);
void readGyro(uint8_t* rxData);

#endif /* INC_LMS_H_ */
