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
#define MULTIBYTE_FLAG 0x40
#define AG_CS_PORT GPIOB
#define AG_CS_PIN GPIO_PIN_6
#define AG_G_PORT GPIOB
#define AG_G_PIN GPIO_PIN_7

void LMS_Write(uint8_t reg, uint8_t txData);
void LMS_Read(uint8_t reg, uint8_t rxData, bool multibyte);

void AG_Who_Am_I(uint8_t* rxData);
void Enable_Gyroscope();
void readXL(uint8_t* rxData);
void readG(uint8_t* rxData);

#endif /* INC_LMS_H_ */
