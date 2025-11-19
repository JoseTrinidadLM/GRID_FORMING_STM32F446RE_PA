/*
 * stm32f446xx_control.h
 *
 *  Created on: Nov 18, 2025
 *      Author: jiperez
 */

#ifndef STM32F446XX_CONTROL_H_
#define STM32F446XX_CONTROL_H_

#include "stm32f446xx.h"

void ControlInit(void);

void Control_Start(void);

uint8_t Control_ReadSensors(float* values);

void Control_DutyCycle(void);

uint8_t Control_Mode(void);

void TIM2_IRQHandling(void);


#endif /* STM32F446XX_CONTROL_H_ */
