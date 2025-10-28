/*
 * stm32f446xx_rcc_driver.h
 *
 *  Created on: Oct 16, 2025
 *      Author: jtlopez
 */

#ifndef INC_STM32F446XX_RCC_DRIVER_H_
#define INC_STM32F446XX_RCC_DRIVER_H_

#include "stm32f446xx.h"

/*Global Clock Variables*/

uint32_t RCC_GetPClk1(void); // Peripheral Clock 1 Variable = APB1 Clock
uint32_t RCC_GetPClk2(void); // Peripheral Clock 2 Variable = APB2 Clock

#endif /* INC_STM32F446XX_RCC_DRIVER_H_ */
