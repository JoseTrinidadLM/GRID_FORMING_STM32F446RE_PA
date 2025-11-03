/*
 * stm32f446_rcc_driver.h
 *
 *  Created on: Oct 15, 2025
 *      Author: Isaac Perez (ShiLiba) & jtlopez
 */

#ifndef INC_STM32F446_RCC_DRIVER_H_
#define INC_STM32F446_RCC_DRIVER_H_

#include "stm32f446xx.h"

#define HSIRDY_IS_RDY 	(1 << 1)
#define SWS_MASK 		(3 << 2)
#define SWS_IS_SEL		(2 << 2)
#define PLL_IS_RDY		(1 << 25)

/*System Clock Configuration*/

void SystemCLK_ConfigkHz(uint32_t Clk); //(Possible change to function that receives the frequency?)

/*Global Clock Variables*/

uint32_t RCC_GetPClk1(void); // Peripheral Clock 1 Variable = APB1 Clock
uint32_t RCC_GetPClk2(void); // Peripheral Clock 2 Variable = APB2 Clock


#endif /* INC_STM32F446_RCC_DRIVER_H_ */
