/*
 * stm32f446xx_rcc_driver.c
 *
 *  Created on: Oct 16, 2025
 *      Author: jtlopez
 */

#include "stm32f446xx_rcc_driver.h"

uint16_t AHBprescalar[8] = {2,4,8,16,64,128,256,512};
uint16_t APBprescalar[4] = {2,4,8,16};

/*
 *
 */

uint32_t RCC_GetPClk1(void)
{
	uint32_t pclk1, sysclk;
	uint16_t apbp1, ahbp1, temp;

	temp = (RCC->CFGR >> 2) & 0x3;

	if(temp == 0)
	{
		sysclk = 16000000;
	}else if(temp == 1)
	{
		sysclk = 8000000;
	}else if(temp == 2)
	{

	}

	//AHB1 Prescaler
	temp = (RCC->CFGR >> 4) & 0xF;
	if(temp<8)
	{
		ahbp1 = 1;
	}else
	{
		ahbp1 = AHBprescalar[temp-8];
	}

	//APB1 Prescaler
	temp = (RCC->CFGR >> 10) & 0x7;
	if(temp<4)
	{
		apbp1 = 1;
	}else
	{
		apbp1 = APBprescalar[temp-4];
	}
	pclk1 = (sysclk/ahbp1)/apbp1;

	return pclk1;
}

uint32_t RCC_GetPClk2(void)
{
	uint32_t pclk2, sysclk;
	uint16_t apbp2, ahbp1, temp;

	temp = (RCC->CFGR >> 2) & 0x3;

	if(temp == 0)
	{
		sysclk = 16000000;
	}else if(temp == 1)
	{
		sysclk = 8000000;
	}else if(temp == 2)
	{

	}

	//AHB1 Prescaler
	temp = (RCC->CFGR >> 4) & 0xF;
	if(temp<8)
	{
		ahbp1 = 1;
	}else
	{
		ahbp1 = AHBprescalar[temp-8];
	}

	//APB2 Prescaler
	temp = (RCC->CFGR >> 13) & 0x7;
	if(temp<4)
	{
		apbp2 = 1;
	}else
	{
		apbp2 = APBprescalar[temp-4];
	}
	pclk2 = (sysclk/ahbp1)/apbp2;

	return pclk2;
}
