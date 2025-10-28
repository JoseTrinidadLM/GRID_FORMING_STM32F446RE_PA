/*
 * stm32f446xx_rcc_driver.c
 *
 *  Created on: Oct 16, 2025
 *      Author: jtlopez
 */

#include "stm32f446xx_rcc_driver.h"

/*
 * Possible Pre-escalars for AHB and APB
 */

uint16_t AHBprescalar[8] = {2,4,8,16,64,128,256,512};
uint16_t APBprescalar[4] = {2,4,8,16};

/*
 * Function to Get APB1 clock value
 */

uint32_t RCC_GetPClk1(void)
{
	uint32_t pclk1, sysclk;
	uint16_t apbp1, ahbp1, temp;

	//System clock Selection
	temp = (RCC->CFGR >> 2) & 0x3;

	if(temp == 0)
	{
		sysclk = 16000000; //16Mhz
	}else if(temp == 1)
	{
		sysclk = 8000000; //8Mhz
	}else if(temp == 2)
	{

	}

	//AHB1 Pre-escaler
	temp = (RCC->CFGR >> 4) & 0xF;
	if(temp<8)
	{
		ahbp1 = 1;
	}else
	{
		ahbp1 = AHBprescalar[temp-8];
	}

	//APB1 Pre-escaler
	temp = (RCC->CFGR >> 10) & 0x7;
	if(temp<4)
	{
		apbp1 = 1;
	}else
	{
		apbp1 = APBprescalar[temp-4];
	}

	//Calculate APB1 clock
	pclk1 = (sysclk/ahbp1)/apbp1; //(System Clock/AHB1 Pre-escalar)/APB1 Pre-escalar

	return pclk1;
}

/*
 * Function to Get APB2 clock value
 */

uint32_t RCC_GetPClk2(void)
{
	uint32_t pclk2, sysclk;
	uint16_t apbp2, ahbp1, temp;

	//System clock Selection
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

	//AHB1 Pre-escaler
	temp = (RCC->CFGR >> 4) & 0xF;
	if(temp<8)
	{
		ahbp1 = 1;
	}else
	{
		ahbp1 = AHBprescalar[temp-8];
	}

	//APB2 Pre-escaler
	temp = (RCC->CFGR >> 13) & 0x7;
	if(temp<4)
	{
		apbp2 = 1;
	}else
	{
		apbp2 = APBprescalar[temp-4];
	}

	//Calculate APB2 clock
	pclk2 = (sysclk/ahbp1)/apbp2; //(System Clock/AHB2 Pre-escalar)/APB2 Pre-escalar

	return pclk2;
}
