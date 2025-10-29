/*
 * stm32f446_rcc_driver.c
 *
 *  Created on: Oct 15, 2025
 *      Author: jiperez
 */
#include "stm32f446xx_rcc_driver.h"

/************************************************************************************
 * @fn				- SystemCLK_Config_84MHz
 *
 * @brief			- Configures internal clk to 84 MHz
 *
 * @param[in]		- none
 *
 * @return			- none
 *
 * @Note			- none
 */

void SystemCLK_Config_84MHz(void){

    RCC->CR |= (1 << 0);
    while((RCC->CR & (1 << 1)) == 0); // Wait for HSIRDY
    for(__vo uint16_t i = 0; i<1000; i++);
    /*PLL_M = 8; 	HSI/PLL_M
    * PLL_N= 168;	PLL_N*HSI/PLL_M
   	* PLL_P = 4;	(PLL_N*HSI/PLL_M)/PLL_P = Final_Frequency
   	*/
    RCC->PLLCFGR &=~((0x7F << 24) | (1 << 22) | (0x3 << 16) | ( 0x7FFF << 0 ));
    RCC->PLLCFGR |= (8 << 0);    // PLLM = 8
    RCC->PLLCFGR |= (168 << 6);  // PLLN = 168
    RCC->PLLCFGR |= (1 << 16);   // PLLP = 4 (01 = divide by 4)
    RCC->PLLCFGR &= ~(1 << 22);  // HSI as PLL source (bit 22 = 0)


    RCC->CR |= (1 << 24);
    while((RCC->CR & (1 << 25)) == 0); // Wait for PLLRDY
    for(__vo uint16_t i = 0; i<1000; i++);


    FLASH->ACR |= (2 << 0);


    RCC->CFGR &= ~((0xFF << 24) | (0x1FFF << 10) | ( 0xFF << 0 ));
    RCC->CFGR |= (0 << 4);
    RCC->CFGR |= (4 << 10);   // APB1 = /2 (42MHz)
    RCC->CFGR |= (5 << 13);   // APB2 = /4 (21MHz)


    RCC->CFGR |= (2 << 0);
    while((RCC->CFGR & (3 << 2)) != (2 << 2)); // Wait for SWS = PLL
    for(__vo uint16_t i = 0; i<1000; i++);
}

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
		 uint32_t pllm = (RCC->PLLCFGR & 0x3F);
		uint32_t plln = (RCC->PLLCFGR >> 6) & 0x1FF;
		uint32_t pllp = (((RCC->PLLCFGR >> 16) & 0x3) + 1) * 2;
		uint32_t pllsrc = (RCC->PLLCFGR >> 22) & 0x1;

		if(pllsrc == 0) {
			sysclk = (16000000 / pllm) * plln / pllp; // HSI
		} else {
			sysclk = (8000000 / pllm) * plln / pllp;  // HSE (asumiendo 8MHz)
		}
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
        uint32_t pllm = (RCC->PLLCFGR & 0x3F);
        uint32_t plln = (RCC->PLLCFGR >> 6) & 0x1FF;
        uint32_t pllp = (((RCC->PLLCFGR >> 16) & 0x3) + 1) * 2;
        uint32_t pllsrc = (RCC->PLLCFGR >> 22) & 0x1;

        if(pllsrc == 0) {
            sysclk = (16000000 / pllm) * plln / pllp; // HSI
        } else {
            sysclk = (8000000 / pllm) * plln / pllp;  // HSE (asumiendo 8MHz)
        }
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
