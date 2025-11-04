/*
 * stm32f446_rcc_driver.c
 *
 *  Created on: Oct 15, 2025
 *      Author: jtlopez & jiperez
 */
#include "stm32f446xx_rcc_driver.h"

uint16_t AHBprescalar[8] = {2,4,8,16,64,128,256,512};
uint16_t APBprescalar[4] = {2,4,8,16};


uint32_t RCC_GetSystemClk()
{
	uint32_t sysclk, temp;
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
	return sysclk;
}


/*
 * @fn				- SystemCLK_ConfigkHz
 *
 * @brief			- Configures internal clk to given frequency
 *
 * @param[in]		- Clk
 *
 * @return			- none
 *
 * @Note			- none
 */

void RCC_SystemCLK_ConfigkHz(uint32_t Clk)
{
	uint32_t sysclk;
    RCC->CR |= (1 << 0);
    while((RCC->CR & (1 << 1)) == 0); // Wait for HSIRDY
    for(__vo uint16_t i = 0; i<1000; i++);

    RCC->PLLCFGR &=~((0x7F << 24) | (1 << 22) | (0x3 << 16) | ( 0x7FFF << 0 )); // Clear PLL Configuration

	if(Clk < 180000)
	{
		uint32_t pllm, plln, pllp;
		//PLLM Minimum mClk = HSI/PLLM >= Clk/25 Note: PLLM minimum 2
		pllm = 2; // PLLM Minimum
		while(16000/pllm < Clk/25) pllm++;
		if(pllm>63) pllm = 63;
		RCC->PLLCFGR |= (pllm & 0x3F);
		//PLLN Miminum nClk = mClk*PLLN >= Clk*2 Note: PLLN minimum 50 -> mClk >= Clk/25
		plln = 50;
		while((16000/pllm)*plln < Clk*2) plln++;
		if(plln>432) plln = 432;
		RCC->PLLCFGR |= (plln & 0x1FF) << 6;
		//PLLP Clk = nClk/PLLP Note: PLLP minimum 2 -> nClk >= Clk*2
		pllp = 2;
		while(((16000/pllm)*plln)/pllp < Clk) pllp++;
		if(plln>8) plln = 8;
		RCC->PLLCFGR |= (pllp & 0x3) << 16;
	}
	else
	{

	}
    RCC->PLLCFGR &= ~(1 << 22);  // HSI as PLL source (bit 22 = 0)

    RCC->CR |= (1 << 24); //PLL ON
    while((RCC->CR & (1 << 25)) == 0); // Wait for PLLRDY
    for(__vo uint16_t i = 0; i<1000; i++);

    FLASH->ACR |= (2 << 0);

	sysclk = RCC_GetSystemClk();


    RCC->CFGR &= ~((0xFF << 24) | (0x1FFF << 10) | ( 0xFF << 0 ));
	uint32_t apb1, apb2;
	//AHB Clock 180Mhz
    RCC->CFGR |= (0 << 4); 		//AHB prescaler
	//APB1 Clock Max 45MHz
	if(sysclk > 45000)
	{
		while(sysclk/AHBprescalar[apb1] > 45000) apb1++;
    	RCC->CFGR |= ((apb1+4) << 10);	//APB1 prescaler
	}
	//APB2 Clock Max 90MHz
	if(sysclk > 90000)
	{
		while(sysclk/AHBprescalar[apb2] > 90000) apb2++;
		RCC->CFGR |= ((apb2+4) << 13);	//APB2 prescaler
	}

    RCC->CFGR |= (2 << 0);		//Set PLL as system clock
    while((RCC->CFGR & (3 << 2)) != (2 << 2)); // Wait for SWS = PLL
    for(__vo uint16_t i = 0; i<1000; i++);
}

void RCC_AHBClk_ConfigkHz(uint32_t Clk)
{
	uint32_t sysclk = RCC_GetSystemClk();
	uint32_t ahb;
	
	RCC->CFGR &= ~(0xF << 4);

	if(Clk > 180000)
	{
		Clk = 180000;
	}
	if(sysclk > Clk)
	{
		while((sysclk/AHBprescalar[ahb] > Clk) && (ahb<3)) ahb++;
		RCC->CFGR |= ((ahb+8) << 10);	//APB1 prescaler
	}
}

void RCC_APB1Clk_ConfigkHz(uint32_t Clk)
{
	uint32_t sysclk = RCC_GetSystemClk();
	uint32_t apb1;
	
	RCC->CFGR &= ~(0x7 << 10);

	if(Clk > 45000)
	{
		Clk = 45000;
	}
	if(sysclk > Clk)
	{
		while((sysclk/APBprescalar[apb1] > Clk) && (apb1<3)) apb1++;
		RCC->CFGR |= ((apb1+4) << 10);	//APB1 prescaler
	}
}

void RCC_APB2Clk_ConfigkHz(uint32_t Clk)
{
	uint32_t sysclk = RCC_GetSystemClk();
	uint32_t apb2;
	
	RCC->CFGR &= ~(0x7 << 13);

	if(Clk > 90000)
	{
		Clk = 90000;
	}
	if(sysclk > Clk)
	{
		while((sysclk/APBprescalar[apb2] > Clk) && (apb2<3)) apb2++;
		RCC->CFGR |= ((apb2+4) << 10);	//APB2 prescaler
	}
}

/*
 *
 */

uint32_t RCC_GetPClk1(void)
{
	uint32_t pclk1, sysclk;
	uint16_t apbp1, ahbp1, temp;

	sysclk = RCC_GetSystemClk();

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

	sysclk = RCC_GetSystemClk();

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
