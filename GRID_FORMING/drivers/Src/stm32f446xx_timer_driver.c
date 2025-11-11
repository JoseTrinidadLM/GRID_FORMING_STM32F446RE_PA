/*
 * stm32f446_timer_driver.c
 *
 *  Created on: Oct 14, 2025
 *      Author: jiperez
 */

#include "stm32f446xx_timer_driver.h"

/************************************************************************************
 * @fn				- TIM_PeriClockControl
 *
 * @brief			- This function enables or disables peripheral clock for the given TIMER
 *
 * @param[in]		- base address of the TIM
 * @param[in]		- ENABLE or DISABLE macros
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			- none
 *
 * */
void TIM_PClkC(TIM_RegDef_t *pTIMx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if (pTIMx == TIM1) TIM1_PCLK_EN();
		else if (pTIMx == TIM2) TIM2_PCLK_EN();
		else if (pTIMx == TIM3) TIM3_PCLK_EN();
		else if (pTIMx == TIM4) TIM4_PCLK_EN();
		else if (pTIMx == TIM5) TIM5_PCLK_EN();
		else if (pTIMx == TIM6) TIM6_PCLK_EN();
		else if (pTIMx == TIM7) TIM7_PCLK_EN();
		else if (pTIMx == TIM8) TIM8_PCLK_EN();
		else if (pTIMx == TIM9) TIM9_PCLK_EN();
		else if (pTIMx == TIM10) TIM10_PCLK_EN();
		else if (pTIMx == TIM11) TIM11_PCLK_EN();
		else if (pTIMx == TIM12) TIM12_PCLK_EN();
		else if (pTIMx == TIM13) TIM13_PCLK_EN();
		else if (pTIMx == TIM14) TIM14_PCLK_EN();
	}
	else
	{
		if (pTIMx == TIM1) TIM1_PCLK_DI();
		else if (pTIMx == TIM2) TIM2_PCLK_DI();
		else if (pTIMx == TIM3) TIM3_PCLK_DI();
		else if (pTIMx == TIM4) TIM4_PCLK_DI();
		else if (pTIMx == TIM5) TIM5_PCLK_DI();
		else if (pTIMx == TIM6) TIM6_PCLK_DI();
		else if (pTIMx == TIM7) TIM7_PCLK_DI();
		else if (pTIMx == TIM8) TIM8_PCLK_DI();
		else if (pTIMx == TIM9) TIM9_PCLK_DI();
		else if (pTIMx == TIM10) TIM10_PCLK_DI();
		else if (pTIMx == TIM11) TIM11_PCLK_DI();
		else if (pTIMx == TIM12) TIM12_PCLK_DI();
		else if (pTIMx == TIM13) TIM13_PCLK_DI();
		else if (pTIMx == TIM14) TIM14_PCLK_DI();
	}
}

/************************************************************************************
 * @fn				- @TIM_Init
 *
 * @brief			- This function initializes given TIMER
 *
 * @param[in]		- pointer to TIMER handle
 *
 * @return			- none
 *
 * @Note			- none
 *
 * */

void TIM_Init(TIM_Handle_t *pTIMHandle)
{
	TIM_PClkC(pTIMHandle->pTIMx, ENABLE);
	uint64_t f_CK_PSC = 0;
	if((pTIMHandle->pTIMx == TIM2)||(pTIMHandle->pTIMx == TIM3)||
			(pTIMHandle->pTIMx == TIM4)||(pTIMHandle->pTIMx == TIM5)||
			(pTIMHandle->pTIMx == TIM6)||(pTIMHandle->pTIMx == TIM7)||
			(pTIMHandle->pTIMx == TIM12)||(pTIMHandle->pTIMx == TIM13)||
			(pTIMHandle->pTIMx == TIM14)){

		f_CK_PSC = 84000000;

	} else{

		f_CK_PSC = 84000000;

	}


	uint64_t InputFrequency = pTIMHandle->TIM_Config.TIM_Frequency;
	//The counter clock frequency CK_CNT is equal to fCK_PSC / (PSC[15:0] + 1). (RM PG[604])
	//In upcounting mode, the counter counts from 0 to the auto-reload value
	//fCK_PSC = APB1_Timer_Clock = 84 MHz
	//FrequencyRatio = fCK_PSC/InputFrequency = (PSC + 1)(ARR + 1)
	uint32_t psc_temp = 0;
	uint32_t arr_temp = 0;
	//Max value ARR can count up to is 0xFFFF
	arr_temp = (((f_CK_PSC*1000)/InputFrequency) / (psc_temp + 1) ) - 1;
	while( arr_temp > 0xFFFF ){
		psc_temp++ ;
		arr_temp = (((f_CK_PSC*1000)/InputFrequency) / (psc_temp + 1) ) - 1;
	}

	pTIMHandle->pTIMx->PSC = psc_temp; //Setting PSC to calculated PSC

	pTIMHandle->pTIMx->ARR = arr_temp; //Setting ARR to calculated ARR

	pTIMHandle->pTIMx->CR1 &= ~( 0x1FF << 0 ); //Clear ALL
	uint32_t cr1_temp = 0x0000;
	cr1_temp |= (pTIMHandle->TIM_Config.TIM_CLKDivision << 8);
	cr1_temp |= (pTIMHandle->TIM_Config.TIM_AutoReloadPreload << 7);
	cr1_temp |= (pTIMHandle->TIM_Config.TIM_CAModeSel << 5);
	cr1_temp |= (pTIMHandle->TIM_Config.TIM_CNTMode << 4);
	pTIMHandle->pTIMx->CR1 |= cr1_temp; //set register to TIMx configuration
	for(__vo uint16_t j = 0; j < 5000; j++);

    if(pTIMHandle->TIM_Config.TIM_Mode == TIM_MODE_PWM) {
        TIM_PWM_Channel_Init(pTIMHandle);
    }

    pTIMHandle->pTIMx->SMCR &= ~(0x7 << 0);  // Clear SMS
    pTIMHandle->pTIMx->SMCR &= ~(0x7 << 4);  // Clear TS
    pTIMHandle->pTIMx->SMCR &= ~(0xF << 8);  // Clear ETF

    pTIMHandle->pTIMx->SMCR |= (pTIMHandle->TIM_Config.TIM_SlaveMode << 0);

    pTIMHandle->pTIMx->SMCR |= (pTIMHandle->TIM_Config.TIM_TriggerSource << 4);

	pTIMHandle->pTIMx->CR2 &= ~( 0x1F << 3 ); //clear

	pTIMHandle->pTIMx->CR2 |= ( pTIMHandle->TIM_Config.TIM_MasterModeSel << 4 );

	for(__vo uint16_t j = 0; j < 50000; j++);
	if(pTIMHandle->TIM_Config.TIM_IntEnable == TIM_IT_ENABLE) pTIMHandle->pTIMx->DIER |= ( 1 << 0 );
	else pTIMHandle->pTIMx->DIER &= ~( 1 << 0 );
	for(__vo uint16_t j = 0; j < 50000; j++);
}
/************************************************************************************
 * @fn				- @TIM_DeInit
 *
 * @brief			- This function de-initializes given TIMER
 *
 * @param[in]		- pointer to TIMER handle
 *
 * @return			- none
 *
 * @Note			- none
 *
 * */
void TIM_DeInit(TIM_RegDef_t *pTIMx)
{
	if (pTIMx == TIM1) TIM1_REG_RESET();
	else if (pTIMx == TIM2) TIM2_REG_RESET();
	else if (pTIMx == TIM3) TIM3_REG_RESET();
	else if (pTIMx == TIM4) TIM4_REG_RESET();
	else if (pTIMx == TIM5) TIM5_REG_RESET();
	else if (pTIMx == TIM6) TIM6_REG_RESET();
	else if (pTIMx == TIM7) TIM7_REG_RESET();
	else if (pTIMx == TIM8) TIM8_REG_RESET();
	else if (pTIMx == TIM9) TIM9_REG_RESET();
	else if (pTIMx == TIM10) TIM10_REG_RESET();
	else if (pTIMx == TIM11) TIM11_REG_RESET();
	else if (pTIMx == TIM12) TIM12_REG_RESET();
	else if (pTIMx == TIM13) TIM13_REG_RESET();
	else if (pTIMx == TIM14) TIM14_REG_RESET();
}


/************************************************************************************
 * @fn				- @TIM_Start
 *
 * @brief			- This function starts counter
 *
 * @param[in]		- pointer to TIMER handle
 *
 * @return			- none
 *
 * @Note			- none
 *
 * */
void TIM_Start(TIM_Handle_t *pTIMHandle)
{
	pTIMHandle->pTIMx->CR1 |= (1 << 0);
}

/************************************************************************************
 * @fn				- @TIM_Stop
 *
 * @brief			- This function stops counter
 *
 * @param[in]		- pointer to TIMER handle
 *
 * @return			- none
 *
 * @Note			- none
 *
 * */
void TIM_Stop(TIM_Handle_t *pTIMHandle)
{
	pTIMHandle->pTIMx->CR1 &= ~(1 << 0);
}

void TIM_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		*NVIC_ISER[IRQNumber/32] |= (1<<IRQNumber%32);
	}else
	{
		*NVIC_ISER[IRQNumber/32] |= (1<<IRQNumber%32);
	}

}

/************************************************************************************
 * @fn				- TIM_IRQPriorityConfig
 *
 * @brief			- Configures IRQ priority for TIMERS
 *
 * @param[in]		- IRQ_NO and IRQ Priority Macros
 *
 * @return			- none
 *
 * @Note			- none
 *
 * */
void TIM_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	//1. find the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED );
	*(NVIC_PR_BASE_ADDR + (iprx*4)) |= ( IRQPriority << shift_amount );
}

/************************************************************************************
 * @fn				- TIM_IRQHandling
 *
 * @brief			- Handles/clears detected IRQ flag
 *
 * @param[in]		- pointer to TIM handle
 *
 * @return			- none
 *
 * @Note			- none
 *
 * */
void TIM_IRQHandling(TIM_Handle_t *pTIMHandle)
{
	if(pTIMHandle->pTIMx->SR & (1 <<0)) pTIMHandle->pTIMx->SR &= ~(1 << 0);
}

void TIM_PWM_Channel_Init(TIM_Handle_t *pTIMHandle){

	if(pTIMHandle->TIM_Config.TIM_Channel == TIM_CHANNEL_1){
		pTIMHandle->pTIMx->CCR1 = 0;
		pTIMHandle->pTIMx->CCMR1 &= ~(0xFF << 0);		//Only clear channel 1

        pTIMHandle->pTIMx->CCMR1 |= (pTIMHandle->TIM_Config.TIM_OCMode << 4);

        if(pTIMHandle->TIM_Config.TIM_OCPreload)
            pTIMHandle->pTIMx->CCMR1 |= (1 << 3);

        pTIMHandle->pTIMx->CCMR1 &= ~(0x3 << 0); //To secure output

        pTIMHandle->pTIMx->CCER &= ~(0xB << 0); //Clean b3,b1,b0
        pTIMHandle->pTIMx->CCER |= (1 << 0);
        if(pTIMHandle->TIM_Config.TIM_OCPolarity)
            pTIMHandle->pTIMx->CCER |= (1 << 1); //Active low

	}else if((pTIMHandle->TIM_Config.TIM_Channel == TIM_CHANNEL_2)){
		pTIMHandle->pTIMx->CCR2 = 0;
		pTIMHandle->pTIMx->CCMR1 &= ~(0xFF << 8);		//Only clear channel 2

        pTIMHandle->pTIMx->CCMR1 |= (pTIMHandle->TIM_Config.TIM_OCMode << 12);

        if(pTIMHandle->TIM_Config.TIM_OCPreload)
            pTIMHandle->pTIMx->CCMR1 |= (1 << 11);

        pTIMHandle->pTIMx->CCMR1 &= ~(0x3 << 8); //To secure output

        pTIMHandle->pTIMx->CCER &= ~(0xB << 4); //Clean b7,b5,b4
        pTIMHandle->pTIMx->CCER |= (1 << 4);
        if(pTIMHandle->TIM_Config.TIM_OCPolarity)
            pTIMHandle->pTIMx->CCER |= (1 << 5); //Active low


	}else if((pTIMHandle->TIM_Config.TIM_Channel == TIM_CHANNEL_3)){
		pTIMHandle->pTIMx->CCR3 = 0;

		pTIMHandle->pTIMx->CCMR2 &= ~(0xFF << 0);		//Only clear channel 3

        pTIMHandle->pTIMx->CCMR2 |= (pTIMHandle->TIM_Config.TIM_OCMode << 4);

        if(pTIMHandle->TIM_Config.TIM_OCPreload)
            pTIMHandle->pTIMx->CCMR2 |= (1 << 3);

        pTIMHandle->pTIMx->CCMR2 &= ~(0x3 << 0); //To secure output

        pTIMHandle->pTIMx->CCER &= ~(0xB << 8); //Clean b11,b9,b8
        pTIMHandle->pTIMx->CCER |= (1 << 8);
        if(pTIMHandle->TIM_Config.TIM_OCPolarity)
            pTIMHandle->pTIMx->CCER |= (1 << 9); //Active low

	}else if((pTIMHandle->TIM_Config.TIM_Channel == TIM_CHANNEL_4)){
		pTIMHandle->pTIMx->CCR4 = 0;

		pTIMHandle->pTIMx->CCMR2 &= ~(0xFF << 8);		//Only clear channel 4

        pTIMHandle->pTIMx->CCMR2 |= (pTIMHandle->TIM_Config.TIM_OCMode << 12);

        if(pTIMHandle->TIM_Config.TIM_OCPreload)
            pTIMHandle->pTIMx->CCMR2 |= (1 << 11);

        pTIMHandle->pTIMx->CCMR2 &= ~(0x3 << 8); //To secure output

        pTIMHandle->pTIMx->CCER &= ~(0xB << 12); //Clean b15,b13,b12
        pTIMHandle->pTIMx->CCER |= (1 << 12);
        if(pTIMHandle->TIM_Config.TIM_OCPolarity)
            pTIMHandle->pTIMx->CCER |= (1 << 13); //Active low

	}
}


void TIM_PWM_DutyCycle(TIM_Handle_t *pTIMHandle, uint16_t dutyCycle){

	if(pTIMHandle->TIM_Config.TIM_Channel == TIM_CHANNEL_1) pTIMHandle->pTIMx->CCR1 = dutyCycle;
	if(pTIMHandle->TIM_Config.TIM_Channel == TIM_CHANNEL_2) pTIMHandle->pTIMx->CCR2 = dutyCycle;
	if(pTIMHandle->TIM_Config.TIM_Channel == TIM_CHANNEL_3)  pTIMHandle->pTIMx->CCR3 = dutyCycle;
	if(pTIMHandle->TIM_Config.TIM_Channel == TIM_CHANNEL_4)  pTIMHandle->pTIMx->CCR4 = dutyCycle;
}

void TIM_PWM_Enable(TIM_Handle_t *pTIMHandle)
{
	pTIMHandle->pTIMx->CCER |= (1 << ((pTIMHandle->TIM_Config.TIM_Channel - 1 )*4));
}

void TIM_PWM_Disable(TIM_Handle_t *pTIMHandle)
{
	pTIMHandle->pTIMx->CCER &= ~(1 << ((pTIMHandle->TIM_Config.TIM_Channel - 1 )*4));
}

/*
void TIM_DMAConfig(TIM_RegDef_t *pTIMx)
{
	pTIMx->DIER &= ~( 1 << 0 );
	pTIMx->DIER |= ( 1 << 8 );
}
*/
