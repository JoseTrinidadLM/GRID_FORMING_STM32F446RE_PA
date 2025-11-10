/*
 * stm32f446_adc_driver.c
 *
 *  Created on: Oct 21, 2025
 *      Author: jiperez
 */


#include "stm32f446xx_adc_driver.h"

/************************************************************************************
 * @fn				- ADC_PeriClockControl
 *
 * @brief			- This function enables or disables peripheral clock for the given ADC
 *
 * @param[in]		- base address of the ADC peripheral
 * @param[in]		- ENABLE or DISABLE macros
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			- none
 *
 * */
void ADC_PClkC(ADC_RegDef_t *pADCx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if (pADCx == ADC1) ADC1_PCLK_EN();
		else if (pADCx == ADC2) ADC2_PCLK_EN();
		else if (pADCx == ADC3) ADC3_PCLK_EN();
	}
	else
	{
		if (pADCx == ADC1) ADC1_PCLK_DI();
		else if (pADCx == ADC2) ADC2_PCLK_DI();
		else if (pADCx == ADC3) ADC3_PCLK_DI();	}
}

/************************************************************************************
 * @fn				- ADC_Init
 *
 * @brief			- This function initializes ADC
 *
 * @param[in]		- base address of the ADC handle
 *
 * @return			- none
 *
 * @Note			- none
 *
 * */
void ADC_Init(ADC_Handle_t *pADCHandle)
{
	ADC_PClkC(pADCHandle->pADCx, ENABLE);
	for(__vo uint16_t i = 0; i < 1000; i++);

	pADCHandle->pADCx->SR &= ~(0x3F << 0);

	pADCHandle->pADCx->CR1 &= ~(( 0x1F << 22 ) | ( 0xFFFF << 0 ));
	pADCHandle->pADCx->CR2 &= ~(( 0x7F << 24 ) | ( 0x7F << 16 ) | ( 0xF << 8 ) | (0x3 << 0));

	pADCHandle->pADCx->CR2 |= (1 << 0);

	for(__vo uint16_t i = 0; i < 2000; i++);

	pADCHandle->pADCx->CR1 |= (pADCHandle->ADC_Config.ADC_Resolution << 24);

	if(pADCHandle->ADC_Config.ADC_ConversionMode == ADC_CONV_MODE_DISCONTINUOUS)
	{
		pADCHandle->pADCx->CR1 |= (1 << 11);
		pADCHandle->pADCx->CR2 &= ~(1 << 1);
	}else if(pADCHandle->ADC_Config.ADC_ConversionMode == ADC_CONV_MODE_CONTINUOUS)
	{
		pADCHandle->pADCx->CR1 &= ~(1 << 11);
		pADCHandle->pADCx->CR2 |= (1 << 1);
	}else if(pADCHandle->ADC_Config.ADC_ConversionMode == ADC_CONV_MODE_SINGLE)
	{
		pADCHandle->pADCx->CR1 &= ~(1 << 11);
		pADCHandle->pADCx->CR2 &= ~(1 << 1);
	}

	pADCHandle->pADCx->CR1 |= (pADCHandle->ADC_Config.ADC_ScanMode << 8);

	pADCHandle->pADCx->CR1 |= (pADCHandle->ADC_Config.ADC_EOCInterrupt << 5);

	pADCHandle->pADCx->CR2 |= (pADCHandle->ADC_Config.ADC_ExternalTriggerDetection << 28);

	pADCHandle->pADCx->CR2 |= (pADCHandle->ADC_Config.ADC_ExternalTrigger << 24);

	pADCHandle->pADCx->CR2 |= (pADCHandle->ADC_Config.ADC_DataAlignment << 11);

	pADCHandle->pADCx->CR2 |= (pADCHandle->ADC_Config.ADC_EOCSelection << 10);

	pADCHandle->pADCx->CR2 |= (pADCHandle->ADC_Config.ADC_DDSelection << 9);

	pADCHandle->pADCx->CR2 |= (pADCHandle->ADC_Config.ADC_DMAContinuousRequests << 8);

	while(!(pADCHandle->pADCx->CR2 & (1<<0)));

	for(__vo uint16_t i = 0; i < 1000; i++);
}


/************************************************************************************
 * @fn				- ADC_DeInit
 *
 * @brief			- This function resets all ADCs
 *
 * @param[in]		- none
 *
 * @return			- none
 *
 * @Note			- none
 *
 * */
void ADC_DeInit(void)
{
	ADC_REG_RESET();
}

void ADC_ChannelConfig(ADC_Handle_t *pADCHandle, uint8_t channel, uint8_t rank, uint8_t samplingTime)
{
	//rank goes the same as index 0-15, the closer to 0 the higher the rank
	pADCHandle->ADC_Channels[rank] = channel;
	pADCHandle->ADC_SamplingTime[rank] = samplingTime;

	if( channel <= 9 )
	{
		pADCHandle->pADCx->SMPR2 |= ( samplingTime << 3*channel  );
	} else if ( (channel >= 10) && (channel <= 18) )
	{
		pADCHandle->pADCx->SMPR1 |= ( samplingTime << 3*(channel - 10 )  );
	}

}

void ADC_ConfigSequence(ADC_Handle_t *pADCHandle)
{
	pADCHandle->pADCx->SQR3 &= ~( 0x3FFFFFFF << 0 );
	pADCHandle->pADCx->SQR2 &= ~( 0x3FFFFFFF << 0 );
	pADCHandle->pADCx->SQR1 &= ~( 0xFFFFFF << 0 );

	pADCHandle->pADCx->SQR1 |= ( ( pADCHandle->ADC_NumChannels - 1 ) << 20 );

	for(uint8_t i = 0; i < pADCHandle->ADC_NumChannels; i++ )
	{
		uint8_t sequence_position = i + 1;
		if( sequence_position <= 6 ) pADCHandle->pADCx->SQR3 |= ( ( pADCHandle->ADC_Channels[i] ) << 5*(sequence_position - 1) );
		else if( sequence_position <= 12 ) pADCHandle->pADCx->SQR2 |= ( ( pADCHandle->ADC_Channels[i] ) << 5*(sequence_position - 7) );
		else if( sequence_position <= 16 ) pADCHandle->pADCx->SQR1 |= ( ( pADCHandle->ADC_Channels[i] ) << 5*(sequence_position - 13) );
	}
}


/************************************************************************************
 * @fn				- ADC_StartConversion
 *
 * @brief			- This function starts conversion of regular channels
 *
 * @param[in]		- base address of the ADC handle
 *
 * @return			- none
 *
 * @Note			- none
 *
 * */
void ADC_StartConversion(ADC_Handle_t *pADCHandle)
{
	pADCHandle->pADCx->CR2 |= ( 1 << 30 );
}

/************************************************************************************
 * @fn				- ADC_StartConversion
 *
 * @brief			- This function stops conversion of regular channels
 *
 * @param[in]		- base address of the ADC handle
 *
 * @return			- none
 *
 * @Note			- none
 *
 * */
void ADC_StopConversion(ADC_Handle_t *pADCHandle)
{
	pADCHandle->pADCx->CR2 &= ~( 1 << 30 );
}

/************************************************************************************
 * @fn				- ADC_ReadData
 *
 * @brief			- This reads
 *
 * @param[in]		- base address of the ADC handle
 *
 * @return			- none
 *
 * @Note			- none
 *
 * */
uint16_t ADC_ReadData(ADC_Handle_t *pADCHandle)
{
	return pADCHandle->pADCx->DR;
}

/************************************************************************************
 * @fn				- ADC_GetConversionStatus
 *
 * @brief			- This function resets all ADCs
 *
 * @param[in]		- base address of the ADC handle
 *
 * @return			- none
 *
 * @Note			- none
 *
 * */
uint8_t ADC_GetConversionStatus(ADC_Handle_t *pADCHandle)
{

	if ( pADCHandle->pADCx->SR & (1 << 1) ) return 1;
	else return 0;
}


void ADC_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		*NVIC_ISER[IRQNumber/32] |= (1<<IRQNumber%32);
	}else
	{
		*NVIC_ISER[IRQNumber/32] |= (1<<IRQNumber%32);
	}

}
void ADC_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	//1. find the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED );
	*(NVIC_PR_BASE_ADDR + (iprx*4)) |= ( IRQPriority << shift_amount );
}


void ADC_IRQHandling(ADC_Handle_t *pADCHandle)
{
	if ( pADCHandle->pADCx->SR & ( 1 << 0) ) pADCHandle->pADCx->SR &= ~( 1 << 0 )  ; //analog watchdog flag
	if ( pADCHandle->pADCx->SR & ( 1 << 1) ) pADCHandle->pADCx->SR &= ~( 1 << 1 )  ; //Regular channel end of conversion flag
	if ( pADCHandle->pADCx->SR & ( 1 << 2) ) pADCHandle->pADCx->SR &= ~( 1 << 2 )  ; //Injected channel end of conversion flag
	if ( pADCHandle->pADCx->SR & ( 1 << 3) ) pADCHandle->pADCx->SR &= ~( 1 << 3 )  ; //Injected channel start flag
	if ( pADCHandle->pADCx->SR & ( 1 << 4) ) pADCHandle->pADCx->SR &= ~( 1 << 4 )  ; //Regular channel start flag
	if ( pADCHandle->pADCx->SR & ( 1 << 5) ) pADCHandle->pADCx->SR &= ~( 1 << 5 )  ; //Overrun flag

}

/************************************************************************************
 * @fn				- ADC_DMAControl
 *
 * @brief			- This function enables or disables DMA mode of ADC
 *
 * @param[in]		- base address of the ADC handle
 *
 * @return			- none
 *
 * @Note			- none
 *
 * */
void ADC_DMAControl(ADC_Handle_t *pADCHandle, uint8_t EnorDi)
{
	if ( EnorDi == ENABLE ) pADCHandle->pADCx->CR2 |= ( 1 << 8 );
	else pADCHandle->pADCx->CR2 &= ~( 1 << 8 );
}

/************************************************************************************
 * @fn				- ADC_ConfigWatchdog
 *
 * @brief			- This function configures Watchdog
 *
 * @param[in]		- base address of the ADC handle
 *
 * @param[in]		- lower limit
 *
 *  @param[in]		- upper limit
 *
 * @return			- none
 *
 * @Note			- none
 *
 * */
void ADC_ConfigWatchdog(ADC_Handle_t *pADCHandle, uint16_t LowerLimit , uint16_t UpperLimit)
{
	pADCHandle->pADCx->LTR |= (LowerLimit << 0);
	pADCHandle->pADCx->HTR |= (UpperLimit << 0);
	pADCHandle->pADCx->CR1 |= (1 << 23);
}

void ADC_SoftReset(ADC_Handle_t *pADCHandle){
	pADCHandle->pADCx->CR2 &= ~(1 << 0);
	pADCHandle->pADCx->CR2 |= (1 << 0);
	for(__vo uint16_t i = 0; i<1000; i++);
}
