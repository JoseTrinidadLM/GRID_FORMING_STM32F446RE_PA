/*
 * stm32f446_dma_driver.c
 *
 *  Created on: Oct 20, 2025
 *      Author: jiperez
 */

#include "stm32f446xx_dma_driver.h"


/************************************************************************************
 * @fn				- DMA_PeriClockControl
 *
 * @brief			- This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]		- base address of the gpio peripheral
 * @param[in]		- ENABLE or DISABLE macros
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			- none
 *
 * */
void DMA_PClkC(DMA_RegDef_t *pDMAx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if (pDMAx == DMA1) DMA1_PCLK_EN();
		else if (pDMAx == DMA2) DMA2_PCLK_EN();
	}
	else
	{
		if (pDMAx == DMA1) DMA1_PCLK_DI();
		else if (pDMAx == DMA2) DMA2_PCLK_DI();
	}
}

/************************************************************************************
 * @fn				- DMA_PeriClockControl
 *
 * @brief			- This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]		- base address of the gpio peripheral
 * @param[in]		- ENABLE or DISABLE macros
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			- none
 *
 * */
void DMA_Init(DMA_Handle_t *pDMAHandle)
{
	DMA_PClkC(pDMAHandle->pDMAx, ENABLE);
	DMA_StopTransfer(pDMAHandle);
	DMA_ClearFlags(pDMAHandle);
	pDMAHandle->pDMAx->STREAM[pDMAHandle->DMA_stream].CR &= ~(0x0FEFFFFF);
	pDMAHandle->pDMAx->STREAM[pDMAHandle->DMA_stream].NDTR &= ~(0xFFFF);
	pDMAHandle->pDMAx->STREAM[pDMAHandle->DMA_stream].PAR = 0;
	pDMAHandle->pDMAx->STREAM[pDMAHandle->DMA_stream].M0AR = 0;
	pDMAHandle->pDMAx->STREAM[pDMAHandle->DMA_stream].M1AR = 0;
	pDMAHandle->pDMAx->STREAM[pDMAHandle->DMA_stream].FCR &= ~(0xBF);

	uint32_t cr_temp = 0;
	cr_temp |= ( pDMAHandle->DMA_Config.DMA_Channel << 25 );
	cr_temp |= ( pDMAHandle->DMA_Config.DMA_Priority << 16 );
	cr_temp |= ( pDMAHandle->DMA_Config.DMA_MemDataSize << 13 );
	cr_temp |= ( pDMAHandle->DMA_Config.DMA_PeriphDataSize << 11 );
	cr_temp |= ( pDMAHandle->DMA_Config.DMA_MemInc << 10 );
	cr_temp |= ( pDMAHandle->DMA_Config.DMA_PeriphInc << 9 );
	cr_temp |= ( pDMAHandle->DMA_Config.DMA_Mode << 8 );
	cr_temp |= ( pDMAHandle->DMA_Config.DMA_Direction << 6 );
	cr_temp |= ( pDMAHandle->DMA_Config.DMA_TransferIT << 4 );

	pDMAHandle->pDMAx->STREAM[pDMAHandle->DMA_stream].CR |= cr_temp;

	uint32_t fcr_temp = 0;
	fcr_temp |= ( pDMAHandle->DMA_Config.DMA_FIFOMode << 2);
	fcr_temp |= ( pDMAHandle->DMA_Config.DMA_FIFOThreshold << 0);

	pDMAHandle->pDMAx->STREAM[pDMAHandle->DMA_stream].FCR |= fcr_temp;

	pDMAHandle->pDMAx->STREAM[pDMAHandle->DMA_stream].NDTR |= (pDMAHandle->BufferSize << 0);
}

/************************************************************************************
 * @fn				- DMA_PeriClockControl
 *
 * @brief			- This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]		- base address of the gpio peripheral
 * @param[in]		- ENABLE or DISABLE macros
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			- none
 *
 * */
void DMA_DeInit(DMA_RegDef_t *pDMAx)
{
	if (pDMAx == DMA1) DMA1_REG_RESET();
	else if (pDMAx == DMA2) DMA2_REG_RESET();

}

/************************************************************************************
 * @fn				- DMA_PeriClockControl
 *
 * @brief			- This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]		- base address of the gpio peripheral
 * @param[in]		- ENABLE or DISABLE macros
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			- none
 *
 * */
void DMA_StartTransfer(DMA_Handle_t *pDMAHandle)
{
	pDMAHandle->pDMAx->STREAM[pDMAHandle->DMA_stream].CR |= ( 1 << 0);
}

/************************************************************************************
 * @fn				- DMA_PeriClockControl
 *
 * @brief			- This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]		- base address of the gpio peripheral
 * @param[in]		- ENABLE or DISABLE macros
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			- none
 *
 * */
void DMA_StopTransfer(DMA_Handle_t *pDMAHandle)
{
	pDMAHandle->pDMAx->STREAM[pDMAHandle->DMA_stream].CR &= ~( 1<< 0);
}

/************************************************************************************
 * @fn				- DMA_PeriClockControl
 *
 * @brief			- This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]		- base address of the gpio peripheral
 * @param[in]		- ENABLE or DISABLE macros
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			- none
 *
 * */
void DMA_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
 * @fn				- DMA_PeriClockControl
 *
 * @brief			- This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]		- base address of the gpio peripheral
 * @param[in]		- ENABLE or DISABLE macros
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			- none
 *
 * */
void DMA_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	//1. find the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED );
	*(NVIC_PR_BASE_ADDR + (iprx*4)) |= ( IRQPriority << shift_amount );
}

/************************************************************************************
 * @fn				- DMA_PeriClockControl
 *
 * @brief			- This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]		- base address of the gpio peripheral
 * @param[in]		- ENABLE or DISABLE macros
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			- none
 *
 * */
void DMA_IRQHandling(DMA_Handle_t *pDMAHandle)
{
	uint8_t stream = pDMAHandle->DMA_stream;
	const uint8_t TCIF_BITS[8] = {5, 11, 21, 27, 5, 11, 21, 27};
	const uint8_t HTIF_BITS[8] = {4, 10, 20, 26, 4, 10, 20, 26};
	const uint8_t TEIF_BITS[8] = {3, 9, 19, 25, 3, 9, 19, 25};
	const uint8_t DMEIF_BITS[8] = {2, 8, 18, 24, 2, 8, 18, 24};
	const uint8_t FEIF_BITS[8] = {0, 6, 16, 22, 0, 6, 16, 22};

	if(stream <= 3)
	{
		if( pDMAHandle->pDMAx->LISR & ( 1 << (TCIF_BITS[stream]) )) pDMAHandle->pDMAx->LIFCR |= ( 1 << (TCIF_BITS[stream]) );
		if( pDMAHandle->pDMAx->LISR & ( 1 << (HTIF_BITS[stream]) )) pDMAHandle->pDMAx->LIFCR |= ( 1 << (HTIF_BITS[stream]) );
		if( pDMAHandle->pDMAx->LISR & ( 1 << (TEIF_BITS[stream]) )) pDMAHandle->pDMAx->LIFCR |= ( 1 << (TEIF_BITS[stream]) );
		if( pDMAHandle->pDMAx->LISR & ( 1 << (DMEIF_BITS[stream]) )) pDMAHandle->pDMAx->LIFCR |= ( 1 << (DMEIF_BITS[stream]) );
		if( pDMAHandle->pDMAx->LISR & ( 1 << (FEIF_BITS[stream]) )) pDMAHandle->pDMAx->LIFCR |= ( 1 << (FEIF_BITS[stream]) );
	}else
	{
		if( pDMAHandle->pDMAx->HISR & ( 1 << (TCIF_BITS[stream]) )) pDMAHandle->pDMAx->HIFCR |= ( 1 << (TCIF_BITS[stream]) );
		if( pDMAHandle->pDMAx->HISR & ( 1 << (HTIF_BITS[stream]) )) pDMAHandle->pDMAx->HIFCR |= ( 1 << (HTIF_BITS[stream]) );
		if( pDMAHandle->pDMAx->HISR & ( 1 << (TEIF_BITS[stream]) )) pDMAHandle->pDMAx->HIFCR |= ( 1 << (TEIF_BITS[stream]) );
		if( pDMAHandle->pDMAx->HISR & ( 1 << (DMEIF_BITS[stream]) )) pDMAHandle->pDMAx->HIFCR |= ( 1 << (DMEIF_BITS[stream]) );
		if( pDMAHandle->pDMAx->HISR & ( 1 << (FEIF_BITS[stream]) )) pDMAHandle->pDMAx->HIFCR |= ( 1 << (FEIF_BITS[stream]) );

	}
}

/************************************************************************************
 * @fn				- DMA_PeriClockControl
 *
 * @brief			- This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]		- base address of the gpio peripheral
 * @param[in]		- ENABLE or DISABLE macros
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			- none
 *
 * */
void DMA_ClearFlags(DMA_Handle_t *pDMAHandle)
{
	uint8_t stream = pDMAHandle->DMA_stream;
	if(stream <= 3)
	{
		pDMAHandle->pDMAx->LIFCR |= ( 0x3D << 6*stream );
	}else
	{
		pDMAHandle->pDMAx->HIFCR |= ( 0x3D << 6*(stream - 4) );
	}
}

/************************************************************************************
 * @fn				- DMA_PeriClockControl
 *
 * @brief			- This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]		- base address of the gpio peripheral
 * @param[in]		- ENABLE or DISABLE macros
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			- none
 *
 * */
void DMA_SetAddresses(DMA_Handle_t *pDMAHandle, void *pSrc, void *pDest)
{

	pDMAHandle->pSourceAddr = (uint32_t)pSrc;
	pDMAHandle->pDestAddr = (uint32_t)pDest;

	if( !(pDMAHandle->pDMAx->STREAM[pDMAHandle->DMA_stream].CR & ( 1 << 0 )) )
	{
		pDMAHandle->pDMAx->STREAM[pDMAHandle->DMA_stream].PAR = pDMAHandle->pSourceAddr;
		pDMAHandle->pDMAx->STREAM[pDMAHandle->DMA_stream].M0AR = pDMAHandle->pDestAddr;
	}
}

/************************************************************************************
 * @fn				- DMA_PeriClockControl
 *
 * @brief			- This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]		- base address of the gpio peripheral
 * @param[in]		- ENABLE or DISABLE macros
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			- none
 *
 * */
uint8_t DMA_GetTransferStatus(DMA_Handle_t *pDMAHandle)
{
	uint8_t stream = pDMAHandle->DMA_stream;
	uint32_t status;
	const uint8_t TCIF_BITS[8] = {5, 11, 21, 27, 5, 11, 21, 27};
	const uint8_t TEIF_BITS[8] = {3, 9, 19, 25, 3, 9, 19, 25};
	if(stream <= 3)
	{
		status = pDMAHandle->pDMAx->LISR;
	}else
	{
		status = pDMAHandle->pDMAx->HISR;
	}

	if( status & ( 1 << (TEIF_BITS[stream]) ) ) return 2;
	else if ( status & ( 1 << (TCIF_BITS[stream]) ) ) return 1;
	else return 0;
}
