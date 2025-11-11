/*
 * stm32f446_dma_driver.c
 *
 *  Created on: Oct 20, 2025
 *      Author: jiperez
 */

#include "stm32f446xx_dma_driver.h"

const uint8_t FLAGS_BITS[5][4] = {{5, 11, 21, 27}, {4, 10, 20, 26} ,{3, 9, 19, 25} ,{2, 8, 18, 24} ,{0, 6, 16, 22}};

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
	pDMAHandle->pDMAx->STREAM[pDMAHandle->DMA_stream].CR = 0;
	pDMAHandle->pDMAx->STREAM[pDMAHandle->DMA_stream].NDTR &= ~(0xFFFF);
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
void DMA_ConfigureBuffer(DMA_Handle_t *pDMAHandle, uint8_t BufferSize)
{
	pDMAHandle->pDMAx->STREAM[pDMAHandle->DMA_stream].NDTR |= (BufferSize << 0);
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
		*NVIC_ICER[IRQNumber/32] |= (1<<IRQNumber%32);
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
	uint8_t h_l = stream/4;
	uint8_t pos = stream%4;

/*************************Check for Transfer Complete flag **************************/

	if( pDMAHandle->pDMAx->ISR[h_l] & ( 1 << FLAGS_BITS[DMA_TCIF_FLAG][pos] ))
	{
		DMA_ClearFlag(pDMAHandle,DMA_TCIF_FLAG);
		DMA_ApplicationEventCallback(pDMAHandle,DMA_EVENT_TCIF_CMPLT);
	}

/****************************Check for Half Transfer flag ***************************/

	if( pDMAHandle->pDMAx->ISR[h_l] & ( 1 << FLAGS_BITS[DMA_HTIF_FLAG][pos] ))
	{
		DMA_ClearFlag(pDMAHandle,DMA_HTIF_FLAG);
		DMA_ApplicationEventCallback(pDMAHandle,DMA_EVENT_HTIF);
	}

/****************************Check for Transfer Error flag **************************/

	if( pDMAHandle->pDMAx->ISR[h_l] & ( 1 << FLAGS_BITS[DMA_TEIF_FLAG][pos] ))
	{
		DMA_ClearFlag(pDMAHandle,DMA_TEIF_FLAG);
		DMA_ApplicationEventCallback(pDMAHandle,DMA_EVENT_TEIF);
	}

/***************************Check for Direct Mode Error flag ************************/

	if( pDMAHandle->pDMAx->ISR[h_l] & ( 1 << FLAGS_BITS[DMA_DMEIF_FLAG][pos] ))
	{
		DMA_ClearFlag(pDMAHandle,DMA_DMEIF_FLAG);
		DMA_ApplicationEventCallback(pDMAHandle,DMA_EVENT_DMEIF);
	}

/*******************************Check for FIFO Error flag ***************************/

	if( pDMAHandle->pDMAx->ISR[h_l] & ( 1 << FLAGS_BITS[DMA_FEIF_FLAG][pos] ))
	{
		DMA_ClearFlag(pDMAHandle,DMA_FEIF_FLAG);
		DMA_ApplicationEventCallback(pDMAHandle,DMA_EVENT_FEIF);
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
	uint8_t h_l = stream/4;
	uint8_t pos = stream%4;
	pDMAHandle->pDMAx->IFCR[h_l] |= ( 0x3D << 6*pos );
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
void DMA_ClearFlag(DMA_Handle_t *pDMAHandle, uint8_t FlagName)
{
	uint8_t stream = pDMAHandle->DMA_stream;
	uint8_t h_l = stream/4;
	uint8_t pos = stream%4;
	pDMAHandle->pDMAx->IFCR[h_l] |= ( 0x1 << FLAGS_BITS[FlagName][pos]);
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
		if(pDMAHandle->DMA_Config.DMA_Direction == DMA_DIR_MEM_TO_PERIPH)
		{
			pDMAHandle->pDMAx->STREAM[pDMAHandle->DMA_stream].PAR = pDMAHandle->pDestAddr;
			pDMAHandle->pDMAx->STREAM[pDMAHandle->DMA_stream].M0AR = pDMAHandle->pSourceAddr;
		}else
		{
			pDMAHandle->pDMAx->STREAM[pDMAHandle->DMA_stream].PAR = pDMAHandle->pSourceAddr;
			pDMAHandle->pDMAx->STREAM[pDMAHandle->DMA_stream].M0AR = pDMAHandle->pDestAddr;
		}
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
	uint8_t h_l = stream/4;
	uint8_t pos = stream%4;
	uint32_t status;

	status = pDMAHandle->pDMAx->ISR[h_l];

	if( status & ( 1 << FLAGS_BITS[DMA_TEIF_FLAG][pos] ) ) return 2;
	else if ( status & ( 1 << FLAGS_BITS[DMA_TCIF_FLAG][pos] ) ) return 1;
	else return 0;
}

/*
 * @fn      		  -DMA_ApplicationEventCallback
 *
 * @brief             -Weak implementation of Application Event Call function
 *
 * @param[in]         -Handling Structure of the DMA peripheral
 * @param[in]         -DMA Event macros
 *
 * @return            -none
 *
 * @Note              -Expect user to handle termination of different interrupts events
 */

__weak void DMA_ApplicationEventCallback(DMA_Handle_t *pDMAHandle, uint8_t DMA_EVENT)
{

}
