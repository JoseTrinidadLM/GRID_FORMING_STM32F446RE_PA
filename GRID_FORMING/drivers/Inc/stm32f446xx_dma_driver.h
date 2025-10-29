/*
 * stm32f446_dma_driver.h
 *
 *  Created on: Oct 20, 2025
 *      Author: Isaac Pérez (ShiLiba)
 */

#ifndef INC_STM32F446_DMA_DRIVER_H_
#define INC_STM32F446_DMA_DRIVER_H_

#include "stm32f446xx.h"

/*
 * @DMA_CHANNEL
 */
#define DMA_CHANNEL_0		0
#define DMA_CHANNEL_1		1
#define DMA_CHANNEL_2		2
#define DMA_CHANNEL_3		3
#define DMA_CHANNEL_4		4
#define DMA_CHANNEL_5		5
#define DMA_CHANNEL_6		6
#define DMA_CHANNEL_7		7

/*
 * @DMA_DIR
 */
#define DMA_DIR_PERIPH_TO_MEM		0
#define DMA_DIR_MEM_TO_PERIPH		1
#define DMA_DIR_MEM_TO_MEM			2

/*
 * @DMA_PRIORITY
 */
#define DMA_PRIORITY_LOW			0
#define DMA_PRIORITY_MEDIUM			1
#define DMA_PRIORITY_HIGH			2
#define DMA_PRIORITY_V_HIGH			3

/*
 * @DMA_DATA
 */
#define DMA_DATA_SIZE_BYTE			0
#define DMA_DATA_SIZE_HALFWORD		1
#define DMA_DATA_SIZE_WORD			2

/*
 * @DMA_MODE
 */
#define DMA_MODE_NORMAL				0
#define DMA_MODE_CIRCULAR			1

/*
 * @DMA_FIFO_THRESHOLD
 */
#define DMA_FIFO_THRESHOLD_1_4		0
#define DMA_FIFO_THRESHOLD_1_2		1
#define DMA_FIFO_THRESHOLD_3_4		2
#define DMA_FIFO_THRESHOLD_FULL		3

/*
 * @DMA_FIFO_MODE
 */
#define DMA_FIFO_MODE_ENABLED		0
#define DMA_FIFO_MODE_DISABLED		1

/*
 * This is Configuration structure for DMA
 */
typedef struct
{
	uint8_t DMA_Channel;
	uint8_t DMA_Direction;
	uint8_t DMA_Priority;
	uint8_t DMA_MemDataSize;
	uint8_t DMA_PeriphDataSize;
	uint8_t DMA_MemInc;
	uint8_t DMA_PeriphInc;
	uint8_t DMA_Mode;
	uint8_t DMA_TransferIT;
	uint8_t DMA_FIFOMode;
	uint8_t DMA_FIFOThreshold;
}DMA_Config_t;

/*
 * This is Handle structure for DMAx
 */
typedef struct
{
	DMA_RegDef_t *pDMAx;				/* <  This holds the base address of the DMA > */
	DMA_Config_t DMA_Config;			/* <  This holds DMA configuration settings > */
	uint8_t DMA_stream;
	uint32_t pSourceAddr;
	uint32_t pDestAddr;
	uint16_t BufferSize;
}DMA_Handle_t;

/************************************************************************************
 * 							APIs supported by this driver
 * 				For more information about the APIS check the function definitions
 ************************************************************************************/

/*
 * DMA Clock setup
 */
void DMA_PeriClockControl(DMA_RegDef_t *pDMAx, uint8_t EnorDi);

/*
 * Init and de-init
 */
void DMA_Init(DMA_Handle_t *pDMAHandle);
void DMA_DeInit(DMA_RegDef_t *pDMAx);

/*
 * DMA Start/Stop
 */
void DMA_StartTransfer(DMA_Handle_t *pDMAHandle);
void DMA_StopTransfer(DMA_Handle_t *pDMAHandle);

/*
 * IRQ and ISR Handling
 */
void DMA_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void DMA_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void DMA_IRQHandling(DMA_Handle_t *pDMAHandle);

/*
 * Status and utility
 */
void DMA_ClearFlags(DMA_Handle_t *pDMAHandle);
void DMA_SetAddresses(DMA_Handle_t *pDMAHandle, void *pSrc, void *pDest);
uint8_t DMA_GetTransferStatus(DMA_Handle_t *pDMAHandle);


#endif /* INC_STM32F446_DMA_DRIVER_H_ */
