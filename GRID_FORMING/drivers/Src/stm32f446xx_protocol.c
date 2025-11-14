/*
 * stm32f446xx_protocol.c
 *
 *  Created on: Nov 14, 2025
 *      Author: jtlopez
 */

#include "stm32f446xx_protocol.h"

USART_Handle_t USARTxHandle;

TIM_Handle_t TIMxHandle;

DMA_Handle_t DMAx_TXHandle;

DMA_Handle_t DMAx_RXHandle;

char packets_keys[] = {'V','C','F','D','Z','S','X','N'};
float *pBuffer_values; 										//Data packet to be sent via UART
int valid_send = 1;											//Flag to indicate when data packet is ready to be sent

uint8_t heartbeat[5];
/*HeartBeat Package Stucture*/
heartbeat[0] = '$';
heartbeat[1] = 'S';
heartbeat[2] = 2;

uint8_t telemetry[35];
/*Telemetry Package Stucture*/
telemetry[0] = '$';
telemetry[1] = 'V';
telemetry[2] = 4;
telemetry[7] = '$';
telemetry[8] = 'C';
telemetry[9] = 4;
telemetry[14] = '$';
telemetry[15] = 'F';
telemetry[16] = 4;
telemetry[21] = '$';
telemetry[22] = 'D';
telemetry[23] = 4;
telemetry[28] = '$';
telemetry[29] = 'Z';
telemetry[30] = 4;

uint8_t receive_data[3];

uint8_t dma_ready;				//Flag DMA Ready to use (Free)
uint8_t telemetry_status;		//Flag Telemetry send request
uint8_t heartbeat_status;		//Flag HeatBeat send request

#define DMA_TR_NONE			0
#define DMA_TR_TELEMETRY	1
#define DMA_TR_HEARTBEAT	2

uint8_t dma_transfer_mode = DMA_TR_NONE;	//Flag DMA Transfer Mode (Transfering Telemetry or HeartBeat)

void USARTx_GPIOInits(USART_RegDef_t *pUSARTx)
{
	GPIO_Handle_t USARTxpin;
	USARTxpin.pGPIOx = USARTx_PORT(pUSARTx);
	USARTxpin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	USARTxpin.GPIO_PinConfig.GPIO_PinAltFunMode = 7;
	USARTxpin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	USARTxpin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	USARTxpin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	//TX
	USARTxpin.GPIO_PinConfig.GPIO_PinNumber = USARTx_PIN_TX(pUSARTx);
	GPIO_Init(&USARTxpin);
	//RX
	USARTxpin.GPIO_PinConfig.GPIO_PinNumber = USARTx_PIN_RX(pUSARTx);
	GPIO_Init(&USARTxpin);
}

void USARTx_Inits(USART_RegDef_t *pUSARTx)
{
	USARTxHandle.pUSARTx = pUSARTx;
	USARTxHandle.USARTConfig.USART_Baud = 2200000;
	USARTxHandle.USARTConfig.USART_HWFlowControl = USART_HW_FC_NONE;
	USARTxHandle.USARTConfig.USART_Mode = USART_MODE_TX_RX;
	USARTxHandle.USARTConfig.USART_NoOfStopBits = USART_1_STOPBITS;
	USARTxHandle.USARTConfig.USART_ParityControl = USART_PARITY_DISABLE;
	USARTxHandle.USARTConfig.USART_WordLength = USART_WLEN_8BITS;
	USARTxHandle.USARTConfig.USART_DMA = USART_DMA_TX_RX;

	USART_Init(&USARTxHandle);
}

void DMAx_Inits(USART_RegDef_t *pUSARTx)
{
	//DMA TX for USARTx
	DMAx_TXHandle.pDMAx = USARTx_DMAx(pUSARTx);
	DMAx_TXHandle.DMA_stream = DMA_TX_Stream(DMAx_TXHandle.pDMAx);
	DMAx_TXHandle.DMA_Config.DMA_Channel = USART_DMA_Channel(pUSARTx);
	DMAx_TXHandle.DMA_Config.DMA_Direction = DMA_DIR_MEM_TO_PERIPH;
	DMAx_TXHandle.DMA_Config.DMA_Priority = DMA_PRIORITY_MEDIUM;
	DMAx_TXHandle.DMA_Config.DMA_MemDataSize = DMA_DATA_SIZE_BYTE;
	DMAx_TXHandle.DMA_Config.DMA_PeriphDataSize = DMA_DATA_SIZE_BYTE;
	DMAx_TXHandle.DMA_Config.DMA_MemInc = ENABLE;
	DMAx_TXHandle.DMA_Config.DMA_PeriphInc = DISABLE;
	DMAx_TXHandle.DMA_Config.DMA_FIFOMode = DMA_FIFO_MODE_DISABLED;
	DMAx_TXHandle.DMA_Config.DMA_FIFOThreshold = 0;
	DMAx_TXHandle.DMA_Config.DMA_Mode = DMA_MODE_NORMAL;
	DMAx_TXHandle.DMA_Config.DMA_TransferIT = ENABLE;
	DMAx_TXHandle.BufferSize = 4;

	DMA_Init(&DMAx_TXHandle);

	//DMA RX for USARTx
	DMAx_RXHandle.pDMAx = USARTx_DMAx(pUSARTx);
	DMAx_RXHandle.DMA_stream = DMA_RX_Stream(DMAx_TXHandle.pDMAx);
	DMAx_RXHandle.DMA_Config.DMA_Channel = USART_DMA_Channel(pUSARTx);
	DMAx_RXHandle.DMA_Config.DMA_Direction = DMA_DIR_PERIPH_TO_MEM;
	DMAx_RXHandle.DMA_Config.DMA_MemDataSize = DMA_DATA_SIZE_BYTE;
	DMAx_RXHandle.DMA_Config.DMA_PeriphDataSize = DMA_DATA_SIZE_BYTE;
	DMAx_RXHandle.DMA_Config.DMA_MemInc = ENABLE;
	DMAx_RXHandle.DMA_Config.DMA_PeriphInc = DISABLE;
	DMAx_RXHandle.DMA_Config.DMA_FIFOMode = DMA_FIFO_MODE_DISABLED;
	DMAx_RXHandle.DMA_Config.DMA_FIFOThreshold = 0;
	DMAx_RXHandle.DMA_Config.DMA_Mode = DMA_MODE_CIRCULAR;
	DMAx_RXHandle.DMA_Config.DMA_TransferIT = ENABLE;
	DMAx_RXHandle.BufferSize = 3;

	DMA_Init(&DMAx_RXHandle);
	DMA_SetAddresses(&DMAx_RXHandle, (void*)&USARTxHandle.pUSARTx->DR, (void*)receive_data);

	DMA_IRQInterruptConfig(DMA_IRQ_TX(DMAx_TXHandle.pDMAx),ENABLE);
	DMA_IRQPriorityConfig(DMA_IRQ_TX(DMAx_TXHandle.pDMAx),NVIC_IRQ_PRI1);

	DMA_IRQInterruptConfig(DMA_IRQ_RX(DMAx_RXHandle.pDMAx),ENABLE);
	DMA_IRQPriorityConfig(DMA_IRQ_RX(DMAx_RXHandle.pDMAx),NVIC_IRQ_PRI1);
}

void ProtocolInit(USART_RegDef_t *pUSARTx, float *Buffer_values, uint8_t *Buffer_heartbeat)
{
	USARTx_GPIOInits(pUSARTx);
	USARTx_Inits(pUSARTx);
	DMAx_Inits(pUSARTx)
	pBuffer_values = Buffer_values;
	pStatus = &Buffer_heartbeat[0];
	pFrequency = &Buffer_heartbeat[1];
}

void Protocol_TIMInit(TIM_RegDef_t *pTIMx)
{
	TIMxHandle.pTIMx = pTIMx;
	TIMxHandle.TIM_Config.TIM_AutoReloadPreload = TIM_ARPE_ENABLE;
	TIMxHandle.TIM_Config.TIM_CLKDivision = TIM_CKD_DIV1;
	TIMxHandle.TIM_Config.TIM_CNTMode = TIM_UPCOUNT_MODE;
	TIMxHandle.TIM_Config.TIM_Frequency = 1;
	TIMxHandle.TIM_Config.TIM_IntEnable = TIM_IT_ENABLE;
	TIMxHandle.TIM_Config.TIM_MasterModeSel = TIM_MMS_UPDATE;

	TIM_Init(&TIMxHandle);

	TIM_IRQInterruptConfig(TIM_IRQ(pTIMx),ENABLE);
	TIM_IRQPriorityConfig(TIM_IRQ(pTIMx),NVIC_IRQ_PRI15);
}

void Protocol_Start(void)
{
	USART_PeripheralControl(USARTxHandle.pUSARTx, ENABLE);
	DMA_StartTransfer(&DMAx_RXHandle);
	TIM_Start(&TIMxHandle);
	dma_ready = ENABLE;
}

void Protocol_HeartBeat(void)
{
	if(dma_ready == DISABLE) return;
	if(heartbeat_status == DISABLE) return;
	USART_ClearFlag(USARTxHandle.pUSARTx, USART_TC_FLAG);
	heartbeat[3] = *pStatus;
	heartbeat[4] = *pFrequency;

	dma_transfer_mode = DMA_TR_HEARTBEAT;

	DMA_SetAddresses(&DMAx_TXHandle, (void*)heartbeat, (void*)&USARTxHandle.pUSARTx->DR);
	DMA_ConfigureBuffer(&DMAx_TXHandle, 5);
	DMA_StartTransfer(&DMAx_TXHandle);
}

uint8_t getValue_Variable(char s, uint8_t byteIndex)
{
	uint8_t x;
	for(x = 0; pBuffer_values[x] != s; x++);

	uint32_t *p = (uint32_t*)&pBuffer_values[x];

	return (*p >> ((3-byteIndex)*8)) & 0xFF;
}

void Protocol_Telemetry(void)
{
	if(dma_ready == DISABLE) return;
	if(telemetry_status == DISABLE) return;
	USART_ClearFlag(USARTxHandle.pUSARTx, USART_TC_FLAG);
	telemetry[3] = getValue_Variable('V', 0);
	telemetry[4] = getValue_Variable('V', 1);
	telemetry[5] = getValue_Variable('V', 2);
	telemetry[6] = getValue_Variable('V', 3);

	telemetry[10] = getValue_Variable('C', 0);
	telemetry[11] = getValue_Variable('C', 1);
	telemetry[12] = getValue_Variable('C', 2);
	telemetry[13] = getValue_Variable('C', 3);

	telemetry[17] = getValue_Variable('F', 0);
	telemetry[18] = getValue_Variable('F', 1);
	telemetry[19] = getValue_Variable('F', 2);
	telemetry[20] = getValue_Variable('F', 3);

	telemetry[24] = getValue_Variable('D', 0);
	telemetry[25] = getValue_Variable('D', 1);
	telemetry[26] = getValue_Variable('D', 2);
	telemetry[27] = getValue_Variable('D', 3);

	telemetry[31] = getValue_Variable('Z', 0);
	telemetry[32] = getValue_Variable('Z', 1);
	telemetry[33] = getValue_Variable('Z', 2);
	telemetry[34] = getValue_Variable('Z', 3);

	dma_transfer_mode = DMA_TR_TELEMETRY;

	DMA_SetAddresses(&DMAx_TXHandle, (void*)telemetry, (void*)&USARTxHandle.pUSARTx->DR);
	DMA_ConfigureBuffer(&DMAx_TXHandle, 35);
	DMA_StartTransfer(&DMAx_TXHandle);
}

void Protocol_DecodeRX(void)
{
	comm[0] = receive_data[0];
	comm[1] = receive_data[1];
	comm[2] = receive_data[2];
	if(receive_data[0] == '$' && receive_data[1] == 'X')
	{
		executeCommand(receive_data[2]);
	}
}

__weak void executeCommand(uint8_t command)
{
	/*Expect other library to complete*/
}

void TIM3_IRQHandler(void)
{
	heartbeat_status = ENABLE;
	TIM_IRQHandling(&TIM3Handle);
}

void DMA1_Stream6_IRQHandler(void)
{
	DMA_IRQHandling(&DMA1_TX2Handle);
}

void DMA1_Stream5_IRQHandler(void)
{
	DMA_IRQHandling(&DMA1_RX2Handle);
}

void TIM3_IRQHandler(void)
{
	heartbeat_status = ENABLE;
	TIM_IRQHandling(&TIM3Handle);
}

void DMA_ApplicationEventCallback(DMA_Handle_t *pDMAHandle, uint8_t ApEv)
{
	if(ApEv == DMA_EVENT_TCIF_CMPLT)
	{
		if(pDMAHandle->DMA_stream == 6)
		{
			DMA_StopTransfer(&DMA1_TX2Handle);
			dma_ready = ENABLE;
			if(dma_transfer_mode == DMA_TR_HEARTBEAT)
			{
				heartbeat_status = DISABLE;
			}else if(dma_transfer_mode == DMA_TR_TELEMETRY )
			{
				telemetry_status = DISABLE;
			}
			dma_transfer_mode = DMA_TR_NONE;
		}
		if(pDMAHandle->DMA_stream == 5)
		{
			USART_DecodeRX(&USART2Handle);
		}
	}
}