/*
 * stm32f446xx_protocol.c
 *
 *  Created on: Nov 14, 2025
 *      Author: jtlopez
 */

#include "stm32f446xx_protocol.h"

static USART_Handle_t USARTxHandle;

static TIM_Handle_t TIMxHandle;

static DMA_Handle_t DMAx_TXHandle;

static DMA_Handle_t DMAx_RXHandle;

char packets_keys[] = {'V','C','F','D','Z','S','X','N'};
float *pBuffer_values; 										//Data packet to be sent via UART
static uint8_t *pStatus;
static uint8_t *pFrequency;

uint8_t heartbeat_package[5];
uint8_t telemetry[35];

uint8_t receive_data[3];

uint8_t dma_ready;				//Flag DMA Ready to use (Free)
uint8_t telemetry_status;		//Flag Telemetry send request
uint8_t heartbeat_status;		//Flag HeatBeat send request

#define DMA_TR_NONE			0
#define DMA_TR_TELEMETRY	1
#define DMA_TR_HEARTBEAT	2

uint8_t dma_transfer_mode = DMA_TR_NONE;	//Flag DMA Transfer Mode (Transfering Telemetry or HeartBeat)

USART_RegDef_t *usart[] = {USART1,USART2,USART3,UART4,UART5,USART6};

uint8_t AltFunModeUSART(uint8_t usart_n, GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA || pGPIOx == GPIOB || GPIOC || GPIOD || GPIOE || GPIOG)
	{
		if( usart_n < 3) return 7;
		return 8;
	}
}

void USARTx_GPIOInits(uint8_t usart_n, GPIO_RegDef_t *pGPIOx_TX, GPIO_RegDef_t *pGPIOx_RX, uint8_t Pin_TX, uint8_t Pin_RX)
{
	//GPIO TX
	GPIO_Handle_t USARTxpin;
	USARTxpin.pGPIOx = pGPIOx_TX;
	USARTxpin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	USARTxpin.GPIO_PinConfig.GPIO_PinAltFunMode = AltFunModeUSART(usart_n,pGPIOx_TX);
	USARTxpin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	USARTxpin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	USARTxpin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	//TX
	USARTxpin.GPIO_PinConfig.GPIO_PinNumber = Pin_TX;
	GPIO_Init(&USARTxpin);
	if(pGPIOx_TX != pGPIOx_RX)
	{
		USARTxpin.pGPIOx = pGPIOx_RX;
		USARTxpin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
		USARTxpin.GPIO_PinConfig.GPIO_PinAltFunMode = AltFunModeUSART(usart_n,pGPIOx_RX);
		USARTxpin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
		USARTxpin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
		USARTxpin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	}
	//RX
	USARTxpin.GPIO_PinConfig.GPIO_PinNumber = Pin_RX;
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

DMA_RegDef_t* USARTx_DMAx(uint8_t usart_n)
{
	if(usart_n < 5 || usart_n > 0) return DMA1;
	return DMA2;
}

uint8_t dma_usart_stream[6][2] = {{7,5},{6,5},{3,1},{4,2},{7,0},{6,1}};
uint8_t irq_dma1_stream[] = {IRQ_NO_DMA1_STREAM0,IRQ_NO_DMA1_STREAM1,IRQ_NO_DMA1_STREAM2,IRQ_NO_DMA1_STREAM3, IRQ_NO_DMA1_STREAM4, IRQ_NO_DMA1_STREAM5, IRQ_NO_DMA1_STREAM6,IRQ_NO_DMA1_STREAM7};
uint8_t irq_dma2_stream[] = {IRQ_NO_DMA2_STREAM0,IRQ_NO_DMA2_STREAM1,IRQ_NO_DMA2_STREAM2,IRQ_NO_DMA2_STREAM3, IRQ_NO_DMA2_STREAM4, IRQ_NO_DMA2_STREAM5, IRQ_NO_DMA2_STREAM6,IRQ_NO_DMA2_STREAM7};

uint8_t USART_DMA_Channel(uint8_t usart_n)
{
	if(usart_n < 5) return DMA_CHANNEL_4;
	return DMA_CHANNEL_5;
}

uint8_t irq_tx,irq_rx;

void DMA_IRQ_USART(uint8_t usart_n)
{
	irq_tx = irq_dma2_stream[dma_usart_stream[usart_n][0]];
	irq_rx = irq_dma2_stream[dma_usart_stream[usart_n][1]];
	if(usart_n < 5 || usart_n > 0)
	{
		irq_tx = irq_dma1_stream[dma_usart_stream[usart_n][0]];
		irq_rx = irq_dma1_stream[dma_usart_stream[usart_n][1]];
	}
}

void DMAx_Inits(uint8_t usart_n)
{
	//DMA TX for USARTx
	DMAx_TXHandle.pDMAx = USARTx_DMAx(usart_n);
	DMAx_TXHandle.DMA_stream = dma_usart_stream[usart_n][0];
	DMAx_TXHandle.DMA_Config.DMA_Channel = USART_DMA_Channel(usart_n);
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
	DMAx_RXHandle.pDMAx = USARTx_DMAx(usart_n);
	DMAx_RXHandle.DMA_stream = dma_usart_stream[usart_n][1];
	DMAx_RXHandle.DMA_Config.DMA_Channel = USART_DMA_Channel(usart_n);
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

	DMA_IRQ_USART(usart_n);

	DMA_IRQInterruptConfig(irq_tx,ENABLE);
	DMA_IRQPriorityConfig(irq_tx,NVIC_IRQ_PRI1);

	DMA_IRQInterruptConfig(irq_rx,ENABLE);
	DMA_IRQPriorityConfig(irq_rx,NVIC_IRQ_PRI1);
}

void ProtocolInit(USART_RegDef_t *pUSARTx, GPIO_RegDef_t *pGPIOx_TX, GPIO_RegDef_t *pGPIOx_RX , uint8_t Pin_TX, uint8_t Pin_RX ,float *Buffer_values, uint8_t *Buffer_heartbeat)
{
	uint8_t usart_n;
	for(usart_n = 0; pUSARTx != usart[usart_n]; usart_n++);
	USARTx_GPIOInits(usart_n, pGPIOx_TX, pGPIOx_RX, Pin_TX, Pin_RX);
	USARTx_Inits(pUSARTx);
	DMAx_Inits(usart_n);

	/*HeartBeat Package Stucture*/
	heartbeat_package[0] = '$';
	heartbeat_package[1] = 'S';
	heartbeat_package[2] = 2;

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

	pBuffer_values = Buffer_values;
	pStatus = &Buffer_heartbeat[0];
	pFrequency = &Buffer_heartbeat[1];
}

uint8_t irq_tim[] = {IRQ_NO_TIM1_UP_TIM10,IRQ_NO_TIM2,IRQ_NO_TIM3,IRQ_NO_TIM4,IRQ_NO_TIM5,IRQ_NO_TIM6_DAC,IRQ_NO_TIM7,IRQ_NO_TIM8_UP_TIM13};
TIM_RegDef_t* timers[] = {TIM1,TIM2,TIM3,TIM4,TIM5,TIM6,TIM7,TIM8};

uint8_t TIM_IRQ(TIM_RegDef_t *pTIMx)
{
	uint8_t timer;
	for(timer = 0; pTIMx != timers[timer]; timer++);
	return irq_tim[timer];
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

void Protocol_HeartBeat_EN(void)
{
	heartbeat_status = ENABLE;
}

void Protocol_HeartBeat(void)
{
	if(dma_ready == DISABLE) return;
	if(heartbeat_status == DISABLE) return;
	USART_ClearFlag(USARTxHandle.pUSARTx, USART_TC_FLAG);
	heartbeat_package[3] = *pStatus;
	heartbeat_package[4] = *pFrequency;

	dma_transfer_mode = DMA_TR_HEARTBEAT;

	DMA_SetAddresses(&DMAx_TXHandle, (void*)heartbeat_package, (void*)&USARTxHandle.pUSARTx->DR);
	DMA_ConfigureBuffer(&DMAx_TXHandle, 5);
	DMA_StartTransfer(&DMAx_TXHandle);
}

uint8_t getValue_Variable(char s, uint8_t byteIndex)
{
	uint8_t x;
	for(x = 0; packets_keys[x] != s; x++);

	uint32_t *p = (uint32_t*)&pBuffer_values[x];

	return (*p >> ((3-byteIndex)*8)) & 0xFF;
}

void Protocol_Telemetry_EN(void)
{
	telemetry_status = ENABLE;
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
	if(receive_data[0] == '$' && receive_data[1] == 'X')
	{
		executeCommand(receive_data[2]);
	}
}

void DMA_ApplicationEventCallback(DMA_Handle_t *pDMAHandle, uint8_t ApEv)
{
	if(ApEv == DMA_EVENT_TCIF_CMPLT)
	{
		if(pDMAHandle->DMA_stream == 6)
		{
			DMA_StopTransfer(&DMAx_TXHandle);
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
			Protocol_DecodeRX();
		}
	}
}

__weak void executeCommand(uint8_t command)
{
	/*Expect other library to complete*/
}
