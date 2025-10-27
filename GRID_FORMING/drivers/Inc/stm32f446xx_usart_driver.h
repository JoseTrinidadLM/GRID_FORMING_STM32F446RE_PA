/*
 * stm32f446xx_usart_driver.h
 *
 *  Created on: Oct 16, 2025
 *      Author: jtlopez
 */

#ifndef INC_STM32F446XX_USART_DRIVER_H_
#define INC_STM32F446XX_USART_DRIVER_H_

#include "stm32f446xx.h"

/*
 * Configuration structure for USARTx peripheral
 */

typedef struct
{
	uint8_t USART_Mode;
	uint32_t USART_Baud;
	uint8_t	USART_NoOfStopBits;
	uint8_t USART_WordLength;
	uint8_t	USART_ParityControl;
	uint8_t USART_HWFlowControl;
}USART_Config_t;

/*
 * Handle structure for USARTx peripheral
 */

typedef struct
{
	USART_RegDef_t *pUSARTx;
	USART_Config_t USARTConfig;
	uint8_t *pTxBuffer;
	uint32_t TxCount;
	uint8_t TxState;
	uint8_t *pRxBuffer;
	uint32_t RxCount;
	uint32_t RxLen;
	uint8_t RxState;
	uint8_t RxStopChar;
	uint8_t RxStopUntil;
}USART_Handle_t;

/*
 * USART Tx and Rx State
 */

#define USART_READY			0
#define USART_BUSY_IN_TX	1
#define USART_BUSY_IN_RX	2

/*
 * USART Mode
 */

#define USART_MODE_ONLY_TX	1
#define USART_MODE_ONLY_RX	2
#define USART_MODE_TX_RX	3

/*
 * USART Baud
 */

#define USART_STD_BAUD_1200					1200
#define USART_STD_BAUD_2400					400
#define USART_STD_BAUD_9600					9600
#define USART_STD_BAUD_19200 				19200
#define USART_STD_BAUD_38400 				38400
#define USART_STD_BAUD_57600 				57600
#define USART_STD_BAUD_115200 				115200
#define USART_STD_BAUD_230400 				230400
#define USART_STD_BAUD_460800 				460800
#define USART_STD_BAUD_921600 				921600
#define USART_STD_BAUD_2M 					2000000
#define SUART_STD_BAUD_3M 					3000000

/*
 * NoOfStopBits
 */

#define USART_0_5_STOPBITS	1
#define USART_1_STOPBITS	0
#define USART_1_5_STOPBITS	3
#define USART_2_STOPBITS	2

/*
 * USART Word Length
 */

#define	USART_WLEN_8BITS	0
#define USART_WLEN_9BITS	1

/*
 * USART Parity Control
 */

#define USART_PARITY_DISABLE	0
#define USART_PARITY_ODD		1
#define USART_PARITY_EVEN		2

/*
 * USART Hardware Flow Control
 */

#define	USART_HW_FC_NONE	0
#define USART_HW_FC_CTS		1
#define USART_HW_FC_RTS		2
#define USART_HW_FC_CTS_RTS	3

/*
 * SPI related status flags definitions
 */

#define USART_PE_FLAG		(1<<USART_SR_PE)
#define USART_FE_FLAG		(1<<USART_SR_FE)
#define USART_NF_FLAG		(1<<USART_SR_NF)
#define USART_ORE_FLAG		(1<<USART_SR_ORE)
#define USART_IDLE_FLAG		(1<<USART_SR_IDLE)
#define USART_RXNE_FLAG		(1<<USART_SR_RXNE)
#define USART_TC_FLAG		(1<<USART_SR_TC)
#define USART_TXE_FLAG		(1<<USART_SR_TXE)
#define USART_LBD_FLAG		(1<<USART_SR_LBD)
#define USART_CTS_FLAG		(1<<USART_SR_CTS)

/*
 * Possible Events
 */

#define USART_EVENT_TX_CMPLT 	1
#define USART_EVENT_RX_CMPLT 	2
#define USART_EVENT_CTS			3
#define USART_EVENT_IDLE		4
#define USART_EVENT_ORE			5
#define USART_ERREVENT_FE		6
#define USART_ERREVENT_NF		7
#define USART_ERREVENT_ORE		8


/*
 *		APIs
 *
 */

/*
 * Calculate Baud Rate
 */

void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate);

/*
 * Peripheral Clock setup
 */

void USART_PClkC(USART_RegDef_t *pUSARTx, uint8_t EnorDi);

/*
 * Init and De-Init
 */

void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_RegDef_t *pUSARTx);

/*
 * Flag Status
 */

uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint16_t FlagName);
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t FlagName);

/*
 * Data Send and Receive
 */

void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t* pTxBuffer, uint32_t Len);
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t* pTxBuffer, uint32_t Len);
void USART_ReceiveDataUntil(USART_Handle_t *pUSARTHandle, uint8_t* pTxBuffer, uint8_t Char);

uint8_t USART_SendDataWithIT(USART_Handle_t *pUSARTHandle, uint8_t* pTxBuffer, uint32_t Len);
uint8_t USART_ReceiveDataWithIT(USART_Handle_t *pUSARTHandle, uint8_t* pTxBuffer, uint32_t Len);
uint8_t USART_ReceiveDataUntilWithIT(USART_Handle_t *pUSARTHandle, uint8_t* pTxBuffer, uint8_t Char);


/*
 * IRQ Configuration and ISR handling
 */

void USART_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void USART_IRQHandling(USART_Handle_t *pUSARTHandle);

//

void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi);

//
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t USART_EVENT);


#endif /* INC_STM32F446XX_USART_DRIVER_H_ */
