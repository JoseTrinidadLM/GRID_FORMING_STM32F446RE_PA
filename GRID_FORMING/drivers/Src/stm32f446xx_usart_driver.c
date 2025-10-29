/*
 * stm32f446xx_usart_driver.c
 *
 *  Created on: Oct 16, 2025
 *      Author: jtlopez
 */

#include "stm32f446xx_usart_driver.h"


/*
 * @fn				-USART_ClearTCFlag
 *
 * @brief			-This function clears TC flag of SR for the given USART
 *
 * @param[in]		-Base address of the USART peripheral
 *
 * @return			-none
 *
 * @Note			-none
 */

void USART_ClearTCFlag(USART_RegDef_t *pUSARTx)
{
	/*
	uint32_t dummyRead;
	dummyRead = pUSARTx->SR;
	pUSARTx->DR |= 0x000;
	(void)dummyRead;
	*/
	pUSARTx->SR &= ~(USART_TC_FLAG);
}

/*
 * @fn				-USART_ClearIDLEFlag
 *
 * @brief			-This function clears IDLE flag of SR for the given USART
 *
 * @param[in]		-Base address of the USART peripheral
 *
 * @return			-none
 *
 * @Note			-none
 */

void USART_ClearIDLEFlag(USART_RegDef_t *pUSARTx)
{
	uint32_t dummyRead;
	dummyRead = pUSARTx->SR;
	dummyRead = pUSARTx->DR;
	(void)dummyRead;
}

/*
 * @fn				-USART_ClearOREFlag
 *
 * @brief			-This function clears ORE flag of SR for the given USART
 *
 * @param[in]		-Base address of the USART peripheral
 *
 * @return			-none
 *
 * @Note			-none
 */

void USART_ClearOREFlag(USART_RegDef_t *pUSARTx)
{
	USART_ClearIDLEFlag(pUSARTx);
}

/*
 * @fn				-USART_SetBaudRate
 *
 * @brief			-This function calculates the needed USARTDIV to get the BaudRate given, and set it in BRR register of the given USART
 *
 * @param[in]		-Base address of the USART peripheral
 * @param[in]		-Baud Rate
 *
 * @return			-none
 *
 * @Note			-none
 */

void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{

	//Variable to hold the APB clock
	uint32_t PCLKx;

	uint32_t usartdiv;

	//variables to hold Mantissa and Fraction values
	uint32_t M_part,F_part;

	uint32_t tempreg=0;

	//Get the value of APB bus clock in to the variable PCLKx
	if(pUSARTx == USART1 || pUSARTx == USART6)
	{
	   //USART1 and USART6 are hanging on APB2 bus
	   PCLKx = RCC_GetPClk2();
	}else
	{
	   PCLKx = RCC_GetPClk1();
	}

	//Check for OVER8 configuration bit
	if(pUSARTx->CR1 & (1 << USART_CR1_OVER8))
	{
	   //OVER8 = 1 , over sampling by 8
	   usartdiv = ((25 * PCLKx) / (2 *BaudRate));
	}else
	{
	   //over sampling by 16
	  usartdiv = ((25 * PCLKx) / (4 *BaudRate));
	}

	//Calculate the Mantissa part
	M_part = usartdiv/100;

	//Place the Mantissa part in appropriate bit position . refer USART_BRR
	tempreg |= M_part << USART_BRR_DIV_MANTISSA;

	//Extract the fraction part
	F_part = (usartdiv - (M_part * 100));

	//Calculate the final fractional
	if(pUSARTx->CR1 & ( 1 << USART_CR1_OVER8))
	{
	  //OVER8 = 1 , over sampling by 8
	  F_part = ((( F_part * 8)+ 50) / 100)& ((uint8_t)0x07);

	}else
	{
	   //over sampling by 16
	   F_part = ((( F_part * 16)+ 50) / 100) & ((uint8_t)0x0F);

	}

	//Place the fractional part in appropriate bit position . refer USART_BRR
	tempreg |= (F_part << USART_BRR_DIV_FRACTION);

	//copy the value of tempreg in to BRR register
	pUSARTx->BRR = tempreg;
}

/*
 * Peripheral Clock setup
 */

/*
 * @fn				-USART_PClkC
 *
 * @brief			-This function enables or disables peripheral clock for the given USART
 *
 * @param[in]		-Base address of the USART peripheral
 * @param[in]		-Enable or Disable macros
 *
 * @return			-none
 *
 * @Note			-none
 */

void USART_PClkC(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			if(pUSARTx == USART1)
			{
				USART1_PCLK_EN();
			}else if(pUSARTx == USART2)
			{
				USART2_PCLK_EN();
			}else if(pUSARTx == USART3)
			{
				USART3_PCLK_EN();
			}else if(pUSARTx == UART4)
			{
				UART4_PCLK_EN();
			}else if(pUSARTx == UART5)
			{
				UART5_PCLK_EN();
			}else if(pUSARTx == USART6)
			{
				USART6_PCLK_EN();
			}
		}else
		{
			if(pUSARTx == USART1)
			{
				USART1_PCLK_DI();
			}else if(pUSARTx == USART2)
			{
				USART2_PCLK_DI();
			}else if(pUSARTx == USART3)
			{
				USART3_PCLK_DI();
			}else if(pUSARTx == UART4)
			{
				UART4_PCLK_DI();
			}else if(pUSARTx == UART5)
			{
				UART5_PCLK_DI();
			}else if(pUSARTx == USART6)
			{
				USART6_PCLK_DI();
			}
		}
}

/*
 * Init and De-Init
 */


/*
 * @fn				-USART_Init
 *
 * @brief			-Initialize variables for the given USART
 *
 * @param[in]		-Handling Structure of the USART peripheral
 *
 * @return			-none
 *
 * @Note			-none
 */

void USART_Init(USART_Handle_t *pUSARTHandle)
{

	//Temporary variable
	uint32_t tempreg=0;

/******************************** Configuration of CR1******************************************/

	//Implement the code to enable the Clock for given USART peripheral
	USART_PClkC(pUSARTHandle->pUSARTx, ENABLE);

	//Enable USART Tx and Rx engines according to the USART_Mode configuration item
	if (pUSARTHandle->USARTConfig.USART_Mode == USART_MODE_ONLY_RX)
	{
		//Implement the code to enable the Receiver bit field
		tempreg|= (1 << USART_CR1_RE);
	}else if (pUSARTHandle->USARTConfig.USART_Mode == USART_MODE_ONLY_TX)
	{
		//Implement the code to enable the Transmitter bit field
		tempreg |= ( 1 << USART_CR1_TE);

	}else if (pUSARTHandle->USARTConfig.USART_Mode == USART_MODE_TX_RX)
	{
		//Implement the code to enable the both Transmitter and Receiver bit fields
		tempreg |= ( ( 1 << USART_CR1_RE) | ( 1 << USART_CR1_TE) );
	}

    //Implement the code to configure the Word length configuration item
	tempreg |= pUSARTHandle->USARTConfig.USART_WordLength << USART_CR1_M ;


    //Configuration of parity control bit fields
	if ( pUSARTHandle->USARTConfig.USART_ParityControl == USART_PARITY_EVEN)
	{
		//Implement the code to enable the parity control
		tempreg |= ( 1 << USART_CR1_PCE);

		//Implement the code to enable EVEN parity
		//Not required because by default EVEN parity will be selected once you enable the parity control

	}else if (pUSARTHandle->USARTConfig.USART_ParityControl == USART_PARITY_ODD )
	{
		//Implement the code to enable the parity control
	    tempreg |= ( 1 << USART_CR1_PCE);

	    //Implement the code to enable ODD parity
	    tempreg |= ( 1 << USART_CR1_PS);

	}

   //Program the CR1 register
	pUSARTHandle->pUSARTx->CR1 = tempreg;

/******************************** Configuration of CR2******************************************/

	tempreg=0;

	//Implement the code to configure the number of stop bits inserted during USART frame transmission
	tempreg |= pUSARTHandle->USARTConfig.USART_NoOfStopBits << USART_CR2_STOP;

	//Program the CR2 register
	pUSARTHandle->pUSARTx->CR2 = tempreg;

/******************************** Configuration of CR3******************************************/

	tempreg=0;

	//Configuration of USART hardware flow control
	if ( pUSARTHandle->USARTConfig.USART_HWFlowControl == USART_HW_FC_CTS)
	{
		//Implement the code to enable CTS flow control
		tempreg |= ( 1 << USART_CR3_CTSE);


	}else if (pUSARTHandle->USARTConfig.USART_HWFlowControl == USART_HW_FC_RTS)
	{
		//Implement the code to enable RTS flow control
		tempreg |= (1 << USART_CR3_RTSE);

	}else if (pUSARTHandle->USARTConfig.USART_HWFlowControl == USART_HW_FC_CTS_RTS)
	{
		//Implement the code to enable both CTS and RTS Flow control
		tempreg |= ((1 << USART_CR3_RTSE) | (1 << USART_CR3_CTSE));
	}


	pUSARTHandle->pUSARTx->CR3 = tempreg;

/******************************** Configuration of BRR(Baudrate register)******************************************/

	//Implement the code to configure the baud rate
	//We will cover this in the lecture. No action required here
	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USARTConfig.USART_Baud);
}

/*
 * @fn				-USART_DeInit
 *
 * @brief			-Resets the configuration registers for the given USART
 *
 * @param[in]		-Base address of the USART peripheral
 *
 * @return			-none
 *
 * @Note			-none
 */

void USART_DeInit(USART_RegDef_t *pUSARTx)
{
	if(pUSARTx == USART1)
	{
		USART1_REG_RESET();
	}else if(pUSARTx == USART2)
	{
		USART2_REG_RESET();
	}else if(pUSARTx == USART3)
	{
		USART3_REG_RESET();
	}else if(pUSARTx == UART4)
	{
		UART4_REG_RESET();
	}else if(pUSARTx == UART5)
	{
		UART5_REG_RESET();
	}else if(pUSARTx == USART6)
	{
		USART6_REG_RESET();
	}
}

/*
 * @fn				-USART_GetFlagStatus
 *
 * @brief			-Returns Flag Status for the given Flag Name macro of SR register for the given USART
 *
 * @param[in]		-Base address of the USART peripheral
 * @param[in]		-Flag Name macro
 *
 * @return			-FLAG_SET or FLAG_RESET
 *
 * @Note			-none
 */

uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint16_t FlagName)
{
	if(pUSARTx->SR & FlagName){
		return FLAG_SET;
	}else
	{
		return FLAG_RESET;
	}
}

/*
 * @fn				-USART_ClearFlag
 *
 * @brief			-Clears Flag for the given Flag Name macro of SR register for the given USART
 *
 * @param[in]		-Base address of the USART peripheral
 * @param[in]		-Flag Name macro
 *
 * @return			-none
 *
 * @Note			-none
 */

void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t FlagName)
{
	pUSARTx->SR &= ~FlagName;
}

/*
 * Data Send and Receive
 */

/*
 * @fn				-USART_SendData
 *
 * @brief			-Sends data of the given Data Buffer pointer of given length, in the given USART
 *
 * @param[in]		-Base address of the USART peripheral
 * @param[in]		-Pointer of Data Buffer
 * @param[in]		-Length of Data
 *
 * @return			-none
 *
 * @Note			-Blocking type send data
 */

void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t* pTxBuffer, uint32_t Len)
{

	uint16_t *pdata;
	//Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//Implement the code to wait until TXE flag is set in the SR
		while(USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_TXE_FLAG) == FLAG_RESET);

		 //Check the USART_WordLength item for 9BIT or 8BIT in a frame
		if(pUSARTHandle->USARTConfig.USART_WordLength == USART_WLEN_9BITS)
		{
			//if 9BIT, load the DR with 2bytes masking the bits other than first 9 bits
			pdata = (uint16_t*) pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

			//check for USART_ParityControl
			if(pUSARTHandle->USARTConfig.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used in this transfer. so, 9bits of user data will be sent
				//Implement the code to increment pTxBuffer twice
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				//Parity bit is used in this transfer . so , 8bits of user data will be sent
				//The 9th bit will be replaced by parity bit by the hardware
				pTxBuffer++;
			}
		}
		else
		{
			//This is 8bit data transfer
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer  & (uint8_t)0xFF);

			//Implement the code to increment the buffer address
			pTxBuffer++;
		}
	}

	//Implement the code to wait till TC flag is set in the SR
	while(USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_TC_FLAG) == FLAG_RESET);
}

/*
 * @fn				-USART_ReceiveData
 *
 * @brief			-Receives data on the given Data Buffer pointer until given length, in the given USART
 *
 * @param[in]		-Base address of the USART peripheral
 * @param[in]		-Pointer of Data Buffer
 * @param[in]		-Length of Data
 *
 * @return			-none
 *
 * @Note			-Blocking type receive data
 */

void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t* pRxBuffer, uint32_t Len)
{
	//Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//Implement the code to wait until RXNE flag is set in the SR
		while(USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_RXNE_FLAG) == FLAG_RESET)

		//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
		if(pUSARTHandle->USARTConfig.USART_WordLength == USART_WLEN_9BITS)
		{
			//We are going to receive 9bit data in a frame

			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->USARTConfig.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used. so, all 9bits will be of user data

				//read only first 9 bits. so, mask the DR with 0x01FF
				*((uint16_t*) pRxBuffer) = (uint16_t)(pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

				//Now increment the pRxBuffer two times
				pRxBuffer++;
				pRxBuffer++;
			}
			else
			{
				//Parity is used, so, 8bits will be of user data and 1 bit is parity
				*pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);

				//Increment the pRxBuffer
				pRxBuffer++;
			}
		}
		else
		{
			//We are going to receive 8bit data in a frame

			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->USARTConfig.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used , so all 8bits will be of user data

				//read 8 bits from DR
				 *pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
			}

			else
			{
				//Parity is used, so , 7 bits will be of user data and 1 bit is parity

				//read only 7 bits , hence mask the DR with 0X7F
				 *pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t)0x7F);
			}

			//increment the pRxBuffer
			pRxBuffer++;
		}
	}
}

/*
 * @fn				-USART_ReceiveDataUntil
 *
 * @brief			-Receives data on the given Data Buffer pointer until given character is found in Data, in the given USART
 *
 * @param[in]		-Base address of the USART peripheral
 * @param[in]		-Pointer of Data Buffer
 * @param[in]		-Delimiter Character
 *
 * @return			-none
 *
 * @Note			-Blocking type receive data
 */

void USART_ReceiveDataUntil(USART_Handle_t *pUSARTHandle, uint8_t* pRxBuffer, uint8_t Char)
{
	uint8_t receivedByte;
	//Loop over until the RxBuffer is Char
	do
	{
		//Implement the code to wait until RXNE flag is set in the SR
		while(USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_RXNE_FLAG) == FLAG_RESET)

		//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
		if(pUSARTHandle->USARTConfig.USART_WordLength == USART_WLEN_9BITS)
		{
			//We are going to receive 9bit data in a frame

			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->USARTConfig.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used. so, all 9bits will be of user data

				//read only first 9 bits. so, mask the DR with 0x01FF
				*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

				//Now increment the pRxBuffer two times
				receivedByte = *pRxBuffer;
				pRxBuffer++;
				pRxBuffer++;
			}
			else
			{
				//Parity is used, so, 8bits will be of user data and 1 bit is parity
				*pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);

				receivedByte = *pRxBuffer;
				//Increment the pRxBuffer
				pRxBuffer++;
			}
		}
		else
		{
			//We are going to receive 8bit data in a frame

			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->USARTConfig.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used , so all 8bits will be of user data

				//read 8 bits from DR
				 *pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
			}
			else
			{
				//Parity is used, so , 7 bits will be of user data and 1 bit is parity

				//read only 7 bits , hence mask the DR with 0X7F
				 *pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t)0x7F);
			}

			//increment the pRxBuffer
			receivedByte = *pRxBuffer;
			pRxBuffer++;
		}
	}while(receivedByte != Char);
}

/*
 * @fn				-USART_SendDataWithIT
 *
 * @brief			-Returns USART State macro. If USART State wasn't busy in TX, it Activates Interrupts for Send data and configure Handler to use the given Data Buffer pointer of given length
 *
 * @param[in]		-Handling Structure of the USART peripheral
 * @param[in]		-Pointer of Data Buffer
 * @param[in]		-Length of Data
 *
 * @return			-USART State macro
 *
 * @Note			-Non-blocking type send data
 */

uint8_t USART_SendDataWithIT(USART_Handle_t *pUSARTHandle, uint8_t* pTxBuffer, uint32_t Len)
{
	uint8_t txstate = pUSARTHandle->TxState;

	if(txstate != USART_BUSY_IN_TX)
	{
		pUSARTHandle->TxCount = Len;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxState = USART_BUSY_IN_TX;

		//Implement the code to enable interrupt for TXE
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TXEIE);


		//Implement the code to enable interrupt for TC
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TCIE);


	}

	return txstate;
}

/*
 * @fn				-USART_ReceiveDataWithIT
 *
 * @brief			-Returns USART State macro. If USART State wasn't busy in RX, it Activates Interrupts for Receive data and configure Handler to use the given Data Buffer pointer of given length
 *
 * @param[in]		-Handling Structure of the USART peripheral
 * @param[in]		-Pointer of Data Buffer
 * @param[in]		-Length of Data
 *
 * @return			-USART State macro
 *
 * @Note			-Non-blocking type send data
 */

uint8_t USART_ReceiveDataWithIT(USART_Handle_t *pUSARTHandle, uint8_t* pRxBuffer, uint32_t Len)
{
	uint8_t rxstate = pUSARTHandle->RxState;

	if(rxstate != USART_BUSY_IN_RX)
	{
		pUSARTHandle->RxLen = Len;
		pUSARTHandle->RxCount = Len;
		pUSARTHandle->RxStopUntil = DISABLE;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->RxState = USART_BUSY_IN_RX;

		//Implement the code to enable interrupt for RXNE
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE);

	}

	return rxstate;
}

/*
 * @fn				-USART_ReceiveDataUntilWithIT
 *
 * @brief			-Returns USART State macro. If USART State wasn't busy in RX, it Activates Interrupts for Receive data and configure Handler to use the given Data Buffer pointer until given character is found in Data
 *
 * @param[in]		-Handling Structure of the USART peripheral
 * @param[in]		-Pointer of Data Buffer
 * @param[in]		-Delimiter Character
 *
 * @return			-USART State macro
 *
 * @Note			-Non-blocking type send data
 */

uint8_t USART_ReceiveDataUntilWithIT(USART_Handle_t *pUSARTHandle, uint8_t* pRxBuffer, uint8_t Char)
{
	uint8_t rxstate = pUSARTHandle->RxState;

	if(rxstate != USART_BUSY_IN_RX)
	{
		pUSARTHandle->RxLen = 0;
		pUSARTHandle->RxCount = 0;
		pUSARTHandle->RxStopChar = Char;
		pUSARTHandle->RxStopUntil = ENABLE;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->RxState = USART_BUSY_IN_RX;

		//Implement the code to enable interrupt for RXNE
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE);

	}

	return rxstate;
}

/*
 * IRQ Configuration and ISR handling
 */

/*
 * @fn				-USART_IRQITConfig
 *
 * @brief			-Enable or Disable NVIC for given Interrupt Request Number
 *
 * @param[in]		-Interrupt Request Number
 * @param[in]		-Enable or Disable
 *
 * @return			-none
 *
 * @Note			-none
 */

void USART_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		*NVIC_ISER[IRQNumber/32] |= (1<<IRQNumber%32);
	}else
	{
		*NVIC_ICER[IRQNumber/32] |= (1<<IRQNumber%32);
	}
}

/*
 * @fn				-USART_IRQPriorityConfig
 *
 * @brief			-Set priority for given Interrupt Request Number
 *
 * @param[in]		-Interrupt Request Number
 * @param[in]		-Priority
 *
 * @return			-none
 *
 * @Note			-none
 */

void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber/4;
	uint8_t iprx_section = IRQNumber%4;
	uint8_t shift_amount = (8*iprx_section)+(8-NO_PR_BITS_IMPLEMENTED);
	NVIC_PR_BASE_ADDR[iprx] |= (IRQPriority << shift_amount);
}

/*
 * @fn      		  -USART_IRQHandling
 *
 * @brief             -Handles Send Data, Receive Data, IDLE State and Errors interrupts
 *
 * @param[in]         -Handling Structure of the USART peripheral
 *
 * @return            -none
 *
 * @Note              -When it finish handling the interrupts it calls Application Event Function
 */

void USART_IRQHandling(USART_Handle_t *pUSARTHandle)
{
	uint32_t temp1 , temp2, temp3;

/*************************Check for TC flag ********************************************/

    //Implement the code to check the state of TC bit in the SR
	temp1 = USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_TC_FLAG);

	 //Implement the code to check the state of TCIE bit
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_TCIE);

	if(temp1 && temp2 )
	{
		//this interrupt is because of TC

		//close transmission and call application callback if TxLen is zero
		if ( pUSARTHandle->TxState == USART_BUSY_IN_TX)
		{
			//Check the TxLen . If it is zero then close the data transmission
			if(pUSARTHandle->TxCount == 0)
			{
				//Implement the code to clear the TC flag
				USART_ClearTCFlag(pUSARTHandle->pUSARTx);

				//Implement the code to clear the TCIE control bit
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_TCIE);
				//Reset the application state
				pUSARTHandle->TxState = USART_READY;

				//Reset Buffer address to NULL
				pUSARTHandle->pTxBuffer = NULL;

				//Reset the length to zero
				pUSARTHandle->TxCount = 0;

				//Call the applicaton call back with event USART_EVENT_TX_CMPLT
				USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_TX_CMPLT);
			}
		}
	}

/*************************Check for TXE flag ********************************************/

	//Implement the code to check the state of TXE bit in the SR
	temp1 = USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_TXE_FLAG);

	//Implement the code to check the state of TXEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_TXEIE);


	if(temp1 && temp2 )
	{
		//this interrupt is because of TXE

		if(pUSARTHandle->TxState == USART_BUSY_IN_TX)
		{
			//Keep sending data until Txlen reaches to zero
			if(pUSARTHandle->TxCount > 0)
			{
				//Check the USART_WordLength item for 9BIT or 8BIT in a frame
				if(pUSARTHandle->USARTConfig.USART_WordLength == USART_WLEN_9BITS)
				{
					//if 9BIT , load the DR with 2bytes masking the bits other than first 9 bits
					uint16_t *pdata = (uint16_t*) pUSARTHandle->pTxBuffer;

					//loading only first 9 bits , so we have to mask with the value 0x01FF
					pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

					//check for USART_ParityControl
					if(pUSARTHandle->USARTConfig.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity is used in this transfer , so, 9bits of user data will be sent
						//Implement the code to increment pTxBuffer twice
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->pTxBuffer++;

						//Implement the code to decrement the length
						pUSARTHandle->TxCount--;
					}
					else
					{
						//Parity bit is used in this transfer . so , 8bits of user data will be sent
						//The 9th bit will be replaced by parity bit by the hardware
						pUSARTHandle->pTxBuffer++;

						//Implement the code to decrement the length
						pUSARTHandle->TxCount--;
					}
				}
				else
				{
					//This is 8bit data transfer
					pUSARTHandle->pUSARTx->DR = (*pUSARTHandle->pTxBuffer  & (uint8_t)0xFF);

					//Implement the code to increment the buffer address
					pUSARTHandle->pTxBuffer++;

					//Implement the code to decrement the length
					pUSARTHandle->TxCount--;
				}

			}
			if (pUSARTHandle->TxCount == 0 )
			{
				//TxLen is zero
				//Implement the code to clear the TXEIE bit (disable interrupt for TXE flag )
				pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_TXEIE);
			}
		}
	}

/*************************Check for RXNE flag ********************************************/

	temp1 = USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_RXNE_FLAG);
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_RXNEIE);

	if(temp1 && temp2 )
	{
		uint8_t receivedByte = 0;
		//this interrupt is because of rxne
		if(pUSARTHandle->RxState == USART_BUSY_IN_RX)
		{

			if((!pUSARTHandle->RxStopUntil && (pUSARTHandle->RxCount > 0)) || (pUSARTHandle->RxStopUntil && (receivedByte != pUSARTHandle->RxStopChar)))
			{
				//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
				if(pUSARTHandle->USARTConfig.USART_WordLength == USART_WLEN_9BITS)
				{
					//We are going to receive 9bit data in a frame

					//Now, check are we using USART_ParityControl control or not
					if(pUSARTHandle->USARTConfig.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity is used. so, all 9bits will be of user data

						//read only first 9 bits so mask the DR with 0x01FF
						*((uint16_t*)pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

						//Now increment the pRxBuffer two times
						receivedByte = *pUSARTHandle->pRxBuffer;
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->pRxBuffer++;

						//Implement the code to decrement the length
						pUSARTHandle->RxCount--;
					}
					else
					{
						//Parity is used. so, 8bits will be of user data and 1 bit is parity
						 *pUSARTHandle->pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);

						 //Now increment the pRxBuffer
						 receivedByte = *pUSARTHandle->pRxBuffer;
						 pUSARTHandle->pRxBuffer++;

						 //Implement the code to decrement the length
						 pUSARTHandle->RxCount--;
					}
				}
				else
				{
					//We are going to receive 8bit data in a frame

					//Now, check are we using USART_ParityControl control or not
					if(pUSARTHandle->USARTConfig.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity is used , so all 8bits will be of user data

						//read 8 bits from DR
						 *pUSARTHandle->pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
					}

					else
					{
						//Parity is used, so , 7 bits will be of user data and 1 bit is parity

						//read only 7 bits , hence mask the DR with 0X7F
						 *pUSARTHandle->pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR  & (uint8_t)0x7F);

					}

					//Now , increment the pRxBuffer
					receivedByte = *pUSARTHandle->pRxBuffer;
					pUSARTHandle->pRxBuffer++;

					//Implement the code to decrement the length
					pUSARTHandle->RxCount--;
				}


			}//if of >0

			if( (!pUSARTHandle->RxStopUntil && !pUSARTHandle->RxCount) || (pUSARTHandle->RxStopUntil && (receivedByte == pUSARTHandle->RxStopChar)))
			{
				//disable the rxne
				receivedByte = 0;
				pUSARTHandle->RxLen += -pUSARTHandle->RxCount;
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_RXNEIE );
				pUSARTHandle->RxState = USART_READY;
				USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_RX_CMPLT);
			}
		}
	}


/*************************Check for CTS flag ********************************************/
//Note : CTS feature is not applicable for UART4 and UART5

	//Implement the code to check the status of CTS bit in the SR
	temp1 = USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_CTS_FLAG);

	//Implement the code to check the state of CTSE bit in CR3
	temp2 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSE);

	//Implement the code to check the state of CTSIE bit in CR3 (This bit is not available for UART4 & UART5.)
	temp3 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSIE);


	if(temp1  && temp2 )
	{
		//Implement the code to clear the CTS flag in SR
		USART_ClearFlag(pUSARTHandle->pUSARTx, USART_CTS_FLAG);

		//this interrupt is because of cts
		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_CTS);
	}

/*************************Check for IDLE detection flag ********************************************/

	//Implement the code to check the status of IDLE flag bit in the SR
	temp1 = USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_IDLE_FLAG);

	//Implement the code to check the state of IDLEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_IDLEIE);


	if(temp1 && temp2)
	{
		//Implement the code to clear the IDLE flag. Refer to the RM to understand the clear sequence
		USART_ClearIDLEFlag(pUSARTHandle->pUSARTx);
		//this interrupt is because of idle
		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_IDLE);
	}

/*************************Check for Overrun detection flag ********************************************/

	//Implement the code to check the status of ORE flag  in the SR
	temp1 = USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_ORE_FLAG);

	//Implement the code to check the status of RXNEIE  bit in the CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & USART_CR1_RXNEIE;


	if(temp1  && temp2 )
	{
		//Need not to clear the ORE flag here, instead give an api for the application to clear the ORE flag .
		USART_ClearOREFlag(pUSARTHandle->pUSARTx);
		//this interrupt is because of Overrun error
		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_ORE);
	}



/*************************Check for Error Flag ********************************************/

//Noise Flag, Overrun error and Framing Error in multibuffer communication
//We dont discuss multibuffer communication in this course. please refer to the RM
//The blow code will get executed in only if multibuffer mode is used.

	temp2 =  pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_EIE) ;

	if(temp2 )
	{
		temp1 = pUSARTHandle->pUSARTx->SR;
		if(temp1 & USART_FE_FLAG)
		{
			/*
				This bit is set by hardware when a de-synchronization, excessive noise or a break character
				is detected. It is cleared by a software sequence (an read to the USART_SR register
				followed by a read to the USART_DR register).
			*/
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERREVENT_FE);
		}

		if(temp1 & USART_NF_FLAG)
		{
			/*
				This bit is set by hardware when noise is detected on a received frame. It is cleared by a
				software sequence (an read to the USART_SR register followed by a read to the
				USART_DR register).
			*/
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERREVENT_NF);
		}

		if(temp1 & USART_ORE_FLAG)
		{
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERREVENT_ORE);
		}
	}
	//temp3 is use for??
	(void)temp3;
}

/*
 * @fn      		  -USART_PeripheralControl
 *
 * @brief             -Enables or Disable given USART peripheral
 *
 * @param[in]         -Base address of the USART peripheral
 *
 * @return            -none
 *
 * @Note              -none
 */

void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pUSARTx->CR1 |= (1 << USART_CR1_UE);
	}else
	{
		pUSARTx->CR1 &= ~(1 << USART_CR1_UE);
	}
}

/*
 * @fn      		  -USART_ApplicationEventCallback
 *
 * @brief             -Weak implementation of Application Event Call function
 *
 * @param[in]         -Handling Structure of the USART peripheral
 * @param[in]         -USART Event macros
 *
 * @return            -none
 *
 * @Note              -Expect user to handle termination of different interrupts events
 */

__weak void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t USART_EVENT)
{

}
