/*
 * stm32f446_gpio_driver.c
 *
 *  Created on: Oct 7, 2025
 *      Author: Isaac Pérez (ShiLiba)
 */

#include "stm32f446xx_gpio_driver.h"

/************************************************************************************
 * @fn				- GPIO_PeriClockControl
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
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if (pGPIOx == GPIOA) GPIOA_PCLK_EN();
		else if (pGPIOx == GPIOB) GPIOB_PCLK_EN();
		else if (pGPIOx == GPIOC) GPIOC_PCLK_EN();
		else if (pGPIOx == GPIOD) GPIOD_PCLK_EN();
		else if (pGPIOx == GPIOE) GPIOE_PCLK_EN();
		else if (pGPIOx == GPIOF) GPIOF_PCLK_EN();
		else if (pGPIOx == GPIOG) GPIOG_PCLK_EN();
		else if (pGPIOx == GPIOH) GPIOH_PCLK_EN();
	}
	else
	{
		if (pGPIOx == GPIOA) GPIOA_PCLK_DI();
		else if (pGPIOx == GPIOB) GPIOB_PCLK_DI();
		else if (pGPIOx == GPIOC) GPIOC_PCLK_DI();
		else if (pGPIOx == GPIOD) GPIOD_PCLK_DI();
		else if (pGPIOx == GPIOE) GPIOE_PCLK_DI();
		else if (pGPIOx == GPIOF) GPIOF_PCLK_DI();
		else if (pGPIOx == GPIOG) GPIOG_PCLK_DI();
		else if (pGPIOx == GPIOH) GPIOH_PCLK_DI();
	}
}

/************************************************************************************
 * @fn				- GPIO_Init
 *
 * @brief			- This function initializes a given gpio
 *
 * @param[in]		- base address of the gpio peripheral
 *
 * @return			- none
 *
 * @Note			- none
 *
 * */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;
	//1 . configure the mode of gpio pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//the non interrupt mode
		temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
		pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
		pGPIOHandle->pGPIOx->MODER |= temp; //setting
	} else
	{
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT ){
				EXTI->FTSR |= ( 1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
				EXTI->RTSR &= ~( 1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT ){
			EXTI->RTSR |= ( 1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR &= ~( 1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT ){
			EXTI->RTSR |= ( 1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= ( 1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		//2. configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFGEN_PCLK_EN();
		SYSCFG->EXTICR[temp1] |= (portcode << (temp2*4));
		//3. enable de exti interrupt delivery
		EXTI->IMR |= ( 1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		for(__vo uint16_t i = 0; i<1000; i++);
	}
	temp = 0;
	//2. configure the speed
	temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->OSPEEDR &= ~( 0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));		//clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;	//setting
	temp = 0;
	//3. configure the pupd settings
	temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->PUPDR &= ~( 0x3 << (2 *  pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp;		//setting
	temp = 0;
	//4. configures the optype
	temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->OTYPER &= ~( 0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;
	//5. configure the alt functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		//configure alt functions
		uint8_t temp1 = 0, temp2 = 0;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~( 0xF << ( 4* temp2 ));
		pGPIOHandle->pGPIOx->AFR[temp1] |= ( pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << ( 4* temp2 ));

	}
}

/************************************************************************************
 * @fn				- GPIO_DeInit
 *
 * @brief			- This function deinitializes a given gpio
 *
 * @param[in]		- base address of the gpio peripheral
 *
 * @return			- none
 *
 * @Note			- none
 *
 * */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if (pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if (pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if (pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if (pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if (pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}
	else if (pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
}

/************************************************************************************
 * @fn				- GPIO_ReadFromInputPin
 *
 * @brief			- This function reads a value from a pin
 *
 * @param[in]		- base address of the gpio peripheral
 *
 * @return			- value from said pin
 *
 * @Note			- none
 *
 * */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber)
{
	uint8_t value;

	value = (uint8_t)(pGPIOx->IDR >> pinNumber)& 0x00000001;

	return value;
}

/************************************************************************************
 * @fn				- GPIO_ReadFromInputPort
 *
 * @brief			- Reads all values from a port
 *
 * @param[in]		- base address of the gpio peripheral
 *
 * @return			- none
 *
 * @Note			- none
 *
 * */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx )
{
	uint16_t value;

	value = (uint16_t)( pGPIOx->IDR );

	return value;
}

/************************************************************************************
 * @fn				- GPIO_WriteToOutputPin
 *
 * @brief			- Reads all values from a port
 *
 * @param[in]		- base address of the gpio peripheral
 *
 * @return			- none
 *
 * @Note			- none
 *
 * */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t Value)
{
	if (Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |=  ( 1<<pinNumber );
	} else
	{
		pGPIOx->ODR &= ~( 1<<pinNumber );
	}
}

/************************************************************************************
 * @fn				- GPIO_ReadFromInputPort
 *
 * @brief			- Reads all values from a port
 *
 * @param[in]		- base address of the gpio peripheral
 *
 * @return			- none
 *
 * @Note			- none
 *
 * */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{

	pGPIOx->ODR = Value;

}

/************************************************************************************
 * @fn				- GPIO_ReadFromInputPort
 *
 * @brief			- Reads all values from a port
 *
 * @param[in]		- base address of the gpio peripheral
 *
 * @return			- none
 *
 * @Note			- none
 *
 * */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber)
{

	pGPIOx->ODR ^= ( 1<<pinNumber );

}

/************************************************************************************
 * @fn				- GPIO_IRQConfig
 *
 * @brief			- Reads all values from a port
 *
 * @param[in]		- base address of the gpio peripheral
 *
 * @return			- none
 *
 * @Note			- none
 *
 * */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//ISER0
			*NVIC_ISER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//ISER1
			*NVIC_ISER1 |= ( 1 << IRQNumber % 32 );
		}else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			//ISER2
			*NVIC_ISER2 |= ( 1 << IRQNumber % 64 );
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//ICER0
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//ICER1
			*NVIC_ICER1 |= ( 1 << IRQNumber % 32 );
		}else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			//ICER2
			*NVIC_ICER2 |= ( 1 << IRQNumber % 64 );
		}
	}

}

/************************************************************************************
 * @fn				- GPIO_IRQPriorityConfig
 *
 * @brief			- Configures IRQ priority for GPIO
 *
 * @param[in]		- base address of the gpio peripheral
 *
 * @return			- none
 *
 * @Note			- none
 *
 * */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	//1. find the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED );
	*(NVIC_PR_BASE_ADDR + (iprx*4)) |= ( IRQPriority << shift_amount );
}

/************************************************************************************
 * @fn				- GPIO_IRQHandling
 *
 * @brief			- Reads all values from a port
 *
 * @param[in]		- base address of the gpio peripheral
 *
 * @return			- none
 *
 * @Note			- none
 *
 * */
void GPIO_IRQHandling(uint8_t pinNumber)
{
	//clear the exti pr register corresponding to the pín number
	if(EXTI->PR & (1 <<pinNumber))	EXTI->PR |= (1 <<pinNumber);
}
