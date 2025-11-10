/*
 * stm32f446xx_gpio_driver.c
 *
 *  Created on: Oct 7, 2025
 *      Author: jtlopez & Isaac Pérez (ShiLiba)
 */

#include "stm32f446xx_gpio_driver.h"

/*
 * Peripheral Clock setup
 */

/*
 * @fn				-GPIO_PClkC
 *
 * @brief			-This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]		-Base address of the GPIO peripheral
 * @param[in]		-Enable or Disable macros
 *
 * @return			-none
 *
 * @Note			-none
 */

void GPIO_PClkC(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA) GPIOA_PCLK_EN();
		else if(pGPIOx == GPIOB) GPIOB_PCLK_EN();
		else if(pGPIOx == GPIOC) GPIOC_PCLK_EN();
		else if(pGPIOx == GPIOD) GPIOD_PCLK_EN();
		else if(pGPIOx == GPIOE) GPIOE_PCLK_EN();
		else if(pGPIOx == GPIOF) GPIOF_PCLK_EN();
		else if(pGPIOx == GPIOG) GPIOG_PCLK_EN();
		else if(pGPIOx == GPIOH) GPIOH_PCLK_EN();
	}else
	{
		if(pGPIOx == GPIOA) GPIOA_PCLK_DI();
		else if(pGPIOx == GPIOB) GPIOB_PCLK_DI();
		else if(pGPIOx == GPIOC) GPIOC_PCLK_DI();
		else if(pGPIOx == GPIOD) GPIOD_PCLK_DI();
		else if(pGPIOx == GPIOE) GPIOE_PCLK_DI();
		else if(pGPIOx == GPIOF) GPIOF_PCLK_DI();
		else if(pGPIOx == GPIOG) GPIOG_PCLK_DI();
		else if(pGPIOx == GPIOH) GPIOH_PCLK_DI();
	}

}

/*
 * Init and De-Init
 */

/*
 * @fn				-GPIO_Init
 *
 * @brief			-Initialize variables for the given GPIO port
 *
 * @param[in]		-Handling Structure of the GPIO port
 *
 * @return			-none
 *
 * @Note			-none
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	GPIO_PClkC(pGPIOHandle->pGPIOx,ENABLE);

	uint32_t temp = 0;
	//Configure pin mode
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//Non Interrupt Mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER |= temp;
	}else
	{
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		//Configure the GPIO port selection in SYSCFG_EXTICR
		uint32_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/4;
		uint32_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%4;
		uint32_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] |= (portcode << 4*temp2);
		EXTI->IMR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	}
	temp = 0;
	//Configure pin speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3<< (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;
	//Configure pin PUPD
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3<< (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;
	//Configure pin optype
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;
	//Configure pin altnf
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint32_t temp1 = 0, temp2 = 0;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xf << (4*temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4*temp2));
		temp = 0;
	}
}

/*
 * @fn				-GPIO_DeInit
 *
 * @brief			-Resets the configuration registers for the given GPIO port
 *
 * @param[in]		-Base address of the GPIO port
 *
 * @return			-none
 *
 * @Note			-none
 */

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA) GPIOA_REG_RESET();
	else if(pGPIOx == GPIOB) GPIOB_REG_RESET();
	else if(pGPIOx == GPIOC) GPIOC_REG_RESET();
	else if(pGPIOx == GPIOD) GPIOD_REG_RESET();
	else if(pGPIOx == GPIOE) GPIOE_REG_RESET();
	else if(pGPIOx == GPIOF) GPIOF_REG_RESET();
	else if(pGPIOx == GPIOG) GPIOG_REG_RESET();
	else if(pGPIOx == GPIOH) GPIOH_REG_RESET();
}

/*
 * Data read and write
 */

/*
 * @fn				-GPIO_ReadFromInputPin
 *
 * @brief			-Read position value for the given pin number in register for the given port
 *
 * @param[in]		-Base address of the GPIO port
 * @param[in]		-Pin number
 *
 * @return			-1 or 0
 *
 * @Note			-none
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> pinNumber) & (0x1));
	return value;
}

/*
 * @fn				-GPIO_ReadFromInputPort
 *
 * @brief			-Read entire register value for the given port
 *
 * @param[in]		-Base address of the GPIO port
 *
 * @return			-0 to 15
 *
 * @Note			-none
 */

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)(pGPIOx->IDR);
	return value;
}

/*
 * @fn				-GPIO_WritetoOutputPin
 *
 * @brief			-Write a value to the position for the given pin number in register for the given port
 *
 * @param[in]		-Base address of the GPIO port
 * @param[in]		-Pin number
 * @param[in]		-Value to write
 *
 * @return			-none
 *
 * @Note			-none
 */

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << pinNumber);
	}else
	{
		pGPIOx->ODR |= ~(0x1 << pinNumber);
	}
}

/*
 * @fn				-GPIO_WritetoOutputPort
 *
 * @brief			-Write a value to the register for the given port
 *
 * @param[in]		-Base address of the GPIO port
 *
 * @return			-none
 *
 * @Note			-none
 */

void GPIO_WritetoOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}

/*
 * @fn				-GPIO_ToggleOutputPin
 *
 * @brief			-Toggle position value for the given pin number in the register for the given port
 *
 * @param[in]		-Base address of the GPIO port
 * @param[in]		-Pin number
 *
 * @return			-none
 *
 * @Note			-none
 */

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber)
{
	pGPIOx->ODR ^= (1 << pinNumber);
}


/*
 * @fn				-GPIO_AtomicWriteToOutputPin
 *
 * @brief			-Write a value to the position for the given pin number in register for the given port using BSRR register
 *
 * @param[in]		-Base address of the GPIO port
 * @param[in]		-Pin number
 * @param[in]		-Value to write (set/reset)
 *
 * @return			-none
 *
 * @Note			-none
 */
void GPIO_AtomicWriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		pGPIOx->BSRR = (1 << pinNumber);
	}else
	{
		pGPIOx->BSRR = (1 << (pinNumber + 16));
	}
}

/*
 * IRQ Configuration and ISR handling
 */

/*
 * @fn				-GPIO_IRQITConfig
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

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
 * @fn				-GPIO_IRQPriorityConfig
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

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber/4;
	uint8_t iprx_section = IRQNumber%4;
	uint8_t shift_amount = (8*iprx_section)+(8-NO_PR_BITS_IMPLEMENTED);
	NVIC_PR_BASE_ADDR[iprx] |= (IRQPriority << shift_amount);
}

/*
 * @fn				-GPIO_IRQHandling
 *
 * @brief			-Enable External Interrupt for given GPIO pin Number
 *
 * @param[in]		-Pin Number
 *
 * @return			-none
 *
 * @Note			-none
 */

void GPIO_IRQHandling(uint8_t PinNumber)
{
	if(EXTI->PR & (1<<PinNumber)) EXTI->PR |= (1<<PinNumber);
}
