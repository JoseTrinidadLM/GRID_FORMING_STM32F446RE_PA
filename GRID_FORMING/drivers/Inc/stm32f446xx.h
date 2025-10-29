/*
 * stm32f446xx.h
 *
 *  Created on: Oct 6, 2025
 *      Author: Isaac Pérez (ShiLiba)
 */


#ifndef INC_STM32F446_H_
#define INC_STM32F446_H_

#include <stddef.h>
#include <stdint.h>

#define __vo volatile
#define __weak __attribute__((weak))
/**********************************************************START: Processor Specific Details***********************************************************/
/*
 * ARM Cortex Mx Processor NVIC ISERx regirter Address
 */
#define NVIC_ISER0						( (__vo uint32_t*)0xE000E100 )
#define NVIC_ISER1						( (__vo uint32_t*)0xE000E104 )
#define NVIC_ISER2						( (__vo uint32_t*)0xE000E108 )
#define NVIC_ISER3						( (__vo uint32_t*)0xE000E10C )
#define NVIC_ISER4						( (__vo uint32_t*)0xE000E110 )
#define NVIC_ISER5						( (__vo uint32_t*)0xE000E114 )
#define NVIC_ISER6						( (__vo uint32_t*)0xE000E118 )
#define NVIC_ISER7						( (__vo uint32_t*)0xE000E11C )

#define NVIC_ISER		(__vo uint32_t*[]){NVIC_ISER0,NVIC_ISER1,NVIC_ISER2,NVIC_ISER3}

/*
 * ARM Cortex Mx Processor NVIC ICERx regirter Address
 */
#define NVIC_ICER0						( (__vo uint32_t*)0xE000E180 )
#define NVIC_ICER1						( (__vo uint32_t*)0xE000E184 )
#define NVIC_ICER2						( (__vo uint32_t*)0xE000E188 )
#define NVIC_ICER3						( (__vo uint32_t*)0xE000E18C )
#define NVIC_ICER4						( (__vo uint32_t*)0xE000E190 )
#define NVIC_ICER5						( (__vo uint32_t*)0xE000E194 )
#define NVIC_ICER6						( (__vo uint32_t*)0xE000E198 )
#define NVIC_ICER7						( (__vo uint32_t*)0xE000E19C )

#define NVIC_ICER		(__vo uint32_t*[]){NVIC_ICER0,NVIC_ICER1,NVIC_ICER2}

/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */

#define SCB_CPACR (*((__vo uint32_t*)0xE000ED88))


#define NVIC_PR_BASE_ADDR				( (__vo uint32_t*)0xE000E400 )

#define NO_PR_BITS_IMPLEMENTED		4

/*
 * ARM Cortex Mx Processor number of priority bits implemented in Priority Register
 */
#define NO_PR_BITS_IMPLEMENTED				4

/*
 * base addresses of Flash and SRAM memories
 */


#define FLASH_BASEADDR					0x08000000U
#define SRAM1_BASEADDR					0x20000000U
#define SRAM2_BASEADDR					0x2001C000U
#define ROM_BASEADDR					0x1FFF0000U
#define	SRAM							SRAM1_BASEADDR
#define FLASH_INTERFACE_BASEADDR		0x40023C00U
/*
 * AHBx and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASEADDR					0x40000000U
#define APB1PERIPH_BASEADDR				PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR				0x40010000U
#define AHB1PERIPH_BASEADDR				0x40020000U
#define AHB2PERIPH_BASEADDR				0x50000000U

/*
 * Base addresses of peripheral which are hanging on AHB1 bus
 * TODO : Complete for all other peripherals
 */

#define GPIOA_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR					(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR					(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR					(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR					(AHB1PERIPH_BASEADDR + 0x1C00)
#define RCC_BASEADDR					(AHB1PERIPH_BASEADDR + 0x3800)

/*
 * Base addresses of peripheral which are hanging on APB1 bus
 * TODO : Complete for all other peripherals
 */

#define SPI2_BASEADDR					(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR					(APB1PERIPH_BASEADDR + 0x3C00)
#define USART2_BASEADDR					(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR					(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR					(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR					(APB1PERIPH_BASEADDR + 0x5000)
#define I2C1_BASEADDR					(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR					(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR					(APB1PERIPH_BASEADDR + 0x5C00)

/*
 * Base addresses of peripheral which are hanging on APB2 bus
 * TODO : Complete for all other peripherals
 */

#define USART1_BASEADDR					(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR					(APB2PERIPH_BASEADDR + 0x1400)
#define SPI1_BASEADDR					(APB2PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR					(APB2PERIPH_BASEADDR + 0x3400)
#define SYSCFG_BASEADDR					(APB2PERIPH_BASEADDR + 0x3800)
#define EXTI_BASEADDR					(APB2PERIPH_BASEADDR + 0x3C00)

/*
 * Base addresses of timers
 */
#define TIM1_BASEADDR			(APB2PERIPH_BASEADDR + 0x0000)
#define TIM2_BASEADDR			(APB1PERIPH_BASEADDR + 0x0000)
#define TIM3_BASEADDR			(APB1PERIPH_BASEADDR + 0x0400)
#define TIM4_BASEADDR			(APB1PERIPH_BASEADDR + 0x0800)
#define TIM5_BASEADDR			(APB1PERIPH_BASEADDR + 0x0C00)
#define TIM6_BASEADDR			(APB1PERIPH_BASEADDR + 0x1000)
#define TIM7_BASEADDR			(APB1PERIPH_BASEADDR + 0x1400)
#define TIM8_BASEADDR			(APB2PERIPH_BASEADDR + 0x0400)
#define TIM9_BASEADDR			(APB2PERIPH_BASEADDR + 0x4000)
#define TIM10_BASEADDR			(APB2PERIPH_BASEADDR + 0x4400)
#define TIM11_BASEADDR			(APB2PERIPH_BASEADDR + 0x4800)
#define TIM12_BASEADDR			(APB1PERIPH_BASEADDR + 0x1800)
#define TIM13_BASEADDR			(APB1PERIPH_BASEADDR + 0x1C00)
#define TIM14_BASEADDR			(APB1PERIPH_BASEADDR + 0x2000)

/*
 *  Base addresses of DMA
 */
#define DMA1_BASEADDR			(AHB1PERIPH_BASEADDR + 0x6000)
#define DMA2_BASEADDR			(AHB1PERIPH_BASEADDR + 0x6400)

/*
 *  Base addresses of ADC
 */
#define ADC1_BASEADDR			(APB2PERIPH_BASEADDR + 0x2000)
#define ADC2_BASEADDR			(APB2PERIPH_BASEADDR + 0x2100)
#define ADC3_BASEADDR			(APB2PERIPH_BASEADDR + 0x2200)

/**************************************peripheral register definition structures*******************************************/


/*
 *  peripheral register definition structure for GPIO
 */

typedef struct
{
	__vo uint32_t MODER;					//GPIO port mode register
	__vo uint32_t OTYPER;					//GPIO port output type register
	__vo uint32_t OSPEEDR;					//GPIO port output speed register
	__vo uint32_t PUPDR;					//GPIO port pull-up/pull-down register
	__vo uint32_t IDR;						//GPIO port input data register
	__vo uint32_t ODR;						//GPIO port output data register
	__vo uint32_t BSRR;						//GPIO port bit set/reset register
	__vo uint32_t LCKR;						//GPIO port configuration lock register
	__vo uint32_t AFR[2];					//GPIO alternate function register (AFR[0] -> GPIO alternate function LOW register, AFR[1] -> GPIO alternate function HIGH register)
} GPIO_RegDef_t;

/*
 *  peripheral register definition structure for RCC
 */

typedef struct{
	__vo uint32_t CR;						//RCC clock control register
	__vo uint32_t PLLCFGR;					//RCC PLL configuration register
	__vo uint32_t CFGR;						//RCC clock configuration register
	__vo uint32_t CIR;						//RCC clock interrupt register
	__vo uint32_t AHB1RSTR;					//RCC AHB1 peripheral reset register
	__vo uint32_t AHB2RSTR;					//RCC AHB2 peripheral reset register
	__vo uint32_t AHB3RSTR;					//RCC AHB3 peripheral reset register
	uint32_t RESERVED0;						//
	__vo uint32_t APB1RSTR;					//RCC APB1 peripheral reset register
	__vo uint32_t APB2RSTR;					//RCC APB2 peripheral reset register
	uint32_t RESERVED1[2];					//
	__vo uint32_t AHB1ENR;					//RCC AHB1 peripheral clock enable register
	__vo uint32_t AHB2ENR;					//RCC AHB2 peripheral reset register
	__vo uint32_t AHB3ENR;					//RCC AHB3 peripheral reset register
	uint32_t RESERVED2;						//
	__vo uint32_t APB1ENR;					//RCC APB1 peripheral reset register
	__vo uint32_t APB2ENR;					//RCC APB2 peripheral reset register
	uint32_t RESERVED3[2];					//
	__vo uint32_t AHB1LPENR;				//RCC AHB1 peripheral clock enable in low power mode register
	__vo uint32_t AHB2LPENR;				//RCC AHB2 peripheral clock enable in low power mode register
	__vo uint32_t AHB3LPENR;				//RCC AHB3 peripheral clock enable in low power mode register
	uint32_t RESERVED4;						//
	__vo uint32_t APB1LPENR;				//RCC APB1 peripheral clock enable in low power mode register
	__vo uint32_t APB2LPENR;				//RCC APB2 peripheral clock enabled in low power mode register
	uint32_t RESERVED5[2];					//
	__vo uint32_t BDCR;						//RCC Backup domain control register
	__vo uint32_t CSR;						//RCC clock control and status register
	uint32_t RESERVED6[2];					//
	__vo uint32_t SSCGR;					//RCC spread spectrum clock generation register
	__vo uint32_t PLLI2SCFGR;				//RCC PLLI2S configuration register
	__vo uint32_t PLLSAICFGR;				//RCC PLL configuration register
	__vo uint32_t DCKCFGR;					//RCC dedicated clock configuration register
	__vo uint32_t CKGATENR;					//RCC clocks gated enable register
	__vo uint32_t DCKCFGR2;					//RCC dedicated clocks configuration register 2

} RCC_RegDef_t;

/*
 *  peripheral register definition structure for EXTI
 */

typedef struct
{
	__vo uint32_t IMR;					//Interrupt mask register
	__vo uint32_t EMR;					//Event mask register
	__vo uint32_t RTSR;					//Rising trigger selection register
	__vo uint32_t FTSR;					//Falling trigger selection register
	__vo uint32_t SWIER;				//Software interrupt event register
	__vo uint32_t PR;					//Pending register
} EXTI_RegDef_t;

/*
 *  peripheral register definition structure for SYSCFG
 */

typedef struct
{
	__vo uint32_t MEMRMP;				//SYSCFG memory remap register
	__vo uint32_t PMC;					//SYSCFG peripheral mode configuration register
	__vo uint32_t EXTICR[4];			//SYSCFG external interrupt configuration register
	uint32_t 	  RESERVED1[2];
	__vo uint32_t CMPCR;				//Compensation cell control register
	uint32_t 	  RESERVED2[2];
	__vo uint32_t CFGR;					//SYSCFG configuration register
} SYSCFG_RegDef_t;

/*
 *  peripheral register definition structure for TIMx
 */

typedef struct
{
	__vo uint32_t CR1;					//TIM control register 1
	__vo uint32_t CR2;					//TIM control register 2
	__vo uint32_t SMCR;					//TIM slave mode control register
	__vo uint32_t DIER;					//TIM DMA/interrupt enable register
	__vo uint32_t SR;					//TIM status register
	__vo uint32_t EGR;					//TIM event generation register
	__vo uint32_t CCMR1;				//TIM capture/compare mode register 1
	__vo uint32_t CCMR2;				//TIM capture/compare mode register 2
	__vo uint32_t CCER;					//TIM capture/compare enable register
	__vo uint32_t CNT;					//TIM counter
	__vo uint32_t PSC;					//TIM prescaler
	__vo uint32_t ARR;					//TIM auto-reload register
	__vo uint32_t RCR;					//TIM repetition counter register
	__vo uint32_t CCR1;					//TIM capture/compare register 1
	__vo uint32_t CCR2;					//TIM capture/compare register 2
	__vo uint32_t CCR3;					//TIM capture/compare register 3
	__vo uint32_t CCR4;					//TIM capture/compare register 4
	__vo uint32_t BDTR;					//TIM break and dead-time register
	__vo uint32_t DCR;					//TIM DMA control register
	__vo uint32_t DMAR;					//TIM DMA address for full transfer
	__vo uint32_t OR;					//TIM option register
} TIM_RegDef_t;

/*
 *  peripheral register definition structure for FLASH
 */
typedef struct
{
	__vo uint32_t ACR;					//Flash access control register
	__vo uint32_t KEYR;					//Flash key register
	__vo uint32_t OPTKEYR;				//Flash option key register
	__vo uint32_t SR;					//Flash status register
	__vo uint32_t CR;					//Flash control register
	__vo uint32_t OPTCR;				//Flash option control register
} FLASH_RegDef_t;

/*
 *  register definition structure for Streams
 */

typedef struct
{
	__vo uint32_t CR;					//Stream x configuration register
	__vo uint32_t NDTR;					//Stream x number of data register
	__vo uint32_t PAR;					//Stream x peripheral address register
	__vo uint32_t M0AR;					//Stream x memory 0 address register
	__vo uint32_t M1AR;					//Stream x memory 1 address register
	__vo uint32_t FCR;					//Stream x FIFO control register
} DMA_Stream_RegDef_t;

/*
 *  peripheral register definition structure for DMA
 */
typedef struct
{
	__vo uint32_t LISR;					//DMA low interrupt status register
	__vo uint32_t HISR;					//DMA high interrupt status register
	__vo uint32_t LIFCR;				//DMA low interrupt flag clear register
	__vo uint32_t HIFCR;				//DMA high interrupt flag clear register
	DMA_Stream_RegDef_t STREAM[8]; 		//8 streams
} DMA_RegDef_t;


/*
 *  register definition structure for ADC
 */
typedef struct
{
	__vo uint32_t SR;					//ADC status register
	__vo uint32_t CR1;					//ADC control register 1
	__vo uint32_t CR2;					//ADC control register 2
	__vo uint32_t SMPR1;				//ADC sample time register 1
	__vo uint32_t SMPR2;				//ADC sample time register 2
	__vo uint32_t JOFR1;				//ADC injected channel data offset register x = 1-4
	__vo uint32_t JOFR2;
	__vo uint32_t JOFR3;
	__vo uint32_t JOFR4;
	__vo uint32_t HTR;					//ADC watchdog higher threshold register
	__vo uint32_t LTR;					//ADC watchdog lower threshold register
	__vo uint32_t SQR1;					//ADC regular sequence register 1
	__vo uint32_t SQR2;					//ADC regular sequence register 2
	__vo uint32_t SQR3;					//ADC regular sequence register 3
	__vo uint32_t JSQR;					//ADC injected sequence register
	__vo uint32_t JDR1;					//ADC injected data register x = 1-4
	__vo uint32_t JDR2;
	__vo uint32_t JDR3;
	__vo uint32_t JDR4;
	__vo uint32_t DR;					//ADC regular data register
	__vo uint32_t CSR;					//ADC Common status register
	__vo uint32_t CCR;					//ADC common control register
	__vo uint32_t CDR;					//ADC common regular data register for dual and triple modes
} ADC_RegDef_t;

/*
 * Peripheral register definition structure for UART
 */

typedef struct
{
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t BRR;
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t CR3;
	__vo uint32_t GTPR;
}USART_RegDef_t;

/*
 *  peripheral definitions (Peripheral base addresses typecasted to xxx_RegDef_t)
 */

#define GPIOA 	( (GPIO_RegDef_t*) GPIOA_BASEADDR )
#define GPIOB 	( (GPIO_RegDef_t*) GPIOB_BASEADDR )
#define GPIOC 	( (GPIO_RegDef_t*) GPIOC_BASEADDR )
#define GPIOD 	( (GPIO_RegDef_t*) GPIOD_BASEADDR )
#define GPIOE 	( (GPIO_RegDef_t*) GPIOE_BASEADDR )
#define GPIOF 	( (GPIO_RegDef_t*) GPIOF_BASEADDR )
#define GPIOG 	( (GPIO_RegDef_t*) GPIOG_BASEADDR )
#define GPIOH 	( (GPIO_RegDef_t*) GPIOH_BASEADDR )

#define RCC		( (RCC_RegDef_t*) RCC_BASEADDR )

#define EXTI	( (EXTI_RegDef_t*) EXTI_BASEADDR )

#define SYSCFG	( (SYSCFG_RegDef_t*) SYSCFG_BASEADDR )

#define DMA1	( (DMA_RegDef_t*) DMA1_BASEADDR )
#define DMA2	( (DMA_RegDef_t*) DMA2_BASEADDR )

#define ADC1	( (ADC_RegDef_t*) ADC1_BASEADDR )
#define ADC2	( (ADC_RegDef_t*) ADC2_BASEADDR )
#define ADC3	( (ADC_RegDef_t*) ADC3_BASEADDR )


/*
 * Peripheral definition for USART
 */

#define USART1	((USART_RegDef_t*)USART1_BASEADDR)
#define USART2	((USART_RegDef_t*)USART2_BASEADDR)
#define USART3	((USART_RegDef_t*)USART3_BASEADDR)
#define UART4	((USART_RegDef_t*)UART4_BASEADDR)
#define UART5	((USART_RegDef_t*)UART5_BASEADDR)
#define USART6	((USART_RegDef_t*)USART6_BASEADDR)


/*
 * peripheral definitions for timers
 */
#define TIM1			((TIM_RegDef_t*) TIM1_BASEADDR)
#define TIM2			((TIM_RegDef_t*) TIM2_BASEADDR)
#define TIM3			((TIM_RegDef_t*) TIM3_BASEADDR)
#define TIM4			((TIM_RegDef_t*) TIM4_BASEADDR)
#define TIM5			((TIM_RegDef_t*) TIM5_BASEADDR)
#define TIM6			((TIM_RegDef_t*) TIM6_BASEADDR)
#define TIM7			((TIM_RegDef_t*) TIM7_BASEADDR)
#define TIM8			((TIM_RegDef_t*) TIM8_BASEADDR)
#define TIM9			((TIM_RegDef_t*) TIM9_BASEADDR)
#define TIM10			((TIM_RegDef_t*) TIM10_BASEADDR)
#define TIM11			((TIM_RegDef_t*) TIM11_BASEADDR)
#define TIM12			((TIM_RegDef_t*) TIM12_BASEADDR)
#define TIM13			((TIM_RegDef_t*) TIM13_BASEADDR)
#define TIM14			((TIM_RegDef_t*) TIM14_BASEADDR)

/*
 * Definition for Flash
 */
#define FLASH			((FLASH_RegDef_t*) FLASH_INTERFACE_BASEADDR)

/*
 * Clock Enable Macros for TIMx peripherals
 */
#define TIM1_PCLK_EN()			( RCC->APB2ENR |= (1 << 0))

#define TIM2_PCLK_EN()			( RCC->APB1ENR |= (1 << 0))
#define TIM3_PCLK_EN()			( RCC->APB1ENR |= (1 << 1))
#define TIM4_PCLK_EN()			( RCC->APB1ENR |= (1 << 2))
#define TIM5_PCLK_EN()			( RCC->APB1ENR |= (1 << 3))
#define TIM6_PCLK_EN()			( RCC->APB1ENR |= (1 << 4))
#define TIM7_PCLK_EN()			( RCC->APB1ENR |= (1 << 5))

#define TIM8_PCLK_EN()			( RCC->APB2ENR |= (1 << 1))

#define TIM9_PCLK_EN()			( RCC->APB2ENR |= (1 << 16))
#define TIM10_PCLK_EN()			( RCC->APB2ENR |= (1 << 17))
#define TIM11_PCLK_EN()			( RCC->APB2ENR |= (1 << 18))

#define TIM12_PCLK_EN()			( RCC->APB1ENR |= (1 << 6))
#define TIM13_PCLK_EN()			( RCC->APB1ENR |= (1 << 7))
#define TIM14_PCLK_EN()			( RCC->APB1ENR |= (1 << 8))

/*
 * Clock Enable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN() 		( RCC->AHB1ENR |= ( 1 << 0) )
#define GPIOB_PCLK_EN() 		( RCC->AHB1ENR |= ( 1 << 1) )
#define GPIOC_PCLK_EN() 		( RCC->AHB1ENR |= ( 1 << 2) )
#define GPIOD_PCLK_EN() 		( RCC->AHB1ENR |= ( 1 << 3) )
#define GPIOE_PCLK_EN() 		( RCC->AHB1ENR |= ( 1 << 4) )
#define GPIOF_PCLK_EN() 		( RCC->AHB1ENR |= ( 1 << 5) )
#define GPIOG_PCLK_EN() 		( RCC->AHB1ENR |= ( 1 << 6) )
#define GPIOH_PCLK_EN() 		( RCC->AHB1ENR |= ( 1 << 7) )

/*
 * Clock Enable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN() 			( RCC->APB1ENR |= ( 1 << 21) )
#define I2C2_PCLK_EN() 			( RCC->APB1ENR |= ( 1 << 22) )
#define I2C3_PCLK_EN() 			( RCC->APB1ENR |= ( 1 << 23) )

/*
 * Clock Enable Macros for SPIx peripherals
 */
#define SPI1_PCLK_EN() 			( RCC->APB2ENR |= ( 1 << 12) )
#define SPI2_PCLK_EN() 			( RCC->APB1ENR |= ( 1 << 14) )
#define SPI3_PCLK_EN() 			( RCC->APB1ENR |= ( 1 << 15) )
#define SPI4_PCLK_EN() 			( RCC->APB2ENR |= ( 1 << 13) )

/*
 * Clock Enable Macros for USARTx peripherals
 */
#define USART1_PCLK_EN() 		( RCC->APB2ENR |= ( 1 << 4) )
#define USART2_PCLK_EN() 		( RCC->APB1ENR |= ( 1 << 17) )
#define USART3_PCLK_EN() 		( RCC->APB1ENR |= ( 1 << 18) )
#define USART6_PCLK_EN() 		( RCC->APB2ENR |= ( 1 << 5) )

/*
 * Clock Enable Macros for UARTx peripherals
 */
#define UART4_PCLK_EN() 		( RCC->APB1ENR |= ( 1 << 19) )
#define UART5_PCLK_EN() 		( RCC->APB1ENR |= ( 1 << 20) )

/*
 * Clock Enable Macros for SYSFG peripherals
 */
#define SYSCFGEN_PCLK_EN() 		( RCC->APB2ENR |= ( 1 << 14) )

/*
 * Clock Enable Macros for DMA peripherals
 */
#define DMA1_PCLK_EN() 			( RCC->AHB1ENR |= ( 1 << 21) )
#define DMA2_PCLK_EN() 			( RCC->AHB1ENR |= ( 1 << 22) )

/*
 * Clock Enable Macros for ADC peripherals
 */
#define ADC1_PCLK_EN() 			( RCC->APB2ENR |= ( 1 << 8) )
#define ADC2_PCLK_EN() 			( RCC->APB2ENR |= ( 1 << 9) )
#define ADC3_PCLK_EN() 			( RCC->APB2ENR |= ( 1 << 10) )

/*
 * Macros to reset GPIOx peripherals
 */
#define	GPIOA_REG_RESET()		do { (RCC->AHB1RSTR |=(1<<0)); (RCC->AHB1RSTR &=~(1<<0)); } while(0)
#define	GPIOB_REG_RESET()		do { (RCC->AHB1RSTR |=(1<<1)); (RCC->AHB1RSTR &=~(1<<1)); } while(0)
#define	GPIOC_REG_RESET()		do { (RCC->AHB1RSTR |=(1<<2)); (RCC->AHB1RSTR &=~(1<<2)); } while(0)
#define	GPIOD_REG_RESET()		do { (RCC->AHB1RSTR |=(1<<3)); (RCC->AHB1RSTR &=~(1<<3)); } while(0)
#define	GPIOE_REG_RESET()		do { (RCC->AHB1RSTR |=(1<<4)); (RCC->AHB1RSTR &=~(1<<4)); } while(0)
#define	GPIOF_REG_RESET()		do { (RCC->AHB1RSTR |=(1<<5)); (RCC->AHB1RSTR &=~(1<<5)); } while(0)
#define	GPIOG_REG_RESET()		do { (RCC->AHB1RSTR |=(1<<6)); (RCC->AHB1RSTR &=~(1<<6)); } while(0)
#define	GPIOH_REG_RESET()		do { (RCC->AHB1RSTR |=(1<<7)); (RCC->AHB1RSTR &=~(1<<7)); } while(0)

/*
 *TIMER Reset Macros
 */
// APB2 Timers
#define TIM1_REG_RESET()		do { (RCC->APB2RSTR |=(1<<0));  (RCC->APB2RSTR &=~(1<<0)); } while(0)
#define TIM8_REG_RESET()		do { (RCC->APB2RSTR |=(1<<1));  (RCC->APB2RSTR &=~(1<<1)); } while(0)
#define TIM9_REG_RESET()		do { (RCC->APB2RSTR |=(1<<16)); (RCC->APB2RSTR &=~(1<<16)); } while(0)
#define TIM10_REG_RESET()		do { (RCC->APB2RSTR |=(1<<17)); (RCC->APB2RSTR &=~(1<<17)); } while(0)
#define TIM11_REG_RESET()		do { (RCC->APB2RSTR |=(1<<18)); (RCC->APB2RSTR &=~(1<<18)); } while(0)

// APB1 Timers
#define TIM2_REG_RESET()		do { (RCC->APB1RSTR |=(1<<0)); (RCC->APB1RSTR &=~(1<<0)); } while(0)
#define TIM3_REG_RESET()		do { (RCC->APB1RSTR |=(1<<1)); (RCC->APB1RSTR &=~(1<<1)); } while(0)
#define TIM4_REG_RESET()		do { (RCC->APB1RSTR |=(1<<2)); (RCC->APB1RSTR &=~(1<<2)); } while(0)
#define TIM5_REG_RESET()		do { (RCC->APB1RSTR |=(1<<3)); (RCC->APB1RSTR &=~(1<<3)); } while(0)
#define TIM6_REG_RESET()		do { (RCC->APB1RSTR |=(1<<4)); (RCC->APB1RSTR &=~(1<<4)); } while(0)
#define TIM7_REG_RESET()		do { (RCC->APB1RSTR |=(1<<5)); (RCC->APB1RSTR &=~(1<<5)); } while(0)
#define TIM12_REG_RESET()		do { (RCC->APB1RSTR |=(1<<6)); (RCC->APB1RSTR &=~(1<<6)); } while(0)
#define TIM13_REG_RESET()		do { (RCC->APB1RSTR |=(1<<7)); (RCC->APB1RSTR &=~(1<<7)); } while(0)
#define TIM14_REG_RESET()		do { (RCC->APB1RSTR |=(1<<8)); (RCC->APB1RSTR &=~(1<<8)); } while(0)

/*
 * DMAx reset macros
 */
#define	DMA1_REG_RESET()		do { (RCC->AHB1RSTR |=(1<<21)); (RCC->AHB1RSTR &=~(1<<21)); } while(0)
#define	DMA2_REG_RESET()		do { (RCC->AHB1RSTR |=(1<<22)); (RCC->AHB1RSTR &=~(1<<22)); } while(0)

/*
 * ADC reset macros
 */
#define ADC_REG_RESET()			do { (RCC->APB2RSTR |=(1<<8)); (RCC->APB2RSTR &=~(1<<8)); } while(0)

/*
 * Macros to reset USARTx peripherals
 */
#define USART1_REG_RESET()	do{ (RCC->APB2RSTR |= (1<<4)); (RCC->APB2RSTR &= ~(1<<4));}while(0)
#define USART2_REG_RESET()	do{ (RCC->APB1RSTR |= (1<<17)); (RCC->APB1RSTR &= ~(1<<17));}while(0)
#define USART3_REG_RESET()	do{ (RCC->APB1RSTR |= (1<<18)); (RCC->APB1RSTR &= ~(1<<18));}while(0)
#define UART4_REG_RESET()	do{ (RCC->APB1RSTR |= (1<<19)); (RCC->APB1RSTR &= ~(1<<19));}while(0)
#define UART5_REG_RESET()	do{ (RCC->APB1RSTR |= (1<<20)); (RCC->APB1RSTR &= ~(1<<20));}while(0)
#define USART6_REG_RESET()	do{ (RCC->APB2RSTR |= (1<<5)); (RCC->APB2RSTR &= ~(1<<5));}while(0)

/*
 * returns port code for given GPIOx base address
 */

#define GPIO_BASEADDR_TO_CODE(x)  (	(x==GPIOA) ? 0 :\
									(x==GPIOB) ? 1 :\
									(x==GPIOC) ? 2 :\
									(x==GPIOD) ? 3 :\
									(x==GPIOE) ? 4 :\
									(x==GPIOF) ? 5 :\
									(x==GPIOG) ? 6 :\
									(x==GPIOH) ? 7 :0 )
/*
 * Clock Disable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI() 		( RCC->AHB1ENR &= ~( 1 << 0) )
#define GPIOB_PCLK_DI() 		( RCC->AHB1ENR &= ~( 1 << 1) )
#define GPIOC_PCLK_DI() 		( RCC->AHB1ENR &= ~( 1 << 2) )
#define GPIOD_PCLK_DI() 		( RCC->AHB1ENR &= ~( 1 << 3) )
#define GPIOE_PCLK_DI() 		( RCC->AHB1ENR &= ~( 1 << 4) )
#define GPIOF_PCLK_DI() 		( RCC->AHB1ENR &= ~( 1 << 5) )
#define GPIOG_PCLK_DI() 		( RCC->AHB1ENR &= ~( 1 << 6) )
#define GPIOH_PCLK_DI() 		( RCC->AHB1ENR &= ~( 1 << 7) )

/*
 * Clock Disable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_DI() 			( RCC->APB1ENR &= ~( 1 << 21) )
#define I2C2_PCLK_DI() 			( RCC->APB1ENR &= ~( 1 << 22) )
#define I2C3_PCLK_DI() 			( RCC->APB1ENR &= ~( 1 << 23) )

/*
 * Clock Disable Macros for SPIx peripherals
 */
#define SPI1_PCLK_DI() 			( RCC->APB2ENR &= ~( 1 << 12) )
#define SPI2_PCLK_DI() 			( RCC->APB1ENR &= ~( 1 << 14) )
#define SPI3_PCLK_DI() 			( RCC->APB1ENR &= ~( 1 << 15) )
#define SPI4_PCLK_DI() 			( RCC->APB2ENR &= ~( 1 << 13) )

/*
 * Clock Disable Macros for USARTx peripherals
 */
#define USART1_PCLK_DI() 		( RCC->APB2ENR &= ~( 1 << 4) )
#define USART2_PCLK_DI() 		( RCC->APB1ENR &= ~( 1 << 17) )
#define USART3_PCLK_DI() 		( RCC->APB1ENR &= ~( 1 << 18) )
#define USART6_PCLK_DI() 		( RCC->APB2ENR &= ~( 1 << 5) )

/*
 * Clock Disable Macros for UARTx peripherals
 */
#define UART4_PCLK_DI() 		( RCC->APB1ENR &= ~( 1 << 19) )
#define UART5_PCLK_DI() 		( RCC->APB1ENR &= ~( 1 << 20) )

/*
 * Clock Disable Macros for SYSFG peripherals
 */
#define SYSCFGEN_PCLK_DI() 		( RCC->APB2ENR &= ~( 1 << 14) )

/*
 * Clock Disable Macros for TIMx peripherals
 */
#define TIM1_PCLK_DI()			( RCC->APB2ENR &= ~( 1 << 0) )

#define TIM2_PCLK_DI()			( RCC->APB1ENR &= ~( 1 << 0) )
#define TIM3_PCLK_DI()			( RCC->APB1ENR &= ~( 1 << 1) )
#define TIM4_PCLK_DI()			( RCC->APB1ENR &= ~( 1 << 2) )
#define TIM5_PCLK_DI()			( RCC->APB1ENR &= ~( 1 << 3) )
#define TIM6_PCLK_DI()			( RCC->APB1ENR &= ~( 1 << 4) )
#define TIM7_PCLK_DI()			( RCC->APB1ENR &= ~( 1 << 5) )

#define TIM8_PCLK_DI()			( RCC->APB2ENR &= ~( 1 << 1) )

#define TIM9_PCLK_DI()			( RCC->APB2ENR &= ~( 1 << 16) )
#define TIM10_PCLK_DI()			( RCC->APB2ENR &= ~( 1 << 17) )
#define TIM11_PCLK_DI()			( RCC->APB2ENR &= ~( 1 << 18) )

#define TIM12_PCLK_DI()			( RCC->APB1ENR &= ~( 1 << 6) )
#define TIM13_PCLK_DI()			( RCC->APB1ENR &= ~( 1 << 7) )
#define TIM14_PCLK_DI()			( RCC->APB1ENR &= ~( 1 << 8) )

/*
 * Clock Disable Macros for DMA peripherals
 */
#define DMA1_PCLK_DI() 			( RCC->AHB1ENR &= ~( 1 << 21) )
#define DMA2_PCLK_DI() 			( RCC->AHB1ENR &= ~( 1 << 22) )

/*
 * Clock Disable Macros for ADC peripherals
 */
#define ADC1_PCLK_DI() 			( RCC->APB2ENR &= ~( 1 << 8) )
#define ADC2_PCLK_DI() 			( RCC->APB2ENR &= ~( 1 << 9) )
#define ADC3_PCLK_DI() 			( RCC->APB2ENR &= ~( 1 << 10) )

/*
 * Some generic macros
 */
#define ENABLE 						1
#define DISABLE						0
#define SET							ENABLE
#define	RESET						DISABLE
#define GPIO_PIN_SET				ENABLE
#define	GPIO_PIN_RESET				DISABLE
#define FLAG_RESET	RESET
#define FLAG_SET	SET

#define IRQ_NO_EXTI0				6
#define IRQ_NO_EXTI1				7
#define IRQ_NO_EXTI2				8
#define IRQ_NO_EXTI3				9
#define IRQ_NO_EXTI4				10
#define IRQ_NO_EXTI9_5				23
#define IRQ_NO_EXTI15_10			40

#define IRQ_NO_TIM1_BRK_TIM9		24
#define IRQ_NO_TIM1_UP_TIM10		25
#define IRQ_NO_TIM1_TRG_COM_TIM11	26
#define IRQ_NO_TIM1_CC				27
#define IRQ_NO_TIM2					28
#define IRQ_NO_TIM3					29
#define IRQ_NO_TIM4					30
#define IRQ_NO_TIM8_BRK_TIM12		43
#define IRQ_NO_TIM8_UP_TIM13		44
#define IRQ_NO_TIM8_TRG_COM_TIM14	45
#define IRQ_NO_TIM8_CC				46
#define IRQ_NO_TIM5					50
#define IRQ_NO_TIM6_DAC				54
#define IRQ_NO_TIM7					55

#define IRQ_NO_DMA1_STREAM0			11
#define IRQ_NO_DMA1_STREAM1			12
#define IRQ_NO_DMA1_STREAM2			13
#define IRQ_NO_DMA1_STREAM3			14
#define IRQ_NO_DMA1_STREAM4			15
#define IRQ_NO_DMA1_STREAM5			16
#define IRQ_NO_DMA1_STREAM6			17
#define IRQ_NO_DMA1_STREAM7			47

#define IRQ_NO_DMA2_STREAM0			56
#define IRQ_NO_DMA2_STREAM1			57
#define IRQ_NO_DMA2_STREAM2			58
#define IRQ_NO_DMA2_STREAM3			59
#define IRQ_NO_DMA2_STREAM4			60
#define IRQ_NO_DMA2_STREAM5			68
#define IRQ_NO_DMA2_STREAM6			69
#define IRQ_NO_DMA2_STREAM7			70

#define IRQ_NO_USART1				37
#define IRQ_NO_USART2				38
#define IRQ_NO_USART3				39
#define IRQ_NO_UART4				52
#define IRQ_NO_UART5				53
#define IRQ_NO_USART6				71

/*
 * Bits definition of USART peripheral
 */

#define USART_SR_PE			0
#define USART_SR_FE			1
#define USART_SR_NF			2
#define USART_SR_ORE		3
#define USART_SR_IDLE		4
#define USART_SR_RXNE		5
#define USART_SR_TC			6
#define USART_SR_TXE		7
#define USART_SR_LBD		8
#define USART_SR_CTS		9

#define USART_BRR_DIV_FRACTION	0
#define USART_BRR_DIV_MANTISSA	4

#define USART_CR1_SBK		0
#define USART_CR1_RWU		1
#define USART_CR1_RE		2
#define USART_CR1_TE		3
#define USART_CR1_IDLEIE	4
#define USART_CR1_RXNEIE	5
#define USART_CR1_TCIE		6
#define USART_CR1_TXEIE		7
#define USART_CR1_PEIE		8
#define USART_CR1_PS		9
#define USART_CR1_PCE		10
#define USART_CR1_WAKE		11
#define USART_CR1_M			12
#define USART_CR1_UE		13
#define USART_CR1_OVER8		15

#define USART_CR2_ADD		0
#define USART_CR2_LBDL		5
#define USART_CR2_LBDIE		6
#define USART_CR2_RES		7
#define USART_CR2_LBCL		8
#define USART_CR2_CPHA		9
#define USART_CR2_CPOL		10
#define USART_CR2_CLKEN		11
#define USART_CR2_STOP		12
#define USART_CR2_LINEN		14

#define USART_CR3_EIE		0
#define USART_CR3_IREN		1
#define USART_CR3_IRLP		2
#define USART_CR3_HDSEL		3
#define USART_CR3_NACK		4
#define USART_CR3_SCEN		5
#define USART_CR3_DMAR		6
#define USART_CR3_DMAT		7
#define USART_CR3_RTSE		8
#define USART_CR3_CTSE		9
#define USART_CR3_CTSIE		10
#define USART_CR3_ONEBIT	11

#define USART_GTPR_PSC		0
#define USART_GTPR_GT		8



#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_rcc_driver.h"
#include "stm32f446xx_timer_driver.h"
#include "stm32f446xx_dma_driver.h"
#include "stm32f446xx_adc_driver.h"
#include "stm32f446xx_uart_driver.h"

#endif /* INC_STM32F446_H_ */
