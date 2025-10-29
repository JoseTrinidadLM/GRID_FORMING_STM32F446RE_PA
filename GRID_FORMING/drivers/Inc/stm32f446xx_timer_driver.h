/*
 * stm32f446_timer_driver.h
 *
 *  Created on: Oct 14, 2025
 *      Author: Isaac Perez (ShiLiba)
 */

#ifndef INC_STM32F446_TIMER_DRIVER_H_
#define INC_STM32F446_TIMER_DRIVER_H_

#include "stm32f446xx.h"

/*
 * @TIM_CKD
 *  Timer clock division
 */

#define	TIM_CKD_DIV1				0
#define TIM_CKD_DIV2				1
#define TIM_CKD_DIV4				2

/*
 * @TIM_CNT
 * Timer count mode up/down
 */
#define	TIM_UPCOUNT_MODE			0
#define TIM_DOWNCOUNT_MODE			1

/*
 * @TIM_ARPE
 * Timer count mode up/down
 */
#define	TIM_ARPE_ENABLE				1
#define TIM_ARPE_DISABLE			0

/*
 * @TIM_IT
 * DIER register, updates interrupt enable
 */
#define	TIM_IT_ENABLE			1
#define	TIM_IT_DISABLE			0

/*
 *
 */
#define TIM_MMS_RESET			0
#define TIM_MMS_ENABLE			1
#define TIM_MMS_UPDATE			2
#define TIM_MMS_COMP_PUL		3
#define TIM_MMS_COMP_OC1		4
#define TIM_MMS_COMP_OC2		5
#define TIM_MMS_COMP_OC3		6
#define TIM_MMS_COMP_OC4		7

/************************************************************************************
 * 							APIs supported by this driver
 * 				For more information about the APIS check the function definitions
 ************************************************************************************/

/*
 * This is Configuration structure for TIMER
 */
typedef struct
{
	uint32_t TIM_Frequency;				//Desired frequency
	uint8_t  TIM_CLKDivision;			/* <  possible values from @TIM_CKD > */
	uint8_t  TIM_CNTMode;				/* <  possible values from @TIM_CNT > */
	uint8_t  TIM_AutoReloadPreload;		/* <  possible values from @TIM_ARPE > */
	uint8_t  TIM_IntEnable;				/* <  possible values from @TIM_IT > */
	uint8_t	 TIM_MasterModeSel;
}TIM_Config_t;

/*
 * This is Handle structure for TIMx
 */
typedef struct
{
	TIM_RegDef_t *pTIMx;					/* <  This holds the base address of the TIM to which the pin belongs > */
	TIM_Config_t TIM_Config;		/* <  This holds TIM configuration settings > */
}TIM_Handle_t;

/*
 * TIM Clock setup
 */
void TIM_PeriClockControl(TIM_RegDef_t *pTIMx, uint8_t EnorDi);

/*
 * Init and de-init
 */
void TIM_Init(TIM_Handle_t *pTIMHandle);
void TIM_DeInit(TIM_RegDef_t *pTIMx);

/*
 *	Timer Start/Stop
 */
void TIM_Start(TIM_Handle_t *pTIMHandle);
void TIM_Stop(TIM_Handle_t *pTIMHandle);


/*
 * IRQ Configuration and ISR handling
 */
void TIM_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void TIM_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void TIM_IRQHandling(TIM_Handle_t *pTIMHandle);

/*
 * DMA Configuration for ADC conversion
 */
//void TIM_DMAConfig(TIM_RegDef_t *pTIMx);


#endif /* INC_STM32F446_TIMER_DRIVER_H_ */
