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
#define	TIM_IT_ENABLE				1
#define	TIM_IT_DISABLE				0

/*
 * @TIM_MMS
 * Master mode selection
 */
#define TIM_MMS_RESET				0
#define TIM_MMS_ENABLE				1
#define TIM_MMS_UPDATE				2
#define TIM_MMS_COMP_PUL			3
#define TIM_MMS_COMP_OC1			4
#define TIM_MMS_COMP_OC2			5
#define TIM_MMS_COMP_OC3			6
#define TIM_MMS_COMP_OC4			7

/*
 * @TIM_SMS
 * Master mode selection
 */
#define TIM_SMS_DISABLE      		0
#define TIM_SMS_ENCODER1     		1
#define TIM_SMS_ENCODER2 		    2
#define TIM_SMS_ENCODER3 		    3
#define TIM_SMS_RESET        		4
#define TIM_SMS_GATED        		5
#define TIM_SMS_TRIGGER      		6
#define TIM_SMS_EXTERNAL    		7

/*
 * @TIM_TS
 * Trigger source
 */
#define TIM_TS_ITR0                 0  // TIM1
#define TIM_TS_ITR1                 1  // TIM2
#define TIM_TS_ITR2                 2  // TIM3
#define TIM_TS_ITR3                 3  // TIM4
#define TIM_TS_TI1F_ED              4  // TI1 Edge Detector
#define TIM_TS_TI1FP1               5  // Filtered Timer Input 1
#define TIM_TS_TI2FP2               6  // Filtered Timer Input 2
#define TIM_TS_ETRF                 7  // External Trigger

/*
 * @TIM_MODE
 * This macro allows to configure PWM features if needed
 */
#define TIM_MODE_BASIC          	0
#define TIM_MODE_PWM              	1


/*
 * @TIM_CHANNEL
 */
#define TIM_CHANNEL_1   	       	1
#define TIM_CHANNEL_2              	2
#define TIM_CHANNEL_3 	         	3
#define TIM_CHANNEL_4              	4

/*
 *	@TIM_PWM
 *	For this application it only configures PWM1 and PWM2
 */
#define TIM_PWM_MODE1            	0x6
#define TIM_PWM_MODE2             	0x7

/*
 *	@TIM_OC_POLARITY
 */
#define TIM_OC_POLARITY_HIGH      	0
#define TIM_OC_POLARITY_LOW       	1

/*
 *	@TIM_OC_PRELOAD
 */
#define TIM_OC_PRELOAD_DISABLED     0
#define TIM_OC_PRELOAD_ENABLED      1


/************************************************************************************
 * 							APIs supported by this driver
 * 				For more information about the APIS check the function definitions
 ************************************************************************************/

/*
 * This is Configuration structure for TIMER
 */
typedef struct
{
	uint64_t TIM_Frequency;				//Desired frequency
	uint8_t  TIM_CLKDivision;			/* <  possible values from @TIM_CKD > */
	uint8_t  TIM_CNTMode;				/* <  possible values from @TIM_CNT > */
	uint8_t  TIM_AutoReloadPreload;		/* <  possible values from @TIM_ARPE > */
	uint8_t  TIM_IntEnable;				/* <  possible values from @TIM_IT > */
	uint8_t	 TIM_MasterModeSel;			/* <  possible values from @TIM_MSM > */
	uint8_t  TIM_SlaveMode;				/* <  possible values from @TIM_SMS > */
	uint8_t  TIM_TriggerSource;			/* <  possible values from @TIM_TS > */

	uint8_t	 TIM_Mode;					/* <  possible values from @TIM_MODE > */
	uint8_t	 TIM_Channel;				/* <  possible values from @TIM_CHANNEL > */
	uint8_t	 TIM_OCMode;				/* <  possible values from @TIM_PWM > */
	uint8_t	 TIM_OCPolarity;			/* <  possible values from @TIM_OC_POLARITY > */
	uint8_t	 TIM_OCPreload;				/* <  possible values from @TIM_OC_PRELOAD > */
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
void TIM_PClkC(TIM_RegDef_t *pTIMx, uint8_t EnorDi);

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

void TIM_PWM_Channel_Init(TIM_Handle_t *pTIMHandle);

void TIM_PWM_DutyCycle(TIM_Handle_t *pTIMHandle, uint16_t dutyCycle);

/*
 * DMA Configuration for ADC conversion
 */
//void TIM_DMAConfig(TIM_RegDef_t *pTIMx);


#endif /* INC_STM32F446_TIMER_DRIVER_H_ */
