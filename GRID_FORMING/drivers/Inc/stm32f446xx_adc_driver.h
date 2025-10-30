/*
 * stm32f446_adc_driver.h
 *
 *  Created on: Oct 20, 2025
 *      Author: Isaac Pérez (ShiLiba)
 */

#ifndef INC_STM32F446_ADC_DRIVER_H_
#define INC_STM32F446_ADC_DRIVER_H_

#include "stm32f446xx.h"

/*
 * @ADC_RESOLUTION
 */
#define ADC_RESOLUTION_12_B					0
#define ADC_RESOLUTION_10_B					1
#define ADC_RESOLUTION_8_B					2
#define ADC_RESOLUTION_6_B					3

/*
 * @ADC_DATA_ALIGNMENT
 */
#define ADC_DATA_ALIGNMENT_RIGHT			0
#define ADC_DATA_ALIGNMENT_LEFT				1

/*
 * @ADC_SCAN_MODE
 */
#define ADC_SCAN_MODE_DI					0
#define ADC_SCAN_MODE_EN					1

/*
 * @ADC_CONV_MODE
 */
#define ADC_CONV_MODE_SINGLE				0
#define	ADC_CONV_MODE_CONTINUOUS			1
#define	ADC_CONV_MODE_DISCONTINUOUS			2

/*
 * @ADC_EXT_TRI
 */
#define	ADC_EXT_TRIG_TIM1_CC1				0
#define	ADC_EXT_TRIG_TIM1_CC2				1
#define	ADC_EXT_TRIG_TIM1_CC3				2
#define	ADC_EXT_TRIG_TIM2_CC2				3
#define	ADC_EXT_TRIG_TIM2_CC3				4
#define	ADC_EXT_TRIG_TIM2_CC4				5
#define	ADC_EXT_TRIG_TIM2_TRGO				6
#define	ADC_EXT_TRIG_TIM3_CC1				7
#define	ADC_EXT_TRIG_TIM3_TRGO				8
#define	ADC_EXT_TRIG_TIM4_CC4				9
#define	ADC_EXT_TRIG_TIM5_CC1				10
#define	ADC_EXT_TRIG_TIM5_CC2				11
#define	ADC_EXT_TRIG_TIM5_CC3				12
#define	ADC_EXT_TRIG_TIM8_CC1				13
#define	ADC_EXT_TRIG_TIM8_TRGO				14
#define	ADC_EXT_TRIG_EXTI_11				15

/*
 *
 */
#define	ADC_EXT_TRIG_DECT_DI				0
#define	ADC_EXT_TRIG_DECT_RE				1
#define	ADC_EXT_TRIG_DECT_FE				2
#define	ADC_EXT_TRIG_DECT_RF				3

/*
 *
 */
#define	ADC_EOC_PER_SEQUENCE				0
#define	ADC_EOC_PER_CONVERSION				1

/*
 *
 */
#define	ADC_EOC_IT_DI						0
#define	ADC_EOC_IT_EN						1


/*
 * @ADC_DMA_MODE
 */
#define ADC_DMA_MODE_DI						0
#define ADC_DMA_MODE_EN						1

/*
 * @ADC_SMP_T
 */
#define ADC_SMP_T_3							0
#define ADC_SMP_T_15						1
#define ADC_SMP_T_28						2
#define ADC_SMP_T_56						3
#define ADC_SMP_T_84						4
#define ADC_SMP_T_112						5
#define ADC_SMP_T_144						6
#define ADC_SMP_T_480						7

#define ADC_DDS_NO_RQ						0
#define ADC_DDS_RQ							1


/*
 * This is Configuration structure for ADC
 */
typedef struct
{
	uint8_t ADC_Resolution;							/* <  possible values from @ADC_RESOLUTION > */
	uint8_t ADC_DataAlignment;						/* <  possible values from @ADC_DATA_ALIGNMENT > */
	uint8_t ADC_ScanMode;							/* <  possible values from @ADC_SCAN_MODE > */
	uint8_t ADC_ConversionMode;						/* <  possible values from @ADC_CONV_MODE > */
	uint8_t ADC_ExternalTriggerDetection;
	uint8_t ADC_ExternalTrigger;					/* <  possible values from @ADC_EXTL_TRI> */
	uint8_t ADC_DMAContinuousRequests;				/* <  possible values from @ADC_DMA_MODE > */
	uint8_t ADC_EOCSelection;
	uint8_t ADC_DDSelection;
	uint8_t ADC_EOCInterrupt;
}ADC_Config_t;

/*
 * This is Handle structure for ADC
 */
typedef struct
{
	ADC_RegDef_t *pADCx;							/* <  This holds the base address of the ADC > */
	ADC_Config_t ADC_Config;						/* <  This holds adc configuration settings > */
	uint8_t ADC_NumChannels;
	uint8_t ADC_Channels[16];
	uint8_t ADC_SamplingTime[16];					/* <  possible values from @ADC_SMP_T > */
}ADC_Handle_t;

/************************************************************************************
 * 							APIs supported by this driver
 * 				For more information about the APIS check the function definitions
 ************************************************************************************/

/*
 * Peripheral Clock setup
 */
void ADC_PClkC(ADC_RegDef_t *pADCx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void ADC_Init(ADC_Handle_t *pADCHandle);
void ADC_DeInit(void);

/*
 * Channel configuration
 */
void ADC_ChannelConfig(ADC_Handle_t *pADCHandle, uint8_t channel, uint8_t rank, uint8_t samplingTime);
void ADC_ConfigSequence(ADC_Handle_t *pADCHandle);

/*
 * Conversion control
 */
void ADC_StartConversion(ADC_Handle_t *pADCHandle);
void ADC_StopConversion(ADC_Handle_t *pADCHandle);

/*
 * Data read
 */
uint16_t ADC_ReadData(ADC_Handle_t *pADCHandle);
uint8_t ADC_GetConversionStatus(ADC_Handle_t *pADCHandle);

/*
 * IRQ and ISR Handling
 */
void ADC_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void ADC_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void ADC_IRQHandling(ADC_Handle_t *pADCHandle);

/*
 * Status and utility
 */
void ADC_DMAControl(ADC_Handle_t *pADCHandle, uint8_t EnorDi);
void ADC_ConfigWatchdog(ADC_Handle_t *pADCHandle, uint16_t LowerLimit , uint16_t UpperLimit);


void ADC_SoftReset(ADC_Handle_t *pADCHandle);

#endif /* INC_STM32F446_ADC_DRIVER_H_ */
