/*
 * stm32f446xx_control.h
 *
 *  Created on: Nov 18, 2025
 *      Author: jiperez
 */

#ifndef STM32F446XX_CONTROL_H_
#define STM32F446XX_CONTROL_H_

#include "stm32f446xx.h"

/*
 * Macros to set the status register
*/

#define MODE_FLAG		    	    1
#define SYSTEM_STATUS_FLAG			0

#define SET_OPEN_LOOP_MODE(var)         var &= ~(1 << MODE_FLAG)
#define SET_CLOSED_LOOP_MODE(var)       var |=  (1 << MODE_FLAG)

#define SYSTEM_OFF_FLAG(var)		    var &= ~(1 << SYSTEM_STATUS_FLAG)
#define SYSTEM_ON_FLAG(var)			    var |=  (1 << SYSTEM_STATUS_FLAG)

/*
 * Macros for Control Modes
 */
#define GRID_FOLLOWING_MODE         0b01
#define VAR_COMPENSATION_MODE       0b11
#define Off                         0b00

/*
 * Macros for documentation purpose
 */

#define BUFFER_LENGTH_9			9
#define START_TIME 			    0
#define SAMPLING_FREQUENCY		9600
#define PWM_FREQUENCY			9600
#define SAMPLING_PERIOD			(1.0f/9600.0f)  //in seconds
#define ADC_RESOLUTION			4095.0f
#define ADC_OFFSET_VOLTAGE		0.5f
#define ADC_VOLTAGE_REF		    2.0f
#define ADC_GRID_VOLTAGE_K	    27.5f
#define ADC_DC_VOLTAGE_K	    26.51f
#define ADC_INV_CURRENT_K	    3.02730f
#define ADC_LOAD_CURRENT_K	    3.07933f

void ControlInit(void);

void Control_Start(void);

void Control_Stop(void);

uint8_t Control_ReadSensors(float* values);

void Control_DutyCycle(void);

uint8_t Control_Mode(uint8_t Power, uint8_t Loop);

uint8_t Control_ChangeMode(uint8_t Status, uint8_t Flag);

void TIM2_IRQHandling(void);


#endif /* STM32F446XX_CONTROL_H_ */
