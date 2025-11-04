/*
 * stm32f446xx_rtc_driver.h
 *
 *  Created on: Nov 4, 2025
 *      Author: jtlopez
 */

#ifndef INC_STM32F446XX_RTC_DRIVER_H_
#define INC_STM32F446XX_RTC_DRIVER_H_

#include "stm32f446xx.h"

/*
 * Configuration Structure for RTC
 */

typedef struct
{

}RTC_Config_t;

/*
 * Handling Structure for GPIO
 */

typedef struct
{
	RTC_RegDef_t *pRTC;					
	RTC_Config_t RTC_Config;		    
}RTC_Handle_t;

#endif /* INC_STM32F446XX_RTC_DRIVER_H_ */
