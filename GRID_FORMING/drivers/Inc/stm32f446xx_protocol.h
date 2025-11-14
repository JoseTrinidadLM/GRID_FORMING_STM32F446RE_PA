/*
 * stm32f446xx_protocol.h
 *
 *  Created on: Nov 14, 2025
 *      Author: jtlopez
 */

#ifndef INC_STM32F446XX_PROTOCOL_H_
#define INC_STM32F446XX_PROTOCOL_H_

#include "stm32f446xx.h"

/*
 *		APIs
 *
 */

void ProtocolInit(USART_RegDef_t *pUSARTx, float *Buffer_values, uint8_t *Buffer_heartbeat);

void Protocol_TIMInit(TIM_RegDef_t *pTIMx);

void Protocol_Start(void);

void Protocol_Telemetry(void);

__weak void executeCommand(uint8_t command);


#endif /* INC_STM32F446XX_PROTOCOL_H_ */
