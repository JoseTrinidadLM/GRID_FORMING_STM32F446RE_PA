/*
 * stm32f446xx_control.h
 *
 *  Created on: Nov 18, 2025
 *      Author: jiperez
 */

#ifndef STM32F446XX_CONTROL_H_
#define STM32F446XX_CONTROL_H_

#include "stm32f446xx.h"

void Utility_GPIOInits(void);

void PWM_GPIOInits(void);

void Sensors_Init(void *pDest);

void SamplingRateTIMInit(float sampling_rate);

void PWM_TIMInits(float carrier_frequency);

float NINETYDegreePhaseShift(float *pCos_Buffer, float cos_wave, __vo uint8_t *pBuffer_Counter, __vo uint8_t *pBuffer_Ready_Flag);

float SignalDelay(float *pSignal_Buffer, float signal, __vo uint8_t *pBuffer_Counter, __vo uint8_t *pBuffer_Ready_Flag, uint8_t samples_to_delay);

float DTransform(float cosine_wt, float sine_wt, float alpha, float beta);

float QTransform(float cosine_wt, float sine_wt, float alpha, float beta);

void CascadeControl(float cosine_wt, float sine_wt, float V_CD, float I_Q, float I_INV, __vo float *pe1_z_0, __vo float *pe1_z_1, __vo float *pe2_z_0, __vo float *pe2_z_1, __vo float *py1_z_0, __vo float *py1_z_1, __vo float *py2_z_0, __vo float *py2_z_1, __vo uint16_t *u_pos, __vo uint16_t *u_neg);

void OpenLoop(float cosine_wt, __vo uint16_t *u_pos, __vo uint16_t *u_neg);

void ResetPIControllers(__vo float *pe1_z_0, __vo float *pe1_z_1, __vo float *pe2_z_0, __vo float *pe2_z_1, __vo float *py1_z_0, __vo float *py1_z_1, __vo float *py2_z_0, __vo float *py2_z_1);

void PWM_Enable(void);

void PWM_Disable(void);

void PWM_dutyCycle_control(uint16_t u_pos ,uint16_t u_neg);

void TIM2_IRQHandling(void);


#endif /* STM32F446XX_CONTROL_H_ */
