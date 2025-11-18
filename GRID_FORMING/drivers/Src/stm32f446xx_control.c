/*
 * stm32f446xx_control.c
 *
 *  Created on: Nov 18, 2025
 *      Author: jiperez
 */

#include "stm32f446xx_control.h"

static GPIO_Handle_t PWM_EN;
static GPIO_Handle_t LOOP_SEL;

static GPIO_Handle_t GpioPWMA;
static GPIO_Handle_t GpioPWMB;

static GPIO_Handle_t GPIO_Sensor[4];
static ADC_Handle_t ADC_1;
static DMA_Handle_t DMA2_ADC1Handle;


static TIM_Handle_t TIM_2;

static TIM_Handle_t TIM_4;
static PWM_Config_t TIM4_PWM_Channel_1;
static PWM_Config_t TIM4_PWM_Channel_2;

/*GPIO pins 14-15 from port B are declared as input that activate EXTI15_10 to control PWM on/off, as well as change Operation Mode*/
void Utility_GPIOInits(void)
{
	GPIO_PClkC(GPIOB, ENABLE);
	PWM_EN.pGPIOx = GPIOB;
	PWM_EN.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	PWM_EN.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RFT;
	PWM_EN.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&PWM_EN);

	LOOP_SEL.pGPIOx = GPIOB;
	LOOP_SEL.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	LOOP_SEL.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RFT;
	LOOP_SEL.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&LOOP_SEL);

	GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10,ENABLE);
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, 0);
}

/*GPIO pin 7 from port B and GPIO pin 6 from port B are declared as High Output Speed for PWM signals*/
void PWM_GPIOInits(void)
{
	GPIO_PClkC(GPIOB, ENABLE);

	GpioPWMA.pGPIOx = GPIOB;
	GpioPWMA.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GpioPWMA.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	GpioPWMA.GPIO_PinConfig.GPIO_PinAltFunMode = 2;
	GpioPWMA.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GpioPWMA.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioPWMA.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GpioPWMA);

	GpioPWMB.pGPIOx = GPIOB;
	GpioPWMB.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GpioPWMB.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	GpioPWMB.GPIO_PinConfig.GPIO_PinAltFunMode = 2;
	GpioPWMB.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GpioPWMB.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioPWMB.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GpioPWMB);
}

/*Add as much sensors you want to read (look up pinout for compatibility) */

void Sensors_Init(void *pDest) /*For this application only 4 sensors will be initialized (name may be changed)*/
{
	/*****************GPIO analog inputs initialization*****************/
	GPIO_PClkC(GPIOA, ENABLE);

	GPIO_Sensor[0].pGPIOx = GPIOA;
	GPIO_Sensor[0].GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ANALOG;
	GPIO_Sensor[0].GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIO_Sensor[0].GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&GPIO_Sensor[0]);

	GPIO_Sensor[1].pGPIOx = GPIOA;
	GPIO_Sensor[1].GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ANALOG;
	GPIO_Sensor[1].GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_1;
	GPIO_Sensor[1].GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&GPIO_Sensor[1]);

	GPIO_Sensor[2].pGPIOx = GPIOA;
	GPIO_Sensor[2].GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ANALOG;
	GPIO_Sensor[2].GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
	GPIO_Sensor[2].GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&GPIO_Sensor[2]);


	GPIO_Sensor[3].pGPIOx = GPIOA;
	GPIO_Sensor[3].GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ANALOG;
	GPIO_Sensor[3].GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Sensor[3].GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&GPIO_Sensor[3]);

	/*****************ADC1 initialization*****************/

	/*ADC configured to start conversion by external trigger set by TIM2 and transfer data via DMA*/
	ADC_PClkC(ADC1, ENABLE);

   	ADC_1.pADCx = ADC1;
	ADC_1.ADC_Config.ADC_Resolution = ADC_RESOLUTION_12_B; 							//Max resolution
	ADC_1.ADC_Config.ADC_DataAlignment = ADC_DATA_ALIGNMENT_RIGHT; 					//Aligned to right to keep format of MSB-LSB
	ADC_1.ADC_Config.ADC_ScanMode =  ADC_SCAN_MODE_EN; 								//Since we want to read several pins, scan mode is enabled
	ADC_1.ADC_Config.ADC_ConversionMode =  ADC_CONV_MODE_SINGLE; 					//It converts just once
	ADC_1.ADC_Config.ADC_ExternalTriggerDetection =  ADC_EXT_TRIG_DECT_RE; 			//Starts conversion every rising edge of an external trigger
	ADC_1.ADC_Config.ADC_ExternalTrigger =  ADC_EXT_TRIG_TIM2_TRGO; 				//External trigger is commanded by TIM2
	ADC_1.ADC_Config.ADC_DMAContinuousRequests =  ADC_DMA_MODE_EN;					//Enables DMA
	ADC_1.ADC_Config.ADC_DDSelection =  ADC_DDS_RQ;									//DMA requests are issued as long as data are converted and DMA=1
	ADC_1.ADC_Config.ADC_EOCSelection =  ADC_EOC_PER_CONVERSION; 					//This set EOC flag to 1 at the end of each regular conversion
	ADC_1.ADC_Config.ADC_EOCInterrupt =  ADC_EOC_IT_DI; 							//Is disabled, ADC conversion and transfer of data is handled on TIM2_IRQHandler

	/*User may config reading sequence*/
	ADC_1.ADC_NumChannels = 4;
	ADC_ChannelConfig(&ADC_1, GPIO_PIN_NO_0, 0, ADC_SMP_T_15);						//Starts sequence on GPIO A0
	ADC_ChannelConfig(&ADC_1, GPIO_PIN_NO_1, 1, ADC_SMP_T_15);
	ADC_ChannelConfig(&ADC_1, GPIO_PIN_NO_4, 2, ADC_SMP_T_15);
	ADC_ChannelConfig(&ADC_1, GPIO_PIN_NO_6, 3, ADC_SMP_T_15);						//Ends sequence on GPIO A6
	ADC_ConfigSequence(&ADC_1);
	ADC_Init(&ADC_1);

	/*****************DMA initialization*****************/
	DMA_PClkC(DMA2, ENABLE);

	/*To transfer data from ADC1 to memory it uses Channel 0 and Stream 0*/
	/*To change this look up to Table 28-29 of RM0390 p.205*/
	DMA2_ADC1Handle.pDMAx = DMA2;

	DMA2_ADC1Handle.DMA_stream = 0;
	DMA2_ADC1Handle.DMA_Config.DMA_Channel = DMA_CHANNEL_0;
	DMA2_ADC1Handle.DMA_Config.DMA_Direction = DMA_DIR_PERIPH_TO_MEM;							//Transfers from ADC1 to local buffer
	DMA2_ADC1Handle.DMA_Config.DMA_Priority = DMA_PRIORITY_HIGH;
	DMA2_ADC1Handle.DMA_Config.DMA_MemDataSize = DMA_DATA_SIZE_WORD;							//Resolution is set to 12 bits, so Halfword can work, but since later it'll be be processed to a float it was set to the same size
	DMA2_ADC1Handle.DMA_Config.DMA_PeriphDataSize = DMA_DATA_SIZE_WORD;
	DMA2_ADC1Handle.DMA_Config.DMA_MemInc = ENABLE;											//Memory address pointer incremented after each data transfer (increment is done according to DMA_MemDataSize)
	DMA2_ADC1Handle.DMA_Config.DMA_PeriphInc = DISABLE;										//It's disabled, since we're reading just one peripheral
	DMA2_ADC1Handle.DMA_Config.DMA_FIFOMode = DMA_FIFO_MODE_DISABLED;
	DMA2_ADC1Handle.DMA_Config.DMA_FIFOThreshold = 0;
	DMA2_ADC1Handle.DMA_Config.DMA_Mode = DMA_MODE_CIRCULAR;									//We want to transfer continuously through the same peripheral to the same buffer
	DMA2_ADC1Handle.BufferSize = 4; 															//Same number of sensors added

	DMA_Init(&DMA2_ADC1Handle);
	DMA_SetAddresses(&DMA2_ADC1Handle,(void*)&ADC_1.pADCx->DR, pDest);
	DMA_StartTransfer(&DMA2_ADC1Handle);
}

void SamplingRateTIMInit(float sampling_rate)
{

	TIM_2.pTIMx = TIM2;
	TIM_2.TIM_Config.TIM_Frequency = sampling_rate;
	TIM_2.TIM_Config.TIM_CLKDivision = TIM_CKD_DIV1;
	TIM_2.TIM_Config.TIM_AutoReloadPreload = TIM_ARPE_ENABLE;
	TIM_2.TIM_Config.TIM_CNTMode = TIM_UPCOUNT_MODE;
	TIM_2.TIM_Config.TIM_IntEnable = TIM_IT_ENABLE;
	TIM_2.TIM_Config.TIM_MasterModeSel = TIM_MMS_UPDATE; 							//Master Mode established as update to trigger ADC1 conversion
	TIM_Init(&TIM_2);

	TIM_IRQInterruptConfig(IRQ_NO_TIM2, ENABLE);
	TIM_IRQPriorityConfig(IRQ_NO_TIM2, 1);

	TIM_Start(&TIM_2);
}

void PWM_TIMInits(float carrier_frequency)
{

	/*****************Master Timer initialization*****************/
	TIM_PClkC(TIM4, ENABLE);

	TIM_4.pTIMx = TIM4;
	TIM_4.TIM_Config.TIM_Frequency = carrier_frequency*2;
	TIM_4.TIM_Config.TIM_CLKDivision = TIM_CKD_DIV1;
	TIM_4.TIM_Config.TIM_AutoReloadPreload = TIM_ARPE_ENABLE;
	TIM_4.TIM_Config.TIM_CAModeSel = TIM_CMS_CA_1;
	TIM_4.TIM_Config.TIM_IntEnable = TIM_IT_DISABLE;
	TIM_4.TIM_Config.TIM_MasterModeSel = TIM_MMS_RESET;

	TIM_Init(&TIM_4);

	TIM4_PWM_Channel_1.PWM_Channel = PWM_CHANNEL_1;
	TIM4_PWM_Channel_1.PWM_OCMode = TIM_PWM_MODE2;
	TIM4_PWM_Channel_1.PWM_OCPolarity = PWM_OC_POLARITY_HIGH;
	TIM4_PWM_Channel_1.PWM_OCPolarity  = PWM_OC_PRELOAD_DISABLED;

	TIM_PWM_Channel_Init(&TIM_4, &TIM4_PWM_Channel_1);

	TIM4_PWM_Channel_2.PWM_Channel = PWM_CHANNEL_2;
	TIM4_PWM_Channel_2.PWM_OCMode = TIM_PWM_MODE2;
	TIM4_PWM_Channel_2.PWM_OCPolarity = PWM_OC_POLARITY_HIGH;
	TIM4_PWM_Channel_2.PWM_OCPolarity  = PWM_OC_PRELOAD_DISABLED;

	TIM_PWM_Channel_Init(&TIM_4, &TIM4_PWM_Channel_2);

	TIM_Start(&TIM_4);  //Starting timer just for minimal tests

}

/*ALL PARAMETERS AND DESIGN OF THIS FUNCTION ARE GIVEN FOR A 60 HZ SIGNAL***/
float NINETYDegreePhaseShift(float *pCos_Buffer, float cos_wave, __vo uint8_t *pBuffer_Counter, __vo uint8_t *pBuffer_Ready_Flag)
{
	float temp_sin = 0;

	//temp_sin stores a 90-degree phase shift signal
	//This buffer stores a quarter a of period of a 60 Hz sine wave
	if(*pBuffer_Ready_Flag == 1) temp_sin =  pCos_Buffer[*pBuffer_Counter];

	//Updates current buffer after saving the temp_sine wave

	pCos_Buffer[*pBuffer_Counter] = cos_wave;

	//Counter updates
	(*pBuffer_Counter)++;

	//Once the buffer is completely filled counter is reset
	if(*pBuffer_Counter >= 40)									//This condition is subject to sampling rate being 9.6 kHz and grid-load fundamental frequency are 60 Hz
	{															//Buffer may be larger but to store and create a ring-buffer we only take account of the first quarter of a period
		*pBuffer_Ready_Flag = 1;
		*pBuffer_Counter = 0;
	}
	return temp_sin;
}

/*Delays a signal a fixed o variable sample periods**/
float SignalDelay(float *pSignal_Buffer, float signal, __vo uint8_t *pBuffer_Counter, __vo uint8_t *pBuffer_Ready_Flag, uint8_t samples_to_delay)
{
	float temp_delayed_signal = 0;

	if(*pBuffer_Ready_Flag == 1) temp_delayed_signal =  pSignal_Buffer[*pBuffer_Counter];


	pSignal_Buffer[*pBuffer_Counter] = signal;

	//Counter updates
	(*pBuffer_Counter)++;

	//Once the buffer is completely filled counter is reset
	if(*pBuffer_Counter >= samples_to_delay)									//This condition is subject to sampling rate being 9.6 kHz
	{
		*pBuffer_Ready_Flag = 1;
		*pBuffer_Counter = 0;
	}
	return temp_delayed_signal;
}



/*This function computes partially the Park Transformation of two-phase orthogonal components its output returns the quadrature axis component */
float DTransform(float cosine_wt, float sine_wt, float alpha, float beta)
{
	return (alpha*cosine_wt + beta*sine_wt);
}

/*This function computes partially the Park Transformation of two-phase orthogonal components its output returns the direct axis component */
float QTransform(float cosine_wt, float sine_wt, float alpha, float beta)
{
	return (-alpha*sine_wt + beta*cosine_wt);
}

/*Parametres highly tied to sampling rate being 9600*/
/*To differentiate local varaibles from global variables names in local instance for this function have been written in caps*/
void CascadeControl(float cosine_wt, float sine_wt, float V_CD, float I_Q, float I_INV, __vo float *pe1_z_0, __vo float *pe1_z_1, __vo float *pe2_z_0, __vo float *pe2_z_1, __vo float *py1_z_0, __vo float *py1_z_1, __vo float *py2_z_0, __vo float *py2_z_1, __vo uint16_t *u_pos, __vo uint16_t *u_neg)
{
	float u_pos_temp = 0;
	float u_neg_temp = 0;

	(*pe1_z_0) = 36 - V_CD;														//Evaluates error among reference and DC sensed value on DC bus of the inverter

	(*py1_z_0) = (*py1_z_1) + 0.167037*(*pe1_z_0) - 0.167028*(*pe1_z_1);		//External discrete PI control loop

	(*pe1_z_1) = (*pe1_z_0);													//Updating last error as the most recent one
	(*py1_z_1) = (*py1_z_0);													//Updating last output PI control value as the most recent one

	(*pe2_z_0) = (*py1_z_0)*cosine_wt + (7.8 + I_Q)*sine_wt - I_INV; 			//Calculates the new error with a reference given by external control loop output, quadrature current and inverter sense current

	(*py2_z_0) = (*py2_z_1) - 0.03203*(*pe2_z_0) + 0.03087*(*pe2_z_1); 			//Internal discrete PI control loop

	if((*py2_z_0)> (0.99)) (*py2_z_0) = 0.99;
	if((*py2_z_0)< (-0.99)) (*py2_z_0) = -0.99;									//Setting boundaries for control signal

	(*pe2_z_1) = (*pe2_z_0);													//Updating last error as the most recent one
	(*py2_z_1) = (*py2_z_0);													//Updating last output PI control value as the most recent one

	u_pos_temp = (((*py2_z_0 )*(0.5)) + 0.5)*(TIM4->ARR);
	u_neg_temp = (((*py2_z_0 )*(-0.5)) + 0.5)*(TIM4->ARR);

	(*u_pos) = (uint16_t)u_pos_temp;				 							//Updates positive control signal in relation to PWM resolution
	(*u_neg) = (uint16_t)u_neg_temp;											//Updates negative control signal in relation to PWM resolution

}

void OpenLoop(float cosine_wt, __vo uint16_t *u_pos, __vo uint16_t *u_neg)
{
	float u_pos_temp = 0;
	float u_neg_temp = 0;

	u_pos_temp = ((cosine_wt*0.5) + 0.5)*(TIM4->ARR);
	u_neg_temp = ((-cosine_wt*0.5) + 0.5)*(TIM4->ARR);

	(*u_pos) = (uint16_t)u_pos_temp;				 							//Updates positive control signal in relation to PWM resolution
	(*u_neg) = (uint16_t)u_neg_temp;											//Updates negative control signal in relation to PWM resolution
}

/*This functions resets CascadeControl() input-output parameters*/
void ResetPIControllers(__vo float *pe1_z_0, __vo float *pe1_z_1, __vo float *pe2_z_0, __vo float *pe2_z_1, __vo float *py1_z_0, __vo float *py1_z_1, __vo float *py2_z_0, __vo float *py2_z_1)
{
	(*pe1_z_0) = 0;
	(*pe1_z_1) = 0;

	(*py1_z_0) = 0;
	(*py1_z_1) = 0;

	(*pe2_z_0) = 0;
	(*pe2_z_1) = 0;

	(*py2_z_0) = 0;
	(*py2_z_1) = 0;
}

void PWM_Enable(void)
{
	TIM_PWM_Enable(&TIM_4, &TIM4_PWM_Channel_1);
	TIM_PWM_Enable(&TIM_4, &TIM4_PWM_Channel_2);
}

void PWM_Disable(void)
{
	TIM_PWM_Disable(&TIM_4, &TIM4_PWM_Channel_1);
	TIM_PWM_Disable(&TIM_4, &TIM4_PWM_Channel_2);

	GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_7, RESET);
	GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_6, RESET);
}

void PWM_dutyCycle_control(uint16_t u_pos ,uint16_t u_neg)
{
	TIM_PWM_DutyCycle(&TIM_4, &TIM4_PWM_Channel_1, u_pos);
	TIM_PWM_DutyCycle(&TIM_4, &TIM4_PWM_Channel_2, u_neg);
}

void TIM2_IRQHandling(void)
{
	TIM_IRQHandling(&TIM_2);
}

