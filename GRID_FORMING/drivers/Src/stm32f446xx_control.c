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

int valid_send = FLAG_SET;		//Flag to indicate when data packet is ready to be sent

/*Buffers to store quarter of a period of cos and i_L*/
/*For every shifted signal it is needed a buffer*/
float cos_buffer[40] = {RESET};
float i_L_buffer[40] = {RESET};


/*Control global variables*/
__vo float e1_z_0 = RESET;
__vo float e1_z_1 = RESET;

__vo float e2_z_0 = RESET;
__vo float e2_z_1 = RESET;

__vo float y1_z_0 = RESET;
__vo float y1_z_1 = RESET;

__vo float y2_z_0 = RESET;
__vo float y2_z_1 = RESET;

/*Flags and counters used for 90-degree shiftimg*/
__vo static uint8_t Buffer_Counter_Cos = RESET;
__vo static uint8_t Buffer_Ready_Flag_Cos = RESET;

__vo static uint8_t Buffer_Counter_iL = RESET;
__vo static uint8_t Buffer_Ready_Flag_iL = RESET;

uint64_t BUFFER_SIZE = BUFFER_LENGTH_9;

/*characterized sensor outputs*/
float v_cd;
float v_g;
float i_L;
float i_L90;
float i_inv;

uint32_t raw_sensor_value[4];

float ElapsedTime = START_TIME;	//Elapsed time variable

/*sine wave for DQ and power-factor correction*/
float cosine;
float sine;

float i_Q;

/*modulator signal*/
__vo uint16_t u_control_pos;
__vo uint16_t u_control_neg;

uint8_t operationMode;

/*********************************************************************************************************************************************************************
 * @fn		               ResetTime
 *
 * @brief                  Resets the global elapsed time counter to the predefined start value.
 *
 * @param  	      		   None
 *
 * @return                 None
 *
 * @Requirements           TO-DO
 *********************************************************************************************************************************************************************/
void ResetTime(void)
{
	ElapsedTime = START_TIME;
}

/*********************************************************************************************************************************************************************
 * @fn 			           Utility_GPIOInits
 *
 * @brief                  Initializes GPIO pins PB14 and PB15 as input with interrupt capability (EXTI15_10) to control PWM on/off and change operation mode.
 *
 * @param	               None
 *
 * @return                 None
 *
 * @Requirements           TO-DO
 *********************************************************************************************************************************************************************/
void Utility_GPIOInits(void)
{
	GPIO_PClkC(GPIOB, ENABLE);
	PWM_EN.pGPIOx = GPIOB;
	PWM_EN.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	PWM_EN.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RT;
	PWM_EN.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&PWM_EN);

	LOOP_SEL.pGPIOx = GPIOB;
	LOOP_SEL.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	LOOP_SEL.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RT;
	LOOP_SEL.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_Init(&LOOP_SEL);

	GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10,ENABLE);
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, 0);
}

/*********************************************************************************************************************************************************************
 * @fn         			   PWM_GPIOInits
 *
 * @brief                  Configures GPIO pins PB6 and PB7 for PWM output by setting them to alternate function mode with high-speed output.
 *
 * @param                  None
 *
 * @return                 None
 *
 * @Requirements           TO-DO
 *********************************************************************************************************************************************************************/
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
/*********************************************************************************************************************************************************************
 * @fn        			   Sensors_Init
 *
 * @brief                  Initializes GPIO pins for analog sensor inputs, configures ADC1 for multi-channel conversion triggered by TIM2, and sets up DMA for 
 * 						   continuous data transfer to memory.
 * 
 * @param             	   pDest – Pointer to the destination buffer where ADC conversion results will be stored.
 *
 * @return                 None
 * 
 * @note				   Add as much sensors you want to read (look up pinout for compatibility)
 *
 * @Requirements           TO-DO
 * 
 *********************************************************************************************************************************************************************/
void Sensors_Init(void *pDest) //For this application only 4 sensors will be initialized
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
}


/*********************************************************************************************************************************************************************
 * @fn                     SamplingRateTIMInit
 *
 * @brief                  Configures TIM2 to generate periodic update events at the specified sampling rate. These events act as triggers for ADC conversions.
 * 
 * @param                  sampling_rate – Desired sampling frequency in Hz (float).
 *
 * @return                 None
 * 
 * @note                   TIM2 is set as the master timer to trigger ADC1 conversions via update events. Interrupts are enabled for TIM2.
 *
 * @Requirements           TO-DO
 * 
 *********************************************************************************************************************************************************************/

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
}

/*********************************************************************************************************************************************************************
 * @fn                     PWM_TIMInits
 *
 * @brief                  Configures TIM4 as the master timer for PWM generation on channels 1 and 2, using center-aligned mode and PWM Mode 2.
 * 
 * @param                  carrier_frequency – Desired PWM carrier frequency in Hz (float).
 *
 * @return                 None
 * 
 * @note                   TIM4 is initialized with center-aligned mode (CA_1) and two PWM channels are configured with high polarity and Mode 2.
 *
 * @Requirements           TO-DO
 * 
 *********************************************************************************************************************************************************************/
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
}

/*********************************************************************************************************************************************************************
 * @fn                     NINETYDegreePhaseShift
 *
 * @brief                  Generates a 90° phase-shifted signal from a cosine wave by using a circular buffer that stores one-quarter of a 60 Hz period.
 * 
 * @param                  pCos_Buffer – Pointer to the buffer storing cosine samples.
 * @param                  cos_wave – Current cosine sample to be added to the buffer.
 * @param                  pBuffer_Counter – Pointer to the buffer index counter (updated each call).
 * @param                  pBuffer_Ready_Flag – Pointer to a flag indicating when the buffer is fully populated.
 *
 * @return                 temp_sin – A float representing the sine value (90° phase shift) derived from the buffer.
 * 
 * @note                   - Designed for a 60 Hz signal with a sampling rate of 9.6 kHz.
 *                         - Buffer stores 40 samples (¼ of a 60 Hz period).
 *                         - Implements a ring-buffer mechanism for continuous phase-shift generation.
 *
 * @Requirements           TO-DO
 * 
 *********************************************************************************************************************************************************************/
float NINETYDegreePhaseShift(float *pCos_Buffer, float cos_wave, __vo uint8_t *pBuffer_Counter, __vo uint8_t *pBuffer_Ready_Flag)
{
	float temp_sin = RESET;

	//temp_sin stores a 90-degree phase shift signal
	//This buffer stores a quarter a of period of a 60 Hz sine wave
	if(*pBuffer_Ready_Flag == FLAG_SET) temp_sin =  pCos_Buffer[*pBuffer_Counter];

	//Updates current buffer after saving the temp_sine wave

	pCos_Buffer[*pBuffer_Counter] = cos_wave;

	//Counter updates
	(*pBuffer_Counter)++;

	//Once the buffer is completely filled counter is reset
	if(*pBuffer_Counter >= 40)									//This condition is subject to sampling rate being 9.6 kHz and grid-load fundamental frequency are 60 Hz
	{															//Buffer may be larger but to store and create a ring-buffer we only take account of the first quarter of a period
		*pBuffer_Ready_Flag = FLAG_SET;
		*pBuffer_Counter = RESET;
	}
	return temp_sin;
}

/*********************************************************************************************************************************************************************
 * @fn                     DTransform
 *
 * @brief                  Computes the quadrature-axis component of the Park Transformation using two-phase orthogonal components.
 * 
 * @param                  cosine_wt – Cosine of the electrical angle (ωt).
 * @param                  sine_wt   – Sine of the electrical angle (ωt).
 * @param                  alpha     – α-axis component.
 * @param                  beta      – β-axis component.
 *
 * @return                 Quadrature-axis component (float).
 * 
 * @note                   This function performs a partial Park Transformation, returning only the q-axis value.
 *
 * @Requirements           TO-DO
 * 
 *********************************************************************************************************************************************************************/
float DTransform(float cosine_wt, float sine_wt, float alpha, float beta)
{
	return (alpha*cosine_wt + beta*sine_wt);
}

/*********************************************************************************************************************************************************************
 * @fn                     QTransform
 *
 * @brief                  Computes the direct-axis component of the Park Transformation using two-phase orthogonal components.
 * 
 * @param                  cosine_wt – Cosine of the electrical angle (ωt).
 * @param                  sine_wt   – Sine of the electrical angle (ωt).
 * @param                  alpha     – α-axis component.
 * @param                  beta      – β-axis component.
 *
 * @return                 Direct-axis component (float).
 * 
 * @note                   This function performs a partial Park Transformation, returning only the d-axis value.
 *
 * @Requirements           TO-DO
 * 
 *********************************************************************************************************************************************************************/
float QTransform(float cosine_wt, float sine_wt, float alpha, float beta)
{
	return (-alpha*sine_wt + beta*cosine_wt);
}

/*********************************************************************************************************************************************************************
 * @fn                     CascadeControl
 *
 * @brief                  Implements a cascaded control strategy with two discrete PI loops: 
 *                         - Outer loop regulates DC bus voltage.
 *                         - Inner loop regulates inverter current using Park Transformation components.
 *                         Updates PWM duty cycles for positive and negative signals based on control output.
 * 
 * @param                  cosine_wt   – Cosine of the electrical angle (ωt).
 * @param                  sine_wt     – Sine of the electrical angle (ωt).
 * @param                  V_CD        – Measured DC bus voltage.
 * @param                  I_Q         – Quadrature current component.
 * @param                  I_INV       – Measured inverter current.
 * @param                  pe1_z_0     – Pointer to current error of outer PI loop.
 * @param                  pe1_z_1     – Pointer to previous error of outer PI loop.
 * @param                  pe2_z_0     – Pointer to current error of inner PI loop.
 * @param                  pe2_z_1     – Pointer to previous error of inner PI loop.
 * @param                  py1_z_0     – Pointer to current output of outer PI loop.
 * @param                  py1_z_1     – Pointer to previous output of outer PI loop.
 * @param                  py2_z_0     – Pointer to current output of inner PI loop.
 * @param                  py2_z_1     – Pointer to previous output of inner PI loop.
 * @param                  u_pos       – Pointer to positive PWM duty cycle (uint16_t).
 * @param                  u_neg       – Pointer to negative PWM duty cycle (uint16_t).
 *
 * @return                 None
 * 
 * @note                   - Sampling rate is tightly coupled to control parameters (designed for ~9600 Hz).
 *                         - Local variables are written in uppercase to differentiate from global variables.
 *                         - Saturation limits applied to inner loop output: [-0.99, 0.99].
 *                         - PWM duty cycles computed relative to TIM4->ARR register.
 *
 * @Requirements           TO-DO
 * 
 *********************************************************************************************************************************************************************/

void CascadeControl(float cosine_wt, float sine_wt, float V_CD, float I_Q, float I_INV, __vo float *pe1_z_0, __vo float *pe1_z_1, __vo float *pe2_z_0, __vo float *pe2_z_1, __vo float *py1_z_0, __vo float *py1_z_1, __vo float *py2_z_0, __vo float *py2_z_1, __vo uint16_t *u_pos, __vo uint16_t *u_neg)
{
	float u_pos_temp = RESET;
	float u_neg_temp = RESET;

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

/*********************************************************************************************************************************************************************
 * @fn                     OpenLoop
 *
 * @brief                  Generates open-loop PWM duty cycles based on a cosine reference signal. Updates positive and negative control signals relative to TIM4 ARR.
 * 
 * @param                  cosine_wt – Cosine of the electrical angle (ωt).
 * @param                  u_pos     – Pointer to positive PWM duty cycle (uint16_t).
 * @param                  u_neg     – Pointer to negative PWM duty cycle (uint16_t).
 *
 * @return                 None
 * 
 * @note                   - Computes duty cycles using a normalized cosine signal.
 *                         - Positive and negative signals are scaled to the PWM resolution defined by TIM4->ARR.
 *
 * @Requirements           TO-DO
 * 
 *********************************************************************************************************************************************************************/
void OpenLoop(float cosine_wt, __vo uint16_t *u_pos, __vo uint16_t *u_neg)
{
	float u_pos_temp = RESET;
	float u_neg_temp = RESET;

	u_pos_temp = ((cosine_wt*0.5) + 0.5)*(TIM4->ARR);
	u_neg_temp = ((-cosine_wt*0.5) + 0.5)*(TIM4->ARR);

	(*u_pos) = (uint16_t)u_pos_temp;				 							//Updates positive control signal in relation to PWM resolution
	(*u_neg) = (uint16_t)u_neg_temp;											//Updates negative control signal in relation to PWM resolution
}

/*********************************************************************************************************************************************************************
 * @fn                     ResetPIControllers
 *
 * @brief                  Resets all input-output parameters of the cascaded PI controllers used in CascadeControl() to zero.
 * 
 * @param                  pe1_z_0 – Pointer to current error of outer PI loop.
 * @param                  pe1_z_1 – Pointer to previous error of outer PI loop.
 * @param                  pe2_z_0 – Pointer to current error of inner PI loop.
 * @param                  pe2_z_1 – Pointer to previous error of inner PI loop.
 * @param                  py1_z_0 – Pointer to current output of outer PI loop.
 * @param                  py1_z_1 – Pointer to previous output of outer PI loop.
 * @param                  py2_z_0 – Pointer to current output of inner PI loop.
 * @param                  py2_z_1 – Pointer to previous output of inner PI loop.
 *
 * @return                 None
 * 
 * @note                   This function is typically called during system initialization or when resetting control states after a mode change.
 *
 * @Requirements           TO-DO
 * 
 *********************************************************************************************************************************************************************/
void ResetPIControllers(__vo float *pe1_z_0, __vo float *pe1_z_1, __vo float *pe2_z_0, __vo float *pe2_z_1, __vo float *py1_z_0, __vo float *py1_z_1, __vo float *py2_z_0, __vo float *py2_z_1)
{
	(*pe1_z_0) = RESET;
	(*pe1_z_1) = RESET;

	(*py1_z_0) = RESET;
	(*py1_z_1) = RESET;

	(*pe2_z_0) = RESET;
	(*pe2_z_1) = RESET;

	(*py2_z_0) = RESET;
	(*py2_z_1) = RESET;
}

/*********************************************************************************************************************************************************************
 * @fn                     PWM_Enable
 *
 * @brief                  Enables PWM output on TIM4 channels 1 and 2.
 * 
 * @param                  None
 *
 * @return                 None
 * 
 * @note                   Uses TIM_PWM_Enable() to activate PWM signals on both configured channels.
 *
 * @Requirements           TO-DO
 * 
 *********************************************************************************************************************************************************************/
void PWM_Enable(void)
{
	TIM_PWM_Enable(&TIM_4, &TIM4_PWM_Channel_1);
	TIM_PWM_Enable(&TIM_4, &TIM4_PWM_Channel_2);
}

/*********************************************************************************************************************************************************************
 * @fn                     PWM_Disable
 *
 * @brief                  Disables PWM output on TIM4 channels 1 and 2 and resets GPIO pins PB6 and PB7 to LOW state.
 * 
 * @param                  None
 *
 * @return                 None
 * 
 * @note                   - Calls TIM_PWM_Disable() for both channels.
 *                         - Ensures GPIO pins PB6 and PB7 are set to RESET to avoid floating outputs.
 *
 * @Requirements           TO-DO
 * 
 *********************************************************************************************************************************************************************/
void PWM_Disable(void)
{
	TIM_PWM_Disable(&TIM_4, &TIM4_PWM_Channel_1);
	TIM_PWM_Disable(&TIM_4, &TIM4_PWM_Channel_2);

	GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_7, RESET);
	GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_NO_6, RESET);
}

/*********************************************************************************************************************************************************************
 * @fn                     PWM_dutyCycle_control
 *
 * @brief                  Updates the PWM duty cycle for TIM4 channels 1 and 2 based on the provided values.
 * 
 * @param                  u_pos – Duty cycle value for PWM channel 1 (uint16_t).
 * @param                  u_neg – Duty cycle value for PWM channel 2 (uint16_t).
 *
 * @return                 None
 * 
 * @note                   - Uses TIM_PWM_DutyCycle() to apply new duty cycle values.
 *                         - Duty cycle values should be within the range defined by TIM4->ARR.
 *
 * @Requirements           TO-DO
 * 
 *********************************************************************************************************************************************************************/
void PWM_dutyCycle_control(uint16_t u_pos ,uint16_t u_neg)
{
	TIM_PWM_DutyCycle(&TIM_4, &TIM4_PWM_Channel_1, u_pos);
	TIM_PWM_DutyCycle(&TIM_4, &TIM4_PWM_Channel_2, u_neg);
}

/*********************************************************************************************************************************************************************
 * @fn                     TIM2_IRQHandling
 *
 * @brief                  Handles the interrupt request for TIM2 by calling the corresponding IRQ handler function.
 * 
 * @param                  None
 *
 * @return                 None
 * 
 * @note                   Delegates interrupt handling to TIM_IRQHandling() for TIM2.
 *
 * @Requirements           TO-DO
 * 
 *********************************************************************************************************************************************************************/
void TIM2_IRQHandling(void)
{
	TIM_IRQHandling(&TIM_2);
}

/*********************************************************************************************************************************************************************
 * @fn                     ControlInit
 *
 * @brief                  Initializes all essential peripherals and configurations for the control system, including GPIO, sensors, timers, and PWM.
 * 
 * @param                  None
 *
 * @return                 None
 * 
 * @note                   - Calls initialization routines for GPIO inputs, PWM outputs, sensor ADC with DMA, sampling timer, and PWM timer.
 *                         - Uses global constants `SAMPLING_FREQUENCY` and `PWM_FREQUENCY` for timing configuration.
 *                         - Passes `raw_sensor_value` buffer to Sensors_Init() for DMA data storage.
 *
 * @Requirements           TO-DO
 * 
 *********************************************************************************************************************************************************************/
void ControlInit(void)
{
	Utility_GPIOInits();
	PWM_GPIOInits();
	Sensors_Init((void*)raw_sensor_value);
	SamplingRateTIMInit(SAMPLING_FREQUENCY);
	PWM_TIMInits(PWM_FREQUENCY);
}

/*********************************************************************************************************************************************************************
 * @fn                     Control_Start
 *
 * @brief                  Starts the control system by enabling timers and initiating DMA transfer for sensor data acquisition.
 * 
 * @param                  None
 *
 * @return                 None
 * 
 * @note                   - TIM2 is started to generate sampling triggers for ADC conversions.
 *                         - TIM4 is started to enable PWM generation (initial tests).
 *                         - DMA transfer begins to continuously move ADC data to the destination buffer.
 *
 * @Requirements           TO-DO
 * 
 *********************************************************************************************************************************************************************/
void Control_Start(void)
{
	TIM_Start(&TIM_2);
	TIM_Start(&TIM_4);  //Starting timer just for minimal tests
	DMA_StartTransfer(&DMA2_ADC1Handle);
}

/*********************************************************************************************************************************************************************
 * @fn                     Control_Stop
 *
 * @brief                  Stops the control system by disabling timers, halting DMA transfers, turning off PWM outputs, and clearing internal buffers.
 * 
 * @param                  None
 *
 * @return                 None
 * 
 * @note                   - TIM2 and TIM4 are stopped to halt sampling and PWM generation.
 *                         - DMA transfer for ADC data acquisition is disabled.
 *                         - PWM outputs are turned off using PWM_Disable().
 *                         - Clears `cos_buffer` and `i_L_buffer` arrays (40 elements each) to reset stored data.
 *
 * @Requirements           TO-DO
 * 
 *********************************************************************************************************************************************************************/
void Control_Stop(void)
{
	TIM_Stop(&TIM_2);
	TIM_Stop(&TIM_4);  //Starting timer just for minimal tests
	DMA_StopTransfer(&DMA2_ADC1Handle);
	PWM_Disable();
	for (int i=0; i<40; i++) 
	{
		cos_buffer[i] = 0;
		i_L_buffer[i] = 0;
	}
}

/*********************************************************************************************************************************************************************
 * @fn                     Control_ReadSensors
 *
 * @brief                  Reads raw ADC sensor values, normalizes them to a [-1, 1] range, updates global variables, and optionally stores them in the provided buffer.
 * 
 * @param                  values – Pointer to a float array where processed sensor values will be stored (size ≥ 5).
 *
 * @return                 uint8_t – Returns `valid_send` flag indicating whether new data was stored (FLAG_SET or FLAG_RESET).
 * 
 * @note                   - Converts raw ADC readings (12-bit resolution) to normalized values using the formula: ((raw / 4095) - 0.5) * 2.
 *                         - Updates global variables: `v_g`, `i_inv`, `i_L`, `v_cd`, and increments `ElapsedTime` based on sampling rate (9600 Hz).
 *                         - Stores values in the array only when `valid_send == FLAG_SET`; otherwise toggles the flag.
 *                         - The output array contains: [v_g, i_L, i_inv, v_cd, ElapsedTime].
 *
 * @Requirements           TO-DO
 * 
 *********************************************************************************************************************************************************************/
uint8_t Control_ReadSensors(float* values)
{
	/*TO DO: Read and characterize sensors */

	v_g = 	(raw_sensor_value[0]/ADC_RESOLUTION - ADC_OFFSET_VOLTAGE)*ADC_VOLTAGE_REF;
	i_inv = (raw_sensor_value[1]/ADC_RESOLUTION - ADC_OFFSET_VOLTAGE)*ADC_VOLTAGE_REF;
	i_L = 	(raw_sensor_value[2]/ADC_RESOLUTION - ADC_OFFSET_VOLTAGE)*ADC_VOLTAGE_REF;
	v_cd = 	(raw_sensor_value[3]/ADC_RESOLUTION - ADC_OFFSET_VOLTAGE)*ADC_VOLTAGE_REF;
	ElapsedTime = ElapsedTime + SAMPLING_PERIOD;

	if (valid_send == FLAG_SET) {
		values[0] = v_g;
		values[1] = i_L ;
		values[2] = i_inv;
		values[3] = v_cd;
		values[4] = ElapsedTime;
		valid_send = FLAG_RESET;
	}else {
		valid_send = FLAG_SET;
	}
	return valid_send;
}

/*********************************************************************************************************************************************************************
 * @fn                     Control_DutyCycle
 *
 * @brief                  Computes PWM duty cycles based on the selected operation mode. Applies signal processing, phase shifting, and control algorithms.
 * 
 * @param                  None
 *
 * @return                 None
 * 
 * @note                   - Filters signals if noise is present (future implementation).
 *                         - Calculates `cosine` from `v_g` and computes `sine` using a 90° phase shift via NINETYDegreePhaseShift().
 *                         - Computes quadrature current component `i_Q` using QTransform().
 *                         - If `operationMode` bit 0 is DISABLE → Executes OpenLoop() for PWM control.
 *                         - Else → Executes CascadeControl() for closed-loop control using PI regulators.
 *                         - Updates PWM duty cycles through PWM_dutyCycle_control() using computed positive and negative control signals.
 *
 * @Requirements           TO-DO
 * 
 *********************************************************************************************************************************************************************/
void Control_DutyCycle(void)
{
	/*In case there is a high presence of noise, signals will be filtered*/

	cosine = v_g;		//This is just an example to show its functionality

	sine = NINETYDegreePhaseShift(cos_buffer, cosine, &Buffer_Counter_Cos, &Buffer_Ready_Flag_Cos);

	i_L90 = NINETYDegreePhaseShift(i_L_buffer, i_L, &Buffer_Counter_iL, &Buffer_Ready_Flag_iL);
	
	i_Q = QTransform(cosine, sine, i_L, i_L90);

	if((operationMode & 0b1) == DISABLE)
	{
		OpenLoop(v_g, &u_control_pos, &u_control_neg);

	} else
	{
		CascadeControl(cosine, sine, v_cd, i_Q, i_inv, &e1_z_0, &e1_z_1, &e2_z_0, &e2_z_1, &y1_z_0, &y1_z_1, &y2_z_0, &y2_z_1, &u_control_pos, &u_control_neg);
	}

	PWM_dutyCycle_control(u_control_pos, u_control_neg);
}

/*********************************************************************************************************************************************************************
 * @fn                     Control_Mode
 *
 * @brief                  Sets the control system operation mode (Open Loop or Closed Loop) and power state (Enable or Disable). Handles safe transitions by resetting PI controllers when switching to Open Loop.
 * 
 * @param                  Power – Indicates whether the system should be powered ON (ENABLE) or OFF (DISABLE).
 * @param                  Loop  – Indicates whether the control loop should operate in Closed Loop (ENABLE) or Open Loop (DISABLE).
 *
 * @return                 uint8_t – Returns the updated `operationMode` status flag.
 * 
 * @note                   - If `Loop == DISABLE`, resets PI controllers and sets Open Loop mode.
 *                         - If `Loop == ENABLE`, sets Closed Loop mode.
 *                         - If `Power == DISABLE`, stops control system and sets PWM status to OFF.
 *                         - If `Power == ENABLE`, starts control system and sets PWM status to ON.
 *                         - Uses macros: `SET_OPEN_LOOP_MODE`, `SET_CLOSED_LOOP_MODE`, `SYSTEM_OFF_FLAG`, `SYSTEM_ON_FLAG`.
 *
 * @Requirements           TO-DO
 * 
 *********************************************************************************************************************************************************************/
uint8_t Control_Mode(uint8_t Power, uint8_t Loop)
{
	/*When Operation Mode is zero it resets PI controllers from CascadeControl(), to assure safe and smooth transition to Closed Loop Mode Operation*/
	if( Loop == DISABLE)
	{
		ResetPIControllers(&e1_z_0, &e1_z_1, &e2_z_0, &e2_z_1, &y1_z_0, &y1_z_1, &y2_z_0, &y2_z_1);
		SET_OPEN_LOOP_MODE;	 //Set Loop Status Flag to Open
	} else if ( Loop == ENABLE )
	{
		SET_CLOSED_LOOP_MODE; //Set Loop Status Flag to Closed
	}
	if( Power == DISABLE )
	{
		Control_Stop(); 
		SYSTEM_OFF_FLAG; //Set PWM Status Flag to Disabled
	} else if( Power == ENABLE )
	{
		Control_Start(); 
		SYSTEM_ON_FLAG;	 //Set PWM Status Flag to Enabled
	}
	return operationMode;
}

/*********************************************************************************************************************************************************************
 * @fn		  				- ShiftSensorsValue
 *
 * @brief					- Pending function to shift processed sensor values in a buffer for further analysis or filtering.
 *
 * @param					None
 *
 * @return					- None
 *
 * @Requirements			- TO-DO
 *********************************************************************************************************************************************************************/
void ShiftSensorsValue(void)
{
	for(uint64_t x = BUFFER_SIZE-1; x>0; x--)
	{
		//TO-DO: shift processed values
		;
	}
}