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

static uint8_t PWM_ENABLE = DISABLE;

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


uint8_t OPERATION_MODE = DISABLE;
uint8_t operationMode;

/*This function resets the value of Elapsed time*/
void ResetTime(void)
{
	ElapsedTime = START_TIME;
}

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

void ControlInit(void)
{
	Utility_GPIOInits();
	PWM_GPIOInits();
	Sensors_Init((void*)raw_sensor_value);
	SamplingRateTIMInit(SAMPLING_FREQUENCY);
	PWM_TIMInits(PWM_FREQUENCY);
}

void Control_Start(void)
{
	TIM_Start(&TIM_2);
	TIM_Start(&TIM_4);  //Starting timer just for minimal tests
	DMA_StartTransfer(&DMA2_ADC1Handle);
}

void TIM2_IRQHandling(void)
{
	TIM_IRQHandling(&TIM_2);
}

uint8_t Control_ReadSensors(float* values)
{
	/*TO DO: Read and characterize sensors */

	v_g = 	(raw_sensor_value[0]/4095.0f - 0.5f)*2.0f;
	i_inv = (raw_sensor_value[1]/4095.0f - 0.5f)*2.0f;
	i_L = 	(raw_sensor_value[2]/4095.0f - 0.5f)*2.0f;
	v_cd = 	(raw_sensor_value[3]/4095.0f - 0.5f)*2.0f;
	ElapsedTime = ElapsedTime + (1.0f/9600.0f);

	if (valid_send == FLAG_SET) {
		*values[0] = v_g;
		*values[1] = i_L ;
		*values[2] = i_inv;
		*values[3] = v_cd;
		*values[4] = ElapsedTime;
		valid_send = FLAG_RESET;
	}else {
		valid_send = FLAG_SET;
	}

	return valid_send;
}

void Control_DutyCycle(void)
{
	/*In case there is a high presence of noise, signals will be filtered*/

	cosine = v_g;		//This is just an example to show its functionality

	sine = NINETYDegreePhaseShift(cos_buffer, cosine, &Buffer_Counter_Cos, &Buffer_Ready_Flag_Cos);

	i_L90 = NINETYDegreePhaseShift(i_L_buffer, i_L, &Buffer_Counter_iL, &Buffer_Ready_Flag_iL);
	
	i_Q = QTransform(cosine, sine, i_L, i_L90);

	if(OPERATION_MODE == DISABLE)
	{
		OpenLoop(v_g, &u_control_pos, &u_control_neg);

	} else
	{
		CascadeControl(cosine, sine, v_cd, i_Q, i_inv, &e1_z_0, &e1_z_1, &e2_z_0, &e2_z_1, &y1_z_0, &y1_z_1, &y2_z_0, &y2_z_1, &u_control_pos, &u_control_neg);
	}

	PWM_dutyCycle_control(u_control_pos, u_control_neg);
}

uint8_t Control_Mode(void)
{
	//TO-DO: Change to use Command value
	GPIO_IRQHandling(GPIO_PIN_NO_14);
	/*Both pins are read*/
	PWM_ENABLE = GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_NO_14);
	OPERATION_MODE = GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_NO_15);				//This lecture autmatically changes Operation Mode as: Open Loop when Operation Mode = 0, Closed Loop when 1


	/*When Operation Mode is zero it resets PI controllers from CascadeControl(), to assure safe and smooth transition to Closed Loop Mode Operation*/
	if( OPERATION_MODE == DISABLE)
	{
		ResetPIControllers(&e1_z_0, &e1_z_1, &e2_z_0, &e2_z_1, &y1_z_0, &y1_z_1, &y2_z_0, &y2_z_1);
		SET_OPEN_LOOP_MODE	 //Set Loop Status Flag to Open
	} else
	{
		SET_CLOSED_LOOP_MODE //Set Loop Status Flag to Closed
	}

	if( PWM_ENABLE == DISABLE )
	{
		PWM_Disable();

		PWM_DISABLE_FLAG //Set PWM Status Flag to Disabled

	} else if( PWM_ENABLE == ENABLE )
	{
		PWM_Enable();
		PWM_ENABLE_FLAG	 //Set PWM Status Flag to Enabled

	}
	return operationMode;
}

void ShiftSensorsValue(void)
{
	for(uint64_t x = BUFFER_SIZE-1; x>0; x--)
	{
		//TO-DO: shift processed values
		;
	}
}
