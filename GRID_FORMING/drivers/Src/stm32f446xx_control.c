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
int sent = 0;

/*Buffers to store quarter of a period of cos and i_L*/
/*For every shifted signal it is needed a buffer*/
//float cos_buffer[40] = {RESET};
float i_L_buffer[154] = {RESET};


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
float delayed_i_L;

/*
 * Variables for delayed i_L
*/

__vo static uint8_t Buffer_Counter_delayed_i_L = RESET;
__vo static uint8_t Buffer_Ready_Flag_delayed_i_L = RESET;

static uint8_t index = 0;

uint32_t raw_sensor_value[4];

float ElapsedTime = START_TIME;	//Elapsed time variable

/*sine wave for DQ and power-factor correction*/
float cosine;
float sine;

float i_Q;

float coseno[160] = {0, 0.039259815759068610041548907929609, 0.078459095727844943568562996460969, 0.11753739745783763070985372678479, 0.15643446504023086895962535436411, 0.19509032201612824808378832130984, 0.23344536385590536342604650599242, 0.27144044986507420702537274337374, 0.30901699437494739575171820433752, 0.34611705707749296223596502386499, 0.38268343236508978177923268049199, 0.41865973753742802276889278800809, 0.45399049973954674896958749741316, 0.48862124149695490560318944517348, 0.52249856471594879891995333309751, 0.55557023301960217764872140833177, 0.58778525229247313710345679282909, 0.6190939493098338575194361510512, 0.64944804833018365819441441999516, 0.67880074553294167394312808028189, 0.70710678118654752440084436210485, 0.73432250943568555534568531584227, 0.76040596560003081982870298816124, 0.78531693088074483455329755088314, 0.80901699437494734024056697307969, 0.8314696123025452356714026791451, 0.85264016435409217820051708258688, 0.87249600707279706401919838754111, 0.89100652418836778778654661437031, 0.90814317382508114029349144402659, 0.92387953251128673848313610506011, 0.93819133592248415975944908495876, 0.95105651629515353118193843329209, 0.96245523645364716713856978458352, 0.97236992039767655704451954079559, 0.98078528040323043057924223830923, 0.98768834059513777035022030759137, 0.99306845695492629300105136280763, 0.99691733373312796384624334677937, 0.99922903624072290096336246278952, 1.0, 0.99922903624072290096336246278952, 0.99691733373312796384624334677937, 0.99306845695492629300105136280763, 0.98768834059513777035022030759137, 0.98078528040323054160154470082489, 0.97236992039767666806682200331124, 0.96245523645364738918317470961483, 0.95105651629515364220424089580774, 0.93819133592248415975944908495876, 0.92387953251128673848313610506011, 0.90814317382508136233809636905789, 0.89100652418836789880884907688596, 0.87249600707279728606380331257242, 0.85264016435409228922281954510254, 0.83146961230254534669370514166076, 0.80901699437494767330747436062666, 0.78531693088074516762020493843011, 0.76040596560003126391791283822386, 0.73432250943568544432338285332662, 0.70710678118654752440084436210485, 0.67880074553294200701003546782886, 0.64944804833018376921671688251081, 0.61909394930983419058634353859816, 0.58778525229247324812575925534475, 0.55557023301960251071562879587873, 0.52249856471594935403146564567578, 0.48862124149695523867009683272045, 0.45399049973954730408109980999143, 0.41865973753742841134695140681288, 0.38268343236508989280153514300764, 0.34611705707749329530287241141195, 0.30901699437494750677402066685318, 0.27144044986507459560343136217853, 0.2334453638559055299595001997659, 0.19509032201612860890627132448572, 0.15643446504023142407113766694238, 0.1175373974578379776545489221462, 0.078459095727845512557863116853696, 0.0392598157590689500473501993838, 0.00000000000000012246467991473532071737640294584, -0.039259815759068263096853712568191, -0.078459095727844818668472726130858, -0.11753739745783729764294633923782, -0.15643446504023073018174727621954, -0.19509032201612791501688093376288, -0.23344536385590486382568542467197, -0.27144044986507431804767520588939, -0.30901699437494689615135712301708, -0.34611705707749262916905763631803, -0.38268343236508922666772036791372, -0.41865973753742774521313663171895, -0.45399049973954624936922641609272, -0.48862124149695462804743328888435, -0.52249856471594835483074348303489, -0.55557023301960195560411648330046, -0.58778525229247269301424694276648, -0.61909394930983374649713368853554, -0.6494480483301835471721119574795, -0.67880074553294145189852315525059, -0.70710678118654752440084436210485, -0.734322509435685000234173003264, -0.76040596560003059778409806312993, -0.78531693088074472353099508836749, -0.80901699437494734024056697307969, -0.83146961230254479158219282908249, -0.85264016435409184513360969503992, -0.87249600707279695299689592502546, -0.89100652418836778778654661437031, -0.90814317382508136233809636905789, -0.9238795325112865164385311800288, -0.93819133592248404873714662244311, -0.95105651629515319811503104574513, -0.96245523645364727816087224709918, -0.97236992039767644602221707827994, -0.98078528040323031955693977579358, -0.98768834059513754830561538256006, -0.99306845695492618197874890029198, -0.99691733373312796384624334677937, -0.99922903624072290096336246278952, -1.0, -0.99922903624072301198566492530517, -0.99691733373312807486854580929503, -0.99306845695492629300105136280763, -0.98768834059513777035022030759137, -0.98078528040323065262384716334054, -0.9723699203976767790891244658269, -0.96245523645364761122777963464614, -0.95105651629515364220424089580774, -0.93819133592248415975944908495876, -0.92387953251128696052774103009142, -0.90814317382508147336039883157355, -0.89100652418836834289805892694858, -0.87249600707279706401919838754111, -0.85264016435409251126742447013385, -0.83146961230254545771600760417641, -0.80901699437494811739668421068927, -0.78531693088074550068711232597707, -0.76040596560003137494021530073951, -0.73432250943568577739029024087358, -0.70710678118654752440084436210485, -0.67880074553294234007694285537582, -0.64944804833018410228362427005777, -0.61909394930983430160864600111381, -0.5877852522924733591480617178604, -0.55557023301960295480483864594135, -0.52249856471594946505376810819143, -0.48862124149695612684851653284568, -0.45399049973954697101419242244447, -0.41865973753742891094731248813332, -0.38268343236509039240189622432808, -0.34611705707749340632517487392761, -0.30901699437494850597474282949406, -0.27144044986507426253652397463156, -0.23344536385590608507101251234417, -0.19509032201612871992857378700137, -0.15643446504023197918264997952065, -0.11753739745783854664384904253893, -0.07845909572784562358016557936935, -0.039259815759069074947440469713911};

/*modulator signal*/
__vo uint16_t u_control_pos;
__vo uint16_t u_control_neg;

uint8_t systemState;

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
 * 
 * @callby					Control_Stop
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
 * 
 * @callby					ControlInit
 * 
 * @calls					GPIO_PClkC
 * 							GPIO_Init
 * 							GPIO_IRQInterruptConfig
 * 							GPIO_IRQPriorityConfig
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
 * 
 * @callby					ControlInit
 * 
 * @calls					GPIO_PClkC
 * 							GPIO_Init
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
 * @callby					ControlInit
 * 
 * @calls					GPIO_PClkC
 * 							GPIO_Init
 * 							ADC_PClkC
 * 							ADC_ChannelConfig
 * 							ADC_Init
 * 							DMA_PClkC
 * 							DMA_Init
 * 							DMA_SetAddresses
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
 * @Requirements           7.2.1 [10]
 * 
 * @callby					ControlInit
 * 
 * @calls					TIM_Init
 * 							TIM_IRQInterruptConfig
 * 							TIM_IRQPriorityConfig
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
 * @Requirements           7.2.1 [020]
 * 
 * @callby					ControlInit
 * 
 * @calls					TIM_PClkC
 * 							TIM_Init
 * 							TIM_PWM_Channel_Init
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
 * @callby					Control_DutyCycle
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
 * @callby					Control_DutyCycle
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
 * @callby					Control_DutyCycle
 * 
 *********************************************************************************************************************************************************************/

void CascadeControl(float cosine_wt, float sine_wt, float V_CD, float I_Q, float I_INV, __vo float *pe1_z_0, __vo float *pe1_z_1, __vo float *pe2_z_0, __vo float *pe2_z_1, __vo float *py1_z_0, __vo float *py1_z_1, __vo float *py2_z_0, __vo float *py2_z_1)
{
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

	Refresh_Duty_Cycle(*py2_z_0);
}

/*********************************************************************************************************************************************************************
 * @fn                     CascadeControl2
 *
 * @brief                  A new version of cascaded control strategy with two discrete PI loops: 
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
 * @callby					Control_DutyCycle
 * 
 *********************************************************************************************************************************************************************/

void iCascadeControl2( float cosine_wt, float V_G, float I_INV, float I_L, __vo float *pe1_z_0, __vo float *pe1_z_1, __vo float *pe2_z_0, __vo float *pe2_z_1, __vo float *py1_z_0, __vo float *py1_z_1, __vo float *py2_z_0, __vo float *py2_z_1)
{
 
	(*pe1_z_0) = I_L*(0.5f) - I_INV*(0.4615384615384615f);																			//Evaluates error among reference and DC sensed value on DC bus of the inverter
 
	(*py1_z_0) = (*py1_z_1) + 0.60000000000003128608483393691131f*(*pe1_z_0) - 0.59999999999996878052854754059808f*(*pe1_z_1);		//External discrete PI control loop
 
	(*pe1_z_1) = (*pe1_z_0);																										//Updating last error as the most recent one
	(*py1_z_1) = (*py1_z_0);																										//Updating last output PI control value as the most recent one
 
	(*pe2_z_0) = (*py1_z_0) + (5.0f)*cosine_wt - (0.3278688524590164f)*V_G; 														//Calculates the new error with a reference given by external control loop output, quadrature current and inverter sense current
 
	(*py2_z_0) = (*py2_z_1) + 0.11559895833333333333333333333333f*(*pe2_z_0) - 0.11440104166666666666666666666667f*(*pe2_z_1); 																//Internal discrete PI control loop
 
	if((*py2_z_0)> (0.99)) (*py2_z_0) = 0.99;
	if((*py2_z_0)< (-0.99)) (*py2_z_0) = -0.99;																						//Setting boundaries for control signal
 
	(*pe2_z_1) = (*pe2_z_0);																										//Updating last error as the most recent one
	(*py2_z_1) = (*py2_z_0);																										//Updating last output PI control value as the most recent one
 
	Refresh_Duty_Cycle(*py2_z_0);
 
}

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
 * @callby					Control_Mode
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
 * @callby					Control_Stop
 * 
 * @calls					TIM_PWM_Disable
 * 							GPIO_WriteToOutputPin
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
 * @callby					Control_DutyCycle
 * 
 * @calls					TIM_PWM_DutyCycle
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
 * @calls					TIM_IRQHandling
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
 * @calls					Utility_GPIOInits
 * 							PWM_GPIOInits
 * 							Sensors_Init
 * 							SamplingRateTIMInit
 * 							PWM_TIMInits
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
 * @callby					Control_Mode
 * 
 * @calls					TIM_Start
 * 							DMA_StartTransfer
 * 
 *********************************************************************************************************************************************************************/
void Control_Start(void)
{
	TIM_Start(&TIM_2);
	TIM_Start(&TIM_4);
	DMA_StartTransfer(&DMA2_ADC1Handle);
	PWM_Enable();
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
 * @Requirements           7.2.2 [030.C10]
 * 
 * @callby					Control_Mode
 * 
 * @calls					TIM_Stop
 * 							DMA_StopTransfer
 * 							PWM_Disable
 * 							ResetTime
 * 
 *********************************************************************************************************************************************************************/
void Control_Stop(void)
{
	TIM_Stop(&TIM_2);
	TIM_Stop(&TIM_4);
	DMA_StopTransfer(&DMA2_ADC1Handle);
	PWM_Disable();
	ResetTime();
	for (int i=0; i<40; i++) 
	{
		i_L_buffer[i] = 0;
	}
}


/*********************************************************************************************************************************************************************
 * @fn                     Refresh_Duty_Cycle
 *
 * @brief                  Updates the PWM duty cycle for two complementary channels based on a normalized control variable.
 * 
 * @param                  var  - Normalized control input in range [-1.0, +1.0], representing modulation index or reference signal.
 *
 * @return                 None
 * 
 * @note                   - Computes duty cycle for Channel 1 as ((var * 0.5) + 0.5) * ARR, mapping [-1,+1] to [0,ARR].
 *                         - Computes duty cycle for Channel 2 as ((-var * 0.5) + 0.5) * ARR to maintain complementary behavior.
 *                         - Uses TIM_PWM_DutyCycle() to apply new values to TIM4 PWM outputs.
 *                         - Ensures symmetrical modulation around 50% duty for sinusoidal reference.
 *
 * @Requirements           TO-DO
 * 
 * @callby                 Control_DutyCycle and CascadeControl
 * 
 * @calls				   TIM_PWM_DutyCycle
 * 
 *********************************************************************************************************************************************************************/

void Refresh_Duty_Cycle(float var)
{
	TIM_PWM_DutyCycle(&TIM_4, &TIM4_PWM_Channel_1, (float)(( var*0.5) + 0.5)*(TIM4->ARR));
	TIM_PWM_DutyCycle(&TIM_4, &TIM4_PWM_Channel_2, (float)((-var*0.5) + 0.5)*(TIM4->ARR));
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
	sent++;
	index++;
	v_g = 	(raw_sensor_value[0]/ADC_RESOLUTION - ADC_OFFSET_VOLTAGE)*ADC_VOLTAGE_REF*ADC_GRID_VOLTAGE_K;
	i_inv = (raw_sensor_value[1]/ADC_RESOLUTION - ADC_OFFSET_VOLTAGE)*ADC_VOLTAGE_REF*ADC_INV_CURRENT_K;
	i_L = 	(raw_sensor_value[2]/ADC_RESOLUTION - ADC_OFFSET_VOLTAGE)*ADC_VOLTAGE_REF*ADC_LOAD_CURRENT_K;
	v_cd = 	(raw_sensor_value[3]/ADC_RESOLUTION)*ADC_DC_VOLTAGE_K;

	if(index >= 160) index = 0; //Resets index after one full cycle of 60 Hz at 9.6 kHz sampling rate

	if(sent == 4)
	{
		values[0] = v_g;
		values[1] = i_L ;
		values[2] = i_inv;
		values[3] = v_cd;
		values[4] = ElapsedTime;
		valid_send = FLAG_RESET;
		sent = 0;
	}else
	{
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
 *                         - If `systemState` bit 0 is DISABLE → Executes OpenLoop() for PWM control.
 *                         - Else → Executes CascadeControl() for closed-loop control using PI regulators.
 *                         - Updates PWM duty cycles through PWM_dutyCycle_control() using computed positive and negative control signals.
 *
 * @Requirements           TO-DO
 * 
 * @calls					NINETYDegreePhaseShift
 * 							QTransform
 * 							OpenLoop
 * 							CascadeControl
 * 							PWM_dutyCycle_control
 * 
 *********************************************************************************************************************************************************************/
void Control_DutyCycle(void)
{
	/*In case there is a high presence of noise, signals will be filtered*/

	cosine = coseno[index];

	delayed_i_L = SignalDelay(i_L_buffer, i_L, &Buffer_Counter_delayed_i_L, &Buffer_Ready_Flag_delayed_i_L, 154);

	/*-------------------------------Cementery-----------------------------------------------------------------------
	
			sine = NINETYDegreePhaseShift(cos_buffer, cosine, &Buffer_Counter_Cos, &Buffer_Ready_Flag_Cos);

			i_L90 = NINETYDegreePhaseShift(i_L_buffer, i_L, &Buffer_Counter_iL, &Buffer_Ready_Flag_iL);
	
			i_Q = QTransform(cosine, sine, i_L, i_L90);

			Rest in peace Q Transform
	-----------------------------------------------------------------------------------------------------------------*/

	if(systemState == GRID_FOLLOWING_MODE) Refresh_Duty_Cycle(cosine);

	if(systemState == VAR_COMPENSATION_MODE) CascadeControl2(cosine, v_g, i_inv, delayed_i_L, &e1_z_0, &e1_z_1, &e2_z_0, &e2_z_1, &y1_z_0, &y1_z_1, &y2_z_0, &y2_z_1);

}

/*********************************************************************************************************************************************************************
 * @fn                     Control_Mode
 *
 * @brief                  Sets the control system operation mode (Open Loop or Closed Loop) and power state (Enable or Disable). Handles safe transitions by resetting PI controllers when switching to Open Loop.
 * 
 * @param                  Power – Indicates whether the system should be powered ON (ENABLE) or OFF (DISABLE).
 * @param                  Loop  – Indicates whether the control loop should operate in Closed Loop (ENABLE) or Open Loop (DISABLE).
 *
 * @return                 uint8_t – Returns the updated `systemState` status flag.
 * 
 * @note                   - If `Loop == DISABLE`, resets PI controllers and sets Open Loop mode.
 *                         - If `Loop == ENABLE`, sets Closed Loop mode.
 *                         - If `Power == DISABLE`, stops control system and sets PWM status to OFF.
 *                         - If `Power == ENABLE`, starts control system and sets PWM status to ON.
 *                         - Uses macros: `SET_OPEN_LOOP_MODE`, `SET_CLOSED_LOOP_MODE`, `SYSTEM_OFF_FLAG`, `SYSTEM_ON_FLAG`.
 *
 * @Requirements           TO-DO
 * 
 * @calls					Control_Stop
 * 							Control_Start
 * 							ResetPIControllers
 * 
 *********************************************************************************************************************************************************************/
uint8_t Control_Mode(uint8_t Power, uint8_t Loop)
{
	/*When Operation Mode is zero it resets PI controllers from CascadeControl(), to assure safe and smooth transition to Closed Loop Mode Operation*/
	if( Power == DISABLE && ((systemState >> SYSTEM_STATUS_FLAG) & GRID_FOLLOWING_MODE) == FLAG_SET )
	{
		Control_Stop(); 
		SYSTEM_OFF_FLAG(systemState);	//Set System Status Flag to Disabled
		SET_OPEN_LOOP_MODE(systemState); //Set Loop Status Flag to Open
	} else if( Power == ENABLE && ((systemState >> SYSTEM_STATUS_FLAG) & GRID_FOLLOWING_MODE) == FLAG_RESET )
	{
		Control_Start(); 
		SYSTEM_ON_FLAG(systemState); //Set System Status Flag to Enabled
	}
	if((systemState >> SYSTEM_STATUS_FLAG) & GRID_FOLLOWING_MODE)
	{
		if( Loop == DISABLE && (systemState >> MODE_FLAG) == FLAG_SET)
		{
			ResetPIControllers(&e1_z_0, &e1_z_1, &e2_z_0, &e2_z_1, &y1_z_0, &y1_z_1, &y2_z_0, &y2_z_1);
			SET_OPEN_LOOP_MODE(systemState); //Set Loop Status Flag to Open
		} else if ( Loop == ENABLE && (systemState >> MODE_FLAG) == FLAG_RESET)
		{
			SET_CLOSED_LOOP_MODE(systemState); //Set Loop Status Flag to Closed
		}
	}
	return systemState;
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
