/*
 * stm32f446_fmc_driver.h
 *
 *  Created on: Nov 5, 2025
 *      Author: jtlopez
 */

#ifndef INC_STM32F446_FMC_DRIVER_H_
#define INC_STM32F446_FMC_DRIVER_H_

#include "stm32f446xx.h"

/*
 * Configuration Structure for FMC
 */

typedef struct
{
    uint8_t ADDR_MUX_EN;
    uint8_t MEMORY_TYPE;
    uint8_t MEMORY_DATABUS_WIDTH;
    uint8_t WAIT_BIT_POLARITY;
    uint8_t ASYNC_WAIT_EN;
    uint8_t EXTENDED_MODE;
    
    uint8_t ACCESS_MODE;
    uint8_t BUST_TURNAROUND_DURATION;
    uint8_t DATA_PHASE_DURATION;
    uint8_t ADDR_HOLD_DURATION;
    uint8_t ADDR_SET_DURATION;
}FMC_Config_t;

/*
 * Handling Structure for FMC
 */

typedef struct
{
	FMC_RegDef_t *pFMC = FMC;
    uint32_t* BCR[4];
    uint32_t* BTR[4];
    uint32_t* BWTR[4];
	FMC_Config_t FMC_Config;
    uint8_t Bank;
}FMC_Handle_t;

/*
 * @EXTENDED_MODE
 * Extended mode enable
 */

#define FMC_EXTMOD_EN			                ENABLE
#define FMC_EXTMOD_DI			                DISABLE


/*
 * @MEMORY_TYPE
 * Memory Type
 */

#define FMC_MEMORY_TYPE_SRAM			        0
#define FMC_MEMORY_TYPE_PSRAM                   1
#define FMC_MEMORY_TYPE_NOR_ONENAND_FLASH       2

/*
 * @MEMORY_DATABUS_WIDTH
 * Memory Data Bus width
 */

#define FMC_MEMORY_DATABUS_8			        0
#define FMC_MEMORY_DATABUS_16			        0

/************************************************************************************
 ****************************APIs supported by this driver***************************
 *********For more information about the APIS check the function definitions*********
 ************************************************************************************/

/* 
 * Peripheral Clock setup
 */

void FMC_PClkC(uint8_t EnorDi);

/*
 * Peripheral Control
 */

void FMC_PeripheralControl(uint8_t Bank, uint8_t EnorDi);

/*
 * Init and De-Init
 */

void FMC_Init(FMC_Handle_t *pFMCHandle);
void FMC_DeInit();

/*
 * Read and Write
 */

void FMC_Read()
void FMC_Write()






#endif /* INC_STM32F446XX_FMC_DRIVER_H_ */