/*
 * stm32f446_fmc_driver.c
 *
 *  Created on: Nov 5, 2025
 *      Author: jtlopez
 */

#include "stm32f446xx_fmc_driver.h"

/* 
 * Peripheral Clock setup
 */

void FMC_PClkC(uint8_t EnorDi)
{
    if(EnorDi)
    {
        FMC_PCLK_EN();
    }else
    {
        FMC_PCLK_DI();
    }
}

/*
 * Peripheral Control
 */

void FMC_PeripheralControl(FMC_Handle_t *pFMCHandle, uint8_t EnorDi)
{
    if(EnorDi)
    {
        *pFMCHandle->BCR[pFMCHandle->Bank] |= (1 << FMC_BCR_MBKEN);
    }else
    {
        *pFMCHandle->BCR[pFMCHandle->Bank] &= ~(1 << FMC_BCR_MBKEN);
    }
}

/*
 * Init and De-Init
 */

void FMC_Init(FMC_Handle_t *pFMCHandle)
{
    pFMCHandle->BCR[0] = &FMC->BCR1;
    pFMCHandle->BCR[1] = &FMC->BCR2;
    pFMCHandle->BCR[2] = &FMC->BCR3;
    pFMCHandle->BCR[3] = &FMC->BCR4;
    pFMCHandle->BTR[0] = &FMC->BTR1;
    pFMCHandle->BTR[1] = &FMC->BTR2;
    pFMCHandle->BTR[2] = &FMC->BTR3;
    pFMCHandle->BTR[3] = &FMC->BTR4;
    pFMCHandle->BWTR[0] = &FMC->BWTR1;
    pFMCHandle->BWTR[1] = &FMC->BWTR2;
    pFMCHandle->BWTR[2] = &FMC->BWTR3;
    pFMCHandle->BWTR[3] = &FMC->BWTR4;

    FMC_PClkC(ENABLE);

    uint32_t temp;

    temp |= (pFMCHandle->FMC_Config.ADDR_MUX_EN << FMC_BCR_MUXEN);
    temp |= (pFMCHandle->FMC_Config.MEMORY_TYPE << FMC_BCR_MTYP);
    temp |= (pFMCHandle->FMC_Config.MEMORY_DATABUS_WIDTH << FMC_BCR_MWID);
    temp |= (pFMCHandle->FMC_Config.WAIT_BIT_POLARITY << FMC_BCR_WAITPOL);
    temp |= (pFMCHandle->FMC_Config.EXTENDED_MODE << FMC_BCR_EXTMOD);
    temp |= (pFMCHandle->FMC_Config.ASYNC_WAIT_EN << FMC_BCR_ASYNCWAIT);

    *pFMCHandle->BCR[pFMCHandle->Bank] = temp;

    temp = 0;

    temp |= (pFMCHandle->FMC_Config.ADDR_HOLD_DURATION << FMC_BTR_ADDHLD);
    temp |= (pFMCHandle->FMC_Config.DATA_PHASE_DURATION << FMC_BTR_DATAST);
    temp |= (pFMCHandle->FMC_Config.BUST_TURNAROUND_DURATION << FMC_BTR_BUSTURN);
    temp |= (pFMCHandle->FMC_Config.ACCESS_MODE << FMC_BTR_ACCMOD);

    *pFMCHandle->BTR[pFMCHandle->Bank] = temp;

    FMC_PeripheralControl(pFMCHandle, ENABLE);
}


void FMC_DeInit()
{
    FMC_REG_RESET();
}

/*
 * Read and Write
 */

void FMC_Read()
{

}

void FMC_Write()
{
    
}