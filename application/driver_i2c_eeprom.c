#include "board.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "driver_all_board.h"
#include "driver_i2c_eeprom.h"

I2CM_XFER_T *g_EEPROMXfer;

void eeprom_init(void)
{
    // GPIO Initialized in driver_all_board.c
    // I2C0 Initialized in driver_all_board.c
    
    g_EEPROMXfer = &g_I2C1Xfer;
}

void eeprom_write(uint32_t EEPROMAddr, uint8_t* EEPROMData, uint32_t EEPROMSize)
{
    volatile uint8_t* EEPROMTxBuf;

    EEPROMTxBuf = (uint8_t *)malloc(EEPROMSize+1);
    EEPROMTxBuf[0] = EEPROMAddr;
    memcpy((void *)&EEPROMTxBuf[1], EEPROMData, EEPROMSize);
	g_EEPROMXfer->slaveAddr = EEPROM_I2C_ADR;
	g_EEPROMXfer->txBuff = (const uint8_t *)EEPROMTxBuf;
	g_EEPROMXfer->txSz = EEPROMSize+1;
	g_EEPROMXfer->rxBuff = NULL;
	g_EEPROMXfer->rxSz = 0;
	g_EEPROMXfer->status = 0;
	Chip_I2CM_Xfer(EEPROM_I2C_PORT, (I2CM_XFER_T *)g_EEPROMXfer);
    g_I2C1IntEnable = 0;
    /* Enable I2C Interrupt */
    Chip_I2C_EnableInt(EEPROM_I2C_PORT, I2C_INTENSET_MSTPENDING | I2C_INTENSET_MSTRARBLOSS | I2C_INTENSET_MSTSTSTPERR);
    
    if(g_I2C1IntEnable == 0)
    {
        /* Enable the interrupt for the I2C */
        NVIC_EnableIRQ(EEPROM_I2C_IRQNUM);
        /* Mark flag */
        g_I2C1IntEnable = 1;
    }
	/* Wait for transfer completion */
	while ( 1 ) {
		if (g_EEPROMXfer->status == I2CM_STATUS_OK) {
			Chip_I2C_ClearInt(EEPROM_I2C_PORT, I2C_INTENSET_MSTPENDING | I2C_INTENSET_MSTRARBLOSS | I2C_INTENSET_MSTSTSTPERR);
			break;
		}
	}
    
    free((void *)EEPROMTxBuf);
}

void eeprom_read(uint32_t EEPROMAddr, uint8_t* EEPROMData, uint32_t EEPROMSize)
{
    volatile uint8_t EEPROMTxBuf[1];
    EEPROMTxBuf[0] = EEPROMAddr;

	g_EEPROMXfer->slaveAddr = EEPROM_I2C_ADR;
	g_EEPROMXfer->txBuff    = (const uint8_t *)EEPROMTxBuf;
	g_EEPROMXfer->txSz      = 1;
	g_EEPROMXfer->rxBuff    = EEPROMData;
	g_EEPROMXfer->rxSz      = EEPROMSize;
	g_EEPROMXfer->status    = 0;
	Chip_I2CM_Xfer(EEPROM_I2C_PORT, (I2CM_XFER_T *)g_EEPROMXfer);
    
    /* Enable I2C Interrupt */
    Chip_I2C_EnableInt(EEPROM_I2C_PORT, I2C_INTENSET_MSTPENDING | I2C_INTENSET_MSTRARBLOSS | I2C_INTENSET_MSTSTSTPERR);
    g_I2C1IntEnable = 0;
    if(g_I2C1IntEnable == 0)
    {
        /* Enable the interrupt for the I2C */
        NVIC_EnableIRQ(EEPROM_I2C_IRQNUM);
        /* Mark flag */
        g_I2C1IntEnable = 1;
    }
	/* Wait for transfer completion */
	while ( 1 ) {
		if (g_EEPROMXfer->status == I2CM_STATUS_OK) {
			Chip_I2C_ClearInt(EEPROM_I2C_PORT, I2C_INTENSET_MSTPENDING | I2C_INTENSET_MSTRARBLOSS | I2C_INTENSET_MSTSTSTPERR);
			break;
		}
	}
}

void eeprom_test(void)
{
    volatile uint8_t EEPROMTestBuf[8];
    eeprom_write(0x00, "01234567",  8);
    DEBUGOUT("EPROM write data are %s", "01234567");
    eeprom_read (0x00, (uint8_t *)EEPROMTestBuf, 8);
    DEBUGOUT("EPROM read  data are %8s", EEPROMTestBuf);
}


// end file
