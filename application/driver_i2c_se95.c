#include "board.h"
#include <stdio.h>
#include <string.h>

#include "driver_all_board.h"
#include "driver_i2c_se95.h"

I2CM_XFER_T *g_SE95Xfer;

void se95_init(void)
{
    // GPIO Initialized in driver_all_board.c
    // I2C0 Initialized in driver_all_board.c
    
    g_SE95Xfer = &g_I2C0Xfer;
}

void se95_read(float *TempData, uint8_t *OrgDataBuf)
{
    uint8_t SE95TxBuf[2];
    uint8_t SE95RxBuf[2];
    SE95TxBuf[0] = 0x00;
    SE95TxBuf[1] = 0x00;
    SE95RxBuf[0] = 0x00;
    SE95RxBuf[1] = 0x00;
    
	g_SE95Xfer->slaveAddr = SE95_I2C_ADR;
	g_SE95Xfer->txBuff = SE95TxBuf;
	g_SE95Xfer->txSz = 1;
	g_SE95Xfer->rxBuff = SE95RxBuf;
	g_SE95Xfer->rxSz = 2;
	g_SE95Xfer->status = 0;
	Chip_I2CM_Xfer(SE95_I2C_PORT, (I2CM_XFER_T *)g_SE95Xfer);
    
    /* Enable I2C Interrupt */
    Chip_I2C_EnableInt(SE95_I2C_PORT, I2C_INTENSET_MSTPENDING | I2C_INTENSET_MSTRARBLOSS | I2C_INTENSET_MSTSTSTPERR);
    
    if(g_I2C0IntEnable == 0)
    {
        /* Enable the interrupt for the I2C */
        NVIC_EnableIRQ(SE95_I2C_IRQNUM);
        /* Mark flag */
        g_I2C0IntEnable = 1;
    }
	/* Wait for transfer completion */
	while ( 1 ) {
		if (g_SE95Xfer->status == I2CM_STATUS_OK) {
			Chip_I2C_ClearInt(SE95_I2C_PORT, I2C_INTENSET_MSTPENDING | I2C_INTENSET_MSTRARBLOSS | I2C_INTENSET_MSTSTSTPERR);
			break;
		}
	}
    
#if 0		
/* SE95 test */
//		SE95RxBuf[0] = 0xE7; SE95RxBuf[1] = 0x00; // -25oC
		SE95RxBuf[0] = 0xC9; SE95RxBuf[1] = 0x20; // -54.875oC
//		SE95RxBuf[0] = 0xFF; SE95RxBuf[1] = 0xE0; // -1oC
#endif		
    
    if( (SE95RxBuf[0]&0x80) == 0x00)
    {
        *TempData = ((float)( ((SE95RxBuf[0]<<8) + SE95RxBuf[1])>>5) * 0.125); 
    }
    else
    {
        *TempData = 0x800 - ((SE95RxBuf[0]<<8) + SE95RxBuf[1]>>5 );
        *TempData = -(((float)(*TempData)) * 0.125);
    }
    OrgDataBuf[0] = SE95RxBuf[0];
    OrgDataBuf[1] = SE95RxBuf[1];
}


// end file
