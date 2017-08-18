/*
 * @brief LPC84X I2C driver
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2014
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licenser disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

#include "chip.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/
/* I2C Master send polling */
void Chip_I2C_Master_Send_Blocking( LPC_I2C_T *pI2C, I2C_DATA_SETUP_T *pI2CSetup )
{
	uint32_t i;

	pI2C->MSTDAT = pI2CSetup->addr;
	pI2C->MSTCTL = I2C_CTL_MSTSTART;

	for ( i = 0; i < pI2CSetup->MstTxCnt; i++ ) {
		/* Move only if TXRDY is ready */
		while ( !( pI2C->STAT & I2C_STAT_MSTPEND ) );
		if ( ( pI2C->STAT & I2C_MASTER_STATE_MASK ) != I2C_STAT_MSTTX ) {
			while (1);
		}
		pI2C->MSTDAT = (uint32_t)*(pI2CSetup->pMstTx)++;
		pI2CSetup->MstTxTmtd++;
		pI2C->MSTCTL = I2C_CTL_MSTCONTINUE;
	}

	/* Wait for the last one to go out. */
	while ( !( pI2C->STAT & I2C_STAT_MSTPEND ) );
	if ( ( ( pI2C->STAT & I2C_MASTER_STATE_MASK ) != I2C_STAT_MSTTX ) && ( ( pI2C->STAT & I2C_MASTER_STATE_MASK ) != I2C_STAT_MSTNACKTX ) ) {
		while (1);
	}

	/* Send STOP condition. */  
	pI2C->MSTCTL = I2C_CTL_MSTSTOP | I2C_CTL_MSTCONTINUE;
	/* Should back to the IDLE mode once the transmission is done. */
	while ( ( pI2C->STAT & ( I2C_STAT_MSTPEND | I2C_MASTER_STATE_MASK ) ) != ( I2C_STAT_MSTPEND | I2C_STAT_MSTIDLE ) );
	return; 
}

/* I2C Master receive polling */
void Chip_I2C_Master_Receive_Blocking( LPC_I2C_T *pI2C, I2C_DATA_SETUP_T *pI2CSetup )
{
	uint32_t i;

	pI2C->MSTDAT = pI2CSetup->addr|I2C_RD_BIT;
	pI2C->MSTCTL = I2C_CTL_MSTSTART;

	for ( i = 0; i < pI2CSetup->MstRxCnt; i++ ) {
		/* Slave address has been sent, master receive is ready. */
		while ( !( pI2C->STAT & I2C_STAT_MSTPEND ) );
		if ( ( pI2C->STAT & I2C_MASTER_STATE_MASK ) != I2C_STAT_MSTRX ) {
			while (1);
		}
		*pI2CSetup->pMstRx = (uint8_t)pI2C->MSTDAT;
		pI2CSetup->pMstRx++;
		pI2CSetup->MstRxRcvd++;		
		if ( i != pI2CSetup->MstRxCnt - 1 ) {
			pI2C->MSTCTL = I2C_CTL_MSTCONTINUE;
		}
	}
	pI2C->MSTCTL = I2C_CTL_MSTSTOP | I2C_CTL_MSTCONTINUE;
	/* Should back to the IDLE mode once the transmission is done. */
	while ( ( pI2C->STAT & ( I2C_STAT_MSTPEND | I2C_MASTER_STATE_MASK ) ) != ( I2C_STAT_MSTPEND | I2C_STAT_MSTIDLE ) );
	return; 
}

/* I2C Master send and receive polling */
void Chip_I2C_Master_Send_Receive_Blocking( LPC_I2C_T *pI2C, I2C_DATA_SETUP_T *pI2CSetup )
{
	uint32_t i;

	pI2C->MSTDAT = pI2CSetup->addr;
	pI2C->MSTCTL = I2C_CTL_MSTSTART;

	for ( i = 0; i < pI2CSetup->MstTxCnt; i++ ) {
		/* Move only if TXRDY is ready */
		while ( !( pI2C->STAT & I2C_STAT_MSTPEND ) );
		if ( ( pI2C->STAT & I2C_MASTER_STATE_MASK ) != I2C_STAT_MSTTX ) {
			while (1);
		}
		pI2C->MSTDAT = (uint32_t)*(pI2CSetup->pMstTx)++;
		pI2CSetup->MstTxTmtd++;
		pI2C->MSTCTL = I2C_CTL_MSTCONTINUE;
	}

	/* Wait for the last TX to finish before setting repeated start. */
	/* Move on only if TXRDY is ready */
	while ( !( pI2C->STAT & I2C_STAT_MSTPEND ) );
	if ( ( ( pI2C->STAT & I2C_MASTER_STATE_MASK ) != I2C_STAT_MSTTX ) && ( ( pI2C->STAT & I2C_MASTER_STATE_MASK ) != I2C_STAT_MSTNACKTX ) ) {
		while (1);
	}

	/* Repeated Start */
	pI2C->MSTDAT = pI2CSetup->addr|I2C_RD_BIT;   
	pI2C->MSTCTL = I2C_CTL_MSTSTART|I2C_CTL_MSTCONTINUE;

	for ( i = 0; i < pI2CSetup->MstRxCnt; i++ ) {
		/* Slave address has been sent, master receive is ready. */
		while ( !( pI2C->STAT & I2C_STAT_MSTPEND ) );
		if ( ( pI2C->STAT & I2C_MASTER_STATE_MASK ) != I2C_STAT_MSTRX ) {
			while (1);
		}
		*pI2CSetup->pMstRx = (uint8_t)pI2C->MSTDAT;
		pI2CSetup->pMstRx++;
		pI2CSetup->MstRxRcvd++;		
		if ( i != pI2CSetup->MstRxCnt - 1 ) {
			pI2C->MSTCTL = I2C_CTL_MSTCONTINUE;
		}
	}

	pI2C->MSTCTL = I2C_CTL_MSTSTOP | I2C_CTL_MSTCONTINUE;
	/* Should back to the IDLE mode once the transmission is done. */
	while ( ( pI2C->STAT & ( I2C_STAT_MSTPEND | I2C_MASTER_STATE_MASK ) ) != ( I2C_STAT_MSTPEND | I2C_STAT_MSTIDLE ) );
	return; 
}

/* I2C Slave send polling */
void Chip_I2C_Slave_Send_Blocking( LPC_I2C_T *pI2C, I2C_DATA_SETUP_T *pI2CSetup )
{
	uint32_t i;

	pI2C->SLVCTL = I2C_CTL_SLVCONTINUE;
	while ( ( pI2C->STAT & ( I2C_STAT_SLVPEND | I2C_SLAVE_STATE_MASK ) ) != ( I2C_STAT_SLVPEND | I2C_STAT_SLVADDR ) );
	if ( pI2C->SLVDAT != ( pI2C->SLVADR0 | I2C_RD_BIT ) ) {
		while (1);
	}
	pI2C->SLVCTL = I2C_CTL_SLVCONTINUE;

	for ( i = 0; i < pI2CSetup->SlvTxCnt; i++ ) {
		/* Move only if TX is ready */
		while ( ( pI2C->STAT & ( I2C_STAT_SLVPEND | I2C_SLAVE_STATE_MASK ) ) != ( I2C_STAT_SLVPEND | I2C_STAT_SLVTX ) );
		pI2C->SLVDAT = (uint32_t)*(pI2CSetup->pSlvTx)++;
		pI2C->SLVCTL = I2C_CTL_SLVCONTINUE;
	}
	return; 
}

/* I2C Slave receive polling */
void Chip_I2C_Slave_Receive_Blocking( LPC_I2C_T *pI2C, I2C_DATA_SETUP_T *pI2CSetup )
{
	uint32_t i;

	pI2C->SLVCTL = I2C_CTL_SLVCONTINUE;
	while ( ( pI2C->STAT & ( I2C_STAT_SLVPEND | I2C_SLAVE_STATE_MASK ) ) != ( I2C_STAT_SLVPEND | I2C_STAT_SLVADDR ) );
	if ( pI2C->SLVDAT != ( pI2C->SLVADR0 & ~I2C_RD_BIT ) ) {
		while (1);
	}
	pI2C->SLVCTL = I2C_CTL_SLVCONTINUE;

	for ( i = 0; i < pI2CSetup->SlvRxCnt; i++ ) {
		/* Move only if RX is ready */
		while ( ( pI2C->STAT & ( I2C_STAT_SLVPEND | I2C_SLAVE_STATE_MASK ) ) != ( I2C_STAT_SLVPEND | I2C_STAT_SLVRX ) );
		*pI2CSetup->pSlvRx = (uint8_t)pI2C->SLVDAT;
		pI2CSetup->pSlvRx++;
		if ( i != pI2CSetup->SlvRxCnt - 1 ) {
			pI2C->SLVCTL = I2C_CTL_SLVCONTINUE;
		}
		else {
			// I2Cx->SLVCTL = I2C_CTL_SLVNACK | I2C_CTL_SLVCONTINUE; 
			pI2C->SLVCTL = I2C_CTL_SLVCONTINUE;
		}
	}
	return; 
}

/* I2C receive/transmit interrupt handler */
void Chip_I2C_IRQHandler(LPC_I2C_T *pI2C, I2C_DATA_SETUP_T *pI2CSetup )
{
	uint32_t active = Chip_I2C_GetIntStatus(pI2C);
	uint32_t i2cmststate = Chip_I2C_GetStatus(pI2C) & I2C_MASTER_STATE_MASK;	/* Only check Master and Slave State */
	uint32_t i2cslvstate = Chip_I2C_GetStatus(pI2C) & I2C_SLAVE_STATE_MASK;
	uint32_t slave_addr;

	if ( active & (I2C_STAT_MONRDY | I2C_STAT_MONOVERRUN | I2C_STAT_MONIDLE) ) {
		if ( active & I2C_STAT_MONRDY ) {
			*pI2CSetup->pMon = (uint16_t)pI2C->MONRXDAT;
			pI2CSetup->pMon++;	
			pI2CSetup->MonCnt++;
		}
		if ( active & I2C_STAT_MONOVERRUN ) {
			Chip_I2C_ClearStatus(pI2C, I2C_STAT_MONOVERRUN);
		}
		if ( active & I2C_STAT_MONIDLE ) {
			Chip_I2C_ClearStatus(pI2C, I2C_STAT_MONIDLE);
		}
	}

	if ( active & I2C_STAT_MSTPEND ) {
		/* Below five states are for Master mode: IDLE, TX, RX, NACK_ADDR, NAC_TX. 
		IDLE is not under consideration for now. */
		switch ( i2cmststate ) {
			case I2C_STAT_MSTIDLE:
			default:
				pI2CSetup->Error = i2cmststate;
				if ( ( pI2CSetup->MstRxRcvd == pI2CSetup->MstRxCnt ) && ( pI2CSetup->MstTxTmtd == pI2CSetup->MstTxCnt ) ) {
					pI2CSetup->master_comp_flag = TRUE;
					Chip_I2C_Int_Cmd(pI2C, I2C_STAT_MSTPEND, (FunctionalState)DISABLE);
				}
				else if ( pI2CSetup->MstRxRcvd != pI2CSetup->MstRxCnt ) {
					Chip_I2C_SendStartWithAddr(pI2C, pI2CSetup->addr | I2C_RD_BIT);
				}
				else if ( pI2CSetup->MstTxTmtd != pI2CSetup->MstTxCnt ) {
					Chip_I2C_SendStartWithAddr(pI2C, pI2CSetup->addr);
				}
			break;

			case I2C_STAT_MSTRX:
				*pI2CSetup->pMstRx = (uint8_t)pI2C->MSTDAT;
				pI2CSetup->pMstRx++;	
				pI2CSetup->MstRxRcvd++;
				if ( pI2CSetup->MstRxRcvd >= pI2CSetup->MstRxCnt ) {
					Chip_I2C_SetMasterCtrl(pI2C, I2C_CTL_MSTSTOP | I2C_CTL_MSTCONTINUE);
				}
				else {
					Chip_I2C_SetMasterCtrl(pI2C, I2C_CTL_MSTCONTINUE);
				}
			break;

			case I2C_STAT_MSTTX:
				if ( pI2CSetup->MstTxTmtd >= pI2CSetup->MstTxCnt ) {
					if ( pI2CSetup->tx_rx_flag == TRUE) {  //do a restart and master receive mode?
						Chip_I2C_SendStartWithAddr(pI2C, pI2CSetup->addr | I2C_RD_BIT);
						return ;                    // next intr will take to master-receive mode 
					}
					Chip_I2C_SetMasterCtrl(pI2C, I2C_CTL_MSTSTOP);
				}	
				else {
					pI2C->MSTDAT = *pI2CSetup->pMstTx;
					pI2CSetup->pMstTx++;
					pI2CSetup->MstTxTmtd++;
					pI2C->MSTCTL = I2C_CTL_MSTCONTINUE;
				}
			break;

			case I2C_STAT_MSTNACKADDR:
			case I2C_STAT_MSTNACKTX:
				pI2CSetup->Error = i2cmststate;
				Chip_I2C_SetMasterCtrl(pI2C, I2C_CTL_MSTSTOP);
			break;
		}	  
  }
	
	if ( active & I2C_STAT_SLVPEND ) {  	
		/* Below three states are for Slave mode: Address Received, TX, and RX. */
		switch ( i2cslvstate ) {
			case I2C_STAT_SLVRX:
				if ( pI2CSetup->SlvRxRcvd >= pI2CSetup->SlvRxCnt ) {
					Chip_I2C_SetSlaveCtrl(pI2C, I2C_CTL_SLVNACK);
					return;
				}
				*pI2CSetup->pSlvRx = (uint8_t)pI2C->SLVDAT;
				pI2CSetup->pSlvRx++;	
				pI2CSetup->SlvRxRcvd++;
				Chip_I2C_SetSlaveCtrl(pI2C, I2C_CTL_SLVCONTINUE);
			break;

			case I2C_STAT_SLVTX:
				pI2C->SLVDAT = *pI2CSetup->pSlvTx;
				pI2CSetup->pSlvTx++;
				pI2CSetup->SlvTxTmtd++;
				Chip_I2C_SetSlaveCtrl(pI2C, I2C_CTL_SLVCONTINUE);
			break;
	  
			case I2C_STAT_SLVADDR:
			default:
				slave_addr = pI2C->SLVDAT;
				pI2CSetup->Error = i2cslvstate;
				/* slave address is received. */
				if ( ( slave_addr & I2C_RD_BIT ) && ( pI2CSetup->SlvTxTmtd != pI2CSetup->SlvTxCnt ) ) {			/* Slave write to the master */
					Chip_I2C_SetSlaveCtrl(pI2C, I2C_CTL_SLVCONTINUE);
				}
				else if ( ( ( slave_addr & I2C_RD_BIT ) == 0 ) ) { /* Slave read from the master */
					if ( pI2CSetup->SlvRxRcvd >= pI2CSetup->SlvRxCnt )
					{
						Chip_I2C_SetSlaveCtrl(pI2C, I2C_CTL_SLVNACK);
						return;
					}
					*pI2CSetup->pSlvRx = (uint8_t)pI2C->SLVDAT;
					pI2CSetup->pSlvRx++;	
					pI2CSetup->SlvRxRcvd++;
					Chip_I2C_SetSlaveCtrl(pI2C, I2C_CTL_SLVCONTINUE);
				}
			break;
		}
	}
	if ( active & I2C_STAT_SLVDESEL ) {
		Chip_I2C_ClearStatus(pI2C, I2C_STAT_SLVDESEL);
		Chip_I2C_Int_Cmd(pI2C, I2C_STAT_SLVPEND | I2C_STAT_SLVDESEL, (FunctionalState)DISABLE);
		pI2CSetup->slave_comp_flag = TRUE;
	}
	return;
}
