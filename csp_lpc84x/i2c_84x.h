/*
 * @brief LPC8xx I2C driver
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2014
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
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

#ifndef __I2C_84X_H_
#define __I2C_84X_H_

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup CHIP_I2C_84X CHIP: LPC8xx I2C driver (clock enable/disable only)
 * @ingroup CHIP_84X_Drivers
 * This driver provides the clock functions needed for I2C support.
 * @{
 */
/**
 * @brief I2C register block structure
 */
typedef struct {					/*!< I2C Structure */
  __IO uint32_t  CFG;			  /* 0x00 */
  __IO uint32_t  STAT;
  __IO uint32_t  INTENSET;
  __O  uint32_t  INTENCLR;
  __IO uint32_t  TIMEOUT;		/* 0x10 */
  __IO uint32_t  CLKDIV;
  __IO uint32_t  INTSTAT;
       uint32_t  Reserved0[1];  
  __IO uint32_t  MSTCTL;			  /* 0x20 */
  __IO uint32_t  MSTTIME;
  __IO uint32_t  MSTDAT;
       uint32_t  Reserved1[5];
  __IO uint32_t  SLVCTL;			  /* 0x40 */
  __IO uint32_t  SLVDAT;
  __IO uint32_t  SLVADR0;
  __IO uint32_t  SLVADR1;
  __IO uint32_t  SLVADR2;			  /* 0x50 */
  __IO uint32_t  SLVADR3;
  __IO uint32_t  SLVQUAL0;
       uint32_t  Reserved2[9];
  __I  uint32_t  MONRXDAT;			/* 0x80 */
} LPC_I2C_T;

/** @brief I2C MASTER STAT */
typedef enum I2C_MASTER_STAT {
	I2C_STAT_MSTIDLE =     (0x0 << 1),		
	I2C_STAT_MSTRX =       (0x1 << 1),		
	I2C_STAT_MSTTX =       (0x2 << 1),		
	I2C_STAT_MSTNACKADDR = (0x3 << 1),
	I2C_STAT_MSTNACKTX =   (0x4 << 1)
} I2C_MASTER_STAT_T;

/** @brief I2C SLAVE STAT */
typedef enum I2C_SLAVE_STAT {
	I2C_STAT_SLVADDR =     (0x0 << 9),		
	I2C_STAT_SLVRX =       (0x1 << 9),		
	I2C_STAT_SLVTX =       (0x2 << 9)
} I2C_SLAVE_STAT_T;

#define I2C_CFG_MSTENA				(1 << 0)
#define I2C_CFG_SLVENA				(1 << 1)
#define I2C_CFG_MONENA				(1 << 2)
#define I2C_CFG_TIMEOUTENA		(1 << 3)
#define I2C_CFG_MONCLKSTR			(1 << 4)

#define I2C_CTL_MSTCONTINUE		(1 << 0)
#define I2C_CTL_MSTSTART			(1 << 1)
#define I2C_CTL_MSTSTOP 			(1 << 2)
#define I2C_CTL_MSTDMA				(1 << 3)

#define I2C_CTL_SLVCONTINUE		(1 << 0)
#define I2C_CTL_SLVNACK				(1 << 1)
#define I2C_CTL_SLVDMA				(1 << 3)

#define I2C_TIM_MSTSCLLOW(d)	((d) << 0)
#define I2C_TIM_MSTSCLHIGH(d)	((d) << 4)

#define I2C_STAT_MSTPEND  		(1 << 0)
#define I2C_MASTER_STATE_MASK	(0x7<<1)
#define I2C_STAT_MSTIDLE	 		(0x0 << 1)
#define I2C_STAT_MSTRX	 			(0x1 << 1)
#define I2C_STAT_MSTTX	 			(0x2 << 1)
#define I2C_STAT_MSTNACKADDR	(0x3 << 1)
#define I2C_STAT_MSTNACKTX		(0x4 << 1)
#define I2C_STAT_MSTARBLOSS		(1 << 4)
#define I2C_STAT_MSTSSERR	 		(1 << 6)
#define I2C_STAT_MST_ERROR_MASK	(I2C_MSTNACKADDR|I2C_STAT_MSTNACKTX|I2C_STAT_MSTARBLOSS|I2C_STAT_MSTSSERR)

#define I2C_STAT_SLVPEND 			(1 << 8)
#define I2C_SLAVE_STATE_MASK	(0x3<<9)
#define I2C_STAT_SLVADDR			(0x0 << 9)
#define I2C_STAT_SLVRX  	 		(0x1 << 9)
#define I2C_STAT_SLVTX  	 		(0x2 << 9)
#define I2C_STAT_SLVNOTSTR		(1 << 11)
#define I2C_STAT_SLVSEL		 		(1 << 14)
#define I2C_STAT_SLVDESEL			(1 << 15)

#define I2C_STAT_MONRDY				(1 << 16)
#define I2C_STAT_MONOVERRUN 	(1 << 17)
#define I2C_STAT_MONACTIVE		(1 << 18)
#define I2C_STAT_MONIDLE			(1 << 19)

#define I2C_STAT_EVTIMEOUT		(1 << 24)
#define I2C_STAT_SCLTIMEOUT		(1 << 25)

#define I2C_RD_BIT             0x01

/**
 * @brief SPI data setup structure
 */
typedef struct {
  uint32_t  addr;
  uint32_t  tx_rx_flag;	/**< TRUE if it's a master send then receive, FALSE it's either a master send or receive */
	uint8_t   *pMstTx;	/**< Pointer to Master TX data buffer*/
	uint32_t  MstTxTmtd;
	uint32_t  MstTxCnt;/* Transmit Counter */
	uint8_t   *pMstRx;	/**< Pointer to Master RX data buffer*/
	uint32_t  MstRxRcvd;
	uint32_t  MstRxCnt;/* Receive Counter */
	uint32_t  master_comp_flag;
	uint8_t   *pSlvTx;	/**< Pointer to Slave TX data buffer*/
	uint32_t  SlvTxTmtd;
	uint32_t  SlvTxCnt;/* Transmit Counter */
	uint8_t   *pSlvRx;	/**< Pointer to Slave RX data buffer*/
	uint32_t  SlvRxRcvd;
	uint32_t  SlvRxCnt;/* Receive Counter */
	uint32_t  slave_comp_flag;
	uint16_t  *pMon;	/**< Pointer to Monitor data buffer*/
	uint32_t  MonCnt;
	uint32_t  Error;
} I2C_DATA_SETUP_T;

/**
 * @brief	Initialize I2C Interface
 * @return	Nothing
 * @note	This function enables the I2C clock.
 */
STATIC INLINE void Chip_I2C0_Init(void)
{
	/* Enable I2C clock */
	Chip_Clock_EnablePeriphClock(SYSCON_CLOCK_I2C0);
	/* Peripheral reset control to I2C */
	Chip_SYSCON_PeriphReset(RESET_I2C0);
}

/**
 * @brief	Shutdown I2C Interface
 * @return	Nothing
 * @note	This function disables the I2C clock.
 */
STATIC INLINE void Chip_I2C0_DeInit(void)
{
	/* Disable I2C clock */
	Chip_Clock_DisablePeriphClock(SYSCON_CLOCK_I2C0);
}

/**
 * @brief	Initialize I2C Interface
 * @return	Nothing
 * @note	This function enables the I2C clock.
 */
STATIC INLINE void Chip_I2C1_Init(void)
{
	/* Enable I2C clock */
	Chip_Clock_EnablePeriphClock(SYSCON_CLOCK_I2C1);
	/* Peripheral reset control to I2C */
	Chip_SYSCON_PeriphReset(RESET_I2C1);
}

/**
 * @brief	Shutdown I2C Interface
 * @return	Nothing
 * @note	This function disables the I2C clock.
 */
STATIC INLINE void Chip_I2C1_DeInit(void)
{
	/* Disable I2C clock */
	Chip_Clock_DisablePeriphClock(SYSCON_CLOCK_I2C1);
}

/**
 * @brief	Initialize I2C Interface
 * @return	Nothing
 * @note	This function enables the I2C clock.
 */
STATIC INLINE void Chip_I2C2_Init(void)
{
	/* Enable I2C clock */
	Chip_Clock_EnablePeriphClock(SYSCON_CLOCK_I2C2);
	/* Peripheral reset control to I2C */
	Chip_SYSCON_PeriphReset(RESET_I2C2);
}

/**
 * @brief	Shutdown I2C Interface
 * @return	Nothing
 * @note	This function disables the I2C clock.
 */
STATIC INLINE void Chip_I2C2_DeInit(void)
{
	/* Disable I2C clock */
	Chip_Clock_DisablePeriphClock(SYSCON_CLOCK_I2C2);
}

/**
 * @brief	Initialize I2C Interface
 * @return	Nothing
 * @note	This function enables the I2C clock.
 */
STATIC INLINE void Chip_I2C3_Init(void)
{
	/* Enable I2C clock */
	Chip_Clock_EnablePeriphClock(SYSCON_CLOCK_I2C3);
	/* Peripheral reset control to I2C */
	Chip_SYSCON_PeriphReset(RESET_I2C3);
}

/**
 * @brief	Shutdown I2C Interface
 * @return	Nothing
 * @note	This function disables the I2C clock.
 */
STATIC INLINE void Chip_I2C3_DeInit(void)
{
	/* Disable I2C clock */
	Chip_Clock_DisablePeriphClock(SYSCON_CLOCK_I2C3);
}

/**
 * @brief	Get the current status of I2C controller
 * @param	pI2C	: The base of I2C peripheral on the chip
 * @return	I2C Status (Or-ed bit value of I2C_STAT_*)
 */
STATIC INLINE uint32_t Chip_I2C_GetStatus(LPC_I2C_T *pI2C)
{
	return pI2C->STAT;
}

/**
 * @brief	Get the current interrupt status of I2C controller
 * @param	pI2C	: The base of I2C peripheral on the chip
 * @return	I2C interrupt Status
 */
STATIC INLINE uint32_t Chip_I2C_GetIntStatus(LPC_I2C_T *pI2C)
{
	return pI2C->INTSTAT;
}

/**
 * @brief	Clear I2C status
 * @param	pI2C	: The base of I2C peripheral on the chip
 * @param	Flag	: Clear Flag (Or-ed bit value of I2C_STAT_CLR_*)
 * @return	Nothing
 */
STATIC INLINE void Chip_I2C_ClearStatus(LPC_I2C_T *pI2C, uint32_t Flag)
{
	pI2C->STAT |= Flag;
}

/**
 * @brief	Configure I2C Interface as master
 * @param	pI2C	: The base of I2C peripheral on the chip
 * @param	div	: I2C divider
 * @param	duty cycle	: duty cycle for master mode only
 * @return	Nothing
 * @note	This function Configure the I2C controller.
 */
STATIC INLINE void Chip_I2C_Master_Configure( LPC_I2C_T *pI2C, uint32_t div, uint32_t dutycycle )
{
	/* For master mode plus, if desired I2C clock is 1MHz (SCL high time + SCL low time). 
	If CCLK is 36MHz, MasterSclLow and MasterSclHigh are 0s, 
	SCL high time = (ClkDiv+1) * (MstSclHigh + 2 )
	SCL low time = (ClkDiv+1) * (MstSclLow + 2 )
	Pre-divider should be 8. 
	If fast mode, e.g. communicating with a temp sensor, Max I2C clock is set to 400KHz.
	Pre-divider should be 11. */
  pI2C->CLKDIV = div;
  pI2C->CFG &= ~I2C_CFG_MSTENA;

  pI2C->MSTTIME = I2C_TIM_MSTSCLLOW(dutycycle) | I2C_TIM_MSTSCLHIGH(dutycycle);
  pI2C->CFG |= I2C_CFG_MSTENA;
}

/**
 * @brief	Configure I2C Interface as slave
 * @param	pI2C	: The base of I2C peripheral on the chip
 * @param	div	: I2C divider
 * @param	addr	: Slave address 
 * @return	Nothing
 * @note	This function Configure the I2C controller.
 */
STATIC INLINE void Chip_I2C_Slave_Configure( LPC_I2C_T *pI2C, uint32_t div, uint32_t addr )
{
  pI2C->CLKDIV = div;
  pI2C->CFG &= ~I2C_CFG_SLVENA;
	/* Address qual mode and multiple slave masks to be implemented later. */
  pI2C->SLVADR0 = addr;
  pI2C->CFG |= I2C_CFG_SLVENA;
}

/**
 * @brief	Configure I2C Interface as monitor
 * @param	pI2C	: The base of I2C peripheral on the chip
 * @param	div	: I2C divider
 * @param	clkstr	: clock stretching
 * @return	Nothing
 * @note	This function Configure the I2C controller.
 */
STATIC INLINE void Chip_I2C_Monitor_Configure( LPC_I2C_T *pI2C, uint32_t div, uint32_t clkstr )
{
  pI2C->CLKDIV = div;
  pI2C->CFG &= ~(I2C_CFG_MONENA | I2C_CFG_MONCLKSTR);
  pI2C->CFG |= I2C_CFG_MONENA;
	if (clkstr) {
		pI2C->CFG |= I2C_CFG_MONCLKSTR;
	}
}

/**
 * @brief   Enable/Disable I2C interrupt
 * @param	pSPI			: The base I2C peripheral on the chip
 * @param	IntMask		: Interrupt mask
 * @param	NewState	: ENABLE or DISABLE interrupt
 * @return	Nothing
 */
STATIC INLINE void Chip_I2C_Int_Cmd(LPC_I2C_T *pI2C, uint32_t IntMask, FunctionalState NewState)
{
	if (NewState ==  ENABLE) {
		pI2C->INTENSET = IntMask;
	}
	else {
		pI2C->INTENCLR = IntMask;
	}
}

/**
 * @brief   Set Master Data
 * @param	pI2C			: The base I2C peripheral on the chip
 * @param	data  		: Data to set
 * @return	Nothing
 */
STATIC INLINE void Chip_I2C_SetMasterData(LPC_I2C_T *pI2C, uint32_t data)
{
  pI2C->MSTDAT = data;
}

/**
 * @brief   Get Master data
 * @param	pI2C			: The base I2C peripheral on the chip
 * @return	Master data 
 */
STATIC INLINE uint32_t Chip_I2C_GetMasterData(LPC_I2C_T *pI2C)
{
  return ( pI2C->MSTDAT );
}

/**
 * @brief   Enable/Disable I2C interrupt
 * @param	pSPI			: The base I2C peripheral on the chip
 * @param	IntMask		: Interrupt mask
 * @param	NewState	: ENABLE or DISABLE interrupt
 * @return	Nothing
 */
STATIC INLINE void Chip_I2C_SetMasterCtrl(LPC_I2C_T *pI2C, uint32_t ctrl)
{
  pI2C->MSTCTL = ctrl;
}

/**
 * @brief   Enable/Disable I2C interrupt
 * @param	pSPI			: The base I2C peripheral on the chip
 * @param	IntMask		: Interrupt mask
 * @param	NewState	: ENABLE or DISABLE interrupt
 * @return	Nothing
 */
STATIC INLINE void Chip_I2C_SetSlaveCtrl(LPC_I2C_T *pI2C, uint32_t ctrl)
{
  pI2C->SLVCTL = ctrl;
}

/**
 * @brief	Send Start Condition of I2C controller
 * @param	pI2C	: The base of I2C peripheral on the chip
 * @return	Nothing
 */
STATIC INLINE void Chip_I2C_SendStartWithAddr(LPC_I2C_T *pI2C, uint32_t addr )
{
  Chip_I2C_SetMasterData(pI2C, addr);
	Chip_I2C_SetMasterCtrl(pI2C, I2C_CTL_MSTSTART);
}

/**
 * @brief	Send Stop Condition of I2C controller
 * @param	pI2C	: The base of I2C peripheral on the chip
 * @return	Nothing
 */
STATIC INLINE void Chip_I2C_SendStop(LPC_I2C_T *pI2C)
{
	Chip_I2C_SetMasterCtrl(pI2C, I2C_CTL_MSTSTOP | I2C_CTL_MSTCONTINUE);
	while (!(Chip_I2C_GetStatus(pI2C) & I2C_STAT_MSTPEND));
	/* Should back to the IDLE mode once the master write is done. */
	while ( (Chip_I2C_GetStatus(pI2C) & (I2C_MASTER_STATE_MASK)) != (I2C_STAT_MSTIDLE) );
}

/**
 * @brief	Set CONTINUE bit to make Slave ready to TX/RX
 * @param	pI2C	: The base of I2C peripheral on the chip
 * @return	Nothing
 */
STATIC INLINE void Chip_I2C_SetSlaveReady(LPC_I2C_T *pI2C)
{
  pI2C->SLVCTL = I2C_CTL_SLVCONTINUE;
}

/**
 * @brief   I2C Polling Master send in blocking mode
 * @param	pI2C			: The base I2C peripheral on the chip
 * @param	pI2CSetup	:Pointer to a I2C_DATA_SETUP_T structure that contains specified
 *                          information about transmit/receive data	configuration
 * @return None
 * @note
 */
void Chip_I2C_Master_Send_Blocking( LPC_I2C_T *pI2C, I2C_DATA_SETUP_T *pI2CSetup );

/**
 * @brief   I2C Polling Master receive in blocking mode
 * @param	pI2C			: The base I2C peripheral on the chip
 * @param	pI2CSetup	:Pointer to a I2C_DATA_SETUP_T structure that contains specified
 *                          information about transmit/receive data	configuration
 * @return None
 * @note
 */
void Chip_I2C_Master_Receive_Blocking( LPC_I2C_T *pI2C, I2C_DATA_SETUP_T *pI2CSetup );

/**
 * @brief   I2C Polling Master send and receive in blocking mode
 * @param	pI2C			: The base I2C peripheral on the chip
 * @param	pI2CSetup	:Pointer to a I2C_DATA_SETUP_T structure that contains specified
 *                          information about transmit/receive data	configuration
 * @return None
 * @note
 */
void Chip_I2C_Master_Send_Receive_Blocking( LPC_I2C_T *pI2C, I2C_DATA_SETUP_T *pI2CSetup );

/**
 * @brief   I2C Polling Slave send in blocking mode
 * @param	pI2C			: The base I2C peripheral on the chip
 * @param	pI2CSetup	:Pointer to a I2C_DATA_SETUP_T structure that contains specified
 *                          information about transmit/receive data	configuration
 * @return None
 * @note
 */
void Chip_I2C_Slave_Send_Blocking( LPC_I2C_T *pI2C, I2C_DATA_SETUP_T *pI2CSetup );

/**
 * @brief   I2C Polling Slave receive in blocking mode
 * @param	pI2C			: The base I2C peripheral on the chip
 * @param	pI2CSetup	:Pointer to a I2C_DATA_SETUP_T structure that contains specified
 *                          information about transmit/receive data	configuration
 * @return None
 * @note
 */
void Chip_I2C_Slave_Receive_Blocking( LPC_I2C_T *pI2C, I2C_DATA_SETUP_T *pI2CSetup );

/**
 * @brief   I2C interrupt handler for both master and slave
 * @param	pI2C			: The base I2C peripheral on the chip
 * @param	pI2CSetup	:Pointer to a I2C_DATA_SETUP_T structure that contains specified
 *                          information about transmit/receive data	configuration
 * @return None
 * @note
 */
void Chip_I2C_IRQHandler(LPC_I2C_T *pI2C, I2C_DATA_SETUP_T *pI2CSetup );
/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __I2C_84X_H_ */
