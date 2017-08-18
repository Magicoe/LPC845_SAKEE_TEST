/*
 * @brief LPC8xx UART driver
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2012
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

#ifndef __UART_84X_H_
#define __UART_84X_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "ring_buffer.h"

/** @defgroup UART_84X CHIP: LPC8xx UART Driver
 * @ingroup CHIP_84X_Drivers
 * @{
 */

/**
 * @brief UART register block structure
 */
typedef struct {
	__IO uint32_t  CFG;				/*!< Configuration register */
	__IO uint32_t  CTRL;			/*!< Control register */
	__IO uint32_t  STAT;			/*!< Status register */
	__IO uint32_t  INTENSET;		/*!< Interrupt Enable read and set register */
	__O  uint32_t  INTENCLR;		/*!< Interrupt Enable clear register */
	__I  uint32_t  RXDATA;			/*!< Receive Data register */
	__I  uint32_t  RXDATA_STAT;		/*!< Receive Data with status register */
	__IO uint32_t  TXDATA;			/*!< Transmit data register */
	__IO uint32_t  BRG;				/*!< Baud Rate Generator register */
	__IO uint32_t  INTSTAT;			/*!< Interrupt status register */
	__IO uint32_t  OSR;				/*!< Oversample rate selection register */
	__IO uint32_t  ADDR;				/*!< Address register for automatic address matching */
} LPC_USART_T;

/**
 * @brief UART CFG register definitions
 */
#define UART_CFG_ENABLE_P				0
#define UART_CFG_ENABLE         (0x01 << 0)
#define UART_CFG_DATALEN_P			2							/*!< UART bit length mode */
#define UART_CFG_DATALEN_M			(0x3UL << UART_CFG_DATALEN_P)/*!< UART bit length mode */
#define UART_CFG_DATALEN_SET(value)	(((uint32_t)(value) << UART_CFG_DATALEN_P) & UART_CFG_DATALEN_M)
#define UART_CFG_DATALEN_7      (0x00 << UART_CFG_DATALEN_P)		/*!< UART 7 bit length mode */
#define UART_CFG_DATALEN_8      (0x01 << UART_CFG_DATALEN_P)		/*!< UART 8 bit length mode */
#define UART_CFG_DATALEN_9      (0x02 << UART_CFG_DATALEN_P)		/*!< UART 9 bit length mode */
#define UART_CFG_PARITY_P				4							/*!< UART parity */
#define UART_CFG_PARITY_M				(0x3UL << UART_CFG_PARITY_P)		/*!< UART parity */
#define UART_CFG_PARITY_SET(value)	(((uint32_t)(value) << UART_CFG_PARITY_P) & UART_CFG_PARITY_M)
#define UART_CFG_PARITY_NONE    (0x00 << UART_CFG_PARITY_P)		/*!< No parity */
#define UART_CFG_PARITY_EVEN    (0x02 << UART_CFG_PARITY_P)		/*!< Even parity */
#define UART_CFG_PARITY_ODD     (0x03 << UART_CFG_PARITY_P)		/*!< Odd parity */
#define UART_CFG_STOPLEN_P			6							/*!< UART stop bits */
#define UART_CFG_STOPLEN_M			(0x3UL << UART_CFG_STOPLEN_P)		/*!< UART stop bits */
#define UART_CFG_STOPLEN_SET(value)	(((uint32_t)(value) << UART_CFG_STOPLEN_P) & UART_CFG_STOPLEN_M)
#define UART_CFG_STOPLEN_1      (0x00 << 6)		/*!< UART One Stop Bit Select */
#define UART_CFG_STOPLEN_2      (0x01 << 6)		/*!< UART Two Stop Bits Select */
#define UART_CFG_CTSEN_P				9			/*!< CTS enable bit */
#define UART_CFG_CTSEN          (0x01 << 9)		/*!< CTS enable bit */
#define UART_CFG_SYNCEN_P				11		/*!< Synchronous mode enable bit */
#define UART_CFG_SYNCEN         (0x01 << 11)	/*!< Synchronous mode enable bit */
#define UART_CFG_CLKPOL_P				12		/*!< Un_RXD rising edge sample enable bit */
#define UART_CFG_CLKPOL         (0x01 << 12)	/*!< Un_RXD rising edge sample enable bit */
#define UART_CFG_SYNCMST_P			14		/*!< Select master mode (synchronous mode) enable bit */
#define UART_CFG_SYNCMST        (0x01 << 14)	/*!< Select master mode (synchronous mode) enable bit */
#define UART_CFG_LOOP_P					15		/*!< Loopback mode enable bit */
#define UART_CFG_LOOP           (0x01 << 15)	/*!< Loopback mode enable bit */
#define UART_CFG_OETA_P					18		/*!< Output Enable Turnaround time enable for RS-485 operation. */
#define UART_CFG_OETA						(0x01 << 18)	/*!< Output Enable Turnaround time enable for RS-485 operation. */
#define UART_CFG_AUTOADDR_P			19		/*!< Automatic Address matching enable */
#define UART_CFG_AUTOADDR				(0x01 << 19)	/*!< Automatic Address matching enable */
#define UART_CFG_OESEL_P				20		/*!< Output Enable Select */
#define UART_CFG_OESEL					(0x01 << 20)	/*!< Output Enable Select */
#define UART_CFG_OEPOL_P				21		/*!< Output Enable Polarity high */
#define UART_CFG_OEPOL					(0x01 << 21)	/*!< Output Enable Polarity high */
#define UART_CFG_RXPOL_P				22		/*!< Receive data polarity inverted */
#define UART_CFG_RXPOL					(0x01 << 22)	/*!< Receive data polarity inverted */
#define UART_CFG_TXPOL_P				23		/*!< Transmit data polarity inverted */
#define UART_CFG_TXPOL					(0x01 << 23)	/*!< Transmit data polarity inverted */

/**
 * @brief UART CTRL register definitions
 */
#define UART_CTRL_TXBRKEN_P			1			/*!< Continuous break enable bit */
#define UART_CTRL_TXBRKEN       (0x01 << 1)		/*!< Continuous break enable bit */
#define UART_CTRL_ADDRDET_P			2			/*!< Address detect mode enable bit */
#define UART_CTRL_ADDRDET       (0x01 << 2)		/*!< Address detect mode enable bit */
#define UART_CTRL_TXDIS_P				6			/*!< Transmit disable bit */
#define UART_CTRL_TXDIS         (0x01 << 6)		/*!< Transmit disable bit */
#define UART_CTRL_CC_P					8			/*!< Continuous Clock mode enable bit */
#define UART_CTRL_CC            (0x01 << 8)		/*!< Continuous Clock mode enable bit */
#define UART_CTRL_CLRCCONRX_P		9			/*!< Clear Continuous Clock bit when complete character received */
#define UART_CTRL_CLRCCONRX     (0x01 << 9)		/*!< Clear Continuous Clock bit when complete character received */
#define UART_CTRL_AUTOBAUD_P		16		/*!< Autobaud enable */
#define UART_CTRL_AUTOBAUD			(0x01 << 16)	/*!< Autobaud enable */

/**
 * @brief UART STAT register definitions
 */
#define UART_STAT_RXRDY         (0x01 << 0)			/*!< Receiver ready */
#define UART_STAT_RXIDLE        (0x01 << 1)			/*!< Receiver idle */
#define UART_STAT_TXRDY         (0x01 << 2)			/*!< Transmitter ready for data */
#define UART_STAT_TXIDLE        (0x01 << 3)			/*!< Transmitter idle */
#define UART_STAT_CTS           (0x01 << 4)			/*!< Status of CTS signal */
#define UART_STAT_DELTACTS      (0x01 << 5)			/*!< Change in CTS state */
#define UART_STAT_TXDISINT      (0x01 << 6)			/*!< Transmitter disabled */
#define UART_STAT_OVERRUNINT    (0x01 << 8)			/*!< Overrun Error interrupt flag. */
#define UART_STAT_RXBRK         (0x01 << 10)		/*!< Received break */
#define UART_STAT_DELTARXBRK    (0x01 << 11)		/*!< Change in receive break detection */
#define UART_STAT_START         (0x01 << 12)		/*!< Start detected */
#define UART_STAT_FRAMERRINT    (0x01 << 13)		/*!< Framing Error interrupt flag */
#define UART_STAT_PARITYERRINT	(0x01 << 14)		/*!< Parity Error interrupt flag */
#define UART_STAT_RXNOISEINT    (0x01 << 15)		/*!< Received Noise interrupt flag */
#define UART_STAT_ABERR    			(0x01 << 16)		/*!< Auto-baud Error */

/**
 * @brief UART INTENSET/INTENCLR register definitions
 */
// Can use UART_INT_.. for INTENSET, INTENCLR and INTSTAT
#define UART_INT_RXRDY        	(0x01 << 0)			/*!< Receive Ready interrupt */
#define UART_INT_TXRDY        	(0x01 << 2)			/*!< Transmit Ready interrupt */
#define UART_INT_TXIDLE					(0x01 << 3)			/*!< Transmit Idle interrupt */
#define UART_INT_DELTACTS     	(0x01 << 5)			/*!< Change in CTS state interrupt */
#define UART_INT_TXDIS        	(0x01 << 6)			/*!< Transmitter disable interrupt */
#define UART_INT_OVERRUN      	(0x01 << 8)			/*!< Overrun error interrupt */
#define UART_INT_DELTARXBRK   	(0x01 << 11)		/*!< Change in receiver break detection interrupt */
#define UART_INT_START        	(0x01 << 12)		/*!< Start detect interrupt */
#define UART_INT_FRAMERR      	(0x01 << 13)		/*!< Frame error interrupt */
#define UART_INT_PARITYERR    	(0x01 << 14)		/*!< Parity error interrupt */
#define UART_INT_RXNOISE      	(0x01 << 15)		/*!< Received noise interrupt */
#define UART_INT_ABERR					(0x01 << 16)		/*!< Auto-baud error interrupt */
#define UART_INT_ALL						(UART_INT_RXRDY | UART_INT_TXRDY | UART_INT_TXIDLE		\
																	| UART_INT_DELTACTS | UART_INT_TXDIS | UART_INT_OVERRUN			\
																	| UART_INT_DELTARXBRK | UART_INT_START | UART_INT_FRAMERR		\
																	| UART_INT_PARITYERR | UART_INT_RXNOISE | UART_INT_ABERR)

#define UART_INTEN_RXRDY        (0x01 << 0)			/*!< Receive Ready interrupt */
#define UART_INTEN_TXRDY        (0x01 << 2)			/*!< Transmit Ready interrupt */
#define UART_INTEN_TXIDLE				(0x01 << 3)			/*!< Transmit Idle interrupt */
#define UART_INTEN_DELTACTS     (0x01 << 5)			/*!< Change in CTS state interrupt */
#define UART_INTEN_TXDIS        (0x01 << 6)			/*!< Transmitter disable interrupt */
#define UART_INTEN_OVERRUN      (0x01 << 8)			/*!< Overrun error interrupt */
#define UART_INTEN_DELTARXBRK   (0x01 << 11)		/*!< Change in receiver break detection interrupt */
#define UART_INTEN_START        (0x01 << 12)		/*!< Start detect interrupt */
#define UART_INTEN_FRAMERR      (0x01 << 13)		/*!< Frame error interrupt */
#define UART_INTEN_PARITYERR    (0x01 << 14)		/*!< Parity error interrupt */
#define UART_INTEN_RXNOISE      (0x01 << 15)		/*!< Received noise interrupt */
#define UART_INTEN_ABERR				(0x01 << 16)		/*!< Auto-baud error interrupt */
#define UART_INTEN_ALL					(UART_INTEN_RXRDY | UART_INTEN_TXRDY | UART_INTEN_TXIDLE		\
																	| UART_INTEN_DELTACTS | UART_INTEN_TXDIS | UART_INTEN_OVERRUN			\
																	| UART_INTEN_DELTARXBRK | UART_INTEN_START | UART_INTEN_FRAMERR		\
																	| UART_INTEN_PARITYERR | UART_INTEN_RXNOISE | UART_INTEN_ABERR)

#define UART_RXDATSTAT_RXDAT_M	(0x1FF << 0)			/*!< Rx Data */
#define UART_RXDATSTAT_FRAMERR	(0x01 << 13)			/*!< Framing Error status flag */
#define UART_RXDATSTAT_PARITYERR	(0x01 << 14)			/*!< Parity Error status flag */
#define UART_RXDATSTAT_RXNOISE	(0x01 << 15)			/*!< Received Noise flag */

#define UART_TXDAT_TXDAT_M	(0x1FFUL << 0)			/*!< Tx Data */

#define UART_BRG_BRGVAL			(0x0FFFFUL << 0)		/*!< Baud Rate Generator */

#define UART_OSR_OSRVAL_P			0				/*!< Oversample rate - 1 */
#define UART_OSR_OSRVAL_M			(0xFUL << UART_OSR_OSRVAL_P)				/*!< Oversample rate - 1 */
#define UART_OSR_OSRVAL_SET(value)	(((uint32_t)(value) << UART_OSR_OSRVAL_P) & UART_OSR_OSRVAL_M)

/**
 * @brief	Returns the associated USART index
 * @param	pUART		: Pointer to selected UARTx peripheral
 * @return	USART index
 */
uint32_t Chip_UART_USARTIndex(LPC_USART_T *pUSART);

/**
 * @brief	Returns the associated USART register base address
 * @param	pUART		: USART index
 * @return	USART register base address
 */
LPC_USART_T *Chip_USART_USARTRegBase(uint32_t usartNum);

/**
 * @brief	Get the USARTs LPCxxx IRQ handler number
 * @param	usartNum	: The USART index
 * @return	USARTs NVIC IRQ handler number
 */
IRQn_Type Chip_USART_IRQn(uint32_t usartNum);

/**
 * @brief	Enable the UART
 * @param	pUART		: Pointer to selected UARTx peripheral
 * @return	Nothing
 */
STATIC INLINE void Chip_UART_Enable(LPC_USART_T *pUART)
{
	pUART->CFG |= UART_CFG_ENABLE;
}

/**
 * @brief	Disable the UART
 * @param	pUART	: Pointer to selected UARTx peripheral
 * @return	Nothing
 */
STATIC INLINE void Chip_UART_Disable(LPC_USART_T *pUART)
{
	pUART->CFG &= ~UART_CFG_ENABLE;
}

/**
 * @brief	Enable transmission on UART TxD pin
 * @param	pUART	: Pointer to selected pUART peripheral
 * @return Nothing
 */
STATIC INLINE void Chip_UART_TXEnable(LPC_USART_T *pUART)
{
	pUART->CTRL &= ~UART_CTRL_TXDIS;
}

/**
 * @brief	Disable transmission on UART TxD pin
 * @param	pUART	: Pointer to selected pUART peripheral
 * @return Nothing
 */
STATIC INLINE void Chip_UART_TXDisable(LPC_USART_T *pUART)
{
	pUART->CTRL |= UART_CTRL_TXDIS;
}

/**
 * @brief	Transmit a single data byte through the UART peripheral
 * @param	pUART	: Pointer to selected UART peripheral
 * @param	data	: Byte to transmit
 * @return	Nothing
 * @note	This function attempts to place a byte into the UART transmit
 *			holding register regard regardless of UART state.
 */
STATIC INLINE void Chip_UART_SendByte(LPC_USART_T *pUART, uint8_t data)
{
	pUART->TXDATA = (uint32_t) data;
}

/**
 * @brief	Read a single byte data from the UART peripheral
 * @param	pUART	: Pointer to selected UART peripheral
 * @return	A single byte of data read
 * @note	This function reads a byte from the UART receive FIFO or
 *			receive hold register regard regardless of UART state. The
 *			FIFO status should be read first prior to using this function
 */
STATIC INLINE uint32_t Chip_UART_ReadByte(LPC_USART_T *pUART)
{
	/* Strip off undefined reserved bits, keep 9 lower bits */
	return (uint32_t) (pUART->RXDATA & UART_RXDATSTAT_RXDAT_M);
}

/**
 * @brief	Enable UART interrupts
 * @param	pUART	: Pointer to selected UART peripheral
 * @param	intMask	: OR'ed Interrupts to enable
 * @return	Nothing
 * @note	Use an OR'ed value of UART_INT_* definitions with this function
 *			to enable specific UART interrupts.
 */
STATIC INLINE void Chip_UART_IntEnable(LPC_USART_T *pUART, uint32_t intMask)
{
	pUART->INTENSET = intMask;
}

/**
 * @brief	Disable UART interrupts
 * @param	pUART	: Pointer to selected UART peripheral
 * @param	intMask	: OR'ed Interrupts to disable
 * @return	Nothing
 * @note	Use an OR'ed value of UART_INT_* definitions with this function
 *			to disable specific UART interrupts.
 */
STATIC INLINE void Chip_UART_IntDisable(LPC_USART_T *pUART, uint32_t intMask)
{
	pUART->INTENCLR = intMask;
}

/**
 * @brief	Returns UART interrupts that are enabled
 * @param	pUART	: Pointer to selected UART peripheral
 * @return	Returns the enabled UART interrupts
 * @note	Use an OR'ed value of UART_INT_* definitions with this function
 *			to determine which interrupts are enabled. You can check
 *			for multiple enabled bits if needed.
 */
STATIC INLINE uint32_t Chip_UART_GetIntsEnabled(LPC_USART_T *pUART)
{
	return pUART->INTENSET;
}

/**
 * @brief	Get UART interrupt status
 * @param	pUART	: The base of UART peripheral on the chip
 * @return	The Interrupt status register of UART
 * @note	Multiple interrupts may be pending. Mask the return value
 *			with one or more UART_INT_* definitions to determine
 *			pending interrupts.
 */
STATIC INLINE uint32_t Chip_UART_GetIntStatus(LPC_USART_T *pUART)
{
	return pUART->INTSTAT;
}

/**
 * @brief	Configure data width, parity and stop bits
 * @param	pUART	: Pointer to selected pUART peripheral
 * @param	config	: UART configuration, OR'ed values of select UART_CFG_* defines
 * @return	Nothing
 * @note	Select OR'ed config options for the UART from the UART_CFG_PARITY_*,
 *			UART_CFG_STOPLEN_*, and UART_CFG_DATALEN_* definitions. For example,
 *			a configuration of 8 data bits, 1 stop bit, and even (enabled) parity would be
 *			(UART_CFG_DATALEN_8 | UART_CFG_STOPLEN_1 | UART_CFG_PARITY_EVEN). Will not
 *			alter other bits in the CFG register.
 */
STATIC INLINE void Chip_UART_ConfigData(LPC_USART_T *pUART, uint32_t config)
{
	uint32_t reg;

	reg = pUART->CFG & ~(UART_CFG_DATALEN_M | UART_CFG_PARITY_M | UART_CFG_STOPLEN_M);
	pUART->CFG = reg | config;
}

/**
 * @brief	Get the UART status register
 * @param	pUART	: Pointer to selected UARTx peripheral
 * @return	UART status register
 * @note	Multiple statuses may be pending. Mask the return value
 *			with one or more UART_STAT_* definitions to determine
 *			statuses.
 */
STATIC INLINE uint32_t Chip_UART_GetStatus(LPC_USART_T *pUART)
{
	return pUART->STAT;
}

/**
 * @brief	Clear the UART status register
 * @param	pUART	: Pointer to selected UARTx peripheral
 * @param	stsMask	: OR'ed statuses to disable
 * @return	Nothing
 * @note	Multiple interrupts may be pending. Mask the return value
 *			with one or more UART_INT_* definitions to determine
 *			pending interrupts.
 */
STATIC INLINE void Chip_UART_ClearStatus(LPC_USART_T *pUART, uint32_t stsMask)
{
	pUART->STAT = stsMask;
}

/**
 * @brief	Initialize the UART peripheral
 * @param	pUART	: The base of UART peripheral on the chip
 * @return	Nothing
 */
void Chip_UART_Init(LPC_USART_T *pUART);

/**
 * @brief	Deinitialize the UART peripheral
 * @param	pUART	: The base of UART peripheral on the chip
 * @return	Nothing
 */
void Chip_UART_DeInit(LPC_USART_T *pUART);

/**
 * @brief	Transmit a byte array through the UART peripheral (non-blocking)
 * @param	pUART		: Pointer to selected UART peripheral
 * @param	data		: Pointer to bytes to transmit
 * @param	numBytes	: Number of bytes to transmit
 * @return	The actual number of bytes placed into the FIFO
 * @note	This function places data into the transmit FIFO until either
 *			all the data is in the FIFO or the FIFO is full. This function
 *			will not block in the FIFO is full. The actual number of bytes
 *			placed into the FIFO is returned. This function ignores errors.
 */
int Chip_UART_Send(LPC_USART_T *pUART, const void *data, int numBytes);

/**
 * @brief	Read data through the UART peripheral (non-blocking)
 * @param	pUART		: Pointer to selected UART peripheral
 * @param	data		: Pointer to bytes array to fill
 * @param	numBytes	: Size of the passed data array
 * @return	The actual number of bytes read
 * @note	This function reads data from the receive FIFO until either
 *			all the data has been read or the passed buffer is completely full.
 *			This function will not block. This function ignores errors.
 */
int Chip_UART_Read(LPC_USART_T *pUART, void *data, int numBytes);

/**
 * @brief	Set baud rate for UART
 * @param	pUART	: The base of UART peripheral on the chip
 * @param	baudrate: Baud rate to be set
 * @return	Nothing
 */
void Chip_UART_SetBaud(uint8_t frg_num, LPC_USART_T *pUART, uint32_t baudrate);

/**
 * @brief	Set baud rate for UART
 * @param	pUART	: The base of UART peripheral on the chip
 * @param	baudrate: Baud rate to be set
 * @param	oversampleRate: UART sampling rate (set to 1 for sync mode)
 * @return	Nothing
 */
void Chip_UART_SetBaudrate(uint8_t frg_num, LPC_USART_T *pUART, uint32_t baudrate, uint32_t oversampleRate);

/**
 * @brief	Transmit a byte array through the UART peripheral (blocking)
 * @param	pUART		: Pointer to selected UART peripheral
 * @param	data		: Pointer to data to transmit
 * @param	numBytes	: Number of bytes to transmit
 * @return	The number of bytes transmitted
 * @note	This function will send or place all bytes into the transmit
 *			FIFO. This function will block until the last bytes are in the FIFO.
 */
int Chip_UART_SendBlocking(LPC_USART_T *pUART, const void *data, int numBytes);

/**
 * @brief	Read data through the UART peripheral (blocking)
 * @param	pUART		: Pointer to selected UART peripheral
 * @param	data		: Pointer to data array to fill
 * @param	numBytes	: Size of the passed data array
 * @return	The size of the dat array
 * @note	This function reads data from the receive FIFO until the passed
 *			buffer is completely full. The function will block until full.
 *			This function ignores errors.
 */
int Chip_UART_ReadBlocking(LPC_USART_T *pUART, void *data, int numBytes);

/**
 * @brief	UART receive-only interrupt handler for ring buffers
 * @param	pUART	: Pointer to selected UART peripheral
 * @param	pRB		: Pointer to ring buffer structure to use
 * @return	Nothing
 * @note	If ring buffer support is desired for the receive side
 *			of data transfer, the UART interrupt should call this
 *			function for a receive based interrupt status.
 */
void Chip_UART_RXIntHandlerRB(LPC_USART_T *pUART, RINGBUFF_T *pRB);

/**
 * @brief	UART transmit-only interrupt handler for ring buffers
 * @param	pUART	: Pointer to selected UART peripheral
 * @param	pRB		: Pointer to ring buffer structure to use
 * @return	Nothing
 * @note	If ring buffer support is desired for the transmit side
 *			of data transfer, the UART interrupt should call this
 *			function for a transmit based interrupt status.
 */
void Chip_UART_TXIntHandlerRB(LPC_USART_T *pUART, RINGBUFF_T *pRB);

/**
 * @brief	Populate a transmit ring buffer and start UART transmit
 * @param	pUART	: Pointer to selected UART peripheral
 * @param	pRB		: Pointer to ring buffer structure to use
 * @param	data	: Pointer to buffer to move to ring buffer
 * @param	count	: Number of bytes to move
 * @return	The number of bytes placed into the ring buffer
 * @note	Will move the data into the TX ring buffer and start the
 *			transfer. If the number of bytes returned is less than the
 *			number of bytes to send, the ring buffer is considered full.
 */
uint32_t Chip_UART_SendRB(LPC_USART_T *pUART, RINGBUFF_T *pRB, const void *data, int count);

/**
 * @brief	Copy data from a receive ring buffer
 * @param	pUART	: Pointer to selected UART peripheral
 * @param	pRB		: Pointer to ring buffer structure to use
 * @param	data	: Pointer to buffer to fill from ring buffer
 * @param	bytes	: Size of the passed buffer in bytes
 * @return	The number of bytes placed into the ring buffer
 * @note	Will move the data from the RX ring buffer up to the
 *			the maximum passed buffer size. Returns 0 if there is
 *			no data in the ring buffer.
 */
int Chip_UART_ReadRB(LPC_USART_T *pUART, RINGBUFF_T *pRB, void *data, int bytes);

/**
 * @brief	UART receive/transmit interrupt handler for ring buffers
 * @param	pUART	: Pointer to selected UART peripheral
 * @param	pRXRB	: Pointer to transmit ring buffer
 * @param	pTXRB	: Pointer to receive ring buffer
 * @return	Nothing
 * @note	This provides a basic implementation of the UART IRQ
 *			handler for support of a ring buffer implementation for
 *			transmit and receive.
 */
void Chip_UART_IRQRBHandler(LPC_USART_T *pUART, RINGBUFF_T *pRXRB, RINGBUFF_T *pTXRB);

//******************************************************************************
//
//		DMA Stuff
//
//******************************************************************************

/**
 * @brief	Returns the associated USART index.
 * @param	pUSART		: Pointer to selected UART peripheral
 * @return	The associated USART index
 */
uint32_t Chip_USART_DMAChanToUSARTIndex(DMA_CHID_T pUSARTDMAChan);

/**
 * @brief	Returns the associated TX DMA channel number.
 * @param	pUSART		: Pointer to selected UART peripheral
 * @return	The associated DMA channel number
 */
DMA_CHID_T Chip_USART_DMA_TxChan(LPC_USART_T *pUSART);

/**
 * @brief	Returns the associated RX DMA channel number.
 * @param	pUSART		: Pointer to selected UART peripheral
 * @return	The associated DMA channel number
 */
DMA_CHID_T Chip_USART_DMA_RxChan(LPC_USART_T *pUSART);

/**
 * @brief	Returns the USART register base address associated with the DMA channel.
 * @param	The associated DMA channel numberp
 * @return	LPC_USART_T *: Pointer to selected UART peripheral
 */
LPC_USART_T *Chip_USART_DMAChanToRegBase(DMA_CHID_T dmaChannel);

/**
 * @brief	Returns the associated RX DMA channel number.
 * @param	pUSART		: Pointer to selected UART peripheral
 * @return	The associated DMA channel number
 */
void Chip_USART_Tx_DMA(LPC_USART_T *pUSART, uint8_t *pBuf, uint32_t bufLen, uint8_t width, uint8_t enableCh, uint8_t enableChInt);
void Chip_USART_Rx_DMA(LPC_USART_T *pUSART, uint8_t *pBuf, uint32_t bufLen, uint8_t width, uint8_t enableCh, uint8_t enableChInt);


/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __UART_84X_H_ */
