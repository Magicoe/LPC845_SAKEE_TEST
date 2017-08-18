/*
 * @brief LPC84X UART driver
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2012
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

//******************************************************************************
// Returns the associated USART index.
//******************************************************************************
uint32_t Chip_USART_USARTIndex(LPC_USART_T *pUSART)
{
	if (pUSART == LPC_USART0)
		return 0;
	if (pUSART == LPC_USART1)
		return 1;
	if (pUSART == LPC_USART2)
		return 2;
	if (pUSART == LPC_USART3)
		return 3;
	if (pUSART == LPC_USART4)
		return 4;
	else
		while(1);
}

//******************************************************************************
// Returns the USART register base address.
//******************************************************************************
LPC_USART_T *Chip_USART_USARTRegBase(uint32_t usartNum)
{
	static const LPC_USART_T *usartRegBaseTable[] = {
		LPC_USART0,
		LPC_USART1,
		LPC_USART2,
		LPC_USART3,
		LPC_USART4
	};

	if (usartNum >= (sizeof(usartRegBaseTable)/sizeof(usartRegBaseTable[0])))
		while(1);
	return (LPC_USART_T *)usartRegBaseTable[usartNum];
}


/* Return UART clock ID from the UART register address */
static CHIP_SYSCON_CLOCK_T Chip_USART_GetUARTClockID(LPC_USART_T *pUSART)
{
	static const CHIP_SYSCON_CLOCK_T usartClockTable[] = {
		SYSCON_CLOCK_UART0,
		SYSCON_CLOCK_UART1,
		SYSCON_CLOCK_UART2,
		SYSCON_CLOCK_UART3,
		SYSCON_CLOCK_UART4
	};

	return usartClockTable[Chip_USART_USARTIndex(pUSART)];
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/


/* Initialize the UART peripheral */
void Chip_UART_Init(LPC_USART_T *pUSART)
{
	static const CHIP_SYSCON_PERIPH_RESET_T  usartPeriphResetTable[] = {
		RESET_UART0,
		RESET_UART1,
		RESET_UART2,
		RESET_UART3,
		RESET_UART4
	};

	/* Enable USART clock */
	Chip_Clock_EnablePeriphClock(Chip_USART_GetUARTClockID(pUSART));

	/* UART reset */
	Chip_SYSCON_PeriphReset(usartPeriphResetTable[Chip_USART_USARTIndex(pUSART)]);
}

/* Initialize the UART peripheral */
void Chip_UART_DeInit(LPC_USART_T *pUART)
{
	/* Disable USART clock */
	Chip_Clock_DisablePeriphClock(Chip_USART_GetUARTClockID(pUART));
}

/* Transmit a byte array through the UART peripheral (non-blocking) */
int Chip_UART_Send(LPC_USART_T *pUART, const void *data, int numBytes)
{
	int sent = 0;
	uint8_t *p8 = (uint8_t *) data;

	/* Send until the transmit FIFO is full or out of bytes */
	while ((sent < numBytes) &&
		   ((Chip_UART_GetStatus(pUART) & UART_STAT_TXRDY) != 0)) {
		Chip_UART_SendByte(pUART, *p8);
		p8++;
		sent++;
	}

	return sent;
}

/* Transmit a byte array through the UART peripheral (blocking) */
int Chip_UART_SendBlocking(LPC_USART_T *pUART, const void *data, int numBytes)
{
	int pass, sent = 0;
	uint8_t *p8 = (uint8_t *) data;

	while (numBytes > 0) {
		pass = Chip_UART_Send(pUART, p8, numBytes);
		numBytes -= pass;
		sent += pass;
		p8 += pass;
	}

	return sent;
}

/* Read data through the UART peripheral (non-blocking) */
int Chip_UART_Read(LPC_USART_T *pUART, void *data, int numBytes)
{
	int readBytes = 0;
	uint8_t *p8 = (uint8_t *) data;

	/* Send until the transmit FIFO is full or out of bytes */
	while ((readBytes < numBytes) &&
		   ((Chip_UART_GetStatus(pUART) & UART_STAT_RXRDY) != 0)) {
		*p8 = Chip_UART_ReadByte(pUART);
		p8++;
		readBytes++;
	}

	return readBytes;
}

/* Read data through the UART peripheral (blocking) */
int Chip_UART_ReadBlocking(LPC_USART_T *pUART, void *data, int numBytes)
{
	int pass, readBytes = 0;
	uint8_t *p8 = (uint8_t *) data;

	while (readBytes < numBytes) {
		pass = Chip_UART_Read(pUART, p8, numBytes);
		numBytes -= pass;
		readBytes += pass;
		p8 += pass;
	}

	return readBytes;
}

/* Set baud rate for UART */
void Chip_UART_SetBaud(uint8_t frg_num, LPC_USART_T *pUART, uint32_t baudrate)
{
	Chip_UART_SetBaudrate(frg_num, pUART, baudrate, 16);
}

/* Set baud rate for UART */
void Chip_UART_SetBaudrate(uint8_t frg_num, LPC_USART_T *pUART, uint32_t baudrate, uint32_t oversampleRate)
{
	uint32_t err, uart_fra_multiplier, baudRateGenerator;
	uint32_t mainClock = Chip_Clock_GetMainClockPLLRate();

	/* Calculate baudrate generator value */
	baudRateGenerator = mainClock / (oversampleRate * baudrate);
	if (baudRateGenerator == 0)
		baudRateGenerator = 1;
	err = mainClock - baudRateGenerator * oversampleRate * baudrate;
	uart_fra_multiplier = (err * 0xFF) / (baudRateGenerator * oversampleRate * baudrate);
	pUART->BRG = baudRateGenerator - 1;	/* baud rate */
	Chip_SYSCON_SetUSARTFRGDivider(frg_num, 0xFF);	/* value 0xFF is always used */
	Chip_SYSCON_SetUSARTFRGMultiplier(frg_num, uart_fra_multiplier);
}

/* UART receive-only interrupt handler for ring buffers */
void Chip_UART_RXIntHandlerRB(LPC_USART_T *pUART, RINGBUFF_T *pRB)
{
	/* New data will be ignored if data not popped in time */
	while ((Chip_UART_GetStatus(pUART) & UART_STAT_RXRDY) != 0) {
		uint8_t ch = Chip_UART_ReadByte(pUART);
		RingBuffer_Insert(pRB, &ch);
	}
}

/* UART transmit-only interrupt handler for ring buffers */
void Chip_UART_TXIntHandlerRB(LPC_USART_T *pUART, RINGBUFF_T *pRB)
{
	uint8_t ch;

	/* Fill FIFO until full or until TX ring buffer is empty */
	while (((Chip_UART_GetStatus(pUART) & UART_STAT_TXRDY) != 0) &&
		   RingBuffer_Pop(pRB, &ch)) {
		Chip_UART_SendByte(pUART, ch);
	}
}

/* Populate a transmit ring buffer and start UART transmit */
uint32_t Chip_UART_SendRB(LPC_USART_T *pUART, RINGBUFF_T *pRB, const void *data, int count)
{
	uint32_t ret;
	uint8_t *p8 = (uint8_t *) data;

	/* Don't let UART transmit ring buffer change in the UART IRQ handler */
	Chip_UART_IntDisable(pUART, UART_INTEN_TXRDY);

	/* Move as much data as possible into transmit ring buffer */
	ret = RingBuffer_InsertMult(pRB, p8, count);
	Chip_UART_TXIntHandlerRB(pUART, pRB);

	/* Add additional data to transmit ring buffer if possible */
	ret += RingBuffer_InsertMult(pRB, (p8 + ret), (count - ret));

	/* Enable UART transmit interrupt */
	Chip_UART_IntEnable(pUART, UART_INTEN_TXRDY);

	return ret;
}

/* Copy data from a receive ring buffer */
int Chip_UART_ReadRB(LPC_USART_T *pUART, RINGBUFF_T *pRB, void *data, int bytes)
{
	(void) pUART;

	return RingBuffer_PopMult(pRB, (uint8_t *) data, bytes);
}

/* UART receive/transmit interrupt handler for ring buffers */
void Chip_UART_IRQRBHandler(LPC_USART_T *pUART, RINGBUFF_T *pRXRB, RINGBUFF_T *pTXRB)
{
	/* Handle transmit interrupt if enabled */
	if ((Chip_UART_GetStatus(pUART) & UART_STAT_TXRDY) != 0) {
		Chip_UART_TXIntHandlerRB(pUART, pTXRB);

		/* Disable transmit interrupt if the ring buffer is empty */
		if (RingBuffer_IsEmpty(pTXRB)) {
			Chip_UART_IntDisable(pUART, UART_INTEN_TXRDY);
		}
	}
	if ((Chip_UART_GetStatus(pUART) & UART_STAT_DELTACTS) != 0) {
//		Chip_UART_IntDisable(pUART, UART_INTEN_DELTACTS);
		Chip_UART_ClearStatus(pUART, UART_STAT_DELTACTS);
		// Flag that CTS has changed.
	}
	
	/* Handle receive interrupt */
	Chip_UART_RXIntHandlerRB(pUART, pRXRB);
}


//******************************************************************************
//		UART DMA Stuff
//******************************************************************************

//******************************************************************************
// Returns the associated uart index.
//******************************************************************************
uint32_t Chip_USART_DMAChanToUSARTIndex(DMA_CHID_T usartDMAChan)
{
	switch (usartDMAChan)
	{
		case DMAREQ_USART0_RX:
		case DMAREQ_USART0_TX:
			return 0;
		case DMAREQ_USART1_RX:
		case DMAREQ_USART1_TX:
			return 1;
		case DMAREQ_USART2_RX:
		case DMAREQ_USART2_TX:
			return 2;
		case DMAREQ_USART3_RX:
		case DMAREQ_USART3_TX:
			return 3;
		case DMAREQ_USART4_RX:
		case DMAREQ_USART4_TX:
			return 4;
		default:
			while(1);
	}
}

DMA_CHID_T Chip_USART_DMA_TxChan(LPC_USART_T *pUSART)
{
	static const DMA_CHID_T uartTxDMAChanTable[] = {
		DMAREQ_USART0_TX,
		DMAREQ_USART1_TX,
		DMAREQ_USART2_TX,
		DMAREQ_USART3_TX,
		DMAREQ_USART4_TX
	};

	return uartTxDMAChanTable[Chip_USART_USARTIndex(pUSART)];
}

DMA_CHID_T Chip_USART_DMA_RxChan(LPC_USART_T *pUSART)
{
	static const DMA_CHID_T uartRxDMAChanTable[] = {
		DMAREQ_USART0_RX,
		DMAREQ_USART1_RX,
		DMAREQ_USART2_RX,
		DMAREQ_USART3_RX,
		DMAREQ_USART4_RX
	};

	return uartRxDMAChanTable[Chip_USART_USARTIndex(pUSART)];
}

//******************************************************************************
// Returns the USART register base address associated with the DMA channel.
//******************************************************************************
LPC_USART_T *Chip_USART_DMAChanToRegBase(DMA_CHID_T dmaChannel)
{
	return Chip_USART_USARTRegBase(Chip_USART_DMAChanToUSARTIndex(dmaChannel));
}

//******************************************************************************
// Returns the USART LPCxxx IRQ number.
//******************************************************************************
IRQn_Type Chip_USART_IRQn(uint32_t usartNum)
{
	static const IRQn_Type usartIRQnTable[] = {
		UART0_IRQn,
		UART1_IRQn,
		UART2_IRQn,
		UART3_IRQn,
		UART4_IRQn
	};

	if (usartNum >= (sizeof(usartIRQnTable)/sizeof(usartIRQnTable[0])))
		while(1);
	return usartIRQnTable[usartNum];
}

//******************************************************************************
// Set-up and initate UART Tx DMA
//******************************************************************************
void Chip_USART_Tx_DMA(LPC_USART_T *pUSART, uint8_t *pBuf, uint32_t bufLen, uint8_t width, uint8_t enableCh, uint8_t enableChInt)
{
	DMA_RELOADDESC_T DMAChDesc;
	uint32_t xfercfg, chCFG;
	DMA_CHID_T uartDMAChan;

	xfercfg = DMA_XFERCFG_XFERCOUNT_MINUS1_SET(bufLen - 1) | DMA_XFERCFG_SRCINC_1 | DMA_XFERCFG_DSTINC_0
						| DMA_XFERCFG_WIDTH_SET(width) | DMA_XFERCFG_CFGVALID | DMA_XFERCFG_SWTRIG | DMA_XFERCFG_SETINTA;
	chCFG = DMA_CFG_PERIPHREQEN | DMA_CFG_CHPRIORITY_SET(0);	// Lower priority number is higher DMA priority
	Chip_DMA_ConvertAddresses(pBuf, (uint8_t *)(&(pUSART->TXDATA)), xfercfg, chCFG, &DMAChDesc);
	uartDMAChan = Chip_USART_DMA_TxChan(pUSART);
	Chip_DMA_SetAndEnable(uartDMAChan, (DMA_CHDESC_T *)&DMAChDesc, xfercfg, chCFG, enableCh, enableChInt);
}

//******************************************************************************
// Set-up and initate UART Rx DMA
//******************************************************************************
void Chip_USART_Rx_DMA(LPC_USART_T *pUSART, uint8_t *pBuf, uint32_t bufLen, uint8_t width, uint8_t enableCh, uint8_t enableChInt)
{
	DMA_RELOADDESC_T DMAChDesc;
	uint32_t xfercfg, chCFG;
	DMA_CHID_T uartDMAChan;

	xfercfg = DMA_XFERCFG_XFERCOUNT_MINUS1_SET(bufLen - 1) | DMA_XFERCFG_SRCINC_0 | DMA_XFERCFG_DSTINC_1
					| DMA_XFERCFG_WIDTH_SET(width) | DMA_XFERCFG_CFGVALID | DMA_XFERCFG_SWTRIG | DMA_XFERCFG_SETINTA;
	chCFG = DMA_CFG_PERIPHREQEN | DMA_CFG_CHPRIORITY_SET(0);	// Lower priority number is higher DMA priority
	Chip_DMA_ConvertAddresses((uint8_t *)(&(pUSART->RXDATA)), pBuf, xfercfg, chCFG, &DMAChDesc);
	DMAChDesc.next = 0;
	uartDMAChan = Chip_USART_DMA_RxChan(pUSART);
	Chip_DMA_SetAndEnable(uartDMAChan, (DMA_CHDESC_T *)&DMAChDesc, xfercfg, chCFG, enableCh, enableChInt);
}



