/*
 * @brief LPC84X DMA chip driver
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

#include "chip.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/* DMA SRAM table - this can be optionally used with the Chip_DMA_SetSRAMBase()
   function if a DMA SRAM table is needed. This table is correctly aligned for
     the DMA controller. */
#if defined(__CC_ARM)
/* Keil alignement to 512 bytes */
__align(512) volatile DMA_CHDESC_T Chip_DMA_Table[DMA_CHANNELS];
#if RELOAD_DESC_ENTRIES
 __align(16) volatile DMA_RELOADDESC_T Chip_DMA_ReloadDescTable[RELOAD_DESC_ENTRIES];
#endif
#endif /* defined (__CC_ARM) */

/* IAR support */
#if defined(__ICCARM__)
/* IAR EWARM alignement to 512 bytes */
#pragma data_alignment=512
volatile DMA_CHDESC_T Chip_DMA_Table[DMA_CHANNELS];
#if RELOAD_DESC_ENTRIES
#pragma data_alignment=16
volatile DMA_RELOADDESC_T Chip_DMA_ReloadDescTable[RELOAD_DESC_ENTRIES];
#endif
#endif /* defined (__ICCARM__) */

#if defined( __GNUC__ )
/* GNU alignement to 512 bytes */
__align(512) volatile DMA_CHDESC_T Chip_DMA_Table[DMA_CHANNELS] __attribute__ ((aligned(512)));
#if RELOAD_DESC_ENTRIES
 __align(16) volatile DMA_RELOADDESC_T Chip_DMA_ReloadDescTable[RELOAD_DESC_ENTRIES];
#endif
#endif /* defined (__GNUC__) */

#define DMA_ChannelDescriptor	DMA_RELOADDESC_T
DMA_ChannelInterruptHandler		DMAIntFuncVectorTable[DMA_CHANNELS];
uint32_t	chip_dma_irq_ErrorCount;

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/******************************************************************************
** Function name:		Chip_DMA_Init
**
** Descriptions:		Set clock, reset DMA block, clear all interrupts, set
**							the Channel Config Table base address (SRAMBASE). 
**                      
**
** parameters:          pDMA = LPC_DMA_T *		
** Returned value:		
** 
******************************************************************************/
void Chip_DMA_Init(LPC_DMA_T *pDMA)
{	
	NVIC_DisableIRQ(DMA_IRQn);

	Chip_Clock_EnablePeriphClock(SYSCON_CLOCK_DMA);
	
	// Reset the DMA
//	Chip_SYSCON_PeriphReset(??);		// No reset for DMA in UM

  // Enable DMA controller, master enable, user access, non-bufferable, non-cacheable.
  pDMA->CTRL = 0;
  /* clear all interrupts on all channels */
  pDMA->DMACOMMON[0].ENABLECLR = 0xFFFFFFFF;
  pDMA->DMACOMMON[0].ERRINT = 0xFFFFFFFF;
  pDMA->CTRL = DMA_CTRL_ENABLE;

  while (pDMA->DMACOMMON[0].BUSY);				// No channel should be busy
  pDMA->DMACOMMON[0].ABORT = 0xFFFFFFFF;
  //disable all DMA channel interrupt
  pDMA->DMACOMMON[0].INTENCLR = 0xFFFFFFFF;		// Clear INT enables
  pDMA->DMACOMMON[0].INTA = 0xFFFFFFFF;				// Clear INT status
  pDMA->DMACOMMON[0].INTB = 0xFFFFFFFF;

  pDMA->SRAMBASE = (uint32_t)Chip_DMA_Table; 

	chip_dma_irq_ErrorCount = 0;
//			for ( i = 0; i < 32; i++ )
//					dmaErrCount[i]++;	
	NVIC_ClearPendingIRQ(DMA_IRQn);
  NVIC_EnableIRQ(DMA_IRQn);
  return;
}


/* Set DMA transfer register interrupt bits (safe) */
void Chip_DMA_SetTranBits(LPC_DMA_T *pDMA, DMA_CHID_T ch, uint32_t mask)
{
	uint32_t temp;

	/* Read and write values may not be the same, write 0 to
	   undefined bits */
	temp = pDMA->CHANNEL[ch].XFERCFG & ~0xFC000CC0;

	pDMA->CHANNEL[ch].XFERCFG = temp | mask;
}

/* Clear DMA transfer register interrupt bits (safe) */
void Chip_DMA_ClearTranBits(LPC_DMA_T *pDMA, DMA_CHID_T ch, uint32_t mask)
{
	uint32_t temp;

	/* Read and write values may not be the same, write 0 to
	   undefined bits */
	temp = pDMA->CHANNEL[ch].XFERCFG & ~0xFC000CC0;

	pDMA->CHANNEL[ch].XFERCFG = temp & ~mask;
}

/* Update the transfer size in an existing DMA channel transfer configuration */
void Chip_DMA_SetupChannelTransferSize(LPC_DMA_T *pDMA, DMA_CHID_T ch, uint32_t trans)
{
	Chip_DMA_ClearTranBits(pDMA, ch, (0x3FF << 16));
	Chip_DMA_SetTranBits(pDMA, ch, DMA_XFERCFG_XFERCOUNT(trans));
}

/* Sets up a DMA channel with the passed DMA transfer descriptor */
bool Chip_DMA_SetupTranChannel(LPC_DMA_T *pDMA, DMA_CHID_T ch, DMA_CHDESC_T *desc)
{
	bool good = false;
	DMA_CHDESC_T *pDesc = (DMA_CHDESC_T *) pDMA->SRAMBASE;

	if ((Chip_DMA_GetActiveChannels(pDMA) & (1 << ch)) == 0) {
		/* Channel is not active, so update the descriptor */
		pDesc[ch] = *desc;

		good = true;
	}

	return good;
}

/**
 * Initialize DMA parameters specific to a channel 
 * 
 * @param channel
 * @param src_address
 * @param dst_address
 * @param xfr_width
 * @param length_bytes
 */
void Chip_DMA_InitChannel( DMA_CHID_T channel, uint32_t src_address, uint32_t src_increment,
        uint32_t dst_address, uint32_t dst_increment, uint32_t xfr_width, uint32_t length_bytes, uint32_t priority) 
{
  Chip_DMA_EnableChannel(LPC_DMA, channel);
  Chip_DMA_EnableIntChannel(LPC_DMA, channel);

  Chip_DMA_SetupChannelConfig(LPC_DMA, channel, DMA_CFG_PERIPHREQEN |
          DMA_CFG_CHPRIORITY(priority));

  if (src_increment != DMA_XFERCFG_SRCINC_0) {
    Chip_DMA_Table[channel].source = DMA_ADDR((src_address + length_bytes)
            - (1UL << xfr_width));
  } else {
    Chip_DMA_Table[channel].source = DMA_ADDR(src_address);
  }

  if (dst_increment != DMA_XFERCFG_DSTINC_0) {
    Chip_DMA_Table[channel].dest = DMA_ADDR((dst_address + length_bytes)
            - (1UL << xfr_width));
  } else {
    Chip_DMA_Table[channel].dest = DMA_ADDR(dst_address);
  }
  Chip_DMA_Table[channel].next = DMA_ADDR(0);

}

/**
 * Start the DMA transfer 
 * 
 * @param channel
 * @param src_increment
 * @param dst_increment
 * @param xfr_width
 * @param length_bytes
 */
void Chip_DMA_StartTransfer(DMA_CHID_T channel, uint32_t src_increment, uint32_t dst_increment, uint32_t xfr_width, uint32_t length_bytes) 
{
  uint32_t xfer_count;

  /* Calculate transfer_count ( length in terms of transfers) */
  xfer_count = (xfr_width == DMA_XFERCFG_WIDTH_8) ? length_bytes :
          (xfr_width == DMA_XFERCFG_WIDTH_16) ? (length_bytes >> 1) :
          (length_bytes >> 2);

  Chip_DMA_SetupChannelTransfer(LPC_DMA, channel,
          (DMA_XFERCFG_CFGVALID | DMA_XFERCFG_SETINTA | DMA_XFERCFG_SWTRIG |
          xfr_width | src_increment | dst_increment |
          DMA_XFERCFG_XFERCOUNT(xfer_count)));
}


/******************************************************************************
** Function name:		Chip_DMA_SetAndEnable
**
** Descriptions:		Use the passed DMA descriptors as the initial DMA descriptors.
**										Other setup and starting of the DMA.
**
** Note:  This doesn't adjust src or dest addresses (call Chip_DMA_ConvertAddresses)
** parameters:
**			dmaChannel
**			pDMAChDesc: Channel descriptor containing src, dest, link (to be loaded into
**					the channel's Channel Descriptor Table)
**			xfercfg:		Written to XFERCFG
**			chCFG:			Written to channels CFG register
**			enableCh:		enable channel flag
**			enableChInt: enable channel interrupt flag
**			 
** Returned value:	none		
** 
******************************************************************************/
void Chip_DMA_SetAndEnable(uint32_t dmaChannel, DMA_CHDESC_T *pDMAChDesc,
														uint32_t xfercfg, uint32_t chCFG,
														uint8_t enableCh, uint8_t enableChInt)
{
  LPC_DMA->DMACOMMON[0].ENABLECLR |= (0x01 << dmaChannel);
  LPC_DMA->DMACOMMON[0].INTENCLR |= (0x01 << dmaChannel);

	Chip_DMA_Table[dmaChannel].source = (uint32_t)pDMAChDesc->source;
	Chip_DMA_Table[dmaChannel].dest = (uint32_t)pDMAChDesc->dest;
	Chip_DMA_Table[dmaChannel].next = (uint32_t)pDMAChDesc->next;
	LPC_DMA->CHANNEL[dmaChannel].CFG = chCFG;
	// Must set XFERCFG last or make sure channel is disabled
	LPC_DMA->CHANNEL[dmaChannel].XFERCFG = xfercfg;

	if (enableChInt)
		LPC_DMA->DMACOMMON[0].INTENSET |= 0x01 << dmaChannel;	  // Enable DMA interrupt
	
	if (enableCh)
		LPC_DMA->DMACOMMON[0].ENABLESET |= 0x01 << dmaChannel;  // Enable DMA channel
}


/******************************************************************************
** Function name:		Chip_DMA_ConvertAddresses
**
** Descriptions:		Convert src & dest starting addresses to end address - 1
**
** parameters:			None
**			srcAddr:
**			destAddr:
**			xfercfg:		XFERCFG value
**			chCFG:			channels CFG value
**			
** Returned value:
**			Resulting addresses and XFERCFG value written to pDMAChDesc
** 
******************************************************************************/
void Chip_DMA_ConvertAddresses(uint8_t *srcAddr, uint8_t *destAddr,
														uint32_t xfercfg, uint32_t chCFG,
														DMA_RELOADDESC_T *pDMAChDesc)
{
	uint32_t	bytesWide = (1 << DMA_XFERCFG_WIDTH_GET(xfercfg));
	uint32_t	srcInterval = ((1 << DMA_XFERCFG_SRCINC_GET(xfercfg)) >> 1) * bytesWide;		// 0, 1, 2 or 4 * bytesWide
	uint32_t	destInterval = ((1 << DMA_XFERCFG_DSTINC_GET(xfercfg)) >> 1) * bytesWide;
	uint32_t	txCountMinus1 = DMA_XFERCFG_XFERCOUNT_MINUS1_GET(xfercfg);
	uint32_t	burstLenM1 = (1 << DMA_CFG_BURSTPPOWER_GET(chCFG)) - 1;

	if ((chCFG & DMA_CFG_SRCBURSTWRAP) && (burstLenM1 < txCountMinus1))
		pDMAChDesc->source = ((uint32_t)srcAddr) + (burstLenM1 * srcInterval);
	else
		pDMAChDesc->source = ((uint32_t)srcAddr) + (txCountMinus1 * srcInterval);

	if ((chCFG & DMA_CFG_DSTBURSTWRAP) && (burstLenM1 < txCountMinus1))
		pDMAChDesc->dest = ((uint32_t)destAddr) + (burstLenM1 * destInterval);
	else
		pDMAChDesc->dest = ((uint32_t)destAddr) + (txCountMinus1 * destInterval);

	return;	
}

//******************************************************************************
// Function name:		DMA_RegisterDMAHandler
//
// Descriptions:			Register DMA interrupt handler for specific interrupt.
//
// parameters:        None			
// Returned value:		None
// 
//******************************************************************************
void Chip_DMA_RegisterDMA_IRQHandler(int channelNum, DMA_ChannelInterruptHandler interruptHandler)
{
	DMAIntFuncVectorTable[channelNum] = *interruptHandler;
	return;
}

/******************************************************************************
** Function name:		DMA_IRQHandler
**
** Descriptions:		DMA interrupt handler.  Calls enabled-pending interrupt handler
**							registered through Chip_DMA_RegisterDMA_IRQHandler().
**
** parameters:			None
** Returned value:		None
** 
******************************************************************************/
void DMA_IRQHandler (void) 
{
  uint32_t pendingEnabled, dmaChannel;

	// Get pending and enabled interrupt status
	pendingEnabled = (LPC_DMA->DMACOMMON[0].INTA | LPC_DMA->DMACOMMON[0].INTB | LPC_DMA->DMACOMMON[0].ERRINT)
					& LPC_DMA->DMACOMMON[0].INTENSET; 
	for (dmaChannel = 0; pendingEnabled != 0; dmaChannel++)
	{
		if (pendingEnabled & 1)
		{
			if (DMAIntFuncVectorTable[dmaChannel] != NULL)
			{
				// Don't clear interrupts here, before or after.  Called func may need the status.
				// It may also set-up another DMA transaction that interrupts before it returns here.
				(*DMAIntFuncVectorTable[dmaChannel])(dmaChannel);		// Call the channels interrupt handler
			}
			else
			{
				LPC_DMA->DMACOMMON[0].INTA = (1 << dmaChannel);			// Clear the interrupt first
				LPC_DMA->DMACOMMON[0].INTB = (1 << dmaChannel);			// Clear the interrupt first
				LPC_DMA->DMACOMMON[0].ERRINT = (1 << dmaChannel);		// Clear the error status
				chip_dma_irq_ErrorCount++;
			}
		}
		pendingEnabled >>= 1;
	}

	return;
}


/******************************************************************************
**                            End Of File
******************************************************************************/

