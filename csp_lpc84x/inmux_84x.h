/*
 * @brief LPC8xx DMA input trigger input mux, DMA trigger input mux input,
 * SCT input mux Registers and driver
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

#ifndef __INMUX_84X_H_
#define __INMUX_84X_H_

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup INMUX_84X CHIP: LPC8xx DMA input trigger input mux, DMA trigger input mux input,
 * 	SCT input mux Registers and driver
 * @ingroup CHIP_84X_Drivers
 * @{
 */

typedef struct {
  union{
  uint32_t DMA_ITRIG_INMUX[18];
  struct{
  __IO uint32_t DMA_ITRIG_INMUX0;                   	
  __IO uint32_t DMA_ITRIG_INMUX1;                   	
  __IO uint32_t DMA_ITRIG_INMUX2;                   	
  __IO uint32_t DMA_ITRIG_INMUX3;                   	
  __IO uint32_t DMA_ITRIG_INMUX4;
  __IO uint32_t DMA_ITRIG_INMUX5;                   	
  __IO uint32_t DMA_ITRIG_INMUX6;                   	
  __IO uint32_t DMA_ITRIG_INMUX7;
  __IO uint32_t DMA_ITRIG_INMUX8;                   	
  __IO uint32_t DMA_ITRIG_INMUX9;                   	
  __IO uint32_t DMA_ITRIG_INMUX10;                   	
  __IO uint32_t DMA_ITRIG_INMUX11;                   	
  __IO uint32_t DMA_ITRIG_INMUX12;
  __IO uint32_t DMA_ITRIG_INMUX13;                   	
  __IO uint32_t DMA_ITRIG_INMUX14;                   	
  __IO uint32_t DMA_ITRIG_INMUX15;
  __IO uint32_t DMA_ITRIG_INMUX16;                   	
  __IO uint32_t DMA_ITRIG_INMUX17;
  };
  };
} LPC_ITRIG_INMUX_T;

typedef struct {
  union{
  __IO uint32_t DMA_INMUX_INMUX[2];
  struct{
  __IO uint32_t DMA_INMUX_INMUX0;                   	
  __IO uint32_t DMA_INMUX_INMUX1;                   	
  };
  };
  __I  uint32_t RESERVED[6];
  union{
  __IO uint32_t SCT_INMUX[4];
  struct{
  __IO uint32_t SCT_INMUX0;                   	
  __IO uint32_t SCT_INMUX1;                   	
  __IO uint32_t SCT_INMUX2;                   	
  __IO uint32_t SCT_INMUX3;                   	
  };
  };
} LPC_INMUX_INMUX_T;

/**
 * LPC8xx DMA input trigger input mux source
 */
typedef enum Chip_ITRIG_INMUX_DMA_CFG {
  INMUX_ADC_SEQA_IRQ = 0,
  INMUX_ADC_SEQB_IRQ,
  INMUX_SCT_DMA0,
  INMUX_SCT_DMA1,
  INMUX_ACMP_O,
  INMUX_PININT0,
  INMUX_PININT1,
  INMUX_OTRIG0_DMA,
  INMUX_OTRIG1_DMA
} Chip_ITRIG_INMUX_DMA_CFG_T;

/**
 * @brief	Set ITRIG_INMUX number for DMA 
 * @param	pITRIG_INMUX: The base of ITRIG_INMUX peripheral on the chip
 * @param	ch_num: channel number
 * @param	tg_num: trigger number
 * @return	Nothing
 */
STATIC INLINE void Chip_INMUX_Config_ITRIG_DMA(LPC_ITRIG_INMUX_T *pITRIG_INMUX, DMA_CHID_T channelnum, Chip_ITRIG_INMUX_DMA_CFG_T itriggernum )
{
  pITRIG_INMUX->DMA_ITRIG_INMUX[channelnum] = itriggernum;
}

/**
 * @brief	Set INMUX_INMUX number for DMA 
 * @param	pINMUX_INMUX: The base of INMUX_INMUX peripheral on the chip
 * @param	ch_num: channel number
 * @param	tg_num: trigger number
 * @return	Nothing
 */
STATIC INLINE void Chip_INMUX_Config_INMUX_DMA(LPC_INMUX_INMUX_T *pINMUX_INMUX_T, uint32_t outputnum, DMA_CHID_T channelnum )
{
  pINMUX_INMUX_T->DMA_INMUX_INMUX[outputnum] = channelnum;
}

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __INMUX_84X_H_ */
