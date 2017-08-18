/*
 * @brief Basic CMSIS include file
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2013
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

#ifndef __CMSIS_H_
#define __CMSIS_H_

#include "lpc_types.h"
#include "sys_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup CMSIS_84X_ALL CHIP: LPC8xx CMSIS include file
 * @ingroup CHIP_84X_Drivers
 * @{
 */

#if defined(__ARMCC_VERSION)
// Kill warning "#pragma push with no matching #pragma pop"
  #pragma diag_suppress 2525
  #pragma push
  #pragma anon_unions
#elif defined(__CWCC__)
  #pragma push
  #pragma cpp_extensions on
#elif defined(__GNUC__)
/* anonymous unions are enabled by default */
#elif defined(__IAR_SYSTEMS_ICC__)
//  #pragma push // FIXME not usable for IAR
  #pragma language=extended
#else
  #error Not supported compiler type
#endif

#if !defined(CORE_M0PLUS)
#error Please #define CORE_M0PLUS
#endif

/** @defgroup CMSIS_84X CHIP: LPC8xx Cortex CMSIS definitions
 * @ingroup CMSIS_84X_ALL
 * @{
 */

/* Configuration of the Cortex-M0+ Processor and Core Peripherals */
#define __CM0_REV                 0x0000	/*!< Cortex-M0 Core Revision                          */
#define __MPU_PRESENT             0			/*!< MPU present or not                               */
#define __NVIC_PRIO_BITS          2			/*!< Number of Bits used for Priority Levels          */
#define __Vendor_SysTickConfig    0			/*!< Set to 1 if different SysTick Config is used     */

/**
 * @}
 */

/** @defgroup CMSIS_84X_IRQ CHIP: LPC8xx peripheral interrupt numbers
 * @ingroup CMSIS_84X_ALL
 * @{
 */

typedef enum {
	/******  Cortex-M0 Processor Exceptions Numbers ***************************************************/
	Reset_IRQn                    = -15,	/*!< 1 Reset Vector, invoked on Power up and warm reset */
	NonMaskableInt_IRQn           = -14,	/*!< 2 Non Maskable Interrupt                           */
	HardFault_IRQn                = -13,	/*!< 3 Cortex-M0 Hard Fault Interrupt                   */
	SVCall_IRQn                   = -5,		/*!< 11 Cortex-M0 SV Call Interrupt                     */
	PendSV_IRQn                   = -2,		/*!< 14 Cortex-M0 Pend SV Interrupt                     */
	SysTick_IRQn                  = -1,		/*!< 15 Cortex-M0 System Tick Interrupt                 */

	/******  LPC8xx Specific Interrupt Numbers ********************************************************/
	SPI0_IRQn                     = 0,		/*!< SPI0                                             */
	SPI1_IRQn                     = 1,		/*!< SPI1                                             */
	DAC0_IRQn                     = 2,		/*!< DAC0                                             */
	UART0_IRQn                    = 3,		/*!< USART0                                           */
	UART1_IRQn                    = 4,		/*!< USART1                                           */
	UART2_IRQn                    = 5,		/*!< USART2                                           */
	FAIM_IRQn                     = 6,		/*!< FAIM                                             */
	I2C1_IRQn                     = 7,		/*!< I2C1                                             */
	I2C0_IRQn                     = 8,		/*!< I2C0                                             */
	SCT_IRQn                      = 9,		/*!< SCT                                              */
	MRT_IRQn                      = 10,		/*!< MRT                                              */
	CMP_IRQn                      = 11,		/*!< CMP, CAPT                                        */
	WDT_IRQn                      = 12,		/*!< WDT                                              */
	BOD_IRQn                      = 13,		/*!< BOD                                              */
	FLASH_IRQn                    = 14,		/*!< Flash interrupt                                  */
	WKT_IRQn                      = 15,		/*!< WKT Interrupt                                    */
	ADC_SEQA_IRQn                 = 16,		/*!< ADC SEQA                                         */
	ADC_SEQB_IRQn                 = 17,		/*!< ADC SEQB                                         */
	ADC_THCMP_IRQn                = 18,		/*!< ADC THCMP                                        */
	ADC_OVR_IRQn                  = 19,		/*!< ADC OVERRUN                                      */
	DMA_IRQn                      = 20,		/*!< DMA                                              */
	I2C2_IRQn                     = 21,		/*!< I2C2                                             */
	I2C3_IRQn                     = 22,		/*!< I2C3                                             */
	CTIMER0_IRQn                  = 23,		/*!< CTIMER0                                          */
	PININT0_IRQn                  = 24,		/*!< External Interrupt 0                             */
	PININT1_IRQn                  = 25,		/*!< External Interrupt 1                             */
	PININT2_IRQn                  = 26,		/*!< External Interrupt 2                             */
	PININT3_IRQn                  = 27,		/*!< External Interrupt 3                             */
	PININT4_IRQn                  = 28,		/*!< External Interrupt 4                             */
	PININT5_IRQn                  = 29,		/*!< External Interrupt 5, DAC0                       */
	PININT6_IRQn                  = 30,		/*!< External Interrupt 6, UART3                      */
	PININT7_IRQn                  = 31,		/*!< External Interrupt 7, UART4                      */
} IRQn_Type;

/* ORed IRQs */
#define CAPT_IRQn								CMP_IRQn
#define DAC1_IRQn								PININT5_IRQn
#define UART3_IRQn							PININT6_IRQn
#define UART4_IRQn							PININT7_IRQn
/**
 * @}
 */

#include "core_cm0plus.h"				/*!< Cortex-M0+ processor and core peripherals        */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __CMSIS_H_ */
