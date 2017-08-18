/*
 * @brief NXP LPCXpresso LPC84X board file
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

#ifndef __BOARD_H_
#define __BOARD_H_

#include "chip.h"
/* board_api.h is included at the bottom of this file after DEBUG setup */

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup BOARD_84X NXP LPC84X LPCXpresso board support software API functions
 * @ingroup LPCOPEN_84X_BOARD_XPRESSO_812
 * The board support software API functions provide some simple abstracted
 * functions used across multiple LPCOpen board examples. See @ref BOARD_COMMON_API
 * for the functions defined by this board support layer.<br>
 * @{
 */

/** @defgroup BOARD_NXP_XPRESSO_812_OPTIONS BOARD: LPC84X LPCXpresso board build options
 * This board has options that configure its operation at build-time.<br>
 * @{
 */

/** Define DEBUG_ENABLE to enable IO via the DEBUGSTR, DEBUGOUT, and
    DEBUGIN macros. If not defined, DEBUG* functions will be optimized
	out of the code at build time.
 */
#define DEBUG_ENABLE

/** Define DEBUG_SEMIHOSTING along with DEBUG_ENABLE to enable IO support
    via semihosting. You may need to use a C library that supports
	semihosting with this option.
 */
//#define DEBUG_SEMIHOSTING

/** Board UART used for debug output and input using the DEBUG* macros. This
    is also the port used for Board_UARTPutChar, Board_UARTGetChar, and
	Board_UARTPutSTR functions. Although you can setup multiple UARTs here,
	the board code only supoprts UART0 in the Board_UART_Init() fucntion,
	so be sure to change it there too if not using UART0.
 */
#define DEBUG_UART LPC_USART0

//#define USE_ROM_API

/** Set to 0 to use the on-chip IRC, or 1 to use the external oscillator */
#define LPC8XX_USE_XTAL_OSC 0
#define LPC8XX_USE_CLKIN 		0
//*** LPC8XX_USE_XTAL_OSC should have been defined in the compiler command line,
// not here.
// So, to override the board.h setting, use USE_XTAL_OSC or USE_FRO
// in the compiler command line:
#if defined(USE_XTAL_OSC)
#undef LPC8XX_USE_XTAL_OSC
#define LPC8XX_USE_XTAL_OSC 1
#endif
#if defined(USE_FRO)
#undef LPC8XX_USE_XTAL_OSC
#define LPC8XX_USE_XTAL_OSC 0
#endif


/**
 * @}
 */

/* Board name */
//#define BOARD_NXP_XPRESSO_812
#define BOARD_LPC84x_Validation

#ifdef BOARD_LPC84x_Validation
// For Necha Plus Validation Board:
//	Connect:
//		PIO0_4 to TXD DIN (JP9-P8 to JP12-P5)
//		PIO0_28 to RXD ROUT (JP9-P14 to JP12-P4)
//
// These Pin&Port selections are abitrary and can be changed.
//#define PaP_USART_MAIN_TXD		15
//#define PaP_USART_MAIN_RXD		16
#define PaP_USART_MAIN_TXD		4
#define PaP_USART_MAIN_RXD		0
#define PaP_USART_MAIN_CTS		24
#define PaP_USART_MAIN_RTS		25
#define PaP_USART_MAIN_SCLK		26

#define PaP_USART_2_TXD				14
#define PaP_USART_2_RXD				15
#define PaP_USART_2_CTS				16
#define PaP_USART_2_RTS				17
#define PaP_USART_2_SCLK			18

// These Pin&Port numbers are fix pin assignments
#define PaP_FIXED_ACMP_I1			0
#define PaP_FIXED_ACMP_I2			1
#define PaP_FIXED_ACMP_I3			14
#define PaP_FIXED_ACMP_I4			23

#endif

/**
 * @}
 */

#include "board_api.h"

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H_ */
