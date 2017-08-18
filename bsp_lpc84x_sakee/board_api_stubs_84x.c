/*
 * @brief LPC8xx stubbed common board API functions
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

#include "lpc_types.h"
#include "board_api.h"

#if defined(NO_BOARD_LIB)

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/* Default system oscillator rate and clock rate on the CLKIN pin */
const uint32_t OscRateIn = 12000000;
const uint32_t ExtRateIn = 0;

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* Set up and initialize hardware prior to call to main() */
void Board_SystemInit(void) {}

/* Set up and initialize all required blocks and functions related to the
   board hardware */
void Board_Init(void) {}

/* Initializes board UART for output, required for printf redirection */
void Board_Debug_Init(void) {}

/* Sends a single character on the UART, required for printf redirection */
void Board_UARTPutChar(char ch)
{
	(void) ch;
}

/* Get a single character from the UART, required for scanf input */
int Board_UARTGetChar(void)
{
	return EOF;
}

/* Prints a string to the UART */
void Board_UARTPutSTR(char *str)
{
	(void) str;
}

/* Sets the state of a board LED to on or off */
void Board_LED_Set(uint8_t LEDNumber, bool State)
{
	(void) LEDNumber;
	(void) State;
}

/* Returns the current state of a board LED */
bool Board_LED_Test(uint8_t LEDNumber)
{
	(void) LEDNumber;

	return false;
}

/* Toggles the current state of a board LED */
void Board_LED_Toggle(uint8_t LEDNumber)
{
	(void) LEDNumber;
}

/* Initialize debug output via UART for board */
void Board_Debug_Init(void) {}

#endif /* defined(NO_BOARD_LIB) */
