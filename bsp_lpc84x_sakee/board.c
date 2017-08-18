/*
 * @brief NXP LPCXpresso LPC84X board file
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

#include "board.h"
#include "string.h"
#include "retarget.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

typedef struct {
	uint8_t port;
	uint8_t pin;
} PORT_PIN_T;

static const PORT_PIN_T ledBits[] = {{0, 0}};
static const uint32_t ledBits_ct = sizeof(ledBits) / sizeof(PORT_PIN_T);

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/* System oscillator rate and clock rate on the CLKIN pin */
const uint32_t OscRateIn = 12000000;
const uint32_t ExtRateIn = 0;

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Initialize the LEDs on the NXP LPC812 LPCXpresso Board */
static void Board_LED_Init(void)
{
	int i;
	
	/* Pin muxing setup as part of board_sysinit */
	for (i = 0; i < ledBits_ct; i++) {
		Chip_GPIO_SetPinDIROutput(LPC_GPIO, ledBits[i].port, ledBits[i].pin);
		Chip_GPIO_SetPinState(LPC_GPIO, ledBits[i].port, ledBits[i].pin, true);
	}
}

/* Board Debug UART Initialisation function */
STATIC void Board_UART_Init(void)
{
	/* Enable the clock to the Switch Matrix */
	Chip_Clock_EnablePeriphClock(SYSCON_CLOCK_SWM);

	/* Set FRG0 clock select to use MAIN clock. */
	Chip_Clock_SetFRGClockSource(0, SYSCON_FRGCLKSRC_MAINCLK);
	/* use FRG number 0, Divided by 1 */
	Chip_Clock_SetUARTClockDiv(0, 1);
	
	/* Set FLEXCLK0 to use FRG0 clock for UART communication. */
	Chip_Clock_SetFClockSource(0, SYSCON_FLEXCOMMCLKSELSRC_FRG0);

	/* Connect the U0_TXD_O and U0_RXD_I signals to port pins(P0.4, P0.0) */
	Chip_SWM_MovablePinAssign(SWM_U0_TXD_O, 25);
	Chip_SWM_MovablePinAssign(SWM_U0_RXD_I, 24);

	/* Disable the clock to the Switch Matrix to save power */
	Chip_Clock_DisablePeriphClock(SYSCON_CLOCK_SWM);
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* Set the LED to the state of "On" */
void Board_LED_Set(uint8_t LEDNumber, bool On)
{
	if (LEDNumber < ledBits_ct) {
		Chip_GPIO_SetPinState(LPC_GPIO, ledBits[LEDNumber].port, ledBits[LEDNumber].pin, (bool) !On);
	}
}

/* Return the state of LEDNumber */
bool Board_LED_Test(uint8_t LEDNumber)
{
	if (LEDNumber < ledBits_ct) {
		return (bool) !Chip_GPIO_GetPinState(LPC_GPIO, ledBits[LEDNumber].port, ledBits[LEDNumber].pin);
	}

	return false;
}

/* Toggles the current state of a board LED */
void Board_LED_Toggle(uint8_t LEDNumber)
{
	if (LEDNumber < ledBits_ct) {
		Chip_GPIO_SetPinToggle(LPC_GPIO, ledBits[LEDNumber].port, ledBits[LEDNumber].pin);
	}
}

/* Sends a character on the UART */
void Board_UARTPutChar(char ch)
{
#if defined(DEBUG_UART)
	Chip_UART_SendBlocking(DEBUG_UART, &ch, 1);
#endif
}

/* Gets a character from the UART, returns EOF if no character is ready */
int Board_UARTGetChar(void)
{
#if defined(DEBUG_UART)
	uint8_t data;

	if (Chip_UART_Read(DEBUG_UART, &data, 1) == 1) {
		return (int) data;
	}
#endif
	return EOF;
}

/* Outputs a string on the debug UART */
void Board_UARTPutSTR(char *str)
{
#if defined(DEBUG_UART)
	while (*str != '\0') {
		Board_UARTPutChar(*str++);
	}
#endif
}

/* Initialize debug output via UART for board */
void Board_Debug_Init(void)
{
	/* Using FRG number 0 for debugger UART port */
#if defined(DEBUG_UART)
	Board_UART_Init();
	Chip_UART_Init(DEBUG_UART);
	Chip_UART_ConfigData(DEBUG_UART, UART_CFG_DATALEN_8 | UART_CFG_PARITY_NONE | UART_CFG_STOPLEN_1);
	Chip_UART_SetBaud(0, DEBUG_UART, 115200);
	Chip_UART_Enable(DEBUG_UART);
	Chip_UART_TXEnable(DEBUG_UART);
#endif
}

/* Set up and initialize all required blocks and functions related to the
   board hardware */
void Board_Init(void)
{
	/* Sets up DEBUG UART */
	DEBUGINIT();

	/* Initialize GPIO */
	Chip_GPIO_Init(LPC_GPIO_PORT);

	/* Initialize the LEDs */
	Board_LED_Init();
}
