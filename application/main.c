/*
 * @brief Blinky example using SysTick and interrupt
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
#include "iap.h"
#include <stdio.h>

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

#define TICKRATE_HZ (10)	/* 10 ticks per second */
#define TIMEOUT_TICKS (10 * TICKRATE_HZ)	/* 10 seconds */

/* SystemTick Counter */
static volatile uint32_t sysTick;

/* IAP command variables */
static struct sIAP IAP;

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	Handle interrupt from SysTick timer
 * @return	Nothing
 */
void SysTick_Handler(void)
{
    Board_LED_Toggle(0);
	sysTick++;
}

/**
 * @brief	main routine for blinky example
 * @return	Function should not exit.
 */
int main(void)
{
	SystemCoreClockUpdate();
	Board_Init();
    
    Chip_Clock_EnablePeriphClock(SYSCON_CLOCK_SWM);
    Chip_Clock_EnablePeriphClock(SYSCON_CLOCK_GPIO0);
    Chip_Clock_EnablePeriphClock(SYSCON_CLOCK_GPIO1);
    Chip_Clock_EnablePeriphClock(SYSCON_CLOCK_IOCON);
    
    /* Peripheral reset to the GPIO module. '0' asserts, '1' deasserts reset. */
    LPC_SYSCON->PRESETCTRL[0] &=  ((~(1<<6)) & (~(1<<20)));
    LPC_SYSCON->PRESETCTRL[0] |= ~((~(1<<6)) & (~(1<<20)));
    
	/* Configure PIN0.0 with pull-up */
	Chip_IOCON_PinSetMode(IOCON_PIO0_0, PIN_MODE_PULLUP);
    Chip_GPIO_SetPinDIROutput(LPC_GPIO, 0, 0);
    Chip_GPIO_SetPinState(LPC_GPIO, 0, 0, true);
    
	/* Enable SysTick Timer */
	SysTick_Config(SystemCoreClock / TICKRATE_HZ);
    
    DEBUGOUT("LPC845 SAKEE Board\r\n");
	
	/* Bail out after timeout */
	while (sysTick <= TIMEOUT_TICKS) {
		__WFI();
	}

	/* Reinvoke ISP mode so that reprogamming of Flash possible */
	__disable_irq();
	IAP.cmd = IAP_REINVOKE_ISP;
	IAP.par[0] = REINVOKE_UART;
	IAP_Call(&IAP.cmd, &IAP.stat);

	return 0;
}
