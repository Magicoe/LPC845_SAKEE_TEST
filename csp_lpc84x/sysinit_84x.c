/*
 * @brief LPC84X Chip specific SystemInit
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

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* Set up and initialize hardware prior to call to main */
void Chip_SystemInit(void)
{
#if defined (USE_ROM_API)
	uint32_t cmd[4], resp[2];
#endif

	/* Turn on the IRC by clearing the power down bit */
	Chip_SYSCON_PowerUp(SYSCON_PDRUNCFG_FRO);
	Chip_SYSCON_PowerUp(SYSCON_PDRUNCFG_FROOUT);
	
	/* Select the PLL input in the IRC */
	Chip_Clock_SetSystemPLLSource(SYSCON_PLLCLKSRC_FRO);

	/* Setup FLASH access to 2 clocks (up to 30MHz) */
	Chip_FMC_SetFLASHAccess(FLASHTIM_WS_2);

#if defined (USE_ROM_API)
	/* Use ROM API for setting up PLL */
	cmd[0] = Chip_Clock_GetIntOscRate() / 1000; /* in KHz */
	cmd[1] = 36000000 / 1000; /* 36MHz system clock rate */
	cmd[2] = CPU_FREQ_EQU;
	cmd[2] = 36000000 / 10000;
	LPC_PWRD_API->set_pll(cmd, resp);

	/* Dead loop on fail */
	while (resp[0] != PLL_CMD_SUCCESS) {}

#else
	/* Power down PLL to change the PLL divider ratio */
	Chip_SYSCON_PowerDown(SYSCON_PDRUNCFG_SYSPLL);

	/* Configure the PLL M and P dividers */
	/* Setup PLL for main oscillator rate (FCLKIN = 12MHz) * 5 = 60Hz
	   MSEL = 4 (this is pre-decremented), PSEL = 1 (for P = 2)
	   FCLKOUT = FCLKIN * (MSEL + 1) = 12MHz * 5 = 60MHz
	   FCCO = FCLKOUT * 2 * P = 60MHz * 2 * 2 = 240MHz (within FCCO range) */
	Chip_Clock_SetupSystemPLL(4, 1);

	/* Turn on the PLL by clearing the power down bit */
	Chip_SYSCON_PowerUp(SYSCON_PDRUNCFG_SYSPLL);

	/* Wait for PLL to lock */
	while ((LPC_SYSCON->SYSPLLSTAT & 0x1) == 0);

	/* Set system clock divider to 2 */
	Chip_Clock_SetSysClockDiv(2);

	/* Set main clock source to the system PLL. This will drive 24MHz
	   for the main clock and 24MHz for the system clock */
	Chip_Clock_SetMainClockSource(SYSCON_MAINCLKSRC_FRO);
	Chip_Clock_SetMainClockPLLSource(SYSCON_MAINCLKPLLSRC_PLLOUT);
#endif

	/* Select the CLKOUT clocking source */
	Chip_Clock_SetCLKOUTSource(SYSCON_CLKOUTSRC_MAINCLK, 1);
}
