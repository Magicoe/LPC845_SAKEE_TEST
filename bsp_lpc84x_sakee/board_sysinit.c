/*
 * @brief NXP LPCXpresso LPC84X Sysinit file
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

 #include "board.h"

/* The System initialization code is called prior to the application and
   initializes the board for run-time operation. Board initialization
   for the NXP LPC84X board includes default pin muxing and clock setup
   configuration. */

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Setup system clocking */
STATIC void SystemSetupClocking(void)
{
	volatile uint32_t *faim_addr = (volatile uint32_t *)LPC_FAIM_BASE;
	
#if defined (USE_ROM_API)
	uint32_t cmd[4], resp[2];
	const PWRD_API_T *LPC_PWRD_API = LPC_ROM_API->pPWRD;
#endif

	/* Turn on the IRC by clearing the power down bit */
	Chip_SYSCON_PowerUp(SYSCON_PDRUNCFG_FRO);
	Chip_SYSCON_PowerUp(SYSCON_PDRUNCFG_FROOUT);

	/* Select the PLL input in the IRC */
	Chip_Clock_SetSystemPLLSource(SYSCON_PLLCLKSRC_FRO);

	/* FIXME. Setup FLASH access to 2 clocks (up to 30MHz) */
	Chip_FMC_SetFLASHAccess(FLASHTIM_WS_2);

#if defined (USE_ROM_API)
	cmd[0] = Chip_Clock_GetIntOscRate() / 1000; /* in KHz */
	cmd[1] = 24000000 / 1000; /* 24MHz system clock rate */
	cmd[2] = CPU_FREQ_EQU;
	cmd[3] = 30000000 / 10000;
	LPC_PWRD_API->set_pll(cmd, resp);

	/* Dead loop on fail */
	while (resp[0] != PLL_CMD_SUCCESS) {}

#else
	/* Power down PLL to change the PLL divider ratio */
	Chip_SYSCON_PowerDown(SYSCON_PDRUNCFG_SYSPLL);

	/* Configure the PLL M and P dividers */
	if ( *faim_addr & (0x1<<1) ) {
	/* FIXME, according to the PLL IP spec., the minimum input of PLL needs to be
		10MHz, it seems that it can work at much lower PLL input clock!!! */
	/* Setup PLL for main oscillator rate (FCLKIN = 1.5MHz) * 32 = 60Hz
	   MSEL = 31 (this is pre-decremented), PSEL = 1 (for P = 2)
	   FCLKOUT = FCLKIN * (MSEL + 1) = 1.5MHz * 32 = 48MHz
	   FCCO = FCLKOUT * 2 * P = 48MHz * 2 * 2 = 192MHz (within FCCO range) */
		Chip_Clock_SetupSystemPLL(31, 1);
	}
	else {
	/* Setup PLL for main oscillator rate (FCLKIN = 12MHz) * 5 = 60Hz
	   MSEL = 4 (this is pre-decremented), PSEL = 1 (for P = 2)
	   FCLKOUT = FCLKIN * (MSEL + 1) = 12MHz * 5 = 60MHz
	   FCCO = FCLKOUT * 2 * P = 60MHz * 2 * 2 = 240MHz (within FCCO range) */
		Chip_Clock_SetupSystemPLL(4, 1);
	}

	/* Turn on the PLL by clearing the power down bit */
	Chip_SYSCON_PowerUp(SYSCON_PDRUNCFG_SYSPLL);

	/* Wait for PLL to lock */
	while ((LPC_SYSCON->SYSPLLSTAT & 0x1) == 0);

	/* Set system clock divider to 2 */
	Chip_Clock_SetSysClockDiv(2);

	/* Set main clock source to the system PLL. This will drive 30MHz
	   for the main clock and 30MHz for the system clock */
	Chip_Clock_SetMainClockSource(SYSCON_MAINCLKSRC_FRO);
	Chip_Clock_SetMainClockPLLSource(SYSCON_MAINCLKPLLSRC_PLLOUT);
#endif

	/* Select the CLKOUT clocking source */
	Chip_Clock_SetCLKOUTSource(SYSCON_CLKOUTSRC_MAINCLK, 1);
}

/* Setup system clocking using EXT OSC. */
STATIC void SystemSetupExtclock(void)
{
	uint32_t i;

#if (LPC8XX_USE_CLKIN == 1)
	LPC_SYSCON->EXTCLKSEL = 0x01;
#else
	/* Turn on the SYSOSC by clearing the power down bit */
	Chip_SYSCON_PowerUp(SYSCON_PDRUNCFG_SYSOSC);
	for ( i = 0; i < 0x200; i++ );
	/* EXT oscillator < 15MHz */
	Chip_Clock_SetPLLBypass(false, false);	
	LPC_SYSCON->EXTCLKSEL = 0x00;
#endif
	
	/* Select the PLL input to the external oscillator */
	Chip_Clock_SetSystemPLLSource(SYSCON_PLLCLKSRC_EXT_CLK);
	
	/* Power down PLL to change the PLL divider ratio */
	Chip_SYSCON_PowerDown(SYSCON_PDRUNCFG_SYSPLL);

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
	
	/* FIXME. Setup FLASH access to 2 clocks (up to 30MHz) */
	Chip_FMC_SetFLASHAccess(FLASHTIM_WS_2);

	/* Set system clock divider to 1 */
	Chip_Clock_SetSysClockDiv(1);

	/* Set main clock source to the system PLL. This will drive 30MHz
	   for the main clock and 30MHz for the system clock */
	Chip_Clock_SetMainClockSource(SYSCON_MAINCLKSRC_EXT_CLK);
	Chip_Clock_SetMainClockPLLSource(SYSCON_MAINCLKPLLSRC_PLLOUT);

	/* Select the CLKOUT clocking source */
	Chip_Clock_SetCLKOUTSource(SYSCON_CLKOUTSRC_EXT_CLK, 1);
}

/* Sets up system pin muxing */
STATIC void SystemSetupMuxing(void)
{
	/* Enable IOCON clock */
	Chip_Clock_EnablePeriphClock(SYSCON_CLOCK_IOCON);

#if (LPC8XX_USE_XTAL_OSC == 1)
#if (LPC8XX_USE_CLKIN == 1)
	Chip_SWM_FixedPinEnable(SWM_FIXED_CLKIN, 1);
	/* Configure the pins for CLKIN. */
	Chip_IOCON_PinSetMode(IOCON_PIO0_1, PIN_MODE_INACTIVE);
#else
	/* Use Switch Matrix Tool swm.c file for the Pin Enable 0 variable */
	Chip_SWM_FixedPinEnable(SWM_FIXED_XTALIN, 1);
	Chip_SWM_FixedPinEnable(SWM_FIXED_XTALOUT, 1);
	/* Configure the pins p0.8 and p0.9 for XTALIN/XTALOUT, neither pull-up nor pull-down. */
	Chip_IOCON_PinSetMode(IOCON_PIO0_8, PIN_MODE_INACTIVE);
	Chip_IOCON_PinSetMode(IOCON_PIO0_9, PIN_MODE_INACTIVE);	
#endif
#endif /* (LPC8XX_USE_XTAL_OSC == 1) */

#if (LPC8XX_USE_CLKIN == 1)
	/* Assign the CLKOUT function to pin p0.18 */
	Chip_SWM_MovablePinAssign(SWM_CLKOUT_O, 18);

	/* Configure the pin for CLKOUT, neither pull-up nor pull-down */
	Chip_IOCON_PinSetMode(IOCON_PIO0_18, PIN_MODE_INACTIVE);
#else
	/* Assign the CLKOUT function to a pin, PIO0_1 */
	Chip_SWM_MovablePinAssign(SWM_CLKOUT_O, 1);		// Conflicts with use of CLKIN

	/* Configure the pin for CLKOUT on PIO0_1, neither pull-up nor pull-down */
	Chip_IOCON_PinSetMode(IOCON_PIO0_1, PIN_MODE_INACTIVE);	
#endif
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* Set up and initialize hardware prior to call to main */
void Board_SystemInit(void)
{
	/* Enable the clock to the Switch Matrix */
	Chip_Clock_EnablePeriphClock(SYSCON_CLOCK_SWM);

	/* Setup system clocking and muxing */
	SystemSetupMuxing();
#if (LPC8XX_USE_XTAL_OSC == 1)
	SystemSetupExtclock();
#else
	SystemSetupClocking();
#endif
	/* Disable the clock to the Switch Matrix to save power */
	Chip_Clock_DisablePeriphClock(SYSCON_CLOCK_SWM);
}
