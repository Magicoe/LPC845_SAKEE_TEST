/*
 * @brief LPC84X clock driver
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

/* Inprecise clock rates for the watchdog oscillator */
static const uint32_t wdtOSCRate[WDTLFO_OSC_4_60 + 1] = {
	0,					/* WDT_OSC_ILLEGAL */
	600000,				/* WDT_OSC_0_60 */
	1050000,			/* WDT_OSC_1_05 */
	1400000,			/* WDT_OSC_1_40 */
	1750000,			/* WDT_OSC_1_75 */
	2100000,			/* WDT_OSC_2_10 */
	2400000,			/* WDT_OSC_2_40 */
	2700000,			/* WDT_OSC_2_70 */
	3000000,			/* WDT_OSC_3_00 */
	3250000,			/* WDT_OSC_3_25 */
	3500000,			/* WDT_OSC_3_50 */
	3750000,			/* WDT_OSC_3_75 */
	4000000,			/* WDT_OSC_4_00 */
	4200000,			/* WDT_OSC_4_20 */
	4400000,			/* WDT_OSC_4_40 */
	4600000				/* WDT_OSC_4_60 */
};

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Compute a PLL frequency */
static uint32_t Chip_Clock_GetPLLFreq(uint32_t PLLReg, uint32_t inputRate)
{
	uint32_t msel = ((PLLReg & 0x1F) + 1);

	return inputRate * msel;
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* Set System PLL clock source */
void Chip_Clock_SetSystemPLLSource(CHIP_SYSCON_PLLCLKSRC_T src)
{
	LPC_SYSCON->SYSPLLCLKSEL  = (uint32_t) src;
	LPC_SYSCON->SYSPLLCLKUEN  = 0;
	LPC_SYSCON->SYSPLLCLKUEN  = 1;
}

/* Bypass System Oscillator and set oscillator frequency range */
void Chip_Clock_SetPLLBypass(bool bypass, bool highfr)
{
	uint32_t ctrl = 0;

	if (bypass) {
		ctrl |= (1 << 0);
	}
	if (highfr) {
		ctrl |= (1 << 1);
	}

	LPC_SYSCON->SYSOSCCTRL = ctrl;
}

/* Set main system clock source */
void Chip_Clock_SetMainClockSource(CHIP_SYSCON_MAINCLKSRC_T src)
{
	LPC_SYSCON->MAINCLKSEL  = (uint32_t) src;
	LPC_SYSCON->MAINCLKUEN  = 0;
	LPC_SYSCON->MAINCLKUEN  = 1;
}

/* Set main system clock pll source */
void Chip_Clock_SetMainClockPLLSource(CHIP_SYSCON_MAINCLKPLLSRC_T src)
{
	LPC_SYSCON->MAINCLKPLLSEL  = (uint32_t) src;
	LPC_SYSCON->MAINCLKPLLUEN  = 0;
	LPC_SYSCON->MAINCLKPLLUEN  = 1;
}

/* Set CLKOUT clock source and divider */
void Chip_Clock_SetCLKOUTSource(CHIP_SYSCON_CLKOUTSRC_T src, uint32_t div)
{
	LPC_SYSCON->CLKOUTSEL = (uint32_t) src;
	LPC_SYSCON->CLKOUTDIV = div;
}

/* Return estimated watchdog oscillator rate */
uint32_t Chip_Clock_GetWDTOSCRate(void)
{
	uint32_t div;
	CHIP_WDTLFO_OSC_T clk;

	/* Get WDT oscillator settings */
	clk = (CHIP_WDTLFO_OSC_T) ((LPC_SYSCON->WDTOSCCTRL >> 5) & 0xF);
	div = LPC_SYSCON->WDTOSCCTRL & 0x1F;

	/* Compute clock rate and divided by divde value */
	return wdtOSCRate[clk] / ((div + 1) << 1);
}

/* Return System PLL input clock rate */
uint32_t Chip_Clock_GetSystemPLLInClockRate(void)
{
	uint32_t clkRate;

	switch ((CHIP_SYSCON_PLLCLKSRC_T) (LPC_SYSCON->SYSPLLCLKSEL & 0x3)) {
	case SYSCON_PLLCLKSRC_FRO:
		clkRate = Chip_Clock_GetIntOscRate();
		break;

	case SYSCON_PLLCLKSRC_EXT_CLK:
		clkRate = Chip_Clock_GetExtOscRate();
		break;

	case SYSCON_PLLCLKSRC_WDTOSC:
		clkRate = Chip_Clock_GetWDTOSCRate();
		break;

	case SYSCON_PLLCLKSRC_FRO_DIV:
	/* to-do list: if FAIM is configured and FRO divider is on. */
//		clkRate = Chip_Clock_GetIntOscRate();
		break;
	}

	return clkRate;
}

/* Return System PLL output clock rate */
uint32_t Chip_Clock_GetSystemPLLOutClockRate(void)
{
	return Chip_Clock_GetPLLFreq(LPC_SYSCON->SYSPLLCTRL,
								 Chip_Clock_GetSystemPLLInClockRate());
}

/* Return main clock rate */
uint32_t Chip_Clock_GetMainClockRate(void)
{
	uint32_t clkRate = 0;

	switch ((CHIP_SYSCON_MAINCLKSRC_T) (LPC_SYSCON->MAINCLKSEL & 0x3)) {
	case SYSCON_MAINCLKSRC_FRO:
		clkRate = Chip_Clock_GetIntOscRate();
		break;

	case SYSCON_MAINCLKSRC_EXT_CLK:
		clkRate = Chip_Clock_GetSystemPLLInClockRate();
		break;

	case SYSCON_MAINCLKSRC_WDTOSC:
		clkRate = Chip_Clock_GetWDTOSCRate();
		break;

	case SYSCON_MAINCLKSRC_FRO_DIV:
	/* to-do list: if FAIM is configured and FRO divider is on. */
//		clkRate = Chip_Clock_GetIntOscRate()/;
		break;
	}
	return clkRate;
}

/* Return main clock rate */
uint32_t Chip_Clock_GetMainClockPLLRate(void)
{
	uint32_t clkRate = 0;

	switch ((CHIP_SYSCON_MAINCLKPLLSRC_T) (LPC_SYSCON->MAINCLKPLLSEL & 0x3)) {
	case SYSCON_MAINCLKPLLSRC_MAINCLKOUT:
		clkRate = Chip_Clock_GetMainClockRate();
		break;

	case SYSCON_MAINCLKPLLSRC_PLLOUT:
		clkRate = Chip_Clock_GetSystemPLLOutClockRate();
		break;
	}
	return clkRate;
}

/* Return system clock rate */
uint32_t Chip_Clock_GetSystemClockRate(void)
{
	/* No point in checking for divide by 0 */
	return Chip_Clock_GetMainClockPLLRate() / LPC_SYSCON->SYSAHBCLKDIV;
}

/* Get the base clock frequency of Fractional divider */
uint32_t Chip_Clock_GetFRGInClockRate(uint8_t frg_num)
{
	uint32_t inclk;
	CHIP_SYSCON_FRGCLKSRC_T src = Chip_Clock_GetFRGClockSource(frg_num);
	/* Detect the input clock frequency */
	switch(src) {
		case SYSCON_FRGCLKSRC_FRO:
			inclk = Chip_Clock_GetIntOscRate();
			break;
		case SYSCON_FRGCLKSRC_MAINCLK:
			inclk = Chip_Clock_GetMainClockRate();
			break;
		case SYSCON_FRGCLKSRC_PLL:
			inclk = Chip_Clock_GetSystemPLLOutClockRate();
			break;
		default:
			return 0;
	}
	return inclk;
}

/* Get UART base rate */
uint32_t Chip_Clock_GetFRGClockRate(uint8_t frg_num)
{
	uint64_t inclk;
	uint32_t mult, div;

	/* Get clock rate into FRG */
	inclk = Chip_Clock_GetFRGInClockRate(frg_num);

	if (inclk != 0) {
		if (frg_num == 0) {
			mult = LPC_SYSCON->FRG0MULT;
			div = LPC_SYSCON->FRG0DIV;
		}
		else {
			mult = LPC_SYSCON->FRG1MULT;
			div = LPC_SYSCON->FRG1DIV;
		}
		if ((div & 0xFF) == 0xFF) {
			/* Fractional part is enabled, get multiplier */
			mult &= 0xFF;

			/* Get fractional error */
			inclk = (inclk * 256) / (uint64_t) (256 + mult);
		}
	}

	return (uint32_t) inclk;
}

/* Set UART base rate */
uint32_t Chip_Clock_SetFRGClockRate(uint8_t frg_num, uint32_t rate)
{

	uint32_t div, inclk, err;
	uint64_t uart_fra_multiplier;

	/* Input clock into FRG block is the main system cloock */
	inclk = Chip_Clock_GetFRGInClockRate(frg_num);
	if (!inclk) {
		return 0;
	}

	/* Get integer divider for coarse rate */
	div = inclk / rate;
	if (div == 0) {
		div = 1;
	}

	err = inclk - (rate * div);
	uart_fra_multiplier = (((uint64_t) err + (uint64_t) rate) * 256) / (uint64_t) (rate * div);

	/* Enable fractional divider and set multiplier */
	if ( frg_num == 0 ) {
		LPC_SYSCON->FRG0DIV = 0xFF;
		LPC_SYSCON->FRG0DIV = uart_fra_multiplier;
	}
	else {
		LPC_SYSCON->FRG1DIV = 0xFF;
		LPC_SYSCON->FRG1DIV = uart_fra_multiplier;
	}

	return Chip_Clock_GetFRGClockRate(frg_num);
}

/* Return Fclock clock rate */
uint32_t Chip_Clock_GetFClockRate(uint32_t id)
{
	uint32_t clkRate = 0;

	switch ((CHIP_SYSCON_FCLKSELSRC_T)Chip_Clock_GetFClockSource(id)) {
		case SYSCON_FLEXCOMMCLKSELSRC_FRO:
			clkRate = Chip_Clock_GetIntOscRate();
			break;

		case SYSCON_FLEXCOMMCLKSELSRC_MAINCLK:
			clkRate = Chip_Clock_GetMainClockRate();
			break;

		case SYSCON_FLEXCOMMCLKSELSRC_FRG0:
			clkRate = Chip_Clock_GetFRGClockRate(0);
			break;

		case SYSCON_FLEXCOMMCLKSELSRC_FRG1:
			clkRate = Chip_Clock_GetFRGClockRate(1);
			break;

		case SYSCON_FLEXCOMMCLKSELSRC_FRO_DIV:
	/* to-do list: if FAIM is configured and FRO divider is on. */
//		clkRate = Chip_Clock_GetIntOscRate()/;
			break;
	}	
	return clkRate;
}
