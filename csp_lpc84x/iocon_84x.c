/*
 * @brief LPC84X IOCON driver
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

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/
/* Set the pin mode (pull-up/pull-down). */
void Chip_IOCON_PinSetMode(CHIP_PINx_T pin, CHIP_PIN_MODE_T mode)
{
	uint32_t reg;

	reg = LPC_IOCON->PIO[pin] & ~(PIN_MODE_MASK);
	LPC_IOCON->PIO[pin] = reg | (mode << PIN_MODE_BITNUM);
}

/* Enables/disables the pin hysteresis. */
void Chip_IOCON_PinSetHysteresis(CHIP_PINx_T pin, bool enable)
{
	if (enable == true) {
		LPC_IOCON->PIO[pin] |= PIN_HYS_MASK;
	}
	else {
		LPC_IOCON->PIO[pin] &= ~PIN_HYS_MASK;
	}
}

/*Inverts (or not) the input seen by a pin. */
void Chip_IOCON_PinSetInputInverted(CHIP_PINx_T pin, bool invert)
{
	if (invert == true) {
		LPC_IOCON->PIO[pin] |= PIN_INV_MASK;
	}
	else {
		LPC_IOCON->PIO[pin] &= ~PIN_INV_MASK;
	}
}

/* Enables/disables Open-Drain mode for a pin. */
void Chip_IOCON_PinSetOpenDrainMode(CHIP_PINx_T pin, bool open_drain)
{
	if (open_drain == true) {
		LPC_IOCON->PIO[pin] |= PIN_OD_MASK;
	}
	else {
		LPC_IOCON->PIO[pin] &= ~PIN_OD_MASK;
	}
}

/* Enable/configure digital filter sample mode for a pin. */
void Chip_IOCON_PinSetSampleMode(CHIP_PINx_T pin, CHIP_PIN_SMODE_T smode)
{
	uint32_t reg;

	reg = LPC_IOCON->PIO[pin] & ~(PIN_SMODE_MASK);
	LPC_IOCON->PIO[pin] = reg | (smode << PIN_SMODE_BITNUM);
}

/* Set the peripheral clock divisor for a pin. */
void Chip_IOCON_PinSetClockDivisor(CHIP_PINx_T pin, CHIP_PIN_CLKDIV_T clkdiv)
{
	uint32_t reg;

	reg = LPC_IOCON->PIO[pin] & ~(IOCON_PIO_CLKDIVSEL_M);
	LPC_IOCON->PIO[pin] = reg | (clkdiv << IOCON_PIO_CLKDIVSEL_P);
}

/* Set the I2C mode for a pin. */
void Chip_IOCON_PinSetI2CMode(CHIP_PINx_T pin, CHIP_PIN_I2CMODE_T mode)
{
	uint32_t reg;

	/* I2C mode bits only for I2C pins */
	reg = LPC_IOCON->PIO[pin] & ~(PIN_I2CMODE_MASK);
	LPC_IOCON->PIO[pin] = reg | (mode << PIN_I2CMODE_BITNUM);
}
