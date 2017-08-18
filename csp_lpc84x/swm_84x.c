/*
 * @brief LPC84X Pin Switch Matrix driver
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

#define PINASSIGN_IDX(movable)  (((movable) >> 4))
#define PINSHIFT(movable)       (8 * ((movable) & (0xF)))

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/


/*****************************************************************************
 * Private functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* assign a movable pin function to a physical pin */
void Chip_SWM_MovablePinAssign(CHIP_SWM_PIN_MOVABLE_T movable, uint8_t papNum)
{
	uint32_t temp;
	int pinshift = PINSHIFT(movable), regIndex = PINASSIGN_IDX(movable);

	temp = LPC_SWM->PINASSIGN[regIndex] & (~(0xFF << pinshift));
	LPC_SWM->PINASSIGN[regIndex] = temp | (papNum << pinshift);
}


// Disable all fixed pin enables and switch matrix entries for the pin
void Chip_SWM_PinDisable(uint32_t papNum)
{
	uint32_t	regIndex, fieldShift;
	uint32_t temp;

	Chip_SWM_FixedPinFuncDisable(papNum);

	for (regIndex = 0; regIndex < (sizeof(LPC_SWM->PINASSIGN) / sizeof(LPC_SWM->PINASSIGN[0])); regIndex++)
	{
		temp = LPC_SWM->PINASSIGN[regIndex];
		for (fieldShift = 0; fieldShift <= 24; fieldShift += 8)
		{
			if (((temp >> fieldShift) & 0xFF) == papNum)
				LPC_SWM->PINASSIGN[regIndex] |= 0xFF << fieldShift;
		}
	}
}


/* true enables, false disables a Switch Matrix fixed-pin Function */
void Chip_SWM_FixedPinEnable(CHIP_SWM_PIN_FIXED_T swmPin, bool enable)
{
	if (enable) {
		if ( swmPin < 32 ) {
			LPC_SWM->PINENABLE0 &= ~(1 << (uint32_t) swmPin);
		}
		else {
			LPC_SWM->PINENABLE1 &= ~(1 << (uint32_t) swmPin);
		}
	}
	else {
		if ( swmPin < 32 ) {
			LPC_SWM->PINENABLE0 |= (1 << (uint32_t) swmPin);
		}
		else {
			LPC_SWM->PINENABLE1 |= (1 << (uint32_t) swmPin);
		}
	}
}


// Disable all fixed pin functions associated with pin
void Chip_SWM_FixedPinFuncDisable(uint32_t papNum)
{
// Table, indexed by pin number, of fixed function pin enable bits (PINENABLE0 and PINENABLE1)
static const uint32_t fixedFuncPinEnTable[] = {
 (1 << SWM_FIXED_ACMP_I0),		// PIO0_0
 (1 << SWM_FIXED_CLKIN) | (1 << SWM_FIXED_ACMP_I1),		// PIO0_1
 (1 << SWM_FIXED_SWDIO),		// PIO0_2
 (1 << SWM_FIXED_SWCLK),		// PIO0_3
 (1 << SWM_FIXED_ADC_11),		// PIO0_4
 (1 << SWM_FIXED_RESETN),		// PIO0_5
 (1 << SWM_FIXED_ADC_1) | (1 << SWM_FIXED_VDDCMP),		// PIO0_6
 (1 << SWM_FIXED_ADC_0),		// PIO0_7
 (1 << SWM_FIXED_XTALIN),		// PIO0_8
 (1 << SWM_FIXED_XTALOUT),		// PIO0_9
 (1 << SWM_FIXED_I2C0_SCL),		// PIO0_10
 (1 << SWM_FIXED_I2C0_SDA),		// PIO0_11
 0,		// PIO0_12
 (1 << SWM_FIXED_ADC_10),		// PIO0_13
 (1 << SWM_FIXED_ADC_2) | (1 << SWM_FIXED_ACMP_I2),		// PIO0_14
 0,		// PIO0_15
 0,		// PIO0_16
 (1 << SWM_FIXED_ADC_9) | (1 << SWM_FIXED_DAC_0),		// PIO0_17
 (1 << SWM_FIXED_ADC_8),		// PIO0_18
 (1 << SWM_FIXED_ADC_7),		// PIO0_19
 (1 << SWM_FIXED_ADC_6),		// PIO0_20
 (1 << SWM_FIXED_ADC_5),		// PIO0_21
 (1 << SWM_FIXED_ADC_4),		// PIO0_22
 (1 << SWM_FIXED_ADC_3) | (1 << SWM_FIXED_ACMP_I3),		// PIO0_23
 0,		// PIO0_24
 0,		// PIO0_25
 0,		// PIO0_26
 0,		// PIO0_27
 0,		// PIO0_28
 (1 << SWM_FIXED_DAC_1),		// PIO0_29
 (1 << SWM_FIXED_ACMP_I4),	// PIO0_30
 (1 << SWM_FIXED_CAPT_X0),		// PIO0_31

 (1 << SWM_FIXED_CAPT_X1),		// PIO1_0
 (1 << SWM_FIXED_CAPT_X2),		// PIO1_1
 (1UL << SWM_FIXED_CAPT_X3),		// PIO1_2

 (1 << (SWM_FIXED_CAPT_X4-32)),		// PIO1_3
 (1 << (SWM_FIXED_CAPT_X5-32)),		// PIO1_4
 (1 << (SWM_FIXED_CAPT_X6-32)),		// PIO1_5
 (1 << (SWM_FIXED_CAPT_X7-32)),		// PIO1_6
 (1 << (SWM_FIXED_CAPT_X8-32)),		// PIO1_7
 (1 << (SWM_FIXED_CAPT_YL-32)),		// PIO1_8
 (1 << (SWM_FIXED_CAPT_YH-32)),		// PIO1_9
};

	if ( papNum < 32 ) {
		LPC_SWM->PINENABLE0 |= fixedFuncPinEnTable[papNum];		// Set to disable function
	}
	else {
		LPC_SWM->PINENABLE1 |= fixedFuncPinEnTable[papNum];		// Set to disable function
	}
}




