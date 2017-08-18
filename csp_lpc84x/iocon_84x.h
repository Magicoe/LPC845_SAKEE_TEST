/*
 * @brief LPC8xx IOCON register block and driver
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

#ifndef __IOCON_84X_H_
#define __IOCON_84X_H_

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup IOCON_84X CHIP: LPC8xx IOCON register block and driver
 * @ingroup CHIP_84X_Drivers
 * @{
 */

#define NUM_IOCON_PIO  (64)		/* Port 0 and 1 including all reserved port pins */

/**
 * @brief	IOCON register block structure
 * @note	When accessing this register structure, use the PIOn enumeration
 * as the array index as the pin assignments are not mapped 1-1 with the
 * IOCON structure.<br>
 * Incorrect: LPC_IOCON->PIO0[0] = 0x1; // Index 0 does not map to pin 0!<br>
 * Correct: LPC_IOCON->PIO0[IOCON_PIO0] = 0x1; // Enumeration PIO0 maps to pin 0
 */
typedef struct {		/*!< (@ 0x40044000) IOCONFIG Structure     */
	__IO uint32_t PIO[NUM_IOCON_PIO];
} LPC_IOCON_T;

/**
 * @brief IOCON Register bit definitions
 */

#define IOCON_PIO_MODE_P							3
#define IOCON_PIO_MODE_M							(0x3 << IOCON_PIO_MODE_P)
#define IOCON_PIO_HYS_P								5
#define IOCON_PIO_HYS_M								(0x1 << IOCON_PIO_HYS_P)
#define IOCON_PIO_INV_P								6
#define IOCON_PIO_INV_M								(0x1 << IOCON_PIO_INV_P)
#define IOCON_PIO_I2CMODE_P						8									// For PIO10 and PIO11 only
#define IOCON_PIO_I2CMODE_M						(0x3 << IOCON_PIO_I2CMODE_P)
#define IOCON_PIO_OD_P								10
#define IOCON_PIO_OD_M								(0x1 << IOCON_PIO_OD_P)
#define IOCON_PIO_SMODE_P							11
#define IOCON_PIO_SMODE_M							(0x3 << IOCON_PIOSMODE_P)
#define IOCON_PIO_CLKDIVSEL_P					13
#define IOCON_PIO_CLKDIVSEL_M					(0x7 << IOCON_PIO_CLKDIVSEL_P)


/* Pin Mode mask */
#define PIN_MODE_MASK           (0x3 <<  3)
#define PIN_MODE_BITNUM         (3)

/* Pin Hysteresis mask */
#define PIN_HYS_MASK            (0x1 <<  5)
#define PIN_HYS_BITNUM          (5)

/* Pin invert input mask */
#define PIN_INV_MASK            (0x1 <<  6)
#define PIN_INV_BITNUM          (6)

/* Pin open drain mode mask */
#define PIN_OD_MASK             (0x1 << 10)
#define PIN_OD_BITNUM           (10)

/* Pin digital filter sample mode mask */
#define PIN_SMODE_MASK          (0x3 << 11)
#define PIN_SMODE_BITNUM        (11)

/* Pin clock divider mask */
#define PIN_CLKDIV_MASK         (0x7 << 13)
#define PIN_CLKDIV_BITNUM       (13)

/* Pin I2C mode mask - valid for PIO10 & PIO11 only */
#define PIN_I2CMODE_MASK        (0x3 <<  8)
#define PIN_I2CMODE_BITNUM      (8)

/**
 * @brief IOCON Pin Numbers enum
 * Maps a pin number to an IOCON (register) array index. IOCON indexes
 * are not mapped 1-1 with pin numbers. When access the PIO0 array in
 * the LPC_IOCON_T structure, the array should be indexed with one of
 * these enumerations based on the pin that will have it's settings
 * changed.<br>
 * Example: LPC_IOCON->PIO0[IOCON_PIO0] = 0x1; // Enumeration PIO0 maps to pin 0
 */
typedef enum CHIP_PINx {
	IOCON_PIO0_0  =  0x11,	/*!< P0.0 */
	IOCON_PIO0_1  =  0x0B,	/*!< P0.1 */
	IOCON_PIO0_2  =  0x06,	/*!< P0.2 */
	IOCON_PIO0_3  =  0x05,	/*!< P0.3 */
	IOCON_PIO0_4  =  0x04,	/*!< P0.4 */
	IOCON_PIO0_5  =  0x03,	/*!< P0.5 */
	/* The following pins are not present in DIP8 packages */
	IOCON_PIO0_6  =  0x10,	/*!< P0.6 */
	IOCON_PIO0_7  =  0x0F,	/*!< P0.7 */
	IOCON_PIO0_8  =  0x0E,	/*!< P0.8 */
	IOCON_PIO0_9  =  0x0D,	/*!< P0.9 */
	IOCON_PIO0_10 =  0x08,	/*!< P0.10 */
	IOCON_PIO0_11 =  0x07,	/*!< P0.11 */
	IOCON_PIO0_12 =  0x02,	/*!< P0.12 */
	IOCON_PIO0_13 =  0x01,	/*!< P0.13 */
	/* The following pins are not present in DIP8 & TSSOP16 packages */
	IOCON_PIO0_14 =  0x12,	/*!< P0.14 */
	IOCON_PIO0_15 =  0x0A,	/*!< P0.15 */
	IOCON_PIO0_16 =  0x09,	/*!< P0.16 */
	IOCON_PIO0_17 =  0x00,	/*!< P0.17 */	
	IOCON_PIO0_18 =  0x1E,	/*!< P0.18 */
	IOCON_PIO0_19 =  0x1D,	/*!< P0.19 */
	IOCON_PIO0_20 =  0x1C,	/*!< P0.20 */
	IOCON_PIO0_21 =  0x1B,	/*!< P0.21 */
	IOCON_PIO0_22 =  0x1A,	/*!< P0.22 */
	IOCON_PIO0_23 =  0x19,	/*!< P0.23 */
	IOCON_PIO0_24 =  0x18,	/*!< P0.24 */
	IOCON_PIO0_25 =  0x17,	/*!< P0.25 */
	IOCON_PIO0_26 =  0x16,	/*!< P0.26 */
	IOCON_PIO0_27 =  0x15,	/*!< P0.27 */
	IOCON_PIO0_28 =  0x14,	/*!< P0.28 */		
	IOCON_PIO0_29 =  0x32,	/*!< P0.29 */		
	IOCON_PIO0_30 =  0x33,	/*!< P0.30 */
	IOCON_PIO0_31 =  0x23,	/*!< P0.31 */

	IOCON_PIO1_0  =  0x24,	/*!< P1.0 */
	IOCON_PIO1_1  =  0x25,	/*!< P1.1 */
	IOCON_PIO1_2  =  0x26,	/*!< P1.2 */
	IOCON_PIO1_3  =  0x29,	/*!< P1.3 */
	IOCON_PIO1_4  =  0x2A,	/*!< P1.4 */
	IOCON_PIO1_5  =  0x2B,	/*!< P1.5 */
	IOCON_PIO1_6  =  0x2E,	/*!< P1.6 */
	IOCON_PIO1_7  =  0x31,	/*!< P1.7 */
	IOCON_PIO1_8  =  0x1F,	/*!< P1.8 */
	IOCON_PIO1_9  =  0x20,	/*!< P1.9 */
	IOCON_PIO1_10 =  0x37,	/*!< P1.10 */
	IOCON_PIO1_11 =  0x36,	/*!< P1.11 */
	IOCON_PIO1_12 =  0x21,	/*!< P1.12 */
	IOCON_PIO1_13 =  0x22,	/*!< P1.13 */
	IOCON_PIO1_14 =  0x27,	/*!< P1.14 */
	IOCON_PIO1_15 =  0x28,	/*!< P1.15 */
	IOCON_PIO1_16 =  0x2C,	/*!< P1.16 */
	IOCON_PIO1_17 =  0x2D,	/*!< P1.17 */	
	IOCON_PIO1_18 =  0x2F,	/*!< P1.18 */
	IOCON_PIO1_19 =  0x30,	/*!< P1.19 */
	IOCON_PIO1_20 =  0x34,	/*!< P1.20 */
	IOCON_PIO1_21 =  0x35		/*!< P1.21 */
} CHIP_PINx_T;

/**
 * @brief IOCON Pin Modes enum
 */
typedef enum CHIP_PIN_MODE {
	PIN_MODE_INACTIVE = 0,	/*!< Inactive mode */
	PIN_MODE_PULLDN = 1,	/*!< Pull Down mode */
	PIN_MODE_PULLUP = 2,	/*!< Pull up mode */
	PIN_MODE_REPEATER = 3	/*!< Repeater mode */
} CHIP_PIN_MODE_T;

/**
 * @brief IOCON Digital Filter Sample modes enum
 */
typedef enum CHIP_PIN_SMODE {
	PIN_SMODE_BYPASS = 0,	/*!< Bypass input filter */
	PIN_SMODE_CYC1 = 1,		/*!< Input pulses shorter than 1 filter clock cycle are rejected */
	PIN_SMODE_CYC2 = 2,		/*!< Input pulses shorter than 2 filter clock cycles are rejected */
	PIN_SMODE_CYC3 = 3		/*!< Input pulses shorter than 3 filter clock cycles are rejected */
} CHIP_PIN_SMODE_T;

/**
 * @brief IOCON Perpipheral Clock divider selction for input filter
 * sampling clock
 */
typedef enum CHIP_PIN_CLKDIV {
	IOCONCLKDIV0 = 0,	/*!< Clock divider register 0 */
	IOCONCLKDIV1 = 1,	/*!< Clock divider register 1 */
	IOCONCLKDIV2 = 2,	/*!< Clock divider register 2 */
	IOCONCLKDIV3 = 3,	/*!< Clock divider register 3 */
	IOCONCLKDIV4 = 4,	/*!< Clock divider register 4 */
	IOCONCLKDIV5 = 5,	/*!< Clock divider register 5 */
	IOCONCLKDIV6 = 6	/*!< Clock divider register 6 */
} CHIP_PIN_CLKDIV_T;

/**
 * @brief IOCON I2C Modes enum (Only for I2C pins PIO0_10 and PIO0_11)
 */
typedef enum CHIP_PIN_I2CMODE {
	PIN_I2CMODE_STDFAST = 0,	/*!< I2C standard mode/Fast mode */
	PIN_I2CMODE_GPIO = 1,		/*!< Standard I/O functionality */
	PIN_I2CMODE_FASTPLUS = 2	/*!< I2C Fast plus mode */
} CHIP_PIN_I2CMODE_T;

/**
 * @brief	Sets I/O Control pin mux
 * @param	pIOCON		: The base of IOCON peripheral on the chip
 * @param	pin			: GPIO pin to mux
 * @param	modefunc	: OR'ed values or type IOCON_*
 * @return	Nothing
 */
STATIC INLINE void Chip_IOCON_PinMuxSet(CHIP_PINx_T pin, uint32_t modefunc)
{
	LPC_IOCON->PIO[pin] = modefunc;
}

/**
 * @brief	Sets pull-up or pull-down mode for a pin
 * @param	pIOCON	: The base of IOCON peripheral on the chip
 * @param	pin		: Pin number
 * @param	mode	: Mode (Pull-up/Pull-down mode)
 * @return	Nothing
 * @note	Do not use with pins PIO10 and PIO11.
 */
void Chip_IOCON_PinSetMode(CHIP_PINx_T pin, CHIP_PIN_MODE_T mode);

/**
 * @brief	Enable or disable hysteresis for a pin
 * @param	pIOCON	: The base of IOCON peripheral on the chip
 * @param	pin		: Pin number
 * @param	enable	: true to enable, false to disable
 * @return	Nothing
 * @note	Do not use with pins PIO10 and PIO11.
 */
void Chip_IOCON_PinSetHysteresis(CHIP_PINx_T pin, bool enable);

/**
 * @brief	Enable or disable invert input for a pin
 * @param	pIOCON	: The base of IOCON peripheral on the chip
 * @param	pin		: Pin number
 * @param	invert	: true to invert, false to not to invert
 * @return	Nothing
 */
void Chip_IOCON_PinSetInputInverted(CHIP_PINx_T pin, bool invert);

/**
 * @brief	Enables or disables open-drain mode for a pin
 * @param	pIOCON		: The base of IOCON peripheral on the chip
 * @param	pin			: CHIP_PINx_T Pin number
 * @param	open_drain	: true to enable open-drain mode,
 *                        false to disable open-drain mode
 * @return	Nothing
 */
void Chip_IOCON_PinSetOpenDrainMode(CHIP_PINx_T pin, bool open_drain);

/**
 * @brief	Sets the digital filter sampling mode for a pin
 * @param	pIOCON	: The base of IOCON peripheral on the chip
 * @param	pin		: Pin number
 * @param	smode	: 0x0 = bypass, 0x[1..3] = 1 to 3 clock cycles.
 * @return	Nothing
 */
void Chip_IOCON_PinSetSampleMode(CHIP_PINx_T pin, CHIP_PIN_SMODE_T smode);

/**
 * @brief	Select peripheral clock divider for input filter sampling clock
 * @param	pIOCON	: The base of IOCON peripheral on the chip
 * @param	pin		: Pin number
 * @param	clkdiv	: 0 = no divisor, 1...6 = PCLK/clkdiv
 * @return	Nothing
 */
void Chip_IOCON_PinSetClockDivisor(CHIP_PINx_T pin, CHIP_PIN_CLKDIV_T clkdiv);

/**
 * @brief	Set I2C mode for a pin
 * @param	pIOCON	: The base of IOCON peripheral on the chip
 * @param	pin		: Pin number
 * @param	mode	: 0:Standard/Fast I2C 1: GPIO 2: Fast Plus
 * @return	Nothing
 * @note	Valid for pins PIO0_10 and PIO0_11 only.
 */
void Chip_IOCON_PinSetI2CMode(CHIP_PINx_T pin, CHIP_PIN_I2CMODE_T mode);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __IOCON_84X_H_ */
