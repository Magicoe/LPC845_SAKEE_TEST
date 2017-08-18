/*
 * @brief LPC8xx Switch Matrix driver
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

#ifndef __SWM_84X_H_
#define __SWM_84X_H_

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup SWM_84X CHIP: LPC8xx Switch Matrix Driver
 * @ingroup CHIP_84X_Drivers
 * @{
 */

/**
 * @brief LPC8XX Switch Matrix register block structure
 */
typedef struct {
	__IO uint32_t PINASSIGN[15];		/*!< Pin Assign register array */
	__I  uint32_t RESERVED0[97];
	__IO uint32_t PINENABLE0;		/*!< Pin Enable register */
	__IO uint32_t PINENABLE1;		/*!< Pin Enable register */
} LPC_SWM_T;

/**
 * @brief LPC8XX Switch Matrix Movable pins (works for LPC82x)
 */
typedef enum CHIP_SWM_PIN_MOVABLE  {
	SWM_U0_TXD_O        = 0x00,	/*!< PINASSIGN0 - UART0 TXD Output */
	SWM_U0_RXD_I        = 0x01,	/*!< PINASSIGN0 - UART0 RXD Input */
	SWM_U0_RTS_O        = 0x02,	/*!< PINASSIGN0 - UART0 RTS Output */
	SWM_U0_CTS_I        = 0x03,	/*!< PINASSIGN0 - UART0 CTS Input */
	SWM_U0_SCLK_IO      = 0x10,	/*!< PINASSIGN1 - UART0 SCLK I/O */
	SWM_U1_TXD_O        = 0x11,	/*!< PINASSIGN1 - UART1 TXD Output */
	SWM_U1_RXD_I        = 0x12,	/*!< PINASSIGN1 - UART1 RXD Input */
	SWM_U1_RTS_O        = 0x13,	/*!< PINASSIGN1 - UART1 RTS Output */
	SWM_U1_CTS_I        = 0x20,	/*!< PINASSIGN2 - UART1 CTS Input */
	SWM_U1_SCLK_IO      = 0x21,	/*!< PINASSIGN2 - UART1 SCLK I/O */
	SWM_U2_TXD_O        = 0x22,	/*!< PINASSIGN2 - UART2 TXD Output */
	SWM_U2_RXD_I        = 0x23,	/*!< PINASSIGN2 - UART2 RXD Input */
	SWM_U2_RTS_O        = 0x30,	/*!< PINASSIGN3 - UART2 RTS Output */
	SWM_U2_CTS_I        = 0x31,	/*!< PINASSIGN3 - UART2 CTS Input */
	SWM_U2_SCLK_IO      = 0x32,	/*!< PINASSIGN3 - UART2 SCLK I/O */
	SWM_SPI0_SCK_IO     = 0x33,	/*!< PINASSIGN3 - SPI0 SCK I/O */
	SWM_SPI0_MOSI_IO    = 0x40,	/*!< PINASSIGN4 - SPI0 MOSI I/O */
	SWM_SPI0_MISO_IO    = 0x41,	/*!< PINASSIGN4 - SPI0 MISO I/O */
	SWM_SPI0_SSEL0_IO   = 0x42,	/*!< PINASSIGN4 - SPI0 SSEL0 I/O */
	SWM_SPI0_SSEL1_IO   = 0x43,	/*!< PINASSIGN4 - SPI0 SSEL1 I/O */
	SWM_SPI0_SSEL2_IO   = 0x50,	/*!< PINASSIGN5 - SPI0 SSEL2 I/O */
	SWM_SPI0_SSEL3_IO   = 0x51,	/*!< PINASSIGN5 - SPI0 SSEL3 I/O */
	SWM_SPI1_SCK_IO     = 0x52,	/*!< PINASSIGN5 - SPI1 SCK I/O */
	SWM_SPI1_MOSI_IO    = 0x53,	/*!< PINASSIGN5 - SPI1 MOSI I/O */
	SWM_SPI1_MISO_IO    = 0x60,	/*!< PINASSIGN6 - SPI1 MISO I/O */
	SWM_SPI1_SSEL0_IO   = 0x61,	/*!< PINASSIGN6 - SPI1 SSEL0 I/O */
	SWM_SPI1_SSEL1_IO   = 0x62,	/*!< PINASSIGN6 - SPI1 SSEL1 I/O */	
	SWM_SCTIN_0_I        = 0x63,	/*!< PINASSIGN6 - CTIN0 Input */
	SWM_SCTIN_1_I        = 0x70,	/*!< PINASSIGN7 - CTIN1 Input */
	SWM_SCTIN_2_I        = 0x71,	/*!< PINASSIGN7 - CTIN2 Input */
	SWM_SCTIN_3_I        = 0x72,	/*!< PINASSIGN7 - CTIN3 Input */
	SWM_SCTOUT_0_O       = 0x73,	/*!< PINASSIGN7 - CTOUT0 Output */
	SWM_SCTOUT_1_O       = 0x80,	/*!< PINASSIGN8 - CTOUT1 Output */
	SWM_SCTOUT_2_O       = 0x81,	/*!< PINASSIGN8 - CTOUT2 Output */
	SWM_SCTOUT_3_O       = 0x82,	/*!< PINASSIGN8 - CTOUT3 Output */
	SWM_SCTOUT_4_O       = 0x83,	/*!< PINASSIGN8 - CTOUT4 Output */
	SWM_SCTOUT_5_O       = 0x90,	/*!< PINASSIGN9 - CTOUT5 Output */
	SWM_SCTOUT_6_O       = 0x91,	/*!< PINASSIGN9 - CTOUT6 Output */
	SWM_I2C1_SDA_IO     = 0x92,	/*!< PINASSIGN9 - I2C1 SDA I/O */
	SWM_I2C1_SCL_IO     = 0x93,	/*!< PINASSIGN9 - I2C1 SCL I/O */
	SWM_I2C2_SDA_IO     = 0xA0,	/*!< PINASSIGN10 - I2C2 SDA I/O */
	SWM_I2C2_SCL_IO     = 0xA1,	/*!< PINASSIGN10 - I2C2 SCL I/O */
	SWM_I2C3_SDA_IO     = 0xA2,	/*!< PINASSIGN10 - I2C3 SDA I/O */
	SWM_I2C3_SCL_IO     = 0xA3,	/*!< PINASSIGN10 - I2C3 SCL I/O */
	SWM_COMP0_0         = 0xB0,	/*!< PINASSIGN11 - COMP0 output */
	SWM_CLKOUT_O        = 0xB1,	/*!< PINASSIGN11 - CLKOUT Output */
	SWM_GPIO_INT_BMAT_O = 0xB3,	/*!< PINASSIGN11 - GPIO INT BMAT Output */
	SWM_U3_TXD_O        = 0xB4,	/*!< PINASSIGN11 - UART3 TXD Output */
	SWM_U3_RXD_I        = 0xC0,	/*!< PINASSIGN12 - UART3 RXD Input */
	SWM_U3_SCK_IO       = 0xC1,	/*!< PINASSIGN12 - UART3 SCLK I/O */
	SWM_U4_TXD_O        = 0xC2,	/*!< PINASSIGN12 - UART4 TXD Output */
	SWM_U4_RXD_I        = 0xC3,	/*!< PINASSIGN12 - UART4 RXD Input */
	SWM_U4_SCK_IO       = 0xD0,	/*!< PINASSIGN13 - UART4 SCLK I/O */
	SWM_T0_MAT0_O       = 0xD1,	/*!< PINASSIGN13 - Timer0 Match0 Output */
	SWM_T0_MAT1_O       = 0xD2,	/*!< PINASSIGN13 - Timer0 Match1 Output */
	SWM_T0_MAT2_O       = 0xD3,	/*!< PINASSIGN13 - Timer0 Match2 Output */
	SWM_T0_MAT3_O       = 0xE0,	/*!< PINASSIGN14 - Timer0 Match3 Output */
	SWM_T0_CAP0_I       = 0xE1,	/*!< PINASSIGN14 - Timer0 Capture0 Input */
	SWM_T0_CAP1_I       = 0xE2,	/*!< PINASSIGN14 - Timer0 Capture1 Input */
	SWM_T0_CAP2_I       = 0xE3,	/*!< PINASSIGN14 - Timer0 Capture2 Input */	
} CHIP_SWM_PIN_MOVABLE_T;

/**
 * @brief LPC8XX Switch Matrix Fixed pins (works for LPC82x)
 */
typedef enum CHIP_SWM_PIN_FIXED    {
	SWM_FIXED_ACMP_I0 = 0,		/*!< ACMP I0 */
	SWM_FIXED_ACMP_I1,		/*!< ACMP I1 */
	SWM_FIXED_ACMP_I2,		/*!< ACMP I2 */
	SWM_FIXED_ACMP_I3,		/*!< ACMP I3 */
	SWM_FIXED_ACMP_I4,		/*!< ACMP I4 */		
	SWM_FIXED_SWCLK,		/*!< SWCLK */
	SWM_FIXED_SWDIO,		/*!< SWDIO */
	SWM_FIXED_XTALIN,		/*!< XTALIN */
	SWM_FIXED_XTALOUT,	/*!< XTALOUT */
	SWM_FIXED_RESETN,		/*!< Reset */
	SWM_FIXED_CLKIN,		/*!< Clock Input */
	SWM_FIXED_VDDCMP,		/*!< VDDCMP */
	SWM_FIXED_I2C0_SDA,	/*!< I2C0_SDA */
	SWM_FIXED_I2C0_SCL,	/*!< I2C0_SCL */
	SWM_FIXED_ADC_0,	/*!< ADC_0 */
	SWM_FIXED_ADC_1,	/*!< ADC_1 */
	SWM_FIXED_ADC_2,	/*!< ADC_2 */
	SWM_FIXED_ADC_3,	/*!< ADC_3 */
	SWM_FIXED_ADC_4,	/*!< ADC_4 */
	SWM_FIXED_ADC_5,	/*!< ADC_5 */
	SWM_FIXED_ADC_6,	/*!< ADC_6 */
	SWM_FIXED_ADC_7,	/*!< ADC_7 */
	SWM_FIXED_ADC_8,	/*!< ADC_8 */
	SWM_FIXED_ADC_9,	/*!< ADC_9 */
	SWM_FIXED_ADC_10,	/*!< ADC_10 */
	SWM_FIXED_ADC_11,	/*!< ADC_11 */
	SWM_FIXED_DAC_0,	/*!< DAC_0 */
	SWM_FIXED_DAC_1,	/*!< DAC_1 */	
	SWM_FIXED_CAPT_X0,	/*!< CAPT_X0 */
	SWM_FIXED_CAPT_X1,	/*!< CAPT_X1 */
	SWM_FIXED_CAPT_X2,	/*!< CAPT_X2 */
	SWM_FIXED_CAPT_X3,	/*!< CAPT_X3 */

	SWM_FIXED_CAPT_X4 = 32,	/*!< CAPT_X4 */
	SWM_FIXED_CAPT_X5,	/*!< CAPT_X5 */
	SWM_FIXED_CAPT_X6,	/*!< CAPT_X6 */
	SWM_FIXED_CAPT_X7,	/*!< CAPT_X7 */
	SWM_FIXED_CAPT_X8,	/*!< CAPT_X8 */	
	SWM_FIXED_CAPT_YL,	/*!< CAPT YL */
	SWM_FIXED_CAPT_YH	/*!< CAPT YH */
} CHIP_SWM_PIN_FIXED_T;

/**
 * @brief	Initialise the SWM module
 * @return	Nothing
 * @note	This function only enables the SWM clock.
 */
STATIC INLINE void Chip_SWM_Init(void)
{
	Chip_Clock_EnablePeriphClock(SYSCON_CLOCK_SWM);
}

/**
 * @brief	Deinitialise the SWM module
 * @return	Nothing
 * @note	This function only disables the SWM clock.
 */
STATIC INLINE void Chip_SWM_Deinit(void)
{
	Chip_Clock_DisablePeriphClock(SYSCON_CLOCK_SWM);
}

/**
 * @brief	Assign movable pin function to physical pin in Switch Matrix
 * @param	movable	: Movable pin function
 * @param	assign	: Physical pin to be assigned
 * @return	Nothing
 */
void Chip_SWM_MovablePinAssign(CHIP_SWM_PIN_MOVABLE_T movable, uint8_t papNum);

/**
 * @brief	Disable all fixed pin enables and switch matrix assignments for the pin
 * @param	pinNum	: Physical pin
 * @return	Nothing
 */
void Chip_SWM_PinDisable(uint32_t pinNum);

/**
 * @brief	Enables or Disable Fixed Function Pin in the Switch Matrix
 * @param	pin		: Pin to be enabled or disabled
 * @param	enable	: True to enable the pin, False to disable the pin
 * @return	Nothing
 */
void Chip_SWM_FixedPinEnable(CHIP_SWM_PIN_FIXED_T swmPin, bool enable);

/**
 * @brief	Tests whether a fixed pin is enabled or disabled in the Switch Matrix
 * @param	pin		: The pin to test whether it is enabled or disabled
 * @return	True if the pin is enabled, False if disabled
 */
//STATIC INLINE bool Chip_SWM_IsEnabled(CHIP_SWM_PIN_FIXED_T swmPin)
//{
//	return (bool) ((LPC_SWM->PINENABLE0 & (1 << (uint32_t) swmPin)) == 0);
//}

/**
 * @brief	Disable all fixed pin functions associated with pin
 * @param	pinNum
 * @return	Nothing
 */
void Chip_SWM_FixedPinFuncDisable(uint32_t papNum);


/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __SWM_84X_H_ */
