/*
 * @brief LPC8xx clock driver
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

#ifndef __CLOCK_84X_H_
#define __CLOCK_84X_H_

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup CLOCK_84X CHIP: LPC8xx Clock Driver
 * @ingroup CHIP_84X_Drivers
 * @{
 */

/* Internal FRO oscillator frequency */
#define SYSCON_EXT_OSC     (12000000)
	
#define SYSCON_FRO_18M     (18000000)
#define SYSCON_FRO_24M     (24000000)
#define SYSCON_FRO_30M     (30000000)

/**
 * Clock sources for system and USB PLLs
 */
typedef enum CHIP_SYSCON_PLLCLKSRC {
	SYSCON_PLLCLKSRC_FRO = 0,		/*!< Internal oscillator */
	SYSCON_PLLCLKSRC_EXT_CLK,		/*!< External clock input */
	SYSCON_PLLCLKSRC_WDTOSC,		/*!< WDT oscillator */
	SYSCON_PLLCLKSRC_FRO_DIV		/*!< FRO clock divider */
} CHIP_SYSCON_PLLCLKSRC_T;

/**
 * Watchdog oscillator analog output frequency selection
 * values enum (plus or minus 40%)
 */
typedef enum CHIP_WDTLFO_OSC {
	WDTLFO_OSC_ILLEGAL,
	WDTLFO_OSC_0_60,	/*!< 0.6 MHz watchdog/LFO rate */
	WDTLFO_OSC_1_05,	/*!< 1.05 MHz watchdog/LFO rate */
	WDTLFO_OSC_1_40,	/*!< 1.4 MHz watchdog/LFO rate */
	WDTLFO_OSC_1_75,	/*!< 1.75 MHz watchdog/LFO rate */
	WDTLFO_OSC_2_10,	/*!< 2.1 MHz watchdog/LFO rate */
	WDTLFO_OSC_2_40,	/*!< 2.4 MHz watchdog/LFO rate */
	WDTLFO_OSC_2_70,	/*!< 2.7 MHz watchdog/LFO rate */
	WDTLFO_OSC_3_00,	/*!< 3.0 MHz watchdog/LFO rate */
	WDTLFO_OSC_3_25,	/*!< 3.25 MHz watchdog/LFO rate */
	WDTLFO_OSC_3_50,	/*!< 3.5 MHz watchdog/LFO rate */
	WDTLFO_OSC_3_75,	/*!< 3.75 MHz watchdog/LFO rate */
	WDTLFO_OSC_4_00,	/*!< 4.0 MHz watchdog/LFO rate */
	WDTLFO_OSC_4_20,	/*!< 4.2 MHz watchdog/LFO rate */
	WDTLFO_OSC_4_40,	/*!< 4.4 MHz watchdog/LFO rate */
	WDTLFO_OSC_4_60		/*!< 4.6 MHz watchdog/LFO rate */
} CHIP_WDTLFO_OSC_T;

/**
 * Clock sources for main system clock
 */
typedef enum CHIP_SYSCON_MAINCLKSRC {
	SYSCON_MAINCLKSRC_FRO = 0,		/*!< FRO Internal oscillator */
	SYSCON_MAINCLKSRC_EXT_CLK,		/*!< External clock */
	SYSCON_MAINCLKSRC_WDTOSC,			/*!< Watchdog oscillator rate */
	SYSCON_MAINCLKSRC_FRO_DIV			/*!< FRO divider output */
} CHIP_SYSCON_MAINCLKSRC_T;

/**
 * Clock sources for main system clock
 */
typedef enum CHIP_SYSCON_MAINCLKPLLSRC {
	SYSCON_MAINCLKPLLSRC_MAINCLKOUT = 0,		/*!< Main clock select output */
	SYSCON_MAINCLKPLLSRC_PLLOUT,						/*!< System PLL output */
} CHIP_SYSCON_MAINCLKPLLSRC_T;

/**
 * System and peripheral clocks enum
 */
typedef enum CHIP_SYSCON_CLOCK {
	SYSCON_CLOCK_SYS = 0,		/*!< System clock */
	SYSCON_CLOCK_ROM,				/*!< ROM clock */
	SYSCON_CLOCK_RAM,				/*!< RAM clock */
	SYSCON_CLOCK_FLASHREG,	/*!< FLASH register interface clock */
	SYSCON_CLOCK_FLASH,			/*!< FLASH array access clock */
	SYSCON_CLOCK_I2C0,			/*!< I2C0 clock */
	SYSCON_CLOCK_GPIO0,			/*!< GPIO0 clock */
	SYSCON_CLOCK_SWM,				/*!< Switch matrix clock */
	SYSCON_CLOCK_SCT,				/*!< State configurable timer clock */
	SYSCON_CLOCK_WKT,				/*!< Self wake-up timer clock */
	SYSCON_CLOCK_MRT,				/*!< Multi-rate timer clock */
	SYSCON_CLOCK_SPI0,			/*!< SPI0 clock */
	SYSCON_CLOCK_SPI1,			/*!< SPI01 clock */
	SYSCON_CLOCK_CRC,				/*!< CRC clock */
	SYSCON_CLOCK_UART0,			/*!< UART0 clock */
	SYSCON_CLOCK_UART1,			/*!< UART1 clock */
	SYSCON_CLOCK_UART2,			/*!< UART2 clock */
	SYSCON_CLOCK_WWDT,			/*!< Watchdog clock */
	SYSCON_CLOCK_IOCON,			/*!< IOCON clock */
	SYSCON_CLOCK_ACOMP,			/*!< Analog comparator clock */
	SYSCON_CLOCK_GPIO1,			/*!< GPIO1 clock */
	SYSCON_CLOCK_I2C1,			/*!< I2C1 clock */
	SYSCON_CLOCK_I2C2,			/*!< I2C2 clock */
	SYSCON_CLOCK_I2C3,			/*!< I2C3 clock */
	SYSCON_CLOCK_ADC,				/*!< ADC clock */
	SYSCON_CLOCK_CTIMER0,		/*!< CTIMER0 clock */
	SYSCON_CLOCK_MTB,				/*!< MTB clock */
	SYSCON_CLOCK_DAC0,			/*!< DAC0 clock */
	SYSCON_CLOCK_GPIOINT,		/*!< GPIO INT clock */
	SYSCON_CLOCK_DMA,				/*!< DMA clock */
	SYSCON_CLOCK_UART3,			/*!< UART3 clock */
	SYSCON_CLOCK_UART4,			/*!< UART4 clock */
	
	SYSCON_CLOCK_CAPT = 32,	/*!< CAPT clock */
	SYSCON_CLOCK_DAC1,			/*!< DAC1 clock */
	SYSCON_CLOCK_FAIM				/*!< FAIM clock */		
} CHIP_SYSCON_CLOCK_T;

/**
 * Clock sources for CLKOUT
 */
typedef enum CHIP_SYSCON_CLKOUTSRC {
	SYSCON_CLKOUTSRC_FRO = 0,		/*!< FRO Internal oscillator for CLKOUT */
	SYSCON_CLKOUTSRC_MAINCLK,		/*!< Main system clock for CLKOUT */
	SYSCON_CLKOUTSRC_SYSPLL,		/*!< System PLL for CLKOUT */
	SYSCON_CLKOUTSRC_EXT_CLK,		/*!< External clock for CLKOUT */
	SYSCON_CLKOUTSRC_WDTOSC			/*!< Watchdog oscillator for CLKOUT */
} CHIP_SYSCON_CLKOUTSRC_T;

/**
 * @brief	Set System PLL divider values
 * @param	msel    : PLL feedback divider value
 * @param	psel    : PLL post divider value (This doesn't divide the output of the PLL, part of feedback)
 * @return	Nothing
 * @note	See the user manual for how to setup the PLL
 */
STATIC INLINE void Chip_Clock_SetupSystemPLL(uint8_t msel, uint8_t psel)
{
	LPC_SYSCON->SYSPLLCTRL = (msel & 0x1F) | ((psel & 0x3) << 5);
}

/**
 * @brief	Setup Watchdog oscillator rate and divider
 * @param	wdtclk	: Selected watchdog clock rate
 * @param	div		: Watchdog divider value, even value between 2 and 64
 * @return	Nothing
 * @note	Watchdog rate = selected rate divided by divider rate
 */
STATIC INLINE void Chip_Clock_SetWDTOSC(CHIP_WDTLFO_OSC_T wdtclk, uint8_t div)
{
	LPC_SYSCON->WDTOSCCTRL  = (((uint32_t) wdtclk) << 5) | ((div >> 1) - 1);
}

/**
 * @brief   Returns the main clock source
 * @return	Main clock source
 */
STATIC INLINE CHIP_SYSCON_MAINCLKSRC_T Chip_Clock_GetMainClockSource(void)
{
	return (CHIP_SYSCON_MAINCLKSRC_T) (LPC_SYSCON->MAINCLKSEL);
}

/**
 * @brief	Set system clock divider
 * @param	div	: divider for system clock
 * @return	Nothing
 * @note	Use 0 to disable, or a divider value of 1 to 255. The system clock
 * rate is the main system clock divided by this value.
 */
STATIC INLINE void Chip_Clock_SetSysClockDiv(uint32_t div)
{
	LPC_SYSCON->SYSAHBCLKDIV  = div;
}

/**
 * @brief	Enable system or peripheral clock
 * @param	clk	: Clock to enable
 * @return	Nothing
 */
STATIC INLINE void Chip_Clock_EnablePeriphClock(CHIP_SYSCON_CLOCK_T clk)
{
	if ( clk < 32 ) {
		LPC_SYSCON->SYSAHBCLK0CTRL |= (1 << clk);
	}
	else {
		LPC_SYSCON->SYSAHBCLK1CTRL |= (1 << (clk-32));
	}
}

/**
 * @brief	Disable system or peripheral clock
 * @param	clk	: Clock to disable
 * @return	Nothing
 */
STATIC INLINE void Chip_Clock_DisablePeriphClock(CHIP_SYSCON_CLOCK_T clk)
{
	if ( clk < 32 ) {
		LPC_SYSCON->SYSAHBCLK0CTRL &= ~(1 << clk);
	}
	else {
		LPC_SYSCON->SYSAHBCLK1CTRL &= ~(1 << (clk-32));
	}
}

/**
 * @brief	Set UART divider clock
 * @param	div	: divider for UART clock
 * @return	Nothing
 * @note	Use 0 to disable, or a divider value of 1 to 255. The UART clock
 * rate is the main system clock divided by this value.
 */
STATIC INLINE void Chip_Clock_SetUARTClockDiv(uint8_t frg_num, uint32_t div)
{
	if ( frg_num == 0 ) {
		LPC_SYSCON->FRG0DIV = div;
	}
	else {
		LPC_SYSCON->FRG1DIV = div;
	}
}

/**
 * @brief	Return UART divider
 * @return	divider for UART clock
 * @note	A value of 0 means the clock is disabled.
 */
STATIC INLINE uint32_t Chip_Clock_GetUARTClockDiv(uint8_t frg_num)
{
	if ( frg_num == 0 ) {
		return (LPC_SYSCON->FRG0DIV & SYSCON_UARTCLKDIV_DIV_M) >> SYSCON_UARTCLKDIV_DIV_P;
	}
	else {
		return (LPC_SYSCON->FRG1DIV & SYSCON_UARTCLKDIV_DIV_M) >> SYSCON_UARTCLKDIV_DIV_P;
	}
}

/**
 * @brief	Set The USART Fractional Generator Divider
 * @param   div  :  Fractional Generator Divider value, should be 0xFF
 * @return	Nothing
 */
STATIC INLINE void Chip_SYSCON_SetUSARTFRGDivider(uint8_t frg_num, uint8_t div)
{
	if ( frg_num == 0 ) {
		LPC_SYSCON->FRG0DIV = (uint32_t) div;
	}
	else {
		LPC_SYSCON->FRG1DIV = (uint32_t) div;
	}
}

/**
 * @brief	Set The USART Fractional Generator Divider
 * @return	Value of USART Fractional Generator Divider
 */
STATIC INLINE uint32_t Chip_SYSCON_GetUSARTFRGDivider(uint8_t frg_num)
{
	if ( frg_num == 0 ) {
		return LPC_SYSCON->FRG0DIV;
	}
	else {
		return LPC_SYSCON->FRG1DIV;
	}
}

/**
 * @brief	Set The USART Fractional Generator Multiplier
 * @param   mult  :  An 8-bit value (0-255) U_PCLK = UARTCLKDIV/(1 + MULT/256)
 * @return	Nothing
 */
STATIC INLINE void Chip_SYSCON_SetUSARTFRGMultiplier(uint8_t frg_num, uint8_t mult)
{
	if ( frg_num == 0 ) {
		LPC_SYSCON->FRG0MULT = (uint32_t) mult;
	}
	else {
		LPC_SYSCON->FRG1MULT = (uint32_t) mult;
	}
}

/**
 * @brief	Get The USART Fractional Generator Multiplier
 * @return	Value of USART Fractional Generator Multiplier
 */
STATIC INLINE uint32_t Chip_SYSCON_GetUSARTFRGMultiplier(uint8_t frg_num)
{
	if ( frg_num == 0 ) {	
		return LPC_SYSCON->FRG0MULT;
	}
	else {
		return LPC_SYSCON->FRG1MULT;
	}
}

/**
 * @brief	Returns the main oscillator clock rate
 * @return	main oscillator clock rate
 */
STATIC INLINE uint32_t Chip_Clock_GetExtOscRate(void)
{
	return OscRateIn;
}

/**
 * @brief	Set the desired FRO clock in FROOSCCTRL register.
					if direct mode is set, FRO will be either 16, or 24, or 30
					before FAIM setting. Other wise, it's 8,12,or 15.
					Note: the actual frequency may not be the same as the
					input setting because it also depends on the FAIM setting.
 * @return	Actual FRO clock frequency
 */
STATIC INLINE uint32_t Chip_Clock_SetFRO(uint32_t fro_freq, uint32_t direct_mode )
{
	uint32_t clk, regVal;
	volatile uint32_t *faim_addr = (volatile uint32_t *)LPC_FAIM_BASE;
	
	clk = fro_freq;
	regVal = LPC_SYSCON->FROOSCCTRL;	
	regVal &= 0x03;
	switch ( fro_freq ) {
		case SYSCON_FRO_18M: 
			LPC_SYSCON->FROOSCCTRL = regVal;
			break;
		case SYSCON_FRO_24M: 
			LPC_SYSCON->FROOSCCTRL = regVal | 0x1;
			break;
		case SYSCON_FRO_30M: 
			LPC_SYSCON->FROOSCCTRL = regVal | 0x2;
			break;
		default:
			LPC_SYSCON->FROOSCCTRL = regVal | 0x1;
			break;
	}
	if ( direct_mode ) {
		LPC_SYSCON->FROOSCCTRL |= (0x1<<17);
	}
	else {
		LPC_SYSCON->FROOSCCTRL &= ~(0x1<<17);
		clk /= 2;
		/* Check FAIM configuration checking, either divided by 2 or 16. */
		if ( *faim_addr & (0x1<<1) ) {
			clk /= 8;
		}
	}	
	LPC_SYSCON->FRODIRECTCLKUEN  = 0;
	LPC_SYSCON->FRODIRECTCLKUEN  = 1;
	return ( clk );
}

/**
 * @brief	Returns the internal oscillator (FRO) clock rate
 * @return	internal oscillator (IRC) clock rate
 */
STATIC INLINE uint32_t Chip_Clock_GetIntOscRate(void)
{
	uint32_t clk_sel, clk;
	volatile uint32_t *faim_addr = (volatile uint32_t *)LPC_FAIM_BASE;
	
	/* to-do list: if FAIM is configured and FRO divider is on. */
	LPC_SYSCON->FRODIRECTCLKUEN  = 0;
	LPC_SYSCON->FRODIRECTCLKUEN  = 1;
	clk_sel = LPC_SYSCON->FROOSCCTRL & 0x03;
	switch ( clk_sel ) {
		case 0:
			clk = SYSCON_FRO_18M;
			break;
		case 1:
			clk = SYSCON_FRO_24M;
			break;
		case 2:
		case 3:
		default:
			clk = SYSCON_FRO_30M;
			break;
	}

	if ( LPC_SYSCON->FROOSCCTRL & 0x1<<17 ) {
		return clk;
	}
	else {
		/* Check FAIM configuration checking, either divided by 2 or 16. */
		clk /= 2;
		if ( *faim_addr & (0x1<<1) ) {
			clk /= 8;
		}
		return clk;
	}
}

/**
 * @brief	Returns the external clock input rate
 * @return	External clock input rate
 */
STATIC INLINE uint32_t Chip_Clock_GetExtClockInRate(void)
{
	return ExtRateIn;
}

/**
 * @brief	Set System PLL clock source
 * @param	src	: Clock source for system PLL
 * @return	Nothing
 * @note	This function will also toggle the clock source update register
 * to update the clock source
 */
void Chip_Clock_SetSystemPLLSource(CHIP_SYSCON_PLLCLKSRC_T src);

/**
 * @brief	Bypass System Oscillator and set oscillator frequency range
 * @param	bypass	: Flag to bypass oscillator
 * @param	highfr	: Flag to set oscillator range from 15-25 MHz
 * @return	Nothing
 * @note	Sets the PLL input to bypass the oscillator. This would be
 * used if an external clock that is not an oscillator is attached
 * to the XTALIN pin.
 */
void Chip_Clock_SetPLLBypass(bool bypass, bool highfr);

/**
 * @brief	Set main system clock source
 * @param	src	: Clock source for main system
 * @return	Nothing
 * @note	This function will also toggle the clock source update register
 * to update the clock source
 */
void Chip_Clock_SetMainClockSource(CHIP_SYSCON_MAINCLKSRC_T src);

/**
 * @brief	Set main system clock PLL source
 * @param	src	: Clock source for main system PLL
 * @return	Nothing
 * @note	This function will also toggle the clock source update register
 * to update the clock source
 */
void Chip_Clock_SetMainClockPLLSource(CHIP_SYSCON_MAINCLKPLLSRC_T src);

/**
 * @brief	Set CLKOUT clock source and divider
 * @param	src	: Clock source for CLKOUT
 * @param	div	: divider for CLKOUT clock
 * @return	Nothing
 * @note	Use 0 to disable, or a divider value of 1 to 255. The CLKOUT clock
 * rate is the clock source divided by the divider. This function will
 * also toggle the clock source update register to update the clock
 * source.
 */
void Chip_Clock_SetCLKOUTSource(CHIP_SYSCON_CLKOUTSRC_T src, uint32_t div);

/**
 * @brief	Return estimated watchdog oscillator rate
 * @return	Estimated watchdog oscillator rate
 * @note	This rate is accurate to plus or minus 40%.
 */
uint32_t Chip_Clock_GetWDTOSCRate(void);

/**
 * @brief	Return System PLL input clock rate
 * @return	System PLL input clock rate
 */
uint32_t Chip_Clock_GetSystemPLLInClockRate(void);

/**
 * @brief	Return System PLL output clock rate
 * @return	System PLL output clock rate
 */
uint32_t Chip_Clock_GetSystemPLLOutClockRate(void);

/**
 * @brief	Return main clock rate
 * @return	main clock rate
 */
uint32_t Chip_Clock_GetMainClockRate(void);

/**
 * @brief	Return main clock PLL rate
 * @return	main clock PLL rate
 */
uint32_t Chip_Clock_GetMainClockPLLRate(void);

/**
 * @brief	Return system clock rate
 * @return	system clock rate
 */
uint32_t Chip_Clock_GetSystemClockRate(void);

/**
 * @brief	Fractional Divider clock sources
 */
typedef enum {
	SYSCON_FRGCLKSRC_FRO,    			/*!< FRO */
	SYSCON_FRGCLKSRC_MAINCLK,     /*!< Main Clock */
	SYSCON_FRGCLKSRC_PLL,         /*!< Output clock from PLL */
	SYSCON_FRGCLKSRC_NONE = 3     /*!< No clock input */
}CHIP_SYSCON_FRGCLKSRC_T;

/**
 * @brief	Get the input clock frequency of FRG
 * @return	Frequency in Hz on success (0 on failure)
 */
uint32_t Chip_Clock_GetFRGInClockRate(uint8_t frg_num);

/**
 * @brief	Set clock source used by FRG
 * @return	Clock source used by FRG
 * @note
 */
STATIC INLINE void Chip_Clock_SetFRGClockSource(uint8_t frg_num, CHIP_SYSCON_FRGCLKSRC_T src)
{
	if ( frg_num == 0 ) {
		LPC_SYSCON->FRG0CLKSEL = (uint32_t) src;
	}
	else {
		LPC_SYSCON->FRG1CLKSEL = (uint32_t) src;
	}
}

/**
 * @brief	Get clock source used by FRG
 * @return	Clock source used by FRG
 * @note
 */
STATIC INLINE CHIP_SYSCON_FRGCLKSRC_T Chip_Clock_GetFRGClockSource(uint8_t frg_num)
{
	if ( frg_num == 0 ) {
		return (CHIP_SYSCON_FRGCLKSRC_T)(LPC_SYSCON->FRG0CLKSEL & 0x03);
	}
	else {
		return (CHIP_SYSCON_FRGCLKSRC_T)(LPC_SYSCON->FRG1CLKSEL & 0x03);
	}
}

/**
 * @brief	Get Fraction Rate Generator (FRG) clock rate
 * @return	UART base clock rate
 */
uint32_t Chip_Clock_GetFRGClockRate(uint8_t frg_num);

/**
 * @brief	Set FRG rate to given rate
 * @return	Actual FRG clock rate
 * @note	If FRG is used for UART base clock, @a rate
 * is recommended to be 16 times desired baud rate;
 * <b>This API must only be called after setting the source using
 * Chip_Clock_SetFRGClockSource()</b>
 */
uint32_t Chip_Clock_SetFRGClockRate(uint8_t frg_num, uint32_t rate);

/**
 * Clock sources for FLEXCOMM clock source select
 */
typedef enum {
	SYSCON_FLEXCOMMCLKSELSRC_FRO = 0,						/*!< FRO 12-MHz */
	SYSCON_FLEXCOMMCLKSELSRC_MAINCLK,           /*!< Main clock */
	SYSCON_FLEXCOMMCLKSELSRC_FRG0,              /*!< FRG0 output */
	SYSCON_FLEXCOMMCLKSELSRC_FRG1,              /*!< FRG1 output */
	SYSCON_FLEXCOMMCLKSELSRC_FRO_DIV,           /*!< FRO DIV output */
	SYSCON_FLEXCOMMCLKSELSRC_NONE = 7           /*!< NONE output */
} CHIP_SYSCON_FCLKSELSRC_T;

/**
 * @brief	Set the FLEXCOMM clock source
 * @param	idx	: Index of the flexcomm (0 to 7)
 * @param	src	: FLEXCOMM clock source (See #CHIP_SYSCON_FLEXCOMMCLKSELSRC_T)
 * @return	Nothing
 */
STATIC INLINE void Chip_Clock_SetFClockSource(uint32_t idx, CHIP_SYSCON_FCLKSELSRC_T src)
{
	LPC_SYSCON->FXCOMCLKSEL[idx] = (uint32_t) src;
}

/**
 * @brief   Returns the FLEXCOMM clock source
 * @param	idx	: Index of the flexcomm (0 to 7)
 * @return	Returns which clock is used for the FLEXCOMM clock source
 */
STATIC INLINE CHIP_SYSCON_FCLKSELSRC_T Chip_Clock_GetFClockSource(uint32_t idx)
{
	return (CHIP_SYSCON_FCLKSELSRC_T) (LPC_SYSCON->FXCOMCLKSEL[idx] & 0x07);
}

/**
 * @brief	Return FlexCOMM clock rate
 * @param	id	: FlexCOMM ID (Valid range: 0 to 7)
 * @return	FlexCOMM clock rate
 */
uint32_t Chip_Clock_GetFClockRate(uint32_t id);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __CLOCK_84X_H_ */
