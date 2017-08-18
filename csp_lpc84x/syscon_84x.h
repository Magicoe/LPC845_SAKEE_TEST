/*
 * @brief LPC8xx System & Control driver inclusion file
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

#ifndef __SYSCON_84X_H_
#define __SYSCON_84X_H_

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup SYSCON_84X CHIP: LPC8xx System and Control Driver
 * @ingroup CHIP_84X_Drivers
 * @{
 */

#define SYSCON_WDTOSCCTRL_DIVSEL_P		0
#define SYSCON_WDTOSCCTRL_DIVSEL_M		(0x1FUL << SYSCON_WDTOSCCTRL_DIVSEL_P)
#define SYSCON_WDTOSCCTRL_DIVSEL_SET(value)		(((uint32_t)(value) << SYSCON_WDTOSCCTRL_DIVSEL_P) & SYSCON_WDTOSCCTRL_DIVSEL_M)
#define SYSCON_WDTOSCCTRL_FREQSEL_P		5
#define SYSCON_WDTOSCCTRL_FREQSEL_M		(0xFUL << SYSCON_WDTOSCCTRL_FREQSEL_P)
#define SYSCON_WDTOSCCTRL_FREQSEL_SET(value)		(((uint32_t)(value) << SYSCON_WDTOSCCTRL_FREQSEL_P) & SYSCON_WDTOSCCTRL_FREQSEL_M)

#define SYSCON_UARTCLKDIV_DIV_P			0			/*!< Uart clock divider (of the main clock) */
#define SYSCON_UARTCLKDIV_DIV_M			(0xFFUL << SYSCON_UARTCLKDIV_DIV_P)
#define SYSCON_UARTCLKDIV_DIV_MAX		(SYSCON_UARTCLKDIV_DIV_M >> SYSCON_UARTCLKDIV_DIV_P)

	
/**
 * System reset status values
 */
#define SYSCON_RST_POR    (1 << 0)	/*!< POR reset status */
#define SYSCON_RST_EXTRST (1 << 1)	/*!< External reset status */
#define SYSCON_RST_WDT    (1 << 2)	/*!< Watchdog reset status */
#define SYSCON_RST_BOD    (1 << 3)	/*!< Brown-out detect reset status */
#define SYSCON_RST_SYSRST (1 << 4)	/*!< software system reset status */

/**
 * Non-Maskable Interrupt Enable/Disable value
 */
#define SYSCON_NMISRC_ENABLE   ((uint32_t) 1 << 31)	/*!< Enable the Non-Maskable Interrupt (NMI) source */

/**
 * @brief LPC8XX System Control and Clock register block structure
 */
typedef struct {
	__IO uint32_t SYSMEMREMAP;			/*!< Offset: 0x000 System memory remap (R/W) */
	     uint32_t RESERVED0;
	__IO uint32_t SYSPLLCTRL;				/*!< Offset: 0x008 System PLL control (R/W) */
	__IO uint32_t SYSPLLSTAT;				/*!< Offset: 0x00C System PLL status (R/W ) */
	     uint32_t RESERVED1[4];
	__IO uint32_t SYSOSCCTRL;				/*!< Offset: 0x020 System oscillator control (R/W) */
	__IO uint32_t WDTOSCCTRL;				/*!< Offset: 0x024 Watchdog oscillator control (R/W) */
	__IO uint32_t FROOSCCTRL;				/*!< Offset: 0x028 FRO oscillator control (R/W) */
	     uint32_t RESERVED2;
	__IO uint32_t FRODIRECTCLKUEN;	/*!< Offset: 0x030 FRO direct clock update (R/W ) */
	     uint32_t RESERVED3;
	__IO uint32_t SYSRSTSTAT;				/*!< Offset: 0x038 System reset status Register (R/W ) */
	__IO uint32_t FAIMROWPROTECTCTRL;	/*!< Offset: 0x03C FAIM row protect control (R/W ) */
	__IO uint32_t SYSPLLCLKSEL;			/*!< Offset: 0x040 System PLL clock source select (R/W) */
	__IO uint32_t SYSPLLCLKUEN;			/*!< Offset: 0x044 System PLL clock source update enable (R/W) */
	__IO uint32_t MAINCLKPLLSEL;		/*!< Offset: 0x048 Main clock PLL source select (R/W) */
	__IO uint32_t MAINCLKPLLUEN;		/*!< Offset: 0x04C Main clock PLL source update enable (R/W) */
	__IO uint32_t MAINCLKSEL;				/*!< Offset: 0x050 Main clock source select (R/W) */
	__IO uint32_t MAINCLKUEN;				/*!< Offset: 0x054 Main clock source update enable (R/W) */
	__IO uint32_t SYSAHBCLKDIV;			/*!< Offset: 0x058 System AHB clock divider (R/W) */
	     uint32_t RESERVED4;	
	__IO uint32_t CAPTCLKSEL;				/*!< Offset: 0x060 Capture clock source select (R/W) */
	__IO uint32_t ADCCLKSEL;				/*!< Offset: 0x064 ADC clock source select (R/W) */
	__IO uint32_t ADCCLKDIV;				/*!< Offset: 0x068 ADC clock divider (R/W) */
	__IO uint32_t SCTCLKSEL;				/*!< Offset: 0x06C SCT clock source select (R/W) */
	__IO uint32_t SCTCLKDIV;				/*!< Offset: 0x070 SCT clock divider (R/W) */
	__IO uint32_t EXTCLKSEL;				/*!< Offset: 0x074 External clock source select (R/W) */
	     uint32_t RESERVED5[2];
  union{
  __IO uint32_t SYSAHBCLKCTRL[2];
		struct{
		__IO uint32_t SYSAHBCLK0CTRL;		/*!< Offset: 0x080 System AHB clock0 control (R/W) */
		__IO uint32_t SYSAHBCLK1CTRL;		/*!< Offset: 0x084 System AHB clock1 control (R/W) */                 	
		};
  };
  union{
  __IO uint32_t PRESETCTRL[2];
		struct{
		__IO uint32_t PRESET0CTRL;			/*!< Offset: 0x088 Peripheral Reset0 control (R/W) */
		__IO uint32_t PRESET1CTRL;			/*!< Offset: 0x08C Peripheral Reset1 control (R/W) */                	
		};
  };
  union{
	__IO uint32_t FXCOMCLKSEL[11];	/*!< Offset: 0x090~0xB8 Flexcomm 0~10 clock source select (R/W) */
		struct{
		__IO uint32_t FXCOMCLK0SEL;			/*!< Offset: 0x090~0xB8 Flexcomm 0~10 clock source select (R/W) */
		__IO uint32_t FXCOMCLK1SEL; 
		__IO uint32_t FXCOMCLK2SEL;
		__IO uint32_t FXCOMCLK3SEL; 
		__IO uint32_t FXCOMCLK4SEL;
		__IO uint32_t FXCOMCLK5SEL; 
		__IO uint32_t FXCOMCLK6SEL;
		__IO uint32_t FXCOMCLK7SEL; 
		__IO uint32_t FXCOMCLK8SEL;
		__IO uint32_t FXCOMCLK9SEL;
		__IO uint32_t FXCOMCLK10SEL;  			
		};
  };
	     uint32_t RESERVED6;	
	__IO uint32_t EFLASHREFCLKDIV;	/*!< Offset: 0x0C0 EFLASH REF clock divider (R/W) */
	__IO uint32_t FAIMREFCLKDIV;		/*!< Offset: 0x0C4 FAIM REF clock divider (R/W) */
	     uint32_t RESERVED7[2];
	__IO uint32_t FRG0DIV;					/*!< Offset: 0x0D0 FRG0 clock divider (R/W) */
	__IO uint32_t FRG0MULT;					/*!< Offset: 0x0D4 FRG0 clock multiplier (R/W) */
	__IO uint32_t FRG0CLKSEL;				/*!< Offset: 0x0D8 FRG0 clock source select (R/W) */
	     uint32_t RESERVED8;	
	__IO uint32_t FRG1DIV;					/*!< Offset: 0x0E0 FRG1 clock divider (R/W) */
	__IO uint32_t FRG1MULT;					/*!< Offset: 0x0E4 FRG1 clock multiplier (R/W) */
	__IO uint32_t FRG1CLKSEL;				/*!< Offset: 0x0E8 FRG1 clock source select (R/W) */	
	     uint32_t RESERVED9;	
	__IO uint32_t CLKOUTSEL;				/*!< Offset: 0x0F0 CLKOUT clock source select (R/W) */
	__IO uint32_t CLKOUTDIV;				/*!< Offset: 0x0F4 CLKOUT clock divider (R/W) */
	     uint32_t RESERVED10;
	__IO uint32_t EXTTRACECMD;			/*!< Offset: 0x0FC External trace buffer command register  */
	__IO uint32_t PIOPORCAP0;				/*!< Offset: 0x100 POR captured PIO status 0 (R/ ) */
	__IO uint32_t PIOPORCAP1;				/*!< Offset: 0x104 POR captured PIO status 1 (R/ ) */
	     uint32_t RESERVED11[11];
	__IO uint32_t IOCONCLKDIV[7];		/*!< Offset: 0x134 Peripheral clock x to the IOCON block for programmable glitch filter */
	__IO uint32_t BODCTRL;					/*!< Offset: 0x150 BOD control (R/W) */
	__IO uint32_t SYSTCKCAL;				/*!< Offset: 0x154 System tick counter calibration (R/W) */
	     uint32_t RESERVED12[6];
	__IO uint32_t IRQLATENCY;				/*!< Offset: 0x170 IRQ delay */
	__IO uint32_t NMISRC;						/*!< Offset: 0x174 NMI Source Control     */
	__IO uint32_t PINTSEL[8];				/*!< Offset: 0x178 GPIO Pin Interrupt Select register 0 */
	     uint32_t RESERVED13[27];
	__IO uint32_t STARTERP0;				/*!< Offset: 0x204 Start logic signal enable Register 0 (R/W) */
	     uint32_t RESERVED14[3];
	__IO uint32_t STARTERP1;				/*!< Offset: 0x214 Start logic signal enable Register 0 (R/W) */
	     uint32_t RESERVED15[6];
	__IO uint32_t PDSLEEPCFG;				/*!< Offset: 0x230 Power-down states in Deep-sleep mode (R/W) */
	__IO uint32_t PDAWAKECFG;				/*!< Offset: 0x234 Power-down states after wake-up (R/W) */
	__IO uint32_t PDRUNCFG;					/*!< Offset: 0x238 Power-down configuration Register (R/W) */
	     uint32_t RESERVED16[111];
	__I  uint32_t DEVICEID;					/*!< Offset: 0x3F8 Device ID (R/ ) */
} LPC_SYSCON_T;

/**
 * System memory remap modes used to remap interrupt vectors
 */
typedef enum CHIP_SYSCON_BOOT_MODE_REMAP {
	REMAP_BOOT_LOADER_MODE,	/*!< Interrupt vectors are re-mapped to Boot ROM */
	REMAP_USER_RAM_MODE,	/*!< Interrupt vectors are re-mapped to user Static RAM */
	REMAP_USER_FLASH_MODE	/*!< Interrupt vectors are not re-mapped and reside in Flash */
} CHIP_SYSCON_BOOT_MODE_REMAP_T;

/**
 * Peripheral reset identifiers
 */
typedef enum {
	RESET_FLASH=4,	/*!< FLASH reset control */
	RESET_I2C0,			/*!< I2C0 reset control */
	RESET_GPIO0,		/*!< GPIO0 reset control */
	RESET_SWM,			/*!< SWM reset control */
	RESET_SCT,			/*!< SCT reset control */
	RESET_WKT,			/*!< Self wake-up timer (WKT) control */
	RESET_MRT,			/*!< MRT reset control */
	RESET_SPI0,			/*!< SPI0 reset control */
	RESET_SPI1,			/*!< SPI1 reset control */
	RESET_CRC,			/*!< CRC reset control */
	RESET_UART0,		/*!< UART0 reset control */
	RESET_UART1,		/*!< UART1 reset control */
	RESET_UART2,		/*!< UART2 reset control */
	RESET_NULL0,		/*!< WWDT reset control is not routed out. TT */
	RESET_IOCON,		/*!< IOCON reset control */
	RESET_ACOMP,		/*!< ACOMP reset control */
	RESET_GPIO1,		/*!< GPIO1 reset control */
	RESET_I2C1,			/*!< I2C1 reset control */
	RESET_I2C2,			/*!< I2C2 reset control */
	RESET_I2C3,			/*!< I2C3 reset control */	
	RESET_ADC,			/*!< ADC reset control */
	RESET_CTIMER0,	/*!< CTIMER0 reset control */
	RESET_NULL1,		/*!< MTB reset control is not routed out. TT */
	RESET_DAC0,			/*!< DAC0 reset control */
	RESET_GPIOINT,	/*!< GPIOINT reset control */
	RESET_DMA,			/*!< DMA reset control */
	RESET_UART3,		/*!< UART3 reset control */
	RESET_UART4,		/*!< UART4 reset control */	
	
	RESET_CAPT=32,	/*!< CAPT reset control */
	RESET_DAC1			/*!< DAC1 reset control */
} CHIP_SYSCON_PERIPH_RESET_T;

/**
 * Brown-out detector reset level
 */
typedef enum CHIP_SYSCON_BODRSTLVL {
	SYSCON_BODRSTLVL_0,	/*!< Brown-out reset at 1.46 ~ 1.63v */
	SYSCON_BODRSTLVL_1,	/*!< Brown-out reset at 2.06v ~ 2.15v */
	SYSCON_BODRSTLVL_2,	/*!< Brown-out reset at 2.35v ~ 2.43v */
	SYSCON_BODRSTLVL_3,	/*!< Brown-out reset at 2.63v ~ 2.71v */
} CHIP_SYSCON_BODRSTLVL_T;

/**
 * Brown-out detector interrupt level
 */
typedef enum CHIP_SYSCON_BODRINTVAL {
	SYSCON_BODINTVAL_LVL0,	/* Brown-out interrupt at 1.65 ~ 1.80v */
	SYSCON_BODINTVAL_LVL1,	/* Brown-out interrupt at 2.22v ~ 2.35v*/
	SYSCON_BODINTVAL_LVL2,	/* Brown-out interrupt at 2.52v ~ 2.66v */
	SYSCON_BODINTVAL_LVL3,	/* Brown-out interrupt at 2.80v ~ 2.90v */
} CHIP_SYSCON_BODRINTVAL_T;


// 
#define SYSCON_SYSPLLCTRL_MSEL_P					0
#define SYSCON_SYSPLLCTRL_MSEL_M					(0x1FUL << SYSCON_SYSPLLCTRL_MSEL_P)
#define SYSCON_SYSPLLCTRL_MSEL_SET(value) 	(((uint32_t)(value) << SYSCON_SYSPLLCTRL_MSEL_P) & SYSCON_SYSPLLCTRL_MSEL_M)
#define SYSCON_SYSPLLCTRL_MSEL_GET(regValue)	((regValue & SYSCON_SYSPLLCTRL_MSEL_M) >> SYSCON_SYSPLLCTRL_MSEL_P)
#define SYSCON_SYSPLLCTRL_PSEL_P					5
#define SYSCON_SYSPLLCTRL_PSEL_M					(0x3UL << SYSCON_SYSPLLCTRL_PSEL_P)
#define SYSCON_SYSPLLCTRL_PSEL_SET(value) 	(((uint32_t)(value) << SYSCON_SYSPLLCTRL_PSEL_P) & SYSCON_SYSPLLCTRL_PSEL_M)
#define SYSCON_SYSPLLCTRL_PSEL_GET(regValue)	((regValue & SYSCON_SYSPLLCTRL_PSEL_M) >> SYSCON_SYSPLLCTRL_PSEL_P)

/**
 * PDRUNCFG bit definitions
 */
typedef enum {
	SYSCON_PDRUNCFG_FROOUT=0,
	SYSCON_PDRUNCFG_FRO,
	SYSCON_PDRUNCFG_FLASH,
	SYSCON_PDRUNCFG_BOD,
	SYSCON_PDRUNCFG_ADC,
	SYSCON_PDRUNCFG_SYSOSC,
	SYSCON_PDRUNCFG_WDTOSC,
	SYSCON_PDRUNCFG_SYSPLL,
	SYSCON_PDRUNCFG_VREF2=10,
	SYSCON_PDRUNCFG_DAC0=13,
	SYSCON_PDRUNCFG_DAC1,
	SYSCON_PDRUNCFG_ACMP
} SYSCON_PDRUNCFG_T;

/**
 * @brief	Re-map interrupt vectors
 * @param	remap	: system memory map value
 * @return	Nothing
 */
STATIC INLINE void Chip_SYSCON_Map(CHIP_SYSCON_BOOT_MODE_REMAP_T remap)
{
	LPC_SYSCON->SYSMEMREMAP = (uint32_t) remap;
}

/**
 * @brief	Resets a peripheral
 * @param	periph	:	Peripheral to reset
 * @return	Nothing
 */
STATIC INLINE void Chip_SYSCON_PeriphReset(CHIP_SYSCON_PERIPH_RESET_T periph)
{
	if ( periph < 32 ) {
		LPC_SYSCON->PRESET0CTRL &= ~(1 << (uint32_t) periph);
		LPC_SYSCON->PRESET0CTRL |= (1 << (uint32_t) periph);
	}
	else {
		LPC_SYSCON->PRESET1CTRL &= ~(1 << (uint32_t) (periph-32));
		LPC_SYSCON->PRESET1CTRL |= (1 << (uint32_t) (periph-32));
	}
}

/**
 * @brief	Get system reset status
 * @return	An Or'ed value of SYSCON_RST_*
 * @note	This function returns the detected reset source(s).
 */
STATIC INLINE uint32_t Chip_SYSCON_GetSystemRSTStatus(void)
{
	return LPC_SYSCON->SYSRSTSTAT;
}

/**
 * @brief	Clear system reset status
 * @param	reset	: An Or'ed value of SYSCON_RST_* status to clear
 * @return	Nothing
 * @note	This function clears the specified reset source(s).
 */
STATIC INLINE void Chip_SYSCON_ClearSystemRSTStatus(uint32_t reset)
{
	LPC_SYSCON->SYSRSTSTAT = reset;
}

/**
 * @brief	Read POR captured PIO status
 * @return	captured POR PIO status
 * @note	Some devices only support index 0.
 */
STATIC INLINE uint32_t Chip_SYSCON_GetPORPIOStatus(void)
{
	return LPC_SYSCON->PIOPORCAP0;
}

/**
 * @brief	Set brown-out detection interrupt and reset levels
 * @param	rstlvl	: Brown-out detector reset level
 * @param	intlvl	: Brown-out interrupt level
 * @return	Nothing
 * @note	Brown-out detection reset will be disabled upon exiting this function.
 * Use Chip_SYSCON_EnableBODReset() to re-enable
 */
STATIC INLINE void Chip_SYSCON_SetBODLevels(CHIP_SYSCON_BODRSTLVL_T rstlvl,
											CHIP_SYSCON_BODRINTVAL_T intlvl)
{
	LPC_SYSCON->BODCTRL = ((uint32_t) rstlvl) | (((uint32_t) intlvl) << 2);
}

/**
 * @brief	Enable brown-out detection reset
 * @return	Nothing
 */
STATIC INLINE void Chip_SYSCON_EnableBODReset(void)
{
	LPC_SYSCON->BODCTRL |= (1 << 4);
}

/**
 * @brief	Disable brown-out detection reset
 * @return	Nothing
 */
STATIC INLINE void Chip_SYSCON_DisableBODReset(void)
{
	LPC_SYSCON->BODCTRL &= ~(1 << 4);
}

/**
 * @brief	Set System tick timer calibration value
 * @param	sysCalVal	: System tick timer calibration value
 * @return	Nothing
 */
STATIC INLINE void Chip_SYSCON_SetSYSTCKCAL(uint32_t sysCalVal)
{
	LPC_SYSCON->SYSTCKCAL = sysCalVal;
}

/**
 * @brief	Set System IRQ latency
 * @param	latency	: Latency in clock ticks
 * @return	Nothing
 * @note	Sets the IRQ latency, a value between 0 and 255 clocks. Lower
 * values allow better latency
 */
STATIC INLINE void Chip_SYSCON_SetIRQLatency(uint32_t latency)
{
	LPC_SYSCON->IRQLATENCY = latency;
}

/**
 * @brief	Get System IRQ latency value
 * @return	IRQ Latency in clock ticks
 */
STATIC INLINE uint32_t Chip_SYSCON_GetIRQLatency(void)
{
	return LPC_SYSCON->IRQLATENCY;
}

/**
 * @brief	Set source for non-maskable interrupt (NMI)
 * @param	intsrc	: IRQ number to assign to the NMI
 * @return	Nothing
 * @note	The NMI source will be disabled upon exiting this function. Use the
 * Chip_SYSCON_EnableNMISource() function to enable the NMI source
 */
STATIC INLINE void Chip_SYSCON_SetNMISource(uint32_t intsrc)
{
	LPC_SYSCON->NMISRC = intsrc;
}

/**
 * @brief	Enable interrupt used for NMI source
 * @return	Nothing
 */
STATIC INLINE void Chip_SYSCON_EnableNMISource(void)
{
	LPC_SYSCON->NMISRC |= SYSCON_NMISRC_ENABLE;
}

/**
 * @brief	Disable interrupt used for NMI source
 * @return	Nothing
 */
STATIC INLINE void Chip_SYSCON_DisableNMISource(void)
{
	LPC_SYSCON->NMISRC &= ~(SYSCON_NMISRC_ENABLE);
}

/**
 * @brief	Setup a pin source for the pin interrupts (0-7)
 * @param	intno	: IRQ number
 * @param	pin		: pin number (see comments)
 * @return	Nothing
 * @note	For each pin (0-7) that supports an interrupt, the pin number is assigned
 * to that interrupt with this function. Values 0-17 map to pins PIO0-0 to
 * PIO0-17
 */
STATIC INLINE void Chip_SYSCON_SetPinInterrupt(uint32_t intno, uint32_t pin)
{
	LPC_SYSCON->PINTSEL[intno] = (uint32_t) pin;
}

/**
 * Start enable enumerations - for enabling and disabling peripheral wakeup
 */
typedef enum {
	SYSCON_STARTER_PINT0,
	SYSCON_STARTER_PINT1,
	SYSCON_STARTER_PINT2,
	SYSCON_STARTER_PINT3,
	SYSCON_STARTER_PINT4,
	SYSCON_STARTER_PINT5,
	SYSCON_STARTER_PINT6,
	SYSCON_STARTER_PINT7
} CHIP_SYSCON_WAKEUP0_T;

typedef enum {
	SYSCON_STARTER_SPI0=0,
	SYSCON_STARTER_SPI1,
	SYSCON_STARTER_UART0=3,
	SYSCON_STARTER_UART1,
	SYSCON_STARTER_UART2,
	SYSCON_STARTER_I2C1=6,
	SYSCON_STARTER_I2C0,	
	SYSCON_STARTER_NUL2,
	SYSCON_STARTER_WWDT=12,
	SYSCON_STARTER_BOD,
	SYSCON_STARTER_WKT=15,
	SYSCON_STARTER_I2C2=21,
	SYSCON_STARTER_I2C3,
	SYSCON_STARTER_UART3=30,
	SYSCON_STARTER_UART4
} CHIP_SYSCON_WAKEUP1_T;

/**
 * @brief	Enables a pin's (PINT) wakeup logic
 * @param	pin	: pin number
 * @return	Nothing
 * @note	Different devices support different pins, see the user manual
 * for supported pins
 */
STATIC INLINE void Chip_SYSCON_EnablePINTWakeup(CHIP_SYSCON_WAKEUP0_T pin)
{
	LPC_SYSCON->STARTERP0 |= (1 << pin);
}

/**
 * @brief	Disables a pin's (PINT) wakeup logic
 * @param	pin	: pin number
 * @return	Nothing
 * @note	Different devices support different pins, see the user manual for supported pins.
 */
STATIC INLINE void Chip_SYSCON_DisablePINTWakeup(CHIP_SYSCON_WAKEUP0_T pin)
{
	LPC_SYSCON->STARTERP0 &= ~(1 << pin);
}

/**
 * @brief	Enables peripheral's wakeup logic
 * @param	periphmask	: OR'ed values of SYSCON_WAKEUP_* for wakeup
 * @return	Nothing
 */
STATIC INLINE void Chip_SYSCON_EnablePeriphWakeup(CHIP_SYSCON_WAKEUP1_T periphmask)
{
	LPC_SYSCON->STARTERP1 |= (0x1UL << periphmask);
}

/**
 * @brief	Disables peripheral's wakeup logic
 * @param	periphmask	: OR'ed values of SYSCON_WAKEUP_* for wakeup
 * @return	Nothing
 */
STATIC INLINE void Chip_SYSCON_DisablePeriphWakeup(CHIP_SYSCON_WAKEUP1_T periphmask)
{
	LPC_SYSCON->STARTERP1 &= ~(0x1UL << periphmask);
}

/**
 * @brief	Returns current deep sleep mask
 * @return	OR'ed values of SYSCON_DEEPSLP_* values
 * @note	A high bit indicates the peripheral will power down on deep sleep.
 */
STATIC INLINE uint32_t Chip_SYSCON_GetDeepSleepPD(void)
{
	return LPC_SYSCON->PDSLEEPCFG;
}

/**
 * @brief	Return current wakup mask
 * @return	OR'ed values of SYSCON_SLPWAKE_* values
 * @note	A high state indicates the peripehral will powerup on wakeup.
 */
STATIC INLINE uint32_t Chip_SYSCON_GetWakeup(void)
{
	return LPC_SYSCON->PDAWAKECFG;
}

/**
 * @brief	Power up one or more blocks or peripherals
 * @return	OR'ed values of SYSCON_SLPWAKE_* values
 * @note	A high state indicates the peripheral is powered down.
 */
STATIC INLINE uint32_t Chip_SYSCON_GetPowerStates(void)
{
	return LPC_SYSCON->PDRUNCFG;
}

/**
 * @brief	Return the device ID
 * @return	Device ID
 */
STATIC INLINE uint32_t Chip_SYSCON_GetDeviceID(void)
{
	return LPC_SYSCON->DEVICEID;
}

/**
 * @brief	Setup deep sleep behaviour for power down
 * @param	sleepmask	: OR'ed values of SYSCON_DEEPSLP_* values (high to powerdown on deepsleep)
 * @return	Nothing
 * @note	This must be setup prior to using deep sleep. See the user manual
 * *(PDSLEEPCFG register) for more info on setting this up. This function selects
 * which peripherals are powered down on deep sleep.
 * This function should only be called once with all options for power-down
 * in that call
 */
void Chip_SYSCON_SetDeepSleepPD(uint32_t sleepmask);

/**
 * @brief	Setup wakeup behaviour from deep sleep
 * @param	wakeupmask	: OR'ed values of SYSCON_SLPWAKE_* values (high is powered down)
 * @return	Nothing
 * @note	This must be setup prior to using deep sleep. See the user manual
 * *(PDWAKECFG register) for more info on setting this up. This function selects
 * which peripherals are powered up on exit from deep sleep.
 * This function should only be called once with all options for wakeup
 * in that call
 */
void Chip_SYSCON_SetWakeup(uint32_t wakeupmask);

/**
 * @brief	Power down one or more blocks or peripherals
 * @param	powerdownmask	: OR'ed values of SYSCON_SLPWAKE_* values
 * @return	Nothing
 */
void Chip_SYSCON_PowerDown(SYSCON_PDRUNCFG_T powerdownmask);

/**
 * @brief	Power up one or more blocks or peripherals
 * @param	powerupmask	: OR'ed values of SYSCON_SLPWAKE_* values
 * @return	Nothing
 */
void Chip_SYSCON_PowerUp(SYSCON_PDRUNCFG_T powerupmask);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __SYSCON_84X_H_ */
