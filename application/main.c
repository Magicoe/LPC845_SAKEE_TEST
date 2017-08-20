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
#include <stdio.h>

#include "driver_all_board.h"
#include "driver_spi_oled.h"
#include "driver_i2c_se95.h"
#include "driver_capt_button.h"
#include "driver_int_ec11.h"
#include "driver_spi_sdcard.h"
#include "driver_i2c_eeprom.h"
#include "driver_adc_conti.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

#define TICKRATE_HZ (1000)	/* 10 ticks per second */

/* SystemTick Counter */
volatile uint32_t g_SystemTicks = 0;

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/
volatile float   g_SE95TempValue = 0.000;
volatile uint8_t g_SE95OrgiVluae[2];

volatile uint8_t g_OLEDRefreshEnable = 0;


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
	g_SystemTicks++;
    if( (g_SystemTicks % 1000) == 1)
    {
        Board_LED_Toggle(0);
        g_OLEDRefreshEnable = 1;
    }
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
    Chip_Clock_EnablePeriphClock(SYSCON_CLOCK_SPI0);    // OLED display
    Chip_Clock_EnablePeriphClock(SYSCON_CLOCK_SPI1);    // SD card
    Chip_Clock_EnablePeriphClock(SYSCON_CLOCK_I2C0);    // SE95 & I2C Scanner Probe
    Chip_Clock_EnablePeriphClock(SYSCON_CLOCK_IOCON);
    Chip_Clock_EnablePeriphClock(SYSCON_CLOCK_CAPT);    // Captouch
    Chip_Clock_EnablePeriphClock(SYSCON_CLOCK_GPIOINT);
    Chip_Clock_EnablePeriphClock(SYSCON_CLOCK_DAC0);
    Chip_Clock_EnablePeriphClock(SYSCON_CLOCK_DAC1);
    Chip_Clock_EnablePeriphClock(SYSCON_CLOCK_ADC);
    Chip_Clock_EnablePeriphClock(SYSCON_CLOCK_ACOMP);
    
    /* Peripheral reset to the GPIO module. '0' asserts, '1' deasserts reset. */
    //                                I2C0        GPIO0       GPIO1        SPI0         SPI1         ACMP         ADC        DAC0          GPIOINT
    LPC_SYSCON->PRESETCTRL[0] &=  ( (~(1<<6)) & (~(1<<6)) & (~(1<<20)) & (~(1<<11)) & (~(1<<12)) & (~(1<<19)) & (~(1<<24)) & (~(1<<27)) & (~(1<<28)) );
    LPC_SYSCON->PRESETCTRL[0] |= ~( (~(1<<6)) & (~(1<<6)) & (~(1<<20)) & (~(1<<11)) & (~(1<<12)) & (~(1<<19)) & (~(1<<24)) & (~(1<<27)) & (~(1<<28)) );
    
    //                                CAPT        DAC1
    LPC_SYSCON->PRESETCTRL[1] &=  ( (~(1<<0)) & (~(1<<1)) );
    LPC_SYSCON->PRESETCTRL[1] |= ~( (~(1<<0)) & (~(1<<1)) );
    
    board_pins_init();
    board_spi0_init();  // OLED SPI0
    board_spi1_init();  // SDCARD SPI1
    board_i2c0_init();  // SE95 & I2C Scanner Probe
    board_i2c1_init();  // EEPROM
    
    board_adc_init();   // ADC
    
	/* Configure PIN0.0 with pull-up */
	Chip_IOCON_PinSetMode(IOCON_PIO0_0, PIN_MODE_PULLUP);
    Chip_GPIO_SetPinDIROutput(LPC_GPIO, 0, 0);
    Chip_GPIO_SetPinState(LPC_GPIO, 0, 0, true);
    
	/* Enable SysTick Timer */
	SysTick_Config(SystemCoreClock / TICKRATE_HZ);
    
    DEBUGOUT("LPC845 SAKEE Board\r\n");
	
    oled_init();
    oled_disp_clr();
    oled_disp_string(0, 0, (uint8_t *)"NXP LPC845");    
    oled_disp_string(0, 2, (uint8_t *)"SAKEE"); 
    
    se95_init();
    eeprom_init();
    
    board_capt_init();
    captouch_init();
    
    ec11_init();
    
    sdcard_init();
    
    eeprom_test();
    
    board_dac0_init();
    board_dac1_init();
    
	/* Bail out after timeout */
	while (1)
    {
        se95_read((float *)&g_SE95TempValue, (uint8_t *)g_SE95OrgiVluae);
        //DEBUGOUT("SE95 Temperature Value is %f\r\n", g_SE95TempValue);
        //captouch_task();
        
        ec11_task();
        
        if(g_OLEDRefreshEnable == 1)
        {
            oled_task(NULL, (float *)&g_SE95TempValue, NULL, NULL);
            g_OLEDRefreshEnable = 0;
            continity_adc_task();
        }
		//__WFI();
	}


	return 0;
}
