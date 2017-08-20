#include "board.h"
#include <stdio.h>
#include <string.h>

#include "driver_all_board.h"
#include "driver_int_ec11.h"

volatile uint8_t g_EC11Status = 0;

void ec11_init(void)
{
//    Chip_Clock_EnablePeriphClock(SYSCON_CLOCK_GPIOINT);
	/* Enable clock for GPIOINT. */
//	Chip_Clock_SetFClockSource(5, SYSCON_FLEXCOMMCLKSELSRC_FRO);
    Chip_PININT_Init(LPC_PININT);
    Chip_SYSCON_SetPinInterrupt(0, EC11_KEY_NUM);
    Chip_SYSCON_EnablePINTWakeup((CHIP_SYSCON_WAKEUP0_T)0);
    Chip_PININT_SetPinModeEdge(LPC_PININT, PININTCH0);
    Chip_PININT_EnableFallingEdgeOrLevel(LPC_PININT, PININTCH0); // Risign Edge  Chip_PININT_EnableFallingEdgeOrLevel
    /* Enable interrupt in the NVIC */
    NVIC_EnableIRQ(PININT0_IRQn);
    
    Chip_PININT_Init(LPC_PININT);
    Chip_SYSCON_SetPinInterrupt(1, EC11_PHA_NUM);
    Chip_SYSCON_EnablePINTWakeup((CHIP_SYSCON_WAKEUP0_T)1);
    Chip_PININT_SetPinModeEdge(LPC_PININT, PININTCH1);
    Chip_PININT_EnableFallingEdgeOrLevel(LPC_PININT, PININTCH1); // Risign Edge
    /* Enable interrupt in the NVIC */
    NVIC_EnableIRQ(PININT1_IRQn);

    Chip_PININT_Init(LPC_PININT);
    Chip_SYSCON_SetPinInterrupt(2, EC11_PHB_NUM);
    Chip_SYSCON_EnablePINTWakeup((CHIP_SYSCON_WAKEUP0_T)2);
    Chip_PININT_SetPinModeEdge(LPC_PININT, PININTCH2);
    Chip_PININT_EnableFallingEdgeOrLevel(LPC_PININT, PININTCH2); // Risign Edge
    /* Enable interrupt in the NVIC */
    NVIC_EnableIRQ(PININT2_IRQn);
    
    LPC_SYSCON->STARTERP0 = (1 << 0) | (1 << 1) | (1 << 2) | (LPC_SYSCON->STARTERP0 & ~(~0xFFUL));
}

void ec11_delay(void)
{
    uint32_t i;
    for(i=0; i<1000; i++)
    {
        //__WFI();
    }
}

void ec11_task(void)
{
    if(g_EC11Status != 0)
    { 
        if(g_EC11Status == 1)  // Enter Pressed
        {
            ec11_delay();
            if( Chip_GPIO_ReadPortBit(LPC_GPIO_PORT, EC11_KEY_PRT, EC11_KEY_PIN) == 0 )
            {
                DEBUGOUT("EC11 Pressed\r\n");
            }
            else
            {

            }

        }
        if(g_EC11Status == 2)  // Clock Cycle
        {
            DEBUGOUT("EC11 Clock\r\n");
        }
        if(g_EC11Status == 3)  // Clock Cycle
        {
            DEBUGOUT("1234\r\n");
        }
    }        
}


// end file
