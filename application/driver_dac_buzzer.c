#include "board.h"
#include <stdio.h>
#include <string.h>

#include "driver_all_board.h"
#include "driver_dac_buzzer.h"

//const uint32_t waveform[WAVEFORMSIZE] = {0x0000, 0x1111, 0x2222, 0x3333, 
//                             0x4444, 0x5555, 0x6666, 0x7777, 
//                             0x8888, 0x9999, 0xAAAA, 0xBBBB,
//                             0xCCCC, 0xDDDD, 0xEEEE, 0xFFFF,
//                             0xFFFF, 0xEEEE, 0xDDDD, 0xCCCC,
//                             0xBBBB, 0xAAAA, 0x9999, 0x8888,
//                             0x7777, 0x6666, 0x5555, 0x4444,
//                             0x3333, 0x2222, 0x1111, 0x0000};

const uint32_t waveform[WAVEFORMSIZE] = {0x0000, 0x0000, 0x0000, 0x0000, 
                             0x0000, 0x0000, 0x0000, 0x0000, 
                             0x0000, 0x0000, 0x0000, 0x0000,
                             0x0000, 0x0000, 0x0000, 0x0000,
                             0x0000, 0x0000, 0x0000, 0x0000,
                             0x0000, 0x0000, 0x0000, 0x0000,
                             0x0000, 0x0000, 0x0000, 0x0000,
                             0x0000, 0x0000, 0x0000, 0x0000};

void buzzer_init(void)
{
    // GPIO Initialized in driver_all_board.c
    // DAC0 Initialized in driver_all_board.c
    

}

void buzzer_enable(void)
{
    Chip_GPIO_SetPinOutHigh(LPC_GPIO, BUZZER_STANBY_PRT, BUZZER_STANBY_PIN);
}

void buzzer_disable(void)
{
    Chip_GPIO_SetPinOutLow(LPC_GPIO, BUZZER_STANBY_PRT, BUZZER_STANBY_PIN);
}

// end file
