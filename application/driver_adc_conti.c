#include "board.h"
#include <stdio.h>
#include <string.h>

#include "driver_all_board.h"
#include "driver_adc_conti.h"

volatile uint32_t g_CONTIAdcValue = 0;
volatile uint32_t g_ADCValue[2];
void continity_adc_task(void)
{
    if(g_ADCSequenceDoneFlag == 1)
    {
        g_ADCValue[0] = Chip_ADC_GetDataReg(LPC_ADC, 2);
        g_ADCValue[1] = g_CONTIAdcValue = Chip_ADC_GetDataReg(LPC_ADC, 3);
        DEBUGOUT("\r\n ADC2 is 0x%x ADC3 is 0x%x\r\n", ADC_DR_RESULT(g_ADCValue[0]), ADC_DR_RESULT(g_CONTIAdcValue));
        g_ADCSequenceDoneFlag=0;
        /* Manual start for ADC conversion sequence A */
        Chip_ADC_StartSequencer(LPC_ADC, ADC_SEQA_IDX);
    }

}


// end file
