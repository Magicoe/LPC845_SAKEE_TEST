
#ifndef __DRIVER_ADC_CONTI_H__
#define __DRIVER_ADC_CONTI_H__

#include "board.h"
#include "stdint.h"



#define CONTI_ADC_CON            IOCON_PIO0_23
#define CONTI_ADC_PRT            0
#define CONTI_ADC_PIN            23
#define CONTI_ADC_NUM            23

#define CONTI_ADC_CHANNEL        3

extern volatile uint32_t g_CONTIAdcValue;
extern void continity_adc_task(void);

#endif  /* __DRIVER_I2C_SE95_H__ */
