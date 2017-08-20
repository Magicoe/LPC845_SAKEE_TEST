
#ifndef __DRIVER_DAC_BUZZER_H__
#define __DRIVER_DAC_BUZZER_H__

#include "board.h"
#include "stdint.h"

#define BUZZER_DAC                  LPC_DAC0

#define BUZZER_SWITCH_CON           IOCON_PIO0_18
#define BUZZER_SWITCH_PRT           0
#define BUZZER_SWITCH_PIN           18
#define BUZZER_SWITCH_NUM           18
#define BUZZER_SWITCH_ENA           0                   // 0 enable

#define BUZZER_STANBY_CON           IOCON_PIO1_11
#define BUZZER_STANBY_PRT           1
#define BUZZER_STANBY_PIN           11
#define BUZZER_STANBY_NUM           0x2B                // Dec - 43
#define BUZZER_STANBY_ENA           0                   // 0-stanby, 1-enable
#define BUZZER_STANBY_DIS           1                   // 0-stanby, 1-enable

#define BUZZER_DACOUT_CON           IOCON_PIO0_29
#define BUZZER_DACOUT_PRT           0
#define BUZZER_DACOUT_PIN           29
#define BUZZER_DACOUT_NUM           29

#define WAVEFORMSIZE                32
extern const uint32_t waveform[];

extern void buzzer_init(void);


#endif  /* __DRIVER_DAC_BUZZER_H__ */
