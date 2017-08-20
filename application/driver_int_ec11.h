
#ifndef __DRIVER_INT_EC11_H__
#define __DRIVER_INT_EC11_H__

#include "board.h"
#include "stdint.h"

#define EC11_ENABLE         1

#define EC11_PHA_CON        IOCON_PIO0_20
#define EC11_PHA_PRT        0
#define EC11_PHA_PIN        20
#define EC11_PHA_NUM        20

#define EC11_PHB_CON        IOCON_PIO0_21  
#define EC11_PHB_PRT        0
#define EC11_PHB_PIN        21
#define EC11_PHB_NUM        21

#define EC11_KEY_CON        IOCON_PIO0_19  
#define EC11_KEY_PRT        0
#define EC11_KEY_PIN        19
#define EC11_KEY_NUM        19

extern volatile uint8_t g_EC11Status;

extern void ec11_init(void);
extern void ec11_task(void);

#endif  /* __DRIVER_INT_EC11_H__ */
