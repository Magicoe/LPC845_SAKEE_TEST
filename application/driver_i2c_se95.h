
#ifndef __DRIVER_I2C_SE95_H__
#define __DRIVER_I2C_SE95_H__

#include "board.h"
#include "stdint.h"

#define SE95_USE_I2C            1

#define SE95_I2C_PORT           LPC_I2C0
#define SE95_I2C_IRQHandler     I2C0_IRQHandler
#define SE95_I2C_IRQNUM         I2C0_IRQn

#define SE95_SDA_CON            IOCON_PIO0_11
#define SE95_SDA_PRT            0
#define SE95_SDA_PIN            11
#define SE95_SDA_NUM            11

#define SE95_SCL_CON            IOCON_PIO0_10
#define SE95_SCL_PRT            0
#define SE95_SCL_PIN            10
#define SE95_SCL_NUM            10

#define SE95_INT_CON            IOCON_PIO0_7
#define SE95_INT_PRT            0
#define SE95_INT_PIN            7
#define SE95_INT_NUM            7

#define SE95_I2C_ADR            0x48

extern void se95_init(void);
extern void se95_read(float *DATA, uint8_t *OrgDATABuf);

#endif  /* __DRIVER_I2C_SE95_H__ */
