
#ifndef __DRIVER_ALL_BOARD_H__
#define __DRIVER_ALL_BOARD_H__

#include "board.h"
#include "stdint.h"

#define I2C0_CLK_DIVIDER        2
#define I2C0_BITRATE            400000

#define I2C1_CLK_DIVIDER        2
#define I2C1_BITRATE            400000

#define SPI0_BITRATE            1000000

extern I2CM_XFER_T g_I2C0Xfer;
extern volatile uint8_t g_I2C0IntEnable;

extern I2CM_XFER_T g_I2C1Xfer;
extern volatile uint8_t g_I2C1IntEnable;

extern volatile uint8_t g_ADCSequenceDoneFlag;

extern void board_pins_init(void);

extern void board_spi0_init(void);
extern void board_spi1_init(void);

extern void board_i2c0_init(void);
extern void board_i2c1_init(void);

extern void board_dac0_init(void);
extern void board_dac1_init(void);

extern void board_capt_init(void);

extern void board_adc_init(void);

//------------- DAC ----------------
typedef struct {
  __IO uint32_t CR;             // 0x00
  __IO uint32_t CTRL;           // 0x04
  __IO uint32_t CNTVAL;         // 0x08
} LPC_DAC_T;

#define LPC_DAC0_BASE          (0x40000000UL + 0x14000)
#define LPC_DAC1_BASE          (0x40000000UL + 0x18000)

#define LPC_DAC0              ((LPC_DAC_T    *) LPC_DAC0_BASE  )
#define LPC_DAC1              ((LPC_DAC_T    *) LPC_DAC1_BASE  )

// D/A Converter Register (VAL) shifters
#define DAC_VALUE 6
#define DAC_BIAS  16

// CTRL register shifters
#define DAC_INT_DMA_REQ  0
#define DAC_DBLBUF_ENA   1
#define DAC_CNT_ENA      2
#define DAC_DMA_ENA      3


#endif  /* __DRIVER_ALL_BOARD_H__ */
