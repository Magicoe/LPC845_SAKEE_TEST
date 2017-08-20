
#ifndef __DRIVER_SPI_OLED_H__
#define __DRIVER_SPI_OLED_H__

#include "board.h"
#include "gpio_84x.h"
#include "stdint.h"

//#define OLED_USE_GPIO       1
#define OLED_USE_SPI        1

#define OLED_CS_CON         IOCON_PIO1_18
#define OLED_CS_PRT         1
#define OLED_CS_PIN         18

#define OLED_DC_CON         IOCON_PIO0_1
#define OLED_DC_PRT         0
#define OLED_DC_PIN         1

#define OLED_RST_CON        IOCON_PIO0_16
#define OLED_RST_PRT        0
#define OLED_RST_PIN        16

#define OLED_MO_CON         IOCON_PIO1_19
#define OLED_MO_PRT         1
#define OLED_MO_PIN         19
#define OLED_MO_NUM         0x33

#define OLED_CK_CON         IOCON_PIO0_6
#define OLED_CK_PRT         0
#define OLED_CK_PIN         6
#define OLED_CK_NUM         6

#ifdef OLED_USE_GPIO    

#define OLED_CK_CLR()       Chip_GPIO_SetPinOutLow(LPC_GPIO, OLED_CK_PRT,  OLED_CK_PIN)
#define OLED_CK_SET()       Chip_GPIO_SetPinOutHigh(LPC_GPIO, OLED_CK_PRT,  OLED_CK_PIN)

#define OLED_MO_CLR()       Chip_GPIO_SetPinOutLow(LPC_GPIO, OLED_MO_PRT,  OLED_MO_PIN)
#define OLED_MO_SET()       Chip_GPIO_SetPinOutHigh(LPC_GPIO, OLED_MO_PRT,  OLED_MO_PIN)

#define OLED_RST_CLR()      Chip_GPIO_SetPinOutLow(LPC_GPIO, OLED_RST_PRT, OLED_RST_PIN)
#define OLED_RST_SET()      Chip_GPIO_SetPinOutHigh(LPC_GPIO, OLED_RST_PRT, OLED_RST_PIN)

#define OLED_DC_CLR()       Chip_GPIO_SetPinOutLow(LPC_GPIO, OLED_DC_PRT,  OLED_DC_PIN)
#define OLED_DC_SET()       Chip_GPIO_SetPinOutHigh(LPC_GPIO, OLED_DC_PRT,  OLED_DC_PIN)

#define OLED_CS_CLR()       Chip_GPIO_SetPinOutLow(LPC_GPIO, OLED_CS_PRT,  OLED_CS_PIN)
#define OLED_CS_SET()       Chip_GPIO_SetPinOutHigh(LPC_GPIO, OLED_CS_PRT,  OLED_CS_PIN)

#endif

#ifdef OLED_USE_SPI
#define OLED_CS_CON     IOCON_PIO1_18
#define OLED_CS_PRT     1
#define OLED_CS_PIN     18

#define OLED_DC_CON     IOCON_PIO0_1
#define OLED_DC_PRT     0
#define OLED_DC_PIN     1

#define OLED_RST_CON    IOCON_PIO0_16
#define OLED_RST_PRT    0
#define OLED_RST_PIN    16

#define OLED_MO_CON     IOCON_PIO1_19
#define OLED_MO_PRT     1
#define OLED_MO_PIN     19

#define OLED_CK_CON     IOCON_PIO0_6
#define OLED_CK_PRT     0
#define OLED_CK_PIN     6

#define OLED_RST_CLR()      Chip_GPIO_SetPinOutLow(LPC_GPIO, OLED_RST_PRT, OLED_RST_PIN)
#define OLED_RST_SET()      Chip_GPIO_SetPinOutHigh(LPC_GPIO, OLED_RST_PRT, OLED_RST_PIN)

#define OLED_DC_CLR()       Chip_GPIO_SetPinOutLow(LPC_GPIO, OLED_DC_PRT,  OLED_DC_PIN)
#define OLED_DC_SET()       Chip_GPIO_SetPinOutHigh(LPC_GPIO, OLED_DC_PRT,  OLED_DC_PIN)

#define OLED_CS_CLR()       Chip_GPIO_SetPinOutLow(LPC_GPIO, OLED_CS_PRT,  OLED_CS_PIN)
#define OLED_CS_SET()       Chip_GPIO_SetPinOutHigh(LPC_GPIO, OLED_CS_PRT,  OLED_CS_PIN)

#define OLED_SPI_PORT       LPC_SPI0

#endif

#define OLED_SIZE       16

#define OLED_XLevelL    0x00
#define OLED_XLevelH    0x10

#define OLED_MAX_COL    128
#define OLED_MAX_ROW    64

#define	OLED_Brightness 0xFF 

#define OLED_X_WIDTH    128
#define OLED_Y_WIDTH    64

#define OLED_CMD        0   // command
#define OLED_DATA       1   // data


extern void oled_init(void);
extern void oled_delay(uint32_t nCount);
extern void oled_disp_clr(void);
extern void oled_disp_string(uint8_t x,uint8_t y,uint8_t *chr);

extern void oled_task(void *line1, void *line2, void *line3, void *line4);

#endif  /* __DRIVER_SPI_OLED_H__ */
