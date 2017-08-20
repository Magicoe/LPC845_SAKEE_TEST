
#ifndef __DRIVER_CAPT_BUTTON_H__
#define __DRIVER_CAPT_BUTTON_H__

#include "board.h"
#include "stdint.h"

#define CAPT_ENABLE         1
#define CAPT_ACMP_ENABLE    1

#define CAPT_X0_CON         IOCON_PIO0_31
#define CAPT_X0_PRT         0
#define CAPT_X0_PIN         31
#define CAPT_X0_NUM         31

#define CAPT_X1_CON         IOCON_PIO1_0
#define CAPT_X1_PRT         1
#define CAPT_X1_PIN         0
#define CAPT_X1_NUM         0x20

#define CAPT_YL_CON         IOCON_PIO1_8
#define CAPT_YL_PRT         1
#define CAPT_YL_PIN         8
#define CAPT_YL_NUM         0x28

#define CAPT_YH_CON         IOCON_PIO1_9
#define CAPT_YH_PRT         1
#define CAPT_YH_PIN         9
#define CAPT_YH_NUM         0x29

// ACMP5
#define CAPT_ACMP_CON       IOCON_PIO0_30
#define CAPT_ACMP_PRT       0
#define CAPT_ACMP_PIN       30
#define CAPT_ACMP_NUM       30

// Define the analog comparator input to use
#define ACMP_IN_FOR_CAPTOUCH        ACOMP_IN5
#define ACMP_I_FOR_CAPTOUCH         ACMP_I5
#define ACMP_I_PORT_FOR_CAPTOUCH    ACMP_I5_PORT

#define CAPT_FREQ                   4000000         // Set the divided FCLK frequency

#define CAPT_FDIV                   8               // Use this as a shifter for a 4-bit value

//#define POLLS_PER_SEC 25    // Choose a poll delay
#define CAPT_NOISE_MARGIN       0x40   // Add a noise margin to the baseline threshold
#define CAPT_DATA_SIZE          1         // How many of the most recent touch-data words to save for each X pin
#define CAPT_SENSORS_NUM        2       // How many sensors / X pins are being used
#define CAPT_SAMPLES_NUM        20      // How many consecutive polling rounds w/ ISTOUCH=1 must have the same XVAL to qualify as 'touching'
#define DEBUG_CAPTSENSOR        0       // '1' to output the touched sensor number, '0' to output raw count data for all sensors

#define CAPT_XPINUSE_HIGHZ          (0x0<<12)
#define CAPT_XPINUSE_LOW            (0x1<<12)

#define CAPT_TYPE_TRIGGER_YH   (0x0<<4)
#define CAPT_TYPE_TRIGGER_ACMP (0x1<<4)

// CTRL register 
#define CAPT_POLLMODE_INACTIVE   (0x0<<0)
#define CAPT_POLLMODE_NOW        (0x1<<0)
#define CAPT_POLLMODE_CONTINUOUS (0x2<<0)
#define CAPT_POLLMODE_LOWPWR     (0x3<<0)
#define CAPT_POLLMODE            0             // Use this as a shifter for a 2-bit value

// STATUS, INTENSET, INTENCLR, INTSTAT registers
#define CAPT_YESTOUCH          (1<<0)
#define CAPT_NOTOUCH           (1<<1)
#define CAPT_POLLDONE          (1<<2)
#define CAPT_TIMEOUT           (1<<3)
#define CAPT_OVERRUN           (1<<4)
#define CAPT_BUSY              (1<<8)
#define CAPT_XMAX              (0xFF<<16)      // Use as an AND-mask, when reading this field

// POLL_TCNT register
#define CAPT_TCNT               0              // Use this as a shifter for a 12-bit value
#define CAPT_TOUT               12             // Use this as a shifter for a 4-bit value
#define CAPT_POLL               16             // Use this as a shifter for an 8-bit value
#define CAPT_MDELAY             24             // Use this as a shifter for a 2-bit value
#define CAPT_RDELAY             26             // Use this as a shifter for a 2-bit value
#define CAPT_TCHLOWER           31             // Use this as a shifter for a 1-bit value

// TOUCH register
#define CAPT_TOUCH_COUNT        (0xFFF<<0)     // Use as an AND-mask, when reading this field
#define CAPT_TOUCH_XVAL         (0xF<<12)      // Use as an AND-mask, when reading this field
#define CAPT_TOUCH_ISTOUCH      (0x1<<16)      // Use as an AND-mask, when reading this field
#define CAPT_TOUCH_ISTO         (0x1<<17)      // Use as an AND-mask, when reading this field
#define CAPT_TOUCH_SEQ          (0xF<<20)      // Use as an AND-mask, when reading this field
#define CAPT_TOUCH_CHANGE       (0x1<<31)      // Use as an AND-mask, when reading this field

#define CAPT_X0_ACTV           1<<0
#define CAPT_X1_ACTV           1<<1
#define CAPT_X2_ACTV           1<<2
#define CAPT_X3_ACTV           1<<3
#define CAPT_X4_ACTV           1<<4
#define CAPT_X5_ACTV           1<<5
#define CAPT_X6_ACTV           1<<6
#define CAPT_X7_ACTV           1<<7
#define CAPT_X8_ACTV           1<<8
#define CAPT_X9_ACTV           1<<9
#define CAPT_X10_ACTV          1<<10
#define CAPT_X11_ACTV          1<<11
#define CAPT_X12_ACTV          1<<12
#define CAPT_X13_ACTV          1<<13
#define CAPT_X14_ACTV          1<<14
#define CAPT_X15_ACTV          1<<15
#define CAPT_XPINSEL           16              // Use this as a shifter for a 16-bit value

typedef enum {
  CAPTCLKSEL_FRO_CLK = 0,
  CAPTCLKSEL_MAIN_CLK,
  CAPTCLKSEL_SYSPLL0_CLK,
  CAPTCLKSEL_FRO_DIV_CLK,
  CAPTCLKSEL_WDT_OSC_CLK,
  CAPTCLKSEL_OFF = 7
} SYSCON_CAPTCLKSEL_T;

extern volatile uint8_t  g_CAPTTouching;
extern volatile uint32_t g_xTouchData[CAPT_SENSORS_NUM][CAPT_DATA_SIZE];
extern volatile uint32_t g_xTouchRecord[];
extern volatile uint32_t g_xTouchCnt[];
extern volatile uint32_t g_CAPTTimeOutFlagCnt;
extern volatile uint32_t g_CAPTOverRunFlagCnt;

extern volatile uint32_t g_xTouchLastCnt[];
extern volatile uint32_t g_noTouchBaseLine[];
extern volatile uint32_t g_MeanNoTouchBaseLine;
extern volatile uint8_t  g_CAPTDataLargest;
extern volatile uint8_t  g_xTouchLatestLargests[];

extern volatile uint32_t g_CAPTDataTemp;
extern volatile uint32_t g_CAPTStatusTemp;


//
// Associate the CAPT IRQ slot with the ACMP IRQ slot in the startup code, since they are shared 
//
#define CAPT_IRQHandler CMP_IRQHandler

extern void captouch_init(void);
extern void captouch_task(void);

extern uint8_t capt_find_larger(uint32_t a, uint32_t b);
extern uint8_t capt_is_equal(uint8_t a, uint8_t b);

#endif  /* __DRIVER_DAC_BUZZER_H__ */
