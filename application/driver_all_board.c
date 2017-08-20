#include "board.h"
#include <stdio.h>

#include "driver_all_board.h"

#include "driver_spi_oled.h"
#include "driver_i2c_se95.h"
#include "driver_capt_button.h"
#include "driver_int_ec11.h"
#include "driver_spi_sdcard.h"
#include "driver_i2c_eeprom.h"
#include "driver_dac_buzzer.h"

volatile uint8_t g_I2C0IntEnable = 0;
I2CM_XFER_T g_I2C0Xfer;

volatile uint8_t g_I2C1IntEnable = 0;
I2CM_XFER_T g_I2C1Xfer;

/**
 * @brief	Handle I2C interrupt by calling I2CM interrupt transfer handler
 * @return	Nothing
 */
void I2C0_IRQHandler(void)
{
	uint32_t state = Chip_I2C_GetPendingInt(LPC_I2C0);

	/* Error handling */
	if (state & (I2C_INTENSET_MSTRARBLOSS | I2C_INTENSET_MSTSTSTPERR)) {
		Chip_I2CM_ClearStatus(LPC_I2C0, I2C_STAT_MSTRARBLOSS | I2C_STAT_MSTSTSTPERR);
	}

	/* Call I2CM ISR function with the I2C device and transfer rec */
	if (state & I2C_INTENSET_MSTPENDING) {
		Chip_I2CM_XferHandler(LPC_I2C0, (I2CM_XFER_T *)&g_I2C0Xfer);
	}
}

/**
 * @brief	Handle I2C interrupt by calling I2CM interrupt transfer handler
 * @return	Nothing
 */
void I2C1_IRQHandler(void)
{
	uint32_t state = Chip_I2C_GetPendingInt(LPC_I2C1);

	/* Error handling */
	if (state & (I2C_INTENSET_MSTRARBLOSS | I2C_INTENSET_MSTSTSTPERR))
    {
		Chip_I2CM_ClearStatus(LPC_I2C1, I2C_STAT_MSTRARBLOSS | I2C_STAT_MSTSTSTPERR);
	}

	/* Call I2CM ISR function with the I2C device and transfer rec */
	if (state & I2C_INTENSET_MSTPENDING)
    {
		Chip_I2CM_XferHandler(LPC_I2C1, (I2CM_XFER_T *)&g_I2C1Xfer);
	}
}

uint32_t DAC0SamplesPerCycle = (WAVEFORMSIZE*4/4);
volatile uint32_t g_DAC0Index = 0;
// DAC interrupt service routine
void DAC0_IRQHandler(void)
{
    LPC_DAC0->CR = 0xFFFF; // waveform[g_DAC0Index++];
    if (g_DAC0Index == DAC0SamplesPerCycle)
    {
        g_DAC0Index = 0;
    }
}

#define DAC1_IRQHandler PIN_INT5_IRQHandler
uint32_t DAC1SamplesPerCycle = (WAVEFORMSIZE*4/4);
volatile uint32_t g_DAC1Index = 0;
// DAC interrupt service routine
void DAC1_IRQHandler(void)
{
    LPC_DAC1->CR = 0xFFFF;//waveform[g_DAC1Index++];
    if (g_DAC1Index == DAC1SamplesPerCycle)
    {
        g_DAC1Index = 0;
    }
}

volatile uint8_t g_ADCSequenceDoneFlag = 0;
/**
 * @brief	Handle interrupt from ADC sequencer A
 * @return	Nothing
 */
void ADC_SEQA_IRQHandler(void)
{
	uint32_t pending;

	/* Get pending interrupts */
	pending = Chip_ADC_GetFlags(LPC_ADC);

	/* Sequence A completion interrupt */
	if (pending & ADC_FLAGS_SEQA_INT_MASK) {
		g_ADCSequenceDoneFlag = 1;
	}

//	/* Threshold crossing interrupt on ADC input channel */
//	if (pending & ADC_FLAGS_THCMP_MASK(3)) {
//		thresholdCrossed = true;
//	}

	/* Clear any pending interrupts */
	Chip_ADC_ClearFlags(LPC_ADC, pending);
}

//
// CAP Touch interrupt service routine
//
void CAPT_IRQHandler(void)
{
  uint8_t n;
  bool flag;
    
  g_CAPTStatusTemp = LPC_CAPT->STATUS;                          // Read the status flags from the STATUS register
  g_CAPTDataTemp = LPC_CAPT->TOUCH;                             // Read the data from the TOUCH register

  if (g_CAPTStatusTemp & (CAPT_YESTOUCH|CAPT_NOTOUCH)) {
    // For the sensor that caused the interrupt, update its circular data buffer (always) and last_touch_cnt variable (only if a touch).
    for (n=0; n!=CAPT_SENSORS_NUM; n++) {
      if (((g_CAPTDataTemp & CAPT_TOUCH_XVAL)>>12) == (uint32_t)n)
      { // Sensor Xn caused this interrupt.
        if (g_CAPTStatusTemp & CAPT_YESTOUCH)                        // If a touch event,
          g_xTouchLastCnt[n] = g_CAPTDataTemp & 0xFFF;           // update the last_touch_cnt variable for this sensor.
#if GET_SENSOR == 0
        g_xTouchData[n][g_xTouchCnt[n]++] = g_CAPTDataTemp;           // Always update the x_touch_data circ. buffer for this sensor and advance the buffer pointer
        if (g_xTouchCnt[n] == CAPT_DATA_SIZE)                         // Wrap the buffer pointer when necessary.
          g_xTouchCnt[n] = 0;          
#endif
        break;
      }
    }

    // If a touch event, only report a 'touching' to main when the previous bunch of largest counts are all from the same sensor.
    // This is a type of moving average filter which searches for DC.
    // It outputs a '1' when all taps contain the same value, '0' otherwise
    if (g_CAPTStatusTemp & CAPT_YESTOUCH) {
      
      // find the largest of the last_touch_cnts
      g_CAPTDataLargest = 0;
      for (n=0; n!=CAPT_SENSORS_NUM-1; n++) {
        if (capt_find_larger(g_xTouchLastCnt[g_CAPTDataLargest], g_xTouchLastCnt[n+1]))
          g_CAPTDataLargest = n+1;
      }
      
      // Shift out the oldest, shift in the newest, and shift everybody else by 1.
      for (n=CAPT_SAMPLES_NUM-1; n>0; n--) {
        g_xTouchLatestLargests[n] = g_xTouchLatestLargests[n-1];
      }
      g_xTouchLatestLargests[0] = g_CAPTDataLargest;
      
      // If all elements of latest_largests[] match, report this as a touched sensor
      flag = 0;
      for (n=CAPT_SAMPLES_NUM-1; n>0; n--) {
        if (capt_is_equal(g_xTouchLatestLargests[n], g_xTouchLatestLargests[n-1])) {
          flag = 1;
        }
        else {
          flag = 0;
          break;
        }
      }
      g_CAPTTouching = flag;
    } // end of if(temp_status & YESTOUCH)

  LPC_CAPT->STATUS = CAPT_YESTOUCH|CAPT_NOTOUCH;      // Clear the interrupt flag by writing '1' to it
  return;                                   // Return from interrupt
  } // end of if(temp_status & (YESTOUCH|NOTOUCH))

  // Other interrupts, increment their counters, clear flags, and return
  if (g_CAPTStatusTemp & (CAPT_TIMEOUT))
  {
    g_CAPTTimeOutFlagCnt++;
    LPC_CAPT->STATUS = CAPT_TIMEOUT;
  }
  if (g_CAPTStatusTemp & (CAPT_OVERRUN))
  {
    g_CAPTOverRunFlagCnt++;
    LPC_CAPT->STATUS = CAPT_OVERRUN;
  }
  return;
} // end of ISR


void PIN_INT0_IRQHandler(void)
{
    if( Chip_GPIO_ReadPortBit(LPC_GPIO_PORT, EC11_KEY_PRT, EC11_KEY_PIN) == 0 )
    {
        g_EC11Status = 1; // Enter
    }
	Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH0);
}

void PIN_INT1_IRQHandler(void)
{
    if( Chip_GPIO_ReadPortBit(LPC_GPIO_PORT, EC11_PHB_PRT, EC11_PHB_PIN) == 0 )
    {
        g_EC11Status = 2;
    }
    if( Chip_GPIO_ReadPortBit(LPC_GPIO_PORT, EC11_PHB_PRT, EC11_PHB_PIN) == 1 )
    {
        g_EC11Status = 3;
    }
	Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH1);
}

void PIN_INT2_IRQHandler(void)
{
    if( Chip_GPIO_ReadPortBit(LPC_GPIO_PORT, EC11_PHA_PRT, EC11_PHA_PIN) == 0 )
    {
        g_EC11Status = 3;
    }
    if( Chip_GPIO_ReadPortBit(LPC_GPIO_PORT, EC11_PHA_PRT, EC11_PHA_PIN) == 1 )
    {
        g_EC11Status = 2;
    }
	Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH2);
}


void board_pins_init(void)
{
// Configure GPIO
#ifdef OLED_USE_GPIO
    Chip_IOCON_PinSetMode(OLED_CS_CON,  PIN_MODE_INACTIVE);
    Chip_IOCON_PinSetMode(OLED_DC_CON,  PIN_MODE_INACTIVE);
    Chip_IOCON_PinSetMode(OLED_RST_CON, PIN_MODE_INACTIVE);
    Chip_IOCON_PinSetMode(OLED_MO_CON,  PIN_MODE_INACTIVE);
    Chip_IOCON_PinSetMode(OLED_CK_CON,  PIN_MODE_INACTIVE);

    OLED_CS_SET();
    OLED_DC_SET();
    OLED_RST_SET();
    OLED_MO_SET();
    OLED_CK_SET();
    
    Chip_GPIO_SetPinDIROutput(LPC_GPIO, OLED_CS_PRT,  OLED_CS_PIN );
    Chip_GPIO_SetPinDIROutput(LPC_GPIO, OLED_DC_PRT,  OLED_DC_PIN );
    Chip_GPIO_SetPinDIROutput(LPC_GPIO, OLED_RST_PRT, OLED_RST_PIN);
    Chip_GPIO_SetPinDIROutput(LPC_GPIO, OLED_MO_PRT,  OLED_MO_PIN );
    Chip_GPIO_SetPinDIROutput(LPC_GPIO, OLED_CK_PRT,  OLED_CK_PIN );
#endif
    
#ifdef OLED_USE_SPI
    Chip_IOCON_PinSetMode(OLED_CS_CON,  PIN_MODE_INACTIVE);
    Chip_IOCON_PinSetMode(OLED_DC_CON,  PIN_MODE_INACTIVE);
    Chip_IOCON_PinSetMode(OLED_RST_CON, PIN_MODE_INACTIVE);
    Chip_IOCON_PinSetMode(OLED_MO_CON,  PIN_MODE_INACTIVE);
    Chip_IOCON_PinSetMode(OLED_CK_CON,  PIN_MODE_INACTIVE);

    OLED_CS_SET();
    OLED_DC_SET();
    OLED_RST_SET();
    
    Chip_GPIO_SetPinDIROutput(LPC_GPIO, OLED_CS_PRT,  OLED_CS_PIN );
    Chip_GPIO_SetPinDIROutput(LPC_GPIO, OLED_DC_PRT,  OLED_DC_PIN );
    Chip_GPIO_SetPinDIROutput(LPC_GPIO, OLED_RST_PRT, OLED_RST_PIN);
    
    Chip_SWM_MovablePinAssign(SWM_SPI0_SCK_IO,  OLED_CK_NUM);
    Chip_SWM_MovablePinAssign(SWM_SPI0_MOSI_IO, OLED_MO_NUM);
#endif

#ifdef SE95_USE_I2C
    Chip_SWM_FixedPinEnable(SWM_FIXED_I2C0_SDA, 1);         // PIO0_11 -- I2C0 SDA
    Chip_SWM_FixedPinEnable(SWM_FIXED_I2C0_SCL, 1);         // PIO0_10 -- I2C0 SCL
	/* Enable Fast Mode Plus */
	Chip_IOCON_PinSetI2CMode(IOCON_PIO0_10, PIN_I2CMODE_FASTPLUS);
	Chip_IOCON_PinSetI2CMode(IOCON_PIO0_11, PIN_I2CMODE_FASTPLUS);
#endif

#ifdef CAPT_ENABLE
    Chip_IOCON_PinSetMode(CAPT_X0_CON,  PIN_MODE_INACTIVE);
    Chip_IOCON_PinSetMode(CAPT_X1_CON,  PIN_MODE_INACTIVE);
    Chip_IOCON_PinSetMode(CAPT_YL_CON,  PIN_MODE_INACTIVE);
    Chip_IOCON_PinSetMode(CAPT_YH_CON,  PIN_MODE_INACTIVE);
    
    Chip_IOCON_PinSetMode(IOCON_PIO0_30,  PIN_MODE_INACTIVE);
    
    Chip_SWM_FixedPinEnable(SWM_FIXED_CAPT_X0, 1);
    Chip_SWM_FixedPinEnable(SWM_FIXED_CAPT_X1, 1);
    Chip_SWM_FixedPinEnable(SWM_FIXED_CAPT_YL, 1);
    Chip_SWM_FixedPinEnable(SWM_FIXED_CAPT_YH, 1);
    
    Chip_SWM_FixedPinEnable(SWM_FIXED_ACMP_I4, 1);
#endif

#ifdef EC11_ENABLE
    Chip_SWM_FixedPinFuncDisable(EC11_PHA_NUM);
    Chip_SWM_FixedPinFuncDisable(EC11_PHB_NUM);
    Chip_SWM_FixedPinFuncDisable(EC11_KEY_NUM);
    
    Chip_IOCON_PinSetMode(EC11_PHA_CON,  PIN_MODE_INACTIVE);
    Chip_IOCON_PinSetMode(EC11_PHB_CON,  PIN_MODE_INACTIVE);
    Chip_IOCON_PinSetMode(EC11_KEY_CON,  PIN_MODE_PULLUP);      // Only this pin without pull-up resistors external
    
    Chip_GPIO_SetPinDIRInput(LPC_GPIO, EC11_PHA_PRT,  EC11_PHA_PIN );
    Chip_GPIO_SetPinDIRInput(LPC_GPIO, EC11_PHB_PRT,  EC11_PHB_PIN );
    Chip_GPIO_SetPinDIRInput(LPC_GPIO, EC11_KEY_PRT,  EC11_KEY_PIN );
#endif

#ifdef SDCARD_ENABLE
    Chip_IOCON_PinSetMode(SDCARD_CS_CON,  PIN_MODE_PULLUP);
    Chip_IOCON_PinSetMode(SDCARD_MI_CON,  PIN_MODE_INACTIVE);
    Chip_IOCON_PinSetMode(SDCARD_MO_CON,  PIN_MODE_INACTIVE);
    Chip_IOCON_PinSetMode(SDCARD_CK_CON,  PIN_MODE_INACTIVE);
    Chip_IOCON_PinSetMode(SDCARD_IN_CON,  PIN_MODE_INACTIVE);
    
    SDCARD_CS_SET();
    Chip_GPIO_SetPinDIROutput(LPC_GPIO, SDCARD_CS_PRT,  SDCARD_CS_PIN );
    
    Chip_GPIO_SetPinDIRInput(LPC_GPIO, SDCARD_IN_PRT,  SDCARD_IN_PIN );   
    
    Chip_SWM_FixedPinFuncDisable(SDCARD_CS_NUM);
    Chip_SWM_FixedPinFuncDisable(SDCARD_IN_NUM);
    Chip_SWM_FixedPinFuncDisable(SDCARD_CK_NUM);
    Chip_SWM_FixedPinFuncDisable(SDCARD_MO_NUM);
    Chip_SWM_FixedPinFuncDisable(SDCARD_MI_NUM);
    
    Chip_SWM_MovablePinAssign(SWM_SPI1_SCK_IO,  SDCARD_CK_NUM);
    Chip_SWM_MovablePinAssign(SWM_SPI1_MOSI_IO, SDCARD_MO_NUM);
    Chip_SWM_MovablePinAssign(SWM_SPI1_MISO_IO, SDCARD_MI_NUM);
#endif

#ifdef EEPROM_USE_I2C
    Chip_IOCON_PinSetMode(EEPROM_SDA_CON, PIN_MODE_INACTIVE);
    Chip_IOCON_PinSetMode(EEPROM_SCL_CON, PIN_MODE_INACTIVE);
    
    Chip_SWM_FixedPinFuncDisable(EEPROM_SDA_NUM);
    Chip_SWM_FixedPinFuncDisable(EEPROM_SCL_NUM);
    
    Chip_SWM_MovablePinAssign(SWM_I2C1_SDA_IO,  EEPROM_SDA_NUM);
    Chip_SWM_MovablePinAssign(SWM_I2C1_SCL_IO,  EEPROM_SCL_NUM);
	/* Enable Fast Mode Plus */
	Chip_IOCON_PinSetI2CMode(EEPROM_SDA_CON, PIN_I2CMODE_FASTPLUS);
	Chip_IOCON_PinSetI2CMode(EEPROM_SCL_CON, PIN_I2CMODE_FASTPLUS);
#endif
// DAC 0 & 1 
    Chip_IOCON_PinSetMode(IOCON_PIO0_17, PIN_MODE_INACTIVE);    // DAC0
    Chip_IOCON_PinSetMode(IOCON_PIO0_29, PIN_MODE_INACTIVE);    // DAC1
    
    Chip_SWM_FixedPinFuncDisable(17);
    Chip_SWM_FixedPinFuncDisable(29);
    
    Chip_SWM_FixedPinEnable(SWM_FIXED_DAC_0, 1);
    Chip_SWM_FixedPinEnable(SWM_FIXED_DAC_1, 1);
#define IOCON_MODE_MASK  (0xFFFFFFe7)
#define IOCON_DACEN_MASK (0xFFFeFFFF)    
#define IOCON_MODE          3
#define IOCON_DAC_ENABLE    16
  // Configure the DACOUT pin. Inactive mode (no pullups/pulldowns), DAC function enabled
    LPC_IOCON->PIO[IOCON_PIO0_17] = ((LPC_IOCON->PIO[IOCON_PIO0_17]) & (IOCON_MODE_MASK) & (IOCON_DACEN_MASK)) | ((0<<IOCON_MODE)|(1<<IOCON_DAC_ENABLE));
    LPC_IOCON->PIO[IOCON_PIO0_29] = ((LPC_IOCON->PIO[IOCON_PIO0_29]) & (IOCON_MODE_MASK) & (IOCON_DACEN_MASK)) | ((0<<IOCON_MODE)|(1<<IOCON_DAC_ENABLE));
    
    // Switch for DAC1
    Chip_IOCON_PinSetMode(BUZZER_SWITCH_CON, PIN_MODE_PULLUP);
    Chip_SWM_FixedPinFuncDisable(BUZZER_SWITCH_NUM);
    
    Chip_GPIO_SetPinOutHigh(LPC_GPIO, BUZZER_SWITCH_PRT, BUZZER_SWITCH_NUM);
    Chip_GPIO_SetPinDIROutput(LPC_GPIO, BUZZER_SWITCH_PRT, BUZZER_SWITCH_PIN );
    // Mute?
    Chip_IOCON_PinSetMode(BUZZER_STANBY_CON, PIN_MODE_PULLUP);
    Chip_SWM_FixedPinFuncDisable(BUZZER_STANBY_NUM);
    
    Chip_GPIO_SetPinOutHigh(LPC_GPIO, BUZZER_STANBY_PRT, BUZZER_STANBY_PIN);
    Chip_GPIO_SetPinDIROutput(LPC_GPIO, BUZZER_STANBY_PRT,  BUZZER_STANBY_PIN);


// ADC Conti
//    Chip_IOCON_PinSetMode(IOCON_PIO0_23, PIN_MODE_INACTIVE);    // ADC3
//    Chip_SWM_FixedPinFuncDisable(23);

}

void board_spi0_init(void)
{
    SPI_CONFIG_T SPIConfigStruct;
    SPI_DELAY_CONFIG_T SPIDelayConfigStruct;
    // GPIO Initialized in driver_all_board.c
    
    // Enable main_clk as function clock to SPI
    LPC_SYSCON->FXCOMCLK9SEL = SYSCON_CLKOUTSRC_MAINCLK; // Here FXCOMCLK9SEL should be SPI0CLKSEL
  
	SPIConfigStruct.Mode      = SPI_MODE_MASTER;
	SPIConfigStruct.ClkDiv    = Chip_SPI_CalClkRateDivider(OLED_SPI_PORT, SPI0_BITRATE);
	SPIConfigStruct.ClockMode = SPI_CLOCK_CPHA0_CPOL0;
	SPIConfigStruct.DataOrder = SPI_DATA_MSB_FIRST;
	SPIConfigStruct.SSELPol   = SPI_SSEL0_ACTIVE_LO;

	Chip_SPI_Init(OLED_SPI_PORT, &SPIConfigStruct);

	SPIDelayConfigStruct.FrameDelay    = 0;
	SPIDelayConfigStruct.PostDelay     = 0;
	SPIDelayConfigStruct.PreDelay      = 0;
	SPIDelayConfigStruct.TransferDelay = 0;
	Chip_SPI_DelayConfig(OLED_SPI_PORT, &SPIDelayConfigStruct);
    
    Chip_SPI_SetControlInfo(OLED_SPI_PORT, 8, (SPI_TXDATCTL_EOF|SPI_TXDATCTL_EOT|SPI_TXDATCTL_RXIGNORE) );
                            
    Chip_SPI_Enable(OLED_SPI_PORT);
}

void board_spi1_init(void)
{
    SPI_CONFIG_T SPIConfigStruct;
    SPI_DELAY_CONFIG_T SPIDelayConfigStruct;
    // GPIO Initialized in driver_all_board.c
    
    // Enable main_clk as function clock to SPI
    LPC_SYSCON->FXCOMCLK10SEL = SYSCON_CLKOUTSRC_MAINCLK; // Here FXCOMCLK10SEL should be SPI10CLKSEL
  
	SPIConfigStruct.Mode      = SPI_MODE_MASTER;
	SPIConfigStruct.ClkDiv    = Chip_SPI_CalClkRateDivider(LPC_SPI1, 100000);
	SPIConfigStruct.ClockMode = SPI_CLOCK_CPHA0_CPOL0;
	SPIConfigStruct.DataOrder = SPI_DATA_MSB_FIRST;
	SPIConfigStruct.SSELPol   = SPI_SSEL0_ACTIVE_LO;

	Chip_SPI_Init(LPC_SPI1, &SPIConfigStruct);

	SPIDelayConfigStruct.FrameDelay    = 0;
	SPIDelayConfigStruct.PostDelay     = 0;
	SPIDelayConfigStruct.PreDelay      = 0;
	SPIDelayConfigStruct.TransferDelay = 0;
	Chip_SPI_DelayConfig(LPC_SPI1, &SPIDelayConfigStruct);
    
    Chip_SPI_SetControlInfo(LPC_SPI1, 8, (SPI_TXDATCTL_EOF|SPI_TXDATCTL_EOT|SPI_TXDATCTL_RXIGNORE) );
                            
    Chip_SPI_Enable(LPC_SPI1);
}


void board_i2c0_init(void)
{
    volatile uint8_t I2C0TxBuf[2];
    volatile uint8_t I2C0RxBuf[2];
    
    Chip_Clock_EnablePeriphClock(SYSCON_CLOCK_I2C0);    // SE95 & I2C0 Scanner Probe
	/* Enable clock for I2C0. */
	Chip_Clock_SetFClockSource(5, SYSCON_FLEXCOMMCLKSELSRC_FRO);
	/* Enable I2C clock and reset I2C peripheral */
	Chip_I2C_Init(LPC_I2C0);
	/* Setup I2CM transfer rate */
	Chip_I2CM_SetBusSpeed(LPC_I2C0, I2C0_BITRATE);
	/* Enable I2C master interface */
	Chip_I2CM_Enable(LPC_I2C0);
    
	g_I2C0Xfer.slaveAddr = 0x01;
	g_I2C0Xfer.txBuff = (const uint8_t *)I2C0TxBuf;
	g_I2C0Xfer.txSz = 1;
	g_I2C0Xfer.rxBuff = (uint8_t *)I2C0RxBuf;
	g_I2C0Xfer.rxSz = 0;
	g_I2C0Xfer.status = 0;
	Chip_I2CM_Xfer(LPC_I2C0, (I2CM_XFER_T *)&g_I2C0Xfer);
    
    g_I2C0IntEnable = 0;
}

void board_i2c1_init(void)
{
    volatile uint8_t I2C1TxBuf[2];
    volatile uint8_t I2C1RxBuf[2];
    
    Chip_Clock_EnablePeriphClock(SYSCON_CLOCK_I2C1);    // SE95 & I2C0 Scanner Probe
	/* Enable clock for I2C1. */
	Chip_Clock_SetFClockSource(6, SYSCON_FLEXCOMMCLKSELSRC_FRO);
	/* Enable I2C clock and reset I2C peripheral */
	Chip_I2C_Init(LPC_I2C1);
	/* Setup I2CM transfer rate */
	Chip_I2CM_SetBusSpeed(LPC_I2C1, I2C1_BITRATE);
	/* Enable I2C master interface */
	Chip_I2CM_Enable(LPC_I2C1);
    
	g_I2C1Xfer.slaveAddr = 0x01;
	g_I2C1Xfer.txBuff = (const uint8_t *)I2C1TxBuf;
	g_I2C1Xfer.txSz = 1;
	g_I2C1Xfer.rxBuff = (uint8_t *)I2C1RxBuf;
	g_I2C1Xfer.rxSz = 0;
	g_I2C1Xfer.status = 0;
	Chip_I2CM_Xfer(LPC_I2C1, (I2CM_XFER_T *)&g_I2C1Xfer);
    
    g_I2C1IntEnable = 0;
}

void board_dac0_init(void)
{
    uint32_t Freq, Div;
    
    Chip_SYSCON_PowerUp(SYSCON_PDRUNCFG_DAC0);
    Chip_Clock_EnablePeriphClock(SYSCON_CLOCK_DAC0);

    
    // Configure the 16-bit DAC counter with an initial Freq. (Can you hear me now?)
    // SamplesPerCycle * Freq = samples/sec
    Freq = 440;
    Div = (Chip_Clock_GetSystemClockRate())/(DAC0SamplesPerCycle * Freq);
    LPC_DAC0->CNTVAL = Div - 1;
    // Write to the CTRL register to start the action. Double buffering enabled, Count enabled.
    LPC_DAC0->CTRL = (1<<DAC_DBLBUF_ENA) | (1<<DAC_CNT_ENA);
    // Enable the DAC interrupt
    NVIC_EnableIRQ(DAC0_IRQn);
}

void board_dac1_init(void)
{
    uint32_t Freq, Div;
    
    Chip_SYSCON_PowerUp(SYSCON_PDRUNCFG_DAC1);
    Chip_Clock_EnablePeriphClock(SYSCON_CLOCK_DAC1);
    
    // Configure the 16-bit DAC counter with an initial Freq. (Can you hear me now?)
    // SamplesPerCycle * Freq = samples/sec
    Freq = 440;
    Div = (Chip_Clock_GetSystemClockRate())/(DAC1SamplesPerCycle * Freq);
    LPC_DAC1->CNTVAL = Div - 1;
    // Write to the CTRL register to start the action. Double buffering enabled, Count enabled.
    LPC_DAC1->CTRL = (1<<DAC_DBLBUF_ENA) | (1<<DAC_CNT_ENA);
    // Enable the DAC interrupt
    NVIC_EnableIRQ(DAC1_IRQn);
}


void board_capt_init(void)
{
#ifdef CAPT_ACMP_ENABLE
    Chip_SYSCON_PowerUp(SYSCON_PDRUNCFG_ACMP);
    // CTRL register setup (when Vp>Vm, ACMP_OUT 0->1 ... when Vp<Vm, ACMP_OUT 1->0 with a little hysteresis thrown in)

// CTRL register shifters
#define HYS         25
#define INTENA      24
#define COMPEDGE    23
#define COMPSTAT    21
#define EDGECLR     20
#define COMP_VM_SEL 11
#define COMP_VP_SEL 8
#define COMPSA      6
#define EDGESEL     3
// HYS field descriptors
#define _20mV       0x3
#define _10mV       0x2
#define _5mV        0x1
#define NONE        0x0
// COMP_VM_SEL and COMP_VP_SEL field descriptors
#define DAC_OUT_0    0x7
#define V_BANDGAP    0x6
#define ACOMP_IN5    0x5
#define ACOMP_IN4    0x4
#define ACOMP_IN3    0x3
#define ACOMP_IN2    0x2
#define ACOMP_IN1    0x1
#define V_LADDER_OUT 0x0
// LAD register shifters
#define LADREF      6
#define LADSEL      1
#define LADEN       0
// LADREF field descriptors
#define VDDCMP_PIN  1
#define SUPPLY_VDD  0

// Hysteresis=20 mV, INTENA=interrupt_disabled, Vm=Vladder, Vp=ACMP_I5 
    LPC_CMP->CTRL = (_20mV<<HYS) | (0<<INTENA) | (V_LADDER_OUT<<COMP_VM_SEL) | (ACMP_IN_FOR_CAPTOUCH<<COMP_VP_SEL);
// Voltage ladder setup
// Reference = Vdd, choose select such that (1/2)*Vdd < select < (2/3.3)*Vdd, ladder enabled
// (2.0 V)/(3.3 V) * 32 = 19d = 13h 
//LPC_CMP->LAD = (SUPPLY_VDD<<LADREF) | (0x0F<<LADSEL) | (1<<LADEN);
//LPC_CMP->LAD = (SUPPLY_VDD<<LADREF) | (0x13<<LADSEL) | (1<<LADEN);
    LPC_CMP->LAD = (SUPPLY_VDD<<LADREF) | (0x10<<LADSEL) | (1<<LADEN);
// Enable an ACMP_IN on its pin 
    // This is enabled in pins
// Configure the ACMP_IN pin in IOCON. Inactive mode (no pullups/pulldowns)
    // This is config in pins 
#endif
}

typedef enum {
  ADCCLKSEL_FRO_CLK = 0,
  ADCCLKSEL_SYSPLL0_CLK,
  ADCCLKSEL_OFF = 3
} SYSCON_ADCCLKSEL_T;

void board_adc_init(void)
{
	/* Setup ADC for 12-bit mode and normal power */
	Chip_ADC_Init(LPC_ADC, 0);

    LPC_SYSCON->ADCCLKDIV = 1;                 // Enable clock, and divide-by-1 at this clock divider
    LPC_SYSCON->ADCCLKSEL = ADCCLKSEL_FRO_CLK; // Use fro_clk as source for ADC async clock
    
	/* Need to do a calibration after initialization and trim */
	Chip_ADC_StartCalibration(LPC_ADC);
	while (!(Chip_ADC_IsCalibrationDone(LPC_ADC))) {}
	/* Setup for maximum ADC clock rate using sycnchronous clocking */
	Chip_ADC_SetClockRate(LPC_ADC, 10000000);
	/* Setup a sequencer to do the following:
	   Perform ADC conversion of ADC channels 0 only */
	Chip_ADC_SetupSequencer(LPC_ADC, ADC_SEQA_IDX, (ADC_SEQ_CTRL_CHANSEL(3)|ADC_SEQ_CTRL_CHANSEL(2) | ADC_SEQ_CTRL_MODE_EOS));

	/* Configure the SWM for P0_23 as the input for the ADC1 */
    Chip_SWM_FixedPinEnable(SWM_FIXED_ADC_2, 1);
	Chip_SWM_FixedPinEnable(SWM_FIXED_ADC_3, 1);
        
	/* Clear all pending interrupts */
	Chip_ADC_ClearFlags(LPC_ADC, Chip_ADC_GetFlags(LPC_ADC));

	/* Enable ADC overrun and sequence A completion interrupts */
	Chip_ADC_EnableInt(LPC_ADC, (ADC_INTEN_SEQA_ENABLE | ADC_INTEN_OVRRUN_ENABLE));
        
	/* Enable ADC NVIC interrupt */
	NVIC_EnableIRQ(ADC_SEQA_IRQn);

	/* Enable sequencer */
	Chip_ADC_EnableSequencer(LPC_ADC, ADC_SEQA_IDX);
        
    /* Manual start for ADC conversion sequence A */
    Chip_ADC_StartSequencer(LPC_ADC, ADC_SEQA_IDX);
}

// end file
