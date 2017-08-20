#include "board.h"
#include <stdio.h>
#include <string.h>

#include "driver_all_board.h"
#include "driver_capt_button.h"

volatile uint8_t  g_CAPTTouching = 0;
volatile uint32_t g_xTouchData[CAPT_SENSORS_NUM][CAPT_DATA_SIZE];
volatile uint32_t g_xTouchRecord[CAPT_SENSORS_NUM];
volatile uint32_t g_xTouchCnt[CAPT_SENSORS_NUM]; // Circular buffer pointers into x_touch_data[NUM_SENSORS][DATA_SIZE]

volatile uint32_t g_xTouchLastCnt[CAPT_SENSORS_NUM];
volatile uint32_t g_noTouchBaseLine[CAPT_SENSORS_NUM];
volatile uint32_t g_MeanNoTouchBaseLine;
volatile uint8_t  g_CAPTDataLargest;
volatile uint8_t  g_xTouchLatestLargests[CAPT_SENSORS_NUM];

volatile uint32_t g_CAPTTimeOutFlagCnt = 0;
volatile uint32_t g_CAPTOverRunFlagCnt = 0;

volatile uint32_t g_CAPTDataTemp = 0;
volatile uint32_t g_CAPTStatusTemp = 0;

#if 0
//
// Compute_Notouch_Baseline() function for CapTouch_2 example
//
// This function changes XPINSEL and POLLMODE in ctrl reg, but restores them before returning.
// It uses a software loop to delay the equivalent time as RDELAY between POLLNOWs.
//
static void Compute_Notouch_Baseline(void)
{
  uint32_t temp_ctrl, save_ctrl, save_poll, n, k, s, divideby, rdelay, mult, waitclks;

  // Read and save the current control reg. and poll_tcnt reg. configurations
  save_ctrl = LPC_CAPT->CTRL;
  save_poll = LPC_CAPT->POLL_TCNT;
  
  // Calculate the number of clocks to wait in reset/draning cap. state between POLLNOWs based on current settings 
  divideby = 1 + ((save_ctrl & (0xF<<CAPT_FDIV))>>CAPT_FDIV);
  rdelay   = (save_poll & (0x3<<CAPT_RDELAY))>>CAPT_RDELAY;
  if (rdelay == 0)
  {  // RDELAY = 0 means no added delay
    mult = 0;
  }
  else
  {  // RDELAY = 1, 2, 3 means add 1+(2^RDELAY) divided FCLKs
    mult = 1 + (1<<rdelay);
  }
  waitclks = mult*divideby;                              // Multiply by the function clock divider to calculate number of system clocks
  
  // Set the ctrl reg. for pollnow with one X pin active for baseline calc. (based on pollmode normal, where only 1 pin is active at a time)
  temp_ctrl = save_ctrl & ~((0x3<<POLLMODE));
  temp_ctrl |= (CAPT_POLLMODE_NOW);
  mean_notouch_baseline = 0;
  
  // Do a bunch of POLLNOWs, separated by the reset delay, and compute the mean count-to-trigger for no-touch
  for (s=0; s!=NUM_SENSORS; s++)
  {
    notouch_baseline[s] = 0;
    temp_ctrl &= ~((uint32_t)0xFFFF<<XPINSEL);
    temp_ctrl |= (1<<s)<<XPINSEL;
    
    for (n = 0; n != 1000; n++) {
      LPC_CAPT->CTRL = temp_ctrl;                          // Do a poll-now
      while (!((LPC_CAPT->STATUS)&POLLDONE));              // Wait for POLLDONE = 1
      notouch_baseline[s] += (LPC_CAPT->TOUCH & TOUCH_COUNT); // Add the current count to the running total
      if (waitclks == 0)                                   // RDELAY = 0 means no added delay
        continue;
      for (k=0; k<waitclks; k++);                          // Else, wait in reset/draining state for the number of clocks specified by RDELAY 
    }
    
    notouch_baseline[s] /= 1000;                           // Calculate the mean, this is the no-touch baseline for this sensor
    mean_notouch_baseline += notouch_baseline[s];          // Update the running sum for all the sensors
  }
  
  mean_notouch_baseline /= NUM_SENSORS;                    // Calculate the average for all the sensors
  LPC_CAPT->CTRL = save_ctrl;                              // Restore CTRL reg.
  
  // Initilize the pointers into x_touch_data[NUM_SENSORS][DATA_SIZE] to all zero
  for (n=0; n!=NUM_SENSORS; n++) {
    x_cnt[n] = 0;
  }
  
  // Initialize the ovr_cnt and to_cnt variables
  ovr_cnt = 0;
  to_cnt = 0;
  
  return;
} // end of function
#endif

void captouch_init(void)
{
    uint32_t ctrl_reg_val, fro_clk, poll_tcnt_reg_val;
    // GPIO Initialized in driver_all_board.c
    // DAC0 Initialized in driver_all_board.c
    
    // Setup the FCLK for the CAP Touch block
    LPC_SYSCON->CAPTCLKSEL = CAPTCLKSEL_FRO_CLK;
    // Specify the divided FCLK freq. 
    fro_clk = 12000000;
    ctrl_reg_val = ((fro_clk/CAPT_FREQ)-1)<<CAPT_FDIV;
    // Choose how selected X pins are controlled when not active (Other X during state 1, charge balance).
    ctrl_reg_val |= CAPT_XPINUSE_HIGHZ;    // High-Z
    // Select what is being used as the trigger
    //ctrl_reg_val |= TYPE_TRIGGER_YH;      // Use the YH pin as input
    ctrl_reg_val |= CAPT_TYPE_TRIGGER_ACMP; // Use the Analog comarator ACMP_0 as input
    // Set some count values
    // Time out = 1<<TOUT = 2^TOUT divided FCLKs. 4096 divided FCLKs ~= 1 ms @ FCLK = 4 MHz
    poll_tcnt_reg_val = 12<<CAPT_TOUT;
    // Poll counter is a 12 bit counter that counts from 0 to 4095 then wraps. POLL is the number of wraps to wait between polling rounds
    //poll_tcnt_reg_val |= ((FREQ)/(POLLS_PER_SEC*4096))<<POLL;
    poll_tcnt_reg_val |= 0<<CAPT_POLL;
    // Choose a value for the TCHLOWER bit.
    // TCHLOWER = 1 means touch triggers at a lower count than no-touch.
    // TCHLOWER = 0 means touch triggers at a higher count than no-touch. 
    //poll_tcnt_reg_val |= 0<<TCHLOWER;
    poll_tcnt_reg_val |= 1U<<CAPT_TCHLOWER;

    // Choose an RDELAY. How many divided FCLKs to hold in step 0 (reset state or draining capacitance)
    // RDELAY has a marked effect on the threshold
    poll_tcnt_reg_val |= 3<<CAPT_RDELAY; // 3 MGN

    // Choose an MDELAY. How many divided FCLKs to wait in measurement mode before reading the YH pin or the comparator output.
    poll_tcnt_reg_val |= 0<<CAPT_MDELAY; // 1 MGN

    // Initialize the Poll and Measurement Counter register, except for the threshold count which will be calculated next.
    LPC_CAPT->POLL_TCNT = poll_tcnt_reg_val;

    // Calculate the no-touch baseline count. Please don't touch sensors while this runs.
   // Compute_Notouch_Baseline();
    // Add a noise margin to the baseline threshold, then write the final value to the actual register
    //poll_tcnt_reg_val |= (mean_notouch_baseline + NOISE_MARGIN)<<TCNT; // Use for TOUCHLOWER = 0
    //poll_tcnt_reg_val |= (mean_notouch_baseline - NOISE_MARGIN)<<TCNT;   // Use for TOUCHLOWER = 1
    poll_tcnt_reg_val |= (1000)<<CAPT_TCNT;   // Use for TOUCHLOWER = 1
    LPC_CAPT->POLL_TCNT = poll_tcnt_reg_val;
   
    // Select any or all available X pins to be used
    ctrl_reg_val |= (CAPT_X0_ACTV)<<CAPT_XPINSEL;
    ctrl_reg_val |= (CAPT_X1_ACTV)<<CAPT_XPINSEL;
//    ctrl_reg_val |= (CAPT_X2_ACTV)<<CAPT_XPINSEL;
//    ctrl_reg_val |= (CAPT_X3_ACTV)<<CAPT_XPINSEL;
//    ctrl_reg_val |= (CAPT_X4_ACTV)<<CAPT_XPINSEL;
//    ctrl_reg_val |= (CAPT_X5_ACTV)<<CAPT_XPINSEL;
//    ctrl_reg_val |= (CAPT_X6_ACTV)<<CAPT_XPINSEL;
//    ctrl_reg_val |= (CAPT_X7_ACTV)<<CAPT_XPINSEL;
//    ctrl_reg_val |= (CAPT_X8_ACTV)<<CAPT_XPINSEL;
    
    // Select the poll mode 
    //ctrl_reg_val |= POLLMODE_NOW;     // One-time-only integration cycle with all selected X pins active simultaneously
    ctrl_reg_val |= CAPT_POLLMODE_CONTINUOUS;// Cycle through the selected X pins continuously (with timing and delays as specified above)
    //ctrl_reg_val |= POLLMODE_LOWPWR;  // This mode has been de-spec'd
  
    // Enable some CAP Touch interrupts
    LPC_CAPT->INTENSET = CAPT_YESTOUCH|CAPT_NOTOUCH|CAPT_TIMEOUT|CAPT_OVERRUN;
    // Enable the CAP Touch IRQ in the NVIC
    NVIC_EnableIRQ(CAPT_IRQn);

    // Start the action
    LPC_CAPT->CTRL = ctrl_reg_val;
    
    DEBUGOUT("Touch a sensor\r\n\n");
#if DEBUG_CAPTSENSOR == 0
//    DEBUGOUT("X0    X1    X2    X3    X4    X5    X6    X7    X8    Timeouts    Overruns\n\r");
//    DEBUGOUT("--    --    --    --    --    --    --    --    --    --------    --------\n\r");
    DEBUGOUT("X0    X1        Timeouts    Overruns\n\r");
    DEBUGOUT("--    --        --------    --------\n\r");
#endif
}


void captouch_task(void)
{
    volatile uint8_t i, j;
#if DEBUG_CAPTSENSOR == 1
    g_CAPTTouching = 0;
    while (!g_CAPTTouching);
    DEBUGOUT("%d", largest+1);
#else
    for (i = 0; i != CAPT_DATA_SIZE; i++)
    {
      for (j=0; j<CAPT_SENSORS_NUM-1; j++)
      {
        DEBUGOUT("%3X   ", g_xTouchData[j][i]&0xFFF);
        g_xTouchRecord[j] = g_xTouchData[j][i]&0xFFF;
      }
      
      DEBUGOUT("%3X   %d           %d\r", g_xTouchData[CAPT_SENSORS_NUM-1][i]&0xFFF, g_CAPTTimeOutFlagCnt, g_CAPTOverRunFlagCnt);
      g_xTouchRecord[j] = g_xTouchData[CAPT_SENSORS_NUM-1][i]&0xFFF;
    }
#endif
}

//
// find_larger() function
//
uint8_t capt_find_larger(uint32_t a, uint32_t b)
{
  if (b > a)
    return 1;
  return 0;  
}

//
// is_equal() function
//
uint8_t capt_is_equal(uint8_t a, uint8_t b)
{
  if (a == b)
    return 1;
  return 0;
}



// end file
