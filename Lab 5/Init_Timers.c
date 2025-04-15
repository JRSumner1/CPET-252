// Init_Timers.c
// Runs on MSP432
// Provide functions that initialize Timers 
// Prof. Christman
// 6/28/2023

/* These functions will be used for labs 3-8 in CPET253

There is an initialization function for each instantiation of TimerA;
TimerA0, TimerA1, TimerA2, TimerA3

*/

#include "msp.h"


// ------------TimerA0_Init------------
// Before initializing the timer, do the calculations:
//      Elapsed Time = clock counts * (N/Clock Frequency)
//      clock counts cannot exceed 65,535
//      N is the clock scaler

// Input: none
// Output: none
void TimerA0_Init(void){
  // In the TA0CTL register:
  // 	Disable timer for initialization
  // 	Choose a clock source
  // 	Select a clock scaler if needed
  //    Enable overflow interrupt if needed
  // In the TA0EX0 register:
  //    Select a second clock scaler if needed
  // For EACH CCRn being used, in the TA0CCTLn register
  // 	set the PWM outmode if needed
  //    Enable the CCRn match interrupt if needed
  // For PWM, Load TA0CCR0 with clock
  //	counts for desired period
    TA0CTL = 0;

    TA0CCR0 = 29999;

    TA0CCTL3 = OUTMOD_7;
    TA0CCTL4 = OUTMOD_7;

    TA0CCR3  = 0;
    TA0CCR4  = 0;

    TA0CTL = TASSEL_2 | ID_0 | MC_1 | TACLR;
}

// ------------TimerA1_Init------------
// Before initializing the timer, do the calculations:
//      Elapsed Time = clock counts * (N/Clock Frequency)
//      clock counts cannot exceed 65,535
//      N is the clock scaler

// Input: none
// Output: none
void TimerA1_Init(void){
  // In the TA1CTL register:
  // 	Disable timer for initialization
  // 	Choose a clock source
  // 	Select a clock scaler if needed
  //    Enable overflow interrupt if needed
  // In the TA1EX0 register:
  //    Select a second clock scaler if needed
  // For EACH CCRn being used, in the TA1CCTLn register
  // 	set the PWM outmode if needed
  //    Enable the CCRn match interrupt if needed
  // For PWM, Load TA1CCR0 with clock
  //	counts for desired period
}

// ------------TimerA2_Init------------
// Before initializing the timer, do the calculations:
//      Elapsed Time = clock counts * (N/Clock Frequency)
//      clock counts cannot exceed 65,535
//      N is the clock scaler

// Input: none
// Output: none
void TimerA2_Init(void){
  // In the TA2CTL register:
  // 	Disable timer for initialization
  // 	Choose a clock source
  // 	Select a clock scaler if needed
  //    Enable overflow interrupt if needed
  // In the TA2EX0 register:
  //    Select a second clock scaler if needed
  // For EACH CCRn being used, in the TA2CCTLn register
  // 	set the PWM outmode if needed
  //    Enable the CCRn match interrupt if needed
  // For PWM, Load TA2CCR0 with clock
  //	counts for desired period
    TA2CTL = 0;
}

// ------------TimerA3_Init------------
// Before initializing the timer, do the calculations:
//      Elapsed Time = clock counts * (N/Clock Frequency)
//      clock counts cannot exceed 65,535
//      N is the clock scaler

// Input: none
// Output: none
void TimerA3_Init(void){
  // In the TA3CTL register:
  // 	Disable timer for initialization
  // 	Choose a clock source
  // 	Select a clock scaler if needed
  //    Enable overflow interrupt if needed
  // In the TA3EX0 register:
  //    Select a second clock scaler if needed
  // For EACH CCRn being used, in the TA3CCTLn register
  // 	set the PWM outmode if needed
  //    Enable the CCRn match interrupt if needed
  // For PWM, Load TA3CCR0 with clock
  //	counts for desired period
    TA3CTL = 0;
    TA3CCR0 = 60000 - 1;
    TA3CCTL3 = OUTMOD_7;

    TA3CCR3 = 0;
}
