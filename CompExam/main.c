#include "msp.h"

void PortInit(void){
    // Pushbutton S2
    P1->SEL0 &= ~BIT4;
    P1->SEL1 &= ~BIT4;
    P1->DIR &= ~BIT4; // input
    P1->REN |= BIT4; // pullup enabled
    P1->OUT |= BIT4; // active low output

    // Timer Ports - controlled by timer so no P2OUT register write required
    P6->SEL0 |= BIT6;
    P6->SEL1 &= ~BIT6;
    P6->DIR |= BIT6; // output
    P6->REN &= ~BIT6; // pullup disabled
}

void TimerInit(void){
    TIMER_A2->CTL = 0;

    TIMER_A2->CCR[0] = 59999; // period count

    TIMER_A2->CCTL[3] = OUTMOD_7; // reset/set

    TIMER_A2->CCR[3] = 0; // duty count

    TIMER_A2->CTL = TASSEL_2 | ID_1 | MC_1; // SMCLK divided by 2 in Up mode
}

void main(void)
{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer

	PortInit();
	TimerInit();

	while(1)
	{
	    if ((P1->IN & BIT4) == 0) // button pressed/held
	    {
	        TIMER_A2->CCR[3] = 47999; // 80% Duty Cycle
	    }
	    else
	    {
	        TIMER_A2->CCR[3] = 0; // stop the timer
	    }
	}
}
