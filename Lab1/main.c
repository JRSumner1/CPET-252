//******************************************************************//
// CPET253 Lab1 � I/O Port Review
//
// author: Jeanne Christman
// original version 1/8/2023
//
// This is a simple program that turns on and off different colors of LED2 based on
// which button is pushed
// -the two pushbuttons are P1.1(S1) and P1.4(S2)
// -The LED is controlled by P2.0(red), P2.1(green) and P2.2(blue)
// -In this program, S1 will turn green on or off and S2 will
// turn blue on or off
// -If the color is off, pressing the pushbutton will turn it on
// -If the color is on, pressing the pushbutton will turn it off
//
//This program is written for the MSP432 Launchpad development board.
//*****************************************************************//
#include "msp.h"
#include <stdint.h>
#include "../inc/Init_Ports.h"

void main()
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD; //stop watchdog timer - always include this

    //Call functions from Init_Ports.c to initialize ports
    Port1_Init();
    Port2_Init();

    while(1)
    {
        if ((P1->IN & BIT1) == 0) //determine if S1 is pushed - remember it is active low
        {
            P2->OUT ^= BIT1; //toggle green on LED2
            while((P1->IN & BIT1) == 0); //wait here for pin to be released.
        }
        if ((P1->IN & BIT4) == 0) //determine if S2 is pushed - remember it is active low
        {
            P2->OUT ^= BIT2; //toggle blue on LED2
            while((P1->IN & BIT4) == 0); //wait here for pin to be released.
        }
    } //end while
    return; //code can never get here
}
