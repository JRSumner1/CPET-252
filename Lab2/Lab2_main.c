/****************************************************************************************
         CPET253 Lab2 - State Machines

 Jeanne Christman
 original version 6/28/2023

 This program uses a state machine to display a light show using the multi-colored LED on 
	the MSP432 Launchpad board 
 The sequence of the light show is Green (1 sec), red (.75 sec), blue (1 sec), purple (.75 sec)
	and repeats continuously

 The multi-colored LED is controlled by Port2, pin0(red), pin1(green) and pin2 (blue) 
	Constants are declared for each color
	
Functions called:
	Clock_Init48MHz();   //from Clock.c - sets bus clock to 48Mhz
	Clock_Delay1ms(x);   //from Clock.c - creates a delay of x ms
	Port2_Init();        //from Init_Ports.c - initializes Port2 pins
	LED_color(x);        //written as part of the lab - sets the LED color according to input x
 
The state machine has 4 states
*******************************************************************************************/

#include "msp.h"
#include <stdint.h>
#include <stdbool.h>
#include "..\inc\Clock.h"
#include "..\inc\CortexM.h"
#include "..\inc\Init_Ports.h"

//Declare constants for colored LED
//The LED is controlled by P2.0(red), P2.1(green) and P2.2(blue)
//Make a constant for each RED, GREEN, BLUE and PURPLE
#define RED     0x01
#define GREEN   0x02
#define BLUE    0x04
#define PURPLE  0x05

void LED_Color (uint8_t color)
{
    //This is a simple function to turn on the multi-colored LED on the MSP432 Launchpad
    //board according to the argument passed into the function
    //The LED is controlled by bits 0, 1 and 2 on PORT2
    P2->OUT &= ~(RED | GREEN | BLUE); //first turn off all colors
    P2->OUT |= (color & (RED | GREEN | BLUE)); //second turn on the input color
}

void main(void)
{

       WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer
       Clock_Init48MHz();  // makes bus clock 48 MHz
       //Call the appropriate functions from Init_Ports.c

       //These are the four states of the state machine
       enum

       state                     //start state
       prevState                 //used to know when the state has changed
       uint16_t stateTimer;              //used to stay in a state
       bool isNewState;                  //true when the state has switched

       
       while(1)
       {
           isNewState = (state != prevState);
           prevState = state;  //save state for next time

          switch (state) {
          //each case below should have entry housekeeping, state business and exit housekeeping
          //remember to reset the stateTimer and turn on the LED each time you enter a new state
          //you must assign a new state when stateTimer reaches the correct value
          case
          case
          case
          case
          } //switch
          Clock_Delay1ms(10);  //10ms delay so that each increment of statetimer is 10ms
       } //while(1)
   } //main()
