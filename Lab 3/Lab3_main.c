/****************************************************************************************
         CPET253 Lab3 - PWM and Motor Drivers

 Jeanne Christman
 original version 6/1/2022

 This program uses a state machine to control the TI-RSLK robot to drive
 in a pattern of forward, right turn, backward, left turn, forward, right turn, ….. 

 To control the motors on the TI-RSLK robot, there are three outputs that need
 to be driven.
    :Pin    :Description            :Notes
    :=======:=======================:=========================
    : P5.5  : Right motor direction : 0=forwards, 1=backwards
    : P3.6  : Right motor sleep     : 0=sleep, 1=awake
    : P2.6  : Right motor PWM       : 0=stop, PWM signal = go
    : P5.4  : Left motor direction  : 0=forwards, 1=backwards
    : P3.7  : Left motor sleep      : 0=sleep, 1= awake
    : P2.7  : Left motor PWM        : 0=stop, PWM signal = go

 Functions in this code:
     -Clock_Init48MHz() - function provided by TI to set system clock
     -Clock_Delay1ms(time) - built in function that delays time ms
     -Port2_Init();
     -Port3_Init();
     -Port5_Init();
     -TimerA0_Init();
     -MotorForward(volatile uint16_t rightDuty, volatile uint16_t leftDuty ); 
     -MotorBackward(volatile uint16_t rightDuty, volatile uint16_t leftDuty ); 
     -MotorTurnRight(volatile uint16_t rightDuty, volatile uint16_t leftDuty ); 
     -MotorTurnLeft(volatile uint16_t rightDuty, volatile uint16_t leftDuty ); 

The state machine has 4 states; forward, right, left, backward
use FSM to make a pattern: Forward, right turn 90 degrees, backwards, left turn 90, forward...
*******************************************************************************************/

#include "msp.h"
#include <stdint.h>
#include <stdbool.h>
#include "..\inc\Clock.h"
#include "..\inc\CortexM.h"
#include "..\inc\Init_Ports.h"
#include "..\inc\Init_Timers.h"
#include "..\inc\motor.h"

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
       //Call the appropriate functions from Init_Timers.c
       Port2_Init();
       Port3_Init();
       Port5_Init();
       TimerA0_Init();
       //These are the four states of the state machine
       typedef enum {FORWARD, RIGHT, BACKWARD, LEFT} motor_states;

       motor_states state = FORWARD;                   //start state
       motor_states prevState = LEFT;               //used to know when the state has changed
       uint16_t stateTimer = 0;       //used to stay in a state
       bool isNewState = false;           //true when the state has switched

       
       while(1)
       {
           isNewState = (state != prevState);
           if(isNewState) {
               stateTimer = 0;
           }
           prevState = state;  //save state for next time

          switch (state) {
          //each case below should have entry housekeeping, state business and exit housekeeping
          //remember to reset the stateTimer each time you enter a new state
          //you must assign a new state when stateTimer reaches the correct value
          case FORWARD:
              if(isNewState) {
                  LED_Color(GREEN);
                  Motor_Forward(7500, 7500);
              }
              // remain in FORWARD until e.g. stateTimer > 200 => 2s
              if(stateTimer >= 200) {
               state = RIGHT; // next state
              }
              break;
          case RIGHT:
              if(isNewState){
                  LED_Color(RED);
                  Motor_Right(0, 7500); // pivot right
              }
              if(stateTimer >= 100) {
                  state = BACKWARD;
              }
              break;
          case BACKWARD:
              if(isNewState){
                  LED_Color(PURPLE);
                  Motor_Backward(7500, 7500);
              }
              if(stateTimer >= 200) {
                  state = LEFT;
              }
              break;
          case LEFT:
              if(isNewState){
                  LED_Color(BLUE);
                  Motor_Left(7500, 0); // pivot left
              }
              if(stateTimer >= 100) {
                  state = FORWARD;
              }
              break;
          } //switch
          Clock_Delay1ms(10);  //10ms delay so that each increment of statetimer is 10ms
          stateTimer++;
       } //while(1)
   } //main()
