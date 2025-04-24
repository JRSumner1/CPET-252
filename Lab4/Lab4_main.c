/****************************************************************************************
         CPET253 Lab4 - Servo

 Jeanne Christman
 original version 1/19/2025

 This program uses a software state machine to control the Texas Instrument's RSLK robot to drive forward
 for 1 second, back up for 1/4 second, stops and then control a servo motor to sweep 90 degrees right then
 90 degrees left. After sweeping in both directions, the robot turns right 90 degrees and returns to
 forward motion.

 The servo motor is controlled by a PWM signal from TimerA3. The period of the PWM signal is 20ms
 and the pulse width ranges from 1 ms to 2 ms to control the sweep. The PWM signal is output on
 pin P9.2

 Functions in this code:
     -Clock_Init48MHz() - function provided by TI to set system clock
     -Clock_Delay1ms(time) - built in function that delays time ms
     -Port2_Init();
     -Port3_Init();
     -Port5_Init();
     -Port9_Init();
     -TimerA0_Init();
     -TimerA3_Init();
     -Motor_Forward(volatile uint16_t rightDuty, volatile uint16_t leftDuty );
     -Motor_Backward(volatile uint16_t rightDuty, volatile uint16_t leftDuty );
     -Motor_Right(volatile uint16_t rightDuty, volatile uint16_t leftDuty );
     -Motor_Left(volatile uint16_t rightDuty, volatile uint16_t leftDuty );
     -Motor_Stop();

The state machine has 5 states; forward, tun right, backward, sweep right, sweep left
*******************************************************************************************/

#include "msp.h"
#include <msp432.h>
#include <stdint.h>
#include <stdbool.h>
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/motor.h"
#include "../inc/Init_Ports.h"
#include "../inc/Init_Timers.h"

void Servo(uint16_t angle_count);

void ServoInit(void)  //This function initializes the servo to be centered (0 degrees)
{
    Servo(4500); //call Servo() function to center servo
    Clock_Delay1ms(500); //delay here to give servo time to move - can use built in timer function
    TA3CTL &= ~0x0030; //stop the timer
    return;
}
void Servo(uint16_t angle_count) // this function moves the servo.
//input: angle_count should be in terms of clock counts to create the
//desired pulse width in the PWM signal (1-2 ms)
{
    TA3CCR0  = 60000 - 1; //set period for 20ms
    TA3CCR3  = angle_count; //set high time for the input angle using angle_count
    TA3CTL = TASSEL_2 | ID_2 | MC_1; //set timer for up mode
    return;
}

void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer
    Clock_Init48MHz();  // makes bus clock 48 MHz
    //call all the port initialization functions
    //call all the timer initialization functions
    Port2_Init();
    Port3_Init();
    Port5_Init();
    Port9_Init();
    TimerA0_Init();
    TimerA3_Init();
    ServoInit(); //center the servo using the ServoInit() function
    typedef enum {FORWARD, BACKWARD, SWEEP_RIGHT, SWEEP_LEFT, RIGHT} states; //These are the states of the state machine

    states state = FORWARD; //start in FORWARD state
    states prevState = !FORWARD; //used to know when the state has changed
    uint16_t stateTimer = 0;  //used to stay in a state
    bool isNewState = false; //true when the state has switched

     while(1) {
         isNewState = (state != prevState);
         prevState = state;

         switch(state)
         {
         case FORWARD:
             if(isNewState)
             {
                 stateTimer = 0;
                 Motor_Forward(15000, 15000);
             }
             if(stateTimer >= 50)
             {
                 state = BACKWARD;
             }
             break;

         case BACKWARD:
             if(isNewState)
             {
                 stateTimer = 0;
                 Motor_Backward(15000, 15000);
             }
             if(stateTimer >= 12)
             {
                 state = SWEEP_RIGHT;
             }
             break;

         case SWEEP_RIGHT:
             if(isNewState)
             {
                 stateTimer = 0;
                 Motor_Stop();
                 Servo(6000);
             }
             if(stateTimer >= 50)
             {
                 state = SWEEP_LEFT;
             }
             break;

         case SWEEP_LEFT:
             if(isNewState)
             {
                 stateTimer = 0;
                 Servo(3000);
             }
             if(stateTimer >= 50)
             {
                 state = RIGHT;
             }
             break;

         case RIGHT:
             if(isNewState)
             {
                 stateTimer = 0;
                 Motor_Right(0, 15000);
             }
             if(stateTimer >= 25)
             {
                 state = FORWARD;
             }
             break;
         } //switch
         stateTimer++;
         Clock_Delay1ms(20);
     }  //while
}
