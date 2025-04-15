#include "msp.h"
#include <msp432.h>
#include <stdint.h>
#include <stdbool.h>
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/motor.h"
#include "../inc/Init_Ports.h"
#include "../inc/Init_Timers.h"
#include "../inc/BumpInt.h"

extern volatile int32_t BumpCount;

void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer
    Clock_Init48MHz();  // makes bus clock 48 MHz
    //call all the port initialization functions
    Port2_Init();
    Port3_Init();
    Port4_Init();
    Port5_Init();
    //call all the timer initialization functions
    TimerA0_Init();
    BumpInt_Init();
    typedef enum {FORWARD,BACKWARD,TURN_RIGHT,TURN_LEFT} RobotStates;

    RobotStates state = FORWARD;          //start in FORWARD state
    RobotStates prevState = !FORWARD;   //used to know when the state has changed
    uint16_t stateTimer = 0;           //used to stay in a state
    bool isNewState = false;              //true when the state has switched

    while(1) {
        isNewState = (state != prevState);
        prevState = state;

        switch(state)
        {
        case FORWARD:
            if(isNewState)
            {
                stateTimer = 0;
                Motor_Forward(7500, 7500);
            }

            if(stateTimer >= 50)
            {
                if(BumpCount != 0)
                {
                    state = BACKWARD;
                }
            }
            break;

        case BACKWARD:
            if(isNewState)
            {
                stateTimer = 0;
                Motor_Backward(7500, 7500);
            }
            if(stateTimer >= 50)
            {
                if(BumpCount > 0)
                {
                    state = TURN_LEFT;
                }
                else
                {
                    state = TURN_RIGHT;
                }
            }
            break;

        case TURN_RIGHT:
            if(isNewState)
            {
                stateTimer = 0;
                Motor_Right(0, 7500);
            }
            if(stateTimer >= 25)
            {
                state = FORWARD;
            }
            break;

        case TURN_LEFT:
            if(isNewState)
            {
                stateTimer = 0;
                Motor_Left(7500, 0);
            }
            if(stateTimer >= 25)
            {
                state = FORWARD;
            }
            break;
        } //switch
        Clock_Delay1ms(10);
        stateTimer++;
    }  //while
}

