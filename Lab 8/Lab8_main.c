#include "msp.h"
#include <stdint.h>
#include <stdbool.h>
#include "..\inc\Clock.h"
#include "..\inc\CortexM.h"
#include "..\inc\Init_Ports.h"
#include "..\inc\Init_Timers.h"
#include "..\inc\motor.h"
#include "..\inc\Reflectance.h"

void main(void)
{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer
	Clock_Init48MHz();  // makes bus clock 48 MHz
    Port2_Init(); // Motor
    Port3_Init(); // Motor
    Port5_Init(); // Motor and Sensor
    Port7_Init(); // Sensor
    Port9_Init(); // Sensor
    TimerA0_Init();
	typedef enum {FORWARD, RIGHT, LEFT, VEER_RIGHT, VEER_LEFT} motor_states;

    motor_states state = FORWARD; //start state
    motor_states prevState = !FORWARD; //used to know when the state has changed
    uint16_t stateTimer = 0; //used to stay in a state
    bool isNewState = false; //true when the state has switched

    while(1)
    {
        isNewState = (state != prevState);
        if(isNewState)
        {
            stateTimer = 0;
        }
        prevState = state;

        uint8_t sensorData = Reflectance_Read(1000);
        int32_t position = Reflectance_Position(sensorData);

        if (position > 287) state = RIGHT;
        else if (position < -287) state = LEFT;
        else if (position < -95) state = VEER_RIGHT;
        else if (position > 95) state = VEER_LEFT;
        else state = FORWARD;

        switch(state)
        {
            case FORWARD:
                Motor_Forward(3000, 3000);
                break;
            case RIGHT:
                Motor_Right(0, 3000);
                break;
            case LEFT:
                Motor_Left(3000, 0);
                break;
            case VEER_RIGHT:
                Motor_Forward(1500, 3000);
                break;
            case VEER_LEFT:
                Motor_Forward(3000, 1500);
                break;
            default: // LOST
                Motor_Stop();
                break;
        } // switch

        Clock_Delay1ms(10);  //10ms delay so that each increment of statetimer is 10ms
        stateTimer++;
    } //while(1)
}
