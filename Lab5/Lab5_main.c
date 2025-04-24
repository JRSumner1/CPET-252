/****************************************************************************************
         CPET253 Lab5 - Servos and Ultrasonic Sensing

 Jeanne Christman
 original version 1/19/2025

 This program uses an ultrasonic sensor to determine when there is an object in front of a
 forward moving robot. Once it is determined that the robot is approaching an object, it backs up,
 stops and then controls a servo motor to sweep the sensor 90 degrees right then 90 degrees left.
 A distance measurement is taken on each sweep. The robot then turns in the direction that is "more
 clear" and continues forward again.

 The servo motor is controlled by a PWM signal from TimerA3. The period of the PWM signal is 20ms
 and the pulse width ranges from 1 ms to 2 ms to control the sweep. The PWM signal is output on
 pin P9.2

 The ultrasonic sensor trigger is connected to pin P6.2 and the echo is connected to pin P6.3. The
 distance measurement is calculated using the width of the pulse returned from the sensor. TimerA2
 is used to determine the duration of the pulse from the sensor.

 Functions in this code:
     -Clock_Init48MHz() - function provided by TI to set system clock
     -Clock_Delay1ms(time) - built in function that delays time ms
     -Clock_Delay1us(time) - built in function that delays time us
     -Port2_Init();
     -Port3_Init();
     -Port5_Init();
     -Port6_Init();
     -Port9_Init();
     -TimerA0_Init();
     -TimerA2_Init();
     -TimerA3_Init();
     -Motor_Forward(volatile uint16_t rightDuty, volatile uint16_t leftDuty );
     -Motor_Backward(volatile uint16_t rightDuty, volatile uint16_t leftDuty );
     -Motor_Right(volatile uint16_t rightDuty, volatile uint16_t leftDuty );
     -Motor_Left(volatile uint16_t rightDuty, volatile uint16_t leftDuty );
     -Motor_Stop();

The state machine has 6 states; forward, turn right, turn left, backward, sweep right, sweep left
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

#define TRIGGER 0x04
#define ECHO 0x08

#define microsecondsToClockCycles(a) ( (a) * 1.5 )       //assume 12Mhz clock divided by 8
#define clockCyclesToMicroseconds(a) ( (a) / 1.5 )       // 1.5 clock cycles = 1us

void Servo(uint16_t angle);
uint32_t pulseIn (void);

void ServoInit(void)  //This function initializes the servo to be centered (0 degrees)
{
    Servo(18000); //call Servo() function to center servo
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

uint16_t distanceInCm(void) {  //this function measures and returns the distance to the nearest object
    uint32_t t;
    uint16_t distance;

    P6->OUT |= TRIGGER; //drive trigger pin high
    Clock_Delay1us(10); //wait 10 us - can use built-in timer function
    P6->OUT &= ~TRIGGER; //drive trigger pin low

    t = pulseIn();
    if (t == 0) {
        distance = 400; // if no echo (distance = 0), assume object is at farthest distance
    } else {
        distance = (uint16_t)((0.034/2) * t); //calculate distance using s=t * 0.034/2. t comes from pulseIn() function
    }

    return distance; //return the distance
}
uint32_t pulseIn (void)  //this function returns the width of the return pulse
//from the ultrasonic sensor in terms of microseconds
{
    uint16_t width = 0;   //will be in clock counts
    uint16_t time = 0;    //the result of converting clock counts to microseconds
    uint16_t maxcount = 56999;  //max count for 38 ms (timeout)

    TA2CTL = 0;
    TA2CTL = TASSEL_2 | ID_3 | MC_2; //set timer for continuous mode

    //reset the count register
    //wait for the pulse to start (while Echo is low)
    //if count is greater than maxcount return 0
    while((P6->IN & ECHO) == 0) {
        if(TA2R > maxcount) {
            TA2CTL = MC_0;   // stop timer
            return 0;        // timed out, no echo
        }
    }

    TA2R = 0; //reset the count register

    //wait for the pulse to finish (while Echo is high)
    //if count is greater than maxcount return 0
    while((P6->IN & ECHO) != 0) {
        if(TA2R > maxcount) {
            TA2CTL = MC_0;
            return 0;        // timed out, no echo
        }
    }

    width = TA2R; //read the count (width of the return pulse)
    TA2CTL = MC_0; //stop the timer

    time = (uint32_t)clockCyclesToMicroseconds(width); //convert the reading to microseconds.
    return time; //return the microsecond reading
}

void main(void)
{
    uint16_t distance, right_wall, left_wall;

	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer
	Clock_Init48MHz();  // makes bus clock 48 MHz
	//call all the port initialization functions
    Port2_Init();
    Port3_Init();
    Port5_Init();
    Port6_Init();
    Port9_Init();
	//call all the timer initialization functions
    TimerA0_Init();
    TimerA2_Init();
    TimerA3_Init();
    ServoInit(); //center the servo using the ServoInit() function
    typedef enum {FORWARD,BACKWARD,SWEEP_RIGHT,SWEEP_LEFT,TURN_RIGHT,TURN_LEFT} RobotStates;
	
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
                distance = 0;
                ServoInit();
                stateTimer = 0;
                Motor_Forward(7500, 7500);
            }

            if(stateTimer >= 50)
            {
                distance = distanceInCm();
                if (distance < 50)
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
            if(stateTimer >= 25)
            {
                state = SWEEP_RIGHT;
            }
            break;

        case SWEEP_RIGHT:
            if(isNewState)
            {
                right_wall = 0;
                stateTimer = 0;
                Motor_Stop();
                Servo(24000);
            }

            if(stateTimer >= 50)
            {
                right_wall = distanceInCm();
                state = SWEEP_LEFT;
            }
            break;

        case SWEEP_LEFT:
            if(isNewState)
            {
                left_wall = 0;
                stateTimer = 0;
                Servo(12000);
            }

            if(stateTimer >= 50)
            {
                left_wall = distanceInCm();
                if (right_wall > left_wall)
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
        Clock_Delay1ms(20);
        stateTimer++;
	}  //while
}
