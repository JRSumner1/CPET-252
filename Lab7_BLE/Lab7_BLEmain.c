/****************************************************************************************
         CPET253 Lab7 - OLED LCD and Bluetooth (Library Functions)

 Jeanne Christman
 original version 1/7/2024

 This program uses library functions to use data being received from a smart phone via
 the CC2650 Module Booster Pack and to display to the OLED LCD.
 The Bluetooth data is used to control the TI-RSLK robot. The programmer can choose their own
 commands to move the robot either forward, backward, right, left or stop.  For each case of
 robot motion, the OLED LCD displays the direction.

 Functions in this code:
     -Clock_Init48MHz() - function provided by TI to set system clock
     -EnableInterrupts() - built in function to enable global interrupts
     -DisableInterrupts() - built in function to disable global interrupts
     -Port2_Init();
     -Port3_Init();
     -Port5_Init();
     -TimerA0_Init();
     -Motor_Forward(volatile uint16_t rightDuty, volatile uint16_t leftDuty );
     -Motor_Backward(volatile uint16_t rightDuty, volatile uint16_t leftDuty );
     -Motor_Right(volatile uint16_t rightDuty, volatile uint16_t leftDuty );
     -Motor_Left(volatile uint16_t rightDuty, volatile uint16_t leftDuty );
     -Motor_Stop();
 Also function calls from the provided libraries


*******************************************************************************************/


//------------See AP.c for details of hardware connections to CC2650--------------------
//------------See LaunchPad.c for details of switches and LEDs--------------------------

#include <stdint.h>
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/AP.h"
#include "../inc/UART0.h"
#include "../inc/UART1.h"
#include "../inc/SSD1306.h"
#include "../inc/motor.h"
#include "../inc/Init_Ports.h"
#include "../inc/Init_Timers.h"
#include "msp.h"


uint8_t  BT_ByteData;      // 8-bit user data from the phone


// ********OutValue**********
// Debugging dump of a data value to virtual serial port to PC
// data shown as 1 to 8 hexadecimal characters
// Inputs:  response (number returned by last AP call)
// Outputs: none
void ValueOut(char *label,uint32_t value){
  UART0_OutString(label);
  UART0_OutUHex(value);
}

void MoveRobot (uint8_t command) {
// this function calls the appropriate functions to stop, move forward, move backward, turn right,
// or turn left according to the command received from the BLE.  It also displays to the OLED
// LCD the direction in which the robot is moving
    SSD1306_ClearBuffer();
    switch(command)
    {
        case 1:
            Motor_Forward(3000, 3000);
            SSD1306_DrawString(0, 0, "Direction: Forward", WHITE);
            break;
        case 2:
            Motor_Backward(3000, 3000);
            SSD1306_DrawString(0, 0, "Direction: Backward", WHITE);
            break;
        case 3:
            Motor_Right(0, 3000);
            SSD1306_DrawString(0, 0, "Direction: Right", WHITE);
            break;
        case 4:
            Motor_Left(3000, 0);
            SSD1306_DrawString(0, 0, "Direction: Left", WHITE);
            break;
        case 5:
            Motor_Stop();
            SSD1306_DrawString(0, 0, "Direction: Stop", WHITE);
            break;
        default:
            Motor_Stop();
            SSD1306_DrawString(0, 0, "Direction: INVALID OPTION", WHITE);
            break;
    }
    SSD1306_DisplayBuffer();
}

void WriteByteData(void){ // called on a SNP Characteristic Write Indication on characteristic ByteData
  MoveRobot(BT_ByteData);   // send command to robot
  ValueOut("\n\rWrite BLE_ByteData=",BT_ByteData);
}


int main(void){
  volatile int r;

  DisableInterrupts();
  Clock_Init48MHz();
  UART0_Init();
  //call all necessary Port Initialization functions for the DC motors
  //call all necessary Timer Initialization functions for the DC motors
  Port1_Init();
  Port2_Init();
  Port3_Init();
  Port5_Init();
  TimerA0_Init();
  SSD1306_Init(SSD1306_SWITCHCAPVCC);
  EnableInterrupts();
  UART0_OutString("\n\rApplication Processor - MSP432-CC2650\n\r");
  r = AP_Init();
  AP_GetStatus();  // optional
  AP_GetVersion(); // optional
  AP_AddService(0xFFF0);
  //------------------------
  BT_ByteData = 0;  // write parameter from the phone will be used to control direction
  AP_AddCharacteristic(0xFFF1,1,&BT_ByteData,0x02,0x08,"DirectionData",0,&WriteByteData);

  //------------------------

  AP_RegisterService();
  AP_StartAdvertisementJacki();
  AP_GetStatus(); // optional
  while(1){
    AP_BackgroundProcess();  // handle incoming SNP frames

  }
}
