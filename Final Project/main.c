/******************************************************************************
 * FinalProject.c
 *
 * Demonstration of a TI-RSLK robot final project that includes:
 *  - Motor control (Motor.c)
 *  - Timers, Ports, Clock, Interrupts (Init_Timers.c, Init_Ports.c, Clock.c, CortexM.c)
 *  - LaunchPad usage (LaunchPad.c)
 *  - BLE / UART usage (UART0.c for debug, UART1.c for CC2650)
 *  - SSD1306 OLED display (SSD1306.c)
 *
 * Now featuring a simplified A* path planner based on the Python script (lab1.py),
 * but with:
 *  - No terrain cost (all moves cost the same).
 *  - Only 4 neighbors per cell: up, down, left, right.
 *
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "msp.h"

// --- TI-RSLK libraries ---
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/Motor.h"
#include "../inc/Init_Ports.h"
#include "../inc/Init_Timers.h"
#include "../inc/LaunchPad.h"

// --- BLE / UART code (optional) ---
#include "../inc/UART0.h"   // For debugging over USB
#include "../inc/UART1.h"   // For CC2650
#include "../inc/AP.h"      // If you want to do BLE

// --- SSD1306 OLED library ---
#include "../inc/SSD1306.h"

#define TRIGGER 0x04   // P6.2
#define ECHO    0x08   // P6.3

#define microsecondsToClockCycles(a) ( (a) * 1.5 )       //assume 12Mhz clock divided by 8
#define clockCyclesToMicroseconds(a) ( (a) / 1.5 )       // 1.5 clock cycles = 1us

void Servo(uint16_t angle);
uint32_t pulseIn (void);

void ServoInit(void)  //This function initializes the servo to be centered (0 degrees)
{
    Servo(3900); //call Servo() function to center servo
    Clock_Delay1ms(500); //delay here to give servo time to move - can use built in timer function
    TA3CTL &= ~0x0030; //stop the timer
    return;
}
void Servo(uint16_t angle_count) // this function moves the servo. input: angle_count should be in terms of clock counts to create the desired pulse width in the PWM signal (1-2 ms)
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
uint32_t pulseIn (void)  //this function returns the width of the return pulse from the ultrasonic sensor in terms of microseconds
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

// Example: 4x4 grid, 0=free, 1=obstacle
#define GRID_ROWS   4
#define GRID_COLS   4
static uint8_t grid[GRID_ROWS][GRID_COLS] = {
  {0,0,0,0},
  {0,1,1,0},
  {0,0,0,0},
  {0,0,0,0}
};

static int currentRow = 0;
static int currentCol = 0;
static int destinationRow = 0;
static int destinationCol = 0;

// Data structures for A*
typedef struct {
  int row;
  int col;
  bool open;    // on the open set
  bool closed;  // on the closed set
  int g;        // cost so far
  int f;        // f = g + h
  int parentRow;
  int parentCol;
} Node;

static Node nodes[GRID_ROWS][GRID_COLS];  // One node per cell
static int pathRow[GRID_ROWS * GRID_COLS];
static int pathCol[GRID_ROWS * GRID_COLS];
static int pathLength = 0;

// Simple Manhattan distance as the heuristic
static int heuristic(int r1, int c1, int r2, int c2){
    int dr = (r1 > r2) ? (r1 - r2) : (r2 - r1);
    int dc = (c1 > c2) ? (c1 - c2) : (r2 - c1);
    return dr + dc;
}

// Initialize the node array for each new search
static void AStar_Init(void){
    int r, c;
    for(r=0; r < GRID_ROWS; r++)
    {
        for(c=0; c < GRID_COLS; c++)
        {
            nodes[r][c].row = r;
            nodes[r][c].col = c;
            nodes[r][c].open   = false;
            nodes[r][c].closed = false;
            nodes[r][c].g = 999999;
            nodes[r][c].f = 999999;
            nodes[r][c].parentRow = -1;
            nodes[r][c].parentCol = -1;
        }
    }
    pathLength = 0;
}

// Once exploring is complete, return to starting point
static void reconstructPath(int goalR, int goalC){
    pathLength = 0;
    int r = goalR;
    int c = goalC;

    while(r >= 0 && c >= 0){
        pathRow[pathLength] = r;
        pathCol[pathLength] = c;
        pathLength++;
        int pr = nodes[r][c].parentRow;
        int pc = nodes[r][c].parentCol;
        r = pr;  // move to parent
        c = pc;
        if(pr < 0 || pc < 0) break;
    }
    // Reverse in-place
    int i;
    for(i=0; i < pathLength/2; i++){
        int j = pathLength - 1 - i;

        int tmpRow = pathRow[i];
        pathRow[i] = pathRow[j];
        pathRow[j] = tmpRow;

        int tmpCol = pathCol[i];
        pathCol[i] = pathCol[j];
        pathCol[j] = tmpCol;
    }
}

// Perform A* search from (startRow, startCol) to (goalRow, goalCol)
bool AStar_Search(int startR, int startC, int goalR, int goalC){
    // If start or goal is blocked, fail
    if(grid[startR][startC] == 1) return false;
    if(grid[goalR][goalC]   == 1) return false;

    AStar_Init();
    // Setup the start node
    nodes[startR][startC].g = 0;
    nodes[startR][startC].f = heuristic(startR, startC, goalR, goalC);
    nodes[startR][startC].open = true;

    while(1){
        int bestF = 999999;
        int cr = -1, cc = -1;

        // find the open node with the smallest f
        int r, c;
        for(r=0; r<GRID_ROWS; r++){
            for(c=0; c<GRID_COLS; c++){
                if(nodes[r][c].open && (nodes[r][c].f < bestF)){
                    bestF = nodes[r][c].f;
                    cr = r;
                    cc = c;
                }
            }
        }

        // If we didn't find any open node => path not found
        if(cr < 0 || cc < 0){
            // no solution
            return false;
        }

        // If it's the goal => reconstruct and return
        if(cr == goalR && cc == goalC){
            reconstructPath(goalR, goalC);
            return true;
        }

        // Move this node from open to closed
        nodes[cr][cc].open   = false;
        nodes[cr][cc].closed = true;

        // The 4 neighbors: up, down, left, right
        static int rowOff[4] = { -1, 1, 0, 0 };
        static int colOff[4] = { 0, 0, -1, 1 };

        int i;
        for(i=0; i<4; i++){
            int nr = cr + rowOff[i];
            int nc = cc + colOff[i];

            // Check bounds
            if(nr<0 || nr>=GRID_ROWS || nc<0 || nc>=GRID_COLS) continue;
            // If obstacle or closed, skip
            if(grid[nr][nc] == 1) continue;
            if(nodes[nr][nc].closed) continue;

            // Cost to move from (cr,cc) to (nr,nc) is 1 (no terrain cost).
            int tentG = nodes[cr][cc].g + 1;

            if(!nodes[nr][nc].open){
                // not in open set => add
                nodes[nr][nc].open = true;
            } else {
                // already open => check if this path is better
                if(tentG >= nodes[nr][nc].g){
                    // no improvement
                    continue;
                }
            }
            // This path is better => update parent, cost
            nodes[nr][nc].parentRow = cr;
            nodes[nr][nc].parentCol = cc;
            nodes[nr][nc].g = tentG;
            int h2 = heuristic(nr, nc, goalR, goalC);
            nodes[nr][nc].f = tentG + h2;
        }
    }
}

// Follow the path from pathRow[i], pathCol[i]
void FollowPath(void){
    if(pathLength<=0){
        UART0_OutString("No path.\r\n");
        return;
    }
    // Example approach: assume we start facing 'down' or something.
    // Then for each consecutive pair of cells, figure out the needed turn
    // and move forward. This is a big piece of logic if you want full
    // orientation tracking. We'll do a trivial demo.
    int i;
    for(i=1; i<pathLength; i++){
        int r1 = pathRow[i-1];
        int c1 = pathCol[i-1];
        int r2 = pathRow[i];
        int c2 = pathCol[i];

        // Compare (r1,c1) to (r2,c2) to see which direction we are moving.
        int dr = r2 - r1;
        int dc = c2 - c1;

        // For example: if dr=1 => moving 'down' in the grid
        // This is just a placeholder
        if(dr == 1 && dc == 0){
            // move down one cell
            Motor_Forward(3000, 3000);
            Clock_Delay1ms(1000);
            Motor_Stop();
        } else if(dr == -1 && dc == 0){
            // move up
            // we'd turn to face up, then move
            // etc.
        } else if(dc == 1 && dr == 0){
            // move right
        } else if(dc == -1 && dr == 0){
            // move left
        }
        // Add the turning logic etc. as needed
    }
}

// --------------------------------------------------------------------
// Basic init
// --------------------------------------------------------------------
void System_Init(void){
    DisableInterrupts();
    Clock_Init48MHz();  // 48 MHz
    UART0_Init();
    Port1_Init();
    Port2_Init();
    Port3_Init();
    Port5_Init();
    Port6_Init();
    Port9_Init();
    TimerA0_Init();
    TimerA2_Init();
    TimerA3_Init();
    ServoInit();
    SSD1306_Init(SSD1306_SWITCHCAPVCC);
    EnableInterrupts();

    SSD1306_ClearBuffer();
    SSD1306_DisplayBuffer();
    UART0_OutString("System_Init complete.\r\n");
}

// --------------------------------------------------------------------
// Demo
// --------------------------------------------------------------------
void Demo_AStar(void){
    int startR = 0, startC = 0;    // top-left
    int goalR  = 3, goalC  = 3;    // bottom-right

    bool found = AStar_Search(startR, startC, goalR, goalC);
    if(!found){
        UART0_OutString("No path found!\r\n");
        SSD1306_ClearBuffer();
        SSD1306_DrawString(0,0,"No path!",WHITE);
        SSD1306_DisplayBuffer();
        return;
    }
    // If found, follow it
    UART0_OutString("Path found.\r\n");
    FollowPath();
    // Display success
    SSD1306_ClearBuffer();
    SSD1306_DrawString(0,0,"Path done!", WHITE);
    SSD1306_DisplayBuffer();
}

// --------------------------------------------------------------------
// main
// --------------------------------------------------------------------
int main(void){
    System_Init();

    // Print a message
    UART0_OutString("Press a key to run A*.\r\n");
    char c = UART0_InChar();  // wait

    // Run the search
//    Demo_AStar();

    while(1){
        // Idle or wait for next command
        Clock_Delay1ms(1000);
    }
}
