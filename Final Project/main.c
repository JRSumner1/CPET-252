//**************************************************************************************
//  TIRSLK Final Project – Explore-and-Route Robot with A* Path-Planning
//  Combines Lab5 (servo + ultrasonic sensing) and Lab7 (BLE + OLED) functionality.
//  Written April 17 2025 – Jonathan Sumner & ChatGPT
//
//  Operation overview
//  Two BLE commands are supported:
//      0x01  EXPLORE – Fully explore the arena/grid, building an occupancy map.
//      0x02  ROUTE   – Follow the shortest path to a user-supplied destination cell
//                      sent immediately after the command as two bytes X,Y (signed).
//  During EXPLORE the robot moves cell-by-cell; at every cell the ultrasonic sensor
//    sweeps L-C-R to mark obstacles.  The resulting 2-D grid is stored in SRAM.
//  During ROUTE the on-board A* implementation (4-neighbors, constant cost=1,
//    Manhattan heuristic) plans a path and the robot drives it, displaying status on
//    the SSD1306.
//  Motor, servo and sensor drivers are unmodified from Labs 5 & 7.
//
//  Hardware (unchanged):
//    * HC-SR04  on P6.2 (TRIG) / P6.3 (ECHO)
//    * Servo    on P9.2 (TimerA3-CCR3 PWM)
//    * Motors   via Port 1/2 + TimerA0 PWM (motor.c)
//    * CC2650   BLE BoosterPack (UART1)
//    * SSD1306  OLED (I2C – see SSD1306.c)
//**************************************************************************************

#include "msp.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/motor.h"
#include "../inc/Init_Ports.h"
#include "../inc/Init_Timers.h"
#include "../inc/AP.h"
#include "../inc/UART0.h"
#include "../inc/UART1.h"
#include "../inc/SSD1306.h"

// ==============================  Lab5 ultrasonic/servo ===============================
#define TRIGGER 0x04   // P6.2
#define ECHO    0x08   // P6.3

static void     Servo(uint16_t counts);
static void     ServoInit(void);
static uint16_t distanceInCm(void);
static uint32_t pulseIn(void);

// PWM constants – 20 ms period, 1 ms (1500) to 2 ms (9500) pulse
#define SERVO_CTR   3900   // ~1.5 ms – center
#define SERVO_LEFT  1500   // left extreme
#define SERVO_RIGHT 9500   // right extreme

// ==============================  Lab7 BLE section ====================================
#define CMD_EXPLORE   0x01
#define CMD_ROUTE     0x02

volatile uint8_t  BLE_Cmd;      // current command byte
volatile int8_t   BLE_X, BLE_Y; // destination when CMD_ROUTE
static void WriteByteCmd(void); // Characteristic write callbacks
static void WriteByteX(void);
static void WriteByteY(void);

// ==============================  Mapping constants ===================================
#define GRID_ROWS  5
#define GRID_COLS  5
#define CELL_SIZE_CM 20      // physical cell dimension (~20 cm square)

// Helper struct reused from Python version
typedef struct { int8_t x; int8_t y; } Point;

// Simple fixed-size open list for A* (sufficient for 256 nodes)
#define OPEN_MAX (GRID_ROWS * GRID_COLS)
static Point   openSet[OPEN_MAX];
static uint16_t openCount;

typedef struct {
    uint8_t walkable;  // 0 = obstacle, 1 = free
    int8_t  parent_x;
    int8_t  parent_y;
    uint16_t g, h, f;
} Node_t;

static Node_t grid[GRID_ROWS][GRID_COLS];
static int8_t  curX = 0, curY = 0;     // robot position – updated after each move

// ==============================  Function prototypes ================================
static void Robot_Explore(void);
static bool Astar_Path(Point start, Point goal, Point *outPath, uint16_t *outLen);
static void Robot_FollowPath(Point *path, uint16_t len);

// ==============================  Main =================================================
int main(void)
{
    volatile int temp;

    DisableInterrupts();
    Clock_Init48MHz();
    UART0_Init();

    // Init hardware from Labs 5 & 7
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
    Motor_Stop();
    SSD1306_Init(SSD1306_SWITCHCAPVCC);
    EnableInterrupts();

    // --- BLE init – three characteristics: command, X, Y ---
    UART0_OutString("\n\rRSLK A* Robot\n\r");
    temp = AP_Init();
    AP_AddService(0xFFF0);

    BLE_Cmd = 0;
    BLE_X = BLE_Y = 0;
    AP_AddCharacteristic(0xFFF1, 1, (uint8_t*) & BLE_Cmd, 0x02, 0x08, "Command", 0, &WriteByteCmd);
    AP_AddCharacteristic(0xFFF2, 1,(uint8_t*) & BLE_X,  0x02, 0x08, "DestinationX", 0, &WriteByteX);
    AP_AddCharacteristic(0xFFF3, 1, (uint8_t*) & BLE_Y,  0x02, 0x08, "DestinationY", 0, &WriteByteY);

    AP_RegisterService();
    AP_StartAdvertisementJacki();

    // Initialize map as unknown–free
    int r;
    int c;
    for (r = 0; r < GRID_ROWS; r++)
    {
        for(c = 0; c < GRID_COLS; c++)
        {
            grid[r][c].walkable = 1;
        }
    }

    while(1){
        AP_BackgroundProcess();      // handle BLE stack
        switch(BLE_Cmd)
        {
            case CMD_EXPLORE:
                SSD1306_ClearBuffer();
                SSD1306_DrawString(0, 0, "Exploring…", WHITE);
                SSD1306_DisplayBuffer();
                Robot_Explore();
                BLE_Cmd = 0;
                break;
            case CMD_ROUTE: {
                    Point start = {curX, curY};
                    Point goal = {BLE_X, BLE_Y};
                    Point path[OPEN_MAX];
                    uint16_t len=0;
                    if (Astar_Path(start, goal, path, &len))
                    {
                        SSD1306_ClearBuffer();
                        SSD1306_DrawString(0, 0, "Routing…", WHITE);
                        SSD1306_DisplayBuffer();
                        Robot_FollowPath(path, len);
                    }
                    else
                    {
                        SSD1306_ClearBuffer();
                        SSD1306_DrawString(0, 0, "No Path!", WHITE);
                        SSD1306_DisplayBuffer();
                    }
                    BLE_Cmd = 0;
                break;
            }
            default:
                // idle / wait
                Clock_Delay1ms(50);
        }
    }
}

// ==============================  BLE Callbacks =======================================
static void WriteByteCmd(void){ /* no extra action – main loop will act */ }
static void WriteByteX(void){ /* nothing, value copied */ }
static void WriteByteY(void){ /* nothing, value copied */ }

// ==============================  Servo + Distance (Lab5) =============================
#define microsecondsToClockCycles(a) ( (a) * 1.5 )
#define clockCyclesToMicroseconds(a) ( (a) / 1.5 )

void Servo(uint16_t angle_count)
{
    TA3CCR0 = 60000 - 1;
    TA3CCR3 = angle_count;
    TA3CTL = TASSEL_2 | ID_2 | MC_1;
    return;
}

void ServoInit(void)
{
    Servo(SERVO_CTR);
    Clock_Delay1ms(500);
    TA3CTL &= ~0x0030;
    return;
}

uint16_t distanceInCm(void)
{
    uint32_t t;
    uint16_t distance;

    P6->OUT |= TRIGGER;
    Clock_Delay1us(10);
    P6->OUT &= ~TRIGGER;

    t = pulseIn();
    if (t == 0) {
        distance = 400;
    }
    else {
        distance = (uint16_t)((0.034/2)*t);
    }
    return distance;
}

static uint32_t pulseIn(void)
{
    uint16_t width = 0;   //will be in clock counts
    uint16_t time = 0;    //the result of converting clock counts to microseconds
    uint16_t maxcount = 56999;  //max count for 38 ms (timeout)

    TA2CTL = 0;
    TA2CTL = TASSEL_2 | ID_3 | MC_2;

    while ((P6 -> IN & ECHO) == 0)
    {
        if(TA2R > maxcount)
        {
            TA2CTL = MC_0;
            return 0;
        }
    }

    TA2R = 0;

    while ((P6 -> IN & ECHO) != 0)
    {
        if(TA2R > maxcount)
        {
            TA2CTL=MC_0;
            return 0;
        }
    }

    width = TA2R;
    TA2CTL = MC_0;

    time = (uint32_t)clockCyclesToMicroseconds(width);
    return time;
}

// ==============================  Exploration routine =================================
static void markObstacle(int8_t x, int8_t y)
{
    if ( x >= 0 && x < GRID_COLS && y >= 0 && y < GRID_ROWS)
    {
        grid[y][x].walkable = 0;
    }
}

static void Robot_Explore(void)
{
    typedef enum {FORWARD,BACKWARD,SCAN_RIGHT,SCAN_LEFT,TURN_RIGHT,TURN_LEFT} ExploreStates;
    uint8_t heading = 2;                        // 0 N, 1 E, 2 S, 3 W  (start SOUTH)
    const int8_t  dx[4] = { 0,  1,  0, -1 };
    const int8_t  dy[4] = { -1, 0,  1,  0 };
    uint32_t steps = 0, MAX_STEPS = GRID_ROWS * GRID_COLS * 4;

    ExploreStates state = FORWARD;          //start in FORWARD state
    ExploreStates prevState = !FORWARD;   //used to know when the state has changed
    uint16_t stateTimer = 0;           //used to stay in a state
    bool isNewState = false;              //true when the state has switched
    uint16_t dist, right_wall, left_wall;

    while (steps < MAX_STEPS) {
        isNewState = (state != prevState);
        prevState = state;

        switch (state)
        {
        case FORWARD:
            if(isNewState)
            {
                dist = 0;
                ServoInit();
                stateTimer = 0;
                Motor_Forward(7500, 7500);
            }

            if(stateTimer >= 50)
            {
                dist = distanceInCm();
                if (dist < 12)
                {
                    markObstacle(curX + dx[heading], curY + dy[heading]);
                    state = SCAN_RIGHT;             // try right next
                    Motor_Stop();
                }
                else
                {
                    curX += dx[heading];
                    curY += dy[heading];
                    steps++;
                    state = SCAN_RIGHT;
                    Motor_Stop();
                }
            }
            break;

        case BACKWARD:
            if(isNewState)
            {
                stateTimer = 0;
                Motor_Backward(7500, 7500);
            }
            if (stateTimer >= 25) {
                /* moved one cell backward */
                curX -= dx[heading];
                curY -= dy[heading];
                steps++;
                state = SCAN_RIGHT;
                Motor_Stop();
            }
            break;

        case SCAN_RIGHT:
            if(isNewState)
            {
                right_wall = 0;
                stateTimer = 0;
                Motor_Stop();
                Servo(1500);
            }

            if(stateTimer >= 50)
            {
                right_wall = distanceInCm();
                state = SCAN_LEFT;
            }
            break;

        case SCAN_LEFT:
            if(isNewState)
            {
                left_wall = 0;
                stateTimer = 0;
                Servo(9500);
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
                heading = (heading + 1) & 3;
                state = FORWARD;
                Motor_Stop();
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
                heading = (heading + 3) & 3;
                state = FORWARD;
                Motor_Stop();
            }
            break;
        }
        Clock_Delay1ms(20);
        stateTimer++;
    }
    Motor_Stop();
}

// ==============================  A* implementation ===================================
static uint16_t heuristic(Point a, Point b)
{
    return abs(a.x - b.x) + abs(a.y - b.y);
}

static bool inClosed(uint8_t closed[GRID_ROWS][GRID_COLS], Point p)
{
    return closed[p.y][p.x];

}
static void addOpen(Point p)
{
    if(openCount < OPEN_MAX)
    {
        openSet[openCount++] = p;
    }
}

static int  lowestF(void)
{
    uint16_t min = 0xFFFF;
    int idx = -1;
    int i;
    for (i = 0; i < openCount; i++)
    {
        Point p = openSet[i];
        uint16_t f = grid[p.y][p.x].f;
        if (f < min)
        {
            min = f;
            idx = i;
        }
    }
    return idx;
}

static bool Astar_Path(Point start, Point goal, Point *outPath, uint16_t *outLen)
{
    uint8_t closed[GRID_ROWS][GRID_COLS] = {0};

    int r;
    int c;
    for (r = 0; r < GRID_ROWS; r++)
    {
        for(c = 0; c < GRID_COLS; c++)
        {
            grid[r][c].g = grid[r][c].h = grid[r][c].f = 0xFFFF;
            grid[r][c].parent_x = -1;
        }
    }
    grid[start.y][start.x].g = 0;
    grid[start.y][start.x].h = heuristic(start,goal);
    grid[start.y][start.x].f = grid[start.y][start.x].h;

    openCount=0;
    addOpen(start);

    while(openCount)
    {
        int idx = lowestF();
        Point current = openSet[idx];
        openSet[idx] = openSet[--openCount];
        if (current.x == goal.x && current.y == goal.y)
        {
            uint16_t len = 0;
            Point p = goal;
            while (!(p.x==start.x&&p.y==start.y))
            {
                outPath[len++] = p;
                int8_t px = grid[p.y][p.x].parent_x;
                int8_t py = grid[p.y][p.x].parent_y;
                p.x = px;
                p.y = py;
                if (len >= OPEN_MAX)
                    break;
            }
            outPath[len++] = start; // include start
            // reverse
            int i;
            for (i = 0; i < len/2; i++)
            {
                Point tmp = outPath[i];
                outPath[i] = outPath[len - 1 - i];
                outPath[len - 1 - i] = tmp;
            }
            *outLen = len;
            return true;
        }

        closed[current.y][current.x] = 1;

        // 4 neighbors N,E,S,W
        const int8_t dx[4]= {0, 1, 0, -1};
        const int8_t dy[4]={-1, 0, 1, 0};

        int n;
        for (n = 0; n < 4; n++)
        {
            Point nb = { current.x + dx[n], current.y + dy[n] };
            if (nb.x < 0 || nb.x >= GRID_COLS || nb.y < 0 || nb.y >= GRID_ROWS) continue;
            if (!grid[nb.y][nb.x].walkable || inClosed(closed,nb)) continue;

            uint16_t tentative = grid[current.y][current.x].g + 1; // cost 1 per step
            if(tentative < grid[nb.y][nb.x].g)
            {
                grid[nb.y][nb.x].parent_x = current.x;
                grid[nb.y][nb.x].parent_y = current.y;
                grid[nb.y][nb.x].g = tentative;
                grid[nb.y][nb.x].h = heuristic(nb,goal);
                grid[nb.y][nb.x].f = grid[nb.y][nb.x].g+grid[nb.y][nb.x].h;

                bool inOpen = false;
                int i;
                for(i = 0; i < openCount; i++)
                {
                    if(openSet[i].x == nb.x && openSet[i].y == nb.y)
                    {
                        inOpen=true;
                        break;
                    }
                }
                if(!inOpen) addOpen(nb);
            }
        }
    }
    return false; // no path
}

// ==============================  Path execution ======================================
static void Robot_FollowPath(Point *path, uint16_t len)
{
    uint8_t heading = 2;                 // 0 N, 1 E, 2 S, 3 W
    const uint16_t SPEED = 7500;         // one PWM value for everything
    const uint16_t TURN90 = 350;         // ms for a 90-degree pivot
    const uint16_t CELL_MS = CELL_SIZE_CM * 50;

    uint16_t i;
    for(i = 1; i < len; i++)
    {
        int8_t dx = path[i].x - path[i-1].x;
        int8_t dy = path[i].y - path[i-1].y;
        uint8_t target =
            (dy ==  1) ? 2 : (dy == -1) ? 0 :
            (dx ==  1) ? 1 : 3;          // S,N,E,W = 2,0,1,3

        // Simple heading adjustment: assume robot always faces +x when start
        uint8_t diff = (target - heading + 4) & 3;   // 0-3
        if (diff == 1) {                             // right 90
            Motor_Right(0, SPEED);
            Clock_Delay1ms(TURN90);
        } else if (diff == 3) {                      // left 90
            Motor_Left(SPEED, 0);
            Clock_Delay1ms(TURN90);
        } else if (diff == 2) {                      // 180
            Motor_Right(0, SPEED);
            Clock_Delay1ms(2 * TURN90);
        }
        Motor_Stop();
        heading = target;

        Motor_Forward(SPEED, SPEED);
        Clock_Delay1ms(CELL_MS);
        Motor_Stop();

        curX = path[i].x;
        curY = path[i].y;
        Clock_Delay1ms(100);
    }
}
