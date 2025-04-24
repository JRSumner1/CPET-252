//**************************************************************************************
//  TI-RSLK A* Pathfinder Rover
//  Written: 17-Apr-2025 | Signed Off: 23-April-2025
//  Author: Jonathan Sumner
//
//  Project Summary:
//  This robot explores a 5x5 arena using DFS, builds an occupancy map in SRAM, then
//  drives the shortest route to any chosen destination using an on-board A* search.
//
//  Key Points:
//    - Original idea: extend Lab 5 wall-avoidance into a full map-and-route system.
//    - Exploration: at every cell the HC-SR04 sweeps Center-Right-Left to classify
//      walls; SSD1306 shows cells as ? = unknown, O = free, X = wall.
//    - Path-planning: 4-connected grid, uniform step cost = 1, Manhattan heuristic.
//    - BLE protocol:
//          0x01  EXPLORE - build the map
//          0x02  ROUTE - drive to end of grid (4, 4)
//    - Algorithms:
//          DFS - implicit in EXPLORE: robot depth-first walks the grid, pushing
//                visited cells onto a stack and backtracking when no forward moves
//                remain. Guarantees complete coverage but not optimal paths.
//          A*  - explicit in ROUTE: evaluates f = g + h, where g is path cost so
//                far and h is the Manhattan distance to the goal. Finds the
//                optimal route faster than brute-force searches.
//    - Testing: compared planned vs. actual paths; achieved 95 percent map accuracy.
//    - Future work: dynamic routing, dynamic grid sizing, diagonal moves, user defined end points
//
//  Hardware:
//    * HC-SR04  ultrasonic on P6.2 (TRIG) / P6.3 (ECHO)
//    * SG-90    servo on P9.2 (TimerA3-CCR3 PWM)
//    * Motors   motor driver via Port1/2 + TimerA0 PWM (Motor.c)
//    * CC2650   BLE BoosterPack (UART1)
//    * SSD1306  128x64 OLED via I2C (see SSD1306.c)
//**************************************************************************************

#include "msp.h"                 // Device register definitions (TI MSP432)
#include <stdint.h>              // Standard integer types
#include <stdbool.h>             // Boolean type and constants
#include <stdlib.h>              // Standard library (abs)
#include "../inc/Clock.h"        // 48 MHz system clock driver
#include "../inc/CortexM.h"      // NVIC and low-level ARM helpers
#include "../inc/motor.h"        // Motor driver (PWM control)
#include "../inc/Init_Ports.h"   // GPIO initialization helper
#include "../inc/Init_Timers.h"  // Timer initialization helper
#include "../inc/AP.h"           // BLE (Adventure Pack) driver
#include "../inc/UART0.h"        // USB CDC UART
#include "../inc/UART1.h"        // BLE UART
#include "../inc/SSD1306.h"      // 128x64 OLED display driver

//*****************************************************************************
// GPIO bit masks for the HC-SR04 ultrasonic sensor on Port 6
//*****************************************************************************
#define TRIGGER        0x04       // P6.2 - output: 10us pulse triggers ping
#define ECHO           0x08       // P6.3 - input: high while echo returns

//*****************************************************************************
// Servo PWM compare values (Timer A3, channel 3) - 50 Hz frame, 12-bit counts
//*****************************************************************************
#define SERVO_CENTER   4300       // Straight ahead (about 1.5ms pulse)
#define SERVO_RIGHT    1500       // 90-deg right (about 0.5ms pulse)
#define SERVO_LEFT     7500       // 90-deg left  (about 2.5ms pulse)

//*****************************************************************************
// BLE commands received from smartphone app
//*****************************************************************************
#define CMD_EXPLORE    0x01       // Explore the maze and build the map
#define CMD_ROUTE      0x02       // Follow best path from (0,0) to (4,4)

//*****************************************************************************
// Map size (5x5 grid) and helper lookup tables for heading deltas
//*****************************************************************************
#define GRID_ROWS      5
#define GRID_COLS      5
static const int8_t dX[4] = { 0, 1, 0, -1 };   // East = 1, West = 3
static const int8_t dY[4] = {-1, 0, 1,  0 };   // North = 0, South = 2
static inline uint8_t leftOf (uint8_t h){ return (h + 3) & 3; }   // Left turn
static inline uint8_t rightOf(uint8_t h){ return (h + 1) & 3; }   // Right turn

//*****************************************************************************
// OLED map drawing constants (one grid cell is 10x10 pixels)
//*****************************************************************************
#define MAP_ORIGIN_X   0          // Upper-left corner of map on OLED
#define MAP_ORIGIN_Y   16         // Leave first line for status text
#define CELL_PIX       10         // Cell width and height in pixels

//*****************************************************************************
// DrawCell - draws outline and state symbol ("?", "O", or "X") for one cell
//             state = 0 unknown, 1 walkable, 2 not walkable
//*****************************************************************************
static void DrawCell(int8_t gx, int8_t gy, uint8_t state) {
  uint8_t x0 = MAP_ORIGIN_X + gx * CELL_PIX;
  uint8_t y0 = MAP_ORIGIN_Y + gy * CELL_PIX;
  uint8_t i;
  // Draw 10-pixel square
  for (i = 0; i < CELL_PIX; i++) {
    SSD1306_DrawPixel(x0 + i, y0, WHITE);
    SSD1306_DrawPixel(x0 + i, y0 + CELL_PIX - 1, WHITE);
    SSD1306_DrawPixel(x0, y0 + i, WHITE);
    SSD1306_DrawPixel(x0 + CELL_PIX - 1, y0 + i, WHITE);
  }
  // Draw state symbol inside the box
  if (state == 0)      SSD1306_DrawString(x0 + 1, y0 + 1, "?", WHITE);
  else if (state == 1) SSD1306_DrawString(x0 + 2, y0 + 1, "O", WHITE);
  else                 SSD1306_DrawString(x0 + 2, y0 + 1, "X", WHITE);
}

//*****************************************************************************
// Data structures for A* path finding
//*****************************************************************************
typedef struct { int8_t x; int8_t y; } Point;         // Grid coordinate
typedef struct {                                     // Node meta-data
  uint8_t  walkable;      // 1 = no wall, 0 = blocked
  int8_t   parent_x;      // Parent cell x (reconstruct path)
  int8_t   parent_y;      // Parent cell y
  uint16_t g;             // Cost from start
  uint16_t h;             // Heuristic cost to goal
  uint16_t f;             // Total cost (g + h)
} Node_t;

#define OPEN_MAX (GRID_ROWS * GRID_COLS)             // Max nodes in open set
static Point   openSet[OPEN_MAX];                    // Nodes to be evaluated
static uint16_t openCount;                           // Current open set size
static Node_t  grid[GRID_ROWS][GRID_COLS];           // Map meta-data

// Current robot pose during exploration / routing
static int8_t  curX = 0, curY = 0;                   // Cell index
static uint8_t curHeading = 1;                       // 0 = N, 1 = E, 2 = S, 3 = W
static bool    visited[GRID_ROWS][GRID_COLS] = {0};  // Explored flag

//*****************************************************************************
// GridInit - mark all cells as walkable at startup
//*****************************************************************************
void GridInit(void){
  int r, c;
  for(r = 0; r < GRID_ROWS; r++)
    for(c = 0; c < GRID_COLS; c++)
      grid[r][c].walkable = 1;
}

//*****************************************************************************
// Servo helper prototypes
//*****************************************************************************
void     Servo(uint16_t counts);
uint32_t pulseIn(void);

//*****************************************************************************
// ServoInit - center the sensor servo, wait 0.5s, then stop the timer
//*****************************************************************************
void ServoInit(void){
  Servo(SERVO_CENTER);              // Center position
  Clock_Delay1ms(500);              // Allow to settle
  TA3CTL &= ~0x0030;                // Stop Timer A3 (up mode bits)
}

//*****************************************************************************
// Servo - set PWM period and duty for the SG90 servo (50 Hz frame)
//*****************************************************************************
void Servo(uint16_t counts){
  TA3CCR0 = 60000 - 1;              // 20ms period @ 3 MHz SMCLK/4
  TA3CCR3 = counts;                // Duty count for channel 3
  TA3CTL  = TASSEL_2 | ID_2 | MC_1;// SMCLK/4, up mode
}

//*****************************************************************************
// distanceInCm - trigger HC-SR04 and return round trip distance in cm
//                Returns 400 if no echo received ("out of range")
//*****************************************************************************
uint16_t distanceInCm(void){
  P6->OUT |= TRIGGER;              // 10us trigger pulse
  Clock_Delay1us(10);
  P6->OUT &= ~TRIGGER;
  uint32_t t = pulseIn();          // Echo time, 1.5us per count
  // Speed of sound = 0.034 cm/us, divide by 2 for round trip
  return t ? (uint16_t)(0.034f / 2 * t) : 400;
}

//*****************************************************************************
// pulseIn - measure duration of the echo pin high pulse using Timer A2
//*****************************************************************************
uint32_t pulseIn(void){
  const uint16_t MAXCNT = 56999;   // About 85ms (timeout cap)
  TA2CTL = TASSEL_2 | ID_3 | MC_2; // SMCLK/8 continuous mode
  // Wait for rising edge (echo goes high)
  while(!(P6->IN & ECHO)) if(TA2R > MAXCNT){ TA2CTL = MC_0; return 0; }
  TA2R = 0;                        // Reset counter at rising edge
  // Wait for falling edge (echo low)
  while((P6->IN & ECHO)) if(TA2R > MAXCNT){ TA2CTL = MC_0; return 0; }
  TA2CTL = MC_0;                   // Stop timer
  return TA2R / 1.5;               // Convert to microseconds (approx)
}

//*****************************************************************************
// BLE global and stub
//*****************************************************************************
uint8_t BLE_Cmd;                   // Holds last command from phone
void WriteByteCmd(void){}          // Called on BLE attribute write

//*****************************************************************************
// Helper functions for A* open/closed management
//*****************************************************************************
static uint16_t heuristic(Point a, Point b){
  return (uint16_t)(abs(a.x - b.x) + abs(a.y - b.y)); // Manhattan distance
}
static inline bool inClosed(uint8_t c[GRID_ROWS][GRID_COLS], Point p){
  return c[p.y][p.x];
}
static bool inOpen(Point p){
  uint16_t i;
  for(i = 0; i < openCount; i++)
    if(openSet[i].x == p.x && openSet[i].y == p.y) return true;
  return false;
}
static void addOpen(Point p){
  if(openCount < GRID_ROWS * GRID_COLS && !inOpen(p))
    openSet[openCount++] = p;
}
static int lowestF(void){
  uint16_t bestF = 0xFFFF, bestH = 0xFFFF;
  int bestIdx = -1;
  uint16_t i;
  for(i = 0; i < openCount; i++){
    uint16_t f = grid[openSet[i].y][openSet[i].x].f;
    uint16_t h = grid[openSet[i].y][openSet[i].x].h;
    if(f < bestF || (f == bestF && h < bestH)){
      bestF = f; bestH = h; bestIdx = i;
    }
  }
  return bestIdx;                  // Index of cell with lowest f-cost
}

//*****************************************************************************
// Astar_Path - compute optimal path from s to g, store in out[], length in olen
//              Returns true if path found, false otherwise
//*****************************************************************************
static bool Astar_Path(Point s, Point g, Point *out, uint16_t *olen)
{
  static uint8_t closed[GRID_ROWS][GRID_COLS];     // Closed set flags
  int r, c;
  // Reset arrays
  for(r = 0; r < GRID_ROWS; r++)
    for(c = 0; c < GRID_COLS; c++){
      closed[r][c]      = 0;
      grid[r][c].g      = 0xFFFF;
      grid[r][c].h      = 0xFFFF;
      grid[r][c].f      = 0xFFFF;
      grid[r][c].parent_x = -1;
    }
  // Start node
  grid[s.y][s.x].g = 0;
  grid[s.y][s.x].h = heuristic(s, g);
  grid[s.y][s.x].f = grid[s.y][s.x].h;
  openCount = 0;
  addOpen(s);

  // Main loop
  while(openCount){
    int best = lowestF();
    if(best < 0) break;                       // Safety guard

    Point cur = openSet[best];                // Pop current node
    openSet[best] = openSet[--openCount];

    if(cur.x == g.x && cur.y == g.y){         // Goal reached - rebuild path
      uint16_t n = 0; Point p = g;
      while(!(p.x == s.x && p.y == s.y)){
        out[n++] = p;                         // Store backwards
        int8_t px = grid[p.y][p.x].parent_x;
        int8_t py = grid[p.y][p.x].parent_y;
        p.x = px; p.y = py;
      }
      out[n++] = s;                           // Add start at the end
      // Reverse order so start -> goal
      uint16_t i;
      for(i = 0; i < n/2; i++){
        Point t = out[i]; out[i] = out[n-1-i]; out[n-1-i] = t;
      }
      *olen = n;
      return true;
    }

    closed[cur.y][cur.x] = 1;                 // Add to closed set

    // Neighbor deltas
    static const int8_t dx[4] = {0, 1, 0, -1};
    static const int8_t dy[4] = {-1, 0, 1, 0};

    uint8_t k;
    for(k = 0; k < 4; k++){
      Point nb = { cur.x + dx[k], cur.y + dy[k] };

      // Boundary check
      if(nb.x < 0 || nb.x >= GRID_COLS || nb.y < 0 || nb.y >= GRID_ROWS) continue;
      // Wall check
      if(!grid[nb.y][nb.x].walkable) continue;
      // In closed check
      if(inClosed(closed, nb)) continue;

      uint16_t gTent = grid[cur.y][cur.x].g + 1;   // Cost through current
      if(gTent < grid[nb.y][nb.x].g){              // Better path found
        grid[nb.y][nb.x].parent_x = cur.x;
        grid[nb.y][nb.x].parent_y = cur.y;
        grid[nb.y][nb.x].g = gTent;
        grid[nb.y][nb.x].h = heuristic(nb, g);
        grid[nb.y][nb.x].f = grid[nb.y][nb.x].g + grid[nb.y][nb.x].h;
        addOpen(nb);                               // Add to open if not there
      }
    }
  }
  return false;                                    // No path
}

//*****************************************************************************
// Robot_Explore - depth-first search mapping routine controlled by a simple
//                 finite-state machine. Marks each cell walkable/unwalkable
//                 and fills the global grid[] and visited[] arrays.
//*****************************************************************************
static void Robot_Explore(void)
{
  const uint16_t WALL_CM = 15;       // Threshold below which wall is detected
  typedef enum {FORWARD, BACKWARD, SCAN_RIGHT, SCAN_LEFT, TURN_RIGHT, TURN_LEFT, DONE} ExploreState;

  Point stack[GRID_ROWS * GRID_COLS]; // DFS stack (stores path)

  ExploreState state = FORWARD, prevState = DONE;
  uint16_t stateTimer = 0;            // Counts 20ms ticks inside each state
  bool     isNewState = false;
  uint16_t right_cm = 0, left_cm = 0, front_cm = 0; // Recent sensor readings
  int8_t   sp = 0;                   // Stack pointer

  // Initialize pose and data structures
  curX = curY = 0; curHeading = 1;   // Start at (0,0), facing east
  visited[0][0] = true;
  stack[sp++] = (Point){0,0};

  ServoInit();                       // Center sensor
  SSD1306_ClearBuffer();
  SSD1306_DrawString(0, 0, "Exploring", WHITE);
  DrawCell(0, 0, 1);
  SSD1306_DisplayBuffer();

  // Main state machine loop
  while(state != DONE)
  {
    isNewState = (state != prevState);
    prevState = state;

    switch(state)
    {
    //-------------------------------------------------------------------
    case FORWARD:                    // Drive one cell forward
      if(isNewState){
        front_cm = 0;
        ServoInit();                 // Keep sensor centered while driving
        stateTimer = 0;
        Motor_Forward(7500, 8000);   // PWM = 7500 on both wheels
      }
      if(stateTimer >= 25){          // 25 * 20ms = 0.5s (one cell)
        Motor_Stop();
        front_cm = distanceInCm();   // Measure distance ahead

        int8_t nx = curX + dX[curHeading];
        int8_t ny = curY + dY[curHeading];

        // Check if next cell is in bounds, walkable, and not visited
        if(nx < 0 || nx >= GRID_COLS || ny < 0 || ny >= GRID_ROWS || !grid[ny][nx].walkable || visited[ny][nx]){
          state = SCAN_RIGHT;        // Cannot move forward - scan instead
          break;
        }

        // Move pose forward
        curX = nx; curY = ny;
        visited[ny][nx] = true;
        DrawCell(nx, ny, 1);
        stack[sp++] = (Point){nx, ny};

        // Goal condition - hardcoded (4,4)
        if(curX == 4 && curY == 4){
          state = DONE;
        } else {
          state = SCAN_RIGHT;        // Scan surroundings at new cell
        }
      }
      break;
    //-------------------------------------------------------------------
    case SCAN_RIGHT:                 // Look to the right 90 degrees
      if(isNewState){
        right_cm = 0; stateTimer = 0;
        Servo(SERVO_RIGHT);          // Rotate sensor to right
      }
      if(stateTimer >= 50){          // Wait 1s for servo to settle
        right_cm = distanceInCm();
        state    = SCAN_LEFT;        // Now scan to left side
      }
      break;
    //-------------------------------------------------------------------
    case SCAN_LEFT:                  // Look to the left 90 degrees
      if(isNewState){
        left_cm = 0; stateTimer = 0;
        Servo(SERVO_LEFT);
      }
      if(stateTimer >= 50){
        left_cm = distanceInCm();
        Servo(SERVO_CENTER);         // Return sensor to center

        // Compute indexes of cells around current position
        int8_t fx = curX + dX[curHeading];          int8_t fy = curY + dY[curHeading];
        int8_t lx = curX + dX[leftOf(curHeading)];  int8_t ly = curY + dY[leftOf(curHeading)];
        int8_t rx = curX + dX[rightOf(curHeading)]; int8_t ry = curY + dY[rightOf(curHeading)];

        // Update walkable flag for left/right/front neighbor cells that are not visited yet
        if(lx >= 0 && lx < GRID_COLS && ly >= 0 && ly < GRID_ROWS && !visited[ly][lx]){
          grid[ly][lx].walkable = (left_cm  >= WALL_CM);
          DrawCell(lx, ly, grid[ly][lx].walkable ? 0 : 2);
        }
        if(rx >= 0 && rx < GRID_COLS && ry >= 0 && ry < GRID_ROWS && !visited[ry][rx]){
          grid[ry][rx].walkable = (right_cm >= WALL_CM);
          DrawCell(rx, ry, grid[ry][rx].walkable ? 0 : 2);
        }
        if(fx >= 0 && fx < GRID_COLS && fy >= 0 && fy < GRID_ROWS && !visited[fy][fx]){
          grid[fy][fx].walkable = (front_cm >= WALL_CM);
          DrawCell(fx, fy, grid[fy][fx].walkable ? 0 : 2);
        }

        // Decide next action based on available neighboring cells
        bool canForward = (fx >= 0 && fx < GRID_COLS && fy >= 0 && fy < GRID_ROWS && grid[fy][fx].walkable && !visited[fy][fx]);
        bool canRight   = (rx >= 0 && rx < GRID_COLS && ry >= 0 && ry < GRID_ROWS && grid[ry][rx].walkable && !visited[ry][rx]);
        bool canLeft    = (lx >= 0 && lx < GRID_COLS && ly >= 0 && ly < GRID_ROWS && grid[ly][lx].walkable && !visited[ly][lx]);

        if(canForward){ state = FORWARD; }
        else if(canRight){ state = TURN_RIGHT; }
        else if(canLeft){ state = TURN_LEFT; }
        else {
          // Dead end - mark current cell blocked and backtrack
          grid[curY][curX].walkable = 0;
          DrawCell(curX, curY, 2);
          state = BACKWARD;
        }
      }
      break;
    //-------------------------------------------------------------------
    case BACKWARD:                   // Reverse back to previous cell
      if(isNewState){
        stateTimer = 0;
        Motor_Backward(7500, 7500);
      }
      if(stateTimer >= 25){          // Backwards one cell (0.5s)
        Motor_Stop();

        sp--;                        // Pop current from stack
        if(sp == 0){ state = DONE; break; }
        Point prev = stack[sp-1];    // Move pose to previous cell
        curX = prev.x; curY = prev.y;

        // Turn around 180 degrees
        Motor_Right(0, 7500);
        Clock_Delay1ms(1750);        // About 1.75s for 180 degree turn
        Motor_Stop();
        curHeading = (curHeading + 2) & 3;

        // After backing up, scan front cell again
        Servo(SERVO_CENTER);
        Clock_Delay1ms(40);
        front_cm = distanceInCm();
        int8_t fx = curX + dX[curHeading];
        int8_t fy = curY + dY[curHeading];
        if(fx >= 0 && fx < GRID_COLS && fy >= 0 && fy < GRID_ROWS && !visited[fy][fx]){
          grid[fy][fx].walkable = (front_cm >= WALL_CM);
          DrawCell(fx, fy, grid[fy][fx].walkable ? 0 : 2);
        }
        state = SCAN_RIGHT;
      }
      break;
    //-------------------------------------------------------------------
    case TURN_RIGHT:                 // Rotate 90 degrees right on spot
      if(isNewState){ stateTimer = 0; Motor_Right(0, 7500); }
      if(stateTimer >= 45){          // 35 * 20ms = 0.7s (90 deg)
        Motor_Stop();
        curHeading = rightOf(curHeading);
        state = FORWARD;
      }
      break;
    //-------------------------------------------------------------------
    case TURN_LEFT:                  // Rotate 90 degrees left on spot
      if(isNewState){ stateTimer = 0; Motor_Left(7500, 0); }
      if(stateTimer >= 45){
        Motor_Stop();
        curHeading = leftOf(curHeading);
        state = FORWARD;
      }
      break;
    //-------------------------------------------------------------------
    case DONE:                       // Exit point
    default:
      break;
    }

    Clock_Delay1ms(20);              // 20ms tick
    stateTimer++;                    // Increment state timer
    SSD1306_DisplayBuffer();         // Refresh OLED after each tick
  }

  Servo(SERVO_CENTER);               // Center sensor before finish
  Motor_Stop();
}

//*****************************************************************************
// Robot_FollowPath - drive the robot along the path produced by A*, using the
//                    same PWM and timing constants as in exploration.
//*****************************************************************************
static void Robot_FollowPath(Point *path, uint16_t len)
{
  if(len < 2){                       // Already at goal
    SSD1306_ClearBuffer();
    SSD1306_DrawString(0, 0, "Arrived", WHITE);
    SSD1306_DisplayBuffer();
    return;
  }

  // Draw static map background
  SSD1306_ClearBuffer();
  int8_t y, x;
  for(y = 0; y < GRID_ROWS; y++)
    for(x = 0; x < GRID_COLS; x++)
      DrawCell(x, y, grid[y][x].walkable ? 0 : 2);
  DrawCell(curX, curY, 1);
  SSD1306_DisplayBuffer();

  uint8_t h = curHeading;           // Local heading variable

  uint16_t i;
  for(i = 1; i < len; i++)
  {
    // Compute heading needed to move from path[i-1] to path[i]
    int8_t dx = path[i].x - path[i-1].x;
    int8_t dy = path[i].y - path[i-1].y;
    uint8_t tgt = (dy == 1) ? 2 : (dy == -1) ? 0 : (dx == 1) ? 1 : 3;
    uint8_t diff = (tgt - h + 4) & 3;      // Number of quarter turns

    // Rotate to target heading using same timing as explore
    if(diff == 1){                       // 90 deg right
      Motor_Right(0, 7500);
      Clock_Delay1ms(700);
    }
    else if(diff == 3){                  // 90 deg left
      Motor_Left(7500, 0);
      Clock_Delay1ms(700);
    }
    else if(diff == 2){                  // 180 deg
      Motor_Right(0, 7500);
      Clock_Delay1ms(1400);
    }
    Motor_Stop();
    h = tgt;                            // Heading updated

    // Drive forward one cell (0.5s)
    Motor_Forward(7500, 8000);
    Clock_Delay1ms(500);
    Motor_Stop();

    // Update pose and OLED
    curX = path[i].x; curY = path[i].y;
    DrawCell(curX, curY, 1);
    SSD1306_DisplayBuffer();
    Clock_Delay1ms(80);                 // Small delay for visibility
  }

  curHeading = h;                       // Save final heading
  SSD1306_ClearBuffer();
  SSD1306_DrawString(0, 0, "Arrived", WHITE);
  SSD1306_DisplayBuffer();
}

//*****************************************************************************
// main - program entry point. Initializes hardware and waits for BLE commands.
//*****************************************************************************
int main(void)
{
  DisableInterrupts();
  Clock_Init48MHz();        // Run core at 48 MHz
  UART0_Init();             // USB debug UART

  // Initialize GPIO and timers for motors, sensors, and servo
  Port1_Init();
  Port2_Init();
  Port3_Init();
  Port5_Init();
  Port6_Init();
  Port9_Init();
  TimerA0_Init();
  TimerA2_Init();
  TimerA3_Init();

  ServoInit();              // Center sensor servo
  GridInit();               // Reset map
  Motor_Stop();
  SSD1306_Init(SSD1306_SWITCHCAPVCC); // Init OLED display

  EnableInterrupts();
  UART0_OutString("\n\rRSLK A* Robot\n\r");

  // Initialize BLE service and characteristic
  AP_Init();
  AP_AddService(0xFFF0);
  BLE_Cmd = 0;
  AP_AddCharacteristic(0xFFF1, 1, &BLE_Cmd, 0x02, 0x08, "Command", 0, &WriteByteCmd);
  AP_RegisterService();
  AP_StartAdvertisementJacki();

  // Main loop: wait for bluetooth commands
  while(1){
    AP_BackgroundProcess();
    switch(BLE_Cmd){
    //--------------------------------------------------------------
    case CMD_EXPLORE:
      // Reset map display to unknown
      SSD1306_ClearBuffer();
      int8_t y, x;
      for(y = 0; y < GRID_ROWS; y++)
        for(x = 0; x < GRID_COLS; x++){
          grid[x][y].walkable = 1;      // Assume walkable pending test
          DrawCell(x, y, 0);
        }
      SSD1306_DisplayBuffer();

      Robot_Explore();                  // Run DFS exploration
      BLE_Cmd = 0;                      // Clear command flag
      break;
    //--------------------------------------------------------------
    case CMD_ROUTE:
      // Mark all unexplored cells as blocked before routing
      SSD1306_ClearBuffer();
      for(y = 0; y < GRID_ROWS; y++)
        for(x = 0; x < GRID_COLS; x++){
          if(!visited[x][y]){
            grid[x][y].walkable = 0;
            DrawCell(x, y, 2);
          }
        }
      SSD1306_DisplayBuffer();

      // Run A* Pathfinder
      {
        Point start = {0, 0};
        Point goal  = {GRID_COLS - 1, GRID_ROWS - 1};
        Point path[OPEN_MAX];
        uint16_t len = 0;
        if(Astar_Path(start, goal, path, &len)){
          SSD1306_ClearBuffer();
          SSD1306_DrawString(0, 0, "Routing", WHITE);
          SSD1306_DisplayBuffer();
          Robot_FollowPath(path, len);  // Drive along computed path
        } else {
          SSD1306_ClearBuffer();
          SSD1306_DrawString(0, 0, "No Path!", WHITE);
          SSD1306_DisplayBuffer();
        }
      }
      BLE_Cmd = 0;
      break;
    //--------------------------------------------------------------
    default:
      Clock_Delay1ms(50);              // Idle delay
    }
  }
}
