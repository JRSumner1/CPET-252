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

#define TRIGGER        0x04
#define ECHO           0x08
#define SERVO_CENTER   3900
#define SERVO_LEFT     1500
#define SERVO_RIGHT    9500
#define CMD_EXPLORE    0x01
#define CMD_ROUTE      0x02
#define GRID_ROWS      5
#define GRID_COLS      5
#define CELL_SIZE_CM   20
#define CELL_MS        (CELL_SIZE_CM * 50)
#define TURN90_MS      350

static const int8_t dX[4] = { 0, 1, 0, -1 };
static const int8_t dY[4] = {-1, 0, 1,  0 };
static inline uint8_t leftOf (uint8_t h){ return (h + 3) & 3; }
static inline uint8_t rightOf(uint8_t h){ return (h + 1) & 3; }

#define MAP_ORIGIN_X   0
#define MAP_ORIGIN_Y   16
#define CELL_PIX       10

static void DrawCell(int8_t gx, int8_t gy, uint8_t state) {
  uint8_t x0 = MAP_ORIGIN_X + gx * CELL_PIX;
  uint8_t y0 = MAP_ORIGIN_Y + gy * CELL_PIX;
  uint8_t i;
  for (i = 0; i < CELL_PIX; i++) {
    SSD1306_DrawPixel(x0 + i, y0, WHITE);
    SSD1306_DrawPixel(x0 + i, y0 + CELL_PIX - 1, WHITE);
    SSD1306_DrawPixel(x0, y0 + i, WHITE);
    SSD1306_DrawPixel(x0 + CELL_PIX - 1, y0 + i, WHITE);
  }
  if (state == 0) SSD1306_DrawString(x0 + 1, y0 + 1, "?", WHITE);
  else if (state == 1) SSD1306_DrawString(x0 + 2, y0 + 1, "O", WHITE);
  else SSD1306_DrawString(x0 + 2, y0 + 1, "X", WHITE);
}

typedef struct { int8_t x; int8_t y; } Point;
typedef struct { uint8_t walkable; int8_t parent_x; int8_t parent_y; uint16_t g,h,f; } Node_t;

#define OPEN_MAX (GRID_ROWS * GRID_COLS)
static Point openSet[OPEN_MAX];
static uint16_t openCount;
static Node_t grid[GRID_ROWS][GRID_COLS];

static int8_t curX = 0, curY = 0;
static uint8_t curHeading = 1;

void GridInit(void){
  int r; int c;
  for(r=0;r<GRID_ROWS;r++)
    for(c=0;c<GRID_COLS;c++)
      grid[r][c].walkable=1;
}

void Servo(uint16_t counts);
uint32_t pulseIn(void);

void ServoInit(void){
  Servo(SERVO_CENTER);
  Clock_Delay1ms(500);
  TA3CTL &= ~0x0030;
}
void Servo(uint16_t counts){
  TA3CCR0 = 60000-1;
  TA3CCR3 = counts;
  TA3CTL  = TASSEL_2|ID_2|MC_1;
}
uint16_t distanceInCm(void){
  P6->OUT |= TRIGGER;
  Clock_Delay1us(10);
  P6->OUT &= ~TRIGGER;
  uint32_t t = pulseIn();
  return t ? (uint16_t)(0.034f/2 * t) : 400;
}
uint32_t pulseIn(void){
  const uint16_t MAXCNT=56999;
  TA2CTL = TASSEL_2|ID_3|MC_2;
  while(!(P6->IN&ECHO)) if(TA2R>MAXCNT){ TA2CTL=MC_0; return 0; }
  TA2R=0;
  while( (P6->IN&ECHO)) if(TA2R>MAXCNT){ TA2CTL=MC_0; return 0; }
  TA2CTL=MC_0;
  return TA2R/1.5;
}

uint8_t BLE_Cmd;
void WriteByteCmd(void){}

static bool Astar_Path(Point s, Point g, Point * out, uint16_t * olen) {
  static uint8_t closed[GRID_ROWS][GRID_COLS];
  int r;
  int c;
  for (r = 0; r < GRID_ROWS; r++)
    for (c = 0; c < GRID_COLS; c++) {
      closed[r][c] = 0;
      grid[r][c].g = grid[r][c].h = grid[r][c].f = 0xFFFF;
      grid[r][c].parent_x = -1;
    }
  grid[s.y][s.x].g = 0;
  grid[s.y][s.x].h = abs(s.x - g.x) + abs(s.y - g.y);
  grid[s.y][s.x].f = grid[s.y][s.x].h;
  openCount = 0;
  openSet[openCount++] = s;

  while (openCount) {
    int best = 0;
    uint16_t bestf = grid[openSet[0].y][openSet[0].x].f, besth = grid[openSet[0].y][openSet[0].x].h;
    int i;
    for (i = 1; i < openCount; i++) {
      uint16_t f = grid[openSet[i].y][openSet[i].x].f,
        h = grid[openSet[i].y][openSet[i].x].h;
      if (f < bestf || (f == bestf && h < besth)) {
        best = i;
        bestf = f;
        besth = h;
      }
    }
    Point cur = openSet[best];
    openSet[best] = openSet[--openCount];
    if (cur.x == g.x && cur.y == g.y) {
      uint16_t n = 0;
      Point p = g;
      while (!(p.x == s.x && p.y == s.y)) {
        out[n++] = p;
        int8_t px = grid[p.y][p.x].parent_x, py = grid[p.y][p.x].parent_y;
        p.x = px;
        p.y = py;
      }
      out[n++] = s;
      int i;
      for (i = 0; i < n / 2; i++) {
        Point t = out[i];
        out[i] = out[n - 1 - i];
        out[n - 1 - i] = t;
      }
      * olen = n;
      return true;
    }
    closed[cur.y][cur.x] = 1;
    const int8_t dx[4]={0,1,0,-1}, dy[4]={-1,0,1,0};
    int k;
    for (k = 0; k < 4; k++) {
      Point nb = {
        cur.x + dx[k],
        cur.y + dy[k]
      };
      if (nb.x < 0 || nb.x >= GRID_COLS || nb.y < 0 || nb.y >= GRID_ROWS) continue;
      if (!grid[nb.y][nb.x].walkable || closed[nb.y][nb.x]) continue;
      uint16_t tent = grid[cur.y][cur.x].g + 1;
      if (tent < grid[nb.y][nb.x].g) {
        grid[nb.y][nb.x].parent_x = cur.x;
        grid[nb.y][nb.x].parent_y = cur.y;
        grid[nb.y][nb.x].g = tent;
        grid[nb.y][nb.x].h = abs(nb.x - g.x) + abs(nb.y - g.y);
        grid[nb.y][nb.x].f = grid[nb.y][nb.x].g + grid[nb.y][nb.x].h;
        bool in = false;
        int i;
        for (i = 0; i < openCount; i++)
          if (openSet[i].x == nb.x && openSet[i].y == nb.y) {
            in = true;
            break;
          }
        if (!in && openCount < OPEN_MAX) openSet[openCount++] = nb;
      }
    }
  }
  return false;
}

#define TRY(HVAL)\
do {
  \
  uint8_t h2 = (uint8_t)(HVAL);\
  int8_t nx = curX + dX[h2], ny = curY + dY[h2];\
  if (nx >= 0 && nx < GRID_COLS && ny >= 0 && ny < GRID_ROWS\ &&
    grid[ny][nx].walkable && !visited[ny][nx]) {
    \
    uint8_t diff = (h2 - curHeading + 4) & 3;\
    if (diff == 1) {
      Motor_Right(0, 7500);
      Clock_Delay1ms(TURN90_MS);
    }\
    if (diff == 3) {
      Motor_Left(7500, 0);
      Clock_Delay1ms(TURN90_MS);
    }\
    if (diff == 2) {
      Motor_Right(0, 7500);
      Clock_Delay1ms(TURN90_MS * 2);
    }\
    Motor_Stop();
    curHeading = h2;\
    Motor_Forward(7500, 7500);
    Clock_Delay1ms(CELL_MS);
    Motor_Stop();\
    curX = nx;
    curY = ny;
    visited[ny][nx] = true;
    DrawCell(nx, ny, 1);\
    stack[sp++] = (Point) {
      nx,
      ny
    };\
    moved = true;\
  }\
} while (0)

static void Robot_Explore(void) {
  const uint16_t WALL_CM = 12;
  static bool visited[GRID_ROWS][GRID_COLS] = {0};
  Point stack[GRID_ROWS * GRID_COLS];
   
  int8_t sp = 0;
  curX = curY = 0;
  curHeading = 1;
  visited[0][0] = true;
  stack[sp++] = (Point) {0, 0};

  ServoInit();
  SSD1306_ClearBuffer();
  SSD1306_DrawString(0, 0, "Exploring", WHITE);
  DrawCell(0, 0, 1);
  SSD1306_DisplayBuffer();

  while (sp) {
    Servo(SERVO_RIGHT);
    Clock_Delay1ms(40);
    uint16_t right_cm = distanceInCm();
    Servo(SERVO_LEFT);
    Clock_Delay1ms(40);
    uint16_t left_cm = distanceInCm();
    Servo(SERVO_CENTER);
    Clock_Delay1ms(40);
    uint16_t front_cm = distanceInCm();
    int8_t fx = curX + dX[curHeading], fy = curY + dY[curHeading];
    int8_t lx = curX + dX[leftOf(curHeading)], ly = curY + dY[leftOf(curHeading)];
    int8_t rx = curX + dX[rightOf(curHeading)], ry = curY + dY[rightOf(curHeading)];
    if (lx >= 0 && lx < GRID_COLS && ly >= 0 && ly < GRID_ROWS) {
      grid[ly][lx].walkable = (left_cm >= WALL_CM);
      DrawCell(lx, ly, grid[ly][lx].walkable ? (visited[ly][lx] ? 1 : 0) : 2);
    }
    if (rx >= 0 && rx < GRID_COLS && ry >= 0 && ry < GRID_ROWS) {
      grid[ry][rx].walkable = (right_cm >= WALL_CM);
      DrawCell(rx, ry, grid[ry][rx].walkable ? (visited[ry][rx] ? 1 : 0) : 2);
    }
    if (fx >= 0 && fx < GRID_COLS && fy >= 0 && fy < GRID_ROWS) {
      grid[fy][fx].walkable = (front_cm >= WALL_CM);
      DrawCell(fx, fy, grid[fy][fx].walkable ? (visited[fy][fx] ? 1 : 0) : 2);
    }

    bool moved = false;
    TRY(curHeading);
    if (!moved) TRY(leftOf(curHeading));
    if (!moved) TRY(rightOf(curHeading));
    if (!moved) {
      grid[curY][curX].walkable = 0;
      DrawCell(curX, curY, 2);
      sp--;
      if (sp == 0) break;
      Point prev = stack[sp - 1];
      int8_t dx = prev.x - curX, dy = prev.y - curY;
      uint8_t backH = (dx == 1) ? 1 : (dx == -1) ? 3 : (dy == 1) ? 2 : 0;
      uint8_t diff = (backH - curHeading + 4) & 3;
      if (diff == 1) {
        Motor_Right(0, 7500);
        Clock_Delay1ms(TURN90_MS);
      }
      if (diff == 3) {
        Motor_Left(7500, 0);
        Clock_Delay1ms(TURN90_MS);
      }
      if (diff == 2) {
        Motor_Right(0, 7500);
        Clock_Delay1ms(TURN90_MS * 2);
      }
      Motor_Stop();
      curHeading = backH;
      Motor_Backward(7500, 7500);
      Clock_Delay1ms(CELL_MS);
      Motor_Stop();
      curX = prev.x;
      curY = prev.y;
    }
    SSD1306_DisplayBuffer();
    if (curX == 4 && curY == 4) break;
  }
  Servo(SERVO_CENTER);
  Motor_Stop();
  SSD1306_ClearBuffer();
  SSD1306_DrawString(0, 0, "Explore Complete", WHITE);
  SSD1306_DisplayBuffer();
}
#undef TRY

static void Robot_FollowPath(Point * path, uint16_t len) {
  uint8_t h = curHeading;
  uint16_t i;
  for (i = 1; i < len; i++) {
    int8_t dx = path[i].x - path[i - 1].x, dy = path[i].y - path[i - 1].y;
    uint8_t tgt = (dy == 1) ? 2 : (dy == -1) ? 0 : (dx == 1) ? 1 : 3;
    uint8_t diff = (tgt - h + 4) & 3;
    if (diff == 1) {
      Motor_Right(0, 7500);
      Clock_Delay1ms(TURN90_MS);
    }
    if (diff == 3) {
      Motor_Left(7500, 0);
      Clock_Delay1ms(TURN90_MS);
    }
    if (diff == 2) {
      Motor_Right(0, 7500);
      Clock_Delay1ms(TURN90_MS * 2);
    }
    Motor_Stop();
    h = tgt;
    Motor_Forward(7500, 7500);
    Clock_Delay1ms(CELL_MS);
    Motor_Stop();
    curX = path[i].x;
    curY = path[i].y;
    Clock_Delay1ms(80);
  }
  curHeading = h;
  SSD1306_ClearBuffer();
  SSD1306_DrawString(0, 0, "Arrived", WHITE);
  SSD1306_DisplayBuffer();
}

int main(void) {
  DisableInterrupts();
  Clock_Init48MHz();
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
  GridInit();
  Motor_Stop();
  SSD1306_Init(SSD1306_SWITCHCAPVCC);
  SSD1306_ClearBuffer();
  int8_t y;
  int8_t x;
  for (y = 0; y < GRID_ROWS; y++)
    for (x = 0; x < GRID_COLS; x++)
      DrawCell(x, y, 0);
  SSD1306_DisplayBuffer();
  EnableInterrupts();
  UART0_OutString("\n\rRSLK A* Robot\n\r");
  AP_Init();
  AP_AddService(0xFFF0);
  BLE_Cmd = 0;
  AP_AddCharacteristic(0xFFF1, 1, & BLE_Cmd, 0x02, 0x08, "Command", 0, & WriteByteCmd);
  AP_RegisterService();
  AP_StartAdvertisementJacki();

  while (1) {
    AP_BackgroundProcess();
    switch (BLE_Cmd) {
    case CMD_EXPLORE:
      Robot_Explore();
      BLE_Cmd = 0;
      break;
    case CMD_ROUTE: {
      Point start = {curX, curY}, goal = {GRID_COLS - 1, GRID_ROWS - 1};
      Point path[OPEN_MAX];
      uint16_t len = 0;
      if (Astar_Path(start, goal, path, & len)) {
        SSD1306_ClearBuffer();
        SSD1306_DrawString(0, 0, "Routing", WHITE);
        SSD1306_DisplayBuffer();
        Robot_FollowPath(path, len);
      } else {
        SSD1306_ClearBuffer();
        SSD1306_DrawString(0, 0, "No Path!", WHITE);
        SSD1306_DisplayBuffer();
      }
      BLE_Cmd = 0;
      break;
    }
    default:
      Clock_Delay1ms(50);
    }
  }
}