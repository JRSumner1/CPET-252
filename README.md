# CPET-252 — Microcontrollers Lab (Spring 2025)

Hands-on labs, exams, and a final A* pathfinder robot project for the **CPET-252 / CPET-253 Microcontrollers Lab** course at RIT.  
Everything targets the **TI-RSLK** platform (MSP432P401R LaunchPad) and is built with **TI Code Composer Studio (CCS V12.7.1)**.

---

## Table of Contents
1. [Hardware](#hardware)
2. [Software / Toolchain](#software--toolchain)
3. [Repo Layout](#repo-layout)
4. [Lab Summaries](#lab-summaries)
5. [Building & Flashing](#building--flashing)
6. [Final Project – A* Robot](#final-project--a-robot)
7. [Contributing](#contributing)

---

## Hardware
| Part | Purpose |
|------|---------|
| **MSP432P401R LaunchPad** | Core MCU |
| **TI-RSLK chassis** | Motors, bump sensors, battery, etc. |
| **HC-SR04 ultrasonic module** | Range finding (Lab 5) |
| **SG90 servo** | Sweeps ultrasonic sensor (Lab 4–5) |
| **CC2650 BLE BoosterPack** | Bluetooth Low-Energy link (Lab 7) |
| **SSD1306 128×64 OLED** | On-board UI (Lab 7) |
| **Pololu QTR-8RC (QTRX)** | 8-channel IR line sensor (Lab 8) |

---

## Software / Toolchain
* **Code Composer Studio V12.7.1** – IDE & compiler  
* **TI-RSLK MAX “inc” support library** – shared drivers  
* **CCS debug console** – UART viewer  
* **LightBlue (iOS/Android)** – BLE testing app for Lab 7
* *Optional: For writing programs and using GitHub, I used VS Code with the C/C++ Extension Pack and Github Pull Requests Extensions*

---

## Repo Layout
```bash
CPET-252/
│   .gitignore
│   index.html
│   README.md
│
├───CompExam
│   │   .ccsproject
│   │   .cproject
│   │   .project
│   │   main.c
│   │   msp432p401r.cmd
│   │   startup_msp432p401r_ccs.c
│   │   system_msp432p401r.c
│
├───FinalProject
│   │   .ccsproject
│   │   .cproject
│   │   .gitignore
│   │   .project
│   │   AP.c
│   │   Clock.c
│   │   CortexM.c
│   │   FinalProjectWriteup.docx
│   │   GPIO.c
│   │   Init_Ports.c
│   │   Init_Timers.c
│   │   LaunchPad.c
│   │   main.c
│   │   Motor.c
│   │   msp432p401r.cmd
│   │   signoff.docx
│   │   SSD1306.c
│   │   startup_msp432p401r_ccs.c
│   │   system_msp432p401r.c
│   │   UART0.c
│   │   UART1.c
│
├───inc
│   │   .ccsproject
│   │   .cproject
│   │   .project
│   │   ADC14.c
│   │   ADC14.h
│   │   ADCTA0Trigger.c
│   │   ADCTA0Trigger.h
│   │   AP.c
│   │   AP.h
│   │   blinker.c
│   │   blinker.h
│   │   bmi160.c
│   │   bmi160.h
│   │   bmi160_defs.h
│   │   bmm150.c
│   │   bmm150.h
│   │   bmm150_defs.h
│   │   BmpConvert.exe
│   │   BmpConvertReadme.txt
│   │   Bump.c
│   │   Bump.h
│   │   BumpInt.c
│   │   BumpInt.h
│   │   Clock.c
│   │   Clock.h
│   │   CortexM.c
│   │   CortexM.h
│   │   Enemy.bmp
│   │   Enemy.txt
│   │   EUSCIA0.c
│   │   EUSCIA0.h
│   │   FFT.c
│   │   FFT.h
│   │   fftdesign.xlsx
│   │   FIFO0.c
│   │   FIFO0.h
│   │   FIR_Digital_LowPassFilter.xls
│   │   fixed.c
│   │   fixed.h
│   │   fixedTrig.xlsx
│   │   FlashProgram.c
│   │   FlashProgram.h
│   │   Fuzzy.c
│   │   Fuzzy.h
│   │   GPIO.c
│   │   GPIO.h
│   │   HDC2080.c
│   │   HDC2080.h
│   │   I2CB1.c
│   │   I2CB1.h
│   │   IIR_Digital_LowPassFilter.xls
│   │   incmain.c
│   │   Init_Ports.c
│   │   Init_Ports.h
│   │   Init_Timers.c
│   │   Init_Timers.h
│   │   IRDistance.c
│   │   IRDistance.h
│   │   Lab2data.xlsx
│   │   LaunchPad.c
│   │   LaunchPad.h
│   │   LPF.c
│   │   LPF.h
│   │   Motor.c
│   │   Motor.h
│   │   MotorSimple.c
│   │   MotorSimple.h
│   │   msp432p401r.cmd
│   │   Nokia5110.c
│   │   Nokia5110.h
│   │   odometry.c
│   │   odometry.docx
│   │   odometry.h
│   │   odometry.xlsx
│   │   OPT3001.c
│   │   OPT3001.h
│   │   opt3101.c
│   │   opt3101.h
│   │   OPT3101speedChoices.xlsx
│   │   PWM.c
│   │   PWM.h
│   │   Reflectance.c
│   │   Reflectance.h
│   │   SSD1306.c
│   │   SSD1306.h
│   │   startup_msp432p401r_ccs.c
│   │   Switch.c
│   │   Switch.h
│   │   system_msp432p401r.c
│   │   SysTick.c
│   │   SysTick.h
│   │   SysTickInts.c
│   │   SysTickInts.h
│   │   TA0InputCapture.c
│   │   TA0InputCapture.h
│   │   TA2InputCapture.c
│   │   TA2InputCapture.h
│   │   TA3InputCapture.c
│   │   TA3InputCapture.h
│   │   Tachometer.c
│   │   Tachometer.h
│   │   TExaS.c
│   │   TExaS.h
│   │   ti.bmp
│   │   ti.txt
│   │   Timer32.c
│   │   Timer32.h
│   │   TimerA0.c
│   │   TimerA0.h
│   │   TimerA1.c
│   │   TimerA1.h
│   │   TimerA2.c
│   │   TimerA2.h
│   │   TMP117.c
│   │   TMP117.h
│   │   UART0.c
│   │   UART0.h
│   │   UART1.c
│   │   UART1.h
│   │   Ultrasound.c
│   │   Ultrasound.h
│   │   ValvanoDictionary
│
├───InputOutput
│   │   .ccsproject
│   │   .cproject
│   │   .project
│   │   InputOutput.c
│   │   msp432p401r.cmd
│   │   startup_msp432p401r_ccs.c
│   │   system_msp432p401r.c
│   │
│
├───Lab1
│   │   .ccsproject
│   │   .cproject
│   │   .project
│   │   Init_Ports.c
│   │   main.c
│   │   msp432p401r.cmd
│   │   startup_msp432p401r_ccs.c
│   │   system_msp432p401r.c
│
├───Lab2
│   │   .ccsproject
│   │   .cproject
│   │   .project
│   │   Clock.c
│   │   CortexM.c
│   │   Init_Ports.c
│   │   Lab2_main.c
│   │   msp432p401r.cmd
│   │   startup_msp432p401r_ccs.c
│   │   system_msp432p401r.c
│
├───Lab3
│   │   .ccsproject
│   │   .cproject
│   │   .project
│   │   Clock.c
│   │   CortexM.c
│   │   Init_Ports.c
│   │   Init_Timers.c
│   │   Lab3_main.c
│   │   Motor.c
│   │   msp432p401r.cmd
│   │   startup_msp432p401r_ccs.c
│   │   system_msp432p401r.c
│
├───Lab4
│   │   .ccsproject
│   │   .cproject
│   │   .project
│   │   Clock.c
│   │   CortexM.c
│   │   Init_Ports.c
│   │   Init_Timers.c
│   │   Lab4_main.c
│   │   Motor.c
│   │   msp432p401r.cmd
│   │   startup_msp432p401r_ccs.c
│   │   system_msp432p401r.c
│
├───Lab5
│   │   .ccsproject
│   │   .cproject
│   │   .gitignore
│   │   .project
│   │   Clock.c
│   │   CortexM.c
│   │   Init_Ports.c
│   │   Init_Timers.c
│   │   Lab5_main.c
│   │   Motor.c
│   │   msp432p401r.cmd
│   │   startup_msp432p401r_ccs.c
│   │   system_msp432p401r.c
│   │
│
├───Lab6
│   │   .ccsproject
│   │   .cproject
│   │   .project
│   │   BumpInt.c
│   │   Clock.c
│   │   CortexM.c
│   │   Init_Ports.c
│   │   Init_Timers.c
│   │   Lab6_main.c
│   │   Motor.c
│   │   msp432p401r.cmd
│   │   startup_msp432p401r_ccs.c
│   │   stateMachine.png
│   │   system_msp432p401r.c
│   │
│
├───Lab7_BLE
│   │   .ccsproject
│   │   .cproject
│   │   .gitignore
│   │   .project
│   │   AP.c
│   │   Clock.c
│   │   CortexM.c
│   │   GPIO.c
│   │   Init_Ports.c
│   │   Init_Timers.c
│   │   Lab7_BLEmain.c
│   │   LaunchPad.c
│   │   Motor.c
│   │   msp432p401r.cmd
│   │   SSD1306.c
│   │   startup_msp432p401r_ccs.c
│   │   system_msp432p401r.c
│   │   UART0.c
│   │   UART1.c
│   │   VerySimpleBLEmain.c
│
├───Lab7_OLED
│   │   .ccsproject
│   │   .cproject
│   │   .project
│   │   Clock.c
│   │   CortexM.c
│   │   Lab7_OLEDmain.c
│   │   msp432p401r.cmd
│   │   SSD1306.c
│   │   startup_msp432p401r_ccs.c
│   │   system_msp432p401r.c
│
├───Lab8
│   │   .ccsproject
│   │   .cproject
│   │   .project
│   │   Clock.c
│   │   CortexM.c
│   │   Init_Ports.c
│   │   Init_Timers.c
│   │   Lab8_main.c
│   │   Motor.c
│   │   msp432p401r.cmd
│   │   Reflectance.c
│   │   startup_msp432p401r_ccs.c
│   │   StateMachine.jpg
│   │   system_msp432p401r.c
│   │
```

---

## Lab Summaries
| Lab | Theme & Key Skills | Demo |
|-----|--------------------|------|
| **1 – CCS & GPIO** | Set up workspace; drive RGB LED with push-buttons. | LED toggles |
| **2 – Software State Machines** | Four-state LED pattern with 10 ms tick timer. | Color cycle |
| **3 – PWM & Motor Drivers** | TimerA0 PWM on P2.6/2.7; directional moves; square path. | Robot drives square |
| **4 – Servo Motors** | TimerA3 PWM; 180° sweep; state-timed motion. | Servo scans |
| **5 – Ultrasonic Sensing** | Echo-pulse timing; obstacle distance & avoidance. | Avoids walls |
| **6 – Bump Sensors & IRQs** | Port 4 falling-edge interrupts; collision count steering. | Bounces off wall |
| **7 – OLED + BLE** | SSD1306 SPI graphics; CC2650 BLE UART; phone control. | Phone-controlled |
| **8 – IR Line Tracking** | QTRX weighted integrate; line follow PID-style. | Line follower |

*(Each lab folder includes `main.c` and drivers.)*

---

## Building & Flashing
1. **Clone** the repo:  
  ```bash
  # Example (Windows PowerShell)
  git clone https://github.com/JRSumner1/CPET-252.git
```
2. Open CCS ➜ Import Project… ➜ select the desired lab folder
3. Build (hammer icon)
4. Debug (green bug icon) – make sure the robot is off the floor before enabling motors!

---

## Final Project – A\* Robot
The culmination of **Labs 4 – 8**, a robot that:
* Explores a defined grid
* Maps obstacles with its ultrasonic sensor
* Computes the optimal path with **A\*** search
* Drives the route while displaying it on the OLED

### BLE Commands
| Command | Mode            |
|---------|-----------------|
| `0x01`  | **Explore**     |
| `0x02`  | **Route** (start → goal) |

For full documentation and demo videos, visit **<https://jrsumner1.github.io/CPET-252/>**.

---

## Contributing
Jonathan Sumner – JRSumner1
