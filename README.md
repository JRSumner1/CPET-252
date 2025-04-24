# CPET-252 — Microcontrollers Lab (Spring 2025)

A curated collection of **MSP432 / TI-RSLK** labs, exam prep, and a capstone path-finding robot project completed for the Rochester Institute of Technology course **CPET-252 Microcontrollers Lab**.  

---

## Repository layout

| Path | What it contains |
|------|------------------|
| `Lab 1` … `Lab 8` | Eight progressive labs that build from GPIO to autonomous navigation (details below). |
| `Lab7_OLED` / `Lab7_BLE` | Split starter projects used in Lab 7 for display and BLE exploration. |
| `CompExam` | Review code/snippets used while studying for the comprehensive exam. |
| `Final Project` | A* path-finding robot (“Explore” & “Route” modes) with Bluetooth control, OLED map visualization, and sensor-driven autonomy. |
| `InputOutput` | CCS template illustrating basic GPIO I/O, used early in the course. |
| `inc` | Re-usable driver sources (`Clock.c`, `Motor.c`, `Reflectance.c`, etc.). |
| `.gitignore`, `index.html`, etc. | House-keeping, website landing page for GitHub Pages. |

---

## Quick start

### Prerequisites

* **Hardware:**  
  * TI-RSLK development kit with MSP432P401R LaunchPad  
  * HC-SR04 ultrasonic module, micro-servos, QTRX reflectance array, CC2650 BLE BoosterPack, SSD1306 OLED  
* **Software:**  
  * Code Composer Studio - Version 12.7.1
    * Used VS Code with the C/C++ Extension to write code
    * Used CCS to Build and Flash MSP432P401R

### Build & flash

```bash
# Example (Windows PowerShell)
git clone https://github.com/JRSumner1/CPET-252.git
# Open CCS ➜ Import Project… ➜ select the desired Lab_x folder
# Build   (hammer icon)
# Debug (green bug icon) – make sure the robot is off the floor before enabling motors!
