# Car_Motion: STM32 Mecanum Wheels Robot Controller

## Overview
This project contains the **STM32CubeIDE** firmware for a **Mecanum Wheels Robot Controller**.  
The firmware runs on a **YB-ERF01-V3.0 board from Yahboom** and communicates with a **Raspberry Pi running ROS2 Jazzy** via **serial UART**.

The Car_Motion firmware:
- ✅ Receives velocity commands (`V X Y Z`) from the Raspberry Pi  
- ✅ Implements **PID control** to maintain precise motor speeds  
- ✅ Supports **quadrature encoders** for feedback  
- ✅ Uses **PWM for motor speed control**  
- ✅ Handles **serial communication & failsafe detection**  

The **companion ROS2 repository** is: [mecca_ws](https://github.com/stvenmobile/mecca_ws)

---

## Hardware Requirements

| #  | Component                           | Description |
|----|-------------------------------------|------------|
| 1  | **STM32 MCU** (YB-ERF01-V3.0)      | Main motor controller |
| 1  | **Raspberry Pi 5 (ROS2 Jazzy)**    | Higher-level processing |
| 4  | **JGB37-520 12V Motors w/ Encoders** | Mecanum drive motors |
| 1  | **12V 3S LiPo Battery**             | Power supply |
| 1  | **VL53L1X ToF Sensor**              | Obstacle detection |

---

## Motor Control & Serial Communication

This firmware **receives motion commands** from the Raspberry Pi and translates them into **precise motor speeds**.

### Serial Command Format
The STM32 listens for **UART commands** in the following format:

V <linear_x> <linear_y> <angular_z>


- `linear_x` → Forward/Backward motion
- `linear_y` → Left/Right strafing motion
- `angular_z` → Rotation (turning left/right)

### Example Commands


| Command      | Description                      |
|--------------|----------------------------------|
| `V 500 0 0`  | Move forward at speed 500        |
| `V -500 0 0` | Move backward at speed 500       |
| `V 0 500 0`  | Strafe right at speed 500        |
| `V 0 0 500`  | Rotate clockwise at speed 500    |
| `I ENC`      | Request encoder values           |

---

## Build & Flash Instructions

### 1. Clone Repository
git clone https://github.com/stvenmobile/Car_Motion.git


### 2. Open in STM32CubeIDE
1. Open **STM32CubeIDE**
2. Click **File → New → STM32 Project from an Existing Configuration File (.ioc)
3. Select the **Car_Motion.ioc** file

### 3. Build & Compile
- Click **Project → Build Project** (`Ctrl + B`)
- Ensure compilation succeeds without errors

- Alternatively if you do not need to make any changes to the code you can flash the STM32 using the existing Car_Motion.hex file in the Debug directory.

### 4. Flash to STM32
Use the FlyMCU Utility to flash .hex binary file to STM32
The utility is available for download here: http://www.mcuisp.com/software/FlyMcu.rar

---

## Directory Structure
```
Car_Motion
├───.settings
├───BSP
|    ├───bsp.c
|    ├───bsp.h
|    ├───bsp_beep.c
|    ├───bsp_beep.h
|    ├───bsp_encoder.c
|    ├───bsp_encoder.h
|    ├───bsp_key.c
|    ├───bsp_key.h
|    ├───bsp_motion.c
|    ├───bsp_motion.h
|    ├───bsp_motor.c
|    ├───bsp_motor.h
|    ├───bsp_pid.c
|    ├───bsp_pid.h
|    ├───bsp_uart.c
|    ├───bsp_uart.h
├───Core
│   ├───Inc (main.c file is here)
│   ├───Src
│   └───Startup
├───Debug (Binary file to flash is located here)
└───Drivers
    ├───CMSIS
    └───STM32F1xx_HAL_Driver

```

---

## Current Status & To-Do

### ✅ Completed
✔ Serial communication with Raspberry Pi  
✔ Basic PID tuning for motor control  
✔ Quadrature encoder support  
✔ PWM-based speed control  

### 🚧 In Progress
🛠 Fine-tuning PID gains for smoother acceleration  
🛠 Implementing better error handling for serial input  

### 🔜 Future Improvements
🚀 Add **sensor fusion** (combine encoders + IMU for better localization)  
🚀 Improve **failsafe handling** (auto-stop on serial disconnection)  
🚀 Support **additional sensors** for navigation  

---

## License
This project is licensed under the **MIT License**. See `LICENSE` for details.

---







