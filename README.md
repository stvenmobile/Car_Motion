# Car_Motion: STM32 Mecanum Wheels Robot Controller

## Overview
This project contains the **STM32CubeIDE** firmware for a **Mecanum Wheels Robot Controller**.  
The firmware runs on a **YB-ERF01-V3.0 board from Yahboom** and communicates with a **Raspberry Pi running ROS2 Jazzy** via **serial UART**.
The **companion ROS2 repository** is: [mecca_ws](https://github.com/stvenmobile/mecca_ws)

The Car_Motion firmware:
- ✅ Receives velocity commands (`V X Y Z`) from the Raspberry Pi  
- ✅ Implements **PID control** to maintain precise motor speeds  
- ✅ Supports **quadrature encoders** for feedback  
- ✅ Uses **PWM for motor speed control**  
- ✅ Handles **serial communication & failsafe detection**  
- ✅ Supports **ICM-20948 IMU & Orientation Tracking**


---

## Hardware Requirements

| #  | Component                           | Description |
|----|-------------------------------------|------------|
| 1  | **STM32 MCU (YB-ERF01-V3.0)**      | Main motor controller |
| 1  | **Raspberry Pi 5 (ROS2 Jazzy)**    | Higher-level processing |
| 4  | **JGB37-520 12V Motors w/ Encoders** | Mecanum drive motors |
| 4  | **96mm Mecanum Wheels**            | High-clearance wheels 301.6mm circumference |
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

## Velocity Scaling & Calibration
The firmware is calibrated for 96mm diameter wheels and a 1:20~1:30 gear ratio, resulting in a 1:1 mapping between 
commanded velocity and real-world $mm/s$ when the PID is settled.Encoder Resolution: 440 pulses per wheel revolution. 
Stiction Compensation: The firmware implements a Dead-Zone Jump (1850 PWM) to overcome initial motor friction, ensuring 
movement starts at low commanded velocities ($V\ 50$).
Control Range:
V 50 → Minimum stable crawl ($50\ mm/s$).
V 1000 → Standard cruise speed ($1\ m/s$).
V 2000 → Maximum physical speed/saturation limit.

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

## **ICM-20948 IMU & Orientation Tracking**

The firmware now includes a comprehensive driver for the **ICM-20948 9-axis MotionTracking device**. Unlike standard implementations, this controller uses **Dynamic Delta-Time ($dt$) Integration** to maintain high-accuracy orientation tracking regardless of system load or loop jitter.

### **The Timing-Loop Approach**
* **Heartbeat Synchronization**: The controller measures the exact elapsed time between sensor polls using `HAL_GetTick()`, calculating a unique $dt$ for every iteration.
* **Integrated Yaw**: Angular velocity from the Z-axis gyroscope is integrated using the real-time $dt$, resulting in a stable Yaw (heading) value with approximately 98% rotational accuracy.
* **Hardware Scale Alignment**: The IMU is hardware-configured to **±2000 dps** and **±2g** to match the firmware's mathematical scaling constants (`16.4 LSB/dps` and `16384 LSB/g`).
* **I2C Master Proxy**: The firmware configures the ICM-20948 as an I2C Master to communicate with the internal **AK09916 magnetometer**, allowing absolute orientation data to be read over the primary SPI bus.



### **New Telemetry & Diagnostic Commands**

| Command | Description | Example Output |
| :--- | :--- | :--- |
| `I IMU` | Requests current Accelerometer, Gyroscope, and integrated Yaw readings. | `IMU: A(0.00, 0.00, -1.01) G(0.01, 0.02, 0.00) Yaw: 90.45` |
| `I YAW0` | Resets the current integrated heading/bearing to `0.00`. | `Heading Reset to 0.00` |
| `I TIME` | Diagnostic command reporting the actual duration of the last background loop. | `Internal Heartbeat: 11.00 ms (Target: 10.00ms)` |

---

## **Current Status & To-Do**

### **✅ Completed**
✔ **Quadrature encoder support**  
✔ **PWM-based speed control** 
✔ **Sensor Fusion Groundwork**: Successful integration of 6-axis IMU and 3-axis Magnetometer via SPI.
✔ **Precise Orientation**: Implemented time-invariant Yaw integration to prevent drift during high CPU load.
✔ **Hardware Scaling**: Synchronized silicon configuration with software constants for accurate physical mapping.
✔ Serial communication with Raspberry Pi using ROS2 Serial Bridge 
✔ Dead-Zone Compensation: Implemented an 1850 PWM floor to neutralize heavy-chassis stiction.
✔ Fine-tuning PID gains for smoother acceleration 
✔ Rotation Tuning: Adjusted robot_APB to 500 to compensate for Mecanum scrub friction during $V_z$ maneuvers.


### 🚧 In Progress
🛠 Implementing better error handling for serial input  
🛠 **Rviz Visualization**: Linking STM32 Odometry (Encoders + IMU) to ROS2 Jazzy for real-time obstacle mapping.
🛠 **ROS2 Integration:** Developing the mecca_ws serial bridge to map Twist messages to the calibrated $V\ X\ Y\ Z$ range.


### 🔜 Future Improvements
🚀 Add **sensor fusion** (combine encoders + IMU for better localization)  
🚀 Improve **failsafe handling** (auto-stop on serial disconnection)  
🚀 Support **additional sensors** for navigation  

---

## License
This project is licensed under the **MIT License**. See `LICENSE` for details.

---







