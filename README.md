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

```
V <linear_x> <linear_y> <angular_z>
```

- `linear_x` → Forward/Backward motion (mm/s)
- `linear_y` → Left/Right strafing motion (mm/s)
- `angular_z` → Rotation (turning left/right)

### Command Reference

| Command          | Description                                          | Example Output |
|------------------|------------------------------------------------------|----------------|
| `V 500 0 0`      | Move forward at 500 mm/s                             | *(none)*       |
| `V -500 0 0`     | Move backward at 500 mm/s                            | *(none)*       |
| `V 0 500 0`      | Strafe right at 500 mm/s                             | *(none)*       |
| `V 0 0 500`      | Rotate clockwise                                     | *(none)*       |
| `V 0 0 0`        | Stop (applies electronic brake)                      | *(none)*       |
| `I ENC`          | Request current encoder counts                       | `I ENC 1234 -56 789 -12` |
| `I RESET`        | Reset all encoder counts to zero                     | `ACK: Encoders Reset to Zero` |
| `I IMU`          | Request accelerometer, gyroscope, and yaw data       | `IMU: A(0.00, 0.00, -1.01) G(0.01, 0.02, 0.00) Yaw: 90.45` |
| `I TEST`         | Non-blocking forward drive test at 500 mm/s          | `NON-BLOCKING TEST START: 500 mm/s` |
| `I DIAG`         | Sequential single-motor diagnostic (motors 1–4)      | `Testing M1 (LF)... DONE` |
| `P <kp> <ki> <kd>` | Live-update PID gains for all motors               | `ACK: PID updated - KP:1.50 KI:0.50 KD:0.05` |

> **Note:** The safety watchdog stops motors automatically if no `V` command is received within `CMD_WATCHDOG_TIMEOUT_MS` (default: **1000 ms**). When testing from a serial monitor, resend `V` commands within 1 second or increase this constant in `bsp_motion.h`.

---

## Velocity Scaling & Calibration

The firmware is calibrated for **96mm diameter wheels** with a **1:55 gear ratio**, giving a 1:1 mapping between commanded velocity and real-world mm/s when the PID is settled.

| Parameter | Value | Notes |
|---|---|---|
| Wheel circumference | 301.6 mm | π × 96mm |
| Encoder resolution | **2420 ticks/rev** | 55 (gear) × 11 (PPR) × 4 (TI12 quadrature) |
| Dead-zone floor | **1250 PWM** | Applied with a 200 ms soft-start ramp |
| PWM timer period | 3600 counts (20 kHz) | TIM1 and TIM8 |
| Watchdog timeout | 1000 ms | Configurable via `CMD_WATCHDOG_TIMEOUT_MS` |

**Control range:**

| Command | Speed |
|---|---|
| `V 50 0 0` | Minimum stable crawl (~50 mm/s) |
| `V 1000 0 0` | Standard cruise speed (~1 m/s) |
| `V 2000 0 0` | Approaching physical saturation |

### Dead-Zone Soft-Start
On startup, the dead-zone compensation floor is **linearly ramped** from 0 to 1250 PWM over 20 PID cycles (~200 ms). This prevents the large simultaneous inrush current that all four motors starting at full dead-zone floor would otherwise cause, which was previously responsible for USB-serial brownouts.

### Speed Measurement
Motor speed is calculated using a **measured inter-call dt** (`HAL_GetTick()` delta) rather than a hardcoded 100 Hz assumption. Because the main loop includes ICM-20948 SPI reads and other overhead, the actual loop period is ~12–15 ms; the dynamic dt ensures speed measurements remain accurate regardless of loop jitter.

---

## Build & Flash Instructions

### 1. Clone Repository
```
git clone https://github.com/stvenmobile/Car_Motion.git
```

### 2. Open in STM32CubeIDE
1. Open **STM32CubeIDE**
2. Click **File → New → STM32 Project from an Existing Configuration File (.ioc)**
3. Select the **Car_Motion.ioc** file

### 3. Build & Compile
- Click **Project → Build Project** (`Ctrl + B`)
- Ensure compilation succeeds without errors

Alternatively, if you do not need to make any code changes, flash the STM32 using the pre-built `Car_Motion.hex` file in the `Debug/` directory.

### 4. Flash to STM32
Use the **FlyMCU** utility to flash the `.hex` binary to the STM32.
The utility is available for download here: http://www.mcuisp.com/software/FlyMcu.rar

---

## Directory Structure
```
Car_Motion
├───.settings
├───BSP
|    ├───bsp.c / bsp.h          (top-level init & main loop)
|    ├───bsp_encoder.c / .h     (quadrature encoder reading, TIM2–5)
|    ├───bsp_motion.c / .h      (kinematics, speed calculation, watchdog)
|    ├───bsp_motor.c / .h       (PWM output, dead-zone compensation)
|    ├───bsp_pid.c / .h         (positional PID controller)
|    ├───bsp_uart.c / .h        (serial command parser, printf redirect)
|    ├───bsp_beep.c / .h
|    ├───bsp_key.c / .h
|    ├───bsp_icm20948.c / .h    (ICM-20948 IMU driver)
|    └───bsp_rgb.c / .h
├───Core
│   ├───Inc
│   ├───Src                     (main.c, tim.c, usart.c, etc.)
│   └───Startup
├───Debug                       (pre-built .elf / .hex / .list)
└───Drivers
    ├───CMSIS
    └───STM32F1xx_HAL_Driver
```

---

## ICM-20948 IMU & Orientation Tracking

The firmware includes a driver for the **ICM-20948 9-axis MotionTracking device** using dynamic delta-time integration for accurate orientation regardless of loop jitter.

* **Heartbeat Synchronization**: Exact elapsed time between sensor polls is measured via `HAL_GetTick()`, providing a unique dt for every iteration.
* **Integrated Yaw**: Z-axis gyroscope data is integrated with the real-time dt, yielding a stable heading value (~98% rotational accuracy).
* **Hardware Scale Alignment**: Configured to ±2000 dps and ±2g matching firmware scaling constants (`16.4 LSB/dps`, `16384 LSB/g`).
* **I2C Master Proxy**: The ICM-20948 is configured as I2C Master to read the internal AK09916 magnetometer over the primary SPI bus.

---

## Current Status & To-Do

### ✅ Completed
- ✔ **Quadrature encoder support**
- ✔ **PWM-based speed control**
- ✔ **Sensor Fusion Groundwork**: 6-axis IMU and 3-axis Magnetometer via SPI
- ✔ **Precise Orientation**: Time-invariant Yaw integration to prevent drift
- ✔ **Hardware Scaling**: Synchronized silicon configuration with software constants
- ✔ **Serial communication** with Raspberry Pi using ROS2 Serial Bridge
- ✔ **Dead-Zone Compensation**: 1250 PWM floor with 200 ms soft-start ramp to prevent inrush brownouts
- ✔ **Fine-tuning PID gains** for smoother acceleration
- ✔ **ISR Safety Fix**: Serial command processing deferred out of UART interrupt to main loop, eliminating potential MCU deadlock
- ✔ **Serial Stability**: Removed automatic 10 Hz PID telemetry flood; output is now on-demand only
- ✔ **Accurate Speed Measurement**: Speed calculation uses measured inter-call dt instead of hardcoded 100 Hz
- ✔ **PID Range Fix**: Pre-clamp on raw PID output ensures full control range maps correctly through dead-zone addition

### 🚧 In Progress
- 🛠 **Rviz Visualization**: Linking STM32 Odometry (Encoders + IMU) to ROS2 Jazzy for real-time obstacle mapping
- 🛠 **ROS2 Integration**: Developing the mecca_ws serial bridge to map Twist messages to the calibrated V X Y Z range

### 🔜 Future Improvements
- 🚀 Add **sensor fusion** (combine encoders + IMU for better localization)
- 🚀 Support **additional sensors** for navigation

---

## License
This project is licensed under the **MIT License**. See `LICENSE` for details.
