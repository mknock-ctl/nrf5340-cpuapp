# DotBot Navigation & IMU Control

- [DotBot Navigation & IMU Control](#dotbot-navigation--imu-control)
  - [Overview](#overview)
  - [Core Features](#core-features)
    - [Movement & Odometry](#movement--odometry)
    - [Magnetometer](#magnetometer)
    - [Safety & Interaction](#safety--interaction)
    - [Traction & Crash Detection](#traction--crash-detection)
  - [Hardware Resources](#hardware-resources)
    - [DotBot Platform](#dotbot-platform)
    - [IMU (9-Axis)](#imu-9-axis)
  - [Development Setup](#development-setup)
    - [Using VS Code](#using-vs-code)
    - [Building the Project](#building-the-project)
  - [Flashing the Hardware](#flashing-the-hardware)

---

## Overview

The DotBot is a swarm robotics platform developed at KU Leuven. Distinct from traditional lithium-ion powered robots, it utilizes **supercapacitors** for energy storage. This project provides a Zephyr-based firmware implementation for relative movement, sensor fusion, and safety monitoring using the onboard nRF5340 SoC and 9-axis IMU.

For more details on the hardware platform, refer to the original research:  
*M. Liu et al., “CapBot: Enabling Battery-Free Swarm Robotics”*

---

## Core Features

The system implements the API defined in [`ses_assignment.h`](app/src/ses_assignment.h) to handle complex path traversal and environmental awareness.

### Movement & Odometry
The firmware handles two primary motion primitives:
* **`move(int distance)`**: Implemented using **Hall effect sensors** on the wheels to track precise linear displacement.
* **`turn(int angle)`**: Implemented using **Gyroscope data** from the IMU to handle precise rotations.

### Magnetometer
The system utilizes the LIS3MDL magnetometer to detect **Magnetic North**. A specialized routine allows the robot to automatically orient itself toward the North pole before beginning a mission.

### Safety & Interaction
* **Double Tap Start**: To prevent accidental movement after boot, the robot remains in a "pre-routine" state until a **Double Tap** is detected via the IMU.
* **Voltage Monitoring**: While idle, the onboard LED displays a color gradient representing capacitor charge (Green = Max, Red = Minimum).

### Traction & Crash Detection
The firmware monitors IMU data in real-time to detect undesirable behavior:
* **Crash Detection**: Sudden deceleration triggers an emergency stop, a brief reverse maneuver, and a permanent Red LED state.
* **Motion Verification**: By comparing Accelerometer data against Wheel Hall sensors, the robot detects:
    * **Blue LED**: Downhill loss of traction (Physical velocity > Wheel speed).
    * **Yellow LED**: Uphill loss of traction (Physical velocity < Wheel speed).

---

## Hardware Resources

### DotBot Platform
* **SoC**: Nordic Semiconductor nRF5340 (Dual-core).
* **Architecture**: Based on the Zephyr RTOS.
* **Power**: Supercapacitor-based (Battery-free).
* [DotBot Schematic](doc/DotBot_V3-Main-bd1.3.pdf)

### IMU (9-Axis)
The hardware integrates two primary chips for motion sensing:
* **LSM6DSOX**: 3-axis Gyroscope and 3-axis Accelerometer.
* **LIS3MDL**: 3-axis Magnetometer.
* [LSM6DSOX Datasheet](./doc/lsm6dsox_datasheet.pdf) | [LIS3MDL Datasheet](./doc/lis3mdl_datasheet.pdf)

---

## Development Setup

### Using VS Code
1. Open the project folder through the **nRF Connect** extension.
2. Initialize and Update the **West workspace** when prompted. This downloads a sandboxed copy of the nRF Connect SDK and DotBot board definitions.

### Building the Project
1. Add a build configuration in the nRF Connect extension.
2. Set the Board Target to `mergebot/nrf5340/cpuapp/ns`.
3. Run **Generate and Build**.

---

## Flashing the Hardware

This project uses [pyOCD](https://pyocd.io) for firmware deployment via DAPLink.

1. **Install Requirements**:
   ```console
   $ pip install pyocd
   $ pyocd pack install nrf53
