# jjkVR

Custom 2K 120Hz VR headset with STM32F411 firmware, 9-DoF IMU tracking, USB HID pose streaming, custom PCB hardware, and SteamVR/OpenVR support.

> Status: In progress — hardware, firmware, and driver bring-up are currently being planned and prototyped.

## Overview

**jjkVR** is a custom open-source wired VR headset inspired by Relativty and HadesVR. The goal is to build a PC VR headset with a custom embedded tracking board, 9-DoF IMU sensor fusion, USB HID pose streaming, SteamVR/OpenVR integration, and a high-refresh display path.

The first version focuses on reliable wired **3-DoF orientation tracking** using an STM32F411 MCU and ICM20948 IMU. Future versions may add IR-based 6-DoF positional tracking and wireless support.

```text
[ICM20948 IMU] --I2C--> [STM32F411 MCU Board] --USB HID--> [PC: SteamVR/OpenVR Driver]

[PC GPU] --HDMI/DP--> [Display Driver Board] --> [2K 120Hz HMD Panel]
```

## Goals

* Build a custom 2K 120Hz wired VR headset
* Design a custom HMD controller PCB instead of relying on an off-the-shelf MCU board
* Use an STM32F411 MCU for native USB HID and floating-point sensor fusion
* Read 9-DoF motion data from an ICM20948 IMU over I2C
* Run FastIMU-based sensor fusion for headset orientation tracking
* Stream pose data to the PC using USB HID reports
* Integrate the headset pose stream with SteamVR/OpenVR
* Validate the full hardware/software path from IMU to SteamVR

## Planned Hardware

| Component                    | Purpose                                                                |
| ---------------------------- | ---------------------------------------------------------------------- |
| STM32F411                    | Main MCU for IMU reads, sensor fusion, calibration, and USB HID        |
| ICM20948                     | 9-DoF IMU with accelerometer, gyroscope, and magnetometer              |
| Custom PCB                   | Integrated MCU, IMU interface, USB-C, regulator, LEDs, and test points |
| 2K 120Hz display             | Main HMD display target                                                |
| HDMI/DP-to-MIPI driver board | Converts PC GPU output to the headset display panel                    |
| USB-C                        | Wired power and USB HID data link                                      |

## Firmware Plan

The jjkVR firmware will run on the STM32F411 and handle the motion-tracking chain:

* Initialize and read the ICM20948 over I2C
* Calibrate accelerometer, gyroscope, and magnetometer data
* Run FastIMU sensor fusion to produce headset orientation
* Format orientation data into USB HID reports
* Stream pose data to the PC at a consistent update rate
* Store calibration data to avoid recalibration on every boot
* Drive status LEDs for calibration, IMU fault, and tracking states

## Software / Driver Plan

The PC-side software will adapt the Relativty/OpenVR driver path for jjkVR:

* Match the custom USB VID/PID from the STM32F411 board
* Parse the custom HID report format
* Forward headset orientation data into SteamVR
* Configure display geometry, render resolution, IPD, and distortion settings
* Validate the display pipeline on a normal monitor before final HMD display bring-up

## PCB Design Plan

jjkVR uses a custom HMD controller PCB to replace the stock off-the-shelf microcontroller approach.

Planned PCB features:

* STM32F411 MCU
* ICM20948-compatible IMU interface
* USB-C connector with proper CC resistors
* Protected 3.3V power rail
* Filtering capacitors for cleaner IMU power
* I2C pullups and short IMU routing
* Test points for SDA, SCL, 3V3, GND, and USB
* Status LED for bring-up and calibration feedback
* Expansion header for future accessories

## Roadmap

### Phase 0 — Planning and Validation

* Lock final MCU, IMU, display, and driver-board choices
* Define the USB HID report format
* Set up STM32 firmware toolchain
* Verify FastIMU compatibility with STM32F411 and ICM20948
* Fork and inspect the Relativty OpenVR driver
* Start KiCad schematic for the custom PCB

### Phase 1 — Breadboard Prototype

* Connect STM32F411 dev board to ICM20948 over I2C
* Read raw accelerometer, gyroscope, and magnetometer data
* Run FastIMU calibration and sensor fusion
* Stream pose data over USB HID
* Validate HID parsing on the PC

### Phase 2 — Driver and Display Bring-Up

* Update OpenVR driver for custom HID reports
* Configure SteamVR render and display settings
* Test driver output on a spare monitor
* Validate display-driver board before final headset assembly

### Phase 3 — Custom PCB

* Finish schematic and PCB layout in KiCad
* Add USB-C, power regulation, IMU routing, LEDs, headers, and test points
* Order prototype PCB
* Bring up power, I2C, USB enumeration, and pose streaming

### Phase 4 — Integration

* Assemble the electronics, display, lenses, and headset shell
* Tune calibration and SteamVR settings
* Measure drift, latency, and tracking stability
* Document known issues and next hardware revision

## Future Work

* IR LED + webcam positional tracking for low-cost 6-DoF
* ESP32-based wireless pose streaming
* Battery-powered headset electronics
* Controller support using low-latency wireless modules
* Desktop calibration/configuration utility
* Improved display distortion and IPD tuning

## Tech Stack

* **Firmware:** C/C++, STM32F411, STM32duino, FastIMU
* **Sensors:** ICM20948, I2C, 9-DoF IMU fusion
* **USB:** USB HID pose reports
* **PCB:** KiCad, USB-C, 3.3V regulation, test points
* **PC Software:** SteamVR, OpenVR, Relativty driver
* **Display:** 2K 120Hz target panel with HDMI/DP-to-MIPI driver board

## Why jjkVR

Most DIY VR headset builds rely on off-the-shelf microcontroller boards and simple IMU wiring. jjkVR modernizes that approach with a custom embedded tracking board, stronger MCU, 9-DoF sensor fusion, USB HID pose streaming, and a cleaner split between firmware, PCB hardware, display bring-up, and PC driver integration.

The goal is not just to assemble a headset, but to build and validate the embedded systems stack behind it.

## References

* Relativty open-source VR headset
* HadesVR open-source HMD and tracking references
* FastIMU sensor library
* SteamVR / OpenVR driver architecture
