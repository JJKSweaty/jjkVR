# jjkVR

A custom, wired PC VR headset — built from a bare IMU chip up through SteamVR integration.

`Status: In progress (hardware, firmware, and driver bring-up)`

## What it is

jjkVR is a DIY PC VR headset that pairs a custom embedded tracking board with a high-refresh display path and full SteamVR/OpenVR support. Rather than wrapping an existing headset or dev kit, it implements the entire pose-tracking chain from scratch: a 9-DoF IMU feeds an STM32F411 microcontroller, which runs sensor fusion and streams orientation data to the PC over USB HID, where a custom OpenVR driver hands it to SteamVR. The display side takes GPU output over HDMI/DisplayPort and converts it to drive a 2K 120Hz panel.

It's inspired by (and builds on ideas from) [Relativty](https://github.com/relativty/Relativty) and [HadesVR](https://github.com/UkonnRa/HadesVR), two other open-source HMD projects.

## Why

Commercial VR headsets are closed systems — the tracking hardware, firmware, and drivers are black boxes. jjkVR exists to make every layer of that stack inspectable and hackable: the PCB is your own design, the firmware is yours to modify, and the SteamVR driver is yours to extend. It's a project for understanding *how* headset tracking actually works, not just using one, and for having a hardware/software platform you can freely tinker with, calibrate, and rebuild.

## How it works

```text
[ICM20948 IMU] --I2C--> [STM32F411 MCU Board] --USB HID--> [PC: SteamVR/OpenVR Driver]

[PC GPU] --HDMI/DP--> [Display Driver Board] --> [2K 120Hz HMD Panel]
```

- The **ICM20948** IMU reports raw accelerometer, gyroscope, and magnetometer data over I2C.
- The **STM32F411** reads that data, calibrates it, and runs FastIMU sensor fusion to compute headset orientation.
- Orientation is packed into **USB HID reports** and streamed to the PC at a steady rate.
- A **custom OpenVR driver** (adapted from Relativty's) parses those HID reports and feeds pose data into SteamVR.
- Separately, the PC's GPU output is converted by a **display driver board** to drive the **2K 120Hz HMD panel**.

The first version targets reliable wired **3-DoF orientation tracking**; the architecture leaves room for IR-based 6-DoF positional tracking and wireless streaming later on.

## Features

- Custom HMD controller PCB (not an off-the-shelf dev board)
- 9-DoF IMU sensor fusion via FastIMU
- USB HID pose streaming with a defined custom report format
- SteamVR/OpenVR integration through a Relativty-derived driver
- 2K 120Hz display target with a dedicated HDMI/DP-to-MIPI driver board
- On-device calibration storage, so the headset doesn't need to recalibrate every boot
- Status LEDs for calibration, IMU fault, and tracking state

## Hardware

| Component | Purpose |
|---|---|
| STM32F411 | Main MCU — IMU reads, sensor fusion, calibration, USB HID |
| ICM20948 | 9-DoF IMU (accelerometer, gyroscope, magnetometer) |
| Custom PCB | Integrates MCU, IMU interface, USB-C, regulator, LEDs, test points |
| 2K 120Hz display | Main HMD panel |
| HDMI/DP-to-MIPI driver board | Converts PC GPU output for the headset panel |
| USB-C | Wired power and USB HID data link |

The PCB itself is designed in KiCad and includes USB-C with proper CC resistors, a protected 3.3V rail, IMU power filtering, short/clean I2C routing with pullups, test points (SDA, SCL, 3V3, GND, USB), a status LED, and an expansion header for future accessories.

## Tech stack

- **Firmware:** C/C++, STM32F411, STM32duino, FastIMU
- **Sensors:** ICM20948, I2C, 9-DoF sensor fusion
- **USB:** USB HID pose reports
- **PCB:** KiCad, USB-C, 3.3V regulation
- **PC software:** SteamVR, OpenVR, Relativty driver architecture
- **Display:** 2K 120Hz panel with HDMI/DP-to-MIPI conversion

## Development rules

See `AGENTS.md` for repo-local agent instructions. In short: keep firmware minimal, comment hardware-critical assumptions, and update the relevant README whenever wiring, build, test, or hardware assumptions change.

## Acknowledgments

- [Relativty](https://github.com/relativty/Relativty) — open-source VR headset and OpenVR driver reference
- [HadesVR](https://github.com/UkonnRa/HadesVR) — open-source HMD and tracking reference
- [FastIMU](https://github.com/LiquidCGS/FastIMU) — IMU sensor fusion library
- SteamVR / OpenVR — driver architecture jjkVR integrates with

