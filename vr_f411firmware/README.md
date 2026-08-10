# vr_f411firmware

STM32F411CEU6 CubeMX/CMake firmware for the jjkVR tracking bring-up tests.

## Wiring

| Signal | STM32 pin | Notes |
|---|---:|---|
| TF-Luna pin 1 (+5V) | 5V | Sensor supply must be 3.7-5.2 V; do not use the 3.3 V rail |
| TF-Luna pin 2 (SDA) | PB7 | I2C1 SDA with pull-up to 3.3 V |
| TF-Luna pin 3 (SCL) | PB6 | I2C1 SCL with pull-up to 3.3 V |
| TF-Luna pin 4 (GND) | GND | Shared ground |
| TF-Luna pin 5 (mode) | GND | Required to select I2C mode at power-up |
| LED_R | PA0 | I2C read failed |
| LED_G | PA1 | Valid 20-800 cm reading with adequate signal |
| LED_B | PB0 | Sensor started, but reading is currently unreliable |

## VS Code Build

1. Open this `vr_f411firmware` folder in VS Code.
2. Install/enable the STM32 VS Code extension when prompted.
3. Select the `Debug` CMake preset.
4. Run `CMake: Build`.

Expected artifacts are under `build/Debug/`, including `vr_f411firmware.elf`.

## TF-Luna HID test

The PCB is USB bus-powered, so firmware disables PA9 VBUS sensing and reports
itself as bus-powered. HID uses PA11 (D-) and PA12 (D+).

`LIDAR_HID_TEST` in `Core/Src/main.c` maps 20-200 cm to -90 to +90 degrees
of yaw and sends the stock Relativty 64-byte HID quaternion report every 100 ms.
This intentionally makes TF-Luna range visibly rotate the SteamVR view; it is
not real positional tracking.

USB identity: VID `1155` (`0x0483`), PID `22315` (`0x572B`). Configure the
Relativty driver with `hmdIMUdmpPackets: false` and `startTrackingServer: false`.

This HID build does not create a COM port. Windows should list a vendor-defined
HID named `jjkVR Lidar HID Test`.

For the temporary test, close SteamVR Room Setup if it asks for base stations
and use **Display VR View**. Base stations are not part of this 3-DoF HID test.
The VR View is only a mirror; no mouse cursor or SteamVR controller is provided.

## Hardware Test

Flash the ELF with ST-LINK or STM32CubeProgrammer, then reset the board.

Point the TF-Luna at an opaque target at least 20 cm away. Green means a valid reading, blue means the sensor answered but the distance or signal is unreliable, and red means the sensor did not answer at I2C address `0x10`.

See `../docs/firmware_build_plan.md` for the SteamVR test and IMU integration plan.
