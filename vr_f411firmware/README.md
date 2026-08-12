# vr_f411firmware

STM32F411CEU6 CubeMX/HAL firmware for the jjkVR ICM-20948 pose path.

## Wiring

| Signal | STM32 pin | Notes |
|---|---:|---|
| ICM-20948 breakout VCC | 3.3 V | Only for a module rated for 3.3-V VIN and logic |
| ICM-20948 GND | GND | Shared ground |
| ICM-20948 SDA | PB7 | Shared I2C1 bus |
| ICM-20948 SCL | PB6 | Shared I2C1 bus |
| ICM-20948 AD0, if exposed | GND or 3.3 V | Firmware probes `0x68` and `0x69` |
| ICM-20948 NCS, if exposed | 3.3 V | Hold high for I2C unless the breakout already does so |
| ICM-20948 INT/FSYNC | Not connected | Firmware polls every 5 ms |
| TF-Luna pin 1 (+5V) | 5 V | Supply must be 3.7-5.2 V; do not use 3.3 V |
| TF-Luna pin 2 (RX/SDA) | Not connected | Receive-only UART needs no STM32 TX wire |
| TF-Luna pin 3 (TX) | PA3, GPIO header pin 5 | 115200-baud continuous data stream |
| TF-Luna pin 4 (GND) | GND | Shared ground |
| TF-Luna pin 5 (mode) | 3.3 V, GPIO header pin 3 | Selects UART mode at power-up; this is not a GPIO signal |
| LED_R / LED_G / LED_B | PA0 / PA1 / PB0 | Fault / IMU / TF-Luna |

I2C1 is now dedicated to the ICM-20948 and its bypassed AK09916 compass. The
TF-Luna uses its default 115200-baud UART stream on the single spare PA3 input,
so the two modules can no longer interfere with each other on I2C. A bare
ICM-20948 is not a 3.3-V supply part;
use the regulator/level shifting on the specified GY-ICM20948V2 breakout.
The TF-Luna's 5-V rail must support its documented 150 mA peak current.

On the jjkVR PCB GPIO header, pins `1` and `10` are GND, pin `2` is +5 V, and
pin `3` is 3.3 V, and pin `5` is PA3. Power-cycle after rewiring so TF-Luna
mode pin 5 samples 3.3 V and selects UART. Leave TF-Luna pins 2 and 6
disconnected.

## Mounting and axes

The ICM-20948 axes are assumed to match the headset body: `+X` forward toward
the monitor/display, `+Y` left, and `+Z` up. Mount the TF-Luna beam parallel to
body `+X`.

OpenVR uses `+X` right, `+Y` up, and `-Z` forward. Firmware converts body pose
to OpenVR before transmission: the tracked position is `(0, 0, -body_x)` and
the quaternion is `(w,-body_y,body_z,-body_x)`. The PC driver does not remap it
again.

## Fusion and startup

The firmware uses a small HAL-native ICM-20948 reader and float Mahony filter.
FastIMU remains a register/reference source: its ICM-20948 path is Arduino C++
and does not itself produce a fused quaternion, so importing it would add an
Arduino compatibility layer to this CubeMX C project.

Keep the mounted headset still, level, and with body `+Z` up at every power-up.
Firmware averages 200 accel/gyro samples over about one second, then treats the
first fused orientation and position as the origin. Calibration is RAM-only.
After the headset is stationary for about 150 ms, the fusion code quickly learns
only residual gyro rates below 0.25 degrees per second; faster motion is never
used for bias learning.
The magnetometer trim constants near the top of `Core/Src/icm20948.c` must be
measured with the final display electronics installed.

The LEDs are a direct connection check: solid blue means TF-Luna only, solid
green means IMU only, and green+blue (cyan) means both sensors are connected.
Red means neither sensor could be read or the active IMU transaction failed.
TF-Luna-only mode monitors its checksummed stream continuously and does not
publish a pose because no IMU orientation is available.

## TF-Luna correction

`ENABLE_LIDAR_FORWARD_FUSION` in `Core/Src/main.c` defaults to `1`. Its beam
must be rigidly aligned to body `+X` and aimed at a stable surface. Set the
switch to `0` when building without a TF-Luna.

The TF-Luna streams checksummed measurements over receive-only UART. Firmware
keeps the latest valid frame and applies it to fusion at 20 Hz. The IMU remains
independent on I2C even if the TF-Luna is absent or disconnected.

After the IMU declares the headset stationary, firmware averages eight valid
range readings (about 0.4 seconds) to establish the startup baseline. Movement
before all eight readings are collected restarts that average.

The IMU remains the fast predictor. Each accepted 20 Hz LiDAR reading keeps 90%
of the predicted forward position and applies 10% of the absolute range error;
`LIDAR_CORRECTION_WEIGHT` is the mounted-hardware tuning knob. Corrections stay
active at rest so accumulated forward drift converges back to the measured
position. They never change orientation or the other two position axes.

A 9-axis IMU has nine sensor channels, not nine pose degrees of freedom.
Translation is intentionally limited to body `+X`: the IMU predicts forward
motion and the one TF-Luna beam corrects it. Lateral and vertical position stay
zero instead of exposing unobservable drift. Stable room-scale 6-DoF still
needs an external positional reference.

`ENABLE_IMU_DEMO_POSITION` defaults to `1` because this build is intended to
exercise that requested movement path. Set it to `0` to keep SteamVR
orientation-only; the firmware then clears the packet's position-valid flag.

## USB report

The device is a bus-powered 64-byte vendor HID on PA11/PA12 with VID:PID
`0483:572B` and product name `jjkVR IMU Pose`. It does not expose a COM port.

| Offset | Content |
|---:|---|
| 0 | Report ID `1` |
| 1-16 | Little-endian float32 quaternion `(w,x,y,z)` in OpenVR coordinates |
| 17-28 | Little-endian float32 position `(x,y,z)` in metres, OpenVR coordinates |
| 29 | Flags: bit 0 position valid; bit 1 TF-Luna correction accepted |
| 30 | Protocol version `2` |
| 31 | Sensor status: `2` IMU init, `3` calibration, `4` first read, `5` streaming, `6` runtime IMU fault, `7` LiDAR bus fault, `8` TF-Luna-only test |
| 32-63 | Zero padding |

Bytes 0-16 retain compatibility with the previous orientation-only report.

## Build and test

Open this folder in VS Code, select the `Debug` CMake preset, and run
`CMake: Build`. Flash `build/Debug/vr_f411firmware.elf` with ST-LINK or
STM32CubeProgrammer.

1. Hold the headset still and level, body `+Z` up, until the LED is green (IMU)
   or cyan (IMU plus TF-Luna). If TF-Luna stays absent, verify pin 3 is connected
   to PA3/header pin 5, its mode pin is tied to 3.3 V, and then power-cycle.
2. Verify yaw, pitch, and roll directions before testing translation.
3. Make short, deliberate forward/back movements and verify the matching VR
   direction; lateral and vertical translation intentionally remain fixed.
4. With the TF-Luna rigidly mounted, test forward/back motion against a fixed
   opaque target at least 20 cm away and verify drift settles when stationary.

The host math check is `tests/pose_fusion_test.c`. From a Visual Studio
Developer PowerShell, run:

```powershell
New-Item -ItemType Directory -Force build\host-test | Out-Null
cl /nologo /std:c11 /I Core\Inc Core\Src\pose_fusion.c tests\pose_fusion_test.c /Fo:build\host-test\ /Fe:build\host-test\pose_fusion_test.exe
.\build\host-test\pose_fusion_test.exe
cl /nologo /std:c11 /I Core\Inc Core\Src\tfluna_uart.c tests\tfluna_uart_test.c /Fo:build\host-test\ /Fe:build\host-test\tfluna_uart_test.exe
.\build\host-test\tfluna_uart_test.exe
```

Driver build and SteamVR checks are in
`../docs/steamvr_driver_build_plan.md`.
