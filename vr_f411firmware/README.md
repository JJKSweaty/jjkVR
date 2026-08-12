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
| ICM-20948 INT/FSYNC | Not connected | Firmware polls every 5 ms |
| TF-Luna pin 1 (+5V) | 5 V | Supply must be 3.7-5.2 V; do not use 3.3 V |
| TF-Luna pin 2 (SDA) | PB7 | Shared I2C1 bus |
| TF-Luna pin 3 (SCL) | PB6 | Shared I2C1 bus |
| TF-Luna pin 4 (GND) | GND | Shared ground |
| TF-Luna pin 5 (mode) | GND | Selects I2C mode at power-up |
| LED_R / LED_G / LED_B | PA0 / PA1 / PB0 | Fault / tracking / startup |

I2C1 runs at 400 kHz. Use one set of pull-ups to 3.3 V and never pull SDA or
SCL to the TF-Luna 5-V supply. TF-Luna is `0x10`; the bypassed AK09916 compass
inside the ICM-20948 is `0x0C`. A bare ICM-20948 is not a 3.3-V supply part;
use the regulator/level shifting on the specified GY-ICM20948V2 breakout.

## Mounting and axes

The ICM-20948 axes are assumed to match the headset body: `+X` forward toward
the monitor/display, `+Y` left, and `+Z` up. Mount the TF-Luna beam parallel to
body `+X`.

OpenVR uses `+X` right, `+Y` up, and `-Z` forward. Firmware converts body pose
to OpenVR before transmission: position `(-body_y, body_z, -body_x)` and
quaternion `(w,-body_y,body_z,-body_x)`. The PC driver does not remap it again.

## Fusion and startup

The firmware uses a small HAL-native ICM-20948 reader and float Mahony filter.
FastIMU remains a register/reference source: its ICM-20948 path is Arduino C++
and does not itself produce a fused quaternion, so importing it would add an
Arduino compatibility layer to this CubeMX C project.

Keep the mounted headset still, level, and with body `+Z` up at every power-up.
Firmware averages 200 accel/gyro samples over about one second, then treats the
first fused orientation and position as the origin. Calibration is RAM-only.
After the headset is stationary for about 150 ms, the fusion code slowly learns
only residual gyro rates below 0.25 degrees per second; faster motion is never
used for bias learning.
The magnetometer trim constants near the top of `Core/Src/icm20948.c` must be
measured with the final display electronics installed.

Blue during startup means calibration or retry. Steady blue afterward means
accel/gyro fusion is running without the AK09916 magnetometer. Red means the
main ICM-20948 initialization/calibration/read failed; green means all nine
sensor channels are available and fusion is running.

## TF-Luna correction

`ENABLE_LIDAR_FORWARD_FUSION` in `Core/Src/main.c` defaults to `1`. Its beam
must be rigidly aligned to body `+X` and aimed at a stable surface. Set the
switch to `0` when building without a TF-Luna.

After the IMU declares the headset stationary, firmware averages eight valid
range readings (about 0.4 seconds) to establish the startup baseline. Movement
before all eight readings are collected restarts that average.

The IMU remains the fast predictor. Each accepted 20 Hz LiDAR reading keeps 90%
of the predicted forward position and applies 10% of the absolute range error;
`LIDAR_CORRECTION_WEIGHT` is the mounted-hardware tuning knob. Corrections stay
active at rest so accumulated forward drift converges back to the measured
position. They never change orientation or the other two position axes.

A 9-axis IMU has nine sensor channels, not nine pose degrees of freedom.
Integrated acceleration here is bounded demonstration tracking and will drift;
one TF-Luna beam can constrain only forward distance. Stable room-scale 6-DoF
still needs an external positional reference.

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
| 31-63 | Zero padding |

Bytes 0-16 retain compatibility with the previous orientation-only report.

## Build and test

Open this folder in VS Code, select the `Debug` CMake preset, and run
`CMake: Build`. Flash `build/Debug/vr_f411firmware.elf` with ST-LINK or
STM32CubeProgrammer.

1. Hold the headset still and level, body `+Z` up, until the LED turns green.
2. Verify yaw, pitch, and roll directions before testing translation.
3. Make short, deliberate forward/left/up movements and verify the matching VR
   direction; expect inertial drift.
4. With the TF-Luna rigidly mounted, test forward/back motion against a fixed
   opaque target at least 20 cm away and verify drift settles when stationary.

The host math check is `tests/pose_fusion_test.c`. From a Visual Studio
Developer PowerShell, run:

```powershell
New-Item -ItemType Directory -Force build\host-test | Out-Null
cl /nologo /std:c11 /I Core\Inc Core\Src\pose_fusion.c tests\pose_fusion_test.c /Fo:build\host-test\ /Fe:build\host-test\pose_fusion_test.exe
.\build\host-test\pose_fusion_test.exe
```

Driver build and SteamVR checks are in
`../docs/steamvr_driver_build_plan.md`.
