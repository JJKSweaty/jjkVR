# jjkVR Firmware Build and Test Plan

The earlier TF-Luna-to-synthetic-yaw test passed on hardware and proved the
full `sensor -> STM32 -> USB HID -> JJKVR driver -> SteamVR` path. It has now
been replaced by the live IMU pose implementation.

## Current implementation

- ICM-20948 accel/gyro at about 225 Hz and AK09916 magnetometer at 100 Hz on
  I2C1; firmware samples pose every 5 ms.
- About one second of level, stationary accel/gyro calibration at every boot.
- Float Mahony orientation fusion with acceleration and magnetic-disturbance
  rejection.
- Bounded acceleration integration for short forward/back demonstration
  movement, with stationary zero-velocity updates.
- 64-byte HID pose protocol v2 sent at up to 100 Hz.
- TF-Luna forward correction enabled by default for a rigid body-`+X` mount;
  builds without the sensor can disable it at compile time.

This is a CubeMX/HAL C project. FastIMU was not imported: its ICM-20948 driver
depends on Arduino C++, and it still needs a separate fusion implementation.
The local register driver and filter are the smaller fit for this firmware.

## Pose model

Headset body axes are right-handed: `+X` forward toward the monitor/display,
`+Y` left, and `+Z` up. Firmware converts to OpenVR `+X` right, `+Y` up,
`-Z` forward before sending the report.

The IMU supplies reliable orientation but not drift-free position. Translation
is deliberately limited to body `+X`, the only axis this TF-Luna arrangement
can correct. Lateral and vertical output stay zero instead of drifting. One
rigid TF-Luna can constrain forward position against a fixed plane; it cannot
observe a moving target or its own motion when loose. Stable room-scale 6-DoF
needs another positional reference.

## HID contract

USB is vendor HID, VID:PID `0483:572B`, report ID `1`, 64 input bytes:

| Offset | Content |
|---:|---|
| 0 | report ID `1` |
| 1-16 | float32 quaternion `(w,x,y,z)`, OpenVR coordinates |
| 17-28 | float32 position `(x,y,z)` metres, OpenVR coordinates |
| 29 | bit 0 position valid; bit 1 TF-Luna correction accepted |
| 30 | version `2` |
| 31-63 | zero padding |

The quaternion prefix remains compatible with the previous driver packet.
Firmware and the checked-in JJKVR driver must change together if these offsets
change.

## Build and hardware validation

1. Select the firmware `Debug` CMake preset, build, and flash
   `vr_f411firmware/build/Debug/vr_f411firmware.elf`.
2. Keep the final-mounted headset still and level, body `+Z` up, until the
   startup LED changes from blue to green. Red means the ICM/AK device ID,
   configuration, calibration, or a later I2C read failed.
3. Confirm Windows lists vendor HID `VID_0483&PID_572B`; no COM port is expected.
4. Build/install the driver using
   [steamvr_driver_build_plan.md](steamvr_driver_build_plan.md), then verify all
   three rotation signs in SteamVR Display VR View.
5. Test only short forward/back translations. Lateral and vertical output must
   remain zero; do not tune LiDAR behavior until orientation signs pass.
6. Rigidly align TF-Luna with body `+X`, aim at a fixed opaque plane, and verify
   that only forward/back position is corrected. Disable
   `ENABLE_LIDAR_FORWARD_FUSION` only for builds without the sensor.

The lightweight host check is `vr_f411firmware/tests/pose_fusion_test.c`.
