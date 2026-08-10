# jjkVR Firmware Build Plan

This is the working firmware plan for the STM32F411 PCB. The overall headset
plan remains in [vr_custom_build_plan.md](vr_custom_build_plan.md).

## Current test: TF-Luna to SteamVR

Goal: prove the live path `TF-Luna -> STM32 -> USB HID -> JJKVR driver ->
SteamVR display` before the ICM-20948 is connected.

**Status: passed on hardware.** Changing TF-Luna distance visibly rotated the
SteamVR view through the complete I2C, firmware, USB HID, driver, and compositor
path.

- Firmware define: `LIDAR_HID_TEST` in `Core/Src/main.c`.
- USB: vendor HID, VID `1155` (`0x0483`), PID `22315` (`0x572B`).
- Report: ID `1`, then four 32-bit floats `(w, x, y, z)`, padded to 64 bytes.
- Test mapping: 20-200 cm becomes -90 to +90 degrees of yaw.
- LED: green = report sent from a valid reading; blue = unreliable reading;
  red = no TF-Luna response at I2C address `0x10`.

Driver decision: use the JJKVR driver, a small fork that retains Relativty's
display, distortion, HID, socket, and optional Python code. Its float-packet
path reads one 64-byte HID report containing report ID `1` and quaternion
`(w, x, y, z)`, matching this firmware. LiDAR range is still an artificial yaw
input for this test, not real 3-D position.

The yaw mapping is deliberately artificial. A single forward-facing range
sensor does not measure headset X/Y/Z or eye motion. The test only proves that
changing sensor data reaches and visibly changes the SteamVR pose.

## Install SteamVR and the JJKVR driver

Install SteamVR, then follow the short
[JJKVR driver build and test plan](steamvr_driver_build_plan.md). The checked-in
driver defaults already match this PCB, firmware report, and display layout.

## SteamVR test

Current Windows layout: the `1920x1080` desktop monitor is on the left and the
VR display is immediately to its right at `2880x1440`, 120 Hz, with the top
edges aligned. If the left monitor is the Windows primary display, use
`windowX: 1920`, `windowY: 0`, `windowWidth/renderWidth: 2880`, and
`windowHeight/renderHeight: 1440`. Use `displayFrequency: 120`.

1. Build and flash `build/Debug/vr_f411firmware.elf`, leave BOOT0 low, and
   power-cycle the PCB.
2. In Windows Device Manager, confirm a vendor-defined HID appears. This build
   does not expose a COM port.
3. Configure the JJKVR display coordinates and resolution for the attached
   VR display, then start SteamVR.
4. Cancel or close SteamVR Room Setup when it asks for base-station visibility.
   This temporary 3-DoF test has no lighthouse tracking; open SteamVR's
   **Display VR View** instead.
5. Point the TF-Luna at an opaque target and vary the distance between 20 and
   200 cm. With a green LED, the rendered view should rotate smoothly.

**Display VR View** is a non-interactive mirror and does not show the Windows
mouse cursor. JJKVR adds a separate virtual mouse controller: move the cursor
onto the headset display to enable and aim SteamVR's visible laser cursor,
left-click to select, right-click for a VR secondary click, and middle-click to
toggle the dashboard. Moving back to the main monitor releases the VR buttons
and hides the laser. This is isolated from the firmware pose path.

## Next firmware stages

1. Connect the ICM-20948 on I2C1 and prove its address and raw accel/gyro/mag
   readings with one small hardware check.
2. Add calibration and float quaternion fusion for the 9-DoF IMU.
3. Set `LIDAR_HID_TEST` to `0` and send the fused IMU quaternion in the same
   JJKVR/Relativty-compatible HID report.
4. If lidar translation is still wanted, version the HID report and update the
   PC driver in lockstep to read a position field. Define the physical model
   first: one range value alone is not a 3-D position.
5. Validate orientation, then position, then combined pose in that order.

Driver plan: [steamvr_driver_build_plan.md](steamvr_driver_build_plan.md).
Reference implementation: [Relativty](https://github.com/relativty/Relativty).
