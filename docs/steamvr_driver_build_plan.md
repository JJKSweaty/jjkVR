# JJKVR SteamVR Driver Build and Test Plan

This is the PC-driver plan. Firmware details stay in
[firmware_build_plan.md](firmware_build_plan.md), and the overall headset plan
stays in [vr_custom_build_plan.md](vr_custom_build_plan.md).

## Current target

JJKVR is a small GPLv3 fork of Relativty 0.1.1. It retains the original HMD,
extended-display, distortion, HIDAPI, socket tracking, and optional embedded
Python code. The fork changes only the runtime identity, current hardware
defaults, SteamVR mouse inputs, build repeatability, and directly encountered
Relativty defects.

Current status: x64 Release builds with MSVC v142 with 0 errors and 0 warnings.
The generated DLL exports `HmdDriverFactory`. SteamVR loads active HMD
`jjkvr.zero`, starts the HID reader, loads the mouse binding, creates distortion
surfaces, and reaches compositor `Startup Complete` without error 302. Physical
left/right-click confirmation is the remaining interactive check.

## Fixed hardware contract

- USB HID VID/PID: `0x0483:0x572B` (`1155:22315`).
- HID input report: 64 bytes; byte 0 is report ID `1`; bytes 1-16 are
  little-endian float quaternion `(w, x, y, z)`.
- `hmdIMUdmpPackets` is `false`.
- Headset display: `(1920,0)`, 2880x1440, 120 Hz.
- The temporary `LIDAR_HID_TEST` firmware sends synthetic yaw in this same
  quaternion field. The future ICM-20948 fusion output replaces that test data
  without changing the driver packet format.

## Deliberate JJKVR changes

- Runtime name and resources use `jjkvr`; the DLL is `driver_jjkvr.dll`.
- Head pose supplies SteamVR's dashboard laser direction.
- Left mouse updates `/input/click`; right mouse updates `/input/system`.
- `R` retains Relativty's orientation recenter behavior.
- Relativty's 3-element position normalization loop no longer writes a fourth
  element, and float normalization limits no longer truncate to integers.

The desktop **Display VR View** remains a mirror. Clicking the mirror window is
not direct VR interaction: look at a dashboard target with the headset, then
left-click. Mouse-motion aiming is deferred until this simpler path passes.

## Build

From the repository root:

```powershell
cd .\jjkVRDriver
.\build.ps1
```

Output:
`jjkvr\bin\win64\driver_jjkvr.dll`.

## Install or update

Close SteamVR first. Register the runtime folder containing
`driver.vrdrivermanifest`:

```powershell
$vrpathreg = "C:\Program Files (x86)\Steam\steamapps\common\SteamVR\bin\win64\vrpathreg.exe"
& $vrpathreg adddriver "$PWD\jjkvr"
& $vrpathreg show
```

Only one HID-consuming HMD driver should be enabled during a test. Disable the
installed Relativty driver while validating JJKVR; keep its files as a rollback
until JJKVR passes.

## Hardware test

1. Confirm Windows shows HID `VID_0483&PID_572B` and leave BOOT0 low.
2. Start SteamVR and confirm the active HMD serial begins with `jjkvr.`.
3. Confirm the compositor reaches `Startup Complete` without error 302.
4. Vary TF-Luna distance and confirm the view rotates.
5. Right-click to open the dashboard, aim by moving the headset/LiDAR test pose,
   and left-click a dashboard item.
6. Press `R` and confirm the current orientation becomes the forward reference.

Do not start Driver4VR or the old Relativty Python tracker for this test.
