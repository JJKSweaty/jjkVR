# JJKVR SteamVR Driver Build and Test Plan

This is the PC-driver plan. Firmware details stay in
[firmware_build_plan.md](firmware_build_plan.md), and the overall headset plan
stays in [vr_custom_build_plan.md](vr_custom_build_plan.md).

## Current target

JJKVR is a small GPLv3 fork of Relativty 0.1.1. It retains the original HMD,
extended-display, distortion, HIDAPI, socket tracking, and optional embedded
Python code. The fork changes only the runtime identity, current hardware
defaults, one additive virtual mouse controller, build repeatability, and
directly encountered Relativty defects. Relativty's HMD/display/HID/tracking
architecture remains intact.

Current status: x64 Release builds with MSVC v142 with 0 errors and 0 warnings.
The generated DLL exports `HmdDriverFactory`. SteamVR loads active HMD
`jjkvr.zero`, starts the HID reader, adds right-hand controller `jjkvr.mouse`,
loads its compositor binding, creates distortion surfaces, and reaches
compositor `Startup Complete` without error 302. Visible cursor direction and
click confirmation are the remaining interactive checks.

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
- A separate `jjkvr.mouse` tracked device supplies SteamVR's native visible
  laser cursor without changing the HMD pose.
- Mouse position inside the 2880x1440 headset display aims the cursor;
  left-click is VR select, right-click is VR right-click, and middle-click
  toggles dashboard. On the main monitor, the VR pose is invalid and its
  buttons are released.
- `R` retains Relativty's orientation recenter behavior.
- Relativty's 3-element position normalization loop no longer writes a fourth
  element, and float normalization limits no longer truncate to integers.

The desktop **Display VR View** remains a mirror. The Windows arrow is not drawn
inside the headset; the visible VR cursor is SteamVR's laser/reticle from
`jjkvr.mouse`. The `jjkvr_mouse` settings provide horizontal/vertical
pointer-range tuning without modifying code. Mouse ownership uses the
`jjkvr_extendedDisplay` window rectangle directly.

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
5. On the main monitor, press left, right, and middle mouse over a harmless
   desktop area; confirm no VR laser, click, or dashboard action appears.
6. Move the cursor right onto the headset display, middle-click to open the
   dashboard, aim the visible VR cursor, and left-click a dashboard item.
7. Right-click a target that supports a secondary click.
8. Move the cursor back to the main monitor and confirm the VR laser disappears.
9. Press `R` and confirm the current orientation becomes the forward reference.

Do not start Driver4VR or the old Relativty Python tracker for this test.
