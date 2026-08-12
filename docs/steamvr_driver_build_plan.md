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
and loads its compositor binding. The current driver build and resource JSON
pass local checks; headset and Google Earth interaction remain hardware checks.

## Fixed hardware contract

- USB HID VID/PID: `0x0483:0x572B` (`1155:22315`).
- HID input report v2:

  | Offset | Content |
  |---:|---|
  | 0 | report ID `1` |
  | 1-16 | little-endian float32 quaternion `(w,x,y,z)`, already in OpenVR coordinates |
  | 17-28 | little-endian float32 position `(x,y,z)` in metres |
  | 29 | flags: `0x01` position valid, `0x02` TF-Luna correction accepted |
  | 30 | protocol version `2` |
  | 31 | sensor diagnostic status; logged when it changes |
  | 32-63 | reserved, currently zero |

  The driver consumes v2 HID position only when `startTrackingServer` is
  false. Legacy ID-1 quaternion reports remain orientation-compatible.
- `hmdIMUdmpPackets` is `false`.
- Headset display: `(1920,0)`, 2880x1440, 120 Hz.
- Firmware maps its headset axes to OpenVR before sending; the driver does not
  perform a second coordinate transform.

## Deliberate JJKVR changes

- Runtime name and resources use `jjkvr`; the DLL is `driver_jjkvr.dll`.
- A separate `jjkvr.mouse` tracked device supplies SteamVR's native laser and
  the installed Quest 2 right-controller render model, avoiding the generic
  Xbox-style model without bundling a custom mesh.
- The cursor position on whichever monitor contains it aims the pointer. The
  virtual controller stays right/down/forward of the HMD so it remains visible.
  Left-click is trigger/select, right-click is grip, middle-click is system,
  and mouse side buttons provide menu and trackpad/joystick click compatibility.
- `R` retains Relativty's orientation recenter behavior.
- Relativty's 3-element position normalization loop no longer writes a fourth
  element, and float normalization limits no longer truncate to integers.

The desktop **Display VR View** remains a mirror. The Windows arrow is not drawn
inside the headset; the visible VR cursor is SteamVR's laser/reticle from
`jjkvr.mouse`. The `jjkvr_mouse` settings provide horizontal/vertical
pointer-range tuning without modifying code.

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
4. Remain still through startup, then verify yaw, pitch, and roll signs.
5. Verify short forward/back movement; lateral and vertical position stay zero.
6. With the default-enabled TF-Luna fusion rigidly mounted, verify it only
   corrects forward/back motion against a fixed target and does not change
   rotation.
7. Move the cursor on either monitor and confirm the visible pointer follows it.
8. Middle-click to open the dashboard, then left-click a dashboard item.
9. In Google Earth VR, confirm the right controller is visible and left-click
   selects; use the mouse side buttons only when the app requests menu/pad input.
11. Press `R` and confirm the current orientation becomes the forward reference.

Do not start Driver4VR or the old Relativty Python tracker for this test.
