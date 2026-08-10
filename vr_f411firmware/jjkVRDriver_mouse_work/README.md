# JJKVR SteamVR driver

This is a small, product-focused fork of the Relativty SteamVR driver. The
original display, distortion, HID, socket, and optional embedded-Python paths
are retained. JJKVR changes the runtime identity and adds mouse buttons for
SteamVR's head-gaze dashboard pointer.

Build the x64 Release driver with Visual Studio C++ Build Tools and the v142
toolset:

```powershell
.\build.ps1
```

Register `jjkvr\driver.vrdrivermanifest` through SteamVR's `vrpathreg.exe`.
The default hardware settings are STM32 VID/PID `0x0483:0x572B`, a 2880x1440
extended display at `(1920,0)`, and 120 Hz.

- Aim the dashboard pointer with the headset pose.
- Left mouse selects.
- Right mouse opens or closes the SteamVR dashboard.
- `R` recenters orientation while HID reports are arriving.

Relativty is GPLv3; see `LICENSE`. JJKVR retains the original copyright and
license notices.
