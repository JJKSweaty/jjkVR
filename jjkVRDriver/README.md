# JJKVR SteamVR driver

This is a small, product-focused fork of the Relativty SteamVR driver. The
original display, distortion, HID, socket, and optional embedded-Python paths
are retained as the backbone. The driver, HMD implementation, runtime identity,
and virtual mouse controller are JJKVR-owned.

Build the x64 Release driver with Visual Studio C++ Build Tools and the v142
toolset:

```powershell
.\build.ps1
```

Register `jjkvr\driver.vrdrivermanifest` through SteamVR's `vrpathreg.exe`.
The default hardware settings are STM32 VID/PID `0x0483:0x572B`, a 2880x1440
extended display at `(1920,0)`, and 120 Hz.

The 64-byte HID report keeps report ID `1` and quaternion `(w,x,y,z)` at bytes
0-16. Protocol v2 additionally reads OpenVR position metres from bytes 17-28
when byte 29 has position-valid bit 0 set and byte 30 is `2`. Legacy
orientation-only reports remain supported; HID position is ignored when the
optional TCP tracking server is enabled.

- Move the cursor on any monitor to aim SteamVR's visible laser cursor. The
  virtual right hand stays in front of the HMD instead of disappearing when
  the cursor leaves the headset display.
- Left mouse selects; right mouse sends a VR right-click.
- Middle mouse opens or closes the SteamVR dashboard.
- Mouse side buttons expose application-menu and stick/trackpad clicks when
  present.
- `R` recenters orientation while HID reports are arriving.

If you do not want to use a mouse, the driver also reads an XInput gamepad for
that same right-hand controller path. A PlayStation controller works if Windows
is exposing it as XInput, which usually means Steam Input or DS4Windows is doing
the translation. Right stick moves the cursor, `A` or cross selects, `B` or
circle right-clicks, `Start` opens the SteamVR system action, and `Back` or
share/menu opens application-menu.

The driver reuses SteamVR's animated Quest 2 right-controller model instead of
the Xbox-style generic model. Its Oculus Touch compatibility profile supplies
the legacy trigger, grip, joystick, trackpad, and haptic paths expected by
applications such as Google Earth VR. Tune `jjkvr_mouse` if the pointer's
angular range changes.

Relativty is GPLv3; see `LICENSE`. JJKVR retains the original copyright and
license notices.
