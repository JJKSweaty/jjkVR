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

- Move the cursor right onto the 2880x1440 headset display to enable and aim
  SteamVR's visible laser cursor and its static right-hand model.
- Left mouse selects; right mouse sends a VR right-click.
- Middle mouse opens or closes the SteamVR dashboard.
- `R` recenters orientation while HID reports are arriving.

On the main monitor, the VR mouse pose is invalid and every VR mouse button is
released, so desktop clicks are not mirrored into VR. SteamVR displays its
native laser/reticle while the cursor is on the headset display. Tune
`jjkvr_extendedDisplay` if the headset display rectangle changes, or
`jjkvr_mouse` if the pointer's angular range changes.

Relativty is GPLv3; see `LICENSE`. JJKVR retains the original copyright and
license notices. The CC0 hand geometry is adapted from Robin Lamb's
[Low Poly FPS Rifle and Hands](https://opengameart.org/content/low-poly-fps-rifle-and-hands);
its texture was generated for JJKVR.
