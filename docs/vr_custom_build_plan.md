# Relativty Custom Build — Requirements & 5-Person Team Plan

A working plan for building a custom-PCB version of the Relativty open-source VR headset, split across five people: **PCB/Electronics**, **Firmware**, **Mechanical/Shell**, **Software/Driver**, and **Systems Integration & Tracking**.

---

## 1. System Architecture (what you're actually building)

```
[IMU] --I2C--> [Custom MCU Board] --USB HID--> [PC: SteamVR + Relativty Driver]
                                                        |
[Display Driver Board] <--HDMI/DP-- [PC GPU] -----------
                |
           [Dual/Single Panel in Shell]
```

Three physically separate subsystems have to work together:
1. **Motion tracking chain** — IMU → MCU → USB HID → SteamVR driver (orientation only, 3-DoF).
2. **Display chain** — GPU → driver board → panel(s), completely independent of the MCU; it's just an extended/mirrored monitor.
3. **Mechanical chain** — shell, lenses, strap, cable management, that houses both of the above.

Keeping this separation clear is what lets 5 people work in parallel without blocking each other.

---

## 2. Hardware Requirements (Bill of Materials)

| Component | Requirement | Notes / Example Parts |
|---|---|---|
| MCU | USB HID-capable, I2C master, native USB, hardware FPU preferred | **STM32F411 ("Black Pill" class) — recommended.** Cortex-M4 with hardware FPU (faster/more stable quaternion math than 8-bit AVR), clean USB OTG FS, cheap (~$2–4). Avoid STM32F103 "Blue Pill" clones — many ship with wrong crystal load capacitors, causing flaky USB enumeration. Fallback: ATmega32U4 (proven, zero toolchain friction, what stock firmware targets) if the firmware owner wants to avoid any STM32duino setup risk. |
| IMU | Supported by FastIMU, I2C interface | MPU6050 (cheapest, 6-axis, drifts fastest) / **ICM20948 (chosen — GY-ICM20948V2)** — 9-axis w/ magnetometer, directly supported by FastIMU, same family the stock Relativty guide already validates. Fusion (Mahony/Madgwick) runs in firmware, not onboard the chip. BNO085 remains a valid alternative (onboard fusion, but needs a different library and more RAM — see note in §4 history) if you ever want to swap. |
| Display | Small, high-res, high-refresh, PC-compatible | Needs a driver/controller board — buy display + driver as a matched bundle, never separately. ⚠️ Most cheap driver boards are built around the Toshiba TC358870XBG chip (or a clone of it) — be skeptical of seller claims about supported resolutions/refresh rates, and order one unit to verify before bulk-ordering |
| Lenses | 40mm diameter / 50mm focal length | Aliexpress/AliExpress-equivalent, biconvex or Fresnel |
| Strap + facial interface | Adjustable, washable foam | HTC Vive replacement parts fit well |
| Voltage regulator | Matches IMU + MCU rated voltage | Needed on custom PCB — see §3 |
| USB connector | USB-C recommended over micro-USB | Removes the "cable too long / can't power up" failure mode called out in the base guide |
| Fasteners | M2/M3 screws, heat-set inserts | For 3D-printed shell |
| Cabling | Shielded twisted-pair for I2C runs | Reduces IMU noise if IMU is physically far from MCU |

**Total realistic budget for one prototype unit:** roughly display+driver ($150–190) + lenses/strap/foam (~$30–50) + IMU/MCU/PCB (~$20–40 in parts, see §3 for board fab cost) + printing filament (~$15–25) — the same order of magnitude as the stock build, since you're not buying pre-made electronics boards anymore, just their components.

---

## 3. Custom PCB — Design Requirements

This replaces the "Relativty Motherboard" / off-the-shelf Arduino Pro Micro approach with one integrated board.

**Functional blocks the PCB must include:**
1. **MCU** — **STM32F411 (recommended)**: Cortex-M4 with hardware FPU, native USB OTG FS for HID, plenty of RAM/flash headroom for future features. With the ICM20948 (chosen IMU), the FPU's main benefit is running FastIMU's Mahony/Madgwick fusion math faster and more precisely than an 8-bit core would — a "better" pick, not a hard requirement. Requires the `STM32duino` core so FastIMU can be built with minimal porting effort (same category of work as an RP2040 port). Design in a crystal + correct load capacitors carefully — this is the exact failure mode that makes cheap STM32F103 "Blue Pill" clones unreliable, and you control the layout now, so get it right. **Fallback: ATmega32U4** — genuinely viable with this IMU choice, since FastIMU + ICM20948 is the same combination the stock Relativty guide already validates on Pro Micro-class boards; pick this if the firmware owner wants to avoid any `STM32duino` toolchain setup.
2. **IMU footprint** — place it centrally and away from the USB connector/regulator (magnetic + electrical noise source) if using a magnetometer-equipped IMU.
3. **Power regulation** — a linear or buck regulator stepping USB 5V down to whatever the IMU needs (often 3.3V), with reverse-polarity and overcurrent protection. This is the #1 reliability upgrade over the stock design.
4. **USB-C connector** with proper CC resistors for a data+power-only device.
5. **Status LED(s)** — at minimum one RGB LED for calibration/connection state (see §6).
6. **Expansion header** — broken-out spare I2C/SPI/GPIO pins for future accessories (controllers, eye tracking, extra sensors) even if unused in v1.
7. **Test points** for SDA/SCL/3V3/GND — makes bring-up debugging far faster than probing SMD pins directly.
8. **Mounting holes** matched to the mechanical team's shell mounting bosses — coordinate this early (see §7).

**Deliverables the PCB owner needs to produce:**
- Schematic (KiCad recommended — free, and the original project used Eagle so KiCad is a reasonable modern equivalent)
- PCB layout (2-layer is sufficient; keep IMU traces short and symmetric)
- Gerber files + BOM + pick-and-place file for fab houses (JLCPCB, PCBWay)
- A short **bring-up test procedure** (power on, check regulator output voltage, confirm IMU I2C address responds, confirm MCU enumerates as USB HID)

**Fab cost estimate:** small-batch (5–10 boards) prototyping runs are typically $5–20 for bare boards plus components; assembly service (JLCPCB SMT) adds cost but removes hand-soldering risk for fine-pitch parts like the MCU.

**Design lessons worth stealing from `HadesVR/Basic-HMD-PCB`** (an existing, proven open-source HMD tracking board — worth reviewing directly before finalizing your own schematic):
- Extra filtering capacitance on the 3.3V rail specifically — their Revision 2 called this out as a real reliability fix, not just a nice-to-have.
- Explicit support for 3.3V-only IMUs, not just IMUs that tolerate 5V logic — confirm your chosen IMU's logic level against your regulator output.
- A physical calibration tact switch on the board, paired with a serial-guided calibration flow (plug in, open serial monitor, follow on-screen steps) — much friendlier than a hardcoded calibration-on-boot routine.
- Mount the IMU as flush as possible against the board to minimize offset angle — if using through-hole IMU breakout headers, remove the plastic spacer under the pins so it sits flat.
- A through-hole-only variant is a legitimate design option if you want the PCB hand-solderable by a beginner without reflow equipment — worth discussing in Phase 0 if not everyone on the team has SMD soldering experience.

---

## 4. Firmware Requirements

- Read IMU over I2C at a consistent rate — for the **ICM20948 (chosen IMU)**, the FastIMU library's calibrated sketch is the reference implementation, and it's the same one the stock Relativty build guide already uses.
- Run sensor fusion **in firmware** via FastIMU (Mahony/Madgwick) to turn the ICM20948's raw accel/gyro/magnetometer readings into quaternion or Euler orientation data — unlike the BNO085, the ICM20948 has no onboard fusion silicon, so this step happens on your MCU. This is the main place STM32F411's hardware FPU earns its keep over the ATmega32U4 fallback (§3): faster, more numerically stable fusion math at the same or higher output rate.
- Package orientation data into a USB HID report the SteamVR driver expects (match the existing Relativty driver's HID report format so the PC-side driver doesn't need rewriting — or update both in lockstep if you change the protocol).
- Support an EEPROM-stored calibration routine, triggerable over serial, so users don't recalibrate every boot.
- If moving to STM32F411: set up `STM32duino`, confirm FastIMU compiles against it, and configure USB OTG FS as a HID device (the STM32duino core has a well-trodden USB HID example to start from). Take advantage of the hardware FPU — use the sensor fusion library's float (not fixed-point) code path for the accuracy/latency benefit that's the whole reason to pick this chip.
- Optional: drive the status LED (see §6) based on connection/calibration state, since firmware is what actually knows that state. HadesVR's approach is worth copying directly: define a small color code (e.g. solid green = tracking healthy, blinking red = IMU not found, blinking blue = calibrating) rather than a single on/off indicator — cheap to implement, and it turns "is this even working?" from a serial-monitor debugging session into a glance.
- Account for physical IMU mounting orientation in the calibration routine — if the board sits flat during calibration but is mounted vertically in the final shell (or vice versa), the calibration reference frame needs to account for that, or calibration needs to happen in the final mounted orientation. Make this a firmware `#define`-style toggle rather than hardcoding one orientation.

---

## 5. Software Requirements (PC side)

- **SteamVR + the Relativty OpenVR driver**, updated to:
  - match whatever `hmdPid`/`hmdVid` your custom board reports (get these via Arduino IDE's "Get Board Info" or your MCU's USB descriptor if hand-written).
  - correctly parse the custom PCB's HID report if you changed its format from stock.
- **`default.vrsettings` config** for display geometry (`windowX/Y`, `windowWidth/Height`, `renderWidth/Height`), IPD, and distortion coefficients (`DistortionK1/K2`) — this is per-user/per-display and needs a sane default plus documentation for tuning it.
- A basic **desktop config/calibration utility** is a strong nice-to-have: a small app (even a simple Python/Electron tool) that lets a user set display coordinates and IPD without hand-editing JSON. This is a good scope item for the software owner once the driver itself works.
- Version-control the driver and firmware together, since HID report format changes require synchronized updates on both sides.

---

## 6. Low-Cost Hardware Improvements (differentiators, nothing exotic)

All of these are cheap because you're already committed to a custom PCB and a 3D-printed shell — the marginal cost of most of these is one extra component or one extra print feature.

| Improvement | Why it's worth it | Rough added cost |
|---|---|---|
| 9-axis IMU with magnetometer (ICM20948/BNO085) instead of MPU6050 | Biggest single fix for the yaw-drift complaint that plagues every DIY 3-DoF headset | +$10–20 |
| USB-C instead of micro-USB | Fixes the exact "board won't power up because cable is too long / port underpowered" issue the original guide warns about | ~$0 (same BOM cost, just a different connector) |
| Onboard regulator + protection circuitry | Prevents the "supplying incorrect voltage may damage components" risk mentioned in the guide — currently the user has to get this right manually | ~$1–2 in parts |
| RGB status LED with a defined color code | Immediate, obvious UX win — solid green = healthy, blinking red = IMU fault, blinking blue = calibrating, etc. Removes the "is this even working?" debugging step. This pattern is proven in HadesVR's reference PCB | <$1 |
| Onboard calibration tact switch + serial-guided calibration flow | Turns "recompile firmware to recalibrate" into "press button, follow on-screen prompts" — a real usability upgrade for anyone but the firmware owner | <$1 |
| Physical IPD adjustment slider (mechanical, not just software) | Software-only IPD in the config file doesn't move the lenses — a simple printed slider mechanism makes the headset usable across more face widths without recompiling settings | Print time only |
| Washable/swappable foam face pad via a printed clip frame | Hygiene + comfort differentiator that costs nothing beyond the foam itself | ~$5 |
| Expansion header for future accessories | Costs nothing now, saves a full PCB respin later if you add controllers or an eye-tracking camera | ~$0.50 |
| Basic IR proximity sensor for auto-sleep/wake | Small QoL feature (screen doesn't run full-bright when the headset is set down) | ~$1–2 |
| Cable channeling / strain relief molded into the shell | Reduces the #1 physical failure mode of hand-built headsets (wire fatigue at the MCU/IMU solder joints) | Print design time only |

**Deliberately excluded as "not cheap":** full 6-DoF outside-in or inside-out tracking, wireless streaming, eye tracking cameras, controllers. These are natural v2 roadmap items once the expansion header exists, but they meaningfully raise cost and complexity for a first custom build.

---

## 7. Mechanical/Shell Requirements

- CAD tool (Fusion 360 / FreeCAD / Blender) — should ideally use the original `.STL`/source files as a starting point rather than starting from zero, since lens spacing and IPD geometry are already solved there.
- Must accommodate: the custom PCB's mounting hole pattern (coordinate with PCB owner), the display+driver board's dimensions, the IPD slider mechanism, and cable routing from IMU/MCU location to the display driver board.
- Ventilation consideration for the display driver board (these can run warm).
- Strap attachment points strong enough for repeated on/off cycles (this is a common print failure point — consider heat-set metal inserts instead of printing threads directly into PLA/PETG).

---

## 8. Five-Person Work Split

| Role | Owns | Primary deliverables | Key dependencies on others |
|---|---|---|---|
| **1. PCB/Electronics Lead** | Custom PCB schematic + layout, power design, connectors | KiCad schematic, Gerbers, BOM, bring-up test doc | Needs MCU/IMU choice from Firmware Lead before finalizing schematic; needs shell mounting-hole spec from Mechanical Lead |
| **2. Firmware Engineer** | MCU code: IMU read, sensor fusion, USB HID, calibration, LED control | Working firmware image, calibration routine, HID report spec doc | Needs final IMU part choice to lock in early; HID report spec must be shared with Software Lead before driver work starts |
| **3. Mechanical/Shell Designer** | 3D-printable shell, lens housing, strap system, IPD slider | STL files, screw BOM, assembly instructions | Needs PCB outline/mounting-hole dimensions and display dimensions before finalizing internal geometry |
| **4. Software/Driver Engineer** | SteamVR OpenVR driver, `default.vrsettings` defaults, optional calibration GUI | Updated driver build, config docs, (stretch) desktop config tool | Needs the finalized HID report format from Firmware Engineer; needs real `hmdPid/hmdVid` once PCB is bring-up tested |
| **5. Systems Integration / QA + Tracking** | End-to-end assembly, testing checklist, drift/latency measurement, owns the expansion-header roadmap (future eye tracking / controllers) | Integration test log, known-issues doc, v2 feature roadmap using the expansion header | Depends on all four other roles delivering working sub-parts to integrate |

**Suggested sequencing (rough phases, not hard deadlines):**

1. **Phase 0 — Alignment (all 5):** Agree on MCU choice (STM32F411 vs. ATmega32U4 fallback — decide based on the firmware owner's comfort with `STM32duino` toolchain setup), IMU choice, HID report format, and PCB mounting hole pattern before anyone starts final design. This single meeting prevents the most costly rework.
2. **Phase 1 — Parallel design:** PCB layout, firmware skeleton, shell CAD, and driver updates proceed in parallel, using breadboard/dev-board prototypes (Arduino Pro Micro + loose IMU) instead of waiting on fabbed PCBs.
3. **Phase 2 — PCB fab + bring-up:** Once boards arrive, Electronics + Firmware leads do bring-up together (power check → I2C check → USB enumeration check → orientation data check).
4. **Phase 3 — Integration:** Systems Integration owner assembles a full unit (PCB + shell + display), Software Engineer finalizes driver config against real hardware IDs.
5. **Phase 4 — Iteration:** Fix drift, comfort, cable routing issues found in Phase 3; this is where most of the "cheap improvements" in §6 either get added or get deferred to v2.

---

## 9. Integration Risks to Watch

- **HID report format mismatches** between firmware and driver are the most common "it built but doesn't work" failure — freeze this spec in Phase 0.
- **Display coordinate math** (`windowX/Y`) depends on which physical display port the headset lands on and which monitor is Primary in Windows — this is a per-machine setting, not something to hardcode, so document it clearly rather than trying to "fix" it in code.
- **IMU placement on the PCB relative to the display driver board** — driver boards can be electrically noisy; keep them physically separated if possible.
- **Voltage mismatch** between MCU and IMU rated voltage is called out explicitly in the original build guide as a damage risk — the onboard regulator (§3/§6) is the permanent fix for this.

---

## 10. What To Do First — Zero-Spend Phase 0

Everything below can happen before the team spends a single dollar. This is where the "Phase 0 alignment" meeting from §8 actually gets executed — use it to de-risk the expensive decisions before any ordering starts.

| Owner | Zero-cost task |
|---|---|
| **All 5 (together)** | Hold the Phase 0 alignment meeting: lock in MCU choice (STM32F411 vs ATmega32U4 fallback), IMU choice, HID report format, and PCB mounting-hole coordinate system before any design work starts. |
| **PCB/Electronics Lead** | Install KiCad (free) and start the schematic using the confirmed MCU/IMU. Study `HadesVR/Basic-HMD-PCB`'s open schematic directly — no need to reinvent the regulator/filtering design from scratch. |
| **Firmware Engineer** | Install Arduino IDE + `STM32duino` core, install FastIMU library, and get it compiling against a *simulated* target (even without hardware yet) to catch toolchain issues early. Read `HadesVRBLE`'s firmware for the ESP32 wireless reference. |
| **Mechanical/Shell Designer** | Start CAD (Fusion 360 has a free personal-use tier; FreeCAD is fully free) using the original Relativty `.STL` files as a base. Can model the entire shell, IPD slider mechanism, and mounting bosses before a single part is printed. |
| **Software/Driver Engineer** | Fork the Relativty OpenVR driver repo, read through the existing `default.vrsettings` structure, and draft the updated HID-parsing code path against the *planned* report format — this can be written and even unit-tested without real hardware. |
| **Systems Integration/QA** | Build the actual BOM/cost tracker (this spreadsheet) and start pricing out real supplier quotes without ordering. Also the right person to read HadesVR's `Tracking.md`/`Controllers.md` in full and draft the v2 tracking spec now, while it's fresh. |
| **All 5** | Do the PPD/FOV lens math for your chosen 40mm/50mm lenses and target panel resolution in a spreadsheet — this tells you what to actually expect optically (per the Quest 3 comparison discussion) before committing to a display purchase. |

**The one thing worth spending on first, if anything:** a single unit of the display+driver board (§2), ordered alone, specifically to verify the seller's resolution/refresh/power claims before a bulk order or before finalizing PCB power-rail specs around it. Everything else above can happen in parallel at zero cost while that one part ships.

---

## 11. Positional Tracking & Wireless Roadmap (v2 — informed by HadesVR)

Reviewing `HadesVR`'s docs and its `Basic-HMD-PCB` / `HadesVRBLE` repos surfaced concrete, cheaper paths for two features that were previously scoped as vague "someday" items. Both stay v2 (post-v1-working-headset), owned by the **Systems Integration / Tracking** role, and both plug into the expansion header already planned in §3.

### 6-DoF positional tracking — how to actually build it, and yes, IR is the right call

You're right to expect IR sensors here — and it's worth upgrading the plan from the general "LED-blob + webcam" idea to the specific, well-proven version of it: **an IR LED on the headset, tracked by a webcam modified to see only infrared.** This is the same fundamental approach the original Oculus DK2/CV1 used ("Constellation" tracking), and it's a well-documented DIY path via PSMoveServiceEX.

**Why IR instead of a visible-colored LED (the version I described earlier):** a visible-light colored blob can get confused by anything else that color in the room, and its brightness/visibility shifts with ambient lighting. An IR LED, tracked through a camera that's been filtered to reject visible light, is invisible to your eyes but stands out as the brightest thing in frame to the camera — regardless of room lighting color or clutter. It's a strict upgrade for the same cost.

**What you actually need:**

| Part | Role | Est. Cost | Notes |
|---|---|---|---|
| A cheap USB webcam — PS3Eye is the community-favorite choice | The "IR camera" | $5–20 (often secondhand) | High frame rate (60fps+, some mods hit 120–187fps) matters more here than resolution — faster camera = lower tracking latency |
| IR-cut filter removal + IR-pass filter | Turns the webcam into an IR-only camera | ~$0–10 | Classic hobbyist hack: open the camera, remove the internal IR-cut filter, replace with an IR-pass filter (a piece of fully-exposed/developed camera negative film works and costs nothing; a proper Wratten-87-equivalent filter is a few dollars if you want a cleaner result) |
| IR LED (940nm or 850nm) | The tracked "beacon" on the headset | $0.20–1 | Mount behind a small diffuser (a ping-pong-ball half still works) so the camera sees a soft consistent blob rather than a harsh point source |
| Current-limiting resistor | Protects the LED | <$1 | Sized for your chosen LED's forward voltage/current, powered off the same rail as the rest of your headset electronics |
| PSMoveServiceEX (software, free) | Does the actual computer-vision tracking + calibration | $0 | Same software HadesVR uses; this is the "known-working, documented path" referenced earlier in this plan |

**How position and rotation combine:** your GY-ICM20948V2 is a solid pick — it's exactly the 9-axis IMU already recommended in §2/§6 of this plan, and per the last discussion, adding the magnetometer already fixes yaw drift. Nothing about adding IR tracking changes that role: **the IMU keeps handling all rotation (pitch/roll/yaw), and the IR camera+LED setup only adds the missing position half (X/Y/Z)** — this is the same architecture note in the paragraph below the table, just now concrete about how the position half is actually produced. PSMoveServiceEX estimates position from the LED blob's pixel location (x/y) plus its apparent size in frame, since the ping-pong-ball diffuser has a known physical diameter — bigger in-frame = closer to the camera, smaller = farther. Your SteamVR driver then merges: rotation quaternion from the IMU's USB HID stream, position vector from PSMoveServiceEX's tracking output, into one combined 6-DoF pose update.

**Recommended build order for this specific piece (validate cheaply before modifying anything):**
1. Get PSMoveServiceEX installed and working with a stock, **unmodified** webcam and a plain visible-colored ball first — this proves the whole software pipeline (camera → tracker → position data → your driver) works before you touch a soldering iron or a screwdriver on the camera.
2. Only once that's confirmed, do the IR conversion: open the webcam, pull the IR-cut filter, add the IR-pass filter, swap the visible LED for an IR LED on the headset, and recalibrate. This isolates "does the pipeline work" from "does the IR hack work," so a failure is easy to diagnose.
3. Mount the IR LED somewhere on the headset that stays in the camera's line of sight during normal head movement — occlusion (hands, hair, or your own body blocking the LED) is the main real-world failure mode of single-point tracking, so this is worth testing physically, not just assuming from the CAD model.
4. Calibrate PSMoveServiceEX's camera position/orientation relative to your room once the camera is mounted in its final spot — this needs to be redone if you move the camera.

**Known limitations to set expectations on:** single-camera tracking loses you the moment the LED is occluded from that camera's view (a second camera at a different angle is the fix, addable later, not required for v1 of this feature); tracking volume is limited to roughly the camera's field of view and a few meters of range, so this gives you room-scale-ish tracking, not whole-house tracking; and it's meaningfully less precise than a lighthouse system, which is exactly why lighthouse is still listed below as the v3 upgrade path once this simpler version proves the concept works end-to-end.

### Lighthouse / libsurvive — the true "IR sensors on the headset" path (v3 upgrade)

| Approach | How it works | Cost | Notes |
|---|---|---|---|
| **Lighthouse / libsurvive (outside-in)** | Base stations sweep IR laser lines; photodiode sensors *mounted on the headset itself* time the sweeps to compute precise 3D position | Moderate — used Vive base stations ~$40–80 each, photodiode sensor board is cheap | This is the version where the IR sensors genuinely live on the headset (rather than an external camera). Higher precision, works in normal room lighting (no need to dim lights or avoid sunlight the way camera-based tracking does), but base stations cost more and libsurvive integration is more involved than PSMoveServiceEX. Good v3 target once the camera-based approach above proves the concept and interface. |

### Wireless — two separate links, not one

It's worth being precise about this since it's easy to conflate: a wireless headset actually needs *two different* radio links serving different purposes, and they should use different technologies.

| Link | Purpose | Right technology | Why |
|---|---|---|---|
| Headset ↔ PC | Send headset pose (and eventually video) to/from the computer | WiFi (ESP32) — see prior wireless discussion | Needs range across a room; `HadesVRBLE` (ESP32-C3 fork) is a real reference implementation to study rather than build from scratch |
| Controller ↔ Headset | Send controller button/IMU data to the headset, which relays it to the PC | **NRF24L01** | Very short range, very low latency, dirt cheap (~$1–2/module) — this is what HadesVR uses for exactly this link, and it's the wrong job for WiFi (unnecessary range/power cost for a link that's centimeters-to-a-few-meters) |

This only matters once controllers are in scope — for a headset-only v1/v2, you only need the Headset↔PC link.

---

## 12. Full Bill of Materials & Total Cost (v1 — wired, 3-DoF)

This is the complete per-unit cost to build one working prototype, using the cheaper choices we settled on throughout this plan (STM32F411, ICM20948/BNO085, HDMI-to-MIPI display board instead of the pricier DP version).

> **Live cost tracker:** see `relativty_full_bom.xlsx` for a formula-driven version of this BOM (adjust quantities/prices and totals recalculate automatically), including the **Wisecoco 120Hz DP-to-MIPI board (~$220)** as the priced alternative to the cheaper HDMI-to-MIPI option below — use whichever fits your budget vs. guaranteed-120Hz priority.

| Item | Purpose | Est. Cost | Notes |
|---|---|---|---|
| STM32F411 chip/dev board | MCU | $3–5 | Dev board (Black Pill) for prototyping; bare chip once on custom PCB |
| ICM20948 or BNO085 breakout | IMU | $10–20 | 9-axis, FastIMU-supported |
| LS029B3SX02 panel + HDMI-to-MIPI driver board | Display | $70–120 | The cheaper alternative we found vs. the $225 DP version — **this is your single biggest line item** |
| Lenses (40mm/50mm pair) | Optics | $10–20 | Biconvex or Fresnel |
| Strap + facial foam (Vive-replacement style) | Comfort/mounting | $15–25 | |
| Custom PCB fab (5–10 bare boards) | Electronics | $5–20 | Split across however many boards you order — cheap even at small batch |
| PCB components (regulator, USB-C conn., LED, passives, headers) | Electronics | $10–20 | Per board |
| 3D printing filament | Shell | $15–25 | Assumes access to a printer already — see note below |
| Screws, heat-set inserts, fasteners | Mechanical | $10–15 | |
| Misc: wire, solder, connectors, flux | Assembly | $10–15 | |
| Shipping buffer (AliExpress/China-sourced parts) | — | $15–25 | Commonly underestimated — budget for it explicitly |
| **Total (v1, one unit)** | | **≈ $165–290** | Wide range mainly driven by which display driver board you land on |

**Not included above — check before assuming they're free:**
- A 3D printer, soldering iron, multimeter, and Arduino IDE (free) — if the team doesn't already have access to a printer and basic tools, that's a separate one-time cost (~$150–300 for an entry printer) that should be budgeted and decided *before* Phase 0 below, not discovered mid-build.
- A second/spare PC monitor for the early software validation step in §12 — most people already have one; flag it if not.

**Optional v2 additions, budgeted separately (see §10):**

| Item | Purpose | Est. Cost |
|---|---|---|
| Basic webcam | 6-DoF LED-blob tracking | $20–30 |
| High-brightness LED + ping-pong ball + resistor | 6-DoF tracking target | <$5 |
| ESP32 module | Wireless headset↔PC link | $5–10 |
| LiPo battery + TP4056-class charge/protection module | Onboard power for wireless | $10–20 |
| NRF24L01 modules (×2+) | Controller↔headset link, only if building controllers | $2–4 each |

---

## 13. Spend-Smart Build Order (validate before you buy)

The goal here is to spend almost nothing until you've proven the software/firmware chain actually works — the display is your biggest expense, so it should be the *last* major purchase, not the first. This directly mirrors a warning already in the original Relativty guide: you can test your build on a normal PC monitor before spending money on lenses and a VR panel.

| Phase | What to do | Spend | Why this order |
|---|---|---|---|
| **0. Software/driver bring-up** | Get the SteamVR driver + firmware talking, using whatever cheap/already-owned Arduino Pro Micro + any IMU breakout you can find (doesn't need to be your final parts) | ~$0–15 | Proves the hardest integration risk (§9: HID report mismatches) before you've spent real money on anything |
| **1. Breadboard MCU+IMU prototype** | Wire up your *actual chosen* MCU (STM32F411) and IMU on a breadboard, get calibrated orientation data flowing into SteamVR | ~$15–30 | Validates your real component choices before committing to a PCB fab run |
| **2. Validate display pipeline on a spare monitor** | Configure `default.vrsettings` (`windowX/Y`, resolution) against a normal monitor set as an extended display — confirms your driver's display-handling logic works | $0 (use a monitor you already have) | This is the step that lets you avoid buying the VR panel until everything upstream is proven working |
| **3. Buy the display + driver board** | Now order the LS029B3SX02 + HDMI-to-MIPI board (single unit first — see the sourcing caution in §2) | $70–120 | Your biggest single expense — only spend it once you know the rest of the chain works |
| **4. Buy lenses, strap, foam** | Order once you have real panel dimensions in hand, so the mechanical design isn't guessing | $25–45 | |
| **5. Finalize and fab the custom PCB** | Schematic/layout should already be done in parallel during Phases 0–2 (per §8 Phase 1); order the actual fab run now that MCU/IMU choices are proven on the breadboard | $15–40 | Don't fab before Phase 1 confirms your component choices actually work together |
| **6. 3D print the shell** | Print once PCB and display dimensions are both final, so mounting holes and cutouts are correct the first time | $15–25 | Avoids reprinting a shell around parts that don't fit |
| **7. Assemble & integrate** | Full unit assembly per §8 Phase 3–4 | $0 (labor only) | |
| **8. (Optional) v2 additions** | Wireless tracking, 6-DoF LED-blob tracking, battery — only after v1 works end-to-end | See §12 optional table | Keeps scope creep from delaying a working v1 |

**The one habit this order is designed to build:** never spend on the next phase's parts until the current phase's *software or fit* has been proven — on a breadboard, on a spare monitor, or against real measured dimensions. It's the same principle the original Relativty guide already gives for the display (test on a monitor first) applied consistently across the whole build.

---

*This plan is based on the "Current Recommended Build (May 2023)" approach in the Relativty README, adapted for a custom PCB and split across a 5-person team, and cross-referenced against HadesVR's docs and open-source reference PCB. Component prices are rough estimates — confirm current pricing/availability before finalizing the BOM.*
