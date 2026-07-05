# jjkVR Agent Notes

These rules are project memory for future agent turns in this repo.

- Use minimum working code for the requested feature. Do not add speculative layers, drivers, protocols, or abstractions.
- Keep firmware file structure embedded-friendly: generated/startup/vendor files stay separate from app code, hardware choices are documented near the code that depends on them, and build artifacts stay ignored.
- Add comments where they prevent hardware or embedded mistakes: register constants, pin choices, clock/timing assumptions, blocking behavior, calibration limits, voltage/I2C assumptions, and failure modes.
- Do not comment obvious C syntax or simple assignments. Comments should explain why the code is safe or intentionally simple, not restate what it does.
- Keep `README.md` and the relevant subproject README updated when pins, wiring, build steps, serial output, test flow, dependencies, or hardware assumptions change.
- For firmware bring-up, prefer one small runnable hardware check before adding the next feature.
