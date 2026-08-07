# vr_f411firmware

STM32F411CEU6 CubeMX/CMake firmware for a TF-Luna I2C bring-up test.

## Wiring

| Signal | STM32 pin | Notes |
|---|---:|---|
| TF-Luna pin 1 (+5V) | 5V | Sensor supply must be 3.7-5.2 V; do not use the 3.3 V rail |
| TF-Luna pin 2 (SDA) | PB7 | I2C1 SDA with pull-up to 3.3 V |
| TF-Luna pin 3 (SCL) | PB6 | I2C1 SCL with pull-up to 3.3 V |
| TF-Luna pin 4 (GND) | GND | Shared ground |
| TF-Luna pin 5 (mode) | GND | Required to select I2C mode at power-up |
| LED_R | PA0 | I2C read failed |
| LED_G | PA1 | Valid 20-800 cm reading with adequate signal |
| LED_B | PB0 | Sensor started, but reading is currently unreliable |

## VS Code Build

1. Open this `vr_f411firmware` folder in VS Code.
2. Install/enable the STM32 VS Code extension when prompted.
3. Select the `Debug` CMake preset.
4. Run `CMake: Build`.

Expected artifacts are under `build/Debug/`, including `vr_f411firmware.elf`.

## USB serial output

After flashing, set BOOT0 low and reconnect USB. Open the STM32 virtual COM port at 115200 baud in a serial terminal. The firmware sends `distance_cm,amplitude` every 100 ms:

```text
142,896
143,901
```

An `error,i2c` line means the TF-Luna did not answer.

## Hardware Test

Flash the ELF with ST-LINK or STM32CubeProgrammer, then reset the board.

Point the TF-Luna at an opaque target at least 20 cm away. Green means a valid reading, blue means the sensor answered but the distance or signal is unreliable, and red means the sensor did not answer at I2C address `0x10`.
