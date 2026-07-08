# vr_f411firmware

STM32F411CEU6 CubeMX/CMake firmware for a small ICM20948 bring-up test.

## Wiring

| Signal | STM32 pin | Notes |
|---|---:|---|
| ICM20948 SCL | PB6 | I2C1 SCL from CubeMX, use 3.3 V pull-up if the breakout has none |
| ICM20948 SDA | PB7 | I2C1 SDA from CubeMX, use 3.3 V pull-up if the breakout has none |
| ICM20948 VCC | 3V3 | Do not power a 3.3 V-only IMU from 5 V |
| ICM20948 GND | GND | Shared ground |
| LED_R | PA0 | Fast blink means IMU read failed or WHO_AM_I was not 0xEA |
| LED_G | PA1 | Slow blink means WHO_AM_I matched 0xEA |
| LED_B | PB0 | Off in this test |

## VS Code Build

1. Open this `vr_f411firmware` folder in VS Code.
2. Install/enable the STM32 VS Code extension when prompted.
3. Select the `Debug` CMake preset.
4. Run `CMake: Build`.

Expected artifacts are under `build/Debug/`, including `vr_f411firmware.elf`.

## Hardware Test

Flash the ELF with ST-LINK or STM32CubeProgrammer, then reset the board.

Expected result: green LED blinks briefly once per second. Fast red blink means the IMU did not answer at I2C address `0x68` or `0x69`, or the returned `WHO_AM_I` value was not `0xEA`.
