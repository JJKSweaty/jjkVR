# jjkVR STM32 IMU bring-up

Target: STM32F411CEU7/Black Pill-style board, no display driver, no SteamVR HID yet.

## Pins

| Signal | STM32 pin | Notes |
|---|---:|---|
| ICM20948 SCL | PB8 | I2C1 SCL, add 4.7k pull-up to 3V3 if the breakout lacks one |
| ICM20948 SDA | PB9 | I2C1 SDA, add 4.7k pull-up to 3V3 if the breakout lacks one |
| ICM20948 VCC | 3V3 | Do not feed a 3.3V-only IMU from 5V |
| ICM20948 GND | GND | Shared ground |
| Serial TX | PA2 | USART2 TX, 115200 8N1 |
| Serial RX | PA3 | USART2 RX, optional for this test |
| Status LED | PC13 | Black Pill onboard LED, active low |

## Test

1. Open `jjkVRFirmware` in VS Code.
2. In the STM32 extension, install/activate the GNU Tools bundle if prompted.
3. Build the `Debug` preset.
4. Flash with ST-LINK or STM32CubeProgrammer.
5. Open serial at `115200 8N1`.

Expected serial output:

```text
jjkVR STM32F411 IMU bring-up
I2C1 PB8=SCL PB9=SDA, USART2 PA2=TX PA3=RX, LED=PC13
ICM20948 addr=0x68 WHO_AM_I=0xEA
IMU OK
```

Success blinks PC13 briefly once per second. Failure blinks fast and prints the value it read.
