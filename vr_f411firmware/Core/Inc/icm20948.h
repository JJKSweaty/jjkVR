#ifndef ICM20948_H
#define ICM20948_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

typedef struct
{
  I2C_HandleTypeDef *i2c;
  uint16_t address;
  float accel_bias_g[3];
  float gyro_bias_dps[3];
  bool mag_available;
} Icm20948;

typedef struct
{
  float accel_g[3];
  float gyro_dps[3];
  float mag_uT[3];
  bool mag_valid;
} Icm20948Sample;

bool Icm20948_Init(Icm20948 *imu, I2C_HandleTypeDef *i2c);
bool Icm20948_CalibrateStationary(Icm20948 *imu, uint16_t sample_count,
                                  uint32_t sample_period_ms);
bool Icm20948_Read(Icm20948 *imu, Icm20948Sample *sample);

#endif
