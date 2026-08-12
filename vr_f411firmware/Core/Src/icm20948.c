#include "icm20948.h"

#include <string.h>

#define ICM_ADDRESS_LOW          (0x68U << 1)
#define ICM_ADDRESS_HIGH         (0x69U << 1)
#define AK09916_ADDRESS          (0x0CU << 1)
#define I2C_TIMEOUT_MS           10U

#define ICM_BANK_SELECT          0x7FU
#define ICM_WHO_AM_I             0x00U
#define ICM_USER_CTRL            0x03U
#define ICM_LP_CONFIG            0x05U
#define ICM_PWR_MGMT_1           0x06U
#define ICM_PWR_MGMT_2           0x07U
#define ICM_INT_PIN_CFG          0x0FU
#define ICM_ACCEL_XOUT_H         0x2DU

#define ICM_GYRO_SMPLRT_DIV      0x00U
#define ICM_GYRO_CONFIG_1        0x01U
#define ICM_ACCEL_SMPLRT_DIV_1   0x10U
#define ICM_ACCEL_SMPLRT_DIV_2   0x11U
#define ICM_ACCEL_CONFIG         0x14U

#define AK_WIA1                  0x00U
#define AK_ST1                   0x10U
#define AK_CNTL2                 0x31U
#define AK_CNTL3                 0x32U

#define ACCEL_LSB_PER_G          8192.0f
#define GYRO_LSB_PER_DPS         32.8f
#define MAG_UT_PER_LSB           0.15f
#define MAG_MIN_FIELD_UT         15.0f
#define MAG_MAX_FIELD_UT        100.0f

/* Mounted-sensor tuning knobs; replace these after a real calibration. */
static const float mag_bias_uT[3] = {0.0f, 0.0f, 0.0f};
static const float mag_scale[3] = {1.0f, 1.0f, 1.0f};

static bool write_register(Icm20948 *imu, uint8_t reg, uint8_t value)
{
  return HAL_I2C_Mem_Write(imu->i2c, imu->address, reg,
                           I2C_MEMADD_SIZE_8BIT, &value, 1U,
                           I2C_TIMEOUT_MS) == HAL_OK;
}

static bool read_register(Icm20948 *imu, uint8_t reg, uint8_t *data,
                          uint16_t size)
{
  return HAL_I2C_Mem_Read(imu->i2c, imu->address, reg,
                          I2C_MEMADD_SIZE_8BIT, data, size,
                          I2C_TIMEOUT_MS) == HAL_OK;
}

static bool select_bank(Icm20948 *imu, uint8_t bank)
{
  return write_register(imu, ICM_BANK_SELECT, (uint8_t)(bank << 4));
}

static int16_t big_endian_i16(const uint8_t *data)
{
  return (int16_t)(((uint16_t)data[0] << 8) | data[1]);
}

static int16_t little_endian_i16(const uint8_t *data)
{
  return (int16_t)(((uint16_t)data[1] << 8) | data[0]);
}

static bool read_motion_raw(Icm20948 *imu, int16_t accel[3], int16_t gyro[3])
{
  uint8_t data[12];

  if (!select_bank(imu, 0U) ||
      !read_register(imu, ICM_ACCEL_XOUT_H, data, sizeof(data)))
  {
    return false;
  }

  for (uint32_t axis = 0U; axis < 3U; axis++)
  {
    accel[axis] = big_endian_i16(&data[axis * 2U]);
    gyro[axis] = big_endian_i16(&data[6U + axis * 2U]);
  }
  return true;
}

static bool configure_magnetometer(Icm20948 *imu)
{
  uint8_t id[2];
  uint8_t value;

  if (HAL_I2C_Mem_Read(imu->i2c, AK09916_ADDRESS, AK_WIA1,
                       I2C_MEMADD_SIZE_8BIT, id, sizeof(id),
                       I2C_TIMEOUT_MS) != HAL_OK ||
      id[0] != 0x48U || id[1] != 0x09U)
  {
    return false;
  }

  value = 0x01U;
  if (HAL_I2C_Mem_Write(imu->i2c, AK09916_ADDRESS, AK_CNTL3,
                        I2C_MEMADD_SIZE_8BIT, &value, 1U,
                        I2C_TIMEOUT_MS) != HAL_OK)
  {
    return false;
  }
  HAL_Delay(1U);

  value = 0x00U;
  if (HAL_I2C_Mem_Write(imu->i2c, AK09916_ADDRESS, AK_CNTL2,
                        I2C_MEMADD_SIZE_8BIT, &value, 1U,
                        I2C_TIMEOUT_MS) != HAL_OK)
  {
    return false;
  }
  HAL_Delay(1U);

  /* Continuous mode 4 is the AK09916's 100 Hz setting. */
  value = 0x08U;
  return HAL_I2C_Mem_Write(imu->i2c, AK09916_ADDRESS, AK_CNTL2,
                           I2C_MEMADD_SIZE_8BIT, &value, 1U,
                           I2C_TIMEOUT_MS) == HAL_OK;
}

bool Icm20948_Init(Icm20948 *imu, I2C_HandleTypeDef *i2c)
{
  uint8_t identity;
  const uint16_t addresses[] = {ICM_ADDRESS_LOW, ICM_ADDRESS_HIGH};

  memset(imu, 0, sizeof(*imu));
  imu->i2c = i2c;
  for (uint32_t index = 0U; index < 2U; index++)
  {
    imu->address = addresses[index];
    if (select_bank(imu, 0U) &&
        read_register(imu, ICM_WHO_AM_I, &identity, 1U) &&
        identity == 0xEAU)
    {
      break;
    }
    imu->address = 0U;
  }
  if (imu->address == 0U)
  {
    return false;
  }

  if (!write_register(imu, ICM_PWR_MGMT_1, 0x80U))
  {
    return false;
  }
  HAL_Delay(100U);
  if (!select_bank(imu, 0U) ||
      !write_register(imu, ICM_PWR_MGMT_1, 0x01U) ||
      !write_register(imu, ICM_PWR_MGMT_2, 0x00U) ||
      !write_register(imu, ICM_LP_CONFIG, 0x00U))
  {
    return false;
  }
  HAL_Delay(50U);

  /* 225 Hz, +/-1000 dps and +/-4 g, both with roughly 50 Hz DLPF. */
  if (!select_bank(imu, 2U) ||
      !write_register(imu, ICM_GYRO_SMPLRT_DIV, 4U) ||
      !write_register(imu, ICM_GYRO_CONFIG_1, 0x1DU) ||
      !write_register(imu, ICM_ACCEL_SMPLRT_DIV_1, 0U) ||
      !write_register(imu, ICM_ACCEL_SMPLRT_DIV_2, 4U) ||
      !write_register(imu, ICM_ACCEL_CONFIG, 0x1BU) ||
      !select_bank(imu, 0U) ||
      !write_register(imu, ICM_USER_CTRL, 0x00U) ||
      !write_register(imu, ICM_INT_PIN_CFG, 0x02U))
  {
    return false;
  }

  /* Bypass exposes the internal AK09916 at 0x0c on the shared host I2C bus. */
  HAL_Delay(10U);
  imu->mag_available = configure_magnetometer(imu);
  return true;
}

bool Icm20948_CalibrateStationary(Icm20948 *imu, uint16_t sample_count,
                                  uint32_t sample_period_ms)
{
  int16_t accel[3];
  int16_t gyro[3];
  int64_t accel_sum[3] = {0, 0, 0};
  int64_t gyro_sum[3] = {0, 0, 0};
  uint32_t attempts = 0U;
  uint32_t successful_samples = 0U;

  if (sample_count == 0U)
  {
    return false;
  }

  while (successful_samples < sample_count &&
         attempts < (uint32_t)sample_count * 2U)
  {
    attempts++;
    if (read_motion_raw(imu, accel, gyro))
    {
      for (uint32_t axis = 0U; axis < 3U; axis++)
      {
        accel_sum[axis] += accel[axis];
        gyro_sum[axis] += gyro[axis];
      }
      successful_samples++;
    }
    HAL_Delay(sample_period_ms);
  }
  if (successful_samples != sample_count)
  {
    return false;
  }

  for (uint32_t axis = 0U; axis < 3U; axis++)
  {
    float expected_gravity_g = axis == 2U ? 1.0f : 0.0f;
    /* Startup assumes the documented body +Z axis is held level and up. */
    imu->accel_bias_g[axis] =
        (float)accel_sum[axis] / ((float)sample_count * ACCEL_LSB_PER_G) -
        expected_gravity_g;
    imu->gyro_bias_dps[axis] =
        (float)gyro_sum[axis] / ((float)sample_count * GYRO_LSB_PER_DPS);
  }
  return true;
}

bool Icm20948_Read(Icm20948 *imu, Icm20948Sample *sample)
{
  int16_t accel[3];
  int16_t gyro[3];
  uint8_t mag_data[9];

  if (!read_motion_raw(imu, accel, gyro))
  {
    return false;
  }

  for (uint32_t axis = 0U; axis < 3U; axis++)
  {
    sample->accel_g[axis] =
        (float)accel[axis] / ACCEL_LSB_PER_G - imu->accel_bias_g[axis];
    sample->gyro_dps[axis] =
        (float)gyro[axis] / GYRO_LSB_PER_DPS - imu->gyro_bias_dps[axis];
  }

  sample->mag_valid = false;
  memset(sample->mag_uT, 0, sizeof(sample->mag_uT));
  if (imu->mag_available &&
      HAL_I2C_Mem_Read(imu->i2c, AK09916_ADDRESS, AK_ST1,
                       I2C_MEMADD_SIZE_8BIT, mag_data, sizeof(mag_data),
                       I2C_TIMEOUT_MS) == HAL_OK &&
      (mag_data[0] & 0x01U) != 0U && (mag_data[8] & 0x08U) == 0U)
  {
    float raw_mag_uT[3] = {
        little_endian_i16(&mag_data[1]) * MAG_UT_PER_LSB,
        little_endian_i16(&mag_data[3]) * MAG_UT_PER_LSB,
        little_endian_i16(&mag_data[5]) * MAG_UT_PER_LSB};

    /* AK +Y/+Z oppose accel/gyro +Y/+Z inside the ICM-20948 package. */
    sample->mag_uT[0] = (raw_mag_uT[0] - mag_bias_uT[0]) * mag_scale[0];
    sample->mag_uT[1] = (-raw_mag_uT[1] - mag_bias_uT[1]) * mag_scale[1];
    sample->mag_uT[2] = (-raw_mag_uT[2] - mag_bias_uT[2]) * mag_scale[2];
    float field_squared = sample->mag_uT[0] * sample->mag_uT[0] +
                          sample->mag_uT[1] * sample->mag_uT[1] +
                          sample->mag_uT[2] * sample->mag_uT[2];
    /* Reject gross monitor/display magnetic interference before yaw correction. */
    sample->mag_valid =
        field_squared >= MAG_MIN_FIELD_UT * MAG_MIN_FIELD_UT &&
        field_squared <= MAG_MAX_FIELD_UT * MAG_MAX_FIELD_UT;
  }
  return true;
}
