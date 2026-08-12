#include "pose_fusion.h"

#include <math.h>
#include <string.h>

#define DEG_TO_RAD                    0.01745329251994329577f
#define GRAVITY_MPS2                  9.80665f
#define MAHONY_TWO_KP                 1.0f
#define FUSION_ACCEL_MIN_G            0.80f
#define FUSION_ACCEL_MAX_G            1.20f
#define STATIONARY_GYRO_DPS           1.5f
#define STATIONARY_LINEAR_ACCEL_MPS2  0.25f
#define STATIONARY_SAMPLE_COUNT       30U
/* Only very slow stationary residuals are bias; ordinary head motion is not. */
#define GYRO_BIAS_LEARN_MAX_DPS        0.25f
#define GYRO_BIAS_LEARN_PER_SECOND     0.50f
#define LINEAR_ACCEL_DEADBAND_MPS2    0.12f
#define VELOCITY_DAMPING_PER_SECOND   1.0f
#define MAX_DEMO_VELOCITY_MPS         3.0f
#define MAX_DEMO_POSITION_M           3.0f

static float clampf(float value, float minimum, float maximum)
{
  return fminf(fmaxf(value, minimum), maximum);
}

static bool normalize3(float vector[3])
{
  float norm = sqrtf(vector[0] * vector[0] + vector[1] * vector[1] +
                     vector[2] * vector[2]);

  if (norm < 1.0e-6f)
  {
    return false;
  }

  vector[0] /= norm;
  vector[1] /= norm;
  vector[2] /= norm;
  return true;
}

static void normalize4(float quaternion[4])
{
  float norm = sqrtf(quaternion[0] * quaternion[0] +
                     quaternion[1] * quaternion[1] +
                     quaternion[2] * quaternion[2] +
                     quaternion[3] * quaternion[3]);

  if (norm < 1.0e-6f)
  {
    quaternion[0] = 1.0f;
    quaternion[1] = 0.0f;
    quaternion[2] = 0.0f;
    quaternion[3] = 0.0f;
    return;
  }

  quaternion[0] /= norm;
  quaternion[1] /= norm;
  quaternion[2] /= norm;
  quaternion[3] /= norm;
}

static void cross3(const float left[3], const float right[3], float result[3])
{
  result[0] = left[1] * right[2] - left[2] * right[1];
  result[1] = left[2] * right[0] - left[0] * right[2];
  result[2] = left[0] * right[1] - left[1] * right[0];
}

static void initial_quaternion(const float accel_g[3], const float mag_uT[3],
                               bool mag_valid, float quaternion[4])
{
  float up[3] = {accel_g[0], accel_g[1], accel_g[2]};
  float north[3];
  float left[3];
  float dot;
  float matrix[3][3];
  float trace;

  if (!normalize3(up))
  {
    up[0] = 0.0f;
    up[1] = 0.0f;
    up[2] = 1.0f;
  }

  if (mag_valid)
  {
    dot = mag_uT[0] * up[0] + mag_uT[1] * up[1] + mag_uT[2] * up[2];
    north[0] = mag_uT[0] - dot * up[0];
    north[1] = mag_uT[1] - dot * up[1];
    north[2] = mag_uT[2] - dot * up[2];
  }
  else
  {
    dot = up[0];
    north[0] = 1.0f - dot * up[0];
    north[1] = -dot * up[1];
    north[2] = -dot * up[2];
  }

  if (!normalize3(north))
  {
    uint32_t axis = 0U;
    if (fabsf(up[1]) < fabsf(up[axis]))
    {
      axis = 1U;
    }
    if (fabsf(up[2]) < fabsf(up[axis]))
    {
      axis = 2U;
    }
    memset(north, 0, sizeof(north));
    north[axis] = 1.0f;
    dot = up[axis];
    north[0] -= dot * up[0];
    north[1] -= dot * up[1];
    north[2] -= dot * up[2];
    normalize3(north);
  }
  cross3(up, north, left);
  normalize3(left);

  /* Rows are the startup-world axes expressed in the sensor body frame. */
  memcpy(matrix[0], north, sizeof(north));
  memcpy(matrix[1], left, sizeof(left));
  memcpy(matrix[2], up, sizeof(up));

  trace = matrix[0][0] + matrix[1][1] + matrix[2][2];
  if (trace > 0.0f)
  {
    float scale = sqrtf(trace + 1.0f) * 2.0f;
    quaternion[0] = 0.25f * scale;
    quaternion[1] = (matrix[2][1] - matrix[1][2]) / scale;
    quaternion[2] = (matrix[0][2] - matrix[2][0]) / scale;
    quaternion[3] = (matrix[1][0] - matrix[0][1]) / scale;
  }
  else if ((matrix[0][0] > matrix[1][1]) && (matrix[0][0] > matrix[2][2]))
  {
    float scale = sqrtf(1.0f + matrix[0][0] - matrix[1][1] - matrix[2][2]) * 2.0f;
    quaternion[0] = (matrix[2][1] - matrix[1][2]) / scale;
    quaternion[1] = 0.25f * scale;
    quaternion[2] = (matrix[0][1] + matrix[1][0]) / scale;
    quaternion[3] = (matrix[0][2] + matrix[2][0]) / scale;
  }
  else if (matrix[1][1] > matrix[2][2])
  {
    float scale = sqrtf(1.0f + matrix[1][1] - matrix[0][0] - matrix[2][2]) * 2.0f;
    quaternion[0] = (matrix[0][2] - matrix[2][0]) / scale;
    quaternion[1] = (matrix[0][1] + matrix[1][0]) / scale;
    quaternion[2] = 0.25f * scale;
    quaternion[3] = (matrix[1][2] + matrix[2][1]) / scale;
  }
  else
  {
    float scale = sqrtf(1.0f + matrix[2][2] - matrix[0][0] - matrix[1][1]) * 2.0f;
    quaternion[0] = (matrix[1][0] - matrix[0][1]) / scale;
    quaternion[1] = (matrix[0][2] + matrix[2][0]) / scale;
    quaternion[2] = (matrix[1][2] + matrix[2][1]) / scale;
    quaternion[3] = 0.25f * scale;
  }
  normalize4(quaternion);
}

static void relative_quaternion(const PoseFusion *fusion, float result[4])
{
  const float *reference = fusion->reference;
  const float *current = fusion->quaternion;

  result[0] = reference[0] * current[0] + reference[1] * current[1] +
              reference[2] * current[2] + reference[3] * current[3];
  result[1] = reference[0] * current[1] - reference[1] * current[0] -
              reference[2] * current[3] + reference[3] * current[2];
  result[2] = reference[0] * current[2] + reference[1] * current[3] -
              reference[2] * current[0] - reference[3] * current[1];
  result[3] = reference[0] * current[3] - reference[1] * current[2] +
              reference[2] * current[1] - reference[3] * current[0];
  normalize4(result);
}

static void rotate_vector(const float quaternion[4], const float vector[3],
                          float result[3])
{
  float tx = 2.0f * (quaternion[2] * vector[2] - quaternion[3] * vector[1]);
  float ty = 2.0f * (quaternion[3] * vector[0] - quaternion[1] * vector[2]);
  float tz = 2.0f * (quaternion[1] * vector[1] - quaternion[2] * vector[0]);

  result[0] = vector[0] + quaternion[0] * tx +
              (quaternion[2] * tz - quaternion[3] * ty);
  result[1] = vector[1] + quaternion[0] * ty +
              (quaternion[3] * tx - quaternion[1] * tz);
  result[2] = vector[2] + quaternion[0] * tz +
              (quaternion[1] * ty - quaternion[2] * tx);
}

static void mahony_update(float quaternion[4], const float accel_g[3],
                          const float gyro_dps[3], const float mag_uT[3],
                          bool mag_valid, float dt_s)
{
  float accel[3] = {accel_g[0], accel_g[1], accel_g[2]};
  float mag[3] = {mag_uT[0], mag_uT[1], mag_uT[2]};
  float gx = gyro_dps[0] * DEG_TO_RAD;
  float gy = gyro_dps[1] * DEG_TO_RAD;
  float gz = gyro_dps[2] * DEG_TO_RAD;
  float q0 = quaternion[0];
  float q1 = quaternion[1];
  float q2 = quaternion[2];
  float q3 = quaternion[3];
  float halfvx;
  float halfvy;
  float halfvz;
  float halfex;
  float halfey;
  float halfez;
  float accel_norm_squared = accel[0] * accel[0] + accel[1] * accel[1] +
                             accel[2] * accel[2];

  /* Linear acceleration is not gravity; reject it before tilt correction. */
  if (accel_norm_squared >= FUSION_ACCEL_MIN_G * FUSION_ACCEL_MIN_G &&
      accel_norm_squared <= FUSION_ACCEL_MAX_G * FUSION_ACCEL_MAX_G &&
      normalize3(accel))
  {
    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;
    halfex = accel[1] * halfvz - accel[2] * halfvy;
    halfey = accel[2] * halfvx - accel[0] * halfvz;
    halfez = accel[0] * halfvy - accel[1] * halfvx;

    bool normalized_mag = mag_valid && normalize3(mag);
    float mag_up_dot = mag[0] * accel[0] + mag[1] * accel[1] +
                       mag[2] * accel[2];
    if (normalized_mag &&
        (1.0f - mag_up_dot * mag_up_dot) > 0.04f)
    {
      float hx = 2.0f * (mag[0] * (0.5f - q2 * q2 - q3 * q3) +
                         mag[1] * (q1 * q2 - q0 * q3) +
                         mag[2] * (q1 * q3 + q0 * q2));
      float hy = 2.0f * (mag[0] * (q1 * q2 + q0 * q3) +
                         mag[1] * (0.5f - q1 * q1 - q3 * q3) +
                         mag[2] * (q2 * q3 - q0 * q1));
      float bx = sqrtf(hx * hx + hy * hy);
      float bz = 2.0f * (mag[0] * (q1 * q3 - q0 * q2) +
                         mag[1] * (q2 * q3 + q0 * q1) +
                         mag[2] * (0.5f - q1 * q1 - q2 * q2));
      float halfwx = bx * (0.5f - q2 * q2 - q3 * q3) +
                     bz * (q1 * q3 - q0 * q2);
      float halfwy = bx * (q1 * q2 - q0 * q3) +
                     bz * (q0 * q1 + q2 * q3);
      float halfwz = bx * (q0 * q2 + q1 * q3) +
                     bz * (0.5f - q1 * q1 - q2 * q2);

      halfex += mag[1] * halfwz - mag[2] * halfwy;
      halfey += mag[2] * halfwx - mag[0] * halfwz;
      halfez += mag[0] * halfwy - mag[1] * halfwx;
    }

    gx += MAHONY_TWO_KP * halfex;
    gy += MAHONY_TWO_KP * halfey;
    gz += MAHONY_TWO_KP * halfez;
  }

  gx *= 0.5f * dt_s;
  gy *= 0.5f * dt_s;
  gz *= 0.5f * dt_s;
  quaternion[0] += -q1 * gx - q2 * gy - q3 * gz;
  quaternion[1] += q0 * gx + q2 * gz - q3 * gy;
  quaternion[2] += q0 * gy - q1 * gz + q3 * gx;
  quaternion[3] += q0 * gz + q1 * gy - q2 * gx;
  normalize4(quaternion);
}

static float apply_deadband(float value, float deadband)
{
  if (fabsf(value) <= deadband)
  {
    return 0.0f;
  }
  return copysignf(fabsf(value) - deadband, value);
}

void PoseFusion_Init(PoseFusion *fusion, const float accel_g[3],
                     const float mag_uT[3], bool mag_valid)
{
  memset(fusion, 0, sizeof(*fusion));
  initial_quaternion(accel_g, mag_uT, mag_valid, fusion->quaternion);
  memcpy(fusion->reference, fusion->quaternion, sizeof(fusion->reference));
}

void PoseFusion_Update(PoseFusion *fusion, const float accel_g[3],
                       const float gyro_dps[3], const float mag_uT[3],
                       bool mag_valid, float dt_s)
{
  float gyro_norm;
  float linear_accel_norm;
  float relative[4];
  float accel_world_g[3];
  float linear_accel_mps2[3];
  float corrected_gyro_dps[3];
  float damping;

  if ((dt_s <= 0.0f) || (dt_s > 0.05f))
  {
    return;
  }

  for (uint32_t axis = 0U; axis < 3U; axis++)
  {
    corrected_gyro_dps[axis] =
        gyro_dps[axis] - fusion->residual_gyro_bias_dps[axis];
  }
  mahony_update(fusion->quaternion, accel_g, corrected_gyro_dps,
                mag_uT, mag_valid, dt_s);

  relative_quaternion(fusion, relative);
  rotate_vector(relative, accel_g, accel_world_g);
  accel_world_g[2] -= 1.0f;
  for (uint32_t axis = 0U; axis < 3U; axis++)
  {
    linear_accel_mps2[axis] = apply_deadband(accel_world_g[axis] * GRAVITY_MPS2,
                                             LINEAR_ACCEL_DEADBAND_MPS2);
  }

  gyro_norm = sqrtf(corrected_gyro_dps[0] * corrected_gyro_dps[0] +
                    corrected_gyro_dps[1] * corrected_gyro_dps[1] +
                    corrected_gyro_dps[2] * corrected_gyro_dps[2]);
  linear_accel_norm = sqrtf(linear_accel_mps2[0] * linear_accel_mps2[0] +
                            linear_accel_mps2[1] * linear_accel_mps2[1] +
                            linear_accel_mps2[2] * linear_accel_mps2[2]);
  if ((gyro_norm < STATIONARY_GYRO_DPS) &&
      (linear_accel_norm < STATIONARY_LINEAR_ACCEL_MPS2))
  {
    if (fusion->stationary_samples < STATIONARY_SAMPLE_COUNT)
    {
      fusion->stationary_samples++;
    }
  }
  else
  {
    fusion->stationary_samples = 0U;
  }

  if (PoseFusion_IsStationary(fusion))
  {
    if (gyro_norm < GYRO_BIAS_LEARN_MAX_DPS)
    {
      float learning = clampf(GYRO_BIAS_LEARN_PER_SECOND * dt_s, 0.0f, 1.0f);
      for (uint32_t axis = 0U; axis < 3U; axis++)
      {
        /* ponytail: learn only the small bias left after stationary boot calibration. */
        fusion->residual_gyro_bias_dps[axis] +=
            learning * corrected_gyro_dps[axis];
      }
    }
    memset(fusion->velocity_mps, 0, sizeof(fusion->velocity_mps));
    return;
  }

  damping = fmaxf(0.0f, 1.0f - VELOCITY_DAMPING_PER_SECOND * dt_s);
  for (uint32_t axis = 0U; axis < 3U; axis++)
  {
    fusion->position_m[axis] += fusion->velocity_mps[axis] * dt_s +
                                0.5f * linear_accel_mps2[axis] * dt_s * dt_s;
    fusion->velocity_mps[axis] =
        (fusion->velocity_mps[axis] + linear_accel_mps2[axis] * dt_s) * damping;
    fusion->velocity_mps[axis] = clampf(fusion->velocity_mps[axis],
                                        -MAX_DEMO_VELOCITY_MPS,
                                        MAX_DEMO_VELOCITY_MPS);
    /* ponytail: bound IMU-only drift; replace position with an external tracker for room scale. */
    fusion->position_m[axis] = clampf(fusion->position_m[axis],
                                      -MAX_DEMO_POSITION_M,
                                      MAX_DEMO_POSITION_M);
  }
}

void PoseFusion_GetOpenVrPose(const PoseFusion *fusion, float quaternion[4],
                              float position_m[3])
{
  float relative[4];

  relative_quaternion(fusion, relative);

  /* Headset +X forward, +Y left, +Z up -> OpenVR +X right, +Y up, -Z forward. */
  quaternion[0] = relative[0];
  quaternion[1] = -relative[2];
  quaternion[2] = relative[3];
  quaternion[3] = -relative[1];
  position_m[0] = -fusion->position_m[1];
  position_m[1] = fusion->position_m[2];
  position_m[2] = -fusion->position_m[0];
}

float PoseFusion_ForwardCosine(const PoseFusion *fusion)
{
  float relative[4];

  relative_quaternion(fusion, relative);
  return 1.0f - 2.0f * (relative[2] * relative[2] +
                        relative[3] * relative[3]);
}

void PoseFusion_CorrectForward(PoseFusion *fusion, float forward_m, float weight)
{
  fusion->position_m[0] += clampf(weight, 0.0f, 1.0f) *
                           (forward_m - fusion->position_m[0]);
}

bool PoseFusion_IsStationary(const PoseFusion *fusion)
{
  return fusion->stationary_samples >= STATIONARY_SAMPLE_COUNT;
}
