#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "pose_fusion.h"

static int close_to(float value, float expected, float tolerance)
{
  return fabsf(value - expected) <= tolerance;
}

int main(void)
{
  PoseFusion fusion;
  const float accel_g[3] = {0.0f, 0.0f, 1.0f};
  const float mag_uT[3] = {25.0f, 0.0f, 0.0f};
  const float yaw_dps[3] = {0.0f, 0.0f, 90.0f};
  const float zero_accel[3] = {0.0f, 0.0f, 0.0f};
  const float zero_mag[3] = {0.0f, 0.0f, 0.0f};
  float quaternion[4];
  float position_m[3];

  PoseFusion_Init(&fusion, accel_g, mag_uT, true);
  PoseFusion_GetOpenVrPose(&fusion, quaternion, position_m);
  assert(close_to(quaternion[0], 1.0f, 0.001f));
  assert(close_to(position_m[0], 0.0f, 0.001f));
  assert(close_to(position_m[1], 0.0f, 0.001f));
  assert(close_to(position_m[2], 0.0f, 0.001f));

  for (int sample = 0; sample < 200; sample++)
  {
    PoseFusion_Update(&fusion, accel_g, yaw_dps, zero_mag, false, 0.005f);
  }
  PoseFusion_GetOpenVrPose(&fusion, quaternion, position_m);
  assert(close_to(quaternion[0], 0.7071f, 0.01f));
  assert(close_to(quaternion[2], 0.7071f, 0.01f));
  assert(close_to(PoseFusion_ForwardCosine(&fusion), 0.0f, 0.03f));

  PoseFusion_Init(&fusion, accel_g, mag_uT, true);
  const float roll_dps[3] = {90.0f, 0.0f, 0.0f};
  for (int sample = 0; sample < 200; sample++)
  {
    PoseFusion_Update(&fusion, zero_accel, roll_dps, zero_mag, false, 0.005f);
  }
  PoseFusion_GetOpenVrPose(&fusion, quaternion, position_m);
  assert(close_to(quaternion[3], -0.7071f, 0.01f));

  PoseFusion_Init(&fusion, accel_g, mag_uT, true);
  const float pitch_dps[3] = {0.0f, 90.0f, 0.0f};
  for (int sample = 0; sample < 200; sample++)
  {
    PoseFusion_Update(&fusion, zero_accel, pitch_dps, zero_mag, false, 0.005f);
  }
  PoseFusion_GetOpenVrPose(&fusion, quaternion, position_m);
  assert(close_to(quaternion[1], -0.7071f, 0.01f));

  PoseFusion_CorrectForward(&fusion, 1.0f, 1.0f);
  fusion.position_m[1] = 0.5f;
  fusion.position_m[2] = 0.25f;
  PoseFusion_GetOpenVrPose(&fusion, quaternion, position_m);
  assert(close_to(position_m[0], 0.0f, 0.001f));
  assert(close_to(position_m[1], 0.0f, 0.001f));
  assert(close_to(position_m[2], -1.0f, 0.001f));

  PoseFusion_Init(&fusion, accel_g, mag_uT, true);
  const float forward_accel_g[3] = {0.1f, 0.0f, 1.0f};
  const float backward_accel_g[3] = {-0.1f, 0.0f, 1.0f};
  const float zero_gyro[3] = {0.0f, 0.0f, 0.0f};
  for (int sample = 0; sample < 100; sample++)
  {
    PoseFusion_Update(&fusion, forward_accel_g, zero_gyro, zero_mag, false, 0.005f);
  }
  for (int sample = 0; sample < 100; sample++)
  {
    PoseFusion_Update(&fusion, backward_accel_g, zero_gyro, zero_mag, false, 0.005f);
  }
  assert(fusion.position_m[0] > 0.05f);
  assert(!PoseFusion_IsStationary(&fusion));

  PoseFusion_Init(&fusion, accel_g, mag_uT, true);
  const float left_accel_g[3] = {0.0f, 0.1f, 1.0f};
  for (int sample = 0; sample < 200; sample++)
  {
    PoseFusion_Update(&fusion, left_accel_g, zero_gyro, zero_mag, false, 0.005f);
  }
  assert(close_to(fusion.position_m[1], 0.0f, 0.0001f));
  PoseFusion_GetOpenVrPose(&fusion, quaternion, position_m);
  assert(close_to(position_m[0], 0.0f, 0.0001f));

  const float up_y[3] = {0.0f, 1.0f, 0.0f};
  const float mag_y[3] = {0.0f, 25.0f, 0.0f};
  PoseFusion_Init(&fusion, up_y, mag_y, true);
  for (int sample = 0; sample < 1000; sample++)
  {
    PoseFusion_Update(&fusion, up_y, zero_gyro, mag_y, true, 0.005f);
  }
  PoseFusion_GetOpenVrPose(&fusion, quaternion, position_m);
  assert(close_to(quaternion[0], 1.0f, 0.01f));
  assert(close_to(quaternion[1], 0.0f, 0.01f));
  assert(close_to(quaternion[2], 0.0f, 0.01f));
  assert(close_to(quaternion[3], 0.0f, 0.01f));

  const float mag_body_after_yaw[3] = {0.0f, -25.0f, 0.0f};
  PoseFusion_Init(&fusion, accel_g, mag_body_after_yaw, true);
  for (int sample = 0; sample < 1000; sample++)
  {
    PoseFusion_Update(&fusion, accel_g, zero_gyro, mag_body_after_yaw, true,
                      0.005f);
  }
  PoseFusion_GetOpenVrPose(&fusion, quaternion, position_m);
  assert(close_to(quaternion[0], 1.0f, 0.01f));
  assert(close_to(quaternion[1], 0.0f, 0.01f));
  assert(close_to(quaternion[2], 0.0f, 0.01f));
  assert(close_to(quaternion[3], 0.0f, 0.01f));

  PoseFusion_Init(&fusion, accel_g, mag_uT, true);
  const float residual_yaw_dps[3] = {0.0f, 0.0f, 0.2f};
  for (int sample = 0; sample < 400; sample++)
  {
    PoseFusion_Update(&fusion, accel_g, residual_yaw_dps, zero_mag, false,
                      0.005f);
  }
  PoseFusion_GetOpenVrPose(&fusion, quaternion, position_m);
  assert(fusion.residual_gyro_bias_dps[2] > 0.195f);
  assert(fabsf(quaternion[2]) < 0.001f);

  for (int sample = 400; sample < 4000; sample++)
  {
    PoseFusion_Update(&fusion, accel_g, residual_yaw_dps, zero_mag, false,
                      0.005f);
  }
  PoseFusion_GetOpenVrPose(&fusion, quaternion, position_m);
  assert(fabsf(quaternion[2]) < 0.02f);

  const float moving_yaw_dps[3] = {0.0f, 0.0f, 20.2f};
  float learned_bias_dps = fusion.residual_gyro_bias_dps[2];
  for (int sample = 0; sample < 200; sample++)
  {
    PoseFusion_Update(&fusion, accel_g, moving_yaw_dps, zero_mag, false,
                      0.005f);
  }
  assert(close_to(fusion.residual_gyro_bias_dps[2], learned_bias_dps,
                  0.0001f));

  puts("pose_fusion_test: passed");
  return 0;
}
