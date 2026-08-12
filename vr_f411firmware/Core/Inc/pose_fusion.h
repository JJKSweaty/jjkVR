#ifndef POSE_FUSION_H
#define POSE_FUSION_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  float quaternion[4];
  float reference[4];
  float position_m[3];
  float velocity_mps[3];
  float residual_gyro_bias_dps[3];
  uint16_t stationary_samples;
} PoseFusion;

void PoseFusion_Init(PoseFusion *fusion, const float accel_g[3],
                     const float mag_uT[3], bool mag_valid);
void PoseFusion_Update(PoseFusion *fusion, const float accel_g[3],
                       const float gyro_dps[3], const float mag_uT[3],
                       bool mag_valid, float dt_s);
void PoseFusion_GetOpenVrPose(const PoseFusion *fusion, float quaternion[4],
                              float position_m[3]);
float PoseFusion_ForwardCosine(const PoseFusion *fusion);
void PoseFusion_CorrectForward(PoseFusion *fusion, float forward_m, float weight);
bool PoseFusion_IsStationary(const PoseFusion *fusion);

#endif
