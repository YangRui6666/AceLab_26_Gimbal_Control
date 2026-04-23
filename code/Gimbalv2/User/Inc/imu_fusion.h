#ifndef IMU_FUSION_H
#define IMU_FUSION_H

#include "gimbal_types.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

bool imu_fusion_init(void);
bool imu_fusion_update(const IMURawData *raw);
AttitudeAngle imu_fusion_get_angle(void);
bool imu_fusion_check(void);

#ifdef __cplusplus
}
#endif

#endif
