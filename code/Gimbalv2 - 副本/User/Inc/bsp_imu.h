#ifndef BSP_IMU_H
#define BSP_IMU_H

#include "gimbal_types.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

bool bsp_imu_init(void);
bool bsp_imu_update(IMURawData *raw);
bool imu_check(void);

#ifdef __cplusplus
}
#endif

#endif
