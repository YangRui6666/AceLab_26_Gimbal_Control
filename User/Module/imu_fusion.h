//
// Created by CORE on 2026/4/10.
//

#ifndef GIMBAL_UM_IMU_FUSION_H
#define GIMBAL_UM_IMU_FUSION_H

typedef struct imu_data_t {
    float yaw;    // 单位：deg
    float pitch;  // 单位：deg
    float roll;   // 单位：deg
} imu_data_t;

#ifdef __cplusplus
extern "C" {
#endif

void imu_init(void);
void imu_update(void);
void imu_get_data(imu_data_t *imu_data);
bool imu_check();

#ifdef __cplusplus
}
#endif

#endif //GIMBAL_UM_IMU_FUSION_H
