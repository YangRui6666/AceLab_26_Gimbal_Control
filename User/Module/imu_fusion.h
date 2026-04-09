//
// Created by CORE on 2026/4/10.
//

#ifndef GIMBAL_UM_IMU_FUSION_H
#define GIMBAL_UM_IMU_FUSION_H

typedef struct imu_data_t
{
    float yaw;
    float pitch;
    float roll;
};

void imu_init();
void imu_update();
void imu_get_data( imu_data_t *imu_data);


#endif //GIMBAL_UM_IMU_FUSION_H