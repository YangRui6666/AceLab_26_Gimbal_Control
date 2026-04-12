//
// Created by CORE on 2026/4/9.
//

//这里需要内部调用library里面的Mahony算法

#include "imu_fusion.h"

static imu_data_t g_imu_data = {0.0f, 0.0f, 0.0f};

void imu_init(void)
{
    g_imu_data.yaw = 0.0f;
    g_imu_data.pitch = 0.0f;
    g_imu_data.roll = 0.0f;
}

void imu_update(void)
{
    // TODO: 接入实际 IMU 采样和融合算法
}

void imu_get_data(imu_data_t *imu_data)
{
    if (imu_data == 0) {
        return;
    }

    *imu_data = g_imu_data;
}
