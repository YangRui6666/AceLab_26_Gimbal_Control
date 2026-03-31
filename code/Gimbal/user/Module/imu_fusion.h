//
// Created by CORE on 2026/3/14.
//

#ifndef VISION_F405_IMU_FUSION_H
#define VISION_F405_IMU_FUSION_H

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    int16_t yaw;
    int16_t pitch;
    int16_t roll;
    uint16_t tick;
} imu_data;

/**
 * @brief 初始化 IMU 和卡尔曼滤波器
 * @return true 初始化成功
 * @return false 初始化失败
 */
bool module_imu_init(void);

/**
 * @brief 读取 IMU 数据并运行卡尔曼滤波
 * @param imu_data 输出融合后的欧拉角数据（放大 100 倍）
 */
void module_imu_get(imu_data *imu_data);

/**
 * @brief 获取浮点数格式的欧拉角
 * @param roll 横滚角指针（度），可为 NULL
 * @param pitch 俯仰角指针（度），可为 NULL
 * @param yaw 偏航角指针（度），可为 NULL
 */
void module_imu_get_float(float *roll, float *pitch, float *yaw);

#endif //VISION_F405_IMU_FUSION_H