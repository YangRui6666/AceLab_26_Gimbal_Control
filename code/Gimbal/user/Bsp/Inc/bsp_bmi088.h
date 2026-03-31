//
// Created by CORE on 2026/3/14.
//

#ifndef VISION_F405_BSP_BMI088_H
#define VISION_F405_BSP_BMI088_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    int16_t accel_x;    // 加速度计 X 轴原始数据
    int16_t accel_y;    // 加速度计 Y 轴原始数据
    int16_t accel_z;    // 加速度计 Z 轴原始数据
    int16_t gyro_x;     // 陀螺仪 X 轴原始数据
    int16_t gyro_y;     // 陀螺仪 Y 轴原始数据
    int16_t gyro_z;     // 陀螺仪 Z 轴原始数据
    uint32_t tick;      // 最近一次成功读取的时间戳
} bmi088_data_t;

/**
 * @brief       BMI088 初始化
 * @retval      true 初始化成功
 * @retval      false 初始化失败
 */
bool bsp_imu_init(void);

/**
 * @brief       读取 BMI088 六轴原始数据
 * @param[out]  imu_data 六轴原始数据输出结构体
 * @retval      true 读取成功
 * @retval      false 读取失败
 */
bool bsp_imu_get(bmi088_data_t *imu_data);

/**
 * @brief       检查 BMI088 最近一次读取是否超时
 * @retval      true 设备在线
 * @retval      false 设备离线或未初始化
 */
bool bsp_imu_check(void);

#endif //VISION_F405_BSP_BMI088_H
