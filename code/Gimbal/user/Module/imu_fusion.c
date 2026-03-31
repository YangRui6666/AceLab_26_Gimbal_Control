//
// Created by CORE on 2026/3/12.
//

#include "imu_fusion.h"

#include <stddef.h>

#include "stm32f4xx_hal.h"
#include "../Bsp/Inc/bsp_bmi088.h"
#include "../Library/Kalman.h"

// 物理单位转换系数
#define ACC_SCALE_3G  (3.0f * 9.80665f / 32768.0f)    // ±3g -> m/s²
#define GYRO_SCALE_2000DPS  (2000.0f / 32768.0f)      // ±2000DPS -> deg/s

static bmi088_data_t bsp_raw_data = {0};
static BMI088_ReceiveDataTypedef kalman_input = {0};
static imu_data fused_output = {0};
static bool initialized = false;

/**
 * @brief 将 bsp_bmi088 原始数据转换为卡尔曼滤波输入格式
 */
static void convert_to_kalman_format(bmi088_data_t *raw, BMI088_ReceiveDataTypedef *kalman)
{
    kalman->acc_x = raw->accel_x * ACC_SCALE_3G;
    kalman->acc_y = raw->accel_y * ACC_SCALE_3G;
    kalman->acc_z = raw->accel_z * ACC_SCALE_3G;

    kalman->gyro_x = raw->gyro_x * GYRO_SCALE_2000DPS;
    kalman->gyro_y = raw->gyro_y * GYRO_SCALE_2000DPS;
    kalman->gyro_z = raw->gyro_z * GYRO_SCALE_2000DPS;

    kalman->x_error = 0.0f;
    kalman->y_error = 0.0f;
    kalman->z_error = 0.0f;
}

/**
 * @brief 初始化 IMU 和卡尔曼滤波
 * @return true 初始化成功
 * @return false 初始化失败
 */
bool module_imu_init(void)
{
    if (initialized)
    {
        return true;  // 已经初始化过
    }

    if (bsp_imu_init())
    {
        initialize_BMI088_and_kalman(&KalmanX, &KalmanY, &KalmanZ);
        initialized = true;
        return true;
    }
    else
    {
        return false;  // BMI088初始化失败
    }
}

/**
 * @brief 读取 IMU 数据并运行卡尔曼滤波
 * @param imu_data 输出融合后的欧拉角数据
 */
void module_imu_get(imu_data *imu_data)
{

    if (!initialized || imu_data == NULL)
    {
        return;
    }

    if (bsp_imu_get(&bsp_raw_data))
    {
        convert_to_kalman_format(&bsp_raw_data, &kalman_input);

        BMI088_Kalman_Filter(&KalmanX, &KalmanY, &KalmanZ, &kalman_input);

        fused_output.roll = (int16_t)(kalman_input.roll * 100.0f);
        fused_output.pitch = (int16_t)(kalman_input.pitch * 100.0f);
        fused_output.yaw = (int16_t)(kalman_input.yaw * 100.0f);
        fused_output.tick = HAL_GetTick();

        *imu_data = fused_output;
    }
}

/**
 * @brief 获取浮点数格式的欧拉角
 */
void module_imu_get_float(float *roll, float *pitch, float *yaw)
{
    if (!initialized)
    {
        return;
    }

    if (bsp_imu_get(&bsp_raw_data))
    {
        convert_to_kalman_format(&bsp_raw_data, &kalman_input);
        BMI088_Kalman_Filter(&KalmanX, &KalmanY, &KalmanZ, &kalman_input);

        if (roll) *roll = kalman_input.roll;
        if (pitch) *pitch = kalman_input.pitch;
        if (yaw) *yaw = kalman_input.yaw;
    }
}