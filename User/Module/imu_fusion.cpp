//
// Created by CORE on 2026/4/9.
//

#include "imu_fusion.h"

#include "MahonyAHRS.h"
#include "bsp_imu.h"

#include <math.h>
#include <stddef.h>

namespace {

constexpr float k_gravity_mps2 = 9.80665f;
constexpr float k_acc_lsb_per_g_12g = 2730.0f;
constexpr float k_gyro_lsb_per_dps_1000 = 32.768f;
constexpr float k_deg_to_rad = 0.01745329252f;
constexpr float k_gyro_static_threshold_dps = 3.0f;
constexpr float k_acc_norm_tolerance_mps2 = 1.5f;
constexpr unsigned int k_bias_sample_target = 1000U;

imu_data_t g_imu_data = {0.0f, 0.0f, 0.0f};
bool s_imu_online = false;
bool s_bias_ready = false;
unsigned int s_bias_sample_count = 0U;
float s_gyro_bias_sum[3] = {0.0f, 0.0f, 0.0f};
float s_gyro_bias[3] = {0.0f, 0.0f, 0.0f};
mahony_ahrs_t s_mahony;

void imu_fusion_reset_bias()
{
    s_bias_ready = false;
    s_bias_sample_count = 0U;
    s_gyro_bias_sum[0] = 0.0f;
    s_gyro_bias_sum[1] = 0.0f;
    s_gyro_bias_sum[2] = 0.0f;
    s_gyro_bias[0] = 0.0f;
    s_gyro_bias[1] = 0.0f;
    s_gyro_bias[2] = 0.0f;
}

bool imu_fusion_sample_is_static(const float gyro_dps[3], const float acc_mps2[3])
{
    const float acc_norm = sqrtf(acc_mps2[0] * acc_mps2[0] +
                                 acc_mps2[1] * acc_mps2[1] +
                                 acc_mps2[2] * acc_mps2[2]);

    return (fabsf(gyro_dps[0]) < k_gyro_static_threshold_dps) &&
           (fabsf(gyro_dps[1]) < k_gyro_static_threshold_dps) &&
           (fabsf(gyro_dps[2]) < k_gyro_static_threshold_dps) &&
           (fabsf(acc_norm - k_gravity_mps2) < k_acc_norm_tolerance_mps2);
}

void imu_fusion_publish_zero()
{
    g_imu_data.yaw = 0.0f;
    g_imu_data.pitch = 0.0f;
    g_imu_data.roll = 0.0f;
}

} // namespace

void imu_init(void)
{
    imu_fusion_publish_zero();
    imu_fusion_reset_bias();
    mahony_ahrs_init(&s_mahony);
    s_imu_online = bsp_imu_init();
}

void imu_update(void)
{
    IMURawData raw = {0};
    float acc_mps2[3];
    float gyro_dps[3];
    float gyro_rad_s[3];

    if (!s_imu_online) {
        s_imu_online = bsp_imu_init();
        if (!s_imu_online) {
            return;
        }

        imu_fusion_reset_bias();
        mahony_ahrs_init(&s_mahony);
    }

    if (!bsp_imu_update(&raw)) {
        s_imu_online = imu_check();
        return;
    }

    s_imu_online = true;

    acc_mps2[0] = ((float)raw.acc_x / k_acc_lsb_per_g_12g) * k_gravity_mps2;
    acc_mps2[1] = ((float)raw.acc_y / k_acc_lsb_per_g_12g) * k_gravity_mps2;
    acc_mps2[2] = ((float)raw.acc_z / k_acc_lsb_per_g_12g) * k_gravity_mps2;

    gyro_dps[0] = (float)raw.gyro_x / k_gyro_lsb_per_dps_1000;
    gyro_dps[1] = (float)raw.gyro_y / k_gyro_lsb_per_dps_1000;
    gyro_dps[2] = (float)raw.gyro_z / k_gyro_lsb_per_dps_1000;

    if (!s_bias_ready) {
        if (imu_fusion_sample_is_static(gyro_dps, acc_mps2)) {
            s_gyro_bias_sum[0] += gyro_dps[0];
            s_gyro_bias_sum[1] += gyro_dps[1];
            s_gyro_bias_sum[2] += gyro_dps[2];
            ++s_bias_sample_count;

            if (s_bias_sample_count >= k_bias_sample_target) {
                s_gyro_bias[0] = s_gyro_bias_sum[0] / (float)s_bias_sample_count;
                s_gyro_bias[1] = s_gyro_bias_sum[1] / (float)s_bias_sample_count;
                s_gyro_bias[2] = s_gyro_bias_sum[2] / (float)s_bias_sample_count;
                s_bias_ready = true;
                mahony_ahrs_init(&s_mahony);
            }
        } else {
            imu_fusion_reset_bias();
        }

        return;
    }

    gyro_rad_s[0] = (gyro_dps[0] - s_gyro_bias[0]) * k_deg_to_rad;
    gyro_rad_s[1] = (gyro_dps[1] - s_gyro_bias[1]) * k_deg_to_rad;
    gyro_rad_s[2] = (gyro_dps[2] - s_gyro_bias[2]) * k_deg_to_rad;

    mahony_ahrs_update_imu(&s_mahony,
                           gyro_rad_s[0],
                           gyro_rad_s[1],
                           gyro_rad_s[2],
                           acc_mps2[0],
                           acc_mps2[1],
                           acc_mps2[2]);
    mahony_ahrs_get_euler_deg(&s_mahony,
                              &g_imu_data.yaw,
                              &g_imu_data.pitch,
                              &g_imu_data.roll);
}

void imu_get_data(imu_data_t *imu_data)
{
    if (imu_data == nullptr) {
        return;
    }

    *imu_data = g_imu_data;
}
