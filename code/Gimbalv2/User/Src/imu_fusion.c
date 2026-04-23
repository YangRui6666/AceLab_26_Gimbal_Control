#include "imu_fusion.h"

#include "gimbal_config.h"
#include "MahonyAHRS.h"

#include <math.h>

static AttitudeAngle s_attitude;
static bool s_fusion_ok = false;

static float clampf_local(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}

bool imu_fusion_init(void)
{
    MahonyAHRSreset();
    MahonyAHRSsetSampleFreq(IMU_SAMPLE_FREQ_HZ);
    s_attitude.yaw_deg = 0.0f;
    s_attitude.pitch_deg = 0.0f;
    s_attitude.roll_deg = 0.0f;
    s_fusion_ok = true;
    return true;
}

bool imu_fusion_update(const IMURawData *raw)
{
    float sinp = 0.0f;

    if (raw == NULL) {
        s_fusion_ok = false;
        return false;
    }

    MahonyAHRSupdateIMU(raw->gyro_x_rad_s,
                        raw->gyro_y_rad_s,
                        raw->gyro_z_rad_s,
                        raw->accel_x_g,
                        raw->accel_y_g,
                        raw->accel_z_g);

    s_attitude.roll_deg = atan2f(2.0f * (q0 * q1 + q2 * q3),
                                 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 57.2957795f;

    sinp = clampf_local(2.0f * (q0 * q2 - q3 * q1), -1.0f, 1.0f);
    s_attitude.pitch_deg = asinf(sinp) * 57.2957795f;

    s_attitude.yaw_deg = atan2f(2.0f * (q0 * q3 + q1 * q2),
                                1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 57.2957795f;

    s_fusion_ok = true;
    return true;
}

AttitudeAngle imu_fusion_get_angle(void)
{
    return s_attitude;
}

bool imu_fusion_check(void)
{
    return s_fusion_ok;
}
