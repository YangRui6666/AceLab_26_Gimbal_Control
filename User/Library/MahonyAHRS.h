#ifndef MAHONY_AHRS_H
#define MAHONY_AHRS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    float q0;
    float q1;
    float q2;
    float q3;
    float integral_fb_x;
    float integral_fb_y;
    float integral_fb_z;
    uint32_t last_tick;
    bool has_time_base;
} mahony_ahrs_t;

void mahony_ahrs_init(mahony_ahrs_t *state);
void mahony_ahrs_update_imu(mahony_ahrs_t *state,
                            float gx_rad_s,
                            float gy_rad_s,
                            float gz_rad_s,
                            float ax_mps2,
                            float ay_mps2,
                            float az_mps2);
void mahony_ahrs_get_euler_deg(const mahony_ahrs_t *state,
                               float *yaw_deg,
                               float *pitch_deg,
                               float *roll_deg);

#ifdef __cplusplus
}
#endif

#endif
