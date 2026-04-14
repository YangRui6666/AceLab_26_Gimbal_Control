#include "MahonyAHRS.h"

#include "cmsis_os2.h"

#include <math.h>
#include <string.h>

#define MAHONY_TWO_KP        (2.0f * 0.5f)
#define MAHONY_TWO_KI        (2.0f * 0.0f)
#define MAHONY_DT_MAX_S      0.02f
#define MAHONY_RAD_TO_DEG    57.2957795131f

static float mahony_ahrs_inv_sqrt(float x);
static float mahony_ahrs_clamp(float value, float min_value, float max_value);

void mahony_ahrs_init(mahony_ahrs_t *state)
{
    if (state == NULL) {
        return;
    }

    memset(state, 0, sizeof(*state));
    state->q0 = 1.0f;
}

void mahony_ahrs_update_imu(mahony_ahrs_t *state,
                            float gx_rad_s,
                            float gy_rad_s,
                            float gz_rad_s,
                            float ax_mps2,
                            float ay_mps2,
                            float az_mps2)
{
    float dt_s;
    float recip_norm;
    float halfvx;
    float halfvy;
    float halfvz;
    float halfex;
    float halfey;
    float halfez;
    float qa;
    float qb;
    float qc;
    uint32_t now_tick;
    uint32_t tick_freq;
    uint32_t delta_tick;

    if (state == NULL) {
        return;
    }

    now_tick = osKernelGetTickCount();
    tick_freq = osKernelGetTickFreq();
    if (tick_freq == 0U) {
        tick_freq = 1000U;
    }

    if (!state->has_time_base) {
        state->last_tick = now_tick;
        state->has_time_base = true;
        return;
    }

    delta_tick = now_tick - state->last_tick;
    state->last_tick = now_tick;
    if (delta_tick == 0U) {
        return;
    }

    dt_s = (float)delta_tick / (float)tick_freq;
    if (dt_s > MAHONY_DT_MAX_S) {
        dt_s = MAHONY_DT_MAX_S;
    }

    if (!((ax_mps2 == 0.0f) && (ay_mps2 == 0.0f) && (az_mps2 == 0.0f))) {
        recip_norm = mahony_ahrs_inv_sqrt(ax_mps2 * ax_mps2 +
                                          ay_mps2 * ay_mps2 +
                                          az_mps2 * az_mps2);
        ax_mps2 *= recip_norm;
        ay_mps2 *= recip_norm;
        az_mps2 *= recip_norm;

        halfvx = state->q1 * state->q3 - state->q0 * state->q2;
        halfvy = state->q0 * state->q1 + state->q2 * state->q3;
        halfvz = state->q0 * state->q0 - 0.5f + state->q3 * state->q3;

        halfex = (ay_mps2 * halfvz - az_mps2 * halfvy);
        halfey = (az_mps2 * halfvx - ax_mps2 * halfvz);
        halfez = (ax_mps2 * halfvy - ay_mps2 * halfvx);

        if (MAHONY_TWO_KI > 0.0f) {
            state->integral_fb_x += MAHONY_TWO_KI * halfex * dt_s;
            state->integral_fb_y += MAHONY_TWO_KI * halfey * dt_s;
            state->integral_fb_z += MAHONY_TWO_KI * halfez * dt_s;
            gx_rad_s += state->integral_fb_x;
            gy_rad_s += state->integral_fb_y;
            gz_rad_s += state->integral_fb_z;
        } else {
            state->integral_fb_x = 0.0f;
            state->integral_fb_y = 0.0f;
            state->integral_fb_z = 0.0f;
        }

        gx_rad_s += MAHONY_TWO_KP * halfex;
        gy_rad_s += MAHONY_TWO_KP * halfey;
        gz_rad_s += MAHONY_TWO_KP * halfez;
    }

    gx_rad_s *= 0.5f * dt_s;
    gy_rad_s *= 0.5f * dt_s;
    gz_rad_s *= 0.5f * dt_s;

    qa = state->q0;
    qb = state->q1;
    qc = state->q2;

    state->q0 += (-qb * gx_rad_s - qc * gy_rad_s - state->q3 * gz_rad_s);
    state->q1 += (qa * gx_rad_s + qc * gz_rad_s - state->q3 * gy_rad_s);
    state->q2 += (qa * gy_rad_s - qb * gz_rad_s + state->q3 * gx_rad_s);
    state->q3 += (qa * gz_rad_s + qb * gy_rad_s - qc * gx_rad_s);

    recip_norm = mahony_ahrs_inv_sqrt(state->q0 * state->q0 +
                                      state->q1 * state->q1 +
                                      state->q2 * state->q2 +
                                      state->q3 * state->q3);
    state->q0 *= recip_norm;
    state->q1 *= recip_norm;
    state->q2 *= recip_norm;
    state->q3 *= recip_norm;
}

void mahony_ahrs_get_euler_deg(const mahony_ahrs_t *state,
                               float *yaw_deg,
                               float *pitch_deg,
                               float *roll_deg)
{
    float sin_pitch;

    if (state == NULL) {
        return;
    }

    sin_pitch = 2.0f * (state->q0 * state->q2 - state->q3 * state->q1);
    sin_pitch = mahony_ahrs_clamp(sin_pitch, -1.0f, 1.0f);

    if (yaw_deg != NULL) {
        *yaw_deg = atan2f(2.0f * (state->q0 * state->q3 + state->q1 * state->q2),
                          1.0f - 2.0f * (state->q2 * state->q2 + state->q3 * state->q3)) * MAHONY_RAD_TO_DEG;
    }

    if (pitch_deg != NULL) {
        *pitch_deg = asinf(sin_pitch) * MAHONY_RAD_TO_DEG;
    }

    if (roll_deg != NULL) {
        *roll_deg = atan2f(2.0f * (state->q0 * state->q1 + state->q2 * state->q3),
                           1.0f - 2.0f * (state->q1 * state->q1 + state->q2 * state->q2)) * MAHONY_RAD_TO_DEG;
    }
}

static float mahony_ahrs_inv_sqrt(float x)
{
    return 1.0f / sqrtf(x);
}

static float mahony_ahrs_clamp(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}
