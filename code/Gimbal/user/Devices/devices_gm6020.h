#ifndef GIMBAL_DEVICES_GM6020_H
#define GIMBAL_DEVICES_GM6020_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float position_deg;
    float velocity_rpm;
    float current_ma;
    float filtered_current_ma;
    int8_t temp;
    uint32_t last_rx_tick;
    bool online;
} devices_gm6020_feedback_t;

void devices_gimbal_poll(void);
bool devices_gimbal_get_yaw_feedback(devices_gm6020_feedback_t* out);
bool devices_gimbal_get_pitch_feedback(devices_gm6020_feedback_t* out);
void devices_gimbal_set_currents(int16_t yaw_current, int16_t pitch_current);
bool devices_gimbal_send(void);
void devices_gimbal_stop(void);

#ifdef __cplusplus
}
#endif

#endif // GIMBAL_DEVICES_GM6020_H
