#ifndef MOTOR_MANAGE_C_H
#define MOTOR_MANAGE_C_H

#include "gimbal_types.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

extern PIDParam_t g_yaw_pos_pid_param;
extern PIDParam_t g_yaw_spd_pid_param;
extern PIDParam_t g_pitch_pos_pid_param;
extern PIDParam_t g_pitch_spd_pid_param;

bool motor_manage_init(void);
void motor_manage_update_feedback(void);
void motor_manage_set(float yaw_joint_deg, float pitch_joint_deg);
void motor_manage_lock(void);
void motor_manage_disable(void);
uint32_t motor_manage_check(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif
