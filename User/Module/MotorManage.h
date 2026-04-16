//
// Created by CORE on 2026/4/9.
//

#ifndef GIMBAL_UM_MOTORMANAGE_H
#define GIMBAL_UM_MOTORMANAGE_H

#include <stdint.h>

#include "PID.h"
#include "device_gm6020.h"

#define  DDBUG_DATA_ON

#ifdef DDBUG_DATA_ON
typedef struct
{
    float yaw_angle_t;
    float pitch_angle_t;

    float yaw_speed_t;
    float pitch_speed_t;

    float yaw_current_pid;
    float pitch_current_pid;
    int16_t yaw_current_cmd;
    int16_t pitch_current_cmd;

    float yaw_angle_meas_deg;
    float pitch_angle_meas_deg;
    float yaw_speed_meas_dps;
    float pitch_speed_meas_dps;
    int16_t yaw_current_meas;
    int16_t pitch_current_meas;

    float dt_s;
    uint32_t tick_ms;
} MotorManageDebugData;

extern volatile MotorManageDebugData g_motor_manage_debug;
#endif

class MotorManage
{
public:
    MotorManage();

    void update_feedback();
    void send_can_cmd();
    void set(float yaw_target, float pitch_target);
    void lock();

    GM6020::Target get_yaw_target() const;
    GM6020::Target get_pitch_target() const;
    float get_yaw_joint_deg() const;
    float get_pitch_joint_deg() const;

    GM6020 yaw_;
    GM6020 pitch_;

    PID yaw_pid_speed_;
    PID yaw_pid_location_;
    PID pitch_pid_speed_;
    PID pitch_pid_location_;

private:
    static float clamp_target_deg(float value, float min_value, float max_value);
    float yaw_motor_to_joint_deg(float motor_angle_deg) const;
    static float pitch_motor_to_joint_deg(float motor_angle_deg);
    void hold_yaw_axis();
    void hold_pitch_axis();

    bool yaw_zero_ready_;
    float yaw_boot_zero_deg_;
    uint8_t outer_loop_divider_count_;
    uint32_t last_inner_tick_ms_;
    uint32_t last_outer_tick_ms_;
    float yaw_speed_target_cache_;
    float pitch_speed_target_cache_;
};

#endif // GIMBAL_UM_MOTORMANAGE_H
