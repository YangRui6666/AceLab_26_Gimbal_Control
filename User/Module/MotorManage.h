//
// Created by CORE on 2026/4/9.
//

#ifndef GIMBAL_UM_MOTORMANAGE_H
#define GIMBAL_UM_MOTORMANAGE_H

#include <stdint.h>
#include "device_gm6020.h"
#include "PID.h"

#ifdef DDBUG_DATA_ON
typedef struct
{
    // 输入目标（MotorManage::set 的入参）
    float yaw_angle_target_deg;
    float pitch_angle_target_deg;

    // 位置环输出的目标速度（PID 输出）
    float yaw_speed_target_dps;
    float pitch_speed_target_dps;

    // 速度环输出的电流（PID 输出：raw float + 最终下发 int16）
    float yaw_current_pid_raw;
    float pitch_current_pid_raw;
    int16_t yaw_current_cmd;
    int16_t pitch_current_cmd;

    // 当前反馈
    float yaw_angle_meas_deg;
    float pitch_angle_meas_deg;
    float yaw_speed_meas_dps;
    float pitch_speed_meas_dps;
    int16_t yaw_current_meas;
    int16_t pitch_current_meas;

    // 时间信息
    float dt_s;
    uint32_t tick_ms;
} MotorManageDebugData;

extern volatile MotorManageDebugData g_motor_manage_debug;
#endif

class MotorManage {
public:
    MotorManage();

    void update_feedback();
    void send_can_cmd();
    // 关节目标角，单位：deg
    void set(float yaw_target, float pitch_target);
    void lock();

    GM6020::Target get_yaw_target() const;
    GM6020::Target get_pitch_target() const;

    GM6020 yaw_;
    GM6020 pitch_;

    PID yaw_pid_speed_;
    PID yaw_pid_location_;
    PID pitch_pid_speed_;
    PID pitch_pid_location_;
};

#endif //GIMBAL_UM_MOTORMANAGE_H
