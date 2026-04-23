#include "MotorManage.hpp"

#include "gimbal_config.h"
#include "motor_manage_c.h"
#include "stm32f4xx_hal.h"

extern "C" {
PIDParam_t g_yaw_pos_pid_param = {
    YAW_POS_KP_DEFAULT,
    YAW_POS_KI_DEFAULT,
    YAW_POS_KD_DEFAULT,
    YAW_POS_OUT_MIN_DEFAULT,
    YAW_POS_OUT_MAX_DEFAULT,
};

PIDParam_t g_yaw_spd_pid_param = {
    YAW_SPD_KP_DEFAULT,
    YAW_SPD_KI_DEFAULT,
    YAW_SPD_KD_DEFAULT,
    YAW_SPD_OUT_MIN_DEFAULT,
    YAW_SPD_OUT_MAX_DEFAULT,
};

PIDParam_t g_pitch_pos_pid_param = {
    PITCH_POS_KP_DEFAULT,
    PITCH_POS_KI_DEFAULT,
    PITCH_POS_KD_DEFAULT,
    PITCH_POS_OUT_MIN_DEFAULT,
    PITCH_POS_OUT_MAX_DEFAULT,
};

PIDParam_t g_pitch_spd_pid_param = {
    PITCH_SPD_KP_DEFAULT,
    PITCH_SPD_KI_DEFAULT,
    PITCH_SPD_KD_DEFAULT,
    PITCH_SPD_OUT_MIN_DEFAULT,
    PITCH_SPD_OUT_MAX_DEFAULT,
};
}

static MotorManage s_motor_manage;

extern "C" bool motor_manage_init(void)
{
    return s_motor_manage.init();
}

extern "C" void motor_manage_update_feedback(void)
{
    s_motor_manage.update_feedback(HAL_GetTick());
}

extern "C" void motor_manage_set(float yaw_joint_deg, float pitch_joint_deg)
{
    s_motor_manage.set(yaw_joint_deg, pitch_joint_deg);
}

extern "C" void motor_manage_lock(void)
{
    s_motor_manage.lock();
}

extern "C" void motor_manage_disable(void)
{
    s_motor_manage.disable();
}

extern "C" uint32_t motor_manage_check(uint32_t now_ms)
{
    return s_motor_manage.check(now_ms);
}
