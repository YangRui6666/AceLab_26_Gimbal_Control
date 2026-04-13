//
// Created by CORE on 2026/4/9.
//


#include "device_gm6020.h"
#include "PID.h"
#include "MotorManage.h"

#include "cmsis_os2.h"

namespace {

int16_t clamp_current_cmd(float value, int16_t limit)
{
    if (value > (float)limit) {
        return limit;
    }

    if (value < -(float)limit) {
        return static_cast<int16_t>(-limit);
    }

    if (value >= 0.0f) {
        return static_cast<int16_t>(value + 0.5f);
    }

    return static_cast<int16_t>(value - 0.5f);
}

} // namespace

#ifdef DDBUG_DATA_ON
volatile MotorManageDebugData g_motor_manage_debug = {0};
#endif

MotorManage::MotorManage()
: yaw_(0x206, 1000, 10.0f, -10.0f),
  pitch_(0x208, 1000, 10.0f, -10.0f)
{
    yaw_.init();
    pitch_.init();
    yaw_pid_location_.init(1.0f, 0.0f, 0.0f, 100.0f, 1000.0f, 0.1f);
    yaw_pid_speed_.init(0.1f, 0.0f, 0.0f, 100.0f, 1000.0f, 0.1f);
    pitch_pid_location_.init(1.0f, 0.0f, 0.0f, 100.0f, 1000.0f, 0.1f);
    pitch_pid_speed_.init(0.1f, 0.0f, 0.0f, 100.0f, 1000.0f, 0.1f);
}



void MotorManage::update_feedback()
{
    yaw_.update();
    pitch_.update();
}

/**
 * @brief       发送can命令给电机
 * 
 * @date        2026-04-10
 * @author      Rui.
 * 
 */
void MotorManage::send_can_cmd()
{
    uint8_t tx_data[8] = {0};
    const int16_t yaw_current = yaw_.get_target().target_current;
    const int16_t pitch_current = pitch_.get_target().target_current;

    tx_data[2] = (uint8_t)(yaw_current >> 8);
    tx_data[3] = (uint8_t)(yaw_current & 0xFF);
    tx_data[6] = (uint8_t)(pitch_current >> 8);
    tx_data[7] = (uint8_t)(pitch_current & 0xFF);

    bsp_tx(0xFE, tx_data, sizeof(tx_data));
}

void MotorManage::set(float yaw_target, float pitch_target)
{
    //这里先跑位置pid，得到目标速度，再跑速度pid，得到电流值，最后保存电流值，等待发送
    
    #ifdef DDEBUG_ALL_ON
    #define DDEBUG_YAW_ON
    #define DDEBUG_PITCH_ON
    #endif
    
    

    #ifdef DDEBUG_YAW_ON
    volatile static int kp_debug_yaw = 10;
    volatile static int ki_debug_yaw = 0;
    volatile static int kd_debug_yaw = 0;
    yaw_pid_location_.set_kp(kp_debug_yaw);
    yaw_pid_location_.set_ki(ki_debug_yaw);
    yaw_pid_location_.set_kd(kd_debug_yaw);
    #endif
    #ifdef DDEBUG_PITCH_ON
    volatile static int kp_debug_pitch = 10;
    volatile static int ki_debug_pitch = 0;
    volatile static int kd_debug_pitch = 0;
    pitch_pid_location_.set_kp(kp_debug_pitch);
    pitch_pid_location_.set_ki(ki_debug_pitch);
    pitch_pid_location_.set_kd(kd_debug_pitch);
#endif

    auto yaw_state = yaw_.get_state();
    auto pitch_state = pitch_.get_state();

    static uint32_t last_ticks = 0U;
    const uint32_t ticks = osKernelGetTickCount();
    float dt_s = 0.001f;

    if (last_ticks != 0U) {
        const uint32_t delta_ticks = ticks - last_ticks;
        if (delta_ticks != 0U) {
            dt_s = (float)delta_ticks / 1000.0f;
        }
    }
    last_ticks = ticks;

    const float yaw_speed_target = yaw_pid_location_.calculate(yaw_target, yaw_state.angle_deg, dt_s);
    const float yaw_current_target = yaw_pid_speed_.calculate(yaw_speed_target, yaw_state.speed_dps, dt_s);
    const int16_t yaw_current_cmd = clamp_current_cmd(yaw_current_target, 1000);
    yaw_.set_target_speed_dps(yaw_speed_target);
    yaw_.set_target_current(yaw_current_cmd);

    const float pitch_speed_target = pitch_pid_location_.calculate(pitch_target, pitch_state.angle_deg, dt_s);
    const float pitch_current_target = pitch_pid_speed_.calculate(pitch_speed_target, pitch_state.speed_dps, dt_s);
    const int16_t pitch_current_cmd = clamp_current_cmd(pitch_current_target, 1000);
    pitch_.set_target_speed_dps(pitch_speed_target);
    pitch_.set_target_current(pitch_current_cmd);

#ifdef DDBUG_DATA_ON
    g_motor_manage_debug.yaw_angle_target_deg = yaw_target;

    g_motor_manage_debug.pitch_angle_target_deg = pitch_target;

    g_motor_manage_debug.yaw_speed_target_dps = yaw_speed_target;
    g_motor_manage_debug.pitch_speed_target_dps = pitch_speed_target;

    g_motor_manage_debug.yaw_current_pid_raw = yaw_current_target;
    g_motor_manage_debug.pitch_current_pid_raw = pitch_current_target;
    g_motor_manage_debug.yaw_current_cmd = yaw_current_cmd;
    g_motor_manage_debug.pitch_current_cmd = pitch_current_cmd;

    g_motor_manage_debug.yaw_angle_meas_deg = yaw_state.angle_deg;
    g_motor_manage_debug.pitch_angle_meas_deg = pitch_state.angle_deg;
    g_motor_manage_debug.yaw_speed_meas_dps = yaw_state.speed_dps;
    g_motor_manage_debug.pitch_speed_meas_dps = pitch_state.speed_dps;
    g_motor_manage_debug.yaw_current_meas = yaw_state.current;
    g_motor_manage_debug.pitch_current_meas = pitch_state.current;

    g_motor_manage_debug.dt_s = dt_s;
    g_motor_manage_debug.tick_ms = ticks;
#endif

}

void MotorManage::lock()
{
    //这里让电机电流值清零，发送0电流值

    uint8_t tx_data[8] = {0};
    yaw_.set_target_current(0);
    pitch_.set_target_current(0);
    bsp_tx(0xFE, tx_data, sizeof(tx_data));
}

GM6020::Target MotorManage::get_yaw_target() const
{
    return yaw_.get_target();
}

GM6020::Target MotorManage::get_pitch_target() const
{
    return pitch_.get_target();
}
