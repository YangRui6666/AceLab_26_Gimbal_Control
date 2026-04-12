//
// Created by CORE on 2026/4/9.
//


#include "device_gm6020.h"
#include "PID.h"
#include "MotorManage.h"

MotorManage::MotorManage()
: yaw_(0x206, 1000, 1000, -1000),
  pitch_(0x208, 1000, 1000, -1000)
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

    int16_t current_pack[4] = {0};
    current_pack[1] = yaw_.get_target().target_current;
    current_pack[3] = pitch_.get_target().target_current;
    bsp_tx(0xFE, (uint8_t*)current_pack, 8);
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


}

void MotorManage::lock()
{
    //这里让电机电流值清零，发送0电流值

    int16_t current[4] = {0};
    bsp_tx(0xFE, (uint8_t*)current, 8);
}

GM6020::Target MotorManage::get_yaw_target() const
{
    return yaw_.get_target();
}

GM6020::Target MotorManage::get_pitch_target() const
{
    return pitch_.get_target();
}
