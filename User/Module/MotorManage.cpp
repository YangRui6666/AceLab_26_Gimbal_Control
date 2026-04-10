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
    //我们使用的是电流控制
    //这里需要处理电机组合控制的情况
    //具体而言
    int16_t current_pack[4] = {0};
    
    
}

void MotorManage::set(float yaw_target, float pitch_target)
{
    //这里先跑位置pid，得到目标速度，再跑速度pid，得到电流值，最后保存电流值，等待发送
}

void MotorManage::lock()
{
    //这里让电机电流值清零，发送0电流值
}
