//
// Created by CORE on 2026/4/9.
//


#include "device_gm6020.h"
#include "PID.h"
#include "MotorManage.h"



/**
 * @brief       初始化
 * 
 * @date        2026-04-10
 * @author      Rui.
 * 
 * @return true 
 * @return false 
 */
bool MotorManage::init()
{
    GM6020 yaw_motor(0x201, 1000, 1000, -1000);
    GM6020 pitch_motor(0x202, 1000, 1000, -1000);
    yaw_.init();
    pitch_.init();
    yaw_pid_location_.init(1.0f, 0.0f, 0.0f, 100.0f, 1000.0f, 0.1f);
    yaw_pid_speed_.init(0.1f, 0.0f, 0.0f, 100.0f, 1000.0f, 0.1f);
    pitch_pid_location_.init(1.0f, 0.0f, 0.0f, 100.0f, 1000.0f, 0.1f);
    pitch_pid_speed_.init(0.1f, 0.0f, 0.0f, 100.0f, 1000.0f, 0.1f);
    return true;
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
    //我们使用的是电流控制，所以canid的控制帧为0x200
    //这里需要处理电机组合控制的情况
    //具体而言
}

void MotorManage::set(float yaw_target, float pitch_target)
{
    //这里先跑位置pid，得到目标速度，再跑速度pid，得到电流值，最后保存电流值，等待发送
}

void MotorManage::lock()
{
    //这里让电机电流值清零，发送0电流值
}
