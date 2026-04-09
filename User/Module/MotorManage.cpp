//
// Created by CORE on 2026/4/9.
//


#include "device_gm6020.h"

class MotorManage {
public:
    bool init();
    void update_feedback();

    void set(float yaw_target, float pitch_target);
    void lock();

private:
    GM6020 yaw_;
    GM6020 pitch_;

    PID yaw_pid_speed_;
    PID yaw_pid_location_;
    PID pitch_pid_speed_;
    PID pitch_pid_location_;
};

bool MotorManage::init()
{
    GM6020 yaw_motor(0x201, 1000, 1000, -1000);
    GM6020 pitch_motor(0x202, 1000, 1000, -1000);
    
    return true;
}

void MotorManage::update_feedback()
{
}

void MotorManage::set(float yaw_target, float pitch_target)
{
}

void MotorManage::lock()
{
}
