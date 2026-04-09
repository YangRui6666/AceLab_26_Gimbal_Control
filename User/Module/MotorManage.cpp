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
