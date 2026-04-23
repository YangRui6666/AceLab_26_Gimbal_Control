#ifndef USER_CPP_MOTORMANAGE_HPP
#define USER_CPP_MOTORMANAGE_HPP

#include "GM6020.hpp"
#include "PID.hpp"
#include <stdint.h>

class MotorManage {
public:
    MotorManage();

    bool init();
    void update_feedback(uint32_t now_ms);
    void set(float yaw_target_deg, float pitch_target_deg);
    void lock();
    void disable();
    uint32_t check(uint32_t now_ms) const;

private:
    void sync_pid_params();
    void reset_controllers();
    void send_zero_current() const;

    GM6020 yaw_;
    GM6020 pitch_;
    PID yaw_pid_speed_;
    PID yaw_pid_location_;
    PID pitch_pid_speed_;
    PID pitch_pid_location_;
    bool initialized_;
    bool locked_;
    bool disabled_;
};

#endif
