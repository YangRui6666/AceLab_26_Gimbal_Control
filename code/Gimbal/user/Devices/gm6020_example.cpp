//
// Created by CORE on 2026/4/2.
//

#include "motor_manage.hpp"

void gm6020_example_usage(void)
{
    MotorManager motors;

    motors.pollCanRx();
    motors.setYawCurrent(5000);
    motors.setPitchCurrent(-2000);

    MotorSnapshot yaw_state = motors.yawSnapshot();
    MotorSnapshot pitch_state = motors.pitchSnapshot();
    bool tx_ok = motors.sendCurrentCommands();

    (void)yaw_state;
    (void)pitch_state;
    (void)tx_ok;
}
