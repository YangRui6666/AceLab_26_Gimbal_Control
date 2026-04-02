//
// Created by CORE on 2026/4/2.
//

#ifndef GIMBAL_MOTOR_MANAGE_HPP
#define GIMBAL_MOTOR_MANAGE_HPP

#include "gm6020.hpp"

class MotorManager
{
public:
    MotorManager();

    void onCanFrame(uint16_t std_id, const MotorFeedback& fb);
    void setYawCurrent(int16_t current);
    void setPitchCurrent(int16_t current);
    MotorSnapshot yawSnapshot() const;
    MotorSnapshot pitchSnapshot() const;
    void buildTxFrame(int16_t slots[4]) const;

private:
    Gm6020Motor yaw_motor_;
    Gm6020Motor pitch_motor_;
};

#endif //GIMBAL_MOTOR_MANAGE_HPP
