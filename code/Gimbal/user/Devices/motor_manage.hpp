//
// Created by CORE on 2026/4/2.
//

#ifndef GIMBAL_MOTOR_MANAGE_HPP
#define GIMBAL_MOTOR_MANAGE_HPP

#include <cstdint>

#include "gm6020.hpp"

class MotorManager
{
public:
    MotorManager();

    bool onCanFrame(uint16_t std_id, const uint8_t data[8], uint8_t dlc, uint32_t tick);
    void pollCanRx();
    bool sendCurrentCommands() const;
    void setYawCurrent(int16_t current);
    void setPitchCurrent(int16_t current);
    MotorSnapshot yawSnapshot() const;
    MotorSnapshot pitchSnapshot() const;

private:
    Gm6020Motor yaw_motor_;
    Gm6020Motor pitch_motor_;
};

#endif //GIMBAL_MOTOR_MANAGE_HPP
