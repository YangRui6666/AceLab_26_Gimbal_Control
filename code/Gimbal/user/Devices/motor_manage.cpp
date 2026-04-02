//
// Created by CORE on 2026/4/2.
//

#include "motor_manage.hpp"

namespace
{
constexpr MotorConfig kYawConfig = {0x200, 2, 20000};
constexpr MotorConfig kPitchConfig = {0x200, 4, 20000};
}

MotorManager::MotorManager()
    : yaw_motor_(kYawConfig),
      pitch_motor_(kPitchConfig)
{
}

void MotorManager::onCanFrame(uint16_t std_id, const MotorFeedback& fb)
{
    if (std_id == yaw_motor_.rxId())
    {
        yaw_motor_.updateFromFeedback(fb);
    }
    else if (std_id == pitch_motor_.rxId())
    {
        pitch_motor_.updateFromFeedback(fb);
    }
}

void MotorManager::setYawCurrent(int16_t current)
{
    yaw_motor_.setTargetCurrent(current);
}

void MotorManager::setPitchCurrent(int16_t current)
{
    pitch_motor_.setTargetCurrent(current);
}

MotorSnapshot MotorManager::yawSnapshot() const
{
    return yaw_motor_.snapshot();
}

MotorSnapshot MotorManager::pitchSnapshot() const
{
    return pitch_motor_.snapshot();
}

void MotorManager::buildTxFrame(int16_t slots[4]) const
{
    slots[0] = 0;
    slots[1] = 0;
    slots[2] = 0;
    slots[3] = 0;

    slots[yaw_motor_.txSlot() - 1] = yaw_motor_.targetCurrent();
    slots[pitch_motor_.txSlot() - 1] = pitch_motor_.targetCurrent();
}
