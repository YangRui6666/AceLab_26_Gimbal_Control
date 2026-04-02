//
// Created by CORE on 2026/4/2.
//

#include "gm6020.hpp"

namespace
{
constexpr int16_t kGm6020HalfRange = 4096;
constexpr int32_t kGm6020PosRange = 8192;
}

Gm6020Motor::Gm6020Motor(MotorConfig config)
    : config_(config)
{
}

void Gm6020Motor::updateFromFeedback(const MotorFeedback& fb)
{
    unwrap(fb.pos);

    snapshot_.speed = fb.speed;
    snapshot_.current = fb.current;
    snapshot_.temp = fb.temp;
    snapshot_.last_rx_tick = fb.tick;
}

void Gm6020Motor::setTargetCurrent(int16_t current)
{
    if (current > config_.current_limit)
    {
        current = config_.current_limit;
    }
    else if (current < -config_.current_limit)
    {
        current = -config_.current_limit;
    }

    target_current_ = current;
}

MotorSnapshot Gm6020Motor::snapshot() const
{
    return snapshot_;
}

uint16_t Gm6020Motor::rxId() const
{
    return config_.rx_id;
}

uint8_t Gm6020Motor::txSlot() const
{
    return config_.tx_slot;
}

int16_t Gm6020Motor::targetCurrent() const
{
    return target_current_;
}

void Gm6020Motor::unwrap(int16_t pos)
{
    if (snapshot_.last_rx_tick == 0U)
    {
        last_pos_ = pos;
        turn_count_ = 0;
        snapshot_.total_angle = pos;
        return;
    }

    int16_t delta = pos - last_pos_;

    if (delta > kGm6020HalfRange)
    {
        turn_count_--;
    }
    else if (delta < -kGm6020HalfRange)
    {
        turn_count_++;
    }

    snapshot_.total_angle = turn_count_ * kGm6020PosRange + pos;
    last_pos_ = pos;
}
