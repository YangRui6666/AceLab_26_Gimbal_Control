//
// Created by CORE on 2026/4/2.
//

#include "gm6020.hpp"

#include "Bsp/Inc/bsp_can.h"
#include "Config/can_id.h"

namespace
{
constexpr int16_t kGm6020HalfRange = 4096;
constexpr int32_t kGm6020PosRange = 8192;
constexpr uint8_t kMinCanId = 1U;
constexpr uint8_t kMaxCanId = 8U;
}

Gm6020Motor::Gm6020Motor(MotorConfig config)
    : config_(config)
{
}

bool Gm6020Motor::acceptsStdId(uint16_t std_id) const
{
    return std_id == rxStdId();
}

bool Gm6020Motor::updateFromCanPayload(const uint8_t data[8], uint32_t tick)
{
    if (data == nullptr || !isValidCanId(config_.can_id))
    {
        return false;
    }

    const uint16_t raw_angle = static_cast<uint16_t>((static_cast<uint16_t>(data[0]) << 8U) |
                                                     static_cast<uint16_t>(data[1]));
    if (raw_angle >= static_cast<uint16_t>(kGm6020PosRange))
    {
        return false;
    }

    MotorFeedback fb;
    fb.raw_angle = static_cast<int16_t>(raw_angle);
    fb.speed = static_cast<int16_t>((static_cast<uint16_t>(data[2]) << 8U) |
                                    static_cast<uint16_t>(data[3]));
    fb.current = static_cast<int16_t>((static_cast<uint16_t>(data[4]) << 8U) |
                                      static_cast<uint16_t>(data[5]));
    fb.temp = static_cast<int8_t>(data[6]);
    fb.tick = tick;

    updateFromFeedback(fb);
    return true;
}

void Gm6020Motor::updateFromFeedback(const MotorFeedback& fb)
{
    unwrap(fb.raw_angle);

    snapshot_.raw_angle = fb.raw_angle;
    snapshot_.speed = fb.speed;
    snapshot_.current = fb.current;
    if (snapshot_.last_rx_tick == 0U)
    {
        snapshot_.filtered_current = static_cast<float>(fb.current);
    }
    else
    {
        snapshot_.filtered_current = CURRENT_FILTER_ALPHA * static_cast<float>(fb.current) +
                                     (1.0f - CURRENT_FILTER_ALPHA) * snapshot_.filtered_current;
    }
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

uint8_t Gm6020Motor::canId() const
{
    return config_.can_id;
}

uint16_t Gm6020Motor::rxStdId() const
{
    if (!isValidCanId(config_.can_id))
    {
        return 0U;
    }

    return static_cast<uint16_t>(GM6020_FEEDBACK_BASE_STDID + (config_.can_id - 1U));
}

uint16_t Gm6020Motor::txStdId() const
{
    if (!isValidCanId(config_.can_id))
    {
        return 0U;
    }

    return config_.can_id <= 4U ? CAN_STDID_DJI_GROUP1 : CAN_STDID_DJI_GROUP2;
}

uint8_t Gm6020Motor::txSlot() const
{
    if (!isValidCanId(config_.can_id))
    {
        return 0U;
    }

    return static_cast<uint8_t>(((config_.can_id - 1U) % 4U) + 1U);
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

bool Gm6020Motor::isValidCanId(uint8_t can_id)
{
    return can_id >= kMinCanId && can_id <= kMaxCanId;
}
