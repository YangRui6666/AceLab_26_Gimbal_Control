//
// Created by CORE on 2026/4/9.
//

#include "device_gm6020.h"

GM6020::GM6020(uint16_t can_id, int16_t max_current, float limit_pos, float limit_neg)
    : can_id_(can_id),
      max_current_(max_current),
      limit_pos_(limit_pos),
      limit_neg_(limit_neg),
      last_rx_time_(0U),
      has_feedback_(false)
{
    target_.target_current = 0;
    target_.target_speed_dps = 0.0f;

    state_.angle_deg = 0.0f;
    state_.speed_dps = 0.0f;
    state_.current = 0;
    state_.encoder_raw = 0U;
}

bool GM6020::init()
{
    state_.angle_deg = 0.0f;
    state_.speed_dps = 0.0f;
    state_.current = 0;
    state_.encoder_raw = 0U;

    target_.target_current = 0;
    target_.target_speed_dps = 0.0f;

    last_rx_time_ = 0U;
    has_feedback_ = false;
    return true;
}

bool GM6020::update()
{
    CanRxFrame frame;
    const int32_t encoder_range = 8192;
    const float angle_per_turn_deg = 360.0f;

    if (!bsp_can_rx(can_id_, &frame) || frame.dlc != 8U)
    {
        return false;
    }

    const uint16_t raw_encoder = (uint16_t)((frame.data[0] << 8U) | frame.data[1]);
    const int16_t raw_speed_rpm = (int16_t)((frame.data[2] << 8U) | frame.data[3]);
    const int16_t raw_current = (int16_t)((frame.data[4] << 8U) | frame.data[5]);

    if (raw_encoder >= encoder_range)
    {
        return false;
    }

    if (!has_feedback_)
    {
        state_.angle_deg = (float)raw_encoder * angle_per_turn_deg / (float)encoder_range;
    }
    else
    {
        int32_t delta_encoder = (int32_t)raw_encoder - (int32_t)state_.encoder_raw;

        if (delta_encoder > encoder_range / 2)
        {
            delta_encoder -= encoder_range;
        }
        else if (delta_encoder < -(encoder_range / 2))
        {
            delta_encoder += encoder_range;
        }

        state_.angle_deg += (float)delta_encoder * angle_per_turn_deg / (float)encoder_range;
    }

    state_.encoder_raw = raw_encoder;
    state_.speed_dps = (float)raw_speed_rpm * 6.0f;
    state_.current = raw_current;
    last_rx_time_ = frame.timestamp_ms;
    has_feedback_ = true;

    return true;
}

GM6020::State GM6020::get_state() const
{
    return state_;
}

GM6020::Target GM6020::get_target() const
{
    return target_;
}

bool GM6020::check(uint32_t now_ms) const
{
    return has_feedback_ && ((uint32_t)(now_ms - last_rx_time_) < 500U);
}

bool GM6020::has_feedback() const
{
    return has_feedback_;
}

void GM6020::set_target_current(int16_t target_current)
{
    if (target_current > max_current_)
    {
        target_.target_current = max_current_;
    }
    else if (target_current < -max_current_)
    {
        target_.target_current = -max_current_;
    }
    else
    {
        target_.target_current = target_current;
    }
}

void GM6020::set_target_speed_dps(float speed_dps)
{
    target_.target_speed_dps = speed_dps;
}
