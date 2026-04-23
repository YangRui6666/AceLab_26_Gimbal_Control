#include "GM6020.hpp"

#include "gimbal_config.h"

namespace {
float clampf_local(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}
} // namespace

GM6020::GM6020(uint16_t can_id,
               float max_current,
               float limit_pos_deg,
               float limit_neg_deg,
               uint16_t zero_encoder_raw)
    : can_id_(can_id),
      max_current_(max_current),
      limit_pos_deg_(limit_pos_deg),
      limit_neg_deg_(limit_neg_deg),
      zero_encoder_raw_(zero_encoder_raw),
      last_rx_time_ms_(0U),
      state_{0.0f, 0.0f, 0, 0U},
      initialized_(false)
{
}

void GM6020::init()
{
    state_.angle_deg = 0.0f;
    state_.speed_dps = 0.0f;
    state_.current = 0;
    state_.encoder_raw = 0U;
    last_rx_time_ms_ = 0U;
    initialized_ = false;
}

void GM6020::update(const CanRxFrame &frame, uint32_t now_ms)
{
    const uint16_t encoder =
        (uint16_t)(((uint16_t)frame.data[0] << 8U) | (uint16_t)frame.data[1]);
    const int16_t speed_rpm =
        (int16_t)(((uint16_t)frame.data[2] << 8U) | (uint16_t)frame.data[3]);
    const int16_t current =
        (int16_t)(((uint16_t)frame.data[4] << 8U) | (uint16_t)frame.data[5]);

    state_.encoder_raw = encoder;
    state_.angle_deg = encoder_to_deg(encoder);
    state_.speed_dps = static_cast<float>(speed_rpm) * 6.0f;
    state_.current = current;
    last_rx_time_ms_ = now_ms;
    initialized_ = true;
}

bool GM6020::online(uint32_t now_ms) const
{
    return initialized_ && ((now_ms - last_rx_time_ms_) <= MOTOR_OFFLINE_MS);
}

bool GM6020::limit_error() const
{
    return (state_.angle_deg > (limit_pos_deg_ + MOTOR_LIMIT_ERROR_MARGIN_DEG)) ||
           (state_.angle_deg < (limit_neg_deg_ - MOTOR_LIMIT_ERROR_MARGIN_DEG));
}

int16_t GM6020::clamp_current(float current) const
{
    const float limited = clampf_local(current, -max_current_, max_current_);
    return static_cast<int16_t>(limited);
}

uint16_t GM6020::can_id() const
{
    return can_id_;
}

const GM6020::State &GM6020::state() const
{
    return state_;
}

float GM6020::normalize_deg(float angle_deg)
{
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }

    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }

    return angle_deg;
}

float GM6020::encoder_to_deg(uint16_t encoder_raw) const
{
    int32_t delta = static_cast<int32_t>(encoder_raw) - static_cast<int32_t>(zero_encoder_raw_);

    while (delta > 4096) {
        delta -= 8192;
    }

    while (delta < -4096) {
        delta += 8192;
    }

    return normalize_deg(static_cast<float>(delta) * (360.0f / 8192.0f));
}
