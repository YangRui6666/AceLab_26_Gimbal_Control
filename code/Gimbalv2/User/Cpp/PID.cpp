#include "PID.hpp"

namespace {
constexpr float kDtSeconds = 0.001f;

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

PID::PID()
    : param_{0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
      err_(0.0f),
      last_err_(0.0f),
      integral_(0.0f)
{
}

void PID::init(const PIDParam_t &param)
{
    param_ = param;
    reset();
}

void PID::reset()
{
    err_ = 0.0f;
    last_err_ = 0.0f;
    integral_ = 0.0f;
}

void PID::set_param(const PIDParam_t &param)
{
    param_ = param;
}

float PID::calc(float ref, float fdb)
{
    float derivative = 0.0f;
    float output = 0.0f;

    err_ = ref - fdb;
    integral_ += err_ * kDtSeconds;
    derivative = (err_ - last_err_) / kDtSeconds;

    output = (param_.kp * err_) + (param_.ki * integral_) + (param_.kd * derivative);
    output = clampf_local(output, param_.out_min, param_.out_max);

    if (param_.ki != 0.0f) {
        float integral_limit = output / param_.ki;

        if (integral_limit < 0.0f) {
            integral_ = clampf_local(integral_, integral_limit, -integral_limit);
        } else {
            integral_ = clampf_local(integral_, -integral_limit, integral_limit);
        }
    }

    last_err_ = err_;
    return output;
}

const PIDParam_t &PID::param() const
{
    return param_;
}
