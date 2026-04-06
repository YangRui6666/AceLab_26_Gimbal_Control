#include "pid_controller.hpp"

#include <cmath>

namespace
{
float limitFloat(float value, float min_value, float max_value)
{
    if (value > max_value)
    {
        return max_value;
    }

    if (value < min_value)
    {
        return min_value;
    }

    return value;
}
}

PidController::PidController(const PidConfig& config)
{
    (void)configure(config);
}

bool PidController::configure(const PidConfig& config)
{
    if (config.dt <= 0.0f)
    {
        return false;
    }

    config_ = config;
    reset();
    return true;
}

float PidController::update(float target, float feedback)
{
    error_[2] = error_[1];
    error_[1] = error_[0];
    error_[0] = target - feedback;

    if (std::fabs(error_[0]) < config_.deadzone)
    {
        error_[0] = 0.0f;
    }

    const float proportional = config_.kp * error_[0];

    if (!config_.integral_separation ||
        std::fabs(error_[0]) < config_.separation_threshold)
    {
        integral_ += error_[0] * config_.dt;
        integral_ = limitFloat(integral_, -config_.integral_max, config_.integral_max);
    }
    const float integral = config_.ki * integral_;

    if (update_count_ >= 2U)
    {
        const float raw_derivative = (error_[0] - error_[1]) / config_.dt;
        const float alpha = config_.dt / (config_.derivative_filter_tc + config_.dt);
        derivative_ = alpha * raw_derivative + (1.0f - alpha) * derivative_;
    }
    const float derivative = config_.kd * derivative_;

    output_ = proportional + integral + derivative;
    output_ = limitFloat(output_, -config_.output_max, config_.output_max);

    ++update_count_;
    return output_;
}

void PidController::reset()
{
    error_[0] = 0.0f;
    error_[1] = 0.0f;
    error_[2] = 0.0f;
    integral_ = 0.0f;
    derivative_ = 0.0f;
    output_ = 0.0f;
    update_count_ = 0U;
}

const PidConfig& PidController::config() const
{
    return config_;
}

PidStateSnapshot PidController::snapshot() const
{
    PidStateSnapshot snapshot;
    snapshot.kp = config_.kp;
    snapshot.ki = config_.ki;
    snapshot.kd = config_.kd;
    snapshot.integral_max = config_.integral_max;
    snapshot.output_max = config_.output_max;
    snapshot.error[0] = error_[0];
    snapshot.error[1] = error_[1];
    snapshot.error[2] = error_[2];
    snapshot.integral = integral_;
    snapshot.derivative = derivative_;
    snapshot.output = output_;
    snapshot.deadzone = config_.deadzone;
    snapshot.dt = config_.dt;
    snapshot.integral_separation = config_.integral_separation;
    snapshot.separation_threshold = config_.separation_threshold;
    snapshot.update_count = update_count_;
    return snapshot;
}
