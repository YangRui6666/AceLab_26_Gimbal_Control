#ifndef GIMBAL_PID_CONTROLLER_HPP
#define GIMBAL_PID_CONTROLLER_HPP

#include <cstdint>

struct PidConfig
{
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float integral_max = 0.0f;
    float output_max = 0.0f;
    float deadzone = 0.0f;
    float dt = 0.0f;
    bool integral_separation = true;
    float separation_threshold = 0.0f;
    float derivative_filter_tc = 0.0f;
};

struct PidStateSnapshot
{
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float integral_max = 0.0f;
    float output_max = 0.0f;
    float error[3] = {0.0f, 0.0f, 0.0f};
    float integral = 0.0f;
    float derivative = 0.0f;
    float output = 0.0f;
    float deadzone = 0.0f;
    float dt = 0.0f;
    bool integral_separation = true;
    float separation_threshold = 0.0f;
    uint32_t update_count = 0U;
};

class PidController
{
public:
    PidController() = default;
    explicit PidController(const PidConfig& config);

    bool configure(const PidConfig& config);
    float update(float target, float feedback);
    void reset();

    const PidConfig& config() const;
    PidStateSnapshot snapshot() const;

private:
    PidConfig config_;
    float error_[3] = {0.0f, 0.0f, 0.0f};
    float integral_ = 0.0f;
    float derivative_ = 0.0f;
    float output_ = 0.0f;
    uint32_t update_count_ = 0U;
};

#endif // GIMBAL_PID_CONTROLLER_HPP
