#include "pid_controller.hpp"

#include <cmath>

namespace
{
// 限幅函数和控制器内部实现共用，避免重复依赖外部工具代码。
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

/**
 * @brief 构造 PID 控制器并立即配置参数
 * @param config PID 参数集
 */
PidController::PidController(const PidConfig& config)
{
    (void)configure(config);
}

/**
 * @brief 配置 PID 参数并重置内部状态
 * @param config PID 参数集
 * @return true=配置成功，false=控制周期非法
 */
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
    // 误差历史用于微分项和调试快照。
    error_[2] = error_[1];
    error_[1] = error_[0];
    error_[0] = target - feedback;

    if (std::fabs(error_[0]) < config_.deadzone)
    {
        error_[0] = 0.0f;
    }

    const float proportional = config_.kp * error_[0];

    // 积分分离用于大误差阶段抑制积分累积。
    if (!config_.integral_separation ||
        std::fabs(error_[0]) < config_.separation_threshold)
    {
        integral_ += error_[0] * config_.dt;
        integral_ = limitFloat(integral_, -config_.integral_max, config_.integral_max);
    }
    const float integral = config_.ki * integral_;

    // 微分项采用一阶滤波，降低噪声放大。
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

/**
 * @brief 重置 PID 内部状态
 */
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

/**
 * @brief 获取当前 PID 配置
 * @return 当前 PID 参数集
 */
const PidConfig& PidController::config() const
{
    return config_;
}

/**
 * @brief 导出 PID 状态快照
 * @return 当前 PID 状态副本
 */
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
