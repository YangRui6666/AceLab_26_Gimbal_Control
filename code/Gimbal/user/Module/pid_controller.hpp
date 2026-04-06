#ifndef GIMBAL_PID_CONTROLLER_HPP
#define GIMBAL_PID_CONTROLLER_HPP

#include <cstdint>

/**
 * @brief 独立 PID 参数配置
 * @details 将参数与控制逻辑解耦，便于不同轴单独调参。
 */
struct PidConfig
{
    // 独立 PID 参数配置，避免控制逻辑和调参数据耦合。
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

/**
 * @brief PID 状态快照
 * @details 用于调试和状态导出，不直接参与闭环计算。
 */
struct PidStateSnapshot
{
    // 用于调试和状态导出，不直接参与闭环计算。
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

/**
 * @brief 独立 PID 控制器
 */
class PidController
{
public:
    PidController() = default;
    explicit PidController(const PidConfig& config);

    /**
     * @brief 配置 PID 参数并重置内部状态
     * @param config PID 参数集
     * @return true=配置成功，false=控制周期非法
     */
    bool configure(const PidConfig& config);
    /**
     * @brief 执行一次 PID 更新
     * @param target 目标值
     * @param feedback 反馈值
     * @return 控制输出
     */
    float update(float target, float feedback);
    /**
     * @brief 重置 PID 内部状态
     */
    void reset();

    /**
     * @brief 获取当前 PID 配置
     * @return 当前 PID 参数集
     */
    const PidConfig& config() const;
    /**
     * @brief 导出 PID 状态快照
     * @return 当前 PID 状态副本
     */
    PidStateSnapshot snapshot() const;

private:
    // 保存当前 PID 配置及积分/微分中间状态。
    // 保存当前 PID 配置及积分、微分中间状态。
    PidConfig config_;
    float error_[3] = {0.0f, 0.0f, 0.0f};
    float integral_ = 0.0f;
    float derivative_ = 0.0f;
    float output_ = 0.0f;
    uint32_t update_count_ = 0U;
};

#endif // GIMBAL_PID_CONTROLLER_HPP
