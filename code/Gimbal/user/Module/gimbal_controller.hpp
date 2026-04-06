#ifndef GIMBAL_GIMBAL_CONTROLLER_HPP
#define GIMBAL_GIMBAL_CONTROLLER_HPP

#include <cstdint>

#include "pid_controller.hpp"

/**
 * @brief 单轴反馈数据
 * @details 统一封装位置、速度、电流、温度和在线状态，供控制层使用。
 */
struct AxisFeedback
{
    // 单轴反馈统一入口，既包含运动状态也包含安全判断需要的信息。
    float position_deg = 0.0f;
    float velocity_rpm = 0.0f;
    float current_ma = 0.0f;
    float filtered_current_ma = 0.0f;
    int8_t temp = 0;
    uint32_t last_update_tick = 0U;
    bool motor_online = false;
};

/**
 * @brief 单轴控制参数
 * @details 位置环和速度环参数分开配置，便于独立调参。
 */
struct AxisConfig
{
    // 位置环和速度环参数分开配置，便于单独调参。
    PidConfig position_pid;
    PidConfig velocity_pid;
    float position_min = 0.0f;
    float position_max = 0.0f;
    bool safety_enable = true;
};

/**
 * @brief 单轴状态快照
 * @details 仅用于状态导出和调试，不直接参与闭环计算。
 */
struct AxisStateSnapshot
{
    // 快照只暴露运行态字段，避免对外泄露控制器内部实现。
    float target_position = 0.0f;
    float target_velocity = 0.0f;
    float current_position = 0.0f;
    float current_velocity = 0.0f;
    float current_current = 0.0f;
    int8_t current_temp = 0;
    float position_min = 0.0f;
    float position_max = 0.0f;
    bool safety_enable = true;
    bool motor_online = false;
    bool emergency_stop = false;
    uint32_t last_update_time = 0U;
    int16_t output_current = 0;
};

/**
 * @brief 单轴云台控制器
 * @details 负责单轴的位置环、速度环、安全检查和输出电流管理。
 */
class GimbalAxisController
{
public:
    explicit GimbalAxisController(const AxisConfig& config);

    /**
     * @brief 更新单轴反馈
     * @param feedback 单轴反馈数据
     */
    void setFeedback(const AxisFeedback& feedback);
    /**
     * @brief 设置单轴位置目标
     * @param target_position 目标位置(度)
     */
    void setPositionTarget(float target_position);

    /**
     * @brief 执行单轴安全检查
     * @return true=安全，false=触发保护
     */
    bool safetyCheck();
    /**
     * @brief 更新单轴位置环
     * @return true=更新成功，false=处于急停状态
     */
    bool updatePositionLoop();
    /**
     * @brief 更新单轴速度环
     * @return true=更新成功，false=处于急停状态
     */
    bool updateVelocityLoop();

    /**
     * @brief 计算位置环输出
     * @param target_position 目标位置(度)
     * @param feedback_position 反馈位置(度)
     * @return 位置环输出的目标速度
     */
    float calculatePositionOutput(float target_position, float feedback_position);
    /**
     * @brief 计算速度环输出
     * @param target_velocity 目标速度
     * @param feedback_velocity 反馈速度
     * @return 速度环输出电流
     */
    float calculateVelocityOutput(float target_velocity, float feedback_velocity);

    /**
     * @brief 设置输出电流
     * @param output_current 电流指令
     */
    void setOutputCurrent(int16_t output_current);
    /**
     * @brief 触发单轴紧急停止
     */
    void emergencyStop();
    /**
     * @brief 重置单轴控制状态
     */
    void reset();

    /**
     * @brief 获取单轴状态快照
     * @return 单轴状态快照
     */
    AxisStateSnapshot snapshot() const;

private:
    static float limitFloat(float value, float min_value, float max_value);

private:
    AxisConfig config_;
    PidController position_loop_;
    PidController velocity_loop_;
    AxisFeedback feedback_;
    float target_position_ = 0.0f;
    float target_velocity_ = 0.0f;
    bool emergency_stop_ = false;
    int16_t output_current_ = 0;
};

/**
 * @brief 云台总状态快照
 * @details 汇总双轴快照和全局初始化、急停状态。
 */
struct GimbalStateSnapshot
{
    AxisStateSnapshot yaw;
    AxisStateSnapshot pitch;
    bool system_init = false;
    bool global_emergency = false;
};

/**
 * @brief 云台总控制器
 * @details 负责双轴协调控制、频率分频、初始化和全局安全状态管理。
 */
class GimbalController
{
public:
    GimbalController();

    /**
     * @brief 初始化云台控制器
     * @return true=初始化成功
     */
    bool init();
    /**
     * @brief 更新双轴反馈
     * @param yaw_feedback Yaw 轴反馈
     * @param pitch_feedback Pitch 轴反馈
     */
    void setFeedback(const AxisFeedback& yaw_feedback, const AxisFeedback& pitch_feedback);
    /**
     * @brief 设置双轴位置目标
     * @param yaw_target Yaw 目标角度(度)
     * @param pitch_target Pitch 目标角度(度)
     * @return true=设置成功，false=未初始化
     */
    bool setPositionTarget(float yaw_target, float pitch_target);

    /**
     * @brief 更新位置环
     * @return true=更新成功，false=未初始化或安全异常
     */
    bool positionLoopUpdate();
    /**
     * @brief 更新速度环
     * @return true=更新成功，false=未初始化或全局急停
     */
    bool velocityLoopUpdate();
    /**
     * @brief 执行全局安全检查
     * @return true=系统安全，false=存在异常
     */
    bool safetyCheck();
    /**
     * @brief 执行一次完整控制步
     * @return true=控制成功，false=未初始化或发生异常
     */
    bool controlStep();
    /**
     * @brief 触发全局紧急停止
     */
    void emergencyStop();

    /**
     * @brief 判断控制器是否已初始化
     * @return true=已初始化
     */
    bool initialized() const;
    /**
     * @brief 判断当前是否处于全局急停状态
     * @return true=已急停
     */
    bool globalEmergency() const;

    GimbalAxisController& yaw();
    GimbalAxisController& pitch();
    const GimbalAxisController& yaw() const;
    const GimbalAxisController& pitch() const;

    GimbalStateSnapshot snapshot() const;

private:
    // 全局控制器状态只保留控制流程真正需要的内部状态。
    bool system_init_ = false;
    bool global_emergency_ = false;
    uint8_t velocity_loop_counter_ = 0U;
    uint8_t position_loop_counter_ = 0U;

    GimbalAxisController yaw_axis_;
    GimbalAxisController pitch_axis_;
};

#endif // GIMBAL_GIMBAL_CONTROLLER_HPP
