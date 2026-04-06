#include "gimbal_controller.hpp"

#include "Config/imu_config.h"
#include "Config/pid_config.h"

namespace
{
constexpr float kMaToControlScale = 16384.0f / 20000.0f;

// 通用限幅，避免控制量和目标值越界。
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

// 根据配置生成 PID 参数，减少位置环和速度环的重复初始化代码。
PidConfig makePositionPidConfig(float kp, float ki, float kd,
                                float integral_max, float output_max,
                                float dt, float deadzone)
{
    PidConfig config;
    config.kp = kp;
    config.ki = ki;
    config.kd = kd;
    config.integral_max = integral_max;
    config.output_max = output_max;
    config.deadzone = deadzone;
    config.dt = dt;
    config.integral_separation = true;
    config.separation_threshold = PID_INTEGRAL_SEPARATION_THRESHOLD;
    config.derivative_filter_tc = PID_DERIVATIVE_FILTER_TC;
    return config;
}

// yaw 轴使用独立参数，便于和 pitch 轴分开调试。
AxisConfig makeYawAxisConfig()
{
    AxisConfig config;
    config.position_pid = makePositionPidConfig(YAW_POSITION_PID_KP,
                                                YAW_POSITION_PID_KI,
                                                YAW_POSITION_PID_KD,
                                                YAW_POSITION_PID_IMAX,
                                                YAW_POSITION_PID_LIMIT,
                                                1.0f / POSITION_LOOP_FREQ_HZ,
                                                PID_DEADZONE_POSITION);
    config.velocity_pid = makePositionPidConfig(YAW_VELOCITY_PID_KP,
                                                YAW_VELOCITY_PID_KI,
                                                YAW_VELOCITY_PID_KD,
                                                YAW_VELOCITY_PID_IMAX,
                                                YAW_VELOCITY_PID_LIMIT,
                                                1.0f / VELOCITY_LOOP_FREQ_HZ,
                                                PID_DEADZONE_VELOCITY);
    config.position_min = YAW_ANGLE_MIN;
    config.position_max = YAW_ANGLE_MAX;
    config.safety_enable = true;
    return config;
}

// pitch 轴通常负载和约束不同，因此单独配置。
AxisConfig makePitchAxisConfig()
{
    AxisConfig config;
    config.position_pid = makePositionPidConfig(PITCH_POSITION_PID_KP,
                                                PITCH_POSITION_PID_KI,
                                                PITCH_POSITION_PID_KD,
                                                PITCH_POSITION_PID_IMAX,
                                                PITCH_POSITION_PID_LIMIT,
                                                1.0f / POSITION_LOOP_FREQ_HZ,
                                                PID_DEADZONE_POSITION);
    config.velocity_pid = makePositionPidConfig(PITCH_VELOCITY_PID_KP,
                                                PITCH_VELOCITY_PID_KI,
                                                PITCH_VELOCITY_PID_KD,
                                                PITCH_VELOCITY_PID_IMAX,
                                                PITCH_VELOCITY_PID_LIMIT,
                                                1.0f / VELOCITY_LOOP_FREQ_HZ,
                                                PID_DEADZONE_VELOCITY);
    config.position_min = PITCH_ANGLE_MIN;
    config.position_max = PITCH_ANGLE_MAX;
    config.safety_enable = true;
    return config;
}

// 将电流单位从 mA 转换成电机控制帧需要的量纲。
int16_t maToControlValue(float current_ma)
{
    const float limited = limitFloat(current_ma, -20000.0f, 20000.0f);
    return static_cast<int16_t>(limited * kMaToControlScale);
}
}

/**
 * @brief 构造单轴控制器并绑定初始配置
 * @param config 单轴控制参数
 */
GimbalAxisController::GimbalAxisController(const AxisConfig& config)
    : config_(config),
      position_loop_(config.position_pid),
      velocity_loop_(config.velocity_pid)
{
}

/**
 * @brief 写入单轴反馈数据
 * @param feedback 单轴反馈数据
 */
void GimbalAxisController::setFeedback(const AxisFeedback& feedback)
{
    // 由任务层统一写入反馈，控制层只读取缓存值。
    feedback_ = feedback;
}

/**
 * @brief 设置单轴目标位置并执行限幅
 * @param target_position 目标位置(度)
 */
void GimbalAxisController::setPositionTarget(float target_position)
{
    target_position_ = limitFloat(target_position, config_.position_min, config_.position_max);
}

/**
 * @brief 检查单轴是否满足安全条件
 * @return true=安全，false=触发保护
 */
bool GimbalAxisController::safetyCheck()
{
    // 在线、温度和电流异常都会直接进入保护。
    if (!feedback_.motor_online)
    {
        emergency_stop_ = true;
        return false;
    }

    if (feedback_.temp > TEMP_PROTECTION_LEVEL)
    {
        emergency_stop_ = true;
        return false;
    }

    if (feedback_.filtered_current_ma > CURRENT_EMERGENCY_LEVEL ||
        feedback_.filtered_current_ma < -CURRENT_EMERGENCY_LEVEL)
    {
        emergency_stop_ = true;
        return false;
    }

    if (config_.safety_enable)
    {
        target_position_ = limitFloat(target_position_, config_.position_min, config_.position_max);
    }

    return true;
}

/**
 * @brief 更新单轴位置环并生成目标速度
 * @return true=更新成功，false=处于急停状态
 */
bool GimbalAxisController::updatePositionLoop()
{
    if (emergency_stop_)
    {
        return false;
    }

    target_velocity_ = position_loop_.update(target_position_, feedback_.position_deg);
    return true;
}

/**
 * @brief 更新单轴速度环并换算为控制电流
 * @return true=更新成功，false=处于急停状态
 */
bool GimbalAxisController::updateVelocityLoop()
{
    if (emergency_stop_)
    {
        return false;
    }

    const float current_output = velocity_loop_.update(target_velocity_, feedback_.velocity_rpm);
    output_current_ = maToControlValue(current_output);
    return true;
}

/**
 * @brief 计算位置环 PID 输出
 * @param target_position 目标位置(度)
 * @param feedback_position 反馈位置(度)
 * @return 位置环输出的目标速度
 */
float GimbalAxisController::calculatePositionOutput(float target_position, float feedback_position)
{
    target_velocity_ = position_loop_.update(target_position, feedback_position);
    return target_velocity_;
}

/**
 * @brief 计算速度环 PID 输出
 * @param target_velocity 目标速度
 * @param feedback_velocity 反馈速度
 * @return 速度环输出电流
 */
float GimbalAxisController::calculateVelocityOutput(float target_velocity, float feedback_velocity)
{
    return velocity_loop_.update(target_velocity, feedback_velocity);
}

/**
 * @brief 直接设置输出电流
 * @param output_current 电流指令
 */
void GimbalAxisController::setOutputCurrent(int16_t output_current)
{
    output_current_ = output_current;
}

/**
 * @brief 触发单轴紧急停止
 */
void GimbalAxisController::emergencyStop()
{
    // 紧急停止时清空控制输出并复位环路状态。
    emergency_stop_ = true;
    target_velocity_ = 0.0f;
    output_current_ = 0;
    position_loop_.reset();
    velocity_loop_.reset();
}

/**
 * @brief 重置单轴控制状态
 */
void GimbalAxisController::reset()
{
    emergency_stop_ = false;
    target_velocity_ = 0.0f;
    output_current_ = 0;
    position_loop_.reset();
    velocity_loop_.reset();
}

/**
 * @brief 导出单轴状态快照
 * @return 单轴状态副本
 */
AxisStateSnapshot GimbalAxisController::snapshot() const
{
    AxisStateSnapshot snapshot;
    snapshot.target_position = target_position_;
    snapshot.target_velocity = target_velocity_;
    snapshot.current_position = feedback_.position_deg;
    snapshot.current_velocity = feedback_.velocity_rpm;
    snapshot.current_current = feedback_.filtered_current_ma;
    snapshot.current_temp = feedback_.temp;
    snapshot.position_min = config_.position_min;
    snapshot.position_max = config_.position_max;
    snapshot.safety_enable = config_.safety_enable;
    snapshot.motor_online = feedback_.motor_online;
    snapshot.emergency_stop = emergency_stop_;
    snapshot.last_update_time = feedback_.last_update_tick;
    snapshot.output_current = output_current_;
    return snapshot;
}

float GimbalAxisController::limitFloat(float value, float min_value, float max_value)
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

/**
 * @brief 构造云台总控制器并创建双轴实例
 */
GimbalController::GimbalController()
    : yaw_axis_(makeYawAxisConfig()),
      pitch_axis_(makePitchAxisConfig())
{
}

/**
 * @brief 初始化双轴控制器状态
 * @return true=初始化成功
 */
bool GimbalController::init()
{
    system_init_ = true;
    global_emergency_ = false;
    velocity_loop_counter_ = 0U;
    position_loop_counter_ = 0U;
    yaw_axis_.reset();
    pitch_axis_.reset();
    return true;
}

/**
 * @brief 更新双轴反馈数据
 * @param yaw_feedback Yaw 轴反馈
 * @param pitch_feedback Pitch 轴反馈
 */
void GimbalController::setFeedback(const AxisFeedback& yaw_feedback, const AxisFeedback& pitch_feedback)
{
    yaw_axis_.setFeedback(yaw_feedback);
    pitch_axis_.setFeedback(pitch_feedback);
}

/**
 * @brief 设置双轴目标位置
 * @param yaw_target Yaw 目标角度(度)
 * @param pitch_target Pitch 目标角度(度)
 * @return true=设置成功，false=未初始化
 */
bool GimbalController::setPositionTarget(float yaw_target, float pitch_target)
{
    if (!system_init_)
    {
        return false;
    }

    yaw_axis_.setPositionTarget(yaw_target);
    pitch_axis_.setPositionTarget(pitch_target);
    return true;
}

/**
 * @brief 更新双轴位置环
 * @return true=更新成功，false=未初始化或安全异常
 */
bool GimbalController::positionLoopUpdate()
{
    if (!system_init_)
    {
        return false;
    }

    if (!yaw_axis_.safetyCheck() || !pitch_axis_.safetyCheck())
    {
        global_emergency_ = true;
        return false;
    }

    return yaw_axis_.updatePositionLoop() && pitch_axis_.updatePositionLoop();
}

/**
 * @brief 更新双轴速度环
 * @return true=更新成功，false=未初始化或全局急停
 */
bool GimbalController::velocityLoopUpdate()
{
    if (!system_init_ || global_emergency_)
    {
        return false;
    }

    return yaw_axis_.updateVelocityLoop() && pitch_axis_.updateVelocityLoop();
}

/**
 * @brief 检查全局安全状态
 * @return true=系统安全，false=存在异常
 */
bool GimbalController::safetyCheck()
{
    if (!system_init_)
    {
        return false;
    }

    if (global_emergency_)
    {
        return false;
    }

    const GimbalStateSnapshot state = snapshot();
    if (state.yaw.emergency_stop || state.pitch.emergency_stop)
    {
        global_emergency_ = true;
        return false;
    }

    return true;
}

/**
 * @brief 执行一次完整控制步
 * @details 按速度环、位置环和安全检查的顺序执行分频控制。
 * @return true=控制成功，false=未初始化或发生异常
 */
bool GimbalController::controlStep()
{
    if (!system_init_)
    {
        return false;
    }

    // 速度环频率更高，先执行速度环。
    ++velocity_loop_counter_;
    if (velocity_loop_counter_ >= VELOCITY_LOOP_DIV)
    {
        velocity_loop_counter_ = 0U;
        if (!velocityLoopUpdate())
        {
            emergencyStop();
            return false;
        }
    }

    // 位置环按更低频率执行，输出目标速度。
    ++position_loop_counter_;
    if (position_loop_counter_ >= POSITION_LOOP_DIV)
    {
        position_loop_counter_ = 0U;
        if (!positionLoopUpdate())
        {
            return false;
        }
    }

    if (!safetyCheck())
    {
        emergencyStop();
        return false;
    }

    return true;
}

/**
 * @brief 触发全局紧急停止
 */
void GimbalController::emergencyStop()
{
    global_emergency_ = true;
    yaw_axis_.emergencyStop();
    pitch_axis_.emergencyStop();
}

bool GimbalController::initialized() const
{
    return system_init_;
}

bool GimbalController::globalEmergency() const
{
    return global_emergency_;
}

GimbalAxisController& GimbalController::yaw()
{
    return yaw_axis_;
}

GimbalAxisController& GimbalController::pitch()
{
    return pitch_axis_;
}

const GimbalAxisController& GimbalController::yaw() const
{
    return yaw_axis_;
}

const GimbalAxisController& GimbalController::pitch() const
{
    return pitch_axis_;
}

/**
 * @brief 导出云台总状态快照
 * @return 云台总状态副本
 */
GimbalStateSnapshot GimbalController::snapshot() const
{
    GimbalStateSnapshot snapshot;
    snapshot.yaw = yaw_axis_.snapshot();
    snapshot.pitch = pitch_axis_.snapshot();
    snapshot.system_init = system_init_;
    snapshot.global_emergency = global_emergency_;
    return snapshot;
}
