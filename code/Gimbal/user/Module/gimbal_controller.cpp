#include "gimbal_controller.hpp"

#include "Config/imu_config.h"
#include "Config/pid_config.h"

namespace
{
constexpr float kMaToControlScale = 16384.0f / 20000.0f;

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

int16_t maToControlValue(float current_ma)
{
    const float limited = limitFloat(current_ma, -20000.0f, 20000.0f);
    return static_cast<int16_t>(limited * kMaToControlScale);
}
}

GimbalAxisController::GimbalAxisController(const AxisConfig& config)
    : config_(config),
      position_loop_(config.position_pid),
      velocity_loop_(config.velocity_pid)
{
}

void GimbalAxisController::setFeedback(const AxisFeedback& feedback)
{
    feedback_ = feedback;
}

void GimbalAxisController::setPositionTarget(float target_position)
{
    target_position_ = limitFloat(target_position, config_.position_min, config_.position_max);
}

bool GimbalAxisController::safetyCheck()
{
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

bool GimbalAxisController::updatePositionLoop()
{
    if (emergency_stop_)
    {
        return false;
    }

    target_velocity_ = position_loop_.update(target_position_, feedback_.position_deg);
    return true;
}

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

float GimbalAxisController::calculatePositionOutput(float target_position, float feedback_position)
{
    target_velocity_ = position_loop_.update(target_position, feedback_position);
    return target_velocity_;
}

float GimbalAxisController::calculateVelocityOutput(float target_velocity, float feedback_velocity)
{
    return velocity_loop_.update(target_velocity, feedback_velocity);
}

void GimbalAxisController::setOutputCurrent(int16_t output_current)
{
    output_current_ = output_current;
}

void GimbalAxisController::emergencyStop()
{
    emergency_stop_ = true;
    target_velocity_ = 0.0f;
    output_current_ = 0;
    position_loop_.reset();
    velocity_loop_.reset();
}

void GimbalAxisController::reset()
{
    emergency_stop_ = false;
    target_velocity_ = 0.0f;
    output_current_ = 0;
    position_loop_.reset();
    velocity_loop_.reset();
}

bool GimbalAxisController::emergencyStopActive() const
{
    return emergency_stop_;
}

int16_t GimbalAxisController::outputCurrent() const
{
    return output_current_;
}

float GimbalAxisController::targetPosition() const
{
    return target_position_;
}

float GimbalAxisController::targetVelocity() const
{
    return target_velocity_;
}

const AxisFeedback& GimbalAxisController::feedback() const
{
    return feedback_;
}

AxisStateSnapshot GimbalAxisController::snapshot() const
{
    AxisStateSnapshot snapshot;
    snapshot.position_loop = position_loop_.snapshot();
    snapshot.velocity_loop = velocity_loop_.snapshot();
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

GimbalController::GimbalController()
    : yaw_axis_(makeYawAxisConfig()),
      pitch_axis_(makePitchAxisConfig())
{
}

bool GimbalController::init()
{
    system_init_ = true;
    global_emergency_ = false;
    control_tick_count_ = 0U;
    velocity_loop_counter_ = 0U;
    position_loop_counter_ = 0U;
    total_control_cycles_ = 0U;
    safety_trigger_count_ = 0U;
    yaw_axis_.reset();
    pitch_axis_.reset();
    return true;
}

void GimbalController::setFeedback(const AxisFeedback& yaw_feedback, const AxisFeedback& pitch_feedback)
{
    yaw_axis_.setFeedback(yaw_feedback);
    pitch_axis_.setFeedback(pitch_feedback);
}

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

bool GimbalController::velocityLoopUpdate()
{
    if (!system_init_ || global_emergency_)
    {
        return false;
    }

    return yaw_axis_.updateVelocityLoop() && pitch_axis_.updateVelocityLoop();
}

bool GimbalController::safetyCheck()
{
    if (!system_init_)
    {
        return false;
    }

    if (global_emergency_)
    {
        ++safety_trigger_count_;
        return false;
    }

    if (yaw_axis_.emergencyStopActive() || pitch_axis_.emergencyStopActive())
    {
        global_emergency_ = true;
        ++safety_trigger_count_;
        return false;
    }

    return true;
}

bool GimbalController::controlStep()
{
    if (!system_init_)
    {
        return false;
    }

    ++control_tick_count_;
    ++total_control_cycles_;

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

GimbalStateSnapshot GimbalController::snapshot() const
{
    GimbalStateSnapshot snapshot;
    snapshot.yaw = yaw_axis_.snapshot();
    snapshot.pitch = pitch_axis_.snapshot();
    snapshot.system_init = system_init_;
    snapshot.global_emergency = global_emergency_;
    snapshot.control_tick_count = control_tick_count_;
    snapshot.velocity_loop_counter = velocity_loop_counter_;
    snapshot.position_loop_counter = position_loop_counter_;
    snapshot.total_control_cycles = total_control_cycles_;
    snapshot.safety_trigger_count = safety_trigger_count_;
    return snapshot;
}
