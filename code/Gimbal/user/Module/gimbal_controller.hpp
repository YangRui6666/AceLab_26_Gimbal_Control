#ifndef GIMBAL_GIMBAL_CONTROLLER_HPP
#define GIMBAL_GIMBAL_CONTROLLER_HPP

#include <cstdint>

#include "pid_controller.hpp"

struct AxisFeedback
{
    float position_deg = 0.0f;
    float velocity_rpm = 0.0f;
    float current_ma = 0.0f;
    float filtered_current_ma = 0.0f;
    int8_t temp = 0;
    uint32_t last_update_tick = 0U;
    bool motor_online = false;
};

struct AxisConfig
{
    PidConfig position_pid;
    PidConfig velocity_pid;
    float position_min = 0.0f;
    float position_max = 0.0f;
    bool safety_enable = true;
};

struct AxisStateSnapshot
{
    PidStateSnapshot position_loop;
    PidStateSnapshot velocity_loop;
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

class GimbalAxisController
{
public:
    explicit GimbalAxisController(const AxisConfig& config);

    void setFeedback(const AxisFeedback& feedback);
    void setPositionTarget(float target_position);

    bool safetyCheck();
    bool updatePositionLoop();
    bool updateVelocityLoop();

    float calculatePositionOutput(float target_position, float feedback_position);
    float calculateVelocityOutput(float target_velocity, float feedback_velocity);

    void setOutputCurrent(int16_t output_current);
    void emergencyStop();
    void reset();

    bool emergencyStopActive() const;
    int16_t outputCurrent() const;
    float targetPosition() const;
    float targetVelocity() const;
    const AxisFeedback& feedback() const;
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

struct GimbalStateSnapshot
{
    AxisStateSnapshot yaw;
    AxisStateSnapshot pitch;
    bool system_init = false;
    bool global_emergency = false;
    uint32_t control_tick_count = 0U;
    uint8_t velocity_loop_counter = 0U;
    uint8_t position_loop_counter = 0U;
    uint32_t total_control_cycles = 0U;
    uint32_t safety_trigger_count = 0U;
};

class GimbalController
{
public:
    GimbalController();

    bool init();
    void setFeedback(const AxisFeedback& yaw_feedback, const AxisFeedback& pitch_feedback);
    bool setPositionTarget(float yaw_target, float pitch_target);

    bool positionLoopUpdate();
    bool velocityLoopUpdate();
    bool safetyCheck();
    bool controlStep();
    void emergencyStop();

    bool initialized() const;
    bool globalEmergency() const;

    GimbalAxisController& yaw();
    GimbalAxisController& pitch();
    const GimbalAxisController& yaw() const;
    const GimbalAxisController& pitch() const;

    GimbalStateSnapshot snapshot() const;

private:
    bool system_init_ = false;
    bool global_emergency_ = false;
    uint32_t control_tick_count_ = 0U;
    uint8_t velocity_loop_counter_ = 0U;
    uint8_t position_loop_counter_ = 0U;
    uint32_t total_control_cycles_ = 0U;
    uint32_t safety_trigger_count_ = 0U;

    GimbalAxisController yaw_axis_;
    GimbalAxisController pitch_axis_;
};

#endif // GIMBAL_GIMBAL_CONTROLLER_HPP
