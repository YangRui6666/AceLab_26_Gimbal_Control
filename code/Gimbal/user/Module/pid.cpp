#include "pid.h"

#include <cmath>
#include <cstring>

#include "FreeRTOS.h"
#include "task.h"

#include "gimbal_controller.hpp"

namespace
{
GimbalController g_controller;
gimbal_control_system_t gimbal_system = {};
gimbal_world_state_t world_state = {};

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

float gyroLowPassFilter(float new_value, float old_value, float alpha)
{
    return alpha * new_value + (1.0f - alpha) * old_value;
}

pid_controller_t makePidView(const PidStateSnapshot& snapshot)
{
    pid_controller_t view = {};
    view.kp = snapshot.kp;
    view.ki = snapshot.ki;
    view.kd = snapshot.kd;
    view.integral_max = snapshot.integral_max;
    view.output_max = snapshot.output_max;
    view.error[0] = snapshot.error[0];
    view.error[1] = snapshot.error[1];
    view.error[2] = snapshot.error[2];
    view.integral = snapshot.integral;
    view.derivative = snapshot.derivative;
    view.output = snapshot.output;
    view.deadzone = snapshot.deadzone;
    view.dt = snapshot.dt;
    view.integral_separation = snapshot.integral_separation;
    view.separation_threshold = snapshot.separation_threshold;
    view.update_count = snapshot.update_count;
    return view;
}

void syncAxisView(const AxisStateSnapshot& source, gimbal_axis_control_t* target)
{
    if (target == nullptr)
    {
        return;
    }

    target->position_loop = makePidView(source.position_loop);
    target->velocity_loop = makePidView(source.velocity_loop);
    target->target_position = source.target_position;
    target->target_velocity = source.target_velocity;
    target->current_position = source.current_position;
    target->current_velocity = source.current_velocity;
    target->current_current = source.current_current;
    target->current_temp = source.current_temp;
    target->position_min = source.position_min;
    target->position_max = source.position_max;
    target->safety_enable = source.safety_enable;
    target->motor_online = source.motor_online;
    target->emergency_stop = source.emergency_stop;
    target->last_update_time = source.last_update_time;
    target->output_current = source.output_current;
}

void syncSystemView()
{
    const GimbalStateSnapshot snapshot = g_controller.snapshot();
    syncAxisView(snapshot.yaw, &gimbal_system.yaw);
    syncAxisView(snapshot.pitch, &gimbal_system.pitch);
    gimbal_system.system_init = snapshot.system_init;
    gimbal_system.global_emergency = snapshot.global_emergency;
    gimbal_system.control_tick_count = snapshot.control_tick_count;
    gimbal_system.velocity_loop_counter = snapshot.velocity_loop_counter;
    gimbal_system.position_loop_counter = snapshot.position_loop_counter;
    gimbal_system.total_control_cycles = snapshot.total_control_cycles;
    gimbal_system.safety_trigger_count = snapshot.safety_trigger_count;
}
}

bool pid_controller_init(pid_controller_t* pid,
                         float kp, float ki, float kd,
                         float integral_max, float output_max, float dt)
{
    if (pid == nullptr || dt <= 0.0f)
    {
        return false;
    }

    std::memset(pid, 0, sizeof(pid_controller_t));
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral_max = integral_max;
    pid->output_max = output_max;
    pid->dt = dt;
    pid->deadzone = 0.0f;
    pid->integral_separation = true;
    pid->separation_threshold = PID_INTEGRAL_SEPARATION_THRESHOLD;
    return true;
}

float pid_calculate(pid_controller_t* pid, float target, float feedback)
{
    if (pid == nullptr)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->error[0] = target - feedback;

    if (std::fabs(pid->error[0]) < pid->deadzone)
    {
        pid->error[0] = 0.0f;
    }

    const float proportional = pid->kp * pid->error[0];

    if (!pid->integral_separation ||
        std::fabs(pid->error[0]) < pid->separation_threshold)
    {
        pid->integral += pid->error[0] * pid->dt;
        pid->integral = limitFloat(pid->integral, -pid->integral_max, pid->integral_max);
    }
    const float integral = pid->ki * pid->integral;

    if (pid->update_count >= 2U)
    {
        const float raw_derivative = (pid->error[0] - pid->error[1]) / pid->dt;
        const float alpha = pid->dt / (PID_DERIVATIVE_FILTER_TC + pid->dt);
        pid->derivative = alpha * raw_derivative + (1.0f - alpha) * pid->derivative;
    }
    const float derivative = pid->kd * pid->derivative;

    pid->output = proportional + integral + derivative;
    pid->output = limitFloat(pid->output, -pid->output_max, pid->output_max);
    ++pid->update_count;

    return pid->output;
}

void pid_reset(pid_controller_t* pid)
{
    if (pid == nullptr)
    {
        return;
    }

    std::memset(pid->error, 0, sizeof(pid->error));
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->output = 0.0f;
    pid->update_count = 0U;
}

bool gimbal_control_init(void)
{
    if (g_controller.initialized())
    {
        return true;
    }

    if (!g_controller.init())
    {
        return false;
    }

    std::memset(&world_state, 0, sizeof(world_state));
    world_state.world_control_enable = WORLD_COORDINATE_CONTROL_ENABLE;
    world_state.gyro_feedback_enable = true;
    world_state.angle_limit_enable = true;
    world_state.imu_data_valid = false;

    syncSystemView();
    return true;
}

gimbal_control_system_t* gimbal_get_system(void)
{
    syncSystemView();
    return &gimbal_system;
}

bool gimbal_set_position_target(float yaw_target, float pitch_target)
{
    if (!g_controller.initialized())
    {
        return false;
    }

    const bool ok = g_controller.setPositionTarget(yaw_target, pitch_target);
    syncSystemView();
    return ok;
}

bool gimbal_set_encoder_feedback(const gimbal_axis_feedback_t* yaw_feedback,
                                 const gimbal_axis_feedback_t* pitch_feedback)
{
    if (!g_controller.initialized() || yaw_feedback == nullptr || pitch_feedback == nullptr)
    {
        return false;
    }

    AxisFeedback yaw = {};
    yaw.position_deg = yaw_feedback->position_deg;
    yaw.velocity_rpm = yaw_feedback->velocity_rpm;
    yaw.current_ma = yaw_feedback->current_ma;
    yaw.filtered_current_ma = yaw_feedback->filtered_current_ma;
    yaw.temp = yaw_feedback->temp;
    yaw.last_update_tick = yaw_feedback->last_update_tick;
    yaw.motor_online = yaw_feedback->motor_online;

    AxisFeedback pitch = {};
    pitch.position_deg = pitch_feedback->position_deg;
    pitch.velocity_rpm = pitch_feedback->velocity_rpm;
    pitch.current_ma = pitch_feedback->current_ma;
    pitch.filtered_current_ma = pitch_feedback->filtered_current_ma;
    pitch.temp = pitch_feedback->temp;
    pitch.last_update_tick = pitch_feedback->last_update_tick;
    pitch.motor_online = pitch_feedback->motor_online;

    g_controller.setFeedback(yaw, pitch);
    syncSystemView();
    return true;
}

bool gimbal_position_loop_update(void)
{
    if (!g_controller.initialized())
    {
        return false;
    }

    const bool ok = g_controller.positionLoopUpdate();
    syncSystemView();
    return ok;
}

bool gimbal_velocity_loop_update(void)
{
    if (!g_controller.initialized())
    {
        return false;
    }

    const bool ok = g_controller.velocityLoopUpdate();
    syncSystemView();
    return ok;
}

bool gimbal_safety_check(void)
{
    if (!g_controller.initialized())
    {
        return false;
    }

    const bool ok = g_controller.safetyCheck();
    syncSystemView();
    return ok;
}

void gimbal_emergency_stop(void)
{
    if (!g_controller.initialized())
    {
        return;
    }

    g_controller.emergencyStop();
    syncSystemView();
}

void gimbal_get_status(float* yaw_pos, float* pitch_pos,
                       float* yaw_vel, float* pitch_vel)
{
    syncSystemView();

    if (yaw_pos != nullptr)
    {
        *yaw_pos = gimbal_system.yaw.current_position;
    }

    if (pitch_pos != nullptr)
    {
        *pitch_pos = gimbal_system.pitch.current_position;
    }

    if (yaw_vel != nullptr)
    {
        *yaw_vel = gimbal_system.yaw.current_velocity;
    }

    if (pitch_vel != nullptr)
    {
        *pitch_vel = gimbal_system.pitch.current_velocity;
    }
}

bool gimbal_control_task(void)
{
    if (!g_controller.initialized() && !gimbal_control_init())
    {
        return false;
    }

    const bool ok = g_controller.controlStep();
    syncSystemView();
    return ok;
}

void gimbal_get_output_currents(int16_t* yaw_current, int16_t* pitch_current)
{
    syncSystemView();

    if (yaw_current != nullptr)
    {
        *yaw_current = gimbal_system.yaw.output_current;
    }

    if (pitch_current != nullptr)
    {
        *pitch_current = gimbal_system.pitch.output_current;
    }
}

bool gimbal_world_coordinate_control(float world_yaw, float world_pitch,
                                     float gyro_yaw_rate, float gyro_pitch_rate)
{
    if (!g_controller.initialized() || !world_state.world_control_enable)
    {
        return false;
    }

    world_state.world_yaw_current = world_yaw;
    world_state.world_pitch_current = world_pitch;
    world_state.gyro_yaw_rate = gyroLowPassFilter(gyro_yaw_rate,
                                                  world_state.gyro_yaw_rate,
                                                  GYRO_LOWPASS_FILTER_ALPHA);
    world_state.gyro_pitch_rate = gyroLowPassFilter(gyro_pitch_rate,
                                                    world_state.gyro_pitch_rate,
                                                    GYRO_LOWPASS_FILTER_ALPHA);

    if (world_state.angle_limit_enable &&
        !gimbal_check_angle_limits(world_state.world_yaw_target, world_state.world_pitch_target))
    {
        return false;
    }

    if (!g_controller.yaw().safetyCheck() || !g_controller.pitch().safetyCheck())
    {
        g_controller.emergencyStop();
        syncSystemView();
        return false;
    }

    float target_yaw_rate = g_controller.yaw().calculatePositionOutput(world_state.world_yaw_target, world_yaw);
    float target_pitch_rate = g_controller.pitch().calculatePositionOutput(world_state.world_pitch_target, world_pitch);

    target_yaw_rate = limitFloat(target_yaw_rate, -WORLD_YAW_MAX_RATE, WORLD_YAW_MAX_RATE);
    target_pitch_rate = limitFloat(target_pitch_rate, -WORLD_PITCH_MAX_RATE, WORLD_PITCH_MAX_RATE);

    float yaw_current_output = g_controller.yaw().calculateVelocityOutput(target_yaw_rate, world_state.gyro_yaw_rate);
    float pitch_current_output = g_controller.pitch().calculateVelocityOutput(target_pitch_rate, world_state.gyro_pitch_rate);

    g_controller.yaw().setOutputCurrent(static_cast<int16_t>(limitFloat(yaw_current_output,
                                                                        -WORLD_YAW_VEL_OUTPUT_MAX,
                                                                        WORLD_YAW_VEL_OUTPUT_MAX)));
    g_controller.pitch().setOutputCurrent(static_cast<int16_t>(limitFloat(pitch_current_output,
                                                                          -WORLD_PITCH_VEL_OUTPUT_MAX,
                                                                          WORLD_PITCH_VEL_OUTPUT_MAX)));

    world_state.imu_data_valid = true;
    world_state.last_update_tick = static_cast<uint32_t>(xTaskGetTickCount());
    ++world_state.world_control_cycles;

    syncSystemView();
    return true;
}

bool gimbal_set_world_target(float world_yaw_target, float world_pitch_target)
{
    if (!g_controller.initialized())
    {
        return false;
    }

    if (world_state.angle_limit_enable &&
        !gimbal_check_angle_limits(world_yaw_target, world_pitch_target))
    {
        return false;
    }

    world_state.world_yaw_target = world_yaw_target;
    world_state.world_pitch_target = world_pitch_target;
    return true;
}

bool gimbal_set_world_control_enable(bool enable)
{
    if (!g_controller.initialized())
    {
        return false;
    }

    world_state.world_control_enable = enable;
    if (enable)
    {
        g_controller.yaw().reset();
        g_controller.pitch().reset();
        syncSystemView();
    }
    return true;
}

gimbal_world_state_t* gimbal_get_world_state(void)
{
    return &world_state;
}

bool gimbal_check_angle_limits(float yaw, float pitch)
{
    if (yaw < WORLD_YAW_LIMIT_MIN || yaw > WORLD_YAW_LIMIT_MAX)
    {
        return false;
    }

    if (pitch < WORLD_PITCH_LIMIT_MIN || pitch > WORLD_PITCH_LIMIT_MAX)
    {
        return false;
    }

    return true;
}

void moudle_ctrl_gimbal(void)
{
    (void)gimbal_control_task();
}
