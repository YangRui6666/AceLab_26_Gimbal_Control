//
// Created by CORE on 2026/3/12.
// Updated by CORE on 2026/3/15 - 实现完整双环PID控制算法
// Updated by CORE on 2026/3/15 - 添加世界坐标系控制
//

#include "pid.h"
#include <math.h>
#include <string.h>
#include "../Bsp/Inc/bsp_can.h"
#include "../Bsp/Inc/bsp_bmi088.h"
#include "../Module/imu_fusion.h"
#include "FreeRTOS.h"
#include "task.h"

// ================== 私有变量 ==================
static gimbal_control_system_t gimbal_system = {0};
static bool system_initialized = false;

// 世界坐标系控制状态
static gimbal_world_state_t world_state = {0};

// ================== 工具函数 ==================

/**
 * @brief 浮点数限幅函数
 * @param value 输入值
 * @param min 最小值
 * @param max 最大值
 * @return 限幅后的值
 */
static float limit_float(float value, float min, float max)
{
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

/**
 * @brief 角度转换：从GM6020原始值转为角度(度)
 * @param raw_angle GM6020原始角度值
 * @return 角度(度)
 */
static float raw_to_degrees(int32_t raw_angle)
{
    return (float)raw_angle * 360.0f / (float)MOTOR_POS_RANGE;
}

/**
 * @brief GM6020速度转换：从原始值转为rpm
 * @param raw_speed GM6020原始速度值
 * @return 速度(rpm)
 */
static inline float raw_to_rpm(int16_t raw_speed)
{
    return (float)raw_speed; // GM6020直接输出rpm
}

/**
 * @brief 电流转换：从mA转为GM6020控制值
 * @param current_ma 电流(mA)
 * @return GM6020控制值
 */
static inline int16_t ma_to_control_value(float current_ma)
{
    // GM6020电流控制范围: -16384~16384 对应 -20A~20A
    return (int16_t)limit_float(current_ma * 16384.0f / 20000.0f, -16384, 16384);
}

// ================== PID基础算法实现 ==================

bool pid_controller_init(pid_controller_t* pid,
                        float kp, float ki, float kd,
                        float integral_max, float output_max, float dt)
{
    if (pid == NULL || dt <= 0) return false;

    // 清零结构体
    memset(pid, 0, sizeof(pid_controller_t));

    // 设置PID参数
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral_max = integral_max;
    pid->output_max = output_max;
    pid->dt = dt;

    // 设置默认配置
    pid->deadzone = 0.0f;
    pid->integral_separation = true;
    pid->separation_threshold = PID_INTEGRAL_SEPARATION_THRESHOLD;

    return true;
}

float pid_calculate(pid_controller_t* pid, float target, float feedback)
{
    if (pid == NULL) return 0.0f;

    // 计算误差
    pid->error[2] = pid->error[1];  // 保存上上次误差
    pid->error[1] = pid->error[0];  // 保存上次误差
    pid->error[0] = target - feedback; // 当前误差

    // 死区处理
    if (fabs(pid->error[0]) < pid->deadzone)
    {
        pid->error[0] = 0.0f;
    }

    // 比例项
    float proportional = pid->kp * pid->error[0];

    // 积分项(带积分分离)
    if (!pid->integral_separation ||
        fabs(pid->error[0]) < pid->separation_threshold)
    {
        pid->integral += pid->error[0] * pid->dt;
        // 积分限幅
        pid->integral = limit_float(pid->integral,
                                   -pid->integral_max,
                                   pid->integral_max);
    }
    float integral = pid->ki * pid->integral;

    // 微分项(带滤波)
    if (pid->update_count >= 2) // 至少需要3个点才能计算微分
    {
        float raw_derivative = (pid->error[0] - pid->error[1]) / pid->dt;

        // 一阶低通滤波器
        float alpha = pid->dt / (PID_DERIVATIVE_FILTER_TC + pid->dt);
        pid->derivative = alpha * raw_derivative + (1.0f - alpha) * pid->derivative;
    }
    float derivative = pid->kd * pid->derivative;

    // 计算输出
    pid->output = proportional + integral + derivative;

    // 输出限幅
    pid->output = limit_float(pid->output, -pid->output_max, pid->output_max);

    // 更新计数器
    pid->update_count++;

    return pid->output;
}

void pid_reset(pid_controller_t* pid)
{
    if (pid == NULL) return;

    memset(pid->error, 0, sizeof(pid->error));
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->output = 0.0f;
    pid->update_count = 0;
}

// ================== 云台控制系统实现 ==================

/**
 * @brief 初始化单轴控制结构
 * @param axis 轴控制结构指针
 * @param pos_kp,pos_ki,pos_kd 位置环PID参数
 * @param vel_kp,vel_ki,vel_kd 速度环PID参数
 * @param pos_limit 位置环输出限幅
 * @param vel_limit 速度环输出限幅
 * @param angle_min,angle_max 角度限制
 * @return 初始化是否成功
 */
static bool init_axis_control(gimbal_axis_control_t* axis,
                             float pos_kp, float pos_ki, float pos_kd,
                             float vel_kp, float vel_ki, float vel_kd,
                             float pos_imax, float pos_limit,
                             float vel_imax, float vel_limit,
                             float angle_min, float angle_max)
{
    if (axis == NULL) return false;

    // 清零结构体
    memset(axis, 0, sizeof(gimbal_axis_control_t));

    // 初始化位置环PID (200Hz)
    if (!pid_controller_init(&axis->position_loop,
                            pos_kp, pos_ki, pos_kd,
                            pos_imax, pos_limit,
                            1.0f / POSITION_LOOP_FREQ_HZ))
    {
        return false;
    }
    axis->position_loop.deadzone = PID_DEADZONE_POSITION;

    // 初始化速度环PID (500Hz)
    if (!pid_controller_init(&axis->velocity_loop,
                            vel_kp, vel_ki, vel_kd,
                            vel_imax, vel_limit,
                            1.0f / VELOCITY_LOOP_FREQ_HZ))
    {
        return false;
    }
    axis->velocity_loop.deadzone = PID_DEADZONE_VELOCITY;

    // 设置安全限制
    axis->position_min = angle_min;
    axis->position_max = angle_max;
    axis->safety_enable = true;

    return true;
}

bool gimbal_control_init(void)
{
    if (system_initialized) return true;

    // 清零系统结构
    memset(&gimbal_system, 0, sizeof(gimbal_control_system_t));

    // 初始化Yaw轴
    if (!init_axis_control(&gimbal_system.yaw,
                          YAW_POSITION_PID_KP, YAW_POSITION_PID_KI, YAW_POSITION_PID_KD,
                          YAW_VELOCITY_PID_KP, YAW_VELOCITY_PID_KI, YAW_VELOCITY_PID_KD,
                          YAW_POSITION_PID_IMAX, YAW_POSITION_PID_LIMIT,
                          YAW_VELOCITY_PID_IMAX, YAW_VELOCITY_PID_LIMIT,
                          YAW_ANGLE_MIN, YAW_ANGLE_MAX))
    {
        return false;
    }

    // 初始化Pitch轴
    if (!init_axis_control(&gimbal_system.pitch,
                          PITCH_POSITION_PID_KP, PITCH_POSITION_PID_KI, PITCH_POSITION_PID_KD,
                          PITCH_VELOCITY_PID_KP, PITCH_VELOCITY_PID_KI, PITCH_VELOCITY_PID_KD,
                          PITCH_POSITION_PID_IMAX, PITCH_POSITION_PID_LIMIT,
                          PITCH_VELOCITY_PID_IMAX, PITCH_VELOCITY_PID_LIMIT,
                          PITCH_ANGLE_MIN, PITCH_ANGLE_MAX))
    {
        return false;
    }

    // 设置系统状态
    gimbal_system.system_init = true;
    gimbal_system.global_emergency = false;

    // 初始化世界坐标系统
    memset(&world_state, 0, sizeof(gimbal_world_state_t));
    world_state.world_control_enable = WORLD_COORDINATE_CONTROL_ENABLE;
    world_state.gyro_feedback_enable = true;
    world_state.angle_limit_enable = true;
    world_state.imu_data_valid = false;

    system_initialized = true;
    return true;
}

gimbal_control_system_t* gimbal_get_system(void)
{
    return &gimbal_system;
}

bool gimbal_set_position_target(float yaw_target, float pitch_target)
{
    if (!system_initialized) return false;

    // 角度限制检查
    yaw_target = limit_float(yaw_target, YAW_ANGLE_MIN, YAW_ANGLE_MAX);
    pitch_target = limit_float(pitch_target, PITCH_ANGLE_MIN, PITCH_ANGLE_MAX);

    // 设置目标
    gimbal_system.yaw.target_position = yaw_target;
    gimbal_system.pitch.target_position = pitch_target;

    return true;
}

/**
 * @brief 更新单轴反馈数据
 * @param axis 轴控制结构
 * @param motor_id 电机ID
 * @return 更新是否成功
 */
static bool update_axis_feedback(gimbal_axis_control_t* axis, uint8_t motor_id)
{
    if (axis == NULL) return false;

    // 检查电机在线状态
    axis->motor_online = !bsp_can_motor_is_timeout(motor_id);

    if (!axis->motor_online)
    {
        return false;
    }

    // 获取电机状态
    const gm6020_state_t* motor_state = bsp_can_get_motor_state(motor_id);
    if (motor_state == NULL)
    {
        axis->motor_online = false;
        return false;
    }

    // 更新反馈数据
    axis->current_position = raw_to_degrees(get_can_get_motor_angle(motor_id));
    axis->current_velocity = raw_to_rpm(motor_state->speed);
    axis->current_current = motor_state->filtered_current;
    axis->current_temp = motor_state->temp;
    axis->last_update_time = xTaskGetTickCount();

    return true;
}

/**
 * @brief 单轴安全检查
 * @param axis 轴控制结构
 * @return 是否安全
 */
static bool axis_safety_check(gimbal_axis_control_t* axis)
{
    if (axis == NULL) return false;

    // 电机离线检查
    if (!axis->motor_online)
    {
        axis->emergency_stop = true;
        return false;
    }

    // 温度保护
    if (axis->current_temp > TEMP_PROTECTION_LEVEL)
    {
        axis->emergency_stop = true;
        return false;
    }

    // 电流保护
    if (fabs(axis->current_current) > CURRENT_EMERGENCY_LEVEL)
    {
        axis->emergency_stop = true;
        return false;
    }

    // 角度限制检查
    if (axis->safety_enable)
    {
        if (axis->current_position < axis->position_min ||
            axis->current_position > axis->position_max)
        {
            // 不立即停机，但限制目标位置
            axis->target_position = limit_float(axis->target_position,
                                               axis->position_min,
                                               axis->position_max);
        }
    }

    return true;
}

bool gimbal_position_loop_update(void)
{
    if (!system_initialized) return false;

    // 更新反馈数据
    if (!update_axis_feedback(&gimbal_system.yaw, 0) ||
        !update_axis_feedback(&gimbal_system.pitch, 1))
    {
        return false;
    }

    // 安全检查
    if (!axis_safety_check(&gimbal_system.yaw) ||
        !axis_safety_check(&gimbal_system.pitch))
    {
        gimbal_system.global_emergency = true;
        return false;
    }

    // 位置环计算 - 输出作为速度环目标
    gimbal_system.yaw.target_velocity = pid_calculate(
        &gimbal_system.yaw.position_loop,
        gimbal_system.yaw.target_position,
        gimbal_system.yaw.current_position);

    gimbal_system.pitch.target_velocity = pid_calculate(
        &gimbal_system.pitch.position_loop,
        gimbal_system.pitch.target_position,
        gimbal_system.pitch.current_position);

    return true;
}

bool gimbal_velocity_loop_update(void)
{
    if (!system_initialized) return false;
    if (gimbal_system.global_emergency) return false;

    // 速度环计算 - 输出作为电流指令
    float yaw_current = pid_calculate(
        &gimbal_system.yaw.velocity_loop,
        gimbal_system.yaw.target_velocity,
        gimbal_system.yaw.current_velocity);

    float pitch_current = pid_calculate(
        &gimbal_system.pitch.velocity_loop,
        gimbal_system.pitch.target_velocity,
        gimbal_system.pitch.current_velocity);

    // 转换为GM6020控制值
    gimbal_system.yaw.output_current = ma_to_control_value(yaw_current);
    gimbal_system.pitch.output_current = ma_to_control_value(pitch_current);

    // 发送控制指令
    int16_t motor_currents[8] = {0};
    motor_currents[0] = gimbal_system.yaw.output_current;   // Yaw: 0x204对应索引0
    motor_currents[2] = gimbal_system.pitch.output_current; // Pitch: 0x206对应索引2

    bsp_ctrl_motor(motor_currents);

    return true;
}

bool gimbal_safety_check(void)
{
    if (!system_initialized) return false;

    // 检查全局紧急停止标志
    if (gimbal_system.global_emergency)
    {
        gimbal_system.safety_trigger_count++;
        return false;
    }

    // 检查各轴安全状态
    if (gimbal_system.yaw.emergency_stop || gimbal_system.pitch.emergency_stop)
    {
        gimbal_system.global_emergency = true;
        gimbal_system.safety_trigger_count++;
        return false;
    }

    return true;
}

void gimbal_emergency_stop(void)
{
    // 设置紧急停止标志
    gimbal_system.global_emergency = true;
    gimbal_system.yaw.emergency_stop = true;
    gimbal_system.pitch.emergency_stop = true;

    // 立即停止所有电机
    int16_t zero_currents[8] = {0};
    bsp_ctrl_motor(zero_currents);

    // 重置PID状态
    pid_reset(&gimbal_system.yaw.position_loop);
    pid_reset(&gimbal_system.yaw.velocity_loop);
    pid_reset(&gimbal_system.pitch.position_loop);
    pid_reset(&gimbal_system.pitch.velocity_loop);
}

void gimbal_get_status(float* yaw_pos, float* pitch_pos,
                      float* yaw_vel, float* pitch_vel)
{
    if (!system_initialized) return;

    if (yaw_pos) *yaw_pos = gimbal_system.yaw.current_position;
    if (pitch_pos) *pitch_pos = gimbal_system.pitch.current_position;
    if (yaw_vel) *yaw_vel = gimbal_system.yaw.current_velocity;
    if (pitch_vel) *pitch_vel = gimbal_system.pitch.current_velocity;
}

// ================== 主控制接口 ==================

bool gimbal_control_task(void)
{
    if (!system_initialized)
    {
        // 首次调用时初始化
        if (!gimbal_control_init())
        {
            return false;
        }
    }

    // 增加控制周期计数
    gimbal_system.control_tick_count++;
    gimbal_system.total_control_cycles++;

    // 速度环更新 (500Hz - 每2个tick更新一次)
    gimbal_system.velocity_loop_counter++;
    if (gimbal_system.velocity_loop_counter >= VELOCITY_LOOP_DIV)
    {
        gimbal_system.velocity_loop_counter = 0;
        if (!gimbal_velocity_loop_update())
        {
            gimbal_emergency_stop();
            return false;
        }
    }

    // 位置环更新 (200Hz - 每5个tick更新一次)
    gimbal_system.position_loop_counter++;
    if (gimbal_system.position_loop_counter >= POSITION_LOOP_DIV)
    {
        gimbal_system.position_loop_counter = 0;
        if (!gimbal_position_loop_update())
        {
            // 位置环失败时不立即紧急停止，但停止控制
            return false;
        }
    }

    // 安全检查
    if (!gimbal_safety_check())
    {
        gimbal_emergency_stop();
        return false;
    }

    return true;
}

// ================== 兼容性接口 ==================

void moudle_ctrl_gimbal(void)
{
    // 兼容原有接口，调用新的控制函数
    gimbal_control_task();
}

// ================== 世界坐标系控制实现 ==================

/**
 * @brief 角度归一化到 -180° 到 +180° 范围
 */
static float normalize_angle(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

/**
 * @brief 陀螺仪数据低通滤波
 */
static float gyro_low_pass_filter(float new_value, float old_value, float alpha)
{
    return alpha * new_value + (1.0f - alpha) * old_value;
}

bool gimbal_world_coordinate_control(float world_yaw, float world_pitch,
                                   float gyro_yaw_rate, float gyro_pitch_rate)
{
    if (!system_initialized || !world_state.world_control_enable)
    {
        return false;
    }

    // 更新当前状态
    world_state.world_yaw_current = world_yaw;
    world_state.world_pitch_current = world_pitch;

    // 陀螺仪数据滤波
    world_state.gyro_yaw_rate = gyro_low_pass_filter(gyro_yaw_rate,
                                                     world_state.gyro_yaw_rate,
                                                     GYRO_LOWPASS_FILTER_ALPHA);
    world_state.gyro_pitch_rate = gyro_low_pass_filter(gyro_pitch_rate,
                                                       world_state.gyro_pitch_rate,
                                                       GYRO_LOWPASS_FILTER_ALPHA);

    // 检查角度限位
    if (world_state.angle_limit_enable)
    {
        if (!gimbal_check_angle_limits(world_state.world_yaw_target,
                                     world_state.world_pitch_target))
        {
            return false;  // 目标角度超出安全范围
        }
    }

    // ===== 位置环：世界坐标角度误差 → 目标角速度 =====
    // 位置环PID计算 (使用现有的位置环PID结构)
    float target_yaw_rate = pid_calculate(&gimbal_system.yaw.position_loop,
                                        world_state.world_yaw_target, world_yaw);
    float target_pitch_rate = pid_calculate(&gimbal_system.pitch.position_loop,
                                          world_state.world_pitch_target, world_pitch);

    // 限制目标角速度
    target_yaw_rate = limit_float(target_yaw_rate, -WORLD_YAW_MAX_RATE, WORLD_YAW_MAX_RATE);
    target_pitch_rate = limit_float(target_pitch_rate, -WORLD_PITCH_MAX_RATE, WORLD_PITCH_MAX_RATE);

    // ===== 速度环：目标角速度 - 陀螺仪角速度 → 电机电流 =====
    // 速度环PID计算
    float yaw_current_output = pid_calculate(&gimbal_system.yaw.velocity_loop,
                                           target_yaw_rate, world_state.gyro_yaw_rate);
    float pitch_current_output = pid_calculate(&gimbal_system.pitch.velocity_loop,
                                             target_pitch_rate, world_state.gyro_pitch_rate);

    // 电流限幅
    gimbal_system.yaw.output_current = (int16_t)limit_float(yaw_current_output, -8000.0f, 8000.0f);
    gimbal_system.pitch.output_current = (int16_t)limit_float(pitch_current_output, -6000.0f, 6000.0f);

    // 发送电机控制指令
    int16_t currents[8] = {0};
    currents[0] = gimbal_system.yaw.output_current;   // Yaw: 0x204
    currents[2] = gimbal_system.pitch.output_current; // Pitch: 0x206

    bsp_ctrl_motor(currents);

    // 更新状态
    world_state.imu_data_valid = true;
    world_state.last_update_tick = xTaskGetTickCount();
    world_state.world_control_cycles++;

    return true;
}

bool gimbal_set_world_target(float world_yaw_target, float world_pitch_target)
{
    if (!system_initialized)
    {
        return false;
    }

    // 检查角度限位
    if (world_state.angle_limit_enable)
    {
        if (!gimbal_check_angle_limits(world_yaw_target, world_pitch_target))
        {
            return false;
        }
    }

    world_state.world_yaw_target = world_yaw_target;
    world_state.world_pitch_target = world_pitch_target;

    return true;
}

bool gimbal_set_world_control_enable(bool enable)
{
    if (!system_initialized)
    {
        return false;
    }

    world_state.world_control_enable = enable;

    if (enable)
    {
        // 切换到世界坐标控制时，重置PID状态
        pid_reset(&gimbal_system.yaw.position_loop);
        pid_reset(&gimbal_system.yaw.velocity_loop);
        pid_reset(&gimbal_system.pitch.position_loop);
        pid_reset(&gimbal_system.pitch.velocity_loop);
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