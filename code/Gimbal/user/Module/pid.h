//
// Created by CORE on 2026/3/14.
// Updated by CORE on 2026/3/15 - 添加双环PID控制系统
//

#ifndef VISION_F405_PID_H
#define VISION_F405_PID_H

#include <stdint.h>
#include <stdbool.h>
#include "../Config/pid_config.h"
#include "../Config/imu_config.h"

#ifdef __cplusplus
extern "C" {
#endif

// ================== 基础PID控制器结构 ==================
/**
 * @brief 增量式PID控制器结构体
 */
/**
 * @brief 基础 PID 控制器状态
 * @details 保存 PID 参数、误差历史和输出结果。
 */
typedef struct
{
    // PID参数
    float kp;               // 比例系数
    float ki;               // 积分系数
    float kd;               // 微分系数

    // 限幅参数
    float integral_max;     // 积分限幅
    float output_max;       // 输出限幅

    // 内部状态变量
    float error[3];         // 误差历史: [0]当前 [1]上次 [2]上上次
    float integral;         // 积分累积
    float derivative;       // 微分项(滤波后)
    float output;           // 控制输出

    // 配置参数
    float deadzone;         // 死区
    float dt;               // 控制周期
    bool integral_separation; // 积分分离使能
    float separation_threshold; // 积分分离阈值

    // 调试信息
    uint32_t update_count;  // 更新次数计数
} pid_controller_t;

// ================== 世界坐标系控制结构 ==================
/**
 * @brief 世界坐标系云台控制状态
 */
/**
 * @brief 世界坐标系云台控制状态
 * @details 保存世界坐标系目标值、当前测量值、IMU 反馈和控制使能状态。
 */
typedef struct {
    // 目标值(世界坐标系)
    float world_yaw_target;     // 世界坐标系目标Yaw角度 (度)
    float world_pitch_target;   // 世界坐标系目标Pitch角度 (度)

    // 当前值(世界坐标系)
    float world_yaw_current;    // 世界坐标系当前Yaw角度 (度)
    float world_pitch_current;  // 世界坐标系当前Pitch角度 (度)

    // 陀螺仪角速度反馈
    float gyro_yaw_rate;        // 陀螺仪Yaw角速度 (deg/s)
    float gyro_pitch_rate;      // 陀螺仪Pitch角速度 (deg/s)

    // 控制使能
    bool world_control_enable;  // 世界坐标控制使能
    bool gyro_feedback_enable;  // 陀螺仪反馈使能

    // 安全限位
    bool angle_limit_enable;    // 角度限位使能

    // 状态信息
    bool imu_data_valid;        // IMU数据有效性
    uint32_t last_update_tick;  // 最后更新时间戳
    uint32_t world_control_cycles; // 世界坐标控制周期计数
} gimbal_world_state_t;

/**
 * @brief 统一后的电机反馈视图
 * @details 用于 C 控制接口和 C++ 控制层之间传递单轴电机状态。
 */
typedef struct
{
    // 统一后的电机反馈视图，供 C 控制接口和 C++ 控制层之间传递。
    float position_deg;            // 当前位置(度)
    float velocity_rpm;            // 当前速度(rpm)
    float current_ma;              // 原始电流(mA)
    float filtered_current_ma;     // 滤波电流(mA)
    int8_t temp;                   // 当前温度(°C)
    uint32_t last_update_tick;     // 最近反馈时间戳
    bool motor_online;             // 电机在线状态
} gimbal_axis_feedback_t;

// ================== 函数声明 ==================

/**
 * @brief 初始化PID控制器
 * @param pid PID控制器指针
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 * @param integral_max 积分限幅
 * @param output_max 输出限幅
 * @param dt 控制周期(s)
 * @return 初始化是否成功
 */
bool pid_controller_init(pid_controller_t* pid,
                        float kp, float ki, float kd,
                        float integral_max, float output_max, float dt);

/**
 * @brief 增量式PID计算
 * @param pid PID控制器指针
 * @param target 目标值
 * @param feedback 反馈值
 * @return 控制输出
 */
float pid_calculate(pid_controller_t* pid, float target, float feedback);

/**
 * @brief 重置PID控制器状态
 * @param pid PID控制器指针
 */
void pid_reset(pid_controller_t* pid);

/**
 * @brief 初始化云台控制系统
 * @return 初始化是否成功
 */
bool gimbal_control_init(void);

/**
 * @brief 设置云台目标位置
 * @param yaw_target Yaw轴目标位置(度)
 * @param pitch_target Pitch轴目标位置(度)
 * @return 设置是否成功
 */
bool gimbal_set_position_target(float yaw_target, float pitch_target);

/**
 * @brief 更新编码器反馈数据
 * @param yaw_feedback Yaw轴反馈
 * @param pitch_feedback Pitch轴反馈
 * @return 更新是否成功
 */
bool gimbal_set_encoder_feedback(const gimbal_axis_feedback_t* yaw_feedback,
                                 const gimbal_axis_feedback_t* pitch_feedback);

/**
 * @brief 位置环控制更新 (200Hz调用)
 * @return 更新是否成功
 */
bool gimbal_position_loop_update(void);

/**
 * @brief 速度环控制更新 (500Hz调用)
 * @return 更新是否成功
 */
bool gimbal_velocity_loop_update(void);

/**
 * @brief 安全检查
 * @return 系统是否安全
 */
bool gimbal_safety_check(void);

/**
 * @brief 紧急停止
 */
void gimbal_emergency_stop(void);

/**
 * @brief 获取云台状态信息
 * @param yaw_pos 返回Yaw位置
 * @param pitch_pos 返回Pitch位置
 * @param yaw_vel 返回Yaw速度
 * @param pitch_vel 返回Pitch速度
 */
void gimbal_get_status(float* yaw_pos, float* pitch_pos,
                      float* yaw_vel, float* pitch_vel);

/**
 * @brief 主控制接口 - 在ControlTask中调用
 * @return 控制是否成功
 */
bool gimbal_control_task(void);

/**
 * @brief 获取当前输出电流命令
 * @param yaw_current 返回Yaw电流
 * @param pitch_current 返回Pitch电流
 */
void gimbal_get_output_currents(int16_t* yaw_current, int16_t* pitch_current);

// ================== 世界坐标系控制接口 ==================

/**
 * @brief 世界坐标系云台控制
 * @param world_yaw 世界坐标系当前Yaw角度 (度)
 * @param world_pitch 世界坐标系当前Pitch角度 (度)
 * @param gyro_yaw_rate 陀螺仪Yaw角速度 (度/秒)
 * @param gyro_pitch_rate 陀螺仪Pitch角速度 (度/秒)
 * @return 控制是否成功
 */
bool gimbal_world_coordinate_control(float world_yaw, float world_pitch,
                                   float gyro_yaw_rate, float gyro_pitch_rate);

/**
 * @brief 设置世界坐标系目标角度
 * @param world_yaw_target 世界坐标系目标Yaw角度 (度)
 * @param world_pitch_target 世界坐标系目标Pitch角度 (度)
 * @return 设置是否成功
 */
bool gimbal_set_world_target(float world_yaw_target, float world_pitch_target);

/**
 * @brief 使能/禁用世界坐标系控制模式
 * @param enable true=世界坐标控制, false=电机编码器控制
 * @return 设置是否成功
 */
bool gimbal_set_world_control_enable(bool enable);

/**
 * @brief 获取世界坐标系控制状态
 * @return 世界坐标控制状态指针
 */
gimbal_world_state_t* gimbal_get_world_state(void);

/**
 * @brief 检查角度是否在安全限位内
 * @param yaw Yaw角度 (度)
 * @param pitch Pitch角度 (度)
 * @return true=在安全范围内, false=超出限位
 */
bool gimbal_check_angle_limits(float yaw, float pitch);

// 兼容性接口(保持原有函数名)
void moudle_ctrl_gimbal(void);

#ifdef __cplusplus
}
#endif

#endif //VISION_F405_PID_H
