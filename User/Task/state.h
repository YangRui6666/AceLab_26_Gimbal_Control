//
// Created by CORE on 2026/4/9.
//

#ifndef GIMBAL_UM_STATE_H
#define GIMBAL_UM_STATE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    WORK_MODE_STABLE = 0,
    WORK_MODE_SEARCH,
    WORK_MODE_AUTO_AIM
} WorkMode_e;

typedef enum
{
    SEARCH_STAGE_LOCAL = 0,
    SEARCH_STAGE_GLOBAL
} SearchStage_e;

typedef enum
{
    PROTECT_NONE = 0,
    PROTECT_LOCK,
    PROTECT_DISABLE
} ProtectState_e;

typedef enum
{
    AIM_TRACK_IDLE = 0,
    AIM_LARGE_MOVE,
    AIM_SMALL_TRACK,
    AIM_SPIN_TRACK,
    AIM_TRACK_LOST
} AimTrackMode_e;

typedef struct
{
    float yaw;
    float pitch;
    float roll;
} AttitudeAngle;

// 在 ctrl_task.cpp 文件作用域（静态全局变量）
static struct {
    // ===== 目标相关 =====
    float yaw_world_target;      // 对地 yaw 目标角（世界系），单位：deg
    float pitch_world_target;    // 对地 pitch 目标角（世界系），单位：deg
    float yaw_joint_target;      // yaw 关节派生角（调试用），单位：deg
    float pitch_joint_target;    // pitch 关节派生角（调试用），单位：deg
    bool world_target_synced;    // world target 是否已与当前 IMU 姿态同步

    // ===== 状态相关 =====
    WorkMode_e work_mode;        // 当前业务模式
    SearchStage_e search_stage;  // SEARCH 内部子阶段
    ProtectState_e protect_state; // 当前保护状态

    // ===== 搜索轨迹相关 =====
    uint32_t search_stage_start_tick;  // 当前搜索阶段开始时刻
    uint32_t search_phase_start_tick;  // 当前搜索轨迹相位起点时刻
    float search_local_center_yaw;     // 局部搜索中心 yaw，单位：deg
    float search_local_center_pitch;   // 局部搜索中心 pitch，单位：deg

    // ===== 自瞄相关 =====
    float auto_aim_target_yaw;    // 最近一次接收的世界系绝对 yaw 目标，单位：deg
    float auto_aim_target_pitch;  // 最近一次接收的世界系绝对 pitch 目标，单位：deg
    float auto_aim_target_yaw_rate_dps; // 最近一次接收的目标 yaw 角速度，单位：deg/s
    uint32_t auto_aim_target_timestamp; // 最近一次接收的视觉时间戳，单位：ms
    float auto_aim_predicted_yaw; // 延迟补偿后的预测 yaw，单位：deg
    float auto_aim_predicted_pitch; // 延迟补偿后的预测 pitch，单位：deg
    float auto_aim_last_predicted_yaw; // 上一次参与小位移差分的预测 yaw，单位：deg
    float auto_aim_filtered_yaw_vel_ref; // 小位移阶段滤波后的 yaw 速度参考，单位：deg/s
    uint32_t auto_aim_last_ref_tick; // 上一次更新自瞄参考的控制时刻，单位：ms
    uint32_t auto_aim_lost_start_tick; // 进入丢帧保持的时刻，单位：ms
    uint8_t auto_aim_spin_stable_count; // 连续满足 spin 条件的帧计数
    bool auto_aim_vel_ref_ready; // 小位移差分前馈是否已初始化
    bool auto_aim_lost_holding; // 当前是否处于丢帧保持
    AimTrackMode_e auto_aim_mode; // AUTO_AIM 内部参考生成模式
    uint32_t last_ctrl_msg_tick;    // 最近一次收到有效控制消息的时刻
    uint32_t last_auto_aim_tick;    // 最近一次收到自瞄消息的时刻
    uint32_t last_status_tx_tick;   // 最近一次发送状态反馈的时刻

    // ===== 控制参考相关 =====
    float yaw_pos_ref; // 当前下发给执行层的 yaw 位置参考，单位：deg
    float yaw_vel_ref; // 当前下发给执行层的 yaw 速度参考，单位：deg/s
    float yaw_acc_ref; // 当前下发给执行层的 yaw 加速度参考，单位：deg/s^2
    float pitch_pos_ref; // 当前下发给执行层的 pitch 位置参考，单位：deg
    float pitch_vel_ref; // 当前下发给执行层的 pitch 速度参考，单位：deg/s
    float pitch_acc_ref; // 当前下发给执行层的 pitch 加速度参考，单位：deg/s^2

    // ===== 健康状态 =====
    bool imu_online;             // IMU 是否正常
    bool motor_yaw_online;       // yaw 电机是否在线
    bool motor_pitch_online;     // pitch 电机是否在线
    uint32_t last_health_check_tick;

    // ===== 姿态数据（缓存） =====
    AttitudeAngle current_attitude;  // 当前融合姿态角
} ctrl_ctx;


#endif //GIMBAL_UM_STATE_H
