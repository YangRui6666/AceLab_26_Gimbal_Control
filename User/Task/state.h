//
// Created by CORE on 2026/4/9.
//

#ifndef GIMBAL_UM_STATE_H
#define GIMBAL_UM_STATE_H

#include <stdint.h>
// 在 ctrl_task.c 文件作用域（静态全局变量）
static struct {
    // ===== 目标相关 =====
    int32_t yaw_world_target;      // 对地 yaw 目标角（世界系）
    int32_t pitch_world_target;    // 对地 pitch 目标角（世界系）
    int32_t yaw_joint_target;      // yaw 关节目标角（电机系）
    int32_t pitch_joint_target;    // pitch 关节目标角（电机系）

    // ===== 状态相关 =====
    WorkMode_e work_mode;        // 当前业务模式

    // ===== 搜索轨迹相关 =====
    int32_t search_center_yaw;     // 搜索中心 yaw（进入 SEARCH 时锁定）
    int32_t search_center_pitch;   // 搜索中心 pitch
    uint32_t search_start_tick;  // 搜索开始时刻（用于 Lissajous 时间计算）

    // ===== 自瞄相关 =====
    int32_t auto_aim_delta_yaw;    // 最近一次接收的增量（可累积或单次使用）
    int32_t auto_aim_delta_pitch;

    // ===== 健康状态 =====
    bool imu_online;             // IMU 是否正常
    bool motor_yaw_online;       // yaw 电机是否在线
    bool motor_pitch_online;     // pitch 电机是否在线
    uint32_t last_health_check_tick;

    // ===== 姿态数据（缓存） =====
    AttitudeAngle current_attitude;  // 当前融合姿态角
} ctrl_ctx;


#endif //GIMBAL_UM_STATE_H