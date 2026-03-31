//
// Created by CORE on 2026/3/15.
//

#ifndef VISION_F405_PID_CONFIG_H
#define VISION_F405_PID_CONFIG_H

// ================== 控制频率配置 ==================
#define CONTROL_BASE_FREQ_HZ        1000    // 基础控制频率 (1ms任务)
#define VELOCITY_LOOP_FREQ_HZ       500     // 速度环频率
#define POSITION_LOOP_FREQ_HZ       200     // 位置环频率

// 分频计数器最大值
#define VELOCITY_LOOP_DIV           (CONTROL_BASE_FREQ_HZ / VELOCITY_LOOP_FREQ_HZ)  // 2
#define POSITION_LOOP_DIV           (CONTROL_BASE_FREQ_HZ / POSITION_LOOP_FREQ_HZ)  // 5

// ================== Yaw轴PID参数 ==================
// 位置环参数
#define YAW_POSITION_PID_KP         8.0f    // 比例增益
#define YAW_POSITION_PID_KI         0.0f    // 积分增益
#define YAW_POSITION_PID_KD         0.1f    // 微分增益
#define YAW_POSITION_PID_IMAX       1000.0f // 积分限幅
#define YAW_POSITION_PID_LIMIT      2000.0f // 输出限幅 (转速 rpm)

// 速度环参数
#define YAW_VELOCITY_PID_KP         15.0f   // 比例增益
#define YAW_VELOCITY_PID_KI         0.1f    // 积分增益
#define YAW_VELOCITY_PID_KD         0.0f    // 微分增益
#define YAW_VELOCITY_PID_IMAX       5000.0f // 积分限幅
#define YAW_VELOCITY_PID_LIMIT      16000.0f// 输出限幅 (电流 mA)

// ================== Pitch轴PID参数 ==================
// 位置环参数
#define PITCH_POSITION_PID_KP       6.0f    // 比例增益
#define PITCH_POSITION_PID_KI       0.0f    // 积分增益
#define PITCH_POSITION_PID_KD       0.15f   // 微分增益
#define PITCH_POSITION_PID_IMAX     800.0f  // 积分限幅
#define PITCH_POSITION_PID_LIMIT    1500.0f // 输出限幅 (转速 rpm)

// 速度环参数
#define PITCH_VELOCITY_PID_KP       12.0f   // 比例增益
#define PITCH_VELOCITY_PID_KI       0.08f   // 积分增益
#define PITCH_VELOCITY_PID_KD       0.0f    // 微分增益
#define PITCH_VELOCITY_PID_IMAX     4000.0f // 积分限幅
#define PITCH_VELOCITY_PID_LIMIT    14000.0f// 输出限幅 (电流 mA)

// ================== 安全保护参数 ==================
// 角度限制 (单位：度)
#define YAW_ANGLE_MIN               -180.0f // Yaw轴最小角度
#define YAW_ANGLE_MAX               180.0f  // Yaw轴最大角度
#define PITCH_ANGLE_MIN             -30.0f  // Pitch轴最小角度
#define PITCH_ANGLE_MAX             20.0f   // Pitch轴最大角度

// 电流限制 (单位：mA)
#define CURRENT_LIMIT_MAX           30000   // 最大电流限制
#define CURRENT_EMERGENCY_LEVEL     25000   // 紧急停止电流阈值

// 温度保护 (单位：°C)
#define TEMP_WARNING_LEVEL          70      // 温度警告阈值
#define TEMP_PROTECTION_LEVEL       80      // 温度保护阈值

// 超时保护 (单位：ms) - 复用现有BSP定义
#define MOTOR_COMM_TIMEOUT_MS       MOTOR_TIMEOUT_MS

// ================== PID算法配置 ==================
// 死区设置
#define PID_DEADZONE_VELOCITY       5.0f    // 速度死区 (rpm)
#define PID_DEADZONE_POSITION       0.5f    // 位置死区 (度)

// 微分滤波时间常数 (用于微分项噪声滤波)
#define PID_DERIVATIVE_FILTER_TC    0.01f   // 微分滤波时间常数

// PID积分分离阈值
#define PID_INTEGRAL_SEPARATION_THRESHOLD   50.0f  // 误差超过此值时停止积分

#endif //VISION_F405_PID_CONFIG_H