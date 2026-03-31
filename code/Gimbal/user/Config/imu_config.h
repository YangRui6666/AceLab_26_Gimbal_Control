//
// Created by CORE on 2026/3/15.
// IMU和世界坐标控制配置文件
//

#ifndef VISION_F405_IMU_CONFIG_H
#define VISION_F405_IMU_CONFIG_H

// ================== IMU物理参数转换 ==================
// BMI088加速度计转换 (±3g → m/s²)
#define IMU_ACC_SCALE_3G        (3.0f * 9.80665f / 32768.0f)

// BMI088陀螺仪转换 (±2000DPS → deg/s)
#define IMU_GYRO_SCALE_2000DPS  (2000.0f / 32768.0f)

// ================== 世界坐标控制参数 ==================
// 控制频率
#define WORLD_CONTROL_FREQ_HZ       1000    // 控制频率 (Hz)

// 角速度限制
#define WORLD_YAW_MAX_RATE          200.0f  // Yaw最大角速度 (deg/s)
#define WORLD_PITCH_MAX_RATE        150.0f  // Pitch最大角速度 (deg/s)

// ================== 安全限位 ==================
// 世界坐标系角度限制
#define WORLD_YAW_LIMIT_MIN        -180.0f  // Yaw范围限制
#define WORLD_YAW_LIMIT_MAX         180.0f
#define WORLD_PITCH_LIMIT_MIN       -30.0f  // Pitch范围限制 (避免撞击)
#define WORLD_PITCH_LIMIT_MAX        20.0f

// ================== 滤波参数 ==================
// 陀螺仪低通滤波系数 (0.0-1.0, 越小滤波越强)
#define GYRO_LOWPASS_FILTER_ALPHA   0.7f

// IMU数据超时检测
#define IMU_DATA_TIMEOUT_MS         50      // IMU数据超时 (ms)

// ================== 世界坐标PID参数(初始值) ==================
// 注意：这些是初始参数，实际使用时需要调优

// Yaw轴位置环 (世界坐标角度误差 → 目标角速度)
#define WORLD_YAW_POS_KP            0.0f    // 初始设为0，待调优
#define WORLD_YAW_POS_KI            0.0f
#define WORLD_YAW_POS_KD            0.0f
#define WORLD_YAW_POS_INTEGRAL_MAX  50.0f   // 积分限幅(度*秒)
#define WORLD_YAW_POS_OUTPUT_MAX    WORLD_YAW_MAX_RATE  // 输出限幅(度/秒)

// Pitch轴位置环 (世界坐标角度误差 → 目标角速度)
#define WORLD_PITCH_POS_KP          0.0f    // 初始设为0，待调优
#define WORLD_PITCH_POS_KI          0.0f
#define WORLD_PITCH_POS_KD          0.0f
#define WORLD_PITCH_POS_INTEGRAL_MAX 30.0f  // 积分限幅(度*秒)
#define WORLD_PITCH_POS_OUTPUT_MAX  WORLD_PITCH_MAX_RATE // 输出限幅(度/秒)

// Yaw轴速度环 (角速度误差 → 电机电流)
#define WORLD_YAW_VEL_KP            0.0f    // 初始设为0，待调优
#define WORLD_YAW_VEL_KI            0.0f
#define WORLD_YAW_VEL_KD            0.0f
#define WORLD_YAW_VEL_INTEGRAL_MAX  1000.0f // 积分限幅(度/秒*秒)
#define WORLD_YAW_VEL_OUTPUT_MAX    8000.0f // 输出限幅(mA) - GM6020电流限制

// Pitch轴速度环 (角速度误差 → 电机电流)
#define WORLD_PITCH_VEL_KP          0.0f    // 初始设为0，待调优
#define WORLD_PITCH_VEL_KI          0.0f
#define WORLD_PITCH_VEL_KD          0.0f
#define WORLD_PITCH_VEL_INTEGRAL_MAX 800.0f // 积分限幅(度/秒*秒)
#define WORLD_PITCH_VEL_OUTPUT_MAX  6000.0f // 输出限幅(mA) - Pitch轴负载较大

// ================== 工作模式配置 ==================
// 世界坐标控制使能开关
#define WORLD_COORDINATE_CONTROL_ENABLE  true  // true=世界坐标控制, false=电机编码器控制

// 调试输出配置
#define DEBUG_IMU_WORLD_COORDINATE       false  // IMU世界坐标调试输出
#define DEBUG_WORLD_CONTROL_LOOP         false  // 世界坐标控制环调试输出

#endif //VISION_F405_IMU_CONFIG_H