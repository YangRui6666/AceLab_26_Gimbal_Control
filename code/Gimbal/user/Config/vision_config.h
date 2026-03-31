/**
 * @file vision_config.h
 * @brief 视觉通信系统配置参数
 * @author CORE
 * @date 2026-03-15
 */

#ifndef VISION_CONFIG_H
#define VISION_CONFIG_H

// USB通信配置
#define VISION_USB_RX_BUFFER_SIZE   512     // USB接收缓冲区大小
#define VISION_USB_TX_BUFFER_SIZE   256     // USB发送缓冲区大小
#define VISION_PROTOCOL_MAX_LINE    256     // 协议行最大长度

// 超时配置
#define VISION_DATA_TIMEOUT_MS      1000    // 视觉数据超时时间(ms)
#define VISION_STATUS_SEND_PERIOD   20      // 状态发送周期(ms) - 50Hz
#define VISION_USB_TIMEOUT_MS       100     // USB发送超时(ms)

// 哨兵模式配置
#define SENTRY_YAW_MIN             -90.0f   // 哨兵模式Yaw最小角度(度)
#define SENTRY_YAW_MAX              90.0f   // 哨兵模式Yaw最大角度(度)
#define SENTRY_SCAN_SPEED           30.0f   // 扫描速度(度/秒)
#define SENTRY_UPDATE_FREQ          20      // 哨兵模式更新频率(Hz)
#define SENTRY_PITCH_TARGET         0.0f    // 哨兵模式Pitch目标角度(度)

// 角度限制(安全保护)
#define VISION_YAW_LIMIT_MIN       -180.0f  // Yaw轴最小限位(度)
#define VISION_YAW_LIMIT_MAX        180.0f  // Yaw轴最大限位(度)
#define VISION_PITCH_LIMIT_MIN      -30.0f  // Pitch轴最小限位(度)
#define VISION_PITCH_LIMIT_MAX       20.0f  // Pitch轴最大限位(度)

// JSON解析配置
#define VISION_JSON_KEY_MAX_LEN     16      // JSON键名最大长度
#define VISION_JSON_VALUE_MAX_LEN   16      // JSON值最大长度

// 模式定义
typedef enum {
    GIMBAL_MODE_MANUAL = 0,     // 手动模式
    GIMBAL_MODE_AUTO = 1,       // 自瞄模式
    GIMBAL_MODE_SENTRY = 2      // 哨兵模式
} gimbal_mode_t;

// 视觉数据结构
typedef struct {
    bool target_detected;       // 目标检测状态
    float target_yaw;          // 目标Yaw角度(度)
    float target_pitch;        // 目标Pitch角度(度)
    float target_roll;         // 目标Roll角度(度)
    uint32_t last_update_tick; // 上次更新时间戳
    bool data_valid;           // 数据有效性
} vision_data_t;

// 状态信息结构
typedef struct {
    float current_yaw;         // 当前Yaw角度(度)
    float current_pitch;       // 当前Pitch角度(度)
    float gyro_yaw;           // 陀螺仪Yaw角速度(度/秒)
    float gyro_pitch;         // 陀螺仪Pitch角速度(度/秒)
    gimbal_mode_t mode;       // 当前模式
    uint32_t timestamp;       // 时间戳
} gimbal_status_t;

#endif // VISION_CONFIG_H