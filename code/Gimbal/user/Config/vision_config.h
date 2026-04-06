/**
 * @file vision_config.h
 * @brief 视觉通信系统配置与共享数据定义
 * @author CORE
 * @date 2026-04-06
 */

#ifndef VISION_CONFIG_H
#define VISION_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

// USB通信配置
#define VISION_USB_RX_BUFFER_SIZE             512U
#define VISION_USB_TX_BUFFER_SIZE             256U
#define VISION_PROTOCOL_RX_CHUNK_SIZE         64U

// 协议帧配置
#define VISION_PROTOCOL_MAX_DATA_LEN          16U
#define VISION_PROTOCOL_MAX_FRAME_LEN         (2U + 1U + 1U + VISION_PROTOCOL_MAX_DATA_LEN + 2U + 2U)
#define VISION_STATUS_PAYLOAD_LEN             12U
#define VISION_LOCK_PAYLOAD_LEN               5U
#define VISION_SEARCH_PAYLOAD_LEN             4U
#define VISION_AUTO_AIM_PAYLOAD_LEN           10U

#define VISION_FRAME_SOF_0                    0xAAU
#define VISION_FRAME_SOF_1                    0x55U
#define VISION_FRAME_EOF_0                    0x5AU
#define VISION_FRAME_EOF_1                    0xA5U

// 任务节拍与超时
#define VISION_TASK_POLL_PERIOD_MS            5U
#define VISION_STATUS_SEND_PERIOD_MS          20U
#define VISION_COMMUNICATION_TIMEOUT_MS       60000U
#define VISION_SEARCH_TIMEOUT_MS              500U
#define VISION_AUTO_AIM_TIMEOUT_MS            500U

// 搜索模式配置
#define SEARCH_YAW_AMPLITUDE_DEG              60.0f
#define SEARCH_PITCH_TARGET_DEG               0.0f
#define SEARCH_MAX_YAW_SPEED_DEG_PER_S        30.0f

// 角度限制
#define VISION_YAW_LIMIT_MIN                  -60.0f
#define VISION_YAW_LIMIT_MAX                   60.0f
#define VISION_PITCH_LIMIT_MIN                -15.0f
#define VISION_PITCH_LIMIT_MAX                 45.0f

// 角度定点缩放: 协议中 1LSB = 0.01°
#define VISION_ANGLE_SCALE                    100.0f

// 模式定义
typedef enum
{
    GIMBAL_MODE_STABLE = 0,
    GIMBAL_MODE_SEARCH = 1,
    GIMBAL_MODE_AUTO_AIM = 2,
    GIMBAL_MODE_LOCK_PROTECT = 3,
    GIMBAL_MODE_DISABLE = 4
} gimbal_mode_t;

// 协议命令字
typedef enum
{
    VISION_PACKET_STATUS = 0x03,
    VISION_PACKET_LOCKED = 0x08,
    VISION_PACKET_ENABLE_STREAM = 0x82,
    VISION_PACKET_ENTER_SEARCH = 0x83,
    VISION_PACKET_AUTO_AIM = 0x84,
    VISION_PACKET_LOCK = 0x87,
    VISION_PACKET_UNLOCK = 0x88
} vision_packet_cmd_t;

// 锁定原因定义
typedef enum
{
    VISION_LOCK_REASON_MANUAL = 1,
    VISION_LOCK_REASON_BOOT_TIMEOUT = 2,
    VISION_LOCK_REASON_COMM_TIMEOUT = 3
} vision_lock_reason_t;

// VisionTask 发布给 ControlTask 的最新命令快照。
typedef struct
{
    gimbal_mode_t requested_mode;
    bool feedback_enabled;
    float yaw_error_deg;
    float pitch_error_deg;
    uint32_t command_timestamp;
    uint32_t last_valid_packet_tick;
    uint32_t mode_sequence;
    uint32_t auto_aim_sequence;
} vision_command_mailbox_t;

// ControlTask 发布给 VisionTask 的云台反馈快照。
typedef struct
{
    float yaw_deg;
    float pitch_deg;
    float roll_deg;
    float yaw_rate_dps;
    float pitch_rate_dps;
    gimbal_mode_t current_mode;
    uint32_t timestamp;
    uint32_t sequence;
    bool disable_active;
    bool can_online;
    bool imu_online;
    bool world_control_enabled;
} gimbal_feedback_snapshot_t;

#endif // VISION_CONFIG_H
