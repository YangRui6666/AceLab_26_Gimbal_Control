#ifndef GIMBAL_TYPES_H
#define GIMBAL_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t can_id;
    uint8_t dlc;
    uint8_t data[8];
} CanRxFrame;

typedef struct {
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float gyro_x_rad_s;
    float gyro_y_rad_s;
    float gyro_z_rad_s;
} IMURawData;

typedef struct {
    float yaw_deg;
    float pitch_deg;
    float roll_deg;
} AttitudeAngle;

typedef struct {
    float kp;
    float ki;
    float kd;
    float out_min;
    float out_max;
} PIDParam_t;

typedef enum {
    WORK_MODE_STABLE = 0,
    WORK_MODE_SEARCH = 1,
    WORK_MODE_AUTO_AIM = 2,
} WorkMode_e;

typedef enum {
    PROTECT_NONE = 0,
    PROTECT_LOCK = 1,
    PROTECT_DISABLE = 2,
} ProtectState_e;

typedef enum {
    GIMBAL_MODE_STABLE = 0,
    GIMBAL_MODE_SEARCH = 1,
    GIMBAL_MODE_AUTO_AIM = 2,
    GIMBAL_MODE_LOCK_PROTECT = 3,
    GIMBAL_MODE_DISABLE = 4,
} GimbalMode_e;

typedef enum {
    CTRL_MSG_NONE = 0,
    CTRL_MSG_ENABLE_TELEMETRY,
    CTRL_MSG_ENTER_SEARCH,
    CTRL_MSG_AUTO_AIM_DELTA,
    CTRL_MSG_ENTER_LOCK,
    CTRL_MSG_EXIT_LOCK,
} CtrlMsgType_e;

typedef struct {
    CtrlMsgType_e type;
    uint32_t time_stamp_ms;
    float delta_yaw_deg;
    float delta_pitch_deg;
} CtrlMsg_t;

typedef enum {
    LOCK_REASON_MANUAL = 1,
    LOCK_REASON_BOOT_TIMEOUT = 2,
    LOCK_REASON_COMM_TIMEOUT = 3,
} LockReason_e;

enum {
    MOTOR_FAULT_NONE = 0U,
    MOTOR_FAULT_YAW_OFFLINE = 1U << 0,
    MOTOR_FAULT_PITCH_OFFLINE = 1U << 1,
    MOTOR_FAULT_YAW_LIMIT = 1U << 2,
    MOTOR_FAULT_PITCH_LIMIT = 1U << 3,
};

#ifdef __cplusplus
}
#endif

#endif
