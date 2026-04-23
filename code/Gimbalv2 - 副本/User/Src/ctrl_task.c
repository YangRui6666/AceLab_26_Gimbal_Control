#include "ctrl_task.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "main.h"
#include "bsp_can.h"
#include "bsp_imu.h"
#include "gimbal_config.h"
#include "gimbal_protocol.h"
#include "gimbal_rtos.h"
#include "imu_fusion.h"
#include "motor_manage_c.h"

#include <math.h>
#include <stdbool.h>

typedef struct {
    WorkMode_e work_mode;
    ProtectState_e protect_state;
    LockReason_e lock_reason;
    AttitudeAngle attitude;
    bool attitude_valid;
    bool telemetry_enabled;
    float yaw_world_target_deg;
    float pitch_world_target_deg;
    uint32_t last_comm_ms;
    uint32_t last_control_ms;
    uint32_t mode_entry_ms;
    uint32_t last_status_tx_ms;
    uint32_t startup_time_ms;
} CtrlContext_t;

static CtrlContext_t s_ctrl_ctx;

static float clampf_local(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}

static float normalize_angle_deg(float angle_deg)
{
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }

    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }

    return angle_deg;
}

static GimbalMode_e ctrl_current_mode(void)
{
    if (s_ctrl_ctx.protect_state == PROTECT_DISABLE) {
        return GIMBAL_MODE_DISABLE;
    }

    if (s_ctrl_ctx.protect_state == PROTECT_LOCK) {
        return GIMBAL_MODE_LOCK_PROTECT;
    }

    switch (s_ctrl_ctx.work_mode) {
    case WORK_MODE_SEARCH:
        return GIMBAL_MODE_SEARCH;

    case WORK_MODE_AUTO_AIM:
        return GIMBAL_MODE_AUTO_AIM;

    case WORK_MODE_STABLE:
    default:
        return GIMBAL_MODE_STABLE;
    }
}

static void ctrl_enter_stable(uint32_t now_ms, bool retarget_to_current)
{
    s_ctrl_ctx.work_mode = WORK_MODE_STABLE;
    s_ctrl_ctx.mode_entry_ms = now_ms;

    if (retarget_to_current && s_ctrl_ctx.attitude_valid) {
        s_ctrl_ctx.yaw_world_target_deg = s_ctrl_ctx.attitude.yaw_deg;
        s_ctrl_ctx.pitch_world_target_deg = s_ctrl_ctx.attitude.pitch_deg;
    }
}

static void ctrl_enter_search(uint32_t now_ms)
{
    s_ctrl_ctx.work_mode = WORK_MODE_SEARCH;
    s_ctrl_ctx.mode_entry_ms = now_ms;
}

static void ctrl_enter_auto_aim(uint32_t now_ms)
{
    s_ctrl_ctx.work_mode = WORK_MODE_AUTO_AIM;
    s_ctrl_ctx.mode_entry_ms = now_ms;
}

static void ctrl_enter_lock(uint32_t now_ms, LockReason_e reason)
{
    if (s_ctrl_ctx.protect_state != PROTECT_LOCK) {
        s_ctrl_ctx.protect_state = PROTECT_LOCK;
        s_ctrl_ctx.lock_reason = reason;
        motor_manage_lock();
        (void)gimbal_protocol_send_lock_feedback(reason, now_ms);
    }
}

static void ctrl_enter_disable(void)
{
    s_ctrl_ctx.protect_state = PROTECT_DISABLE;
    motor_manage_disable();
}

static bool ctrl_init_modules(void)
{
    uint32_t retry = 0U;

    if (!bsp_can_init()) {
        return false;
    }

    for (retry = 0U; retry < IMU_INIT_RETRY_CNT; ++retry) {
        if (bsp_imu_init()) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(IMU_INIT_RETRY_DELAY_MS));
    }

    if (!imu_check()) {
        return false;
    }

    if (!imu_fusion_init()) {
        return false;
    }

    if (!motor_manage_init()) {
        return false;
    }

    return true;
}

static void ctrl_handle_msg(const CtrlMsg_t *msg, uint32_t now_ms)
{
    if (msg == NULL) {
        return;
    }

    if (s_ctrl_ctx.protect_state == PROTECT_DISABLE) {
        return;
    }

    switch (msg->type) {
    case CTRL_MSG_ENABLE_TELEMETRY:
        s_ctrl_ctx.telemetry_enabled = true;
        s_ctrl_ctx.last_comm_ms = now_ms;
        break;

    case CTRL_MSG_ENTER_SEARCH:
        s_ctrl_ctx.last_comm_ms = now_ms;
        s_ctrl_ctx.last_control_ms = now_ms;
        if (s_ctrl_ctx.protect_state == PROTECT_NONE) {
            ctrl_enter_search(now_ms);
        }
        break;

    case CTRL_MSG_AUTO_AIM_DELTA:
        s_ctrl_ctx.last_comm_ms = now_ms;
        s_ctrl_ctx.last_control_ms = now_ms;
        if (s_ctrl_ctx.protect_state == PROTECT_NONE) {
            if (s_ctrl_ctx.work_mode != WORK_MODE_AUTO_AIM) {
                ctrl_enter_auto_aim(now_ms);
            }
            s_ctrl_ctx.yaw_world_target_deg += msg->delta_yaw_deg;
            s_ctrl_ctx.pitch_world_target_deg += msg->delta_pitch_deg;
        }
        break;

    case CTRL_MSG_ENTER_LOCK:
        s_ctrl_ctx.last_comm_ms = now_ms;
        ctrl_enter_lock(now_ms, LOCK_REASON_MANUAL);
        break;

    case CTRL_MSG_EXIT_LOCK:
        s_ctrl_ctx.last_comm_ms = now_ms;
        if (s_ctrl_ctx.protect_state == PROTECT_LOCK) {
            s_ctrl_ctx.protect_state = PROTECT_NONE;
            ctrl_enter_stable(now_ms, true);
        }
        break;

    case CTRL_MSG_NONE:
    default:
        break;
    }
}

static void ctrl_update_search_target(uint32_t now_ms)
{
    const float t = ((float)(now_ms - s_ctrl_ctx.mode_entry_ms)) / 1000.0f;

    s_ctrl_ctx.yaw_world_target_deg =
        SEARCH_YAW_CENTER_DEG + (SEARCH_YAW_AMP_DEG * sinf(SEARCH_W_RAD * t));
    s_ctrl_ctx.pitch_world_target_deg =
        SEARCH_PITCH_CENTER_DEG +
        (SEARCH_PITCH_AMP_DEG * sinf((2.0f * SEARCH_W_RAD * t) + SEARCH_PITCH_PHASE_RAD));
}

static void ctrl_world_to_joint(const AttitudeAngle *attitude,
                                float yaw_world_target_deg,
                                float pitch_world_target_deg,
                                float *yaw_joint_target_deg,
                                float *pitch_joint_target_deg)
{
    *yaw_joint_target_deg =
        clampf_local(normalize_angle_deg(yaw_world_target_deg - attitude->yaw_deg),
                     YAW_LIMIT_NEG_DEG,
                     YAW_LIMIT_POS_DEG);

    *pitch_joint_target_deg =
        clampf_local(normalize_angle_deg(pitch_world_target_deg - attitude->pitch_deg),
                     PITCH_LIMIT_NEG_DEG,
                     PITCH_LIMIT_POS_DEG);
}

static void ctrl_process_timeouts(uint32_t now_ms)
{
    if (s_ctrl_ctx.protect_state != PROTECT_NONE) {
        return;
    }

    if ((now_ms - s_ctrl_ctx.last_comm_ms) >= LOCK_TIMEOUT_MS) {
        ctrl_enter_lock(now_ms, LOCK_REASON_COMM_TIMEOUT);
        return;
    }

    if ((s_ctrl_ctx.work_mode == WORK_MODE_AUTO_AIM) &&
        ((now_ms - s_ctrl_ctx.last_control_ms) >= COMM_TIMEOUT_MS)) {
        ctrl_enter_search(now_ms);
        return;
    }

    if ((s_ctrl_ctx.work_mode == WORK_MODE_SEARCH) &&
        ((now_ms - s_ctrl_ctx.last_control_ms) >= COMM_TIMEOUT_MS)) {
        ctrl_enter_stable(now_ms, true);
    }
}

static void ctrl_check_health(uint32_t now_ms)
{
    if (!imu_check() || !imu_fusion_check()) {
        ctrl_enter_disable();
        return;
    }

    if ((now_ms - s_ctrl_ctx.startup_time_ms) < STARTUP_GRACE_MS) {
        return;
    }

    if (motor_manage_check(now_ms) != MOTOR_FAULT_NONE) {
        ctrl_enter_disable();
    }
}

void ctrl_Start_Task(void *argument)
{
    TickType_t last_wake_time = xTaskGetTickCount();

    (void)argument;

    s_ctrl_ctx.work_mode = WORK_MODE_STABLE;
    s_ctrl_ctx.protect_state = PROTECT_NONE;
    s_ctrl_ctx.lock_reason = LOCK_REASON_MANUAL;
    s_ctrl_ctx.attitude_valid = false;
    s_ctrl_ctx.telemetry_enabled = false;
    s_ctrl_ctx.yaw_world_target_deg = 0.0f;
    s_ctrl_ctx.pitch_world_target_deg = 0.0f;
    s_ctrl_ctx.last_comm_ms = HAL_GetTick();
    s_ctrl_ctx.last_control_ms = HAL_GetTick();
    s_ctrl_ctx.mode_entry_ms = HAL_GetTick();
    s_ctrl_ctx.last_status_tx_ms = 0U;
    s_ctrl_ctx.startup_time_ms = HAL_GetTick();

    if (!ctrl_init_modules()) {
        ctrl_enter_disable();
    }

    for (;;) {
        CtrlMsg_t msg = {0};
        IMURawData imu_raw = {0};
        uint32_t now_ms = HAL_GetTick();

        while ((g_ctrl_msg_queue != NULL) && (xQueueReceive(g_ctrl_msg_queue, &msg, 0U) == pdPASS)) {
            ctrl_handle_msg(&msg, now_ms);
            now_ms = HAL_GetTick();
        }

        if (bsp_imu_update(&imu_raw) && imu_fusion_update(&imu_raw)) {
            s_ctrl_ctx.attitude = imu_fusion_get_angle();
            if (!s_ctrl_ctx.attitude_valid) {
                s_ctrl_ctx.yaw_world_target_deg = s_ctrl_ctx.attitude.yaw_deg;
                s_ctrl_ctx.pitch_world_target_deg = s_ctrl_ctx.attitude.pitch_deg;
                s_ctrl_ctx.attitude_valid = true;
            }
        }

        motor_manage_update_feedback();
        ctrl_check_health(now_ms);
        ctrl_process_timeouts(now_ms);

        if (s_ctrl_ctx.protect_state == PROTECT_LOCK) {
            motor_manage_lock();
        } else if (s_ctrl_ctx.protect_state == PROTECT_DISABLE) {
            motor_manage_disable();
        } else if (s_ctrl_ctx.attitude_valid) {
            float yaw_joint_target_deg = 0.0f;
            float pitch_joint_target_deg = 0.0f;

            if (s_ctrl_ctx.work_mode == WORK_MODE_SEARCH) {
                ctrl_update_search_target(now_ms);
            }

            ctrl_world_to_joint(&s_ctrl_ctx.attitude,
                                s_ctrl_ctx.yaw_world_target_deg,
                                s_ctrl_ctx.pitch_world_target_deg,
                                &yaw_joint_target_deg,
                                &pitch_joint_target_deg);
            motor_manage_set(yaw_joint_target_deg, pitch_joint_target_deg);
        }

        if (s_ctrl_ctx.attitude_valid &&
            ((now_ms - s_ctrl_ctx.last_status_tx_ms) >= TELEMETRY_PERIOD_MS) &&
            gimbal_protocol_send_status(&s_ctrl_ctx.attitude,
                                        now_ms,
                                        ctrl_current_mode(),
                                        s_ctrl_ctx.telemetry_enabled)) {
            s_ctrl_ctx.last_status_tx_ms = now_ms;
        }

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(CTRL_TASK_PERIOD_MS));
    }
}
