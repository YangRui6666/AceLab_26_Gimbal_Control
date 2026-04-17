//
// Created by CORE on 2026/4/9.
//
#include <cmath>
#include <cstdint>

#include "bsp_can.h"
#include "bsp_usb.h"
#include "cmsis_os2.h"
#include "ctrl_msg_queue.h"
#include "FreeRTOS.h"
#include "imu_fusion.h"
#include "MotorManage.h"
#include "state.h"
#include "task.h"

namespace
{

constexpr bool k_ctrl_task_debug_block = false;
constexpr uint32_t k_ctrl_period_ms = 1U;
constexpr uint32_t k_auto_aim_timeout_ms = 500U;
constexpr uint32_t k_search_to_stable_timeout_ms = 30000U;
constexpr uint32_t k_status_feedback_period_ms = 10U;
constexpr uint16_t k_yaw_can_id = 0x206U;
constexpr uint16_t k_pitch_can_id = 0x208U;
constexpr uint8_t k_lock_reason_manual = 1U;
constexpr float k_deg_per_count = 100.0f;
constexpr float k_yaw_limit_min_deg = -60.0f;
constexpr float k_yaw_limit_max_deg = 60.0f;
constexpr float k_pitch_limit_min_deg = -40.0f;
constexpr float k_pitch_limit_max_deg = 30.0f;
constexpr float k_search_yaw_amp_deg = 25.0f;
constexpr float k_search_pitch_amp_deg = 15.0f;
constexpr float k_search_base_freq_hz = 0.25f;
constexpr float k_two_pi = 6.28318530718f;
constexpr float k_search_pitch_phase_rad = 1.57079632679f;

float clampf(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }

    if (value > max_value)
    {
        return max_value;
    }

    return value;
}

float wrap_period_s(uint32_t elapsed_ms)
{
    return (float)elapsed_ms * 0.001f;
}

int16_t encode_angle_x100(float angle_deg)
{
    const float scaled = angle_deg * k_deg_per_count;

    if (scaled > 32767.0f)
    {
        return 32767;
    }

    if (scaled < -32768.0f)
    {
        return -32768;
    }

    if (scaled >= 0.0f)
    {
        return (int16_t)(scaled + 0.5f);
    }

    return (int16_t)(scaled - 0.5f);
}

uint8_t ctrl_get_usb_mode(void)
{
    if (ctrl_ctx.protect_state == PROTECT_LOCK)
    {
        return 3U;
    }

    if (ctrl_ctx.protect_state == PROTECT_DISABLE)
    {
        return 4U;
    }

    switch (ctrl_ctx.work_mode)
    {
        case WORK_MODE_SEARCH:
            return 1U;

        case WORK_MODE_AUTO_AIM:
            return 2U;

        case WORK_MODE_STABLE:
        default:
            return 0U;
    }
}

void ctrl_reset_context(void)
{
    ctrl_ctx.yaw_world_target = 0.0f;
    ctrl_ctx.pitch_world_target = 0.0f;
    ctrl_ctx.yaw_joint_target = 0.0f;
    ctrl_ctx.pitch_joint_target = 0.0f;
    ctrl_ctx.work_mode = WORK_MODE_STABLE;
    ctrl_ctx.protect_state = PROTECT_NONE;
    ctrl_ctx.search_center_yaw = 0.0f;
    ctrl_ctx.search_center_pitch = 0.0f;
    ctrl_ctx.search_start_tick = 0U;
    ctrl_ctx.auto_aim_delta_yaw = 0.0f;
    ctrl_ctx.auto_aim_delta_pitch = 0.0f;
    ctrl_ctx.last_ctrl_msg_tick = 0U;
    ctrl_ctx.last_auto_aim_tick = 0U;
    ctrl_ctx.last_status_tx_tick = 0U;
    ctrl_ctx.imu_online = false;
    ctrl_ctx.motor_yaw_online = false;
    ctrl_ctx.motor_pitch_online = false;
    ctrl_ctx.last_health_check_tick = 0U;
    ctrl_ctx.current_attitude.yaw = 0.0f;
    ctrl_ctx.current_attitude.pitch = 0.0f;
    ctrl_ctx.current_attitude.roll = 0.0f;
}

void ctrl_enter_search(uint32_t now_tick)
{
    ctrl_ctx.work_mode = WORK_MODE_SEARCH;
    ctrl_ctx.search_center_yaw = ctrl_ctx.yaw_joint_target;
    ctrl_ctx.search_center_pitch = ctrl_ctx.pitch_joint_target;
    ctrl_ctx.search_start_tick = now_tick;
}

void ctrl_enter_stable(void)
{
    ctrl_ctx.work_mode = WORK_MODE_STABLE;
    ctrl_ctx.auto_aim_delta_yaw = 0.0f;
    ctrl_ctx.auto_aim_delta_pitch = 0.0f;
}

void ctrl_update_health(uint32_t now_tick)
{
    ctrl_ctx.imu_online = imu_check();
    ctrl_ctx.motor_yaw_online = can_check(k_yaw_can_id);
    ctrl_ctx.motor_pitch_online = can_check(k_pitch_can_id);
    ctrl_ctx.last_health_check_tick = now_tick;
}

void ctrl_handle_msg(const CtrlMsg_t *msg, uint32_t now_tick, bool *send_lock_feedback)
{
    if (msg == nullptr)
    {
        return;
    }

    switch (msg->type)
    {
        case CTRL_MSG_ENTER_SEARCH:
            ctrl_ctx.last_ctrl_msg_tick = now_tick;
            ctrl_ctx.protect_state = PROTECT_NONE;
            ctrl_enter_search(now_tick);
            break;

        case CTRL_MSG_AUTO_AIM_DELTA:
            ctrl_ctx.last_ctrl_msg_tick = now_tick;
            ctrl_ctx.last_auto_aim_tick = now_tick;
            ctrl_ctx.work_mode = WORK_MODE_AUTO_AIM;
            ctrl_ctx.protect_state = PROTECT_NONE;
            ctrl_ctx.auto_aim_delta_yaw = -msg->delta_yaw;
            ctrl_ctx.auto_aim_delta_pitch = msg->delta_pitch;
            break;

        case CTRL_MSG_ENTER_LOCK:
            ctrl_ctx.last_ctrl_msg_tick = now_tick;
            if (ctrl_ctx.protect_state != PROTECT_LOCK)
            {
                *send_lock_feedback = true;
            }
            ctrl_ctx.protect_state = PROTECT_LOCK;
            break;

        case CTRL_MSG_EXIT_LOCK:
            ctrl_ctx.last_ctrl_msg_tick = now_tick;
            ctrl_ctx.protect_state = PROTECT_NONE;
            ctrl_enter_stable();
            break;

        case CTRL_MSG_NONE:
        default:
            break;
    }
}

void ctrl_consume_msgs(uint32_t now_tick, bool *send_lock_feedback)
{
    CtrlMsg_t msg;

    while (ctrl_msg_queue_pop(&msg, 0U))
    {
        ctrl_handle_msg(&msg, now_tick, send_lock_feedback);
    }
}

void ctrl_update_search_target(uint32_t now_tick)
{
    const float elapsed_s = wrap_period_s(now_tick - ctrl_ctx.search_start_tick);
    const float yaw_offset = k_search_yaw_amp_deg * sinf(k_two_pi * k_search_base_freq_hz * elapsed_s);
    const float pitch_offset = k_search_pitch_amp_deg * sinf((2.0f * k_two_pi * k_search_base_freq_hz * elapsed_s) +
                                                             k_search_pitch_phase_rad);

    ctrl_ctx.yaw_joint_target = clampf(ctrl_ctx.search_center_yaw + yaw_offset,
                                       k_yaw_limit_min_deg,
                                       k_yaw_limit_max_deg);
    ctrl_ctx.pitch_joint_target = clampf(ctrl_ctx.search_center_pitch + pitch_offset,
                                         k_pitch_limit_min_deg,
                                         k_pitch_limit_max_deg);
}

void ctrl_update_mode_timeout(uint32_t now_tick)
{
    if (ctrl_ctx.protect_state != PROTECT_NONE)
    {
        return;
    }

    if ((ctrl_ctx.work_mode == WORK_MODE_AUTO_AIM) &&
        (ctrl_ctx.last_auto_aim_tick != 0U) &&
        ((uint32_t)(now_tick - ctrl_ctx.last_auto_aim_tick) > k_auto_aim_timeout_ms))
    {
        ctrl_enter_search(now_tick);
        return;
    }

    if ((ctrl_ctx.work_mode == WORK_MODE_SEARCH) &&
        (ctrl_ctx.last_ctrl_msg_tick != 0U) &&
        ((uint32_t)(now_tick - ctrl_ctx.last_ctrl_msg_tick) > k_search_to_stable_timeout_ms))
    {
        ctrl_enter_stable();
    }
}

void ctrl_send_status_if_due(uint32_t now_tick)
{
    usb_status_feedback_t status = {0};

    if ((ctrl_ctx.last_status_tx_tick != 0U) &&
        ((uint32_t)(now_tick - ctrl_ctx.last_status_tx_tick) < k_status_feedback_period_ms))
    {
        return;
    }

    status.yaw_target = encode_angle_x100(ctrl_ctx.current_attitude.yaw);
    status.pitch_target = encode_angle_x100(ctrl_ctx.current_attitude.pitch);
    status.roll_target = encode_angle_x100(ctrl_ctx.current_attitude.roll);
    status.time_stamp = now_tick;
    status.mode = ctrl_get_usb_mode();
    status.reserved = 0U;

    if (usb_protocol_send_status(&status))
    {
        ctrl_ctx.last_status_tx_tick = now_tick;
    }
}

} // namespace

extern "C" void StartCtrlTask(void *argument)
{
    /* USER CODE BEGIN StartCtrlTask */
    //osDelay(osWaitForever);
    (void)argument;

    MotorManage motor_manage;
    TickType_t last_wake_time = xTaskGetTickCount();
    imu_data_t imu_data = {0.0f, 0.0f, 0.0f};

    ctrl_reset_context();
    (void)ctrl_msg_queue_init();
    (void)bsp_can_init();
    imu_init();

    //@warning:
    //TODO:
    //  不要删掉这行！！！
    //  osDelay函数的设计目的是阻塞当前程序
#if k_ctrl_task_debug_block
    osDelay(osWaitForever);
#endif
    //  不要删掉这行！现在在跑debug任务！

    for (;;)
    {
        //osDelay(osWaitForever);
        const uint32_t now_tick = osKernelGetTickCount();
        bool send_lock_feedback = false;

        ctrl_consume_msgs(now_tick, &send_lock_feedback);

        imu_update();
        imu_get_data(&imu_data);
        ctrl_ctx.current_attitude.yaw = imu_data.yaw;
        ctrl_ctx.current_attitude.pitch = imu_data.pitch;
        ctrl_ctx.current_attitude.roll = imu_data.roll;

        motor_manage.update_feedback();
        ctrl_update_health(now_tick);
        ctrl_update_mode_timeout(now_tick);

        if (send_lock_feedback)
        {
            (void)usb_protocol_send_lock_feedback(now_tick, k_lock_reason_manual);
        }

        if (ctrl_ctx.protect_state == PROTECT_LOCK)
        {
            motor_manage.lock();
            ctrl_send_status_if_due(now_tick);
            vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(k_ctrl_period_ms));
            continue;
        }

        if (ctrl_ctx.work_mode == WORK_MODE_SEARCH)
        {
            ctrl_update_search_target(now_tick);
        }
        else if (ctrl_ctx.work_mode == WORK_MODE_AUTO_AIM)
        {
            ctrl_ctx.yaw_joint_target = clampf(motor_manage.get_yaw_joint_deg() + ctrl_ctx.auto_aim_delta_yaw,
                                               k_yaw_limit_min_deg,
                                               k_yaw_limit_max_deg);
            ctrl_ctx.pitch_joint_target = clampf(motor_manage.get_pitch_joint_deg() + ctrl_ctx.auto_aim_delta_pitch,
                                                 k_pitch_limit_min_deg,
                                                 k_pitch_limit_max_deg);
        }

        ctrl_ctx.yaw_joint_target = clampf(ctrl_ctx.yaw_joint_target,
                                           k_yaw_limit_min_deg,
                                           k_yaw_limit_max_deg);
        ctrl_ctx.pitch_joint_target = clampf(ctrl_ctx.pitch_joint_target,
                                             k_pitch_limit_min_deg,
                                             k_pitch_limit_max_deg);

        motor_manage.set(ctrl_ctx.yaw_joint_target, ctrl_ctx.pitch_joint_target);
        motor_manage.send_can_cmd();
        ctrl_send_status_if_due(now_tick);

        auto a = uxTaskGetStackHighWaterMark(NULL);
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(k_ctrl_period_ms));
    }
    /* USER CODE END StartCtrlTask */
}
