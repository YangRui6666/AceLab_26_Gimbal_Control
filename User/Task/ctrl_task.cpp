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
constexpr float k_pitch_limit_min_deg = -10.0f;
constexpr float k_pitch_limit_max_deg = 40.0f;
constexpr float k_search_base_freq_hz = 0.25f;
constexpr float k_search_yaw_freq_mul = 2.0f;
constexpr float k_search_pitch_freq_mul = 3.0f;
constexpr float k_two_pi = 6.28318530718f;
constexpr float k_search_pitch_phase_rad = 0.0f;
constexpr uint32_t k_search_local_duration_ms = 1000U;
constexpr float k_search_local_yaw_amp_deg = 8.0f;
constexpr float k_search_local_pitch_amp_deg = 5.0f;
constexpr float k_search_global_yaw_amp_deg = 60.0f;
constexpr float k_search_global_pitch_center_deg = 15.0f;
constexpr float k_search_global_pitch_amp_deg = 25.0f;
constexpr uint32_t k_search_global_reentry_samples = 256U;

/**
 * @brief 将数值限制在指定范围内。
 */
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

/**
 * @brief 将毫秒差值转换为秒，供搜索轨迹计算使用。
 */
float wrap_period_s(uint32_t elapsed_ms)
{
    return (float)elapsed_ms * 0.001f;
}

/**
 * @brief 计算局部搜索阶段的世界系目标角。
 *
 * 局部搜索以当前中心角为基准，在小范围内进行正弦摆动。
 */
void ctrl_get_local_search_target(float elapsed_s, float *yaw_target, float *pitch_target)
{
    const float yaw = ctrl_ctx.search_local_center_yaw +
                      (k_search_local_yaw_amp_deg *
                       sinf(k_search_yaw_freq_mul * k_two_pi * k_search_base_freq_hz * elapsed_s));
    const float pitch = ctrl_ctx.search_local_center_pitch +
                        (k_search_local_pitch_amp_deg *
                         sinf((k_search_pitch_freq_mul * k_two_pi * k_search_base_freq_hz * elapsed_s) +
                              k_search_pitch_phase_rad));

    *yaw_target = yaw;
    *pitch_target = pitch;
}

/**
 * @brief 计算全局搜索阶段的世界系目标角。
 *
 * 全局搜索用于扩大搜索范围，优先覆盖更大角域。
 */
void ctrl_get_global_search_target(float elapsed_s, float *yaw_target, float *pitch_target)
{
    const float yaw = k_search_global_yaw_amp_deg *
                      sinf(k_search_yaw_freq_mul * k_two_pi * k_search_base_freq_hz * elapsed_s);
    const float pitch = k_search_global_pitch_center_deg +
                        (k_search_global_pitch_amp_deg *
                         sinf((k_search_pitch_freq_mul * k_two_pi * k_search_base_freq_hz * elapsed_s) +
                              k_search_pitch_phase_rad));

    *yaw_target = yaw;
    *pitch_target = pitch;
}

/**
 * @brief 在全局搜索轨迹中寻找最接近参考姿态的回归相位。
 *
 * 该函数用于从全局搜索切回时，尽量减少轨迹相位跳变。
 */
uint32_t ctrl_find_global_reentry_elapsed_ms(float ref_yaw, float ref_pitch)
{
    const float period_s = 1.0f / k_search_base_freq_hz;
    const float sample_step_s = period_s / (float)k_search_global_reentry_samples;
    float best_cost = 0.0f;
    uint32_t best_index = 0U;
    bool best_valid = false;

    for (uint32_t i = 0U; i < k_search_global_reentry_samples; ++i)
    {
        const float elapsed_s = sample_step_s * (float)i;
        float sample_yaw = 0.0f;
        float sample_pitch = 0.0f;
        ctrl_get_global_search_target(elapsed_s, &sample_yaw, &sample_pitch);

        const float yaw_error = sample_yaw - ref_yaw;
        const float pitch_error = sample_pitch - ref_pitch;
        const float cost = (yaw_error * yaw_error) + (pitch_error * pitch_error);

        if (!best_valid || (cost < best_cost))
        {
            best_cost = cost;
            best_index = i;
            best_valid = true;
        }
    }

    return (uint32_t)((period_s * 1000.0f * (float)best_index) / (float)k_search_global_reentry_samples);
}

/**
 * @brief 将角度按 100 倍缩放并压缩为 int16_t，供状态上报使用。
 */
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

/**
 * @brief 根据当前业务模式和保护态生成 USB 状态字段。
 */
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

/**
 * @brief 将控制上下文恢复到上电后的默认状态。
 */
void ctrl_reset_context(void)
{
    ctrl_ctx.yaw_world_target = 0.0f;
    ctrl_ctx.pitch_world_target = 0.0f;
    ctrl_ctx.yaw_joint_target = 0.0f;
    ctrl_ctx.pitch_joint_target = 0.0f;
    ctrl_ctx.world_target_synced = false;
    ctrl_ctx.work_mode = WORK_MODE_STABLE;
    ctrl_ctx.search_stage = SEARCH_STAGE_GLOBAL;
    ctrl_ctx.protect_state = PROTECT_NONE;
    ctrl_ctx.search_stage_start_tick = 0U;
    ctrl_ctx.search_phase_start_tick = 0U;
    ctrl_ctx.search_local_center_yaw = 0.0f;
    ctrl_ctx.search_local_center_pitch = 0.0f;
    ctrl_ctx.auto_aim_target_yaw = 0.0f;
    ctrl_ctx.auto_aim_target_pitch = 0.0f;
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

/**
 * @brief 将世界系目标同步到当前 IMU 姿态，避免启动瞬间跳变。
 */
void ctrl_sync_world_target_to_attitude(uint32_t now_tick)
{
    ctrl_ctx.yaw_world_target = clampf(ctrl_ctx.current_attitude.yaw,
                                       k_yaw_limit_min_deg,
                                       k_yaw_limit_max_deg);
    ctrl_ctx.pitch_world_target = clampf(ctrl_ctx.current_attitude.pitch,
                                         k_pitch_limit_min_deg,
                                         k_pitch_limit_max_deg);
    ctrl_ctx.world_target_synced = true;

    if (ctrl_ctx.work_mode == WORK_MODE_SEARCH)
    {
        if (ctrl_ctx.search_stage == SEARCH_STAGE_LOCAL)
        {
            ctrl_ctx.search_local_center_yaw = ctrl_ctx.yaw_world_target;
            ctrl_ctx.search_local_center_pitch = ctrl_ctx.pitch_world_target;
        }
    }
}

/**
 * @brief 缓存云台关节角调试值，便于后续上报和观测。
 */
void ctrl_update_joint_debug_targets(const MotorManage &motor_manage)
{
    ctrl_ctx.yaw_joint_target = motor_manage.get_yaw_joint_deg();
    ctrl_ctx.pitch_joint_target = motor_manage.get_pitch_joint_deg();
}

/**
 * @brief 进入局部搜索模式。
 *
 * 局部搜索以当前世界系目标为中心，先做小范围摆动。
 */
void ctrl_enter_search_local(uint32_t now_tick)
{
    ctrl_ctx.work_mode = WORK_MODE_SEARCH;
    ctrl_ctx.search_stage = SEARCH_STAGE_LOCAL;
    ctrl_ctx.search_stage_start_tick = now_tick;
    ctrl_ctx.search_phase_start_tick = now_tick;
    ctrl_ctx.auto_aim_target_yaw = 0.0f;
    ctrl_ctx.auto_aim_target_pitch = 0.0f;
    ctrl_ctx.search_local_center_yaw = clampf(ctrl_ctx.yaw_world_target,
                                              k_yaw_limit_min_deg,
                                              k_yaw_limit_max_deg);
    ctrl_ctx.search_local_center_pitch = clampf(ctrl_ctx.pitch_world_target,
                                                k_pitch_limit_min_deg,
                                                k_pitch_limit_max_deg);
}

/**
 * @brief 进入全局搜索模式。
 *
 * 全局搜索会复用目标回归相位，减少模式切换时的轨迹突变。
 */
void ctrl_enter_search_global(uint32_t now_tick, float ref_yaw, float ref_pitch)
{
    const uint32_t reentry_elapsed_ms = ctrl_find_global_reentry_elapsed_ms(ref_yaw, ref_pitch);

    ctrl_ctx.work_mode = WORK_MODE_SEARCH;
    ctrl_ctx.search_stage = SEARCH_STAGE_GLOBAL;
    ctrl_ctx.search_stage_start_tick = now_tick;
    ctrl_ctx.search_phase_start_tick = now_tick - reentry_elapsed_ms;
    ctrl_ctx.auto_aim_target_yaw = 0.0f;
    ctrl_ctx.auto_aim_target_pitch = 0.0f;
}

/**
 * @brief 进入稳定模式并清空自瞄目标。
 */
void ctrl_enter_stable(void)
{
    ctrl_ctx.work_mode = WORK_MODE_STABLE;
    ctrl_ctx.auto_aim_target_yaw = 0.0f;
    ctrl_ctx.auto_aim_target_pitch = 0.0f;
}

/**
 * @brief 刷新硬件在线状态与最近一次检测时刻。
 */
void ctrl_update_health(uint32_t now_tick)
{
    ctrl_ctx.imu_online = imu_attitude_ready();
    ctrl_ctx.motor_yaw_online = can_check(k_yaw_can_id);
    ctrl_ctx.motor_pitch_online = can_check(k_pitch_can_id);
    ctrl_ctx.last_health_check_tick = now_tick;
}

/**
 * @brief 将自瞄绝对角目标写入世界系目标，并清空待处理目标。
 */
void ctrl_apply_auto_aim_target(void)
{
    if ((ctrl_ctx.auto_aim_target_yaw == 0.0f) && (ctrl_ctx.auto_aim_target_pitch == 0.0f))
    {
        return;
    }

    ctrl_ctx.yaw_world_target = clampf(ctrl_ctx.auto_aim_target_yaw,
                                       k_yaw_limit_min_deg,
                                       k_yaw_limit_max_deg);
    ctrl_ctx.pitch_world_target = clampf(ctrl_ctx.auto_aim_target_pitch,
                                         k_pitch_limit_min_deg,
                                         k_pitch_limit_max_deg);
    ctrl_ctx.auto_aim_target_yaw = 0.0f;
    ctrl_ctx.auto_aim_target_pitch = 0.0f;
}

/**
 * @brief 处理一条控制消息并更新任务状态。
 */
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
            ctrl_enter_search_local(now_tick);
            break;

        case CTRL_MSG_AUTO_AIM_ABS:
            ctrl_ctx.last_ctrl_msg_tick = now_tick;
            ctrl_ctx.last_auto_aim_tick = now_tick;
            ctrl_ctx.work_mode = WORK_MODE_AUTO_AIM;
            ctrl_ctx.protect_state = PROTECT_NONE;
            ctrl_ctx.auto_aim_target_yaw = msg->yaw_target;
            ctrl_ctx.auto_aim_target_pitch = msg->pitch_target;
            ctrl_ctx.yaw_world_target = clampf(msg->yaw_target,
                                               k_yaw_limit_min_deg,
                                               k_yaw_limit_max_deg);
            ctrl_ctx.pitch_world_target = clampf(msg->pitch_target,
                                                 k_pitch_limit_min_deg,
                                                 k_pitch_limit_max_deg);
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

/**
 * @brief 从消息队列中尽可能消费控制消息。
 */
void ctrl_consume_msgs(uint32_t now_tick, bool *send_lock_feedback)
{
    CtrlMsg_t msg;

    while (ctrl_msg_queue_pop(&msg, 0U))
    {
        ctrl_handle_msg(&msg, now_tick, send_lock_feedback);
    }
}

/**
 * @brief 按当前搜索阶段更新世界系目标角。
 */
void ctrl_update_search_target(uint32_t now_tick)
{
    const float elapsed_s = wrap_period_s(now_tick - ctrl_ctx.search_phase_start_tick);
    float yaw_target = 0.0f;
    float pitch_target = 0.0f;

    if (ctrl_ctx.search_stage == SEARCH_STAGE_LOCAL)
    {
        ctrl_get_local_search_target(elapsed_s, &yaw_target, &pitch_target);
    }
    else
    {
        ctrl_get_global_search_target(elapsed_s, &yaw_target, &pitch_target);
    }

    ctrl_ctx.yaw_world_target = clampf(yaw_target, k_yaw_limit_min_deg, k_yaw_limit_max_deg);
    ctrl_ctx.pitch_world_target = clampf(pitch_target, k_pitch_limit_min_deg, k_pitch_limit_max_deg);
}

/**
 * @brief 根据消息与超时条件切换业务模式。
 */
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
        ctrl_enter_search_local(now_tick);
        return;
    }

    if ((ctrl_ctx.work_mode == WORK_MODE_SEARCH) &&
        (ctrl_ctx.search_stage == SEARCH_STAGE_LOCAL) &&
        ((uint32_t)(now_tick - ctrl_ctx.search_stage_start_tick) > k_search_local_duration_ms))
    {
        ctrl_enter_search_global(now_tick, ctrl_ctx.yaw_world_target, ctrl_ctx.pitch_world_target);
        return;
    }

    if ((ctrl_ctx.work_mode == WORK_MODE_SEARCH) &&
        (ctrl_ctx.last_ctrl_msg_tick != 0U) &&
        ((uint32_t)(now_tick - ctrl_ctx.last_ctrl_msg_tick) > k_search_to_stable_timeout_ms))
    {
        ctrl_enter_stable();
    }
}

/**
 * @brief 按周期向上位机发送状态反馈。
 */
void ctrl_send_status_if_due(uint32_t now_tick, const imu_data_t &imu_data)
{
    usb_status_feedback_t status = {0};

    if ((ctrl_ctx.last_status_tx_tick != 0U) &&
        ((uint32_t)(now_tick - ctrl_ctx.last_status_tx_tick) < k_status_feedback_period_ms))
    {
        return;
    }

    status.yaw_target = encode_angle_x100(imu_data.yaw);
    status.pitch_target = encode_angle_x100(imu_data.pitch);
    status.roll_target = encode_angle_x100(imu_data.roll);
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

    // 任务启动后先完成外设初始化和上下文复位，再进入 1ms 固定周期控制循环。
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
    //osDelay(osWaitForever);
#endif
    //  不要删掉这行！现在在跑debug任务！

    for (;;)
    {
        // 先采样 IMU 和电机反馈，再根据消息、保护态和模式决定目标输出。
        osDelay(osWaitForever);
        const uint32_t now_tick = osKernelGetTickCount();
        bool send_lock_feedback = false;

        imu_update();
        imu_get_data(&imu_data);
        ctrl_ctx.current_attitude.yaw = imu_data.yaw;
        ctrl_ctx.current_attitude.pitch = imu_data.pitch;
        ctrl_ctx.current_attitude.roll = imu_data.roll;

        motor_manage.update_feedback();
        ctrl_update_joint_debug_targets(motor_manage);
        ctrl_update_health(now_tick);
        ctrl_consume_msgs(now_tick, &send_lock_feedback);
        ctrl_update_mode_timeout(now_tick);

        if (send_lock_feedback)
        {
            (void)usb_protocol_send_lock_feedback(now_tick, k_lock_reason_manual);
        }

        if (!ctrl_ctx.imu_online)
        {
            ctrl_ctx.world_target_synced = false;
            ctrl_ctx.auto_aim_target_yaw = 0.0f;
            ctrl_ctx.auto_aim_target_pitch = 0.0f;
            motor_manage.lock();
            ctrl_send_status_if_due(now_tick, imu_data);
            vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(k_ctrl_period_ms));
            continue;
        }

        if (!ctrl_ctx.world_target_synced)
        {
            ctrl_sync_world_target_to_attitude(now_tick);
        }

        if (ctrl_ctx.protect_state == PROTECT_LOCK)
        {
            motor_manage.lock();
            ctrl_send_status_if_due(now_tick, imu_data);
            vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(k_ctrl_period_ms));
            continue;
        }

        if (ctrl_ctx.work_mode == WORK_MODE_SEARCH)
        {
            ctrl_update_search_target(now_tick);
        }
        else if (ctrl_ctx.work_mode == WORK_MODE_AUTO_AIM)
        {
            ctrl_apply_auto_aim_target();
        }

        ctrl_ctx.yaw_world_target = clampf(ctrl_ctx.yaw_world_target,
                                           k_yaw_limit_min_deg,
                                           k_yaw_limit_max_deg);
        ctrl_ctx.pitch_world_target = clampf(ctrl_ctx.pitch_world_target,
                                             k_pitch_limit_min_deg,
                                             k_pitch_limit_max_deg);
        const bool enable_planner =
            (ctrl_ctx.work_mode == WORK_MODE_STABLE) ||
            (ctrl_ctx.work_mode == WORK_MODE_AUTO_AIM);

        motor_manage.set_world_target(ctrl_ctx.yaw_world_target,
                                      ctrl_ctx.pitch_world_target,
                                      ctrl_ctx.current_attitude.yaw,
                                      ctrl_ctx.current_attitude.pitch,
                                      enable_planner);
        motor_manage.send_can_cmd();
        ctrl_send_status_if_due(now_tick, imu_data);

        auto a = uxTaskGetStackHighWaterMark(NULL);
        (void)a;
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(k_ctrl_period_ms));
    }
    /* USER CODE END StartCtrlTask */
}
