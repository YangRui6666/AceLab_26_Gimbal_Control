//
// Created by CORE on 2026/3/14.
// Updated by CORE on 2026/4/6 - 对齐新版视觉协议与增量式自瞄语义
//

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "task.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "../Bsp/Inc/bsp_bmi088.h"
#include "../Config/imu_config.h"
#include "../Config/vision_config.h"
#include "../Devices/devices_gm6020.h"
#include "../Module/imu_fusion.h"
#include "../Module/pid.h"
#include "VisionTask.h"

static gimbal_axis_feedback_t make_axis_feedback(const devices_gm6020_feedback_t *device_feedback)
{
    // 任务层只处理统一后的反馈视图，不直接依赖底层设备快照。
    gimbal_axis_feedback_t axis_feedback = {0};

    if (device_feedback == NULL)
    {
        return axis_feedback;
    }

    axis_feedback.position_deg = device_feedback->position_deg;
    axis_feedback.velocity_rpm = device_feedback->velocity_rpm;
    axis_feedback.current_ma = device_feedback->current_ma;
    axis_feedback.filtered_current_ma = device_feedback->filtered_current_ma;
    axis_feedback.temp = device_feedback->temp;
    axis_feedback.last_update_tick = device_feedback->last_rx_tick;
    axis_feedback.motor_online = device_feedback->online;
    return axis_feedback;
}

static float clamp_float(float value, float min_value, float max_value)
{
    if (value > max_value)
    {
        return max_value;
    }

    if (value < min_value)
    {
        return min_value;
    }

    return value;
}

static void apply_gimbal_target(float yaw_target_deg, float pitch_target_deg, bool use_world_control)
{
    const float limited_yaw = clamp_float(yaw_target_deg, VISION_YAW_LIMIT_MIN, VISION_YAW_LIMIT_MAX);
    const float limited_pitch = clamp_float(pitch_target_deg, VISION_PITCH_LIMIT_MIN, VISION_PITCH_LIMIT_MAX);

    if (use_world_control)
    {
        (void)gimbal_set_world_target(limited_yaw, limited_pitch);
    }
    else
    {
        (void)gimbal_set_position_target(limited_yaw, limited_pitch);
    }
}

void ControlStartTask(void *argument)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1);
    const bool use_world_control = WORLD_COORDINATE_CONTROL_ENABLE;
    const float search_omega = SEARCH_MAX_YAW_SPEED_DEG_PER_S / SEARCH_YAW_AMPLITUDE_DEG;

    float world_yaw = 0.0f;
    float world_pitch = 0.0f;
    float world_roll = 0.0f;
    float gyro_yaw_rate = 0.0f;
    float gyro_pitch_rate = 0.0f;
    bmi088_data_t raw_imu_data = {0};

    uint32_t last_auto_aim_sequence = 0U;
    uint32_t feedback_sequence = 0U;
    uint32_t search_start_tick = xTaskGetTickCount();
    bool disable_latched = false;
    gimbal_mode_t applied_mode = GIMBAL_MODE_STABLE;

    (void)argument;

    if (!gimbal_control_init())
    {
        while (1)
        {
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }

    (void)gimbal_set_world_control_enable(use_world_control);
    apply_gimbal_target(0.0f, 0.0f, use_world_control);

    while (1)
    {
        devices_gm6020_feedback_t yaw_feedback = {0};
        devices_gm6020_feedback_t pitch_feedback = {0};
        vision_command_mailbox_t command = {
            .requested_mode = GIMBAL_MODE_STABLE
        };
        gimbal_feedback_snapshot_t feedback_snapshot = {0};
        gimbal_mode_t requested_mode = GIMBAL_MODE_STABLE;
        gimbal_mode_t actual_mode = GIMBAL_MODE_STABLE;
        const uint32_t current_tick = xTaskGetTickCount();

        float yaw_pos = 0.0f;
        float pitch_pos = 0.0f;
        float yaw_vel = 0.0f;
        float pitch_vel = 0.0f;

        module_imu_get_float(&world_roll, &world_pitch, &world_yaw);

        if (bsp_imu_get(&raw_imu_data))
        {
            gyro_yaw_rate = raw_imu_data.gyro_z * IMU_GYRO_SCALE_2000DPS;
            gyro_pitch_rate = raw_imu_data.gyro_y * IMU_GYRO_SCALE_2000DPS;
        }

        devices_gimbal_poll();

        const bool yaw_feedback_ok = devices_gimbal_get_yaw_feedback(&yaw_feedback);
        const bool pitch_feedback_ok = devices_gimbal_get_pitch_feedback(&pitch_feedback);
        if (yaw_feedback_ok && pitch_feedback_ok)
        {
            gimbal_axis_feedback_t yaw_axis_feedback = make_axis_feedback(&yaw_feedback);
            gimbal_axis_feedback_t pitch_axis_feedback = make_axis_feedback(&pitch_feedback);
            (void)gimbal_set_encoder_feedback(&yaw_axis_feedback, &pitch_axis_feedback);
        }

        gimbal_get_status(&yaw_pos, &pitch_pos, &yaw_vel, &pitch_vel);

        const bool can_communication_ok = yaw_feedback_ok && pitch_feedback_ok &&
                                          yaw_feedback.online && pitch_feedback.online;
        const bool imu_communication_ok = bsp_imu_check();

        (void)vision_read_command_mailbox(&command);
        requested_mode = command.requested_mode;

        if (!disable_latched && (!can_communication_ok || !imu_communication_ok))
        {
            disable_latched = true;
        }

        if (disable_latched || requested_mode == GIMBAL_MODE_DISABLE)
        {
            disable_latched = true;
            applied_mode = GIMBAL_MODE_DISABLE;
        }
        else
        {
            if (requested_mode != applied_mode)
            {
                if (requested_mode == GIMBAL_MODE_SEARCH)
                {
                    search_start_tick = current_tick;
                }

                applied_mode = requested_mode;
            }

            switch (applied_mode)
            {
                case GIMBAL_MODE_STABLE:
                    apply_gimbal_target(0.0f, 0.0f, use_world_control);
                    break;

                case GIMBAL_MODE_SEARCH:
                {
                    const float elapsed_s = ((float)(current_tick - search_start_tick)) / 1000.0f;
                    const float yaw_target = SEARCH_YAW_AMPLITUDE_DEG * sinf(search_omega * elapsed_s);
                    apply_gimbal_target(yaw_target, SEARCH_PITCH_TARGET_DEG, use_world_control);
                    break;
                }

                case GIMBAL_MODE_AUTO_AIM:
                    if (command.auto_aim_sequence != last_auto_aim_sequence)
                    {
                        const float base_yaw = use_world_control ? world_yaw : yaw_pos;
                        const float base_pitch = use_world_control ? world_pitch : pitch_pos;

                        apply_gimbal_target(base_yaw + command.yaw_error_deg,
                                            base_pitch + command.pitch_error_deg,
                                            use_world_control);
                        last_auto_aim_sequence = command.auto_aim_sequence;
                    }
                    break;

                case GIMBAL_MODE_LOCK_PROTECT:
                    break;

                case GIMBAL_MODE_DISABLE:
                default:
                    disable_latched = true;
                    applied_mode = GIMBAL_MODE_DISABLE;
                    break;
            }
        }

        if (disable_latched)
        {
            gimbal_emergency_stop();
            devices_gimbal_stop();
            (void)devices_gimbal_send();
            actual_mode = GIMBAL_MODE_DISABLE;
        }
        else if (applied_mode == GIMBAL_MODE_LOCK_PROTECT)
        {
            devices_gimbal_stop();
            (void)devices_gimbal_send();
            actual_mode = GIMBAL_MODE_LOCK_PROTECT;
        }
        else
        {
            bool control_ok = false;

            if (use_world_control)
            {
                control_ok = gimbal_world_coordinate_control(world_yaw, world_pitch,
                                                             gyro_yaw_rate, gyro_pitch_rate);
            }
            else
            {
                control_ok = gimbal_control_task();
            }

            if (!control_ok)
            {
                disable_latched = true;
                gimbal_emergency_stop();
                devices_gimbal_stop();
                (void)devices_gimbal_send();
                actual_mode = GIMBAL_MODE_DISABLE;
            }
            else
            {
                int16_t yaw_current = 0;
                int16_t pitch_current = 0;

                gimbal_get_output_currents(&yaw_current, &pitch_current);
                devices_gimbal_set_currents(yaw_current, pitch_current);
                (void)devices_gimbal_send();
                actual_mode = applied_mode;
            }
        }

        feedback_snapshot.yaw_deg = use_world_control ? world_yaw : yaw_pos;
        feedback_snapshot.pitch_deg = use_world_control ? world_pitch : pitch_pos;
        feedback_snapshot.roll_deg = world_roll;
        feedback_snapshot.yaw_rate_dps = gyro_yaw_rate;
        feedback_snapshot.pitch_rate_dps = gyro_pitch_rate;
        feedback_snapshot.current_mode = actual_mode;
        feedback_snapshot.timestamp = current_tick;
        feedback_snapshot.sequence = ++feedback_sequence;
        feedback_snapshot.disable_active = disable_latched;
        feedback_snapshot.can_online = can_communication_ok;
        feedback_snapshot.imu_online = imu_communication_ok;
        feedback_snapshot.world_control_enabled = use_world_control && !disable_latched;
        vision_publish_feedback_snapshot(&feedback_snapshot);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
