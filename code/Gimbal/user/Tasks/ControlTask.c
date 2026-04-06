//
// Created by CORE on 2026/3/14.
// Updated by CORE on 2026/3/15 - 集成双环PID控制
//

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "task.h"
#include "../Bsp/Inc/bsp_bmi088.h"
#include "../Devices/devices_gm6020.h"
#include "../Module/pid.h"
#include "../Module/imu_fusion.h"
#include "../Config/vision_config.h"
#include "../Config/imu_config.h"
#include "VisionTask.h"

static gimbal_axis_feedback_t make_axis_feedback(const devices_gm6020_feedback_t* device_feedback)
{
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

void ControlStartTask(void *argument)
{
    /* USER CODE BEGIN ControlStartTask */
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1);  // 1ms周期

    // IMU数据变量
    float world_yaw = 0.0f, world_pitch = 0.0f, world_roll = 0.0f;
    float gyro_yaw_rate = 0.0f, gyro_pitch_rate = 0.0f;
    bmi088_data_t raw_imu_data = {0};

    // 初始化云台控制系统
    if (!gimbal_control_init())
    {
        // 初始化失败，进入错误处理
        while(1)
        {
            // TODO: 添加错误指示(如LED闪烁)
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }

    // 初始化世界坐标控制模式
    gimbal_set_world_control_enable(WORLD_COORDINATE_CONTROL_ENABLE);

    // 设置初始目标位置(水平居中)
    if (WORLD_COORDINATE_CONTROL_ENABLE)
    {
        gimbal_set_world_target(0.0f, 0.0f);  // 世界坐标系目标
    }
    else
    {
        gimbal_set_position_target(0.0f, 0.0f);  // 电机编码器目标
    }

    /* Infinite loop */
    while(1)
    {
        devices_gm6020_feedback_t yaw_feedback = {0};
        devices_gm6020_feedback_t pitch_feedback = {0};

        // 1. 读取IMU世界坐标角度
        module_imu_get_float(&world_roll, &world_pitch, &world_yaw);

        // 2. 读取陀螺仪原始角速度数据
        if (bsp_imu_get(&raw_imu_data))
        {
            // 转换为物理单位 (deg/s)
            gyro_yaw_rate = raw_imu_data.gyro_z * IMU_GYRO_SCALE_2000DPS;
            gyro_pitch_rate = raw_imu_data.gyro_y * IMU_GYRO_SCALE_2000DPS;
        }

        // 3. 处理设备反馈
        devices_gimbal_poll();

        bool yaw_feedback_ok = devices_gimbal_get_yaw_feedback(&yaw_feedback);
        bool pitch_feedback_ok = devices_gimbal_get_pitch_feedback(&pitch_feedback);
        if (yaw_feedback_ok && pitch_feedback_ok)
        {
            gimbal_axis_feedback_t yaw_axis_feedback = make_axis_feedback(&yaw_feedback);
            gimbal_axis_feedback_t pitch_axis_feedback = make_axis_feedback(&pitch_feedback);
            (void)gimbal_set_encoder_feedback(&yaw_axis_feedback, &pitch_axis_feedback);
        }

        // 4. 检查通信状态和控制逻辑
        bool can_communication_ok = yaw_feedback_ok && pitch_feedback_ok &&
                                    yaw_feedback.online && pitch_feedback.online;
        bool imu_communication_ok = bsp_imu_check();
        bool world_control_enabled = gimbal_get_world_state()->world_control_enable;

        if (can_communication_ok)
        {
            // 4a. 哨兵模式处理 (保持现有逻辑)
            static float sentry_yaw_target = 0.0f;
            static uint32_t last_sentry_update = 0;
            static int8_t scan_direction = 1;  // 1为正向，-1为反向

            gimbal_mode_t current_mode = vision_get_current_mode();

            if (current_mode == GIMBAL_MODE_SENTRY)
            {
                uint32_t current_tick = xTaskGetTickCount();

                // 哨兵模式更新频率控制 (SENTRY_UPDATE_FREQ Hz)
                if (current_tick - last_sentry_update >= pdMS_TO_TICKS(1000 / SENTRY_UPDATE_FREQ))
                {
                    // 计算扫描步长 (度/更新周期)
                    float scan_step = SENTRY_SCAN_SPEED / (float)SENTRY_UPDATE_FREQ;

                    // 更新目标角度
                    sentry_yaw_target += scan_step * scan_direction;

                    // 检查扫描边界，调整方向
                    if (sentry_yaw_target >= SENTRY_YAW_MAX)
                    {
                        sentry_yaw_target = SENTRY_YAW_MAX;
                        scan_direction = -1;  // 反向扫描
                    }
                    else if (sentry_yaw_target <= SENTRY_YAW_MIN)
                    {
                        sentry_yaw_target = SENTRY_YAW_MIN;
                        scan_direction = 1;   // 正向扫描
                    }

                    // 设置云台目标位置
                    if (world_control_enabled && imu_communication_ok)
                    {
                        gimbal_set_world_target(sentry_yaw_target, SENTRY_PITCH_TARGET);
                    }
                    else
                    {
                        gimbal_set_position_target(sentry_yaw_target, SENTRY_PITCH_TARGET);
                    }

                    last_sentry_update = current_tick;
                }
            }

            // 4b. 执行控制算法
            if (world_control_enabled && imu_communication_ok)
            {
                // 世界坐标系控制 (使用IMU和陀螺仪数据)
                if (!gimbal_world_coordinate_control(world_yaw, world_pitch,
                                                   gyro_yaw_rate, gyro_pitch_rate))
                {
                    // 世界坐标控制失败，切换到电机编码器控制
                    gimbal_set_world_control_enable(false);
                    (void)gimbal_control_task();
                }
            }
            else
            {
                // 电机编码器控制 (传统双环PID)
                if (!gimbal_control_task())
                {
                    // 控制失败，已自动触发紧急停止
                    // 这里可以添加额外的错误处理逻辑
                }
            }

            int16_t yaw_current = 0;
            int16_t pitch_current = 0;
            gimbal_get_output_currents(&yaw_current, &pitch_current);
            devices_gimbal_set_currents(yaw_current, pitch_current);
            (void)devices_gimbal_send();
        }
        else
        {
            // 5. CAN通信异常，触发紧急停止
            gimbal_emergency_stop();

            // 发送安全电流(零电流)
            devices_gimbal_stop();
            (void)devices_gimbal_send();
        }

        // 6. 调试信息输出(可选)
        #ifdef DEBUG_IMU_WORLD_COORDINATE
        static uint32_t debug_counter = 0;
        debug_counter++;

        // 每1秒输出一次状态信息
        if (debug_counter >= 1000)
        {
            debug_counter = 0;

            // TODO: 通过USB虚拟串口输出调试信息
            // printf("World: Yaw=%.1f°, Pitch=%.1f°, GyroYaw=%.1f°/s, GyroPitch=%.1f°/s\n",
            //        world_yaw, world_pitch, gyro_yaw_rate, gyro_pitch_rate);

            float yaw_pos, pitch_pos, yaw_vel, pitch_vel;
            gimbal_get_status(&yaw_pos, &pitch_pos, &yaw_vel, &pitch_vel);

            // printf("Motor: Yaw=%.2f°(%.0frpm), Pitch=%.2f°(%.0frpm)\n",
            //        yaw_pos, yaw_vel, pitch_pos, pitch_vel);
        }
        #endif

        // 7. 1ms精确延时
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
    /* USER CODE END ControlStartTask */
}
