/**
 * @file VisionTask.c
 * @brief 视觉通信任务实现
 * @author CORE
 * @date 2026-03-15
 */

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "task.h"
#include "semphr.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "../Bsp/Inc/bsp_usb.h"
#include "../Module/pid.h"
#include "../Config/vision_config.h"
#include "../Config/imu_config.h"

// 外部信号量句柄
extern osSemaphoreId_t VisionBinarySemHandle;

// 静态变量
static vision_data_t vision_data = {0};
static gimbal_mode_t current_mode = GIMBAL_MODE_SENTRY;
static uint32_t last_status_send_tick = 0;

/**
 * @brief 简单JSON字符串解析器
 * @param json_str JSON字符串
 * @param data 解析结果存储结构
 * @retval true 解析成功
 * @retval false 解析失败
 */
static bool vision_parse_simple_json(const char* json_str, vision_data_t* data) {
    if (json_str == NULL || data == NULL) {
        return false;
    }

    char *p;
    bool parse_ok = false;

    // 检查是否是target类型消息
    p = strstr(json_str, "\"type\":");
    if (p != NULL) {
        p += 7; // 跳过"type":
        while (*p == ' ' || *p == '\"') p++; // 跳过空格和引号
        if (strncmp(p, "target", 6) != 0) {
            return false; // 不是target类型消息
        }
        parse_ok = true;
    }

    // 解析detected字段
    p = strstr(json_str, "\"detected\":");
    if (p != NULL) {
        p += 11; // 跳过"detected":
        while (*p == ' ') p++; // 跳过空格
        data->target_detected = (strncmp(p, "true", 4) == 0);
        parse_ok = true;
    }

    // 解析yaw字段
    p = strstr(json_str, "\"yaw\":");
    if (p != NULL) {
        data->target_yaw = strtof(p + 6, NULL);
        parse_ok = true;
    }

    // 解析pitch字段
    p = strstr(json_str, "\"pitch\":");
    if (p != NULL) {
        data->target_pitch = strtof(p + 8, NULL);
        parse_ok = true;
    }

    // 解析roll字段(可选)
    p = strstr(json_str, "\"roll\":");
    if (p != NULL) {
        data->target_roll = strtof(p + 7, NULL);
    }

    if (parse_ok) {
        data->data_valid = true;
        data->last_update_tick = xTaskGetTickCount();
    }

    return parse_ok;
}

/**
 * @brief 角度安全限位检查
 * @param yaw Yaw角度
 * @param pitch Pitch角度
 * @retval true 角度在安全范围内
 * @retval false 角度超出安全范围
 */
static bool vision_check_angle_limits(float yaw, float pitch) {
    return (yaw >= VISION_YAW_LIMIT_MIN && yaw <= VISION_YAW_LIMIT_MAX &&
            pitch >= VISION_PITCH_LIMIT_MIN && pitch <= VISION_PITCH_LIMIT_MAX);
}

/**
 * @brief 发送云台状态信息
 */
static void vision_send_status(void) {
    char status_buffer[128];
    float yaw_pos, pitch_pos, yaw_vel, pitch_vel;

    // 获取当前云台状态
    gimbal_get_status(&yaw_pos, &pitch_pos, &yaw_vel, &pitch_vel);

    // 构造JSON状态字符串
    int len = snprintf(status_buffer, sizeof(status_buffer),
        "{\"type\":\"status\",\"yaw\":%.2f,\"pitch\":%.2f,\"gyro_yaw\":%.1f,\"gyro_pitch\":%.1f,\"mode\":\"%s\",\"timestamp\":%lu}\n",
        yaw_pos, pitch_pos, yaw_vel, pitch_vel,
        (current_mode == GIMBAL_MODE_AUTO) ? "auto" :
        (current_mode == GIMBAL_MODE_SENTRY) ? "sentry" : "manual",
        xTaskGetTickCount());

    // 发送状态信息
    if (len > 0 && len < sizeof(status_buffer)) {
        bsp_usb_transmit((uint8_t*)status_buffer, (uint16_t)len);
    }
}

/**
 * @brief 处理视觉目标数据
 * @param data 视觉数据
 */
static void vision_process_target_data(const vision_data_t* data) {
    if (data == NULL || !data->data_valid) {
        return;
    }

    if (data->target_detected) {
        // 检查角度安全限位
        if (vision_check_angle_limits(data->target_yaw, data->target_pitch)) {
            // 根据控制模式选择目标设置方式
            bool set_success = false;
            if (WORLD_COORDINATE_CONTROL_ENABLE) {
                // 世界坐标系控制：视觉目标被认为是世界坐标系下的目标
                set_success = gimbal_set_world_target(data->target_yaw, data->target_pitch);
            } else {
                // 电机编码器控制：传统方式
                set_success = gimbal_set_position_target(data->target_yaw, data->target_pitch);
            }

            if (set_success) {
                current_mode = GIMBAL_MODE_AUTO;
            }
        }
    } else {
        // 没有检测到目标，切换到哨兵模式
        current_mode = GIMBAL_MODE_SENTRY;
    }
}

/**
 * @brief 检查视觉数据超时
 * @retval true 数据超时
 * @retval false 数据正常
 */
static bool vision_check_timeout(void) {
    if (!vision_data.data_valid) {
        return true;
    }

    uint32_t current_tick = xTaskGetTickCount();
    return (current_tick - vision_data.last_update_tick) > pdMS_TO_TICKS(VISION_DATA_TIMEOUT_MS);
}

/**
 * @brief 视觉通信任务主函数
 * @param argument 任务参数(未使用)
 */
void StartVisionTask03(void *argument)
{
    // 初始化USB包装层
    bsp_usb_init();

    // 行缓冲区用于接收完整的JSON消息
    static char rx_line_buffer[VISION_PROTOCOL_MAX_LINE];
    static uint16_t line_pos = 0;
    uint8_t temp_buffer[64];

    // 清空接收缓冲区
    bsp_usb_clear_rx_buffer();

    // 初始化视觉数据
    memset(&vision_data, 0, sizeof(vision_data));
    current_mode = GIMBAL_MODE_SENTRY;
    last_status_send_tick = xTaskGetTickCount();

    /* 无限循环 */
    while(1)
    {
        // 等待USB数据到达或超时
        if (osSemaphoreAcquire(VisionBinarySemHandle, pdMS_TO_TICKS(50)) == osOK) {
            // 读取USB接收缓冲区数据
            uint16_t data_len = bsp_usb_get_rx_data(temp_buffer, sizeof(temp_buffer));

            // 逐字节处理，查找完整的JSON行
            for (uint16_t i = 0; i < data_len; i++) {
                char ch = (char)temp_buffer[i];

                if (ch == '\n' || ch == '\r') {
                    // 收到行结束符
                    if (line_pos > 0) {
                        rx_line_buffer[line_pos] = '\0';

                        // 解析JSON数据
                        vision_data_t temp_data = {0};
                        if (vision_parse_simple_json(rx_line_buffer, &temp_data)) {
                            // 更新视觉数据
                            vision_data = temp_data;

                            // 处理目标数据
                            vision_process_target_data(&vision_data);
                        }

                        line_pos = 0; // 重置行缓冲区
                    }
                } else if (line_pos < sizeof(rx_line_buffer) - 1) {
                    // 添加字符到行缓冲区
                    rx_line_buffer[line_pos++] = ch;
                }
            }
        }

        // 检查视觉数据超时
        if (vision_check_timeout()) {
            // 超时，切换到哨兵模式
            if (current_mode != GIMBAL_MODE_SENTRY) {
                current_mode = GIMBAL_MODE_SENTRY;
                vision_data.data_valid = false;
            }
        }

        // 定期发送状态信息
        uint32_t current_tick = xTaskGetTickCount();
        if (current_tick - last_status_send_tick >= pdMS_TO_TICKS(VISION_STATUS_SEND_PERIOD)) {
            vision_send_status();
            last_status_send_tick = current_tick;
        }

        // 短暂延时，防止任务占用过多CPU
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

/**
 * @brief 获取当前云台工作模式
 * @retval 当前模式
 */
gimbal_mode_t vision_get_current_mode(void) {
    return current_mode;
}

/**
 * @brief 强制设置云台工作模式
 * @param mode 目标模式
 */
void vision_set_mode(gimbal_mode_t mode) {
    current_mode = mode;
}

/**
 * @brief 获取最新的视觉数据
 * @retval 视觉数据结构指针
 */
const vision_data_t* vision_get_data(void) {
    return &vision_data;
}