//
// Created by CORE on 2026/3/14.
//

#ifndef VISION_F405_BSP_CAN_H
#define VISION_F405_BSP_CAN_H
#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "../../Tools/ring_buffer.h"

// 配置宏定义
#define CURRENT_FILTER_ALPHA    0.1f    // 电流滤波系数
#define MOTOR_TIMEOUT_MS        50      // 电机通信超时时间(ms)
#define MOTOR_POS_RANGE         8192    // GM6020位置范围

// CAN接收数据结构
typedef struct
{
    uint32_t std_id;        // CAN标准ID
    uint8_t data[8];        // CAN数据
    uint8_t dlc;            // 数据长度
    TickType_t timestamp;   // 接收时间戳(FreeRTOS tick)
} can_rx_msg_t;

// 电机状态结构 (扩展版本，支持解旋和滤波)
typedef struct
{
    int16_t pos;                    // 当前位置 (原始)
    int16_t speed;                  // 速度
    int16_t current;                // 电流 (原始)
    int8_t temp;                    // 温度
    TickType_t last_recv_time;      // 最后接收时间(FreeRTOS tick)

    // 解旋相关
    int32_t total_angle;            // 累计旋转角度（包含圈数）
    int16_t last_pos;               // 上次位置（用于解旋计算）
    int16_t turn_count;             // 旋转圈数

    // 滤波相关
    float filtered_current;         // 滤波后的电流
} gm6020_state_t;

// 函数声明
bool bsp_can_init(void);
void bsp_ctrl_motor(const int16_t *motor_currents);
void dji_motor_tx(uint16_t tx_std_id, const int16_t data_1, const int16_t data_2, const int16_t data_3, const int16_t data_4);

// CAN接收处理函数
void bsp_process_can_rx_data(void);
const gm6020_state_t* bsp_can_get_motor_state(uint8_t motor_id);
int32_t get_can_get_motor_angle(uint8_t motor_id);
bool bsp_can_motor_is_timeout(uint8_t motor_id);
void bsp_can_reset_motor(uint8_t motor_id);

#endif //VISION_F405_BSP_CAN_H
