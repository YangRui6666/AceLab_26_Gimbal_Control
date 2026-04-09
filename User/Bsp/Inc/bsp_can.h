//
// Created by CORE on 2026/4/9.
//

#ifndef GIMBAL_UM_BSP_CAN_H
#define GIMBAL_UM_BSP_CAN_H
#include <sys/_stdint.h>

typedef struct {
    uint8_t   data[8];       // 数据字节 (最大 8 字节)
    uint8_t   dlc;           // 数据长度码 (0~8)
    uint32_t  timestamp_ms;  // 接收时间戳 (可选但推荐)
} CanRxFrame;

bool bsp_can_rx(uint16_t can_id, CanRxFrame *frame);

#endif //GIMBAL_UM_BSP_CAN_H