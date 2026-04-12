//
// Created by CORE on 2026/4/9.
//

#ifndef GIMBAL_UM_BSP_CAN_H
#define GIMBAL_UM_BSP_CAN_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t data[8];         // 数据字节 (最大 8 字节)
    uint8_t dlc;             // 数据长度码 (0~8)
    uint32_t timestamp_ms;   // 接收时间戳
} CanRxFrame;

bool bsp_can_init(void);
bool bsp_tx(uint16_t can_id, const uint8_t *data, uint8_t dlc);
bool bsp_can_rx(uint16_t can_id, CanRxFrame *frame);
bool can_check(uint16_t can_id);

#ifdef __cplusplus
}
#endif

#endif //GIMBAL_UM_BSP_CAN_H
